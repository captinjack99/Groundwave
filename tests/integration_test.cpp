/**
 * @file integration_test.cpp
 * @brief End-to-end engine test in headless mode.
 *
 * Mirrors what AudioEngine::processTick() does on the live GUI side, but
 * without Qt or threads — just synchronous TX→RX cycles in software
 * loopback so we can mechanically verify:
 *
 *   1. Every (modulation, FEC) combination round-trips without crashing.
 *   2. Decoded frames match transmitted frames at clean SNR.
 *   3. Mid-flight config changes (the FEC-rate crash scenario) don't crash.
 *   4. Tick latency stays within budget at the largest FFT/highest QAM.
 *
 * Run with:  ./integration_test [--quick] [--noisy snr_db]
 *
 * Exits non-zero if any assertion fails, with a summary table at the end.
 * This is the test that should have existed from the start — it surfaces
 * threading-independent integration bugs that the unit tests miss.
 */

#include "types.hpp"
#include "ofdm.hpp"
#include "ldpc.hpp"
#include "frame.hpp"
#include "interleaver.hpp"
#include "soundcard_modem.hpp"
#include "snr_calculator.hpp"
#include "bicm.hpp"
#include "orbgrand.hpp"
#include "soft_decoder.hpp"
#include "papr_reducer.hpp"

#include <chrono>
#include <memory>
#include <cstdio>
#include <cstring>
#include <random>
#include <string>
#include <vector>

using namespace gw;

namespace {

int g_passed = 0;
int g_failed = 0;
std::vector<std::string> g_failures;

#define CHECK(cond, msg)                                                       \
    do {                                                                       \
        if (cond) { ++g_passed; std::printf("  PASS %s\n", msg); }              \
        else      { ++g_failed; g_failures.emplace_back(msg);                   \
                    std::printf("  FAIL %s\n", msg); }                          \
    } while (0)

// =========================================================================
// Single-frame TX→RX cycle through complex_loopback (no upconvert/downconvert
// — just the FEC + OFDM end-to-end). Returns BER on the decoded info bits.
// =========================================================================
struct CycleResult {
    bool tx_built = false;
    bool rx_decoded = false;
    bool crc_ok = false;
    int  bit_errors = -1;
    double tx_ms = 0.0;
    double rx_ms = 0.0;
};

CycleResult runCycle(Modulation mod, FECRate fec, uint16_t fft_size,
                      uint32_t sample_rate, float awgn_snr_db = -1.f,
                      size_t max_iter = 50,
                      bool use_dd = false,
                      uint32_t seed = 0x1234u,
                      bool use_mmse = false,
                      bool use_phase_tracker = false,
                      float cfo_hz = 0.f,
                      const ComplexBuf* chan_taps = nullptr,
                      bool per_bin_llr = true,
                      bool papr_on = false)
{
    CycleResult r{};

    // ---- Configure ----
    OFDMParams ofdm;
    ofdm.fft_size    = fft_size;
    ofdm.modulation  = mod;
    ofdm.sample_rate = sample_rate;
    ofdm.cyclic_prefix = CyclicPrefix::CP_1_8;
    // PAPR tone reservation carves data-free reserved tones out of the data
    // allocation (must be set before the mod/demod are built so both agree).
    if (papr_on) ofdm.papr_reserve_fraction = 0.05f;

    LDPCEncoder ldpc_enc(fec, LDPCBlockSize::Short);
    LDPCDecoder ldpc_dec(fec, LDPCBlockSize::Short, max_iter);
    BitInterleaver inter(ldpc_enc.codewordBits());

    OFDMModulator   tx_mod(ofdm);
    OFDMDemodulator rx_demod(ofdm);
    // Exercise the sync-hardening paths (MMSE Wiener #46, phase-tracker #36)
    // that the default path doesn't, so those fixes have test coverage.
    if (use_mmse)          rx_demod.enableMMSE();
    if (use_phase_tracker) rx_demod.enablePhaseTracker();
    rx_demod.setPerBinLLRWeighting(per_bin_llr);

    // Optional PAPR tone reservation on TX. The reducer operates on the
    // allocation's carved reserved tones (data-free); if it instead stole live
    // data carriers (the old bug) the RX would demap garbage and bit_errors
    // would explode. Kept alive for the whole modulate call below.
    std::unique_ptr<PAPRReducer> papr;
    if (papr_on) {
        const auto& a = tx_mod.allocation();
        PAPRConfig pc; pc.enabled = true; pc.reserve_fraction = 0.05f;
        papr = std::make_unique<PAPRReducer>(
            ofdm.fft_size, ofdm.guardLeft(), ofdm.fft_size - ofdm.guardRight(),
            a.data_indices, a.pilot_indices, pc);
        papr->useReservedTones(a.reserved_indices);
        tx_mod.setPAPRReducer(papr.get());
    }

    size_t k_bytes = ldpc_enc.infoBytes();
    size_t n_bytes = ldpc_enc.codewordBytes();

    // ---- TX ----
    auto tx_start = std::chrono::steady_clock::now();

    // Random info payload + channel noise (seed varied by callers that
    // average a marginal-SNR gate over several realizations, #73).
    std::mt19937 rng(seed);
    std::vector<uint8_t> info(k_bytes);
    for (auto& b : info) b = static_cast<uint8_t>(rng() & 0xFF);

    // Build a frame with a single packet on stream 0
    size_t cap = (k_bytes > constants::FRAME_OVERHEAD)
                 ? k_bytes - constants::FRAME_OVERHEAD : 0;
    if (cap < 8) return r;
    FrameBuilder fb(cap);
    std::vector<uint8_t> packet_data(cap - 4, 0);
    for (size_t i = 0; i < packet_data.size(); ++i)
        packet_data[i] = static_cast<uint8_t>(rng() & 0xFF);
    fb.addPacket(0, packet_data.data(), packet_data.size());
    auto frame_bytes = fb.build(/*frame_no*/ 1, fec, mod);
    frame_bytes.resize(k_bytes, 0);

    std::vector<uint8_t> codeword(n_bytes, 0);
    if (!ldpc_enc.encode(frame_bytes.data(), codeword.data())) return r;

    std::vector<uint8_t> ileaved(n_bytes, 0);
    inter.interleave(codeword.data(), ileaved.data());

    ComplexBuf tx_bb;
    tx_mod.modulateBits(ileaved.data(), ldpc_enc.codewordBits(), tx_bb);

    // Preamble (so the demod's processPreamble can establish channel est)
    ComplexBuf preamble = tx_mod.generatePreamble();

    r.tx_built = true;
    r.tx_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - tx_start).count();

    // ---- Channel: optional multipath (frequency-selective) + CFO + AWGN ----
    ComplexBuf channel = preamble;
    channel.insert(channel.end(), tx_bb.begin(), tx_bb.end());

    // Frequency-selective channel: convolve with a short impulse response
    // (delays must fit inside the CP so the OFDM guard absorbs the spread).
    // This makes |H(k)| vary across subcarriers — faded bins suffer ZF noise
    // enhancement that a per-bin LLR weighting must account for. Applied to the
    // whole buffer (preamble included) so the channel estimate sees it too.
    if (chan_taps && !chan_taps->empty()) {
        ComplexBuf conv(channel.size(), ComplexSample(0.f, 0.f));
        for (size_t n = 0; n < channel.size(); ++n) {
            ComplexSample acc(0.f, 0.f);
            for (size_t t = 0; t < chan_taps->size() && t <= n; ++t)
                acc += (*chan_taps)[t] * channel[n - t];
            conv[n] = acc;
        }
        channel = std::move(conv);
    }

    if (cfo_hz != 0.f) {
        // A small carrier offset → a per-symbol common-phase drift the phase
        // tracker must follow. Exercises the DD phase-frame fix (#36).
        const float w = 2.f * static_cast<float>(M_PI) * cfo_hz /
                        static_cast<float>(sample_rate);
        for (size_t n = 0; n < channel.size(); ++n)
            channel[n] *= std::polar(1.f, w * static_cast<float>(n));
    }
    if (awgn_snr_db >= 0.f) {
        float sig = 0.f;
        for (auto& s : channel) sig += std::norm(s);
        sig /= static_cast<float>(channel.size());
        float noise_var = sig / std::pow(10.f, awgn_snr_db / 10.f);
        float sigma = std::sqrt(noise_var * 0.5f);
        std::normal_distribution<float> nd(0.f, sigma);
        for (auto& s : channel) s += ComplexSample(nd(rng), nd(rng));
    }

    // ---- RX ----
    auto rx_start = std::chrono::steady_clock::now();

    // Process preamble (2 long symbols; our generator emits short+long block)
    size_t sym_len    = ofdm.symbolLength();
    size_t short_total = 10 * (ofdm.fft_size / 4);
    size_t pre_long_off = short_total;
    if (channel.size() < pre_long_off + 2 * sym_len) return r;

    ComplexBuf long_syms(channel.begin() + pre_long_off,
                          channel.begin() + pre_long_off + 2 * sym_len);
    if (!rx_demod.processPreamble(long_syms)) return r;

    // Skip preamble, demod data
    size_t data_off = preamble.size();
    size_t bps = bitsPerSymbol(mod);
    size_t bits_per_ofdm = tx_mod.bitsPerOFDMSymbol();
    if (bits_per_ofdm == 0) return r;
    size_t syms_per_cw = (ldpc_enc.codewordBits() + bits_per_ofdm - 1)
                       / bits_per_ofdm;

    std::vector<float> all_llrs;
    for (size_t s = 0; s < syms_per_cw; ++s) {
        size_t off = data_off + s * sym_len;
        if (off + sym_len > channel.size()) break;
        ComplexBuf one(channel.begin() + off, channel.begin() + off + sym_len);
        std::vector<float> llrs;
        if (use_dd) {
            rx_demod.demodulateSoftDD(one, llrs, rx_demod.noiseVariance(),
                                       /*use_pwl=*/false);
        } else {
            rx_demod.demodulateSoft(one, llrs, rx_demod.noiseVariance());
        }
        all_llrs.insert(all_llrs.end(), llrs.begin(), llrs.end());
    }
    all_llrs.resize(ldpc_enc.codewordBits(), 0.f);

    std::vector<float> deint(all_llrs.size());
    inter.deinterleave(all_llrs.data(), deint.data());

    std::vector<uint8_t> decoded(k_bytes, 0);
    auto dec_result = ldpc_dec.decode(deint.data(), decoded.data());

    r.rx_decoded = dec_result.converged;
    r.rx_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - rx_start).count();

    // Compare ONLY the actual info bits. The last info byte's trailing
    // bits (when k % 8 != 0) overlap with the first parity bits of the
    // codeword — `decoded` writes the first k bits + parity overspill,
    // and comparing whole bytes counts those parity bits as "errors".
    int bit_errs = 0;
    size_t info_bits = ldpc_enc.infoBits();
    for (size_t i = 0; i < info_bits; ++i) {
        bool a = (decoded[i >> 3]    >> (7 - (i & 7))) & 1;
        bool b = (frame_bytes[i >> 3] >> (7 - (i & 7))) & 1;
        if (a != b) ++bit_errs;
    }
    r.bit_errors = bit_errs;
    (void)bps;

    // CRC check via FrameParser
    ParsedFrame pf;
    if (FrameParser::parse(decoded, pf)) r.crc_ok = pf.crc_ok;
    return r;
}

// =========================================================================
// Test 1: every modulation × representative FECs round-trip cleanly
// =========================================================================
void test_all_modcods() {
    std::printf("\n=== Test 1: clean-channel modcod sweep ===\n");
    Modulation mods[] = {
        Modulation::BPSK, Modulation::QPSK,
        Modulation::QAM16, Modulation::QAM64,
        Modulation::QAM256
    };
    FECRate fecs[] = {
        FECRate::Rate_1_4, FECRate::Rate_1_2,
        FECRate::Rate_3_4, FECRate::Rate_9_10
    };
    for (auto m : mods) {
        for (auto f : fecs) {
            auto r = runCycle(m, f, 256, 48000, /*clean*/ -1.f);
            char label[128];
            std::snprintf(label, sizeof(label),
                          "%-7s / %-5s round-trip (TX=%.1fms RX=%.1fms err=%d)",
                          modulationName(m), fecRateName(f),
                          r.tx_ms, r.rx_ms, r.bit_errors);
            CHECK(r.tx_built && r.rx_decoded && r.bit_errors == 0, label);
        }
    }
}

// =========================================================================
// Test 2: rapid config changes — the FEC-rate-change crash scenario
// =========================================================================
void test_rapid_modcod_changes() {
    std::printf("\n=== Test 2: rapid mid-flight modcod changes ===\n");
    Modulation mods[] = { Modulation::QPSK, Modulation::QAM16, Modulation::QAM64 };
    FECRate fecs[] = {
        FECRate::Rate_1_4, FECRate::Rate_1_2, FECRate::Rate_3_4, FECRate::Rate_9_10
    };
    int ok = 0, total = 0;
    for (auto m : mods) {
        for (auto f : fecs) {
            ++total;
            auto r = runCycle(m, f, 128, 48000, -1.f);
            if (r.tx_built && r.rx_decoded && r.bit_errors == 0) ++ok;
        }
    }
    char label[64];
    std::snprintf(label, sizeof(label),
                  "rapid modcod sweep: %d/%d clean", ok, total);
    CHECK(ok == total, label);
}

// =========================================================================
// Test 3: tick-latency budget at biggest configs
// =========================================================================
void test_tick_latency_budget() {
    std::printf("\n=== Test 3: tick-latency budget ===\n");
    struct Cfg { Modulation m; FECRate f; uint16_t fft; uint32_t sr; double bud_ms; };
    Cfg configs[] = {
        { Modulation::QPSK,   FECRate::Rate_1_2,  256,  48000, 50.0 },
        { Modulation::QAM64,  FECRate::Rate_3_4, 1024,  96000, 80.0 },
        { Modulation::QAM256, FECRate::Rate_9_10,2048, 192000, 200.0 },
    };
    // The budgets are calibrated for optimized local builds. Unoptimized
    // (Debug) code is legitimately 3-6x slower, and shared CI runners add
    // load jitter on top — this is the suite's only wall-clock assertion,
    // so scale it rather than let the whole pipeline flake on it. The
    // bit-exactness check stays strict in every configuration.
#ifdef NDEBUG
    constexpr double kBudgetScale = 1.0;
#else
    constexpr double kBudgetScale = 6.0;
#endif
    for (auto& c : configs) {
        auto r = runCycle(c.m, c.f, c.fft, c.sr, -1.f);
        const double budget = c.bud_ms * kBudgetScale;
        char label[128];
        std::snprintf(label, sizeof(label),
                      "%-7s / %-5s FFT=%u SR=%u: TX+RX=%.1fms (budget %.0fms)",
                      modulationName(c.m), fecRateName(c.f), c.fft, c.sr,
                      r.tx_ms + r.rx_ms, budget);
        CHECK(r.tx_ms + r.rx_ms < budget && r.bit_errors == 0, label);
    }
}

// =========================================================================
// Test 4: SoundcardModem complex_loopback (the GUI's actual mode)
// =========================================================================
void test_software_loopback_modem() {
    std::printf("\n=== Test 4: SoundcardModem complex_loopback ===\n");
    OFDMParams ofdm;
    ofdm.fft_size = 256;
    ofdm.modulation = Modulation::QPSK;
    ofdm.sample_rate = 48000;

    ModemConfig mc;
    mc.sample_rate = 48000;
    mc.center_freq = 12000;
    mc.loopback = false;
    mc.complex_loopback = true;

    bool ok = true;
    try {
        SoundcardModem modem(mc, ofdm);
        OFDMModulator txm(ofdm);
        OFDMDemodulator rxm(ofdm);

        ComplexBuf preamble = txm.generatePreamble();
        modem.transmit(preamble);
        ComplexBuf seen = modem.receive(preamble.size());
        if (seen.size() < preamble.size() / 2) ok = false;
    } catch (const std::exception& e) {
        std::printf("    exception: %s\n", e.what());
        ok = false;
    }
    CHECK(ok, "SoundcardModem complex_loopback round-trips");
}

// =========================================================================
// Test 5: re-build the LDPC engine for every rate WITHOUT crash
// (this is the "FEC change crashes" repro)
// =========================================================================
void test_ldpc_rebuild_all_rates() {
    std::printf("\n=== Test 5: LDPC enc/dec rebuild for every rate ===\n");
    FECRate all_rates[] = {
        FECRate::Rate_1_4, FECRate::Rate_1_3, FECRate::Rate_2_5,
        FECRate::Rate_1_2, FECRate::Rate_3_5, FECRate::Rate_2_3,
        FECRate::Rate_3_4, FECRate::Rate_4_5, FECRate::Rate_5_6,
        FECRate::Rate_8_9, FECRate::Rate_9_10
    };
    bool any_crash = false;
    for (auto r : all_rates) {
        std::fprintf(stderr, "    -> rate %s\n", fecRateName(r));
        std::fflush(stderr);
        try {
            std::fprintf(stderr, "       ctor enc...\n"); std::fflush(stderr);
            LDPCEncoder enc(r, LDPCBlockSize::Short);
            std::fprintf(stderr, "       ctor dec... k=%zu n=%zu\n",
                         enc.infoBits(), enc.codewordBits());
            std::fflush(stderr);
            LDPCDecoder dec(r, LDPCBlockSize::Short, 50);
            std::fprintf(stderr, "       ctor inter...\n"); std::fflush(stderr);
            BitInterleaver inter(enc.codewordBits());
            std::fprintf(stderr, "       encode...\n"); std::fflush(stderr);
            std::vector<uint8_t> info(enc.infoBytes(), 0);
            std::vector<uint8_t> cw(enc.codewordBytes(), 0);
            enc.encode(info.data(), cw.data());
            std::fprintf(stderr, "       decode...\n"); std::fflush(stderr);
            std::vector<float> llrs(enc.codewordBits(), 5.f);
            for (size_t i = 0; i < enc.codewordBits(); ++i) {
                bool b = (cw[i >> 3] >> (7 - (i & 7))) & 1;
                llrs[i] = b ? -5.f : 5.f;
            }
            std::vector<uint8_t> out(enc.infoBytes(), 0);
            dec.decode(llrs.data(), out.data());
            std::fprintf(stderr, "       OK\n"); std::fflush(stderr);
        } catch (const std::exception& e) {
            std::printf("    rate %s: exception %s\n", fecRateName(r), e.what());
            any_crash = true;
        }
    }
    CHECK(!any_crash, "every FEC rate builds enc+dec without exception");
}

// =========================================================================
// Test 6: SNR threshold monotonicity — coding gain should DROP threshold
// =========================================================================
void test_snr_threshold_monotonic() {
    std::printf("\n=== Test 6: coding gain monotonicity ===\n");
    Modulation mods[] = { Modulation::BPSK, Modulation::QPSK,
                          Modulation::QAM16, Modulation::QAM64,
                          Modulation::QAM256, Modulation::QAM1024,
                          Modulation::QAM4096 };
    FECRate ordered[] = {
        FECRate::Rate_9_10, FECRate::Rate_8_9, FECRate::Rate_5_6,
        FECRate::Rate_4_5,  FECRate::Rate_3_4, FECRate::Rate_2_3,
        FECRate::Rate_3_5,  FECRate::Rate_1_2, FECRate::Rate_2_5,
        FECRate::Rate_1_3,  FECRate::Rate_1_4
    };
    for (auto m : mods) {
        float prev = 1e9f;
        bool monotone = true;
        float worst_gap = 0.f;
        for (auto f : ordered) {
            auto t = computeThreshold(m, f);
            if (t.threshold_db > prev + 0.001f) {
                monotone = false;
            }
            float gap = prev - t.threshold_db;
            if (prev < 1e8f && gap > worst_gap) worst_gap = gap;
            prev = t.threshold_db;
        }
        char label[128];
        std::snprintf(label, sizeof(label),
                      "%s thresholds drop monotonically with FEC strength "
                      "(max coding gain %.1f dB)",
                      modulationName(m), worst_gap);
        CHECK(monotone, label);
    }
}

// =========================================================================
// Test 7: 25-iter LDPC convergence vs 50-iter at moderate AWGN.
// The live engine uses max_iter=25 to cut tick latency; verify error rates
// stay comparable at the operating SNR where the decoder is actually doing
// work (not trivial clean-channel where both converge in <10 iters).
// =========================================================================
void test_ldpc_iter_cap_25_vs_50() {
    std::printf("\n=== Test 7: LDPC 25-iter vs 50-iter at moderate SNR ===\n");
    struct Cfg { Modulation m; FECRate f; float snr_db; };
    // SNR points chosen ~2 dB above the FEC threshold for each modcod so
    // the decoder is iterating, not falling out trivially.
    Cfg configs[] = {
        { Modulation::QPSK,  FECRate::Rate_1_2,  6.0f },
        { Modulation::QAM16, FECRate::Rate_3_4, 13.0f },
        { Modulation::QAM64, FECRate::Rate_3_4, 18.0f },
    };
    // Average over several noise realizations so the gate doesn't hinge on a
    // single std::normal_distribution draw, which differs across stdlibs and
    // made a 1-bit margin non-deterministic across CI platforms (#73).
    constexpr int N_SEEDS = 8;
    for (auto& c : configs) {
        long e25 = 0, e50 = 0;
        for (int s = 0; s < N_SEEDS; ++s) {
            uint32_t sd = 0x1234u + static_cast<uint32_t>(s) * 0x9E37u;
            e25 += runCycle(c.m, c.f, 256, 48000, c.snr_db, 25, false, sd).bit_errors;
            e50 += runCycle(c.m, c.f, 256, 48000, c.snr_db, 50, false, sd).bit_errors;
        }
        char label[176];
        std::snprintf(label, sizeof(label),
                      "%-7s / %-5s @ %.1f dB (%d seeds): iter25 errs=%ld  iter50 errs=%ld",
                      modulationName(c.m), fecRateName(c.f), c.snr_db,
                      N_SEEDS, e25, e50);
        // 25-iter must not be materially worse than 50-iter in aggregate
        // (≤ ~1 bit/seed of slack absorbs per-realization noise).
        CHECK(e25 <= e50 + N_SEEDS, label);
    }
}

// =========================================================================
// Test 8: decision-directed channel-estimate refinement
// At a marginal SNR (chosen so the non-DD baseline shows residual errors
// even with LDPC), DD should match or beat the baseline. The expected
// gain is modest (~0.2-0.4 dB) so on a clean channel both pass with 0
// errors — the DD-helpful regime is the modcod cliff.
// =========================================================================
void test_decision_directed_chest() {
    std::printf("\n=== Test 8: decision-directed channel estimation ===\n");
    // Operate just above the FEC cliff: DD should not regress here, and
    // should hold its own at clean SNR too. Run at clean and at a near-
    // cliff AWGN level; assert DD bit errors ≤ no-DD bit errors + small
    // slack (DD can be slightly worse on a single random seed but
    // shouldn't be catastrophically worse).
    struct Cfg { Modulation m; FECRate f; float snr_db; };
    Cfg configs[] = {
        { Modulation::QPSK,  FECRate::Rate_3_4,  -1.f }, // clean
        { Modulation::QAM16, FECRate::Rate_3_4,  13.f }, // marginal
        { Modulation::QAM64, FECRate::Rate_2_3,  16.f }, // marginal
    };
    // Clean channel is deterministic (0 errors). Marginal cases are averaged
    // over several seeds so the gate is robust to the noise realization (#73).
    constexpr int N_SEEDS = 8;
    for (auto& c : configs) {
        bool ok;
        char label[176];
        if (c.snr_db < 0.f) {
            auto r_plain = runCycle(c.m, c.f, 256, 48000, c.snr_db, 25, false);
            auto r_dd    = runCycle(c.m, c.f, 256, 48000, c.snr_db, 25, true);
            std::snprintf(label, sizeof(label),
                          "%-7s / %-5s @ clean : plain err=%d  DD err=%d",
                          modulationName(c.m), fecRateName(c.f),
                          r_plain.bit_errors, r_dd.bit_errors);
            ok = (r_plain.bit_errors == 0 && r_dd.bit_errors == 0);
        } else {
            long e_plain = 0, e_dd = 0;
            for (int s = 0; s < N_SEEDS; ++s) {
                uint32_t sd = 0x1234u + static_cast<uint32_t>(s) * 0x9E37u;
                e_plain += runCycle(c.m, c.f, 256, 48000, c.snr_db, 25, false, sd).bit_errors;
                e_dd    += runCycle(c.m, c.f, 256, 48000, c.snr_db, 25, true,  sd).bit_errors;
            }
            std::snprintf(label, sizeof(label),
                          "%-7s / %-5s @ marginal (%d seeds): plain errs=%ld  DD errs=%ld",
                          modulationName(c.m), fecRateName(c.f), N_SEEDS, e_plain, e_dd);
            // DD must not be materially worse than the baseline in aggregate.
            ok = (e_dd <= e_plain + N_SEEDS);
        }
        CHECK(ok, label);
    }
}

// =========================================================================
// Test 9: BICM-ID iterative decoder round-trips and converges
// =========================================================================
void test_bicm_id() {
    std::printf("\n=== Test 9: BICM-ID iterative decoder ===\n");
    struct Cfg { Modulation m; FECRate f; size_t iters; };
    Cfg configs[] = {
        { Modulation::QPSK,  FECRate::Rate_1_2,  1 }, // single-pass equivalent
        { Modulation::QAM16, FECRate::Rate_3_4,  3 },
        { Modulation::QAM64, FECRate::Rate_2_3,  3 },
    };
    for (auto& c : configs) {
        OFDMParams ofdm;
        ofdm.fft_size      = 256;
        ofdm.modulation    = c.m;
        ofdm.sample_rate   = 48000;
        ofdm.cyclic_prefix = CyclicPrefix::CP_1_8;

        LDPCEncoder enc(c.f, LDPCBlockSize::Short);
        LDPCDecoder dec(c.f, LDPCBlockSize::Short, 25);
        BitInterleaver inter(enc.codewordBits());
        SymbolMapper mapper(c.m);
        OFDMModulator tx_mod(ofdm);
        OFDMDemodulator rx_demod(ofdm);

        BICMConfig bc;
        bc.outer_iterations = c.iters;
        bc.ldpc_inner_iter  = 25;
        bc.use_extrinsic    = true;
        BICMDecoder bicm(&mapper, &inter, &dec, bc);

        std::mt19937 rng(0xB1CB1C);
        std::vector<uint8_t> info((enc.infoBits() + 7) / 8, 0);
        for (auto& b : info) b = static_cast<uint8_t>(rng() & 0xFF);
        std::vector<uint8_t> cw((enc.codewordBits() + 7) / 8, 0);
        enc.encode(info.data(), cw.data());
        std::vector<uint8_t> ileaved(cw.size(), 0);
        inter.interleave(cw.data(), ileaved.data());

        ComplexBuf preamble = tx_mod.generatePreamble();
        ComplexBuf payload;
        tx_mod.modulateBits(ileaved.data(), enc.codewordBits(), payload);

        size_t short_total = 10 * (ofdm.fft_size / 4);
        size_t sym_len = ofdm.symbolLength();
        ComplexBuf long_syms(preamble.begin() + static_cast<ptrdiff_t>(short_total),
                              preamble.begin() + static_cast<ptrdiff_t>(short_total) + 2 * sym_len);
        rx_demod.processPreamble(long_syms);

        ComplexBuf agg_syms;
        for (size_t off = 0; off + sym_len <= payload.size(); off += sym_len) {
            ComplexBuf one(payload.begin() + static_cast<ptrdiff_t>(off),
                            payload.begin() + static_cast<ptrdiff_t>(off + sym_len));
            ComplexBuf eq;
            if (!rx_demod.demodulate(one, eq)) break;
            agg_syms.insert(agg_syms.end(), eq.begin(), eq.end());
        }

        std::vector<uint8_t> decoded((enc.infoBits() + 7) / 8, 0);
        auto r = bicm.decodeIterative(agg_syms, rx_demod.noiseVariance(),
                                       decoded.data());
        int errs = 0;
        for (size_t i = 0; i < enc.infoBits(); ++i) {
            bool a = (decoded[i >> 3] >> (7 - (i & 7))) & 1;
            bool b = (info[i >> 3]    >> (7 - (i & 7))) & 1;
            if (a != b) ++errs;
        }
        char label[160];
        std::snprintf(label, sizeof(label),
                      "%-7s / %-5s × %zu iter: converged=%d err=%d",
                      modulationName(c.m), fecRateName(c.f), c.iters,
                      r.converged ? 1 : 0, errs);
        CHECK(r.converged && errs == 0, label);
    }

    // Noisy case: forces the iterative prior-aware demapper (demapWithPriors)
    // to actually run — on a clean channel iteration 0 converges and the
    // re-demap is never reached. At a marginal SNR the 3-iteration BICM-ID
    // must not do WORSE than a single non-iterative pass (the iterative
    // demap is the point of #10; we assert non-regression, which also
    // exercises the new code path end-to-end).
    {
        Modulation m = Modulation::QAM64;
        FECRate    f = FECRate::Rate_3_4;
        uint16_t fft = 256; uint32_t sr = 48000;
        OFDMParams ofdm; ofdm.fft_size = fft; ofdm.modulation = m;
        ofdm.sample_rate = sr; ofdm.cyclic_prefix = CyclicPrefix::CP_1_8;
        LDPCEncoder enc(f, LDPCBlockSize::Short);
        LDPCDecoder dec(f, LDPCBlockSize::Short, 25);
        BitInterleaver inter(enc.codewordBits());
        SymbolMapper mapper(m);
        OFDMModulator tx_mod(ofdm);
        OFDMDemodulator rx_demod(ofdm);

        std::mt19937 rng(0xB1C0FFEE);
        std::vector<uint8_t> info((enc.infoBits() + 7) / 8, 0);
        for (auto& b : info) b = static_cast<uint8_t>(rng() & 0xFF);
        std::vector<uint8_t> cw((enc.codewordBits() + 7) / 8, 0);
        enc.encode(info.data(), cw.data());
        std::vector<uint8_t> ileaved(cw.size(), 0);
        inter.interleave(cw.data(), ileaved.data());
        ComplexBuf preamble = tx_mod.generatePreamble();
        ComplexBuf payload;
        tx_mod.modulateBits(ileaved.data(), enc.codewordBits(), payload);
        ComplexBuf channel = preamble;
        channel.insert(channel.end(), payload.begin(), payload.end());

        // Add AWGN near the 64-QAM/3/4 cliff (~15 dB), where the codeword
        // partially decodes and the LDPC-posterior priors feeding the
        // iterative re-demap carry real information. (The old 8 dB anchor
        // was deep in the failure region — ~14% BER both ways — where the
        // iter-1 vs iter-3 comparison is a coin flip on the noise seed.)
        float sig = 0.f; for (auto& s : channel) sig += std::norm(s);
        sig /= static_cast<float>(channel.size());
        float nvar = sig / std::pow(10.f, 14.0f / 10.f);
        float sigma = std::sqrt(nvar * 0.5f);
        std::normal_distribution<float> nd(0.f, sigma);
        for (auto& s : channel) s += ComplexSample(nd(rng), nd(rng));

        size_t short_total = 10 * (fft / 4);
        size_t sym_len = ofdm.symbolLength();
        ComplexBuf long_syms(channel.begin() + static_cast<ptrdiff_t>(short_total),
                              channel.begin() + static_cast<ptrdiff_t>(short_total) + 2 * sym_len);
        rx_demod.processPreamble(long_syms);
        float nv = rx_demod.noiseVariance();

        ComplexBuf agg;
        for (size_t off = preamble.size(); off + sym_len <= channel.size(); off += sym_len) {
            ComplexBuf one(channel.begin() + static_cast<ptrdiff_t>(off),
                            channel.begin() + static_cast<ptrdiff_t>(off + sym_len));
            ComplexBuf eq;
            if (!rx_demod.demodulate(one, eq)) break;
            agg.insert(agg.end(), eq.begin(), eq.end());
        }

        auto runIters = [&](size_t iters) {
            BICMConfig bc; bc.outer_iterations = iters; bc.ldpc_inner_iter = 25;
            bc.use_extrinsic = true;
            BICMDecoder bicm(&mapper, &inter, &dec, bc);
            std::vector<uint8_t> dout((enc.infoBits() + 7) / 8, 0);
            bicm.decodeIterative(agg, nv, dout.data());
            int e = 0;
            for (size_t i = 0; i < enc.infoBits(); ++i) {
                bool a = (dout[i >> 3] >> (7 - (i & 7))) & 1;
                bool b = (info[i >> 3] >> (7 - (i & 7))) & 1;
                if (a != b) ++e;
            }
            return e;
        };
        int e1 = runIters(1);
        int e3 = runIters(3);
        char label[176];
        // With GRAY labeling the demapper EXIT transfer is nearly flat, so
        // BICM-ID iteration is documented as ~zero-gain here (anti-Gray is
        // the labeling that iterates productively — labeling_test asserts
        // strict convergence gains there, with the same 1/sigma^2 metric).
        // This gate therefore asserts BOUNDED behavior, not improvement:
        // the iterative path must run end-to-end without blowing up
        // (allow ~10% + 5 bits of cliff-noise wobble around iter-1).
        std::snprintf(label, sizeof(label),
                      "64-QAM / 3/4 @ 14 dB: BICM-ID 3-iter err=%d "
                      "within wobble of 1-iter err=%d (Gray: ~no-gain regime)",
                      e3, e1);
        CHECK(e3 <= e1 + e1 / 10 + 5, label);
    }

    // ---------------------------------------------------------------------
    // SOGRAND-inner BICM path: wiring guard. The engine can swap the BICM-ID
    // inner ISoftDecoder from BP to SOGRAND (ORBGRANDDecoder) via
    // AudioEngineConfig::bicm_inner_sogrand. This block exercises that exact
    // polymorphic path end-to-end on a clean channel so the wiring stays
    // covered REGARDLESS of the runtime default (which is BP — see the A/B
    // block below). It must converge and decode with zero info-bit errors.
    // ---------------------------------------------------------------------
    {
        Modulation m = Modulation::QAM16;
        FECRate    f = FECRate::Rate_3_4;
        OFDMParams ofdm; ofdm.fft_size = 256; ofdm.modulation = m;
        ofdm.sample_rate = 48000; ofdm.cyclic_prefix = CyclicPrefix::CP_1_8;

        LDPCEncoder enc(f, LDPCBlockSize::Short);
        ORBGRANDConfig ocfg; ocfg.max_queries = 5000; ocfg.max_weight = 4;
        ORBGRANDDecoder sogrand(f, LDPCBlockSize::Short, ocfg);  // ISoftDecoder
        BitInterleaver inter(enc.codewordBits());
        SymbolMapper mapper(m);
        OFDMModulator tx_mod(ofdm);
        OFDMDemodulator rx_demod(ofdm);

        BICMConfig bc; bc.outer_iterations = 3; bc.ldpc_inner_iter = 25;
        bc.use_extrinsic = true;
        ISoftDecoder* inner = &sogrand;            // bind ORBGRAND as ISoftDecoder*
        BICMDecoder bicm(&mapper, &inter, inner, bc);

        std::mt19937 rng(0x50C9A4D);
        std::vector<uint8_t> info((enc.infoBits() + 7) / 8, 0);
        for (auto& b : info) b = static_cast<uint8_t>(rng() & 0xFF);
        std::vector<uint8_t> cw((enc.codewordBits() + 7) / 8, 0);
        enc.encode(info.data(), cw.data());
        std::vector<uint8_t> ileaved(cw.size(), 0);
        inter.interleave(cw.data(), ileaved.data());

        ComplexBuf preamble = tx_mod.generatePreamble();
        ComplexBuf payload;
        tx_mod.modulateBits(ileaved.data(), enc.codewordBits(), payload);
        size_t short_total = 10 * (ofdm.fft_size / 4);
        size_t sym_len = ofdm.symbolLength();
        ComplexBuf long_syms(preamble.begin() + static_cast<ptrdiff_t>(short_total),
                              preamble.begin() + static_cast<ptrdiff_t>(short_total) + 2 * sym_len);
        rx_demod.processPreamble(long_syms);

        ComplexBuf agg;
        for (size_t off = 0; off + sym_len <= payload.size(); off += sym_len) {
            ComplexBuf one(payload.begin() + static_cast<ptrdiff_t>(off),
                            payload.begin() + static_cast<ptrdiff_t>(off + sym_len));
            ComplexBuf eq;
            if (!rx_demod.demodulate(one, eq)) break;
            agg.insert(agg.end(), eq.begin(), eq.end());
        }
        std::vector<uint8_t> dec((enc.infoBits() + 7) / 8, 0);
        auto r = bicm.decodeIterative(agg, rx_demod.noiseVariance(), dec.data());
        int errs = 0;
        for (size_t i = 0; i < enc.infoBits(); ++i) {
            bool a = (dec[i >> 3] >> (7 - (i & 7))) & 1;
            bool b = (info[i >> 3] >> (7 - (i & 7))) & 1;
            if (a != b) ++errs;
        }
        char label[160];
        std::snprintf(label, sizeof(label),
                      "16-QAM / 3/4 clean: BICM-ID over SOGRAND inner converged=%d err=%d",
                      r.converged ? 1 : 0, errs);
        CHECK(r.converged && errs == 0, label);
    }

    // ---------------------------------------------------------------------
    // BP-inner vs SOGRAND-inner A/B (the measurement that governs the
    // default). Both inner decoders see the IDENTICAL received-symbol + noise
    // realization, averaged over several seeds, on AWGN at a near-knee SNR for
    // QAM16/3-4 and QAM64/2-3. The verdict (reproducible here): BP-inner's
    // info-bit and frame-error counts are <= SOGRAND-inner's — List-GRAND's
    // posterior is over only L candidates, so at these short low/medium-rate
    // blocks its APP is biased whenever the ML codeword falls outside the
    // list. This is WHY the engine default (bicm_inner_sogrand) is false (BP).
    // The assertion guards that the data still supports that default.
    // ---------------------------------------------------------------------
    {
        struct AB { Modulation m; FECRate f; float snr_db; };
        AB cases[] = {
            { Modulation::QAM16, FECRate::Rate_3_4, 11.f },
            { Modulation::QAM64, FECRate::Rate_2_3, 16.f },
        };
        const size_t N_SEEDS = 8;
        for (auto& c : cases) {
            OFDMParams ofdm; ofdm.fft_size = 256; ofdm.modulation = c.m;
            ofdm.sample_rate = 48000; ofdm.cyclic_prefix = CyclicPrefix::CP_1_8;
            LDPCEncoder enc(c.f, LDPCBlockSize::Short);
            BitInterleaver inter(enc.codewordBits());
            SymbolMapper mapper(c.m);
            LDPCDecoder bp_dec(c.f, LDPCBlockSize::Short, 25);
            ORBGRANDConfig ocfg; ocfg.max_queries = 5000; ocfg.max_weight = 4;
            ORBGRANDDecoder so_dec(c.f, LDPCBlockSize::Short, ocfg);

            size_t kbits = enc.infoBits();
            int bp_bit = 0, bp_frm = 0, so_bit = 0, so_frm = 0;

            for (size_t seed = 0; seed < N_SEEDS; ++seed) {
                std::mt19937 rng(0xB1A50000u + static_cast<unsigned>(seed) * 7919u
                                 + static_cast<unsigned>(c.snr_db * 31));
                std::vector<uint8_t> info((kbits + 7) / 8, 0);
                for (auto& b : info) b = static_cast<uint8_t>(rng() & 0xFF);
                std::vector<uint8_t> cw((enc.codewordBits() + 7) / 8, 0);
                enc.encode(info.data(), cw.data());
                std::vector<uint8_t> ileaved(cw.size(), 0);
                inter.interleave(cw.data(), ileaved.data());

                OFDMModulator tx_mod(ofdm);
                OFDMDemodulator rx_demod(ofdm);
                ComplexBuf preamble = tx_mod.generatePreamble();
                ComplexBuf payload;
                tx_mod.modulateBits(ileaved.data(), enc.codewordBits(), payload);
                ComplexBuf channel = preamble;
                channel.insert(channel.end(), payload.begin(), payload.end());

                // AWGN — the SAME samples drive both inner decoders.
                float sig = 0.f; for (auto& s : channel) sig += std::norm(s);
                sig /= static_cast<float>(channel.size());
                float nvar = sig / std::pow(10.f, c.snr_db / 10.f);
                float sigma = std::sqrt(nvar * 0.5f);
                std::normal_distribution<float> nd(0.f, sigma);
                for (auto& s : channel) s += ComplexSample(nd(rng), nd(rng));

                size_t short_total = 10 * (ofdm.fft_size / 4);
                size_t sym_len = ofdm.symbolLength();
                ComplexBuf long_syms(channel.begin() + static_cast<ptrdiff_t>(short_total),
                                      channel.begin() + static_cast<ptrdiff_t>(short_total) + 2 * sym_len);
                rx_demod.processPreamble(long_syms);
                float nv = rx_demod.noiseVariance();

                ComplexBuf agg;
                for (size_t off = preamble.size(); off + sym_len <= channel.size(); off += sym_len) {
                    ComplexBuf one(channel.begin() + static_cast<ptrdiff_t>(off),
                                    channel.begin() + static_cast<ptrdiff_t>(off + sym_len));
                    ComplexBuf eq;
                    if (!rx_demod.demodulate(one, eq)) break;
                    agg.insert(agg.end(), eq.begin(), eq.end());
                }

                auto run = [&](ISoftDecoder* inner) {
                    BICMConfig bc; bc.outer_iterations = 3; bc.ldpc_inner_iter = 25;
                    bc.use_extrinsic = true;
                    BICMDecoder bicm(&mapper, &inter, inner, bc);
                    std::vector<uint8_t> dout((kbits + 7) / 8, 0);
                    bicm.decodeIterative(agg, nv, dout.data());
                    int e = 0;
                    for (size_t i = 0; i < kbits; ++i) {
                        bool a = (dout[i >> 3] >> (7 - (i & 7))) & 1;
                        bool b = (info[i >> 3] >> (7 - (i & 7))) & 1;
                        if (a != b) ++e;
                    }
                    return e;
                };
                int eb = run(&bp_dec);
                int es = run(&so_dec);
                bp_bit += eb; if (eb) ++bp_frm;
                so_bit += es; if (es) ++so_frm;
            }

            char label[200];
            std::snprintf(label, sizeof(label),
                "%-7s / %-5s @ %.0f dB A/B: BP-inner %d bitErr/%d frmErr "
                "<= SOGRAND-inner %d bitErr/%d frmErr",
                modulationName(c.m), fecRateName(c.f), c.snr_db,
                bp_bit, bp_frm, so_bit, so_frm);
            // Measured default-governing assertion: BP-inner is no worse than
            // SOGRAND-inner on both bit-error and frame-error counts.
            CHECK(bp_bit <= so_bit && bp_frm <= so_frm, label);
        }
    }
}

} // anonymous

// =========================================================================
// =========================================================================
// Test 10: sync-hardening paths — MMSE Wiener (#46) and the phase-tracker
// decision-directed phase frame (#36). The default RX path enables neither,
// so without this these fixes would ship untested. A small injected CFO
// gives the phase tracker a real per-symbol drift to follow.
// =========================================================================
void test_sync_hardening_paths() {
    std::printf("\n=== Test 10: MMSE / phase-tracker / DD sync-hardening ===\n");
    struct Cfg {
        Modulation m; FECRate f; float snr; bool dd; bool mmse; bool phase;
        float cfo; const char* tag;
    };
    Cfg cfgs[] = {
        { Modulation::QAM16, FECRate::Rate_3_4, -1.f, false, true,  false, 0.f, "MMSE clean" },
        { Modulation::QAM16, FECRate::Rate_3_4, 18.f, false, true,  false, 0.f, "MMSE @18dB" },
        // A 2nd-order PLL needs a few symbols to lock the frequency; a single
        // ~10-symbol codeword tolerates a modest CFO (a few Hz). Larger
        // offsets are the SFO/AFC loop's job, not this per-codeword smoke.
        { Modulation::QPSK,  FECRate::Rate_1_2, -1.f, true,  false, true,  3.f, "phase+DD QPSK +3Hz" },
        { Modulation::QAM16, FECRate::Rate_3_4, -1.f, true,  false, true,  2.f, "phase+DD QAM16 +2Hz" },
        { Modulation::QAM16, FECRate::Rate_3_4, -1.f, true,  true,  true,  2.f, "MMSE+phase+DD +2Hz" },
    };
    for (auto& c : cfgs) {
        auto r = runCycle(c.m, c.f, 256, 48000, c.snr, 25, c.dd, 0x1234u,
                          c.mmse, c.phase, c.cfo);
        char label[176];
        std::snprintf(label, sizeof(label),
                      "%-26s built=%d decoded=%d err=%d",
                      c.tag, r.tx_built, r.rx_decoded, r.bit_errors);
        CHECK(r.tx_built && r.bit_errors == 0, label);
    }
}

// =========================================================================
// Test 11: CFO robustness of the DEFAULT receive path.
//
// Regression guard for fractional carrier-frequency-offset correction.
// Historically the RX path applied NO CFO correction (coarseSync computed
// freq_offset_hz but the engine discarded it), so even a few tens of Hz of
// soundcard/SSB carrier offset wrecked high-order QAM: 64-QAM failed at just
// 5 Hz. OFDMDemodulator now estimates the fractional CFO from the two
// identical long-preamble symbols (Moose) and derotates the baseband before
// every FFT, so the apparent channel stops rotating symbol-to-symbol.
//
// Subcarrier spacing here is 48000/256 = 187.5 Hz; the fractional-CFO capture
// range is fs/(2*sym_len) ~ +/-83 Hz. This sweep asserts ZERO info-bit errors
// on a clean channel across the swept offsets (kept inside the capture range
// with margin). Offsets beyond the capture range need integer-CFO correction,
// which is still report-only (SOTA roadmap).
// =========================================================================
void test_cfo_robustness() {
    std::printf("\n=== Test 11: CFO robustness of the default RX path ===\n");
    const float cfos[] = {0.f, 5.f, 10.f, 20.f, 40.f, 60.f};
    struct MC { Modulation m; FECRate f; };
    MC cases[] = {
        { Modulation::QPSK,  FECRate::Rate_1_2 },
        { Modulation::QAM16, FECRate::Rate_1_2 },
        { Modulation::QAM64, FECRate::Rate_2_3 },
    };
    const uint32_t sr = 48000; const uint16_t fft = 256;
    const float sc_spacing = static_cast<float>(sr) / static_cast<float>(fft);
    const size_t cp = fft / 8; // CP_1_8 used by runCycle
    const float frac_range = static_cast<float>(sr) /
                             (2.f * static_cast<float>(fft + cp));
    std::printf("    subcarrier spacing = %.1f Hz;  fractional-CFO capture range ~ +/-%.0f Hz\n",
                sc_spacing, frac_range);
    std::printf("    %-7s | %-5s |", "mod", "fec");
    for (float c : cfos) std::printf("  %4.0fHz", c);
    std::printf("    <- info-bit errors (clean channel, CFO-corrected)\n");

    for (auto& mc : cases) {
        std::printf("    %-7s | %-5s |", modulationName(mc.m), fecRateName(mc.f));
        int total_err = 0;
        for (float c : cfos) {
            auto r = runCycle(mc.m, mc.f, fft, sr, /*clean*/ -1.f, 25,
                              /*dd*/ false, 0x1234u, /*mmse*/ false,
                              /*phase*/ false, /*cfo*/ c);
            int e = r.tx_built ? r.bit_errors : -1;
            if (e != 0) total_err += (e < 0 ? 1 : e);
            std::printf("  %5d", e);
        }
        std::printf("\n");
        char label[96];
        std::snprintf(label, sizeof(label),
                      "%-7s / %-5s: 0 errors across +/-%.0f Hz CFO (was broken by 5 Hz)",
                      modulationName(mc.m), fecRateName(mc.f), cfos[5]);
        CHECK(total_err == 0, label);
    }
}

// =========================================================================
// Test 12: frequency-selective channel — per-subcarrier LLR weighting.
//
// On a multipath channel |H(k)| varies across subcarriers. ZF equalization
// (Y/H) enhances noise on faded bins by 1/|H(k)|^2, but the soft demapper is
// handed a single scalar noise variance for ALL bins, so it OVER-TRUSTS the
// LLRs of faded subcarriers and the LDPC decoder is misled. Weighting each
// bin's LLRs by its true effective noise (∝ 1/|H(k)|^2) is the classic fix.
//
// This sweep runs high-order QAM over a 2-ray selective channel at several
// SNRs and reports info-bit errors (averaged over noise realizations). It is
// the before/after harness for per-bin LLR weighting.
// =========================================================================
void test_selective_channel_llr() {
    std::printf("\n=== Test 12: selective-channel LLR weighting (A/B) ===\n");
    // 2-ray channel, echo at delay 3 (< CP=32), amplitude 0.7 → |H|^2 spans
    // ~[0.09, 2.89]: strong frequency selectivity with ~-10 dB fades.
    ComplexBuf taps = { ComplexSample(1.f, 0.f), ComplexSample(0.f, 0.f),
                        ComplexSample(0.f, 0.f), ComplexSample(0.7f, 0.f) };
    struct Cfg { Modulation m; FECRate f; float snr; };
    Cfg cfgs[] = {
        { Modulation::QAM16,  FECRate::Rate_1_2, 10.f },
        { Modulation::QAM64,  FECRate::Rate_2_3, 20.f },
        { Modulation::QAM256, FECRate::Rate_1_2, 22.f },
    };
    constexpr int N_SEEDS = 8;
    std::printf("    2-ray channel (echo@delay3 amp 0.7); info-bit errors over %d seeds\n",
                N_SEEDS);
    std::printf("    %-7s | %-5s | %-5s | scalar-noise | per-bin |H|^2\n",
                "mod", "fec", "SNR");
    for (auto& c : cfgs) {
        long off = 0, on = 0;
        for (int s = 0; s < N_SEEDS; ++s) {
            uint32_t sd = 0x51A1u + static_cast<uint32_t>(s) * 0x9E37u;
            // Same seed → SAME channel + noise realization for both arms; only
            // the LLR weighting differs.
            off += runCycle(c.m, c.f, 256, 48000, c.snr, 25, false, sd,
                            false, false, 0.f, &taps, /*per_bin_llr*/ false).bit_errors;
            on  += runCycle(c.m, c.f, 256, 48000, c.snr, 25, false, sd,
                            false, false, 0.f, &taps, /*per_bin_llr*/ true ).bit_errors;
        }
        if (off < 0) off = 0; if (on < 0) on = 0;
        std::printf("    %-7s | %-5s | %4.0fdB |   %8ld   |   %8ld\n",
                    modulationName(c.m), fecRateName(c.f), c.snr, off, on);
        char label[160];
        std::snprintf(label, sizeof(label),
                      "%-7s/%-5s selective: per-bin |H|^2 weighting beats scalar "
                      "(%ld -> %ld errs)",
                      modulationName(c.m), fecRateName(c.f), off, on);
        // Per-bin weighting must strictly help on a frequency-selective channel.
        CHECK(on < off, label);
    }
}

// =========================================================================
// Test 13: PAPR tone-reservation does not corrupt data.
//
// Reserved peak-reduction tones are now carved OUT of the data allocation
// (computeAllocation) so TX and RX agree they carry no data. Previously the
// reducer stole live data carriers and zeroed them, so the RX demapped garbage
// there. This runs the full chain with PAPR-TR enabled and asserts the payload
// still decodes bit-exact on a clean channel.
// =========================================================================
void test_papr_integrity() {
    std::printf("\n=== Test 13: PAPR tone-reservation data integrity ===\n");
    struct MC { Modulation m; FECRate f; };
    MC cases[] = {
        { Modulation::QPSK,  FECRate::Rate_1_2 },
        { Modulation::QAM16, FECRate::Rate_3_4 },
        { Modulation::QAM64, FECRate::Rate_2_3 },
    };
    for (auto& c : cases) {
        auto r = runCycle(c.m, c.f, 256, 48000, /*clean*/ -1.f, 25,
                          /*dd*/ false, 0x1234u, /*mmse*/ false,
                          /*phase*/ false, /*cfo*/ 0.f, /*taps*/ nullptr,
                          /*per_bin_llr*/ true, /*papr_on*/ true);
        char label[140];
        std::snprintf(label, sizeof(label),
                      "%-7s / %-5s with PAPR-TR: decoded=%d err=%d "
                      "(reserved tones carry no data)",
                      modulationName(c.m), fecRateName(c.f),
                      r.rx_decoded, r.bit_errors);
        CHECK(r.tx_built && r.rx_decoded && r.bit_errors == 0, label);
    }
}

// =========================================================================
// Test 14: acquisition under a timing offset — exercises the wired fineSync.
//
// The live RX no longer assumes the preamble is at sample 0. coarseSync
// (Schmidl-Cox CP autocorrelation) snaps to *a* symbol boundary, then fineSync
// cross-correlates against the known Zadoff-Chu long-preamble body to pin the
// FFT window to the exact sample. This test prepends K samples of low-level
// noise so the preamble sits MID-buffer, for K both < one FFT length and >
// one FFT length (and a leading-noise case at mild SNR), then runs the REAL
// OFDMSynchronizer + OFDMDemodulator acquisition exactly as the engine does
// and asserts the payload decodes bit-exact (or CRC-clean at mild SNR).
//
// Without the fineSync wiring the FFT window would be misplaced by K mod
// sym_len samples and the payload would fail — so this directly verifies the
// new acquisition path, not just that it compiles.
// =========================================================================
namespace {
struct AcqResult { bool found = false; int bit_errors = -1; bool crc_ok = false;
                   size_t pre_start = 0; };
} // namespace

void test_acquisition_timing_offset() {
    std::printf("\n=== Test 14: acquisition under timing offset (fineSync) ===\n");

    OFDMParams ofdm;
    ofdm.fft_size      = 256;
    ofdm.modulation    = Modulation::QPSK;
    ofdm.sample_rate   = 48000;
    ofdm.cyclic_prefix = CyclicPrefix::CP_1_8;

    const FECRate fec = FECRate::Rate_1_2;
    const size_t sym_len     = ofdm.symbolLength();
    const size_t short_total = 10 * (ofdm.fft_size / 4);
    const size_t preamble_len = short_total + 2 * sym_len;
    const size_t cp = ofdm.cpLength();

    LDPCEncoder enc(fec, LDPCBlockSize::Short);
    LDPCDecoder dec(fec, LDPCBlockSize::Short, 25);
    BitInterleaver inter(enc.codewordBits());
    OFDMModulator   tx_mod(ofdm);

    // ---- Build one TX frame (preamble + payload codeword) ----
    std::mt19937 rng(0xACC0FF5Eu);
    const size_t k_bytes = enc.infoBytes();
    const size_t n_bytes = enc.codewordBytes();
    size_t cap = (k_bytes > constants::FRAME_OVERHEAD)
                 ? k_bytes - constants::FRAME_OVERHEAD : 0;
    FrameBuilder fb(cap);
    std::vector<uint8_t> packet_data(cap > 4 ? cap - 4 : 0, 0);
    for (auto& b : packet_data) b = static_cast<uint8_t>(rng() & 0xFF);
    fb.addPacket(0, packet_data.data(), packet_data.size());
    auto frame_bytes = fb.build(/*frame_no*/ 1, fec, ofdm.modulation);
    frame_bytes.resize(k_bytes, 0);

    std::vector<uint8_t> codeword(n_bytes, 0);
    enc.encode(frame_bytes.data(), codeword.data());
    std::vector<uint8_t> ileaved(n_bytes, 0);
    inter.interleave(codeword.data(), ileaved.data());

    ComplexBuf preamble = tx_mod.generatePreamble();
    ComplexBuf payload;
    tx_mod.modulateBits(ileaved.data(), enc.codewordBits(), payload);

    ComplexBuf frame = preamble;
    frame.insert(frame.end(), payload.begin(), payload.end());

    const size_t bits_per_ofdm = tx_mod.bitsPerOFDMSymbol();
    const size_t syms_per_cw =
        (enc.codewordBits() + bits_per_ofdm - 1) / bits_per_ofdm;

    // Run the REAL synchronizer + demod acquisition on a stream with K leading
    // samples prepended, then demod the payload and count info-bit errors.
    auto runAt = [&](size_t K, float drift_ppm, float snr_db,
                     bool noise_prefix, uint32_t seed) -> AcqResult {
        std::mt19937 r2(seed);
        ComplexBuf stream;
        stream.reserve(K + frame.size() + sym_len);

        // Leading slack: low-level noise (or zeros) so the preamble is NOT at 0.
        std::normal_distribution<float> lead(0.f, 0.02f); // ~-34 dB vs unit sig
        for (size_t i = 0; i < K; ++i) {
            stream.push_back(noise_prefix
                ? ComplexSample(lead(r2), lead(r2))
                : ComplexSample(0.f, 0.f));
        }

        // Optional per-sample timing drift: resample the frame by (1+ppm) via
        // nearest-neighbour. Small ppm exercises trackTiming's tolerance window
        // without breaking the within-CP guard.
        ComplexBuf body = frame;
        if (drift_ppm != 0.f) {
            double ratio = 1.0 + static_cast<double>(drift_ppm) * 1e-6;
            ComplexBuf rs;
            rs.reserve(frame.size());
            for (size_t n = 0; ; ++n) {
                double src = static_cast<double>(n) / ratio;
                size_t si = static_cast<size_t>(src + 0.5);
                if (si >= frame.size()) break;
                rs.push_back(frame[si]);
            }
            body = std::move(rs);
        }
        stream.insert(stream.end(), body.begin(), body.end());

        // A little trailing slack so fineSync's +N/8 search never runs off end.
        for (size_t i = 0; i < sym_len; ++i)
            stream.push_back(ComplexSample(0.f, 0.f));

        // Optional AWGN over the whole stream.
        if (snr_db >= 0.f) {
            float sig = 0.f;
            for (auto& s : body) sig += std::norm(s);
            sig /= static_cast<float>(std::max<size_t>(body.size(), 1));
            float nv = sig / std::pow(10.f, snr_db / 10.f);
            float sigma = std::sqrt(nv * 0.5f);
            std::normal_distribution<float> nd(0.f, sigma);
            for (auto& s : stream) s += ComplexSample(nd(r2), nd(r2));
        }

        // ---- Acquisition: fineSync ACQUIRE scan over the buffer ----
        // The preamble may start an arbitrary K samples into the buffer (more
        // than a symbol), so a CP-autocorrelation coarse search (which only
        // localizes within one symbol) cannot place it. fineSync cross-
        // correlates against the known ZC long-preamble body; given a wide
        // search_range it scans the whole buffer and locks onto the exact body
        // start. This is the path the new wiring exposes. We centre the scan on
        // the buffer mid-point with a range covering the full buffer.
        OFDMSynchronizer sync(ofdm);
        OFDMDemodulator  demod(ofdm);
        AcqResult r{};
        ComplexBuf buf = stream;
        if (buf.size() < preamble_len) return r;
        size_t pre_start = 0;
        {
            // Body is at preamble_start + short_total + cp; scan the entire
            // buffer for it (centre + range spans [0, size)).
            int centre = static_cast<int>(buf.size() / 2);
            size_t wide = buf.size();
            SyncResult fr;
            if (sync.fineSync(buf, centre, fr, wide) && fr.valid) {
                long ps = static_cast<long>(fr.timing_offset)
                        - static_cast<long>(cp) - static_cast<long>(short_total);
                if (ps < 0) ps = 0;
                if (static_cast<size_t>(ps) + preamble_len > buf.size())
                    ps = static_cast<long>(buf.size() - preamble_len);
                pre_start = static_cast<size_t>(ps);
            }
        }
        r.pre_start = pre_start;
        if (pre_start + preamble_len > buf.size()) return r;

        ComplexBuf long_syms(
            buf.begin() + static_cast<ptrdiff_t>(pre_start + short_total),
            buf.begin() + static_cast<ptrdiff_t>(pre_start + preamble_len));
        if (!demod.processPreamble(long_syms)) return r;
        r.found = true;

        // ---- Demod the payload that follows the preamble ----
        size_t data_off = pre_start + preamble_len;
        std::vector<float> all_llrs;
        for (size_t s = 0; s < syms_per_cw; ++s) {
            size_t off = data_off + s * sym_len;
            if (off + sym_len > buf.size()) break;
            ComplexBuf one(buf.begin() + static_cast<ptrdiff_t>(off),
                           buf.begin() + static_cast<ptrdiff_t>(off + sym_len));
            std::vector<float> llrs;
            demod.demodulateSoft(one, llrs, demod.noiseVariance());
            all_llrs.insert(all_llrs.end(), llrs.begin(), llrs.end());
        }
        all_llrs.resize(enc.codewordBits(), 0.f);
        std::vector<float> deint(all_llrs.size());
        inter.deinterleave(all_llrs.data(), deint.data());
        std::vector<uint8_t> decoded(k_bytes, 0);
        dec.decode(deint.data(), decoded.data());

        int errs = 0;
        size_t info_bits = enc.infoBits();
        for (size_t i = 0; i < info_bits; ++i) {
            bool a = (decoded[i >> 3]     >> (7 - (i & 7))) & 1;
            bool b = (frame_bytes[i >> 3]  >> (7 - (i & 7))) & 1;
            if (a != b) ++errs;
        }
        r.bit_errors = errs;
        ParsedFrame pf;
        if (FrameParser::parse(decoded, pf)) r.crc_ok = pf.crc_ok;
        return r;
    };

    // K < one FFT length (sub-symbol offset) — clean channel, exact decode.
    {
        size_t K = ofdm.fft_size / 3; // < N
        auto r = runAt(K, 0.f, /*snr*/ -1.f, /*noise*/ true, 0x1111u);
        char label[160];
        std::snprintf(label, sizeof(label),
            "K=%zu (<FFT) leading noise: acquired=%d pre_start=%zu err=%d crc=%d",
            K, r.found, r.pre_start, r.bit_errors, r.crc_ok);
        CHECK(r.found && r.bit_errors == 0 && r.crc_ok, label);
    }

    // K > one FFT length (multi-symbol offset) — clean channel, exact decode.
    {
        size_t K = ofdm.fft_size + ofdm.fft_size / 2 + 7; // > N, not a multiple
        auto r = runAt(K, 0.f, /*snr*/ -1.f, /*noise*/ true, 0x2222u);
        char label[160];
        std::snprintf(label, sizeof(label),
            "K=%zu (>FFT) leading noise: acquired=%d pre_start=%zu err=%d crc=%d",
            K, r.found, r.pre_start, r.bit_errors, r.crc_ok);
        CHECK(r.found && r.bit_errors == 0 && r.crc_ok, label);
    }

    // K > FFT + mild SNR + small timing drift — robustness, CRC-clean decode.
    {
        size_t K = 2 * ofdm.fft_size + 13;
        auto r = runAt(K, /*ppm*/ 20.f, /*snr*/ 20.f, /*noise*/ true, 0x3333u);
        char label[176];
        std::snprintf(label, sizeof(label),
            "K=%zu (>FFT) +20ppm drift @20dB: acquired=%d pre_start=%zu err=%d crc=%d",
            K, r.found, r.pre_start, r.bit_errors, r.crc_ok);
        CHECK(r.found && r.crc_ok && r.bit_errors == 0, label);
    }
}

// =========================================================================
// Test 15: PERSISTENT (cross-symbol) decision-directed channel tracking.
//
// The single-codeword runCycle CANNOT show this benefit: it has ONE preamble
// right before a short (~10-symbol) codeword, so pilot interpolation off the
// fresh preamble is already good and the channel barely moves. The win shows up
// when MANY data OFDM symbols follow a SINGLE preamble and the channel both
//   (a) is frequency-selective — a 2-3 tap echo within the CP, giving deep-ish
//       |H(k)| fades that pilot interpolation reconstructs imperfectly, and
//   (b) slowly TIME-VARIES — the echo tap's phase rotates and its amplitude
//       drifts a little per OFDM symbol, so as symbols accumulate the true
//       channel walks away from the one-shot preamble estimate.
//
// Persistent DD accumulates the data-bin decision-directed correction ACROSS
// symbols (dd_persist_), so it tracks that walk; intra-symbol DD (OFF) discards
// it every symbol and re-leans on pilot interpolation each time. We run the SAME
// channel + noise realization twice (use_dd=true both arms; only persistent
// toggled) and assert ON yields strictly fewer codeword/info-bit errors at an
// operating point where the drift is visible, with NO regression on a static
// selective channel or the clean case.
// =========================================================================
namespace {

// Build ONE preamble + many codewords of payload, push through a (optionally
// time-varying) frequency-selective channel + AWGN, then demod symbol-by-symbol
// with persistent DD on/off and count codeword + info-bit errors. Same seed →
// identical channel + noise realization, so the only difference between arms is
// the persistent-DD toggle. Returns {codeword_errors, info_bit_errors}.
struct DDResult { int cw_errors = 0; long bit_errors = 0; bool built = false; };

DDResult runPersistentDD(Modulation mod, FECRate fec, size_t n_codewords,
                         float snr_db, float echo_amp, float echo_drift_amp,
                         float echo_phase_step_rad, bool persistent,
                         uint32_t seed)
{
    DDResult res{};

    OFDMParams ofdm;
    ofdm.fft_size      = 256;
    ofdm.modulation    = mod;
    ofdm.sample_rate   = 48000;
    ofdm.cyclic_prefix = CyclicPrefix::CP_1_8;

    LDPCEncoder enc(fec, LDPCBlockSize::Short);
    LDPCDecoder dec(fec, LDPCBlockSize::Short, 25);
    BitInterleaver inter(enc.codewordBits());
    OFDMModulator   tx_mod(ofdm);
    OFDMDemodulator rx_demod(ofdm);
    rx_demod.setPersistentDD(persistent);

    const size_t k_bytes = enc.infoBytes();
    const size_t n_bytes = enc.codewordBytes();
    const size_t sym_len = ofdm.symbolLength();
    const size_t cp      = ofdm.cpLength();
    const size_t bits_per_ofdm = tx_mod.bitsPerOFDMSymbol();
    if (bits_per_ofdm == 0) return res;
    const size_t syms_per_cw = (enc.codewordBits() + bits_per_ofdm - 1)
                             / bits_per_ofdm;

    std::mt19937 rng(seed);

    // ---- Build all codewords' payload into one contiguous OFDM stream ----
    std::vector<std::vector<uint8_t>> info_blocks(n_codewords);
    ComplexBuf payload; // all data OFDM symbols, back-to-back
    for (size_t c = 0; c < n_codewords; ++c) {
        std::vector<uint8_t> info(k_bytes);
        for (auto& b : info) b = static_cast<uint8_t>(rng() & 0xFF);
        info_blocks[c] = info;

        std::vector<uint8_t> codeword(n_bytes, 0);
        if (!enc.encode(info.data(), codeword.data())) return res;
        std::vector<uint8_t> ileaved(n_bytes, 0);
        inter.interleave(codeword.data(), ileaved.data());

        ComplexBuf cw_bb;
        tx_mod.modulateBits(ileaved.data(), enc.codewordBits(), cw_bb);
        // modulateBits pads to a whole number of OFDM symbols already; ensure
        // each codeword occupies exactly syms_per_cw symbols so demod indexing
        // stays codeword-aligned.
        cw_bb.resize(syms_per_cw * sym_len, ComplexSample(0.f, 0.f));
        payload.insert(payload.end(), cw_bb.begin(), cw_bb.end());
    }

    ComplexBuf preamble = tx_mod.generatePreamble();
    ComplexBuf stream = preamble;
    stream.insert(stream.end(), payload.begin(), payload.end());

    // ---- Time-varying frequency-selective channel --------------------------
    // 2-ray: direct path (1.0) + an echo at delay `echo_delay` (< CP). The echo
    // tap's COMPLEX gain walks per OFDM symbol: its phase advances by
    // echo_phase_step_rad and its amplitude drifts sinusoidally by
    // echo_drift_amp. Static when both deltas are 0 (→ frequency-selective but
    // time-invariant). The walk is referenced to OFDM-symbol index so the
    // preamble (symbol-block 0) sees the base tap and the data symbols drift
    // away from it. Convolution is causal and the echo delay fits in the CP, so
    // the OFDM guard absorbs the spread (no ISI), keeping the per-bin equalizer
    // valid — only |H(k)| and its slow drift change.
    const size_t echo_delay = 3; // < CP (=32)
    auto echo_gain = [&](size_t sample_index) -> ComplexSample {
        // Which OFDM symbol (counting preamble as a block) this sample sits in.
        double t = static_cast<double>(sample_index) / static_cast<double>(sym_len);
        float amp = echo_amp + echo_drift_amp *
                    std::sin(0.21f * static_cast<float>(t));
        float ph  = echo_phase_step_rad * static_cast<float>(t);
        return std::polar(amp, ph);
    };
    {
        ComplexBuf conv(stream.size(), ComplexSample(0.f, 0.f));
        for (size_t n = 0; n < stream.size(); ++n) {
            ComplexSample acc = stream[n]; // direct path, unit gain
            if (n >= echo_delay)
                acc += echo_gain(n) * stream[n - echo_delay];
            conv[n] = acc;
        }
        stream = std::move(conv);
    }

    // ---- AWGN (same realization for both arms via shared seed) ----
    if (snr_db >= 0.f) {
        float sig = 0.f;
        for (auto& s : stream) sig += std::norm(s);
        sig /= static_cast<float>(stream.size());
        float nvar = sig / std::pow(10.f, snr_db / 10.f);
        float sigma = std::sqrt(nvar * 0.5f);
        std::normal_distribution<float> nd(0.f, sigma);
        for (auto& s : stream) s += ComplexSample(nd(rng), nd(rng));
    }

    // ---- RX: ONE preamble, then demod every data symbol with persistent DD --
    size_t short_total = 10 * (ofdm.fft_size / 4);
    if (stream.size() < short_total + 2 * sym_len) return res;
    ComplexBuf long_syms(stream.begin() + static_cast<ptrdiff_t>(short_total),
                         stream.begin() + static_cast<ptrdiff_t>(short_total) + 2 * sym_len);
    if (!rx_demod.processPreamble(long_syms)) return res;
    res.built = true;

    size_t data_off = preamble.size();
    for (size_t c = 0; c < n_codewords; ++c) {
        std::vector<float> all_llrs;
        for (size_t s = 0; s < syms_per_cw; ++s) {
            size_t off = data_off + (c * syms_per_cw + s) * sym_len;
            if (off + sym_len > stream.size()) break;
            ComplexBuf one(stream.begin() + static_cast<ptrdiff_t>(off),
                           stream.begin() + static_cast<ptrdiff_t>(off + sym_len));
            std::vector<float> llrs;
            rx_demod.demodulateSoftDD(one, llrs, rx_demod.noiseVariance(),
                                      /*use_pwl=*/false);
            all_llrs.insert(all_llrs.end(), llrs.begin(), llrs.end());
        }
        all_llrs.resize(enc.codewordBits(), 0.f);
        std::vector<float> deint(all_llrs.size());
        inter.deinterleave(all_llrs.data(), deint.data());
        std::vector<uint8_t> decoded(k_bytes, 0);
        dec.decode(deint.data(), decoded.data());

        int cw_bit_errs = 0;
        size_t info_bits = enc.infoBits();
        const auto& info = info_blocks[c];
        for (size_t i = 0; i < info_bits; ++i) {
            bool a = (decoded[i >> 3] >> (7 - (i & 7))) & 1;
            bool b = (info[i >> 3]    >> (7 - (i & 7))) & 1;
            if (a != b) ++cw_bit_errs;
        }
        res.bit_errors += cw_bit_errs;
        if (cw_bit_errs > 0) ++res.cw_errors;
    }
    (void)cp;
    return res;
}

} // namespace

void test_persistent_dd_tracking() {
    std::printf("\n=== Test 15: persistent (cross-symbol) DD channel tracking ===\n");

    // 16 codewords per stream so the time-varying channel walks meaningfully
    // away from the single preamble estimate by the end. Aggregate each gate
    // over several noise realizations so it doesn't hinge on one draw (the
    // same-seed channel+noise is shared between the OFF/ON arms, so only the
    // persistent-DD toggle differs within a seed).
    const size_t NCW = 16;
    const Modulation mod = Modulation::QAM16;
    const FECRate    fec = FECRate::Rate_3_4;
    constexpr int N_SEEDS = 10;
    auto seed_at = [](int s) { return 0xD0D0u + static_cast<uint32_t>(s) * 0x9E37u; };

    // Aggregate {off, on} codeword + bit errors over N_SEEDS realizations.
    struct AB { int off_cw = 0, on_cw = 0; long off_bits = 0, on_bits = 0;
                bool built = true; };
    auto sweep = [&](float snr, float amp, float drift, float ps) {
        AB ab{};
        for (int s = 0; s < N_SEEDS; ++s) {
            auto off = runPersistentDD(mod, fec, NCW, snr, amp, drift, ps,
                                       /*persistent*/ false, seed_at(s));
            auto on  = runPersistentDD(mod, fec, NCW, snr, amp, drift, ps,
                                       /*persistent*/ true,  seed_at(s));
            ab.off_cw += off.cw_errors; ab.on_cw += on.cw_errors;
            ab.off_bits += off.bit_errors; ab.on_bits += on.bit_errors;
            ab.built = ab.built && off.built && on.built;
        }
        return ab;
    };

    // --- (1) Clean flat channel: BOTH arms must be error-free (no harm). ---
    {
        auto ab = sweep(/*snr*/ -1.f, /*amp*/ 0.f, /*drift*/ 0.f, /*ps*/ 0.f);
        char label[176];
        std::snprintf(label, sizeof(label),
            "clean flat channel: persistent OFF bits=%ld  ON bits=%ld (both 0)",
            ab.off_bits, ab.on_bits);
        CHECK(ab.built && ab.off_bits == 0 && ab.on_bits == 0, label);
    }

    // --- (2) STATIC frequency-selective channel: ON must not regress OFF. ---
    // Deep echo (strong selectivity) but time-INVARIANT (no drift, no phase
    // walk). The task requires persistent DD not to HURT a static selective
    // channel; assert no regression (ON <= OFF on both codeword and bit errors).
    // In practice it actually HELPS here too: averaging the decision-directed
    // residual across symbols drives down the per-symbol pilot-interpolation
    // noise, so this is comfortably non-regressive rather than merely break-even.
    {
        auto ab = sweep(/*snr*/ 15.f, /*amp*/ 0.90f, /*drift*/ 0.f, /*ps*/ 0.f);
        std::printf("    static selective:      OFF cw=%d bits=%ld | "
                    "ON cw=%d bits=%ld\n",
                    ab.off_cw, ab.off_bits, ab.on_cw, ab.on_bits);
        char label[200];
        std::snprintf(label, sizeof(label),
            "static selective: persistent ON does not regress OFF "
            "(cw %d->%d, bits %ld->%ld)",
            ab.off_cw, ab.on_cw, ab.off_bits, ab.on_bits);
        CHECK(ab.built && ab.on_cw <= ab.off_cw && ab.on_bits <= ab.off_bits,
              label);
    }

    // --- (3) TIME-VARYING frequency-selective channel: ON must beat OFF. ---
    // Deep echo (amp 0.90 → ~-20 dB fades, strong selectivity that linear pilot
    // interpolation reconstructs imperfectly) that SLOWLY walks: its amplitude
    // drifts sinusoidally (±0.05) and its phase advances 0.02 rad/OFDM-symbol,
    // so over the 16-codeword stream the selective shape rotates away from the
    // one-shot preamble estimate. Persistent DD averages the decision-directed
    // residual across symbols and tracks the walk; intra-symbol DD (OFF) throws
    // its single-shot correction away each symbol and re-leans on interpolation.
    // Operating just into the FEC cliff so the difference shows up as
    // codeword + info-bit errors. The cliff moved slightly when the
    // subcarrier allocation became DC-centered (carriers sit at different
    // positions across the echo's frequency response), so the stress point
    // is 14 dB now; at 15 dB both arms decoded everything and the strict
    // ON<OFF gate had nothing to measure.
    {
        auto ab = sweep(/*snr*/ 14.f, /*amp*/ 0.90f, /*drift*/ 0.05f, /*ps*/ 0.02f);
        std::printf("    time-varying selective: OFF cw=%d bits=%ld | "
                    "ON cw=%d bits=%ld\n",
                    ab.off_cw, ab.off_bits, ab.on_cw, ab.on_bits);
        char label[200];
        std::snprintf(label, sizeof(label),
            "time-varying selective: persistent ON beats OFF "
            "(cw %d->%d, bits %ld->%ld)",
            ab.off_cw, ab.on_cw, ab.off_bits, ab.on_bits);
        // Headline: ON strictly fewer codeword errors AND fewer info-bit errors.
        CHECK(ab.built && ab.on_cw < ab.off_cw && ab.on_bits < ab.off_bits,
              label);
    }
}

int main(int argc, char** argv) {
    bool quick = false;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--quick") == 0) quick = true;
    }

    std::printf("=== Groundwave Integration Test ===\n");
    auto start = std::chrono::steady_clock::now();

    auto stage = [](const char* name) {
        // stderr is unbuffered on MSVC so a crash in the next test still
        // shows us where we were.
        std::fprintf(stderr, ">>> entering %s\n", name);
        std::fflush(stderr);
    };

    stage("test_snr_threshold_monotonic"); test_snr_threshold_monotonic();
    stage("test_ldpc_rebuild_all_rates");  test_ldpc_rebuild_all_rates();
    stage("test_software_loopback_modem"); test_software_loopback_modem();
    stage("test_all_modcods");             test_all_modcods();
    stage("test_ldpc_iter_cap_25_vs_50");  test_ldpc_iter_cap_25_vs_50();
    stage("test_decision_directed_chest"); test_decision_directed_chest();
    stage("test_bicm_id");                 test_bicm_id();
    stage("test_sync_hardening_paths");    test_sync_hardening_paths();
    stage("test_cfo_robustness");          test_cfo_robustness();
    stage("test_selective_channel_llr");   test_selective_channel_llr();
    stage("test_papr_integrity");          test_papr_integrity();
    stage("test_acquisition_timing_offset"); test_acquisition_timing_offset();
    stage("test_persistent_dd_tracking");    test_persistent_dd_tracking();
    if (!quick) {
        stage("test_rapid_modcod_changes"); test_rapid_modcod_changes();
        stage("test_tick_latency_budget");  test_tick_latency_budget();
    }
    stage("done");

    auto elapsed_s = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start).count();

    std::printf("\n=== Result: %d passed, %d failed (%.1fs) ===\n",
                g_passed, g_failed, elapsed_s);
    if (!g_failures.empty()) {
        std::printf("\nFailures:\n");
        for (auto& f : g_failures) std::printf("  - %s\n", f.c_str());
    }
    return g_failed == 0 ? 0 : 1;
}
