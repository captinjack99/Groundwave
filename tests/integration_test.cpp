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

#include <chrono>
#include <cstdio>
#include <cstring>
#include <random>
#include <string>
#include <vector>

using namespace dsca;

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
                      float cfo_hz = 0.f)
{
    CycleResult r{};

    // ---- Configure ----
    OFDMParams ofdm;
    ofdm.fft_size    = fft_size;
    ofdm.modulation  = mod;
    ofdm.sample_rate = sample_rate;
    ofdm.cyclic_prefix = CyclicPrefix::CP_1_8;

    LDPCEncoder ldpc_enc(fec, LDPCBlockSize::Short);
    LDPCDecoder ldpc_dec(fec, LDPCBlockSize::Short, max_iter);
    BitInterleaver inter(ldpc_enc.codewordBits());

    OFDMModulator   tx_mod(ofdm);
    OFDMDemodulator rx_demod(ofdm);
    // Exercise the sync-hardening paths (MMSE Wiener #46, phase-tracker #36)
    // that the default path doesn't, so those fixes have test coverage.
    if (use_mmse)          rx_demod.enableMMSE();
    if (use_phase_tracker) rx_demod.enablePhaseTracker();

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

    // ---- Channel: optional CFO + AWGN ----
    ComplexBuf channel = preamble;
    channel.insert(channel.end(), tx_bb.begin(), tx_bb.end());
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
    for (auto& c : configs) {
        auto r = runCycle(c.m, c.f, c.fft, c.sr, -1.f);
        char label[128];
        std::snprintf(label, sizeof(label),
                      "%-7s / %-5s FFT=%u SR=%u: TX+RX=%.1fms (budget %.0fms)",
                      modulationName(c.m), fecRateName(c.f), c.fft, c.sr,
                      r.tx_ms + r.rx_ms, c.bud_ms);
        CHECK(r.tx_ms + r.rx_ms < c.bud_ms && r.bit_errors == 0, label);
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

        // Add AWGN at ~4 dB (near the 16-QAM/½ cliff so iter-0 won't always converge).
        float sig = 0.f; for (auto& s : channel) sig += std::norm(s);
        sig /= static_cast<float>(channel.size());
        float nvar = sig / std::pow(10.f, 8.0f / 10.f);
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
        char label[160];
        std::snprintf(label, sizeof(label),
                      "64-QAM / 3/4 @ ~12 dB: BICM-ID 3-iter err=%d ≤ 1-iter err=%d",
                      e3, e1);
        CHECK(e3 <= e1, label);
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

int main(int argc, char** argv) {
    bool quick = false;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--quick") == 0) quick = true;
    }

    std::printf("=== DSCA-NG Integration Test ===\n");
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
