/**
 * @file passband_loopback_test.cpp
 * @brief Real-passband (IF) loopback coverage — the path complex_loopback
 *        bypasses, and the closest software model of over-the-air operation.
 *
 * Guards two contracts that the complex-baseband tests can never see:
 *
 *   1. ALLOCATION BAND CONTRACT — computeAllocation must place every active
 *      subcarrier (data, pilot, reserved) inside a DC-centered band of
 *      occupiedBandwidthHz(), honoring target_bw_hz, with the guards at the
 *      PHYSICAL band edges (±Nyquist side) and bin 0 empty when dc_null.
 *      The original allocation used the logical carrier range as physical
 *      bins directly, which inverted the layout (guard hole at DC, carriers
 *      out to ±Nyquist): bin-exact in complex loopback, destroyed by the
 *      up/downconverter LPFs on every passband path — 0% decode in
 *      `gw_modem --mode internal`, vcable, and HW mode.
 *
 *   2. PASSBAND ROUND-TRIP — preamble + LDPC codewords pushed through the
 *      full real-IF chain (upconvert + TX LPF -> ring -> AGC -> LPF
 *      downconvert), acquired with the synchronizer's wide fineSync scan
 *      (the stream is NOT front-aligned: two 129-tap FIRs of group delay),
 *      must decode bit-exact on a clean channel and at 20 dB AWGN — and the
 *      demodulator's SNR estimate must be sane, not the ~0 dB a corrupted
 *      channel estimate produces.
 *
 * Run with:  ./passband_loopback_test
 * Exits non-zero if any assertion fails.
 */

#include "types.hpp"
#include "ofdm.hpp"
#include "ldpc.hpp"
#include "interleaver.hpp"
#include "soundcard_modem.hpp"

#include <cstdio>
#include <cstring>
#include <random>
#include <vector>
#include <algorithm>

using namespace gw;

namespace {

int g_passed = 0;
int g_failed = 0;

#define CHECK(cond, msg)                                                       \
    do {                                                                       \
        if (cond) { ++g_passed; std::printf("  [PASS] %s\n", msg); }            \
        else      { ++g_failed; std::printf("  [FAIL] %s\n", msg); }            \
    } while (0)

// Physical frequency (Hz) of an FFT bin in natural order (DC = bin 0).
double binFreqHz(size_t bin, const OFDMParams& p) {
    const double df = p.subcarrierSpacing();
    if (bin < p.fft_size / 2) return static_cast<double>(bin) * df;
    return (static_cast<double>(bin) - static_cast<double>(p.fft_size)) * df;
}

// ---------------------------------------------------------------------------
// Test 1: allocation band contract
// ---------------------------------------------------------------------------
void testAllocationBandContract() {
    std::printf("\n=== Test 1: allocation band contract ===\n");

    struct Case { uint16_t fft; float target_bw; const char* name; };
    const Case cases[] = {
        { 256,  19200.f, "FFT256 target 19.2 kHz" },
        { 256,  0.f,     "FFT256 auto guards"     },
        { 64,   12000.f, "FFT64 target 12 kHz"    },
        { 1024, 9600.f,  "FFT1024 target 9.6 kHz" },
    };

    for (const auto& c : cases) {
        OFDMParams p;
        p.fft_size      = c.fft;
        p.sample_rate   = 48000;
        p.target_bw_hz  = c.target_bw;
        p.papr_reserve_fraction = 0.1f;  // include reserved tones in the check

        SubcarrierAllocation a = computeAllocation(p);
        // The contract the LPFs are designed against: every active bin
        // inside ±target_bw/2 when a target is set. With auto guards the
        // band is the occupied bandwidth — plus one subcarrier of slack
        // for the inherent half-bin asymmetry of an even-FFT DC-centered
        // layout (most-negative carrier sits at -(N/2-g)·Δf, the positive
        // edge at +(N/2-g-1)·Δf).
        const double bound = (c.target_bw > 0.f
                                  ? static_cast<double>(c.target_bw) * 0.5
                                  : p.occupiedBandwidthHz() * 0.5 +
                                        p.subcarrierSpacing()) + 1e-6;

        bool in_band = true, dc_clear = true, sorted = true;
        auto checkSet = [&](const std::vector<size_t>& v) {
            for (size_t i = 0; i < v.size(); ++i) {
                if (std::fabs(binFreqHz(v[i], p)) > bound) in_band = false;
                if (p.dc_null && v[i] == 0) dc_clear = false;
                if (i > 0 && v[i] <= v[i - 1]) sorted = false;
            }
        };
        checkSet(a.data_indices);
        checkSet(a.pilot_indices);
        checkSet(a.reserved_indices);

        char msg[128];
        std::snprintf(msg, sizeof(msg),
                      "%s: all active bins inside +/-%.0f Hz", c.name, bound);
        CHECK(in_band, msg);
        std::snprintf(msg, sizeof(msg), "%s: DC bin empty (dc_null)", c.name);
        CHECK(dc_clear, msg);
        std::snprintf(msg, sizeof(msg),
                      "%s: data/pilot indices sorted ascending", c.name);
        CHECK(sorted, msg);

        if (c.target_bw > 0.f) {
            std::snprintf(msg, sizeof(msg),
                          "%s: occupied %.0f Hz <= target %.0f Hz", c.name,
                          p.occupiedBandwidthHz(),
                          static_cast<double>(c.target_bw));
            CHECK(p.occupiedBandwidthHz() <= c.target_bw + 1e-3, msg);
        }
        // Pilot list pairs with its values 1:1.
        std::snprintf(msg, sizeof(msg), "%s: pilot index/value pairing", c.name);
        CHECK(a.pilot_indices.size() == a.pilot_values.size(), msg);
    }
}

// ---------------------------------------------------------------------------
// Passband TX->RX driver (mirrors cli/modem_cli.cpp's fixed internal mode)
// ---------------------------------------------------------------------------
struct PassbandResult {
    size_t codewords_sent    = 0;
    size_t codewords_ok      = 0;   // converged AND bit-exact
    bool   acquired          = false;
    float  snr_estimate_db   = 0.f;
};

PassbandResult runPassband(float awgn_snr_db, uint32_t seed) {
    PassbandResult r;

    OFDMParams ofdm;
    ofdm.fft_size      = 256;
    ofdm.modulation    = Modulation::QPSK;
    ofdm.sample_rate   = 48000;
    ofdm.target_bw_hz  = 19200.f;

    ModemConfig mcfg;
    mcfg.sample_rate = 48000;
    mcfg.center_freq = 12000.f;
    mcfg.signal_bw   = 19200.f;
    mcfg.loopback    = true;          // REAL passband loopback, not complex

    SoundcardModem modem(mcfg, ofdm);
    OFDMModulator   mod(ofdm);
    OFDMDemodulator demod(ofdm);
    OFDMSynchronizer sync(ofdm);

    LDPCEncoder enc(FECRate::Rate_1_2, LDPCBlockSize::Short);
    LDPCDecoder dec(FECRate::Rate_1_2, LDPCBlockSize::Short, 50);
    BitInterleaver inter(enc.codewordBits());

    // ---- TX: preamble + K codewords of random info ----
    constexpr size_t K = 6;
    std::mt19937 rng(seed);
    std::vector<std::vector<uint8_t>> sent(K);

    modem.transmit(mod.generatePreamble());
    for (size_t f = 0; f < K; ++f) {
        sent[f].resize(enc.infoBytes());
        for (auto& b : sent[f]) b = static_cast<uint8_t>(rng() & 0xFF);

        std::vector<uint8_t> cw(enc.codewordBytes(), 0);
        enc.encode(sent[f].data(), cw.data());
        std::vector<uint8_t> ileaved(cw.size(), 0);
        inter.interleave(cw.data(), ileaved.data());

        ComplexBuf bb;
        mod.modulateBits(ileaved.data(), enc.codewordBits(), bb);
        modem.transmit(bb);
    }
    r.codewords_sent = K;

    // Flush: the TX+RX FIR group delay (~2x64 samples at 129 taps) leaves
    // the tail of the final symbol inside the filter delay lines; push it
    // through with one symbol of silence, as a real key-off ramp would.
    ComplexBuf flush(ofdm.symbolLength(), ComplexSample(0.f, 0.f));
    modem.transmit(flush);

    if (awgn_snr_db >= 0.f) modem.processLoopbackAWGN(awgn_snr_db, seed);

    // ---- RX: drain, wide-scan acquire, then per-codeword decode ----
    ComplexBuf rx_accum;
    for (int pull = 0; pull < 64; ++pull) {
        ComplexBuf rx = modem.receive(9600);
        if (rx.empty()) break;
        rx_accum.insert(rx_accum.end(), rx.begin(), rx.end());
    }

    const size_t cp          = ofdm.cpLength();
    const size_t short_total = 10 * (ofdm.fft_size / 4);
    const size_t long_total  = 2 * ofdm.symbolLength();
    const size_t pre_len     = short_total + long_total;

    if (rx_accum.size() < pre_len + ofdm.symbolLength()) return r;

    SyncResult fr;
    if (!sync.fineSync(rx_accum, static_cast<int>(rx_accum.size() / 2), fr,
                       rx_accum.size()) || !fr.valid) {
        return r;
    }
    long ps = static_cast<long>(fr.timing_offset)
            - static_cast<long>(cp) - static_cast<long>(short_total);
    if (ps < 0) ps = 0;
    if (static_cast<size_t>(ps) + pre_len > rx_accum.size())
        ps = static_cast<long>(rx_accum.size() - pre_len);
    const size_t pre_start = static_cast<size_t>(ps);

    ComplexBuf longp(rx_accum.begin() + static_cast<ptrdiff_t>(pre_start + short_total),
                     rx_accum.begin() + static_cast<ptrdiff_t>(pre_start + pre_len));
    if (!demod.processPreamble(longp)) return r;
    r.acquired = true;
    r.snr_estimate_db = demod.snrEstimate();
    rx_accum.erase(rx_accum.begin(),
                   rx_accum.begin() + static_cast<ptrdiff_t>(pre_start + pre_len));

    const size_t bps_per_sym = mod.bitsPerOFDMSymbol();
    if (bps_per_sym == 0) return r;
    const size_t syms_per_cw =
        (enc.codewordBits() + bps_per_sym - 1) / bps_per_sym;
    const size_t samples_per_cw = syms_per_cw * ofdm.symbolLength();

    for (size_t f = 0; f < K && rx_accum.size() >= samples_per_cw; ++f) {
        std::vector<float> all_llrs;
        for (size_t b = 0; b + ofdm.symbolLength() <= samples_per_cw;
             b += ofdm.symbolLength()) {
            ComplexBuf one(rx_accum.begin() + static_cast<ptrdiff_t>(b),
                           rx_accum.begin() + static_cast<ptrdiff_t>(
                               b + ofdm.symbolLength()));
            std::vector<float> llrs;
            demod.demodulateSoft(one, llrs, demod.noiseVariance());
            all_llrs.insert(all_llrs.end(), llrs.begin(), llrs.end());
        }
        all_llrs.resize(enc.codewordBits(), 0.f);
        std::vector<float> deint(all_llrs.size());
        inter.deinterleave(all_llrs.data(), deint.data());

        std::vector<uint8_t> info(enc.infoBytes(), 0);
        auto res = dec.decode(deint.data(), info.data());
        if (res.converged &&
            std::memcmp(info.data(), sent[f].data(), enc.infoBytes()) == 0) {
            ++r.codewords_ok;
        }
        rx_accum.erase(rx_accum.begin(),
                       rx_accum.begin() + static_cast<ptrdiff_t>(samples_per_cw));
    }
    return r;
}

void testPassbandClean() {
    std::printf("\n=== Test 2: passband loopback, clean channel ===\n");
    PassbandResult r = runPassband(-1.f, 0xC0FFEE01u);
    CHECK(r.acquired, "wide-scan fineSync acquires the preamble");
    char msg[96];
    std::snprintf(msg, sizeof(msg),
                  "all %zu codewords decode bit-exact (got %zu)",
                  r.codewords_sent, r.codewords_ok);
    CHECK(r.codewords_ok == r.codewords_sent, msg);
    std::snprintf(msg, sizeof(msg),
                  "clean-channel SNR estimate sane (>15 dB, got %.1f)",
                  static_cast<double>(r.snr_estimate_db));
    CHECK(r.snr_estimate_db > 15.f, msg);
}

void testPassbandAWGN() {
    std::printf("\n=== Test 3: passband loopback, 20 dB AWGN ===\n");
    PassbandResult r = runPassband(20.f, 0xC0FFEE02u);
    CHECK(r.acquired, "acquires at 20 dB SNR");
    char msg[96];
    std::snprintf(msg, sizeof(msg),
                  "all %zu codewords decode bit-exact at 20 dB (got %zu)",
                  r.codewords_sent, r.codewords_ok);
    CHECK(r.codewords_ok == r.codewords_sent, msg);
}

} // namespace

int main() {
    std::printf("=== Passband (real-IF) loopback test ===\n");

    testAllocationBandContract();
    testPassbandClean();
    testPassbandAWGN();

    std::printf("\n=== Results: %d passed, %d failed ===\n", g_passed, g_failed);
    return g_failed == 0 ? 0 : 1;
}
