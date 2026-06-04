/**
 * @file computed_params_test.cpp
 * @brief Value-sanity sweep for ComputedParams (the bitrate / subcarrier
 *        readouts shown in the GUI).
 *
 * Motivated by a real bug: changing the sample rate sent the displayed bitrate
 * "through the roof". Root cause — ComputedParams::compute() counted usable
 * subcarriers with a hand-rolled `fft_size/2 - gl - gr - dc` formula on a
 * size_t, which UNDERFLOWED (wrapped to ~1.8e19) when the sample-rate-dependent
 * auto-guard maxed out, and which also diverged from the modem's real
 * allocation. This test sweeps modulation × FEC × FFT × sample-rate × signal-BW
 * × pilot-spacing × CP × dc-null and asserts, for every combination:
 *   1. the displayed data/pilot counts EXACTLY match the modem's
 *      computeAllocation() (single source of truth), and
 *   2. every derived value is finite and physically sane (no overflow, no
 *      bitrate above a generous ceiling, monotone gross >= fec >= net).
 */
#include "app_state.hpp"
#include "ofdm.hpp"
#include "types.hpp"

#include <cstdio>
#include <cmath>
#include <vector>

using namespace dsca;

namespace {
int g_fail = 0;
long g_cases = 0;

void fail(const char* what, const OFDMParams& o, FECRate fec, float bw, double val) {
    if (g_fail < 25) {
        std::printf("  [FAIL] %s  (fft=%u sr=%u mod=%d fec=%d signal_bw=%.0f pilot=%u "
                    "cp=%d dc=%d) value=%.6g\n",
                    what, o.fft_size, o.sample_rate, (int)o.modulation, (int)fec,
                    bw, o.pilot_spacing, (int)o.cyclic_prefix, o.dc_null ? 1 : 0, val);
    }
    ++g_fail;
}
} // namespace

int main() {
    std::printf("=== ComputedParams value-sanity sweep ===\n");

    const uint16_t ffts[]   = {64, 128, 256, 512, 1024, 2048, 4096, 8192, 16384};
    const uint32_t srs[]    = {8000, 16000, 24000, 44100, 48000, 96000, 192000};
    const Modulation mods[] = {Modulation::BPSK, Modulation::QPSK, Modulation::QAM16,
                               Modulation::QAM64, Modulation::QAM256,
                               Modulation::QAM1024, Modulation::QAM4096};
    const FECRate fecs[]    = {FECRate::Rate_1_2, FECRate::Rate_2_3,
                               FECRate::Rate_3_4, FECRate::Rate_5_6,
                               FECRate::Rate_9_10};
    const uint8_t pilots[]  = {4, 8, 16};
    const CyclicPrefix cps[]= {CyclicPrefix::CP_1_4, CyclicPrefix::CP_1_8,
                               CyclicPrefix::CP_1_16, CyclicPrefix::CP_1_32};

    // A generous physical ceiling: even FFT=16384 / QAM4096 / 192 kHz is well
    // under ~50 Mbps. The underflow bug produced ~1e16+. 1 Gbps is a safe gate.
    constexpr double BITRATE_CEILING = 1.0e9;

    for (uint16_t fft : ffts)
    for (uint32_t sr : srs)
    for (Modulation mod : mods)
    for (uint8_t psp : pilots)
    for (CyclicPrefix cp : cps)
    for (int dc = 0; dc < 2; ++dc) {
        // signal_bw sweep: 0 = auto, plus fractions of the sample rate that
        // exercise the bandwidth-constrained guard (the bug trigger), incl.
        // values right up to Nyquist.
        std::vector<float> bws = {0.f,
                                  5000.f,
                                  0.20f * sr, 0.40f * sr, 0.49f * sr,
                                  (float)sr};   // intentionally past Nyquist
        for (FECRate fec : fecs)
        for (float bw : bws) {
            OFDMParams o;
            o.fft_size      = fft;
            o.sample_rate   = sr;
            o.modulation    = mod;
            o.pilot_spacing = psp;
            o.cyclic_prefix = cp;
            o.dc_null       = (dc != 0);
            o.target_bw_hz  = bw;     // == AppState::computedParams() mirror of signal_bw

            FrameParams f;
            f.fec_rate = fec;

            ComputedParams c = ComputedParams::compute(o, f, /*rs_enabled=*/true);
            SubcarrierAllocation a = computeAllocation(o);
            ++g_cases;

            // (1) Display MUST match the modem's real allocation.
            if (c.data_subcarriers != a.dataCount())
                fail("data_subcarriers != computeAllocation.dataCount", o, fec, bw,
                     (double)c.data_subcarriers);
            if (c.pilot_subcarriers != a.pilotCount())
                fail("pilot_subcarriers != computeAllocation.pilotCount", o, fec, bw,
                     (double)c.pilot_subcarriers);

            // (2) Physical sanity.
            if (c.active_subcarriers > fft)
                fail("active_subcarriers > fft_size", o, fec, bw, (double)c.active_subcarriers);
            if (c.data_subcarriers > fft)
                fail("data_subcarriers > fft_size", o, fec, bw, (double)c.data_subcarriers);
            if (!std::isfinite(c.gross_bitrate_bps) || c.gross_bitrate_bps < 0.f ||
                c.gross_bitrate_bps > BITRATE_CEILING)
                fail("gross_bitrate out of range", o, fec, bw, c.gross_bitrate_bps);
            if (!std::isfinite(c.net_bitrate_bps) || c.net_bitrate_bps < 0.f)
                fail("net_bitrate not finite/>=0", o, fec, bw, c.net_bitrate_bps);
            if (c.fec_coded_bitrate_bps > c.gross_bitrate_bps * 1.0001f + 1.f)
                fail("fec_coded > gross", o, fec, bw, c.fec_coded_bitrate_bps);
            if (c.net_bitrate_bps > c.fec_coded_bitrate_bps * 1.0001f + 1.f)
                fail("net > fec_coded", o, fec, bw, c.net_bitrate_bps);
            if (!std::isfinite(c.spectral_eff_bps_hz) || c.spectral_eff_bps_hz < 0.f ||
                c.spectral_eff_bps_hz > 30.f)
                fail("spectral_eff out of range", o, fec, bw, c.spectral_eff_bps_hz);
        }
    }

    std::printf("Swept %ld configurations.\n", g_cases);
    std::printf("\n%s (%d failure%s)\n",
                g_fail == 0 ? "ALL PASS" : "FAILURES",
                g_fail, g_fail == 1 ? "" : "s");
    return g_fail == 0 ? 0 : 1;
}
