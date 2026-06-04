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
#include "snr_calculator.hpp"
#include "hierarchical_mod.hpp"

#include <cstdio>
#include <cmath>
#include <vector>

using namespace dsca;

namespace {
int g_fail = 0;
long g_cases = 0;

// Lightweight check for the threshold sweeps (no OFDMParams context).
void chk(bool ok, const char* what, double v) {
    if (!ok) {
        if (g_fail < 25) std::printf("  [FAIL] %s  value=%.6g\n", what, v);
        ++g_fail;
    }
}

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

    std::printf("Swept %ld ComputedParams configurations.\n", g_cases);

    // ---- Link-budget core: computeThreshold(mod, fec) ----
    const FECRate all_fecs[] = {
        FECRate::Rate_1_4, FECRate::Rate_1_3, FECRate::Rate_2_5, FECRate::Rate_1_2,
        FECRate::Rate_3_5, FECRate::Rate_2_3, FECRate::Rate_3_4, FECRate::Rate_4_5,
        FECRate::Rate_5_6, FECRate::Rate_8_9, FECRate::Rate_9_10};
    long thr_cases = 0;
    for (Modulation mod : mods)
    for (FECRate fec : all_fecs) {
        ModCodThreshold t = computeThreshold(mod, fec);
        ++thr_cases;
        float se_expect = bitsPerSymbol(mod) * codeRateValue(fec);
        chk(std::isfinite(t.spectral_eff) && std::fabs(t.spectral_eff - se_expect) < 1e-3f,
            "threshold.spectral_eff", t.spectral_eff);
        chk(std::isfinite(t.shannon_limit_db) && t.shannon_limit_db > -20.f &&
            t.shannon_limit_db < 60.f, "threshold.shannon_limit_db", t.shannon_limit_db);
        chk(std::isfinite(t.impl_loss_db) && t.impl_loss_db >= 0.f && t.impl_loss_db < 15.f,
            "threshold.impl_loss_db", t.impl_loss_db);
        chk(std::isfinite(t.threshold_db) && t.threshold_db > -20.f && t.threshold_db < 70.f,
            "threshold.threshold_db", t.threshold_db);
        // Eb/N0 derivation as the link-budget panel computes it.
        float se = std::max(t.spectral_eff, 1e-6f);
        float eb_n0 = t.threshold_db - 10.f * std::log10(se);
        chk(std::isfinite(eb_n0) && eb_n0 > -30.f && eb_n0 < 70.f, "eb_n0", eb_n0);
    }
    std::printf("Swept %ld computeThreshold configurations.\n", thr_cases);

    // ---- Hierarchical HP/LP thresholds: computeHierThreshold(...) ----
    // alpha sweep INCLUDING out-of-UI-range values, because a loaded/hand-edited
    // config is not range-checked: alpha <= 0 would feed 20*log10(alpha) and
    // produce NaN/-inf in the displayed HP/LP thresholds.
    const float alphas[] = {-1.0f, 0.0f, 0.25f, 0.5f, 1.0f, 2.0f, 4.0f, 8.0f, 100.0f};
    // Custom HP/LP splits across the supported constellation orders.
    const int hp_lp[][2] = {{1,1},{2,2},{2,4},{4,2},{4,4},{2,6},{6,2},{4,6},
                            {2,8},{8,2},{6,6}};
    long hier_cases = 0;
    for (auto& sp : hp_lp)
    for (FECRate hp_fec : all_fecs)
    for (FECRate lp_fec : all_fecs)
    for (float a : alphas) {
        HierarchicalConfig hc;
        hc.enabled = true;
        hc.mode    = HierarchicalMode::Custom;
        hc.hp_bits = static_cast<uint8_t>(sp[0]);
        hc.lp_bits = static_cast<uint8_t>(sp[1]);
        hc.alpha   = a;
        HierThreshold h = computeHierThreshold(hc, hp_fec, lp_fec);
        ++hier_cases;
        chk(std::isfinite(h.hp_threshold_db), "hier.hp_threshold_db", h.hp_threshold_db);
        chk(std::isfinite(h.lp_threshold_db), "hier.lp_threshold_db", h.lp_threshold_db);
        chk(std::isfinite(h.hp_spectral_eff) && h.hp_spectral_eff >= 0.f,
            "hier.hp_spectral_eff", h.hp_spectral_eff);
        chk(std::isfinite(h.lp_spectral_eff) && h.lp_spectral_eff >= 0.f,
            "hier.lp_spectral_eff", h.lp_spectral_eff);
        chk(std::isfinite(h.coverage_gain_db), "hier.coverage_gain_db", h.coverage_gain_db);
    }
    std::printf("Swept %ld computeHierThreshold configurations.\n", hier_cases);

    std::printf("\n%s (%d failure%s)\n",
                g_fail == 0 ? "ALL PASS" : "FAILURES",
                g_fail, g_fail == 1 ? "" : "s");
    return g_fail == 0 ? 0 : 1;
}
