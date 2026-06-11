/**
 * @file scorecard.cpp
 * @brief Throughput + link-budget table generator for the DSCA-NG vs HD Radio
 *        comparison. For each modcod it prints the modem's OWN required Es/N0
 *        (computeThreshold, validated to ~1 dB by benchmark_waterfall) and the
 *        net info bitrate delivered when the OFDM signal occupies each scenario
 *        bandwidth (net_bitrate from ComputedParams = the real allocation math,
 *        RS outer code on, auto guard bands, default pilots/CP).
 *
 * Scenario B: ~36 kHz SCA baseband (rides alongside analog FM).
 * Scenario C: ~140 kHz = the RF footprint hybrid IBOC's digital sidebands use.
 *
 * Occupied bandwidth is modeled as the complex-baseband sample rate, so net
 * bitrate already accounts for guard bands, pilots, CP, FEC and RS overhead.
 */
#include "app_state.hpp"
#include "snr_calculator.hpp"
#include "types.hpp"

#include <cstdio>

using namespace dsca;

namespace {
struct MC { Modulation m; FECRate f; const char* name; };

float netKbps(Modulation m, FECRate fec, uint32_t occupied_bw_hz) {
    OFDMParams o;
    o.fft_size      = 1024;
    o.sample_rate   = occupied_bw_hz;   // occupied BW ~= complex baseband rate
    o.modulation    = m;
    o.pilot_spacing = 8;
    o.cyclic_prefix = CyclicPrefix::CP_1_8;
    o.dc_null       = true;
    o.target_bw_hz  = 0.f;              // auto guard (~5%/side)
    FrameParams fp;  fp.fec_rate = fec;
    ComputedParams c = ComputedParams::compute(o, fp, /*rs_enabled=*/true);
    return c.net_bitrate_bps / 1000.f;
}
} // namespace

int main() {
    const MC ladder[] = {
        {Modulation::BPSK,   FECRate::Rate_1_2, "BPSK   1/2"},
        {Modulation::QPSK,   FECRate::Rate_1_2, "QPSK   1/2"},
        {Modulation::QPSK,   FECRate::Rate_3_4, "QPSK   3/4"},
        {Modulation::QAM16,  FECRate::Rate_1_2, "16-QAM 1/2"},
        {Modulation::QAM16,  FECRate::Rate_3_4, "16-QAM 3/4"},
        {Modulation::QAM64,  FECRate::Rate_2_3, "64-QAM 2/3"},
        {Modulation::QAM64,  FECRate::Rate_5_6, "64-QAM 5/6"},
        {Modulation::QAM256, FECRate::Rate_3_4, "256QAM 3/4"},
    };

    std::printf("=== DSCA-NG modcod ladder: required SNR + net throughput ===\n");
    std::printf("(net kbps = ComputedParams net_bitrate, RS on, auto guards, "
                "pilot/8, CP 1/8)\n\n");
    std::printf("  modcod     | reqEs/N0 | bits/Hz | SCA 36 kHz | IBOC-fp 140 kHz\n");
    std::printf("  -----------+----------+---------+------------+----------------\n");
    for (const auto& mc : ladder) {
        ModCodThreshold t = computeThreshold(mc.m, mc.f);
        float sca  = netKbps(mc.m, mc.f, 36000);
        float iboc = netKbps(mc.m, mc.f, 140000);
        float bits_per_hz = iboc * 1000.f / 140000.f;   // net bits/Hz
        std::printf("  %-10s | %6.1f dB | %6.2f  | %7.1f kb | %9.1f kb\n",
                    mc.name, t.threshold_db, bits_per_hz, sca, iboc);
    }
    std::printf("\nReference (published, approx): hybrid IBOC delivers ~96 kbps "
                "(HD1, MP1) up to ~150 kbps (extended modes) in ~140 kHz of\n"
                "flanking digital sidebands -> ~0.7-1.1 net bits/Hz, fixed "
                "QPSK-class robustness, HDC codec, convolutional FEC.\n");
    return 0;
}
