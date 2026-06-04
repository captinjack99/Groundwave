/**
 * @file sfo_test.cpp
 * @brief Sample-rate-offset (SFO) loop validation (SOTA-2).
 *
 *   1. SFOResampler unit checks: pass-through at step=1, and output-count
 *      behaviour at step=2 / step=0.5.
 *   2. Closed-loop test: inject a KNOWN ppm clock offset into a long OFDM
 *      stream, run the RX symbol extraction with the SRO loop driving the
 *      resampler, and assert the loop (a) converges to the injected offset
 *      and (b) holds lock (decision-directed EVM stays low) where the
 *      uncorrected path drifts off the symbol grid and the constellation
 *      collapses.
 */
#include "types.hpp"
#include "ofdm.hpp"
#include "symbol_mapper.hpp"
#include "sfo_resampler.hpp"

#include <cstdio>
#include <cmath>
#include <random>
#include <vector>
#include <algorithm>

using namespace dsca;

namespace {
int g_fails = 0;
void check(bool ok, const char* msg) {
    std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", msg);
    if (!ok) ++g_fails;
}

struct StreamResult {
    std::vector<float> evm_db;     // decision-directed EVM per demod'd symbol
    std::vector<double> corr_traj; // resampler correction (ppm) per symbol
    std::vector<double> y_traj;    // raw clockPpm per symbol
    double final_corr_ppm = 0.0;   // resampler's correction at end
    double final_clock_ppm = 0.0;  // residual SRO estimate at end
};

// Average EVM (dB) over the last `frac` fraction of symbols.
float lateEvm(const std::vector<float>& v, float frac = 0.25f) {
    if (v.empty()) return 0.f;
    size_t start = static_cast<size_t>(v.size() * (1.f - frac));
    double s = 0; size_t n = 0;
    for (size_t i = start; i < v.size(); ++i) { s += v[i]; ++n; }
    return n ? static_cast<float>(s / n) : 0.f;
}

StreamResult runStream(Modulation mod, uint16_t fft, uint32_t sr,
                       int n_syms, double inject_ppm,
                       bool correct, float kp, float ki,
                       int nudge_every = 1) {
    OFDMParams ofdm;
    ofdm.fft_size = fft;
    ofdm.modulation = mod;
    ofdm.sample_rate = sr;
    ofdm.cyclic_prefix = CyclicPrefix::CP_1_8;

    OFDMModulator tx(ofdm);
    OFDMDemodulator rx(ofdm);
    SROConfig sc; sc.ema_alpha = 0.30f;  // light slope smoothing; PI does the rest
    rx.enableSROTracking(sc);

    const size_t sym_len = ofdm.symbolLength();
    const size_t bpo = tx.bitsPerOFDMSymbol();
    const size_t total_bits = static_cast<size_t>(n_syms) * bpo;

    std::vector<uint8_t> bits((total_bits + 7) / 8);
    std::mt19937 rng(0xBEEF1234u);
    for (auto& b : bits) b = static_cast<uint8_t>(rng() & 0xFF);

    ComplexBuf tx_bb;
    tx.modulateBits(bits.data(), total_bits, tx_bb);
    ComplexBuf stream = tx.generatePreamble();
    stream.insert(stream.end(), tx_bb.begin(), tx_bb.end());

    // Inject the clock offset: resample the whole stream by 1/(1+ε). A
    // positive inject_ppm means the RX clock is faster → more samples → the
    // stream is stretched (step < 1 produces more outputs).
    const double eps = inject_ppm * 1e-6;
    SFOResampler inj;
    inj.setStep(1.0 / (1.0 + eps));
    ComplexBuf off;
    inj.process(stream, off);

    SFOResampler corr;
    SymbolMapper mapper(mod);
    const ComplexBuf& cpts = mapper.constellation();

    StreamResult res;
    ComplexBuf accum;
    const size_t short_total = 10 * (fft / 4);
    const size_t preamble_len = short_total + 2 * sym_len;
    bool pre_done = false;
    int n_demod = 0;

    size_t pos = 0;
    const size_t FEED = sym_len;  // ~one symbol of input per iteration
    while (pos < off.size() && n_demod < n_syms) {
        size_t take = std::min(FEED, off.size() - pos);
        if (correct) {
            corr.process(&off[pos], take, accum);
        } else {
            accum.insert(accum.end(), off.begin() + static_cast<ptrdiff_t>(pos),
                         off.begin() + static_cast<ptrdiff_t>(pos + take));
        }
        pos += take;

        if (!pre_done && accum.size() >= preamble_len) {
            ComplexBuf longs(accum.begin() + static_cast<ptrdiff_t>(short_total),
                             accum.begin() + static_cast<ptrdiff_t>(preamble_len));
            rx.processPreamble(longs);
            pre_done = true;
            accum.erase(accum.begin(),
                        accum.begin() + static_cast<ptrdiff_t>(preamble_len));
        }
        while (pre_done && accum.size() >= sym_len && n_demod < n_syms) {
            ComplexBuf one(accum.begin(), accum.begin() + static_cast<ptrdiff_t>(sym_len));
            accum.erase(accum.begin(), accum.begin() + static_cast<ptrdiff_t>(sym_len));
            ComplexBuf dsyms;
            if (!rx.demodulate(one, dsyms)) continue;
            double e = 0, p = 0;
            for (auto& y : dsyms) {
                uint16_t idx = mapper.demapHard(y);
                ComplexSample ideal = (idx < cpts.size()) ? cpts[idx]
                                                          : ComplexSample(0.f, 0.f);
                e += std::norm(y - ideal);
                p += std::norm(ideal);
            }
            res.evm_db.push_back(p > 1e-12 ? static_cast<float>(10.0 * std::log10(e / p))
                                           : 0.f);
            ++n_demod;
            if (correct && (n_demod % nudge_every) == 0)
                corr.nudge(rx.clockPpm(), kp, ki);
            res.corr_traj.push_back(corr.correctionPpm());
            res.y_traj.push_back(rx.clockPpm());
        }
    }
    res.final_corr_ppm = corr.correctionPpm();
    res.final_clock_ppm = rx.clockPpm();
    return res;
}

} // anonymous

int main() {
    // -------------------------------------------------------------------
    // 1. Resampler unit checks
    // -------------------------------------------------------------------
    std::printf("=== 1. SFOResampler unit checks ===\n");
    {
        // Pass-through (step=1): output[k] ~= input[k+1] (1-sample delay).
        std::vector<ComplexSample> in(64);
        for (size_t i = 0; i < in.size(); ++i)
            in[i] = ComplexSample(std::sin(0.3f * i), std::cos(0.17f * i));
        SFOResampler r;
        ComplexBuf out;
        r.process(in.data(), in.size(), out);
        float maxerr = 0.f;
        for (size_t k = 0; k + 1 < in.size() && k < out.size(); ++k)
            maxerr = std::max(maxerr, std::abs(out[k] - in[k + 1]));
        char m[80];
        std::snprintf(m, sizeof(m), "step=1 pass-through (max err %.2e)", maxerr);
        check(maxerr < 1e-5f, m);
    }
    {
        std::vector<ComplexSample> in(1000, ComplexSample(1.f, 0.f));
        SFOResampler r2; r2.setStep(2.0);
        ComplexBuf o2; r2.process(in.data(), in.size(), o2);
        SFOResampler r3; r3.setStep(0.5);
        ComplexBuf o3; r3.process(in.data(), in.size(), o3);
        char m[96];
        std::snprintf(m, sizeof(m),
                      "step=2 -> ~N/2 (%zu), step=0.5 -> ~2N (%zu)",
                      o2.size(), o3.size());
        check(o2.size() > 480 && o2.size() < 520 &&
              o3.size() > 1960 && o3.size() < 2040, m);
    }

    // -------------------------------------------------------------------
    // 2. Closed-loop: inject a known ppm offset, assert the SFO loop holds
    //    lock where the uncorrected path drifts off the symbol grid.
    //    kp/ki are the production PI loop gains (mirrored in audio_engine).
    // -------------------------------------------------------------------
    std::printf("\n=== 2. SFO loop holds lock (QAM16, FFT256, 600 syms) ===\n");
    const Modulation mod = Modulation::QAM16;
    const uint16_t fft = 256;
    const uint32_t sr = 48000;
    const int n_syms = 600;
    const float KP = -6.0f, KI = -0.5f;

    for (double inj : { 150.0, -120.0 }) {
        StreamResult nc = runStream(mod, fft, sr, n_syms, inj, false, 0.f, 0.f);
        StreamResult wc = runStream(mod, fft, sr, n_syms, inj, true, KP, KI);
        float late_nc = lateEvm(nc.evm_db);
        float late_wc = lateEvm(wc.evm_db);
        std::printf("  inject %+.0f ppm: no-corr EVM=%.1f dB (resid %.0f) | "
                    "corrected EVM=%.1f dB  corr=%.0f  resid=%.1f ppm\n",
                    inj, late_nc, nc.final_clock_ppm,
                    late_wc, wc.final_corr_ppm, wc.final_clock_ppm);

        char m1[140];
        std::snprintf(m1, sizeof(m1),
                      "inject %+.0f: corrected EVM %.1f dB holds lock (< -15 dB)",
                      inj, late_wc);
        check(late_wc < -15.f, m1);

        char m2[140];
        std::snprintf(m2, sizeof(m2),
                      "inject %+.0f: corrected EVM beats uncorrected (%.1f vs %.1f, >8 dB)",
                      inj, late_wc, late_nc);
        check(late_wc < late_nc - 8.f, m2);

        char m3[140];
        std::snprintf(m3, sizeof(m3),
                      "inject %+.0f: loop zeroes residual slope (|%.1f| < 30 ppm)",
                      inj, wc.final_clock_ppm);
        check(std::abs(wc.final_clock_ppm) < 30.0, m3);

        char m4[140];
        std::snprintf(m4, sizeof(m4),
                      "inject %+.0f: correction %.0f ppm tracks the offset",
                      inj, wc.final_corr_ppm);
        check(wc.final_corr_ppm * inj > 0 &&
              std::abs(wc.final_corr_ppm - inj) < 0.4 * std::abs(inj) + 40.0, m4);
    }

    // -------------------------------------------------------------------
    // 3. Per-codeword cadence: production nudges once per codeword (~12
    //    symbols), not per symbol. Confirm the same gains still acquire.
    // -------------------------------------------------------------------
    std::printf("\n=== 3. Per-codeword nudge cadence (every 12 symbols) ===\n");
    {
        StreamResult wc = runStream(mod, fft, sr, 900, 150.0, true, KP, KI, 12);
        float late = lateEvm(wc.evm_db);
        std::printf("  +150 ppm, nudge/12: corrected EVM=%.1f dB  corr=%.0f  resid=%.1f ppm\n",
                    late, wc.final_corr_ppm, wc.final_clock_ppm);
        char m[140];
        std::snprintf(m, sizeof(m),
                      "per-codeword cadence still locks (EVM %.1f dB < -15, resid |%.1f| < 30)",
                      late, wc.final_clock_ppm);
        check(late < -15.f && std::abs(wc.final_clock_ppm) < 30.0, m);
    }

    std::printf("\n%s (%d failure%s)\n",
                g_fails == 0 ? "ALL PASS" : "FAILURES",
                g_fails, g_fails == 1 ? "" : "s");
    return g_fails == 0 ? 0 : 1;
}
