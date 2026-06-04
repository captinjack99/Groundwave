/**
 * @file sfo_resampler.hpp
 * @brief Arbitrary-ratio fractional resampler + sample-rate-offset (SFO)
 *        control loop for the RX path.
 *
 * Two soundcard clocks are never identical: the RX samples the air at
 * f_rx = (1 + ε)·f_tx for some small ε (tens of ppm). Left uncorrected the
 * fixed-length OFDM symbol extraction slides off the true symbol grid, the
 * FFT window misaligns, ICI grows, and a long transmission periodically
 * loses lock. The `SROEstimator` already measures the residual ε from the
 * pilot phase slope (`OFDMDemodulator::clockPpm()`); this class is the
 * actuator that closes the loop.
 *
 * Mechanism: a streaming cubic (Catmull-Rom) interpolator resamples the
 * incoming baseband by a ratio of `step_` input samples per output sample.
 * `step_ = 1` is pass-through (a fixed group delay). `nudge()` integrates the
 * measured residual ppm into `step_`, so once the residual is driven to zero
 * the corrector holds the exact clock ratio. Because ε is essentially
 * constant over a transmission this is a slow first-order loop: the SRO EMA
 * provides the error, this integrator provides the memory.
 *
 * The interpolation kernel is cubic Catmull-Rom (4-tap). The correction is a
 * tiny, slowly-varying fractional delay, so the interpolation error is
 * negligible and any static passband coloration is absorbed by the channel
 * equalizer.
 */
#pragma once

#include "types.hpp"
#include <vector>

namespace dsca {

class SFOResampler {
public:
    SFOResampler() { reset(); }

    /** Clear all filter + loop state. */
    void reset() {
        h0_ = h1_ = h2_ = h3_ = ComplexSample(0.f, 0.f);
        mu_ = 0.0;
        primed_ = 0;
        step_ = 1.0;
        correction_ppm_ = 0.0;
        integ_ = 0.0;
    }

    /** Resample `in` (n_in samples) into `out` (appended). The number of
     *  outputs is approximately n_in / step_; it varies by ±1 as the
     *  fractional phase carries. State persists across calls (streaming). */
    void process(const ComplexSample* in, size_t n_in, ComplexBuf& out) {
        for (size_t i = 0; i < n_in; ++i) {
            // Slide the 4-sample history window: h0 oldest .. h3 newest.
            h0_ = h1_; h1_ = h2_; h2_ = h3_; h3_ = in[i];
            if (primed_ < 3) { ++primed_; continue; }  // need 4 taps
            // Emit every output whose position falls in the [h1,h2) segment.
            while (mu_ < 1.0) {
                out.push_back(cubic(h0_, h1_, h2_, h3_, static_cast<float>(mu_)));
                mu_ += step_;
            }
            mu_ -= 1.0;  // advance to the next input segment
        }
    }

    void process(const ComplexBuf& in, ComplexBuf& out) {
        process(in.data(), in.size(), out);
    }

    /** Drive the resampling ratio from a residual SFO measurement (the
     *  pilot-slope `clockPpm()`), via a 2nd-order PI timing-recovery loop.
     *
     *  Under a clock offset ε the window timing error δ RAMPS (∝ symbol
     *  index) and the pilot slope `y` measures δ. This is the classic
     *  phase(δ)/frequency(ε) tracking problem: the integral term supplies the
     *  constant rate r→ε that stops the ramp (zeroing the steady-state slope),
     *  and the proportional term damps the 2nd-order loop. `kp`/`ki` carry the
     *  loop sign. Feed the RAW slope (run the SRO with ema_alpha≈1) — the loop
     *  does its own filtering. Call once per demodulated symbol/codeword. */
    void nudge(float y, float kp, float ki) {
        integ_ += static_cast<double>(ki) * static_cast<double>(y);
        if (integ_ >  600.0) integ_ =  600.0;   // anti-windup
        if (integ_ < -600.0) integ_ = -600.0;
        double r = static_cast<double>(kp) * static_cast<double>(y) + integ_;
        if (r >  600.0) r =  600.0;
        if (r < -600.0) r = -600.0;
        correction_ppm_ = r;
        step_ = 1.0 + correction_ppm_ * 1e-6;
    }

    /** Directly set the resampling ratio (input samples per output). Used by
     *  test harnesses to inject a known offset. */
    void setStep(double step) { step_ = step; }

    double step() const { return step_; }
    /** The integrated correction in ppm (the loop's running estimate of ε). */
    double correctionPpm() const { return correction_ppm_; }

private:
    // Catmull-Rom cubic interpolation at fractional position mu in [0,1)
    // between y1 and y2 (using neighbours y0,y3). Real coefficients applied
    // to complex samples.
    static ComplexSample cubic(const ComplexSample& y0, const ComplexSample& y1,
                               const ComplexSample& y2, const ComplexSample& y3,
                               float mu) {
        float mu2 = mu * mu;
        float mu3 = mu2 * mu;
        float c0 = -0.5f * mu3 + mu2 - 0.5f * mu;           // weight y0
        float c1 =  1.5f * mu3 - 2.5f * mu2 + 1.0f;         // weight y1
        float c2 = -1.5f * mu3 + 2.0f * mu2 + 0.5f * mu;    // weight y2
        float c3 =  0.5f * mu3 - 0.5f * mu2;                // weight y3
        return c0 * y0 + c1 * y1 + c2 * y2 + c3 * y3;
    }

    ComplexSample h0_, h1_, h2_, h3_;
    double mu_;             // fractional phase within the current [h1,h2) segment
    double step_;           // input samples consumed per output sample
    double correction_ppm_; // current loop correction (ppm) = resampler rate
    double integ_ = 0.0;    // PI integral state (ppm), ≈ ε at lock
    int    primed_;         // history fill counter (0..3)
};

} // namespace dsca
