/**
 * @file dc_blocker.hpp
 * @brief One-pole IIR DC blocking filter.
 *
 * Removes the DC component from a real or complex signal. The filter is:
 *
 *   y[n] = x[n] - x[n-1] + R * y[n-1]
 *
 * with pole radius R close to (but below) 1. Cutoff frequency is
 * approximately f_c ≈ (1 - R) * fs / (2π) for small (1 - R).
 *
 * Defaults give a ~1 Hz cutoff at 48 kHz (R ≈ 0.99987).
 */
#pragma once

#include "types.hpp"
#include <cstddef>

namespace gw {

class DCBlocker {
public:
    /** @param pole_radius  Pole radius R in (0, 1). Closer to 1 = lower cutoff,
     *                      slower transient. 0.999 ≈ 7.6 Hz @ 48 kHz. */
    explicit DCBlocker(float pole_radius = 0.999f) : R_(pole_radius) {}

    /** Set cutoff in Hz given sample rate. */
    void setCutoff(float cutoff_hz, float sample_rate) {
        if (sample_rate <= 0.f) return;
        // Matched-pole mapping R = exp(-2π f_c / fs). The previous
        // first-order approximation (R = 1 - 2π f_c/fs) drifts at high
        // fc/fs (e.g. 200 Hz @ 8 kHz: 0.843 vs matched 0.855), placing the
        // realized -3 dB point off target; exp() is accurate across the
        // whole range and reduces to 1 - w for small w. (#66)
        float w = 2.f * static_cast<float>(M_PI) * cutoff_hz / sample_rate;
        R_ = std::exp(-w);
        if (R_ < 0.f) R_ = 0.f;
        if (R_ > 0.9999f) R_ = 0.9999f;
    }

    /** Process one real sample. */
    float process(float x) {
        float y = x - x_prev_ + R_ * y_prev_;
        x_prev_ = x;
        y_prev_ = y;
        return y;
    }

    /** Process a buffer of real samples in-place. */
    void process(float* buf, size_t n) {
        for (size_t i = 0; i < n; ++i) buf[i] = process(buf[i]);
    }

    /** Process one complex sample (independent I/Q DC removal). */
    ComplexSample processComplex(ComplexSample x) {
        ComplexSample y(x.real() - xc_prev_.real() + R_ * yc_prev_.real(),
                        x.imag() - xc_prev_.imag() + R_ * yc_prev_.imag());
        xc_prev_ = x;
        yc_prev_ = y;
        return y;
    }

    /** Process a buffer of complex samples in-place. */
    void processComplex(ComplexSample* buf, size_t n) {
        for (size_t i = 0; i < n; ++i) buf[i] = processComplex(buf[i]);
    }

    void reset() {
        x_prev_ = 0.f;
        y_prev_ = 0.f;
        xc_prev_ = ComplexSample(0.f, 0.f);
        yc_prev_ = ComplexSample(0.f, 0.f);
    }

    float poleRadius() const { return R_; }

private:
    float R_;
    float x_prev_ = 0.f;
    float y_prev_ = 0.f;
    ComplexSample xc_prev_ {0.f, 0.f};
    ComplexSample yc_prev_ {0.f, 0.f};
};

} // namespace gw
