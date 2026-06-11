/**
 * @file integer_cfo.hpp
 * @brief Integer carrier frequency offset estimator.
 *
 * Schmidl–Cox + CP correlation only resolve fractional CFO (|Δf| < Δf_sub).
 * Larger offsets show up as bin shifts in the FFT of the long preamble.
 *
 * Algorithm:
 *   1. FFT the received long preamble symbol → Y[k]
 *   2. For each candidate shift d ∈ [-d_max, +d_max]:
 *        metric(d) = |Σ_k Y[k+d] · conj(P[k])|² / (Σ |Y[k+d]|²)
 *      where P[k] is the known reference preamble (frequency domain).
 *   3. Pick the d that maximizes metric. CFO = d · Δf_sub.
 *
 * Combine with fractional CFO (from Schmidl–Cox phase) for full estimate.
 */
#pragma once

#include "types.hpp"
#include "fft_engine.hpp"
#include <vector>
#include <cmath>
#include <cstddef>

namespace gw {

class IntegerCFOEstimator {
public:
    /** @param fft_size      OFDM FFT size.
     *  @param ref_freq      Known frequency-domain reference preamble (size N).
     *  @param max_bin_shift Search range in subcarriers (±). */
    IntegerCFOEstimator(size_t fft_size,
                        const ComplexBuf& ref_freq,
                        size_t max_bin_shift = 16)
        : fft_size_(fft_size)
        , max_shift_(max_bin_shift)
        , ref_(ref_freq)
        , fft_(std::make_unique<FFTEngine>(fft_size))
    {
        ref_.resize(fft_size, ComplexSample(0.f, 0.f));
        Y_.resize(fft_size);
    }

    /** @param time_domain  Long preamble symbol in time domain (size = fft_size).
     *  @return  Estimated integer offset in bins (positive = received signal
     *           is shifted UP in frequency by this many subcarriers). */
    int estimate(const ComplexBuf& time_domain) {
        if (time_domain.size() < fft_size_) return 0;
        fft_->forward(time_domain, Y_);

        int best_d = 0;
        float best_metric = -1.f;

        const int dmax = static_cast<int>(max_shift_);
        for (int d = -dmax; d <= dmax; ++d) {
            ComplexSample acc(0.f, 0.f);
            float energy = 0.f;
            for (size_t k = 0; k < fft_size_; ++k) {
                int kd = static_cast<int>(k) + d;
                if (kd < 0 || kd >= static_cast<int>(fft_size_)) continue;
                if (std::abs(ref_[k]) < 1e-9f) continue;
                acc += Y_[static_cast<size_t>(kd)] * std::conj(ref_[k]);
                energy += std::norm(Y_[static_cast<size_t>(kd)]);
            }
            float m = std::norm(acc) / std::max(energy, 1e-12f);
            if (m > best_metric) {
                best_metric = m;
                best_d = d;
            }
        }
        last_metric_ = best_metric;
        return best_d;
    }

    /** Convenience: integer CFO in Hz. */
    float estimateHz(const ComplexBuf& time_domain, float subcarrier_spacing_hz) {
        return static_cast<float>(estimate(time_domain)) * subcarrier_spacing_hz;
    }

    float lastMetric() const { return last_metric_; }

private:
    size_t fft_size_;
    size_t max_shift_;
    ComplexBuf ref_;
    std::unique_ptr<FFTEngine> fft_;
    ComplexBuf Y_;
    float last_metric_ = 0.f;
};

} // namespace gw
