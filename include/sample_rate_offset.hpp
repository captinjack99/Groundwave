/**
 * @file sample_rate_offset.hpp
 * @brief Sample rate offset (SRO) estimator from pilot phase slope.
 *
 * A clock mismatch between TX and RX causes a phase that grows linearly
 * across subcarriers within each OFDM symbol. By tracking the slope of
 * pilot phases versus subcarrier index, we get an unbiased estimate of
 * the timing offset rate (parts-per-million).
 *
 * ε = (1 / 2π · N_OFDM) · slope[ phase(H[k]) vs k ]
 *
 * where N_OFDM is the symbol length in samples. The estimate drives the
 * `SFOResampler` (include/sfo_resampler.hpp) in the RX path. NOTE: the
 * pilot slope measures the window timing error δ (a *position*, which ramps
 * under a clock offset), while the resampler controls the *rate* — so the
 * closing loop is a PROPORTIONAL-INTEGRAL timing-recovery loop, not the
 * "slow integrator" an earlier revision of this comment implied (a pure
 * integrator on a position-vs-rate plant rings/diverges).
 */
#pragma once

#include "types.hpp"
#include <vector>
#include <algorithm>
#include <cmath>

namespace gw {

struct SROConfig {
    float ema_alpha = 0.05f;  ///< EMA on slope estimate
    bool  enabled   = true;
};

class SROEstimator {
public:
    explicit SROEstimator(const SROConfig& cfg = SROConfig()) : cfg_(cfg) {}

    /** Estimate the slope (radians per subcarrier) of pilot phases.
     *  @param pilot_indices  Subcarrier indices of pilots.
     *  @param pilot_phases   arg(H_LS[k]) at each pilot. */
    float estimateSlope(const std::vector<size_t>& pilot_indices,
                        const std::vector<float>& pilot_phases) {
        size_t n = std::min(pilot_indices.size(), pilot_phases.size());
        if (n < 2) return slope_ema_;
        // Unwrap phases first to remove ±2π jumps
        std::vector<float> phi(pilot_phases.begin(),
                               pilot_phases.begin() + static_cast<ptrdiff_t>(n));
        for (size_t i = 1; i < n; ++i) {
            float d = phi[i] - phi[i - 1];
            while (d >  static_cast<float>(M_PI))  { phi[i] -= 2.f * static_cast<float>(M_PI); d -= 2.f * static_cast<float>(M_PI); }
            while (d < -static_cast<float>(M_PI))  { phi[i] += 2.f * static_cast<float>(M_PI); d += 2.f * static_cast<float>(M_PI); }
        }
        // Linear regression: slope = Σ(x-x̄)(y-ȳ) / Σ(x-x̄)²
        double xs = 0.0, ys = 0.0;
        for (size_t i = 0; i < n; ++i) {
            xs += static_cast<double>(pilot_indices[i]);
            ys += static_cast<double>(phi[i]);
        }
        double xm = xs / static_cast<double>(n);
        double ym = ys / static_cast<double>(n);
        double num = 0.0, den = 0.0;
        for (size_t i = 0; i < n; ++i) {
            double dx = static_cast<double>(pilot_indices[i]) - xm;
            num += dx * (static_cast<double>(phi[i]) - ym);
            den += dx * dx;
        }
        if (den < 1e-12) return slope_ema_;
        float instant = static_cast<float>(num / den);
        slope_ema_ = (1.f - cfg_.ema_alpha) * slope_ema_ +
                     cfg_.ema_alpha * instant;
        return slope_ema_;
    }

    /** Convert slope (rad/subcarrier) to PPM clock offset. */
    float slopePpm(size_t fft_size) const {
        if (fft_size == 0) return 0.f;
        // ε = slope / (2π · N) gives fractional clock offset
        float frac = slope_ema_ /
                     (2.f * static_cast<float>(M_PI) * static_cast<float>(fft_size));
        return frac * 1.0e6f;
    }

    float slope() const { return slope_ema_; }

    void reset() { slope_ema_ = 0.f; }

private:
    SROConfig cfg_;
    float slope_ema_ = 0.f;
};

} // namespace gw
