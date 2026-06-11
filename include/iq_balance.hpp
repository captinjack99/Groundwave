/**
 * @file iq_balance.hpp
 * @brief I/Q imbalance estimation and correction.
 *
 * Compensates amplitude mismatch and phase non-orthogonality between the I
 * and Q channels of a quadrature receiver:
 *
 *   y(t) = (1+ε)·cos(ωt + φ/2) + j·(1-ε)·sin(ωt - φ/2)·m(t)
 *
 * is corrected to remove ε (amplitude imbalance) and φ (phase skew). We
 * estimate the second-order statistics:
 *
 *   E[I·Q]   → phase skew
 *   E[I²-Q²] → amplitude skew
 *
 * and apply a 2×2 corrector matrix:
 *
 *   I'  =  α · I
 *   Q'  =  β · I + γ · Q
 *
 * with α, β, γ chosen so E[I'Q'] = 0 and E[I'²] = E[Q'²].
 *
 * Adaptation runs continuously with a slow time constant; the corrector
 * stops adapting when accumulated samples are small (no signal).
 */
#pragma once

#include "types.hpp"
#include <cmath>
#include <algorithm>
#include <cstdint>

namespace gw {

struct IQBalanceConfig {
    float ema_alpha    = 1e-4f; ///< Update rate for I·Q and I²-Q² statistics
    float min_power    = 1e-6f; ///< Minimum signal power to update corrector
    bool  adaptation_on = true;
};

class IQBalance {
public:
    explicit IQBalance(const IQBalanceConfig& cfg = IQBalanceConfig())
        : cfg_(cfg) {}

    /** Process one complex sample with adaptive correction. */
    ComplexSample process(ComplexSample x) {
        float i = x.real();
        float q = x.imag();

        // Apply current correction
        float i_out = alpha_ * i;
        float q_out = beta_ * i + gamma_ * q;

        if (cfg_.adaptation_on) {
            float power = i_out * i_out + q_out * q_out;
            if (power > cfg_.min_power) {
                // Update second-order statistics with EMA
                eiq_  = (1.f - cfg_.ema_alpha) * eiq_ +
                        cfg_.ema_alpha * (i_out * q_out);
                ei2q2_ = (1.f - cfg_.ema_alpha) * ei2q2_ +
                         cfg_.ema_alpha * (i_out * i_out - q_out * q_out);
                // Re-derive corrector after enough samples have accumulated
                if (++sample_count_ >= update_interval_) {
                    sample_count_ = 0;
                    recompute();
                }
            }
        }
        return ComplexSample(i_out, q_out);
    }

    /** Process a buffer in-place. */
    void process(ComplexSample* buf, size_t n) {
        for (size_t i = 0; i < n; ++i) buf[i] = process(buf[i]);
    }

    void reset() {
        alpha_ = 1.f; beta_ = 0.f; gamma_ = 1.f;
        eiq_ = 0.f; ei2q2_ = 0.f;
        sample_count_ = 0;
    }

    void freezeAdaptation(bool freeze) { cfg_.adaptation_on = !freeze; }

    /** Diagnostic: current measured imbalance estimates. */
    float phaseImbalanceRad()    const { return phase_skew_rad_; }
    float amplitudeImbalanceDb() const { return amp_skew_db_; }

private:
    void recompute() {
        // Phase skew (radians): φ ≈ E[IQ] / sqrt(E[I²]·E[Q²])
        // For small skew this approximates sin(φ).
        // Amplitude skew: ε ≈ (E[I²] - E[Q²]) / (E[I²] + E[Q²])
        // We use a rough decoupled update: a small fraction toward zero per call.
        float sin_phi = eiq_ * 2.f;     // 2·E[IQ]
        sin_phi = std::max(-0.5f, std::min(0.5f, sin_phi));
        float epsilon = ei2q2_ * 0.5f;
        epsilon = std::max(-0.5f, std::min(0.5f, epsilon));

        phase_skew_rad_ = std::asin(sin_phi);
        amp_skew_db_ = 20.f * std::log10((1.f + epsilon) / std::max(1e-9f, 1.f - epsilon));

        // Move corrector toward removing this skew (small step for stability).
        const float step = 0.05f;
        // Off-diagonal β cancels the I→Q correlation (phase skew): with
        // Q' = β·I + γ·Q, driving β down by the measured correlation
        // pushes E[I'Q'] → 0. The previous code adjusted γ instead, which
        // only rescales Q amplitude and can NEVER decorrelate I and Q —
        // so pure phase skew was uncorrectable and β stayed 0. (#18)
        beta_  -= step * sin_phi;
        // Amplitude balance: shrink the hotter rail, grow the colder one.
        alpha_ -= step * epsilon * 0.5f;
        gamma_ += step * epsilon * 0.5f;
        // Clamp all three for stability.
        if (beta_  < -0.5f) beta_  = -0.5f;
        if (beta_  >  0.5f) beta_  =  0.5f;
        if (alpha_ < 0.5f) alpha_ = 0.5f;
        if (alpha_ > 2.0f) alpha_ = 2.0f;
        if (gamma_ < 0.5f) gamma_ = 0.5f;
        if (gamma_ > 2.0f) gamma_ = 2.0f;
    }

    IQBalanceConfig cfg_;
    // Corrector matrix [[alpha, 0], [beta, gamma]]
    float alpha_ = 1.f;
    float beta_  = 0.f;
    float gamma_ = 1.f;
    // Statistics
    float eiq_   = 0.f;
    float ei2q2_ = 0.f;
    // Recompute pacing
    uint32_t sample_count_   = 0;
    uint32_t update_interval_ = 4096;
    // Diagnostics
    float phase_skew_rad_ = 0.f;
    float amp_skew_db_    = 0.f;
};

} // namespace gw
