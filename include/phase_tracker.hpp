/**
 * @file phase_tracker.hpp
 * @brief Per-symbol residual phase tracking via pilot averaging.
 *
 * After channel equalization there is still a slowly-varying common phase
 * offset across symbols (uncompensated CFO, oscillator drift). We track it
 * by averaging the phase of equalized pilot symbols against their known
 * reference, run that through a 2nd-order PLL, and rotate equalized data
 * symbols by the negative of that phase before slicing.
 */
#pragma once

#include "types.hpp"
#include <vector>
#include <cmath>

namespace dsca {

struct PhaseTrackerConfig {
    float loop_bw_hz   = 50.0f;   ///< Loop bandwidth in Hz
    float damping      = 0.707f;  ///< Damping factor (0.707 = critical)
    float symbol_rate  = 1000.0f; ///< OFDM symbol rate (used for loop gain)
    bool  enabled      = true;
    float min_pilot_mag = 0.1f;   ///< Min avg |Σ eq·conj(ref)| / N to trust the
                                  ///< phase detection; below this the loop coasts
};

class PhaseTracker {
public:
    explicit PhaseTracker(const PhaseTrackerConfig& cfg = PhaseTrackerConfig())
        : cfg_(cfg) { recomputeGains(); }

    void setConfig(const PhaseTrackerConfig& cfg) {
        cfg_ = cfg;
        recomputeGains();
    }

    /** Compute residual phase from one OFDM symbol's equalized pilots.
     *  @param eq_pilots   Equalized pilot symbols (already divided by H_est).
     *  @param ref_pilots  Known reference pilot symbols.
     *  @return  Detected phase error (radians) before PLL filtering. */
    float detectPhase(const ComplexBuf& eq_pilots, const ComplexBuf& ref_pilots) {
        size_t n = std::min(eq_pilots.size(), ref_pilots.size());
        if (n == 0) { last_valid_ = false; return 0.f; }
        ComplexSample acc(0.f, 0.f);
        for (size_t i = 0; i < n; ++i) acc += eq_pilots[i] * std::conj(ref_pilots[i]);
        // Gate on the coherent-sum magnitude. During a deep fade / very low
        // SNR, acc → 0 and std::arg(acc) is dominated by noise; feeding that
        // raw angle into the loop winds up the unbounded integrator. Flag
        // it so update() coasts instead. (#45)
        last_valid_ = (std::abs(acc) >= cfg_.min_pilot_mag * static_cast<float>(n));
        return std::arg(acc);
    }

    /** Run one PLL update with the latest detected phase error and return
     *  the current corrected phase estimate (to be applied to data). */
    float update(float phase_err) {
        if (!cfg_.enabled) return phase_;
        if (last_valid_) {
            // 2nd-order loop: integrator + proportional path
            freq_ += Ki_ * phase_err;
            phase_ += freq_ + Kp_ * phase_err;
        } else {
            // Coast: hold the integrator frozen and advance phase by the
            // current frequency only, so a noise-dominated frame can't
            // wind up the loop. (#45)
            phase_ += freq_;
        }
        // Wrap into [-π, π]
        if (phase_ > static_cast<float>(M_PI))
            phase_ -= 2.f * static_cast<float>(M_PI);
        else if (phase_ < -static_cast<float>(M_PI))
            phase_ += 2.f * static_cast<float>(M_PI);
        return phase_;
    }

    /** Apply the current phase estimate to a buffer of complex symbols
     *  (rotates by exp(-j·phase)). */
    void apply(ComplexSample* buf, size_t n) const {
        float c = std::cos(-phase_);
        float s = std::sin(-phase_);
        for (size_t i = 0; i < n; ++i) {
            float r = buf[i].real() * c - buf[i].imag() * s;
            float m = buf[i].real() * s + buf[i].imag() * c;
            buf[i] = ComplexSample(r, m);
        }
    }

    void reset() {
        phase_ = 0.f;
        freq_  = 0.f;
    }

    float phaseRad() const { return phase_; }
    float freqRadPerSym() const { return freq_; }

private:
    void recomputeGains() {
        // Standard 2nd-order PLL closed-loop coefficients
        float wn = 2.f * static_cast<float>(M_PI) * cfg_.loop_bw_hz / cfg_.symbol_rate;
        float zeta = cfg_.damping;
        float t1 = 4.f * zeta * wn / (1.f + 2.f * zeta * wn + wn * wn);
        float t2 = 4.f * wn * wn / (1.f + 2.f * zeta * wn + wn * wn);
        Kp_ = t1;
        Ki_ = t2;
    }

    PhaseTrackerConfig cfg_;
    float phase_ = 0.f;
    float freq_  = 0.f;
    float Kp_    = 0.f;
    float Ki_    = 0.f;
    bool  last_valid_ = true;  ///< was the last detectPhase() above threshold
};

} // namespace dsca
