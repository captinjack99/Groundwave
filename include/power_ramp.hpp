/**
 * @file power_ramp.hpp
 * @brief Smooth TX-keying envelope (raised-cosine ramp on/off).
 *
 * Eliminates the spectral splatter that hard TX gating produces. When the
 * key transitions from off → on, a half-period raised cosine ramps the
 * gain from 0 → 1 over `ramp_samples`. On → off does the inverse. The
 * ramp shape is squared-cosine so power (not amplitude) ramps linearly,
 * matching what listeners perceive as a smooth fade.
 */
#pragma once

#include "types.hpp"
#include <cmath>
#include <cstdint>

namespace gw {

class PowerRamp {
public:
    explicit PowerRamp(uint32_t ramp_samples = 256)
        : ramp_samples_(ramp_samples ? ramp_samples : 1) {}

    void setRampSamples(uint32_t n) {
        ramp_samples_ = n ? n : 1;
        if (ramp_pos_ > ramp_samples_) ramp_pos_ = ramp_samples_;
    }

    void setKey(bool on) {
        if (on == key_target_) return;
        key_target_ = on;
        // Don't reset ramp_pos_ — let it continue from current envelope
    }

    /** Gain envelope this sample: returns 0 while keyed off (after ramp-down),
     *  1 while keyed on (after ramp-up), and the cosine taper in between. */
    float step() {
        if (key_target_) {
            if (ramp_pos_ < ramp_samples_) ++ramp_pos_;
        } else {
            if (ramp_pos_ > 0) --ramp_pos_;
        }
        float t = static_cast<float>(ramp_pos_) / static_cast<float>(ramp_samples_);
        // Squared-cosine: amplitude = sin(πt/2), so |env|² rises linearly with t
        float a = std::sin(0.5f * static_cast<float>(M_PI) * t);
        return a;
    }

    /** Apply ramp to a real-valued buffer in-place. */
    void apply(float* buf, size_t n) {
        for (size_t i = 0; i < n; ++i) buf[i] *= step();
    }

    /** Apply ramp to a complex-valued buffer in-place. */
    void apply(ComplexSample* buf, size_t n) {
        for (size_t i = 0; i < n; ++i) {
            float g = step();
            buf[i] = ComplexSample(buf[i].real() * g, buf[i].imag() * g);
        }
    }

    bool isOn() const { return ramp_pos_ == ramp_samples_; }
    bool isOff() const { return ramp_pos_ == 0; }
    bool isInTransition() const { return !isOn() && !isOff(); }
    float currentGain() const {
        float t = static_cast<float>(ramp_pos_) / static_cast<float>(ramp_samples_);
        return std::sin(0.5f * static_cast<float>(M_PI) * t);
    }

    void reset(bool key_state = false) {
        key_target_ = key_state;
        ramp_pos_ = key_state ? ramp_samples_ : 0;
    }

private:
    uint32_t ramp_samples_;
    uint32_t ramp_pos_ = 0;
    bool key_target_   = false;
};

} // namespace gw
