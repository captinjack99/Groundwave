/**
 * @file squelch.hpp
 * @brief Energy-detector squelch with hysteresis.
 *
 * Tracks short-term signal RMS in dBFS via an EMA. Opens when the level
 * exceeds `open_threshold_db` for `attack_samples` consecutive samples,
 * closes when it stays below `close_threshold_db` for `hold_samples`.
 *
 * `open_threshold_db > close_threshold_db` provides Schmitt-trigger style
 * hysteresis to prevent chattering near the threshold.
 */
#pragma once

#include "types.hpp"
#include <cmath>
#include <cstdint>

namespace dsca {

struct SquelchConfig {
    float open_threshold_db   = -50.0f;  ///< dBFS to open the gate
    float close_threshold_db  = -55.0f;  ///< dBFS to close the gate (hysteresis)
    float ema_alpha           = 0.05f;   ///< EMA on signal RMS
    uint32_t attack_samples   = 256;     ///< Consecutive above-threshold to open
    uint32_t hold_samples     = 4800;    ///< Consecutive below-threshold to close (~100ms @ 48k)
    bool enabled              = true;
};

class Squelch {
public:
    explicit Squelch(const SquelchConfig& cfg = SquelchConfig()) : cfg_(cfg) {}

    void setConfig(const SquelchConfig& cfg) { cfg_ = cfg; }
    const SquelchConfig& config() const { return cfg_; }

    /** Process one real sample. Returns true if the gate is open and the
     *  caller should keep this sample. */
    bool processSample(float x) {
        return updateState(x * x);
    }

    bool processSample(ComplexSample x) {
        return updateState(std::norm(x));
    }

    /** Apply squelch to a real-sample buffer in-place — zeroes samples
     *  while the gate is closed. Returns count of samples passed through. */
    size_t apply(float* buf, size_t n) {
        if (!cfg_.enabled) return n;
        size_t pass = 0;
        for (size_t i = 0; i < n; ++i) {
            if (processSample(buf[i])) ++pass;
            else buf[i] = 0.f;
        }
        return pass;
    }

    size_t apply(ComplexSample* buf, size_t n) {
        if (!cfg_.enabled) return n;
        size_t pass = 0;
        for (size_t i = 0; i < n; ++i) {
            if (processSample(buf[i])) ++pass;
            else buf[i] = ComplexSample(0.f, 0.f);
        }
        return pass;
    }

    bool isOpen() const { return open_; }
    float levelDb() const {
        return 10.f * std::log10(std::max(level_pow_, 1e-20f));
    }

    void reset() {
        level_pow_ = 0.f;
        open_ = false;
        above_count_ = 0;
        below_count_ = 0;
    }

private:
    bool updateState(float instant_pow) {
        // EMA of squared amplitude
        level_pow_ = (1.f - cfg_.ema_alpha) * level_pow_ +
                     cfg_.ema_alpha * instant_pow;
        float lvl_db = 10.f * std::log10(std::max(level_pow_, 1e-20f));

        if (open_) {
            if (lvl_db < cfg_.close_threshold_db) {
                if (++below_count_ >= cfg_.hold_samples) {
                    open_ = false;
                    below_count_ = 0;
                }
            } else {
                below_count_ = 0;
            }
        } else {
            if (lvl_db > cfg_.open_threshold_db) {
                if (++above_count_ >= cfg_.attack_samples) {
                    open_ = true;
                    above_count_ = 0;
                }
            } else {
                above_count_ = 0;
            }
        }
        return cfg_.enabled ? open_ : true;
    }

    SquelchConfig cfg_;
    float level_pow_ = 0.f;
    bool open_       = false;
    uint32_t above_count_ = 0;
    uint32_t below_count_ = 0;
};

} // namespace dsca
