/**
 * @file amc.hpp
 * @brief Adaptive Modulation & Coding (AMC) selection table.
 *
 * Wraps the existing 77-combination ModCod space (7 modulations × 11 FEC
 * rates) into a sortable, queryable table indexed by required SNR threshold.
 * The AMC controller picks the highest-throughput ModCod whose threshold
 * is below the link's measured SNR minus a configurable margin.
 *
 * Hysteresis prevents rapid mode-flapping: switching to a higher ModCod
 * requires SNR to exceed the next threshold by `up_margin_db`; switching
 * down only needs SNR to fall by `down_margin_db`.
 *
 * Built on top of snr_calculator.hpp for thresholds.
 */
#pragma once

#include "types.hpp"
#include "snr_calculator.hpp"
#include <vector>
#include <algorithm>
#include <cstdint>

namespace gw {

struct AMCEntry {
    Modulation mod;
    FECRate    fec;
    float      threshold_db;       ///< SNR needed for ~quasi-error-free
    float      spectral_eff_bps_hz;///< Net coded bits per Hz
};

struct AMCConfig {
    float up_margin_db   = 2.0f;  ///< Headroom required to UPGRADE
    float down_margin_db = 1.0f;  ///< Hysteresis on DOWNGRADE
    float ema_alpha_snr  = 0.1f;  ///< SNR smoothing (0=stuck, 1=instant)
    int   min_dwell_frames = 4;   ///< Min frames between mode changes
    bool  enabled        = true;

    // Outer-Loop Link Adaptation (OLLA). The static threshold table is an
    // estimate; real links sit slightly off it. OLLA nudges an SNR bias
    // from observed frame pass/fail to self-calibrate to a target error
    // rate: each failure raises the bias (be more conservative), each
    // success lowers it, with the up/down step ratio set so the loop
    // settles at olla_target_bler. The effective SNR fed to the selector
    // is (measured_snr - olla_bias). Off by default; harmless when AMC is
    // off. (SOTA roadmap #4)
    bool  olla_enabled     = false;
    float olla_target_bler = 0.05f;  ///< Target block-error rate
    float olla_step_db     = 0.2f;   ///< Up-step on a failure
    float olla_max_bias_db = 6.0f;   ///< Clamp on the accumulated bias
};

class AMCSelector {
public:
    explicit AMCSelector(const AMCConfig& cfg = AMCConfig()) : cfg_(cfg) {
        rebuildTable();
    }

    /** Re-derive the threshold table (call after channel/encoder changes). */
    void rebuildTable() {
        auto raw = computeAllThresholds();   // free function in gw::
        table_.clear();
        table_.reserve(raw.size());
        for (const auto& t : raw) {
            AMCEntry e;
            e.mod                = t.modulation;
            e.fec                = t.fec_rate;
            e.threshold_db       = t.threshold_db;
            e.spectral_eff_bps_hz = t.spectral_eff;
            table_.push_back(e);
        }
        std::sort(table_.begin(), table_.end(),
                  [](const AMCEntry& a, const AMCEntry& b){
                      return a.threshold_db < b.threshold_db;
                  });
    }

    /** Feed measured SNR (dB) and current ModCod. Returns the recommended
     *  ModCod. May be the same as input if no change is warranted. */
    AMCEntry recommend(float measured_snr_db, Modulation cur_mod, FECRate cur_fec) {
        if (!cfg_.enabled) {
            return findEntry(cur_mod, cur_fec);
        }
        // EMA-smoothed SNR
        snr_ema_ = (1.f - cfg_.ema_alpha_snr) * snr_ema_ +
                   cfg_.ema_alpha_snr * measured_snr_db;
        ++frames_since_change_;

        AMCEntry current = findEntry(cur_mod, cur_fec);
        AMCEntry best = current;

        if (frames_since_change_ < cfg_.min_dwell_frames) return current;

        // OLLA: operate on the bias-corrected SNR so persistent off-table
        // behavior is absorbed (see reportFrameResult). (SOTA #4)
        const float snr_eff = snr_ema_ - (cfg_.olla_enabled ? olla_bias_db_ : 0.f);
        (void)snr_eff;  // used below in place of snr_ema_

        // Pick the highest-THROUGHPUT (spectral-efficiency) mode whose
        // threshold (+ up_margin) the smoothed SNR clears. Threshold and
        // spectral efficiency are NOT co-monotonic across the modcod table
        // (e.g. QPSK 3/4 out-throughputs QAM16 2/5 at a *lower* threshold),
        // so taking the highest-threshold feasible entry — the old
        // break-on-first-from-rbegin — could select a LOWER-throughput
        // mode. Scan all feasible entries and maximize spectral efficiency. (#27)
        bool found = false;
        for (const auto& e : table_) {
            if (e.threshold_db + cfg_.up_margin_db <= snr_eff) {
                if (!found || e.spectral_eff_bps_hz > best.spectral_eff_bps_hz) {
                    best  = e;
                    found = true;
                }
            }
        }
        if (!found) best = current;  // SNR below every threshold — hold

        // If we're considering a downgrade, require down_margin hysteresis
        if (best.spectral_eff_bps_hz < current.spectral_eff_bps_hz) {
            if (current.threshold_db - cfg_.down_margin_db > snr_eff) {
                // Downgrade is justified
            } else {
                best = current;  // no change
            }
        }

        if (best.mod != current.mod || best.fec != current.fec) {
            frames_since_change_ = 0;
        }
        return best;
    }

    /** Outer-loop update from one frame's decode outcome. On a failure the
     *  SNR bias steps up by olla_step_db; on success it steps down by
     *  olla_step_db·BLER/(1-BLER). The fixed point is reached when the
     *  long-run failure fraction equals olla_target_bler. No-op unless
     *  olla_enabled. (SOTA #4) */
    void reportFrameResult(bool crc_ok) {
        if (!cfg_.olla_enabled) return;
        const float bler = std::clamp(cfg_.olla_target_bler, 1e-3f, 0.5f);
        if (crc_ok) {
            olla_bias_db_ -= cfg_.olla_step_db * (bler / (1.f - bler));
        } else {
            olla_bias_db_ += cfg_.olla_step_db;
        }
        if (olla_bias_db_ >  cfg_.olla_max_bias_db) olla_bias_db_ =  cfg_.olla_max_bias_db;
        if (olla_bias_db_ < -cfg_.olla_max_bias_db) olla_bias_db_ = -cfg_.olla_max_bias_db;
    }
    float ollaBiasDb() const { return olla_bias_db_; }

    const std::vector<AMCEntry>& table() const { return table_; }
    float smoothedSnr() const { return snr_ema_; }
    void  setConfig(const AMCConfig& cfg) { cfg_ = cfg; }
    const AMCConfig& config() const { return cfg_; }
    void  reset() { snr_ema_ = 0.f; frames_since_change_ = cfg_.min_dwell_frames;
                    olla_bias_db_ = 0.f; }

private:
    AMCEntry findEntry(Modulation m, FECRate r) const {
        for (const auto& e : table_) if (e.mod == m && e.fec == r) return e;
        return table_.empty() ? AMCEntry{m, r, 0.f, 0.f} : table_.front();
    }

    AMCConfig cfg_;
    std::vector<AMCEntry> table_;
    float snr_ema_ = 0.f;
    int   frames_since_change_ = 0;
    float olla_bias_db_ = 0.f;   ///< OLLA accumulated SNR bias (SOTA #4)
};

} // namespace gw
