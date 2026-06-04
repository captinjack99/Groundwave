/**
 * @file sync_fsm.hpp
 * @brief Sync state machine — Searching → Acquiring → Tracking → Lost.
 *
 * Replaces the binary "in-sync" flag with an explicit FSM that the GUI
 * and engine can both query. Transitions are driven by per-frame
 * confidence metrics (preamble correlation, BER estimate, lock counter).
 */
#pragma once

#include "types.hpp"  // SyncState lives here
#include <cstdint>
#include <algorithm>

namespace dsca {

inline const char* syncStateName(SyncState s) {
    switch (s) {
        case SyncState::Searching: return "Searching";
        case SyncState::Acquiring: return "Acquiring";
        case SyncState::Locked:    return "Locked";
        case SyncState::Tracking:  return "Tracking";
        case SyncState::Lost:      return "Lost";
    }
    return "?";
}

struct SyncFSMConfig {
    int acquire_frames_required = 4;   ///< Consecutive good frames to enter Locked
    int track_frames_required   = 20;  ///< Sustained good frames in Locked → Tracking
    int lost_frames_required    = 8;   ///< Consecutive bad frames to drop from Locked/Tracking → Lost
    int searching_after_lost    = 16;  ///< Bad frames in Lost → Searching
    float min_correlation       = 0.5f;///< Preamble correlation needed
    float max_ber_locked        = 0.1f;///< BER threshold to keep Locked
};

class SyncFSM {
public:
    explicit SyncFSM(const SyncFSMConfig& cfg = SyncFSMConfig()) : cfg_(cfg) {}

    /** Feed one frame's quality metrics. */
    SyncState feed(bool preamble_detected, float correlation, float ber_estimate) {
        bool good = preamble_detected && correlation >= cfg_.min_correlation;

        switch (state_) {
            case SyncState::Searching:
                if (good) {
                    state_ = SyncState::Acquiring;
                    consecutive_good_ = 1;
                    consecutive_bad_  = 0;
                }
                break;

            case SyncState::Acquiring:
                if (good) {
                    if (++consecutive_good_ >= cfg_.acquire_frames_required) {
                        state_ = SyncState::Locked;
                        consecutive_bad_ = 0;
                    }
                } else {
                    if (++consecutive_bad_ >= 2) {
                        state_ = SyncState::Searching;
                        consecutive_good_ = 0;
                        consecutive_bad_  = 0;
                    }
                }
                break;

            case SyncState::Locked:
                if (good && ber_estimate <= cfg_.max_ber_locked) {
                    consecutive_bad_  = 0;
                    consecutive_good_ = std::min(consecutive_good_ + 1, 1000);
                    // Promote to Tracking once the lock has been stable for
                    // track_frames_required consecutive good frames. (Was
                    // unreachable — the enum/docs/GUI referenced Tracking
                    // but no transition ever set it.) (#44)
                    if (consecutive_good_ >= cfg_.track_frames_required) {
                        state_ = SyncState::Tracking;
                    }
                } else {
                    if (++consecutive_bad_ >= cfg_.lost_frames_required) {
                        state_ = SyncState::Lost;
                        consecutive_good_ = 0;
                    }
                }
                break;

            case SyncState::Tracking:
                // Stable steady-state lock. Good frames keep us here; a
                // run of bad frames falls back to Lost (same threshold as
                // Locked). A single bad frame demotes to Locked so the
                // status reflects the wobble without losing sync.
                if (good && ber_estimate <= cfg_.max_ber_locked) {
                    consecutive_bad_  = 0;
                    consecutive_good_ = std::min(consecutive_good_ + 1, 1000);
                } else {
                    state_ = SyncState::Locked;
                    if (++consecutive_bad_ >= cfg_.lost_frames_required) {
                        state_ = SyncState::Lost;
                        consecutive_good_ = 0;
                    }
                }
                break;

            case SyncState::Lost:
                if (good) {
                    state_ = SyncState::Acquiring;
                    consecutive_good_ = 1;
                    consecutive_bad_  = 0;
                } else if (++consecutive_bad_ >= cfg_.searching_after_lost) {
                    state_ = SyncState::Searching;
                    consecutive_bad_ = 0;
                }
                break;
        }
        return state_;
    }

    SyncState state() const { return state_; }
    int consecutiveGood() const { return consecutive_good_; }
    int consecutiveBad()  const { return consecutive_bad_;  }
    void reset() {
        state_ = SyncState::Searching;
        consecutive_good_ = 0;
        consecutive_bad_  = 0;
    }

private:
    SyncFSMConfig cfg_;
    SyncState state_ = SyncState::Searching;
    int consecutive_good_ = 0;
    int consecutive_bad_  = 0;
};

} // namespace dsca
