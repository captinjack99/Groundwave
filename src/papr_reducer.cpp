/**
 * @file papr_reducer.cpp
 * @brief PAPR reduction via Tone Reservation implementation
 *
 * Tone selection strategy: uniformly space reserved tones across the
 * active band, avoiding existing data and pilot positions.
 *
 * The iterative algorithm clips time-domain peaks and projects the
 * correction back onto the reserved tones only, preserving data integrity.
 */

#include "papr_reducer.hpp"
#include <cmath>
#include <algorithm>
#include <numeric>

namespace dsca {

// =========================================================================
// Construction
// =========================================================================

PAPRReducer::PAPRReducer(size_t fft_size,
                         size_t active_start,
                         size_t active_end,
                         const std::vector<size_t>& data_indices,
                         const std::vector<size_t>& pilot_indices,
                         const PAPRConfig& cfg)
    : cfg_(cfg)
    , fft_size_(fft_size)
    , active_start_(active_start)
    , active_end_(active_end)
    , is_reserved_(fft_size, false)
{
    selectReservedTones(data_indices, pilot_indices);

    // Pre-allocate work buffers
    time_buf_.resize(fft_size);
    clip_freq_.resize(fft_size);
    clip_time_.resize(fft_size);
}

PAPRReducer::~PAPRReducer() = default;

// =========================================================================
// Reserved tone selection
// =========================================================================

void PAPRReducer::selectReservedTones(
        const std::vector<size_t>& data_indices,
        const std::vector<size_t>& pilot_indices) {
    // pilot_indices kept in the signature for API symmetry; reservation
    // currently steals only from data carriers.
    (void)pilot_indices;
    // In a typical OFDM system, all active bins are data or pilot.
    // Tone reservation STEALS data carriers — they become dedicated
    // peak-reduction tones. We pick uniformly spaced data carriers.

    if (data_indices.empty()) {
        reserved_indices_.clear();
        return;
    }

    // Determine how many tones to reserve
    size_t total_active = active_end_ - active_start_;
    size_t n_reserve = static_cast<size_t>(
        std::ceil(cfg_.reserve_fraction * static_cast<float>(total_active)));
    // Clamp to at most half the data carriers
    n_reserve = std::min(n_reserve, data_indices.size() / 2);
    // Minimum 1 tone if enabled and data carriers exist
    if (n_reserve == 0 && !data_indices.empty() && cfg_.enabled) {
        n_reserve = 1;
    }

    if (n_reserve == 0) {
        reserved_indices_.clear();
        return;
    }

    // Select uniformly spaced data carriers for reservation
    reserved_indices_.clear();
    reserved_indices_.reserve(n_reserve);

    float step = static_cast<float>(data_indices.size()) /
                 static_cast<float>(n_reserve);
    for (size_t i = 0; i < n_reserve; ++i) {
        size_t idx = static_cast<size_t>(
            static_cast<float>(i) * step + 0.5f);
        idx = std::min(idx, data_indices.size() - 1);
        reserved_indices_.push_back(data_indices[idx]);
    }

    // Sort for consistent access
    std::sort(reserved_indices_.begin(), reserved_indices_.end());

    // Remove duplicates (in case step < 1)
    reserved_indices_.erase(
        std::unique(reserved_indices_.begin(), reserved_indices_.end()),
        reserved_indices_.end());

    // Build fast lookup
    std::fill(is_reserved_.begin(), is_reserved_.end(), false);
    for (auto idx : reserved_indices_) {
        is_reserved_[idx] = true;
    }
}

// =========================================================================
// PAPR computation
// =========================================================================

float PAPRReducer::computePAPR(const ComplexBuf& td) {
    if (td.empty()) return 0.f;

    float peak_power = 0.f;
    float avg_power  = 0.f;

    for (const auto& s : td) {
        float p = std::norm(s);
        if (p > peak_power) peak_power = p;
        avg_power += p;
    }

    avg_power /= static_cast<float>(td.size());
    if (avg_power < 1e-20f) return 0.f;

    return 10.f * std::log10(peak_power / avg_power);
}

// =========================================================================
// Main reduction algorithm
// =========================================================================

PAPRStats PAPRReducer::reduce(ComplexBuf& freq_symbol, FFTEngine& fft) {
    PAPRStats stats;

    if (!cfg_.enabled || reserved_indices_.empty()) {
        // Not enabled — just compute PAPR for stats
        fft.inverse(freq_symbol, time_buf_);
        stats.papr_before_db = computePAPR(time_buf_);
        stats.papr_after_db  = stats.papr_before_db;
        stats.iterations_used = 0;
        stats.power_increase_db = 0.f;
        return stats;
    }

    // Ensure reserved tones start at zero
    for (auto idx : reserved_indices_) {
        freq_symbol[idx] = ComplexSample(0.f, 0.f);
    }

    // IFFT to get initial time-domain signal
    fft.inverse(freq_symbol, time_buf_);

    // Measure initial PAPR
    stats.papr_before_db = computePAPR(time_buf_);

    // Compute initial average power
    float avg_power_before = 0.f;
    for (const auto& s : time_buf_) avg_power_before += std::norm(s);
    avg_power_before /= static_cast<float>(time_buf_.size());

    // Target threshold in linear amplitude
    float target_papr_lin = std::pow(10.f, cfg_.target_papr_db / 10.f);

    // Iterative clipping and filtering
    int iter = 0;
    for (; iter < cfg_.max_iterations; ++iter) {
        // Compute current average power
        float avg_power = 0.f;
        for (const auto& s : time_buf_) avg_power += std::norm(s);
        avg_power /= static_cast<float>(time_buf_.size());
        if (avg_power < 1e-20f) break;

        // Clipping threshold = sqrt(target_PAPR_linear * avg_power)
        float clip_threshold = std::sqrt(target_papr_lin * avg_power);

        // Find peak
        float peak_amp = 0.f;
        for (const auto& s : time_buf_) {
            float a = std::abs(s);
            if (a > peak_amp) peak_amp = a;
        }

        // Check if we've met the target
        float current_papr_db = 10.f * std::log10(
            (peak_amp * peak_amp) / avg_power);
        if (current_papr_db <= cfg_.target_papr_db) break;

        // Generate clipping signal: c(n) = x(n) - clip(x(n))
        for (size_t n = 0; n < fft_size_; ++n) {
            float amp = std::abs(time_buf_[n]);
            if (amp > clip_threshold) {
                // Clip: keep phase, reduce amplitude to threshold
                clip_time_[n] = time_buf_[n] -
                    time_buf_[n] * (clip_threshold / amp);
            } else {
                clip_time_[n] = ComplexSample(0.f, 0.f);
            }
        }

        // FFT the clipping signal
        fft.forward(clip_time_, clip_freq_);

        // Zero non-reserved tones (keep only reserved-tone components)
        for (size_t k = 0; k < fft_size_; ++k) {
            if (!is_reserved_[k]) {
                clip_freq_[k] = ComplexSample(0.f, 0.f);
            }
        }

        // IFFT back to get peak-reduction signal on reserved tones only
        fft.inverse(clip_freq_, clip_time_);

        // Subtract scaled correction from time-domain signal
        float mu = cfg_.step_size;
        for (size_t n = 0; n < fft_size_; ++n) {
            time_buf_[n] -= mu * clip_time_[n];
        }
    }

    stats.iterations_used = iter;
    stats.papr_after_db = computePAPR(time_buf_);

    // Now extract what went onto the reserved tones:
    // FFT the final time-domain signal to get the modified frequency domain
    ComplexBuf final_freq;
    fft.forward(time_buf_, final_freq);

    // Copy ONLY the reserved tones back to the frequency symbol
    // (data and pilot tones are preserved exactly as they were)
    for (auto idx : reserved_indices_) {
        freq_symbol[idx] = final_freq[idx];
    }

    // Compute power increase
    float avg_power_after = 0.f;
    for (const auto& s : time_buf_) avg_power_after += std::norm(s);
    avg_power_after /= static_cast<float>(time_buf_.size());

    if (avg_power_before > 1e-20f) {
        stats.power_increase_db = 10.f * std::log10(
            avg_power_after / avg_power_before);
    }

    return stats;
}

} // namespace dsca
