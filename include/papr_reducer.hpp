/**
 * @file papr_reducer.hpp
 * @brief PAPR (Peak-to-Average Power Ratio) reduction via Tone Reservation
 *
 * Tone Reservation (TR) reserves a small set of subcarriers (Peak Reduction
 * Tones, PRTs) that carry no data. An iterative clipping-and-filtering
 * algorithm generates correction signals on these tones to reduce
 * time-domain peaks.
 *
 * Algorithm (simplified Active-Set / TR-SOCP):
 *   1. IFFT the data-bearing frequency-domain symbol → time-domain x(n)
 *   2. Find peak: n* = argmax |x(n)|
 *   3. If |x(n*)| / rms(x) <= target PAPR, done
 *   4. Clip: compute correction c(n*) = x(n*) · (1 - threshold/|x(n*)|)
 *   5. FFT the correction → C(k)
 *   6. Zero C(k) on all non-reserved tones → C_R(k)
 *   7. IFFT C_R(k) → c_R(n): the peak-reduction signal
 *   8. x(n) -= μ · c_R(n)     (μ = step size, typically 0.5-1.0)
 *   9. Repeat from step 2 up to max_iterations
 *
 * The reserved tones are chosen to be uniformly spaced across the
 * active bandwidth for best peak-reduction performance. They are
 * excluded from the normal data/pilot allocation.
 *
 * Reference: Tellado (2000), "Multicarrier Modulation with Low PAR"
 */
#pragma once

#include "types.hpp"
#include "fft_engine.hpp"
#include <vector>
#include <memory>

namespace gw {

// =========================================================================
// Configuration
// =========================================================================

struct PAPRConfig {
    float  target_papr_db   = 7.0f;   ///< Target PAPR in dB
    int    max_iterations    = 8;      ///< Max clipping iterations
    float  step_size         = 0.8f;   ///< Gradient step (0 < μ ≤ 1)
    float  reserve_fraction  = 0.05f;  ///< Fraction of active subcarriers reserved (1-10%)
    bool   enabled           = false;  ///< Master enable
};

struct PAPRStats {
    float  papr_before_db = 0.f;   ///< PAPR before reduction
    float  papr_after_db  = 0.f;   ///< PAPR after reduction
    int    iterations_used = 0;    ///< Iterations actually performed
    float  power_increase_db = 0.f; ///< Signal power increase from reserved tones
};

// =========================================================================
// PAPRReducer
// =========================================================================

class PAPRReducer {
public:
    /**
     * @param fft_size       FFT size of the OFDM system
     * @param active_start   First active subcarrier index
     * @param active_end     One-past-last active subcarrier index
     * @param data_indices   Indices used for data subcarriers
     * @param pilot_indices  Indices used for pilot subcarriers
     * @param cfg            PAPR configuration
     */
    PAPRReducer(size_t fft_size,
                size_t active_start,
                size_t active_end,
                const std::vector<size_t>& data_indices,
                const std::vector<size_t>& pilot_indices,
                const PAPRConfig& cfg = PAPRConfig());

    ~PAPRReducer();

    /**
     * Reduce PAPR of a frequency-domain OFDM symbol in-place.
     * The caller has already placed data and pilot symbols in freq_symbol.
     * This method modifies only the reserved tone bins.
     *
     * @param freq_symbol   Frequency-domain symbol (size = fft_size)
     * @param fft           FFT engine to use for transforms
     * @return              Statistics about the reduction
     */
    PAPRStats reduce(ComplexBuf& freq_symbol, FFTEngine& fft);

    /** Override the reserved-tone set with an externally-computed list (from
     *  the SubcarrierAllocation, which carves them out of the data carriers).
     *  Use this so TX/RX agree and the reducer operates on genuinely data-free
     *  tones instead of stealing live data carriers. */
    void useReservedTones(const std::vector<size_t>& reserved);

    /** Get the reserved tone indices */
    const std::vector<size_t>& reservedIndices() const { return reserved_indices_; }

    /** Number of reserved tones */
    size_t reservedCount() const { return reserved_indices_.size(); }

    /** Update configuration */
    void setConfig(const PAPRConfig& cfg) { cfg_ = cfg; }
    const PAPRConfig& config() const { return cfg_; }

    /** Compute PAPR of a time-domain signal in dB */
    static float computePAPR(const ComplexBuf& td);

private:
    void selectReservedTones(const std::vector<size_t>& data_indices,
                             const std::vector<size_t>& pilot_indices);

    PAPRConfig            cfg_;
    size_t                fft_size_;
    size_t                active_start_;
    size_t                active_end_;
    std::vector<size_t>   reserved_indices_;

    // Lookup: is bin k a reserved tone?
    std::vector<bool>     is_reserved_;

    // Work buffers (avoid allocation per call)
    ComplexBuf            time_buf_;
    ComplexBuf            clip_freq_;
    ComplexBuf            clip_time_;
};

} // namespace gw
