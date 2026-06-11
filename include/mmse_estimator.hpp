/**
 * @file mmse_estimator.hpp
 * @brief MMSE Channel Estimator with Wiener Interpolation
 *
 * Replaces the simple LS + linear interpolation in OFDMDemodulator
 * with a proper MMSE (Minimum Mean Square Error) approach:
 *
 *   1. LS estimates at pilot positions: H_LS[k] = Y[k] / X[k]
 *   2. Wiener filtering: smooth LS estimates using frequency-domain
 *      correlation (exploits channel delay spread structure)
 *   3. MMSE interpolation to data subcarriers using Wiener coefficients
 *
 * The Wiener filter minimizes E[|H - H_est|²] given the channel's
 * power delay profile (PDP). For unknown PDP, we assume exponential
 * with configurable RMS delay spread τ_rms.
 *
 * Theory (Edfors, Sandell, van de Beek — 1998):
 *   H_MMSE = R_HH · (R_HH + σ²/σ²_x · I)^{-1} · H_LS
 *
 * where R_HH is the channel autocorrelation matrix in frequency domain.
 * For computational efficiency we use a per-subcarrier scalar MMSE filter
 * (DFT-based approach) rather than full matrix inversion.
 */
#pragma once

#include "types.hpp"
#include <vector>
#include <cstddef>

namespace gw {

struct MMSEConfig {
    float tau_rms_us    = 5.0f;   ///< RMS delay spread (microseconds)
    float pilot_snr_db  = 20.0f;  ///< Assumed pilot SNR for Wiener weights
    size_t interp_order = 4;      ///< Wiener interpolation taps (each side)
    float  smoothing    = 0.8f;   ///< Temporal smoothing (0=no smooth, 1=no update)
};

class MMSEChannelEstimator {
public:
    /**
     * @param fft_size       FFT size (N)
     * @param pilot_indices  Indices of pilot subcarriers in FFT
     * @param data_indices   Indices of data subcarriers in FFT
     * @param pilot_values   Known pilot symbols (for LS estimation)
     * @param sample_rate    Sample rate in Hz (for delay spread → freq correlation)
     * @param cfg            MMSE configuration
     */
    MMSEChannelEstimator(size_t fft_size,
                         const std::vector<size_t>& pilot_indices,
                         const std::vector<size_t>& data_indices,
                         const ComplexBuf& pilot_values,
                         uint32_t sample_rate,
                         const MMSEConfig& cfg = MMSEConfig());

    /**
     * Update channel estimate from one OFDM symbol's frequency-domain data.
     * @param freq_data  FFT output of received OFDM symbol (size = fft_size)
     */
    void update(const ComplexBuf& freq_data);

    /**
     * Process preamble for initial high-quality estimate.
     * Uses two long training symbols averaged (same as OFDMDemodulator).
     * @param h1  Channel estimate from first long symbol (size = fft_size)
     * @param h2  Channel estimate from second long symbol (size = fft_size)
     */
    void initFromPreamble(const ComplexBuf& h1, const ComplexBuf& h2);

    /** Get the full channel estimate (fft_size entries) */
    const ComplexBuf& estimate() const { return h_est_; }

    /** Get channel magnitude at each subcarrier */
    const SampleBuf& magnitude() const { return h_mag_; }

    /** Estimated noise variance */
    float noiseVariance() const { return noise_var_; }

    /** Estimated SNR in dB */
    float snrDB() const { return snr_db_; }

    /** Reset to default (flat) channel */
    void reset();

private:
    void computeWienerWeights();
    void lsEstimateAtPilots(const ComplexBuf& freq_data);
    void wienerInterpolate();
    void updateNoiseEstimate();

    // Configuration
    size_t fft_size_;
    MMSEConfig cfg_;
    uint32_t sample_rate_;

    // Subcarrier maps
    std::vector<size_t> pilot_idx_;
    std::vector<size_t> data_idx_;
    ComplexBuf pilot_ref_;

    // Wiener interpolation weights
    // For each data subcarrier: weights to apply to neighboring pilot LS estimates
    struct InterpWeights {
        std::vector<size_t> pilot_neighbors; // indices into pilot_idx_ array
        std::vector<float>  weights;         // corresponding weights
    };
    std::vector<InterpWeights> interp_weights_; // one per data subcarrier

    // Wiener smoothing weights for pilot positions
    std::vector<float> pilot_wiener_gain_; // MMSE gain at each pilot

    // Channel state
    ComplexBuf h_est_;          // Full channel estimate (fft_size)
    SampleBuf  h_mag_;          // |H| at each subcarrier
    ComplexBuf h_ls_pilot_raw_; // RAW LS estimates at pilot positions (no MMSE gain)
    ComplexBuf h_ls_pilot_;     // MMSE-denoised LS estimates (for interpolation)
    float      noise_var_ = 0.1f;
    float      snr_db_    = 0.f;

    // Noise estimation (from pilot residuals)
    float noise_accum_   = 0.f;

    // Live-SNR adaptation of the Wiener regularization (#46). The weights are
    // expensive to rebuild (a Cholesky per data subcarrier), so they are
    // recomputed only on a slow cadence and only when the tracked SNR has
    // drifted materially from the SNR the current weights assume. Seeded from
    // cfg_.pilot_snr_db; thereafter follows the live estimate.
    float active_snr_db_     = 20.f;   // SNR baked into the current weights
    int   weight_adapt_ctr_  = 0;      // symbols since last adapt check
};

} // namespace gw
