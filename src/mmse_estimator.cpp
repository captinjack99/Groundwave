/**
 * @file mmse_estimator.cpp
 * @brief MMSE Channel Estimator with Wiener Interpolation
 *
 * Implementation details:
 *
 * 1. Frequency-domain channel autocorrelation for exponential PDP:
 *      R_HH(Δf) = 1 / (1 + j·2π·τ_rms·Δf)
 *    where Δf = (k₁ - k₂) · Δf_sub is the frequency separation.
 *
 * 2. MMSE pilot gain:
 *      w_k = R_HH(0) / (R_HH(0) + 1/SNR_pilot)
 *          = SNR / (SNR + 1)
 *    This is a scalar MMSE denoising of each LS pilot estimate.
 *
 * 3. Wiener interpolation from pilots to data:
 *    For each data subcarrier d, find N_interp nearest pilots and compute:
 *      H_est[d] = Σ_k w_dk · H_LS[p_k]
 *    where w_dk = r_d^H · R_pp^{-1} with:
 *      r_d[k]    = R_HH(f_d - f_{p_k})   (cross-correlation)
 *      R_pp[j,k] = R_HH(f_{p_j} - f_{p_k}) + δ_{jk}/SNR  (auto + noise)
 *
 *    We solve via Cholesky on the small NxN system (N = 2·interp_order+1).
 *
 * 4. Temporal smoothing: exponential IIR on the full estimate.
 */

#include "mmse_estimator.hpp"
#include <cmath>
#include <algorithm>
#include <numeric>
#include <complex>

namespace dsca {

// =========================================================================
// Helpers
// =========================================================================

namespace {

// Frequency-domain autocorrelation for exponential PDP
// R_HH(Δf) = 1 / (1 + j·2π·τ_rms·Δf)
// Returns |R_HH(Δf)| for real-valued Wiener weights (common simplification)
inline float r_hh_mag(float delta_f_hz, float tau_rms_sec) {
    float x = 2.0f * static_cast<float>(M_PI) * tau_rms_sec * delta_f_hz;
    return 1.0f / std::sqrt(1.0f + x * x);
}

// Complex R_HH
inline ComplexSample r_hh_complex(float delta_f_hz, float tau_rms_sec) {
    float x = 2.0f * static_cast<float>(M_PI) * tau_rms_sec * delta_f_hz;
    // 1/(1+jx) = (1-jx)/(1+x²)
    float denom = 1.0f + x * x;
    return ComplexSample(1.0f / denom, -x / denom);
}

// Solve small positive-definite system Ax = b via Cholesky
// A is n×n stored row-major, b and x are length n
// Returns false if not positive definite
bool choleskySolve(const std::vector<float>& A, const std::vector<float>& b,
                   std::vector<float>& x, size_t n) {
    // Cholesky: A = L·L^T
    std::vector<float> L(n * n, 0.f);
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = 0; j <= i; ++j) {
            float sum = 0.f;
            for (size_t k = 0; k < j; ++k)
                sum += L[i * n + k] * L[j * n + k];
            if (i == j) {
                float val = A[i * n + i] - sum;
                if (val <= 0.f) return false;
                L[i * n + j] = std::sqrt(val);
            } else {
                L[i * n + j] = (A[i * n + j] - sum) / L[j * n + j];
            }
        }
    }

    // Forward substitution: L·y = b
    std::vector<float> y(n, 0.f);
    for (size_t i = 0; i < n; ++i) {
        float sum = 0.f;
        for (size_t k = 0; k < i; ++k)
            sum += L[i * n + k] * y[k];
        y[i] = (b[i] - sum) / L[i * n + i];
    }

    // Back substitution: L^T·x = y
    x.resize(n, 0.f);
    for (size_t i = n; i > 0; --i) {
        size_t ii = i - 1;
        float sum = 0.f;
        for (size_t k = ii + 1; k < n; ++k)
            sum += L[k * n + ii] * x[k];
        x[ii] = (y[ii] - sum) / L[ii * n + ii];
    }

    return true;
}

} // anonymous

// =========================================================================
// Construction
// =========================================================================

MMSEChannelEstimator::MMSEChannelEstimator(
    size_t fft_size,
    const std::vector<size_t>& pilot_indices,
    const std::vector<size_t>& data_indices,
    const ComplexBuf& pilot_values,
    uint32_t sample_rate,
    const MMSEConfig& cfg)
    : fft_size_(fft_size)
    , cfg_(cfg)
    , sample_rate_(sample_rate)
    , pilot_idx_(pilot_indices)
    , data_idx_(data_indices)
    , pilot_ref_(pilot_values)
{
    h_est_.resize(fft_size, ComplexSample(1.f, 0.f));
    h_mag_.resize(fft_size, 1.f);
    h_ls_pilot_raw_.resize(pilot_idx_.size(), ComplexSample(1.f, 0.f));
    h_ls_pilot_.resize(pilot_idx_.size(), ComplexSample(1.f, 0.f));

    active_snr_db_ = cfg_.pilot_snr_db;   // weights initially assume the config SNR
    computeWienerWeights();
}

// =========================================================================
// Wiener Weight Computation (done once at construction)
// =========================================================================

void MMSEChannelEstimator::computeWienerWeights() {
    float tau_sec = cfg_.tau_rms_us * 1e-6f;
    // Regularize with the SNR the weights currently assume (active_snr_db_),
    // which tracks the live estimate on a slow cadence (#46). Clamp to a sane
    // band so a transient SNR spike/crash can't produce degenerate weights.
    float snr_clamped = std::clamp(active_snr_db_, 3.f, 35.f);
    float snr_lin = std::pow(10.f, snr_clamped / 10.f);
    float subcarrier_spacing = static_cast<float>(sample_rate_) /
                               static_cast<float>(fft_size_);

    // MMSE gain at pilot positions: w = SNR/(SNR+1)
    float pilot_gain = snr_lin / (snr_lin + 1.0f);
    pilot_wiener_gain_.resize(pilot_idx_.size(), pilot_gain);

    // Wiener interpolation weights for each data subcarrier
    interp_weights_.resize(data_idx_.size());
    size_t np = pilot_idx_.size();
    size_t half_win = cfg_.interp_order;

    for (size_t di = 0; di < data_idx_.size(); ++di) {
        size_t d_bin = data_idx_[di];
        float f_d = static_cast<float>(d_bin) * subcarrier_spacing;

        // Find nearest pilot
        size_t nearest = 0;
        float min_dist = std::numeric_limits<float>::max();
        for (size_t pi = 0; pi < np; ++pi) {
            float dist = std::abs(static_cast<float>(d_bin) -
                                  static_cast<float>(pilot_idx_[pi]));
            if (dist < min_dist) { min_dist = dist; nearest = pi; }
        }

        // Select pilot window centered on nearest
        size_t win_lo = (nearest >= half_win) ? nearest - half_win : 0;
        size_t win_hi = std::min(nearest + half_win + 1, np);
        size_t win_n  = win_hi - win_lo;

        auto& iw = interp_weights_[di];
        iw.pilot_neighbors.resize(win_n);
        for (size_t i = 0; i < win_n; ++i)
            iw.pilot_neighbors[i] = win_lo + i;

        // Build R_pp (pilot autocorrelation + noise diagonal)
        std::vector<float> R_pp(win_n * win_n, 0.f);
        for (size_t i = 0; i < win_n; ++i) {
            float f_pi = static_cast<float>(pilot_idx_[win_lo + i]) * subcarrier_spacing;
            for (size_t j = 0; j < win_n; ++j) {
                float f_pj = static_cast<float>(pilot_idx_[win_lo + j]) * subcarrier_spacing;
                float df = f_pi - f_pj;
                R_pp[i * win_n + j] = r_hh_mag(df, tau_sec);
                if (i == j) {
                    R_pp[i * win_n + j] += 1.0f / snr_lin;
                }
            }
        }

        // Build r_dp (cross-correlation between data and pilots)
        std::vector<float> r_dp(win_n, 0.f);
        for (size_t i = 0; i < win_n; ++i) {
            float f_pi = static_cast<float>(pilot_idx_[win_lo + i]) * subcarrier_spacing;
            float df = f_d - f_pi;
            r_dp[i] = r_hh_mag(df, tau_sec);
        }

        // Solve R_pp · w = r_dp
        if (!choleskySolve(R_pp, r_dp, iw.weights, win_n)) {
            // Fallback: equal weights
            iw.weights.assign(win_n, 1.0f / static_cast<float>(win_n));
        }
    }
}

// =========================================================================
// LS Estimation at Pilots
// =========================================================================

void MMSEChannelEstimator::lsEstimateAtPilots(const ComplexBuf& freq_data) {
    for (size_t i = 0; i < pilot_idx_.size(); ++i) {
        size_t idx = pilot_idx_[i];
        ComplexSample ref = pilot_ref_[i];
        if (std::abs(ref) > 1e-6f) {
            ComplexSample h_ls = freq_data[idx] / ref;
            // Keep the raw LS estimate (used for unbiased noise estimation)
            // and a separately denoised version for Wiener interpolation.
            h_ls_pilot_raw_[i] = h_ls;
            h_ls_pilot_[i]     = h_ls * pilot_wiener_gain_[i];
        }
    }
}

// =========================================================================
// Wiener Interpolation
// =========================================================================

void MMSEChannelEstimator::wienerInterpolate() {
    // Set pilot positions directly
    for (size_t i = 0; i < pilot_idx_.size(); ++i) {
        size_t idx = pilot_idx_[i];
        ComplexSample new_est = h_ls_pilot_[i];

        // Temporal smoothing
        if (cfg_.smoothing > 0.f && cfg_.smoothing < 1.f) {
            new_est = cfg_.smoothing * h_est_[idx] +
                      (1.0f - cfg_.smoothing) * new_est;
        }

        h_est_[idx] = new_est;
        h_mag_[idx] = std::abs(new_est);
    }

    // Interpolate to data positions
    for (size_t di = 0; di < data_idx_.size(); ++di) {
        size_t d_bin = data_idx_[di];
        auto& iw = interp_weights_[di];

        ComplexSample h_interp(0.f, 0.f);
        for (size_t k = 0; k < iw.pilot_neighbors.size(); ++k) {
            h_interp += h_ls_pilot_[iw.pilot_neighbors[k]] * iw.weights[k];
        }

        // Temporal smoothing
        if (cfg_.smoothing > 0.f && cfg_.smoothing < 1.f) {
            h_interp = cfg_.smoothing * h_est_[d_bin] +
                       (1.0f - cfg_.smoothing) * h_interp;
        }

        h_est_[d_bin] = h_interp;
        h_mag_[d_bin] = std::abs(h_interp);
    }
}

// =========================================================================
// Noise Estimation
// =========================================================================

void MMSEChannelEstimator::updateNoiseEstimate() {
    // Noise estimate comes from the residual between RAW LS pilot estimates
    // and the smoothed/interpolated channel estimate. Using h_ls_pilot_raw_
    // (un-Wiener'd) here keeps the residual unbiased; using the gain-applied
    // version would shrink the residual proportional to pilot_wiener_gain
    // and underestimate noise (especially at low SNR).
    float residual = 0.f;
    float sig = 0.f;
    for (size_t i = 0; i < pilot_idx_.size(); ++i) {
        size_t idx = pilot_idx_[i];
        ComplexSample diff = h_ls_pilot_raw_[i] - h_est_[idx];
        residual += std::norm(diff);
        sig += std::norm(h_est_[idx]);
    }

    float np = static_cast<float>(pilot_idx_.size());
    if (np > 0.f) {
        // Running average
        float new_noise = residual / np;
        noise_accum_ = 0.9f * noise_accum_ + 0.1f * new_noise;
        noise_var_ = std::max(noise_accum_, 1e-10f);

        float sig_avg = sig / np;
        if (noise_var_ > 0.f) {
            snr_db_ = 10.f * std::log10(sig_avg / noise_var_);
        }
    }
}

// =========================================================================
// Public Interface
// =========================================================================

void MMSEChannelEstimator::update(const ComplexBuf& freq_data) {
    if (freq_data.size() < fft_size_) return;

    lsEstimateAtPilots(freq_data);
    wienerInterpolate();
    updateNoiseEstimate();

    // Adapt the Wiener regularization to the live SNR on a slow cadence (#46).
    // Rebuilding the weights costs a Cholesky per data subcarrier, so only
    // every ~64 symbols, and only when the tracked SNR has drifted ≥ 3 dB from
    // what the current weights assume (otherwise the filter stays tuned to a
    // fixed 20 dB regardless of the real channel).
    if (++weight_adapt_ctr_ >= 64) {
        weight_adapt_ctr_ = 0;
        if (std::abs(snr_db_ - active_snr_db_) >= 3.f) {
            active_snr_db_ = snr_db_;
            computeWienerWeights();
        }
    }
}

void MMSEChannelEstimator::initFromPreamble(const ComplexBuf& h1,
                                             const ComplexBuf& h2) {
    if (h1.size() < fft_size_ || h2.size() < fft_size_) return;

    // Average two preamble estimates
    float sig_power = 0.f, noise_power = 0.f;
    for (size_t i = 0; i < fft_size_; ++i) {
        h_est_[i] = (h1[i] + h2[i]) * 0.5f;
        h_mag_[i] = std::abs(h_est_[i]);
        sig_power += std::norm(h_est_[i]);
        noise_power += std::norm(h1[i] - h2[i]) * 0.5f;
    }

    // Initialize LS pilot estimates from preamble (both raw and denoised
    // start from the averaged preamble estimate, which has noise variance σ²/2
    // already — we still seed the raw buffer so the first noise-update has a
    // sane baseline).
    for (size_t i = 0; i < pilot_idx_.size(); ++i) {
        h_ls_pilot_raw_[i] = h_est_[pilot_idx_[i]];
        h_ls_pilot_[i]     = h_est_[pilot_idx_[i]];
    }

    // Initial SNR and noise
    if (noise_power > 0.f) {
        snr_db_ = 10.f * std::log10(sig_power / noise_power);
        noise_var_ = noise_power / static_cast<float>(fft_size_);
        noise_accum_ = noise_var_;
    }
}

void MMSEChannelEstimator::reset() {
    std::fill(h_est_.begin(), h_est_.end(), ComplexSample(1.f, 0.f));
    std::fill(h_mag_.begin(), h_mag_.end(), 1.f);
    std::fill(h_ls_pilot_.begin(), h_ls_pilot_.end(), ComplexSample(1.f, 0.f));
    std::fill(h_ls_pilot_raw_.begin(), h_ls_pilot_raw_.end(),
              ComplexSample(1.f, 0.f));
    noise_var_ = 0.1f;
    snr_db_ = 0.f;
    noise_accum_ = 0.f;

    // Return the Wiener regularization to the configured baseline on a fresh
    // acquisition; it will re-adapt to the live SNR within ~64 symbols. (#46)
    weight_adapt_ctr_ = 0;
    if (active_snr_db_ != cfg_.pilot_snr_db) {
        active_snr_db_ = cfg_.pilot_snr_db;
        computeWienerWeights();
    }
}

} // namespace dsca
