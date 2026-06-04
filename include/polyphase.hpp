/**
 * @file polyphase.hpp
 * @brief Polyphase decimator, interpolator, and rational-rate resampler.
 *
 * A length-N FIR prototype lowpass is split into M phases of length N/M.
 * Decimation-by-M and interpolation-by-L become single phase-bank lookups.
 * Rational L/M resampling combines them with a common phase index.
 *
 * Filter design: Kaiser-windowed sinc with a configurable transition band
 * (`transition_bw` in fractional sample rate, default 0.05) and stopband
 * attenuation (`stopband_db`, default 80 dB). Length is computed from
 * Kaiser's formula and rounded up to a multiple of M·L.
 */
#pragma once

#include "types.hpp"
#include <vector>
#include <cmath>
#include <algorithm>

namespace dsca {

struct PolyphaseConfig {
    float transition_bw = 0.05f;  ///< Transition band as fraction of fs (output)
    float stopband_db   = 80.0f;  ///< Stopband attenuation
    float gain          = 1.0f;   ///< DC gain of prototype
};

namespace polyphase_detail {

// Modified Bessel I0 series — accurate enough for window design
inline double besselI0(double x) {
    double sum = 1.0, term = 1.0;
    double xh2 = x * x * 0.25;
    for (int k = 1; k < 60; ++k) {
        term *= xh2 / static_cast<double>(k * k);
        sum += term;
        if (term < sum * 1e-12) break;
    }
    return sum;
}

inline std::vector<float> kaiserSinc(size_t length, float cutoff_norm,
                                      float beta, float gain) {
    std::vector<float> taps(length);
    double i0_beta = besselI0(beta);
    double mid = (length - 1) * 0.5;
    for (size_t i = 0; i < length; ++i) {
        double x = static_cast<double>(i) - mid;
        double sinc = (std::abs(x) < 1e-12)
                      ? 1.0
                      : std::sin(M_PI * 2.0 * cutoff_norm * x) /
                        (M_PI * 2.0 * cutoff_norm * x);
        double r = (length > 1) ? (2.0 * x / static_cast<double>(length - 1)) : 0.0;
        double w = besselI0(beta * std::sqrt(std::max(0.0, 1.0 - r * r))) / i0_beta;
        taps[i] = static_cast<float>(gain * 2.0 * cutoff_norm * sinc * w);
    }
    return taps;
}

inline float kaiserBeta(float stopband_db) {
    float A = stopband_db;
    if (A > 50.f) return 0.1102f * (A - 8.7f);
    if (A >= 21.f) return 0.5842f * std::pow(A - 21.f, 0.4f) + 0.07886f * (A - 21.f);
    return 0.f;
}

inline size_t kaiserLength(float transition_bw, float stopband_db) {
    float A = stopband_db;
    float df = std::max(transition_bw, 1e-3f);
    return static_cast<size_t>(std::ceil((A - 8.f) / (14.36f * df))) + 1;
}

} // namespace polyphase_detail

// =========================================================================
// Polyphase decimator: input rate -> input rate / M
// =========================================================================
class PolyphaseDecimator {
public:
    PolyphaseDecimator(size_t M, const PolyphaseConfig& cfg = PolyphaseConfig())
        : M_(M)
    {
        if (M == 0) M_ = 1;
        // Cutoff at 1/(2M) (the new Nyquist, as fraction of input fs)
        float cutoff = 0.5f / static_cast<float>(M_);
        float beta = polyphase_detail::kaiserBeta(cfg.stopband_db);
        size_t L = polyphase_detail::kaiserLength(cfg.transition_bw, cfg.stopband_db);
        // Round up to multiple of M for clean polyphase split
        L = ((L + M_ - 1) / M_) * M_;
        if (L == 0) L = M_;
        taps_ = polyphase_detail::kaiserSinc(L, cutoff, beta, cfg.gain);
        phase_taps_.assign(M_, std::vector<float>(L / M_));
        for (size_t i = 0; i < L; ++i) {
            phase_taps_[i % M_][i / M_] = taps_[i];
        }
        delay_.assign(L, 0.f);
        delay_c_.assign(L, ComplexSample(0.f, 0.f));
    }

    /** Process input samples; output count is input/M (truncated). */
    size_t process(const float* in, size_t n_in, float* out) {
        size_t n_out = 0;
        for (size_t i = 0; i < n_in; ++i) {
            // Shift in
            std::move_backward(delay_.begin(), delay_.end() - 1, delay_.end());
            delay_[0] = in[i];
            ++count_;
            if (count_ == M_) {
                count_ = 0;
                float acc = 0.f;
                for (size_t j = 0; j < taps_.size(); ++j) acc += taps_[j] * delay_[j];
                out[n_out++] = acc;
            }
        }
        return n_out;
    }

    size_t process(const ComplexSample* in, size_t n_in, ComplexSample* out) {
        size_t n_out = 0;
        for (size_t i = 0; i < n_in; ++i) {
            std::move_backward(delay_c_.begin(), delay_c_.end() - 1, delay_c_.end());
            delay_c_[0] = in[i];
            ++count_;
            if (count_ == M_) {
                count_ = 0;
                ComplexSample acc(0.f, 0.f);
                for (size_t j = 0; j < taps_.size(); ++j) acc += taps_[j] * delay_c_[j];
                out[n_out++] = acc;
            }
        }
        return n_out;
    }

    void reset() {
        std::fill(delay_.begin(), delay_.end(), 0.f);
        std::fill(delay_c_.begin(), delay_c_.end(), ComplexSample(0.f, 0.f));
        count_ = 0;
    }

    size_t M() const { return M_; }

private:
    size_t M_;
    std::vector<float> taps_;
    std::vector<std::vector<float>> phase_taps_;
    std::vector<float> delay_;
    std::vector<ComplexSample> delay_c_;
    size_t count_ = 0;
};

// =========================================================================
// Polyphase interpolator: input rate -> input rate * L
// =========================================================================
class PolyphaseInterpolator {
public:
    PolyphaseInterpolator(size_t L, const PolyphaseConfig& cfg = PolyphaseConfig())
        : L_(L)
    {
        if (L == 0) L_ = 1;
        float cutoff = 0.5f / static_cast<float>(L_);
        float beta = polyphase_detail::kaiserBeta(cfg.stopband_db);
        size_t flen = polyphase_detail::kaiserLength(cfg.transition_bw, cfg.stopband_db);
        flen = ((flen + L_ - 1) / L_) * L_;
        if (flen == 0) flen = L_;
        // Compensate for L-times zero-stuffing energy loss
        taps_ = polyphase_detail::kaiserSinc(flen, cutoff, beta, cfg.gain * static_cast<float>(L_));
        phase_taps_.assign(L_, std::vector<float>(flen / L_));
        for (size_t i = 0; i < flen; ++i) {
            phase_taps_[i % L_][i / L_] = taps_[i];
        }
        delay_.assign(flen / L_, 0.f);
        delay_c_.assign(flen / L_, ComplexSample(0.f, 0.f));
    }

    /** Produce L output samples per input sample. */
    size_t process(const float* in, size_t n_in, float* out) {
        size_t taps_per_phase = phase_taps_[0].size();
        size_t n_out = 0;
        for (size_t i = 0; i < n_in; ++i) {
            std::move_backward(delay_.begin(), delay_.end() - 1, delay_.end());
            delay_[0] = in[i];
            for (size_t p = 0; p < L_; ++p) {
                float acc = 0.f;
                for (size_t k = 0; k < taps_per_phase; ++k)
                    acc += phase_taps_[p][k] * delay_[k];
                out[n_out++] = acc;
            }
        }
        return n_out;
    }

    size_t process(const ComplexSample* in, size_t n_in, ComplexSample* out) {
        size_t taps_per_phase = phase_taps_[0].size();
        size_t n_out = 0;
        for (size_t i = 0; i < n_in; ++i) {
            std::move_backward(delay_c_.begin(), delay_c_.end() - 1, delay_c_.end());
            delay_c_[0] = in[i];
            for (size_t p = 0; p < L_; ++p) {
                ComplexSample acc(0.f, 0.f);
                for (size_t k = 0; k < taps_per_phase; ++k)
                    acc += phase_taps_[p][k] * delay_c_[k];
                out[n_out++] = acc;
            }
        }
        return n_out;
    }

    void reset() {
        std::fill(delay_.begin(), delay_.end(), 0.f);
        std::fill(delay_c_.begin(), delay_c_.end(), ComplexSample(0.f, 0.f));
    }

    size_t L() const { return L_; }

private:
    size_t L_;
    std::vector<float> taps_;
    std::vector<std::vector<float>> phase_taps_;
    std::vector<float> delay_;
    std::vector<ComplexSample> delay_c_;
};

// =========================================================================
// Rational L/M resampler
// =========================================================================
class RationalResampler {
public:
    RationalResampler(size_t L, size_t M, const PolyphaseConfig& cfg = PolyphaseConfig())
        : interp_(L, cfg), decim_(M, cfg), L_(L), M_(M) {}

    size_t outputCount(size_t n_in) const {
        // Approx: n_in * L / M (exact would track phase across calls)
        return (n_in * L_) / M_;
    }

    /** Process input through L↑ then M↓.
     *  Returns the number of output samples written. Caller must allocate
     *  out with at least outputCount(n_in) + L slack. */
    size_t process(const float* in, size_t n_in, float* out) {
        std::vector<float> mid(n_in * L_);
        size_t n_mid = interp_.process(in, n_in, mid.data());
        return decim_.process(mid.data(), n_mid, out);
    }

    size_t process(const ComplexSample* in, size_t n_in, ComplexSample* out) {
        std::vector<ComplexSample> mid(n_in * L_);
        size_t n_mid = interp_.process(in, n_in, mid.data());
        return decim_.process(mid.data(), n_mid, out);
    }

    void reset() { interp_.reset(); decim_.reset(); }

    size_t L() const { return L_; }
    size_t M() const { return M_; }

private:
    PolyphaseInterpolator interp_;
    PolyphaseDecimator    decim_;
    size_t L_, M_;
};

} // namespace dsca
