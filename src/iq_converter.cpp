/**
 * @file iq_converter.cpp
 * @brief I/Q upconversion, downconversion, and AGC
 */

#include "iq_converter.hpp"
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <numeric>

namespace dsca {

// =========================================================================
// IQUpconverter
// =========================================================================

IQUpconverter::IQUpconverter(uint32_t sample_rate, float center_freq)
    : fs_(sample_rate), fc_(center_freq)
{
    if (center_freq >= static_cast<float>(sample_rate) / 2.0f) {
        throw std::invalid_argument("Center freq must be < Nyquist");
    }
    nco_.setFrequency(fc_, static_cast<double>(fs_));
}

void IQUpconverter::upconvert(const ComplexBuf& baseband,
                               std::vector<float>& passband) {
    passband.resize(baseband.size());

    const bool filter = !lpf_taps_.empty();
    const size_t N = lpf_taps_.size();

    for (size_t n = 0; n < baseband.size(); ++n) {
        float i_in = baseband[n].real();
        float q_in = baseband[n].imag();

        if (filter) {
            // Push into circular delay lines and compute FIR output.
            bb_delay_i_[bb_delay_pos_] = i_in;
            bb_delay_q_[bb_delay_pos_] = q_in;
            float acc_i = 0.f, acc_q = 0.f;
            size_t idx = bb_delay_pos_;
            for (size_t k = 0; k < N; ++k) {
                acc_i += lpf_taps_[k] * bb_delay_i_[idx];
                acc_q += lpf_taps_[k] * bb_delay_q_[idx];
                idx = (idx == 0) ? (N - 1) : (idx - 1);
            }
            i_in = acc_i;
            q_in = acc_q;
            bb_delay_pos_ = (bb_delay_pos_ + 1) % N;
        }

        float cos_val, sin_val;
        nco_.stepRealImag(cos_val, sin_val);   // table NCO; advances phase (#67)

        // y[n] = I[n]*cos(ωn) - Q[n]*sin(ωn)
        passband[n] = i_in * cos_val - q_in * sin_val;
    }
}

void IQUpconverter::setLPF(float signal_bw, size_t num_taps) {
    if (num_taps == 0) {
        lpf_taps_.clear();
        bb_delay_i_.clear();
        bb_delay_q_.clear();
        bb_delay_pos_ = 0;
        return;
    }
    if (num_taps < 3) num_taps = 3;
    if (num_taps % 2 == 0) ++num_taps;  // keep symmetric for linear phase

    // The baseband LPF cutoff = one-sided signal bandwidth + 10% guard.
    // After upconvert this becomes a bandpass centered at fc with the
    // same cutoff per side, so signal_bw maps directly to mask edges.
    float cutoff = signal_bw * 1.1f;
    designLPF(cutoff, num_taps);
    bb_delay_i_.assign(num_taps, 0.f);
    bb_delay_q_.assign(num_taps, 0.f);
    bb_delay_pos_ = 0;
}

void IQUpconverter::designLPF(float cutoff_hz, size_t num_taps) {
    lpf_taps_.assign(num_taps, 0.f);
    double fc_norm = static_cast<double>(cutoff_hz) /
                     static_cast<double>(fs_);
    int M = static_cast<int>(num_taps - 1);
    double center = static_cast<double>(M) / 2.0;
    double sum = 0.0;
    for (size_t i = 0; i < num_taps; ++i) {
        double n = static_cast<double>(i) - center;
        double h;
        if (std::abs(n) < 1e-10) {
            h = 2.0 * fc_norm;
        } else {
            h = std::sin(2.0 * 3.14159265358979 * fc_norm * n) /
                (3.14159265358979 * n);
        }
        // Blackman window — same shape as the RX LPF for consistent
        // stopband suppression on both ends of the chain.
        double w = 0.42 - 0.5 * std::cos(2.0 * 3.14159265358979 *
                   static_cast<double>(i) / static_cast<double>(M)) +
                   0.08 * std::cos(4.0 * 3.14159265358979 *
                   static_cast<double>(i) / static_cast<double>(M));
        lpf_taps_[i] = static_cast<float>(h * w);
        sum += h * w;
    }
    if (sum != 0.0) {
        for (auto& t : lpf_taps_) t /= static_cast<float>(sum);
    }
}

void IQUpconverter::reset() {
    nco_.reset();
    std::fill(bb_delay_i_.begin(), bb_delay_i_.end(), 0.f);
    std::fill(bb_delay_q_.begin(), bb_delay_q_.end(), 0.f);
    bb_delay_pos_ = 0;
}

// =========================================================================
// IQDownconverter
// =========================================================================

IQDownconverter::IQDownconverter(uint32_t sample_rate, float center_freq,
                                  float signal_bw, size_t filter_taps)
    : fs_(sample_rate), fc_(center_freq), bw_(signal_bw),
      delay_pos_(0)
{
    if (center_freq >= static_cast<float>(sample_rate) / 2.0f) {
        throw std::invalid_argument("Center freq must be < Nyquist");
    }
    nco_.setFrequency(fc_, static_cast<double>(fs_));

    // Ensure odd number of taps
    if (filter_taps % 2 == 0) filter_taps++;

    // Design LPF with cutoff at signal bandwidth + 10% guard
    float cutoff = signal_bw * 1.1f;
    designLPF(cutoff, filter_taps);

    delay_i_.resize(filter_taps, 0.f);
    delay_q_.resize(filter_taps, 0.f);
}

void IQDownconverter::designLPF(float cutoff_hz, size_t num_taps) {
    lpf_taps_.resize(num_taps);

    double fc_norm = static_cast<double>(cutoff_hz) / static_cast<double>(fs_);
    int M = static_cast<int>(num_taps - 1);
    double center = static_cast<double>(M) / 2.0;

    double sum = 0.0;
    for (size_t i = 0; i < num_taps; ++i) {
        double n = static_cast<double>(i) - center;

        // Sinc function
        double h;
        if (std::abs(n) < 1e-10) {
            h = 2.0 * fc_norm;
        } else {
            h = std::sin(2.0 * 3.14159265358979 * fc_norm * n) /
                (3.14159265358979 * n);
        }

        // Blackman window
        double w = 0.42 - 0.5 * std::cos(2.0 * 3.14159265358979 *
                   static_cast<double>(i) / static_cast<double>(M)) +
                   0.08 * std::cos(4.0 * 3.14159265358979 *
                   static_cast<double>(i) / static_cast<double>(M));

        lpf_taps_[i] = static_cast<float>(h * w);
        sum += h * w;
    }

    // Normalize for unity gain at DC
    for (auto& t : lpf_taps_) t /= static_cast<float>(sum);
}

void IQDownconverter::downconvert(const float* passband, size_t num_samples,
                                   ComplexBuf& baseband) {
    baseband.resize(num_samples);

    size_t ntaps = lpf_taps_.size();

    for (size_t n = 0; n < num_samples; ++n) {
        float cos_val, sin_val;
        nco_.stepRealImag(cos_val, sin_val);   // table NCO; advances phase (#67)

        // Mix down: multiply by 2*cos and -2*sin
        float mix_i =  2.0f * passband[n] * cos_val;
        float mix_q = -2.0f * passband[n] * sin_val;

        // Push into delay lines
        delay_i_[delay_pos_] = mix_i;
        delay_q_[delay_pos_] = mix_q;

        // FIR filter (circular buffer convolution)
        float out_i = 0.f, out_q = 0.f;
        size_t idx = delay_pos_;
        for (size_t t = 0; t < ntaps; ++t) {
            out_i += delay_i_[idx] * lpf_taps_[t];
            out_q += delay_q_[idx] * lpf_taps_[t];
            if (idx == 0) idx = ntaps - 1;
            else idx--;
        }

        baseband[n] = ComplexSample(out_i, out_q);

        delay_pos_ = (delay_pos_ + 1) % ntaps;
    }
}

void IQDownconverter::reset() {
    nco_.reset();
    delay_pos_ = 0;
    std::fill(delay_i_.begin(), delay_i_.end(), 0.f);
    std::fill(delay_q_.begin(), delay_q_.end(), 0.f);
}

// =========================================================================
// AGC
// =========================================================================

AGC::AGC(uint32_t sample_rate, const AGCConfig& cfg)
    : fs_(sample_rate), cfg_(cfg), est_rms_(0.f)
{
    gain_ = std::pow(10.f, cfg_.initial_gain / 20.f);

    // Smoothing coefficients from time constants
    // alpha = 1 - exp(-1 / (fs * tau))
    float tau_attack  = cfg_.attack_ms / 1000.f;
    float tau_release = cfg_.release_ms / 1000.f;
    alpha_attack_  = 1.f - std::exp(-1.f / (static_cast<float>(fs_) * tau_attack));
    alpha_release_ = 1.f - std::exp(-1.f / (static_cast<float>(fs_) * tau_release));
}

// BLOCK-BASED gain, not per-sample. A per-sample AGC with a millisecond
// attack tracks the OFDM signal's own envelope (PAPR ~10 dB swings within
// every symbol — the attack tau of 5 ms is LESS than one FFT256 symbol at
// 48 kHz) and acts as a fast compressor: a time-varying gain multiplying
// the waveform inside the FFT window, i.e. a nonlinearity that smears
// subcarriers into each other (ICI). Constant-envelope test signals
// (sines) pass through unchanged, which is why tone-based tests never saw
// it — but no OFDM codeword survives the real-IF path with the envelope
// follower engaged. Here the level estimate is updated ONCE per block
// (attack/release alphas compounded over the block length) and the gain is
// applied as one LINEAR ramp from the previous block's gain to the new one:
// click-free, no intra-symbol modulation, and slow gain drift across
// symbols is absorbed by the pilot/channel tracking like any other slowly
// time-varying channel.
namespace {
inline float compoundAlpha(float per_sample_alpha, size_t n) {
    // 1 - (1-a)^n without pow() blowups for large n.
    if (per_sample_alpha >= 1.f) return 1.f;
    double keep = std::pow(1.0 - static_cast<double>(per_sample_alpha),
                           static_cast<double>(n));
    return static_cast<float>(1.0 - keep);
}
} // anonymous

namespace {
// Internal AGC granularity: gain is updated once per sub-block and applied
// as a linear ramp. ~21 ms at 48 kHz — long against the OFDM envelope
// (no compressor behavior), short enough to track level steps within a
// few blocks regardless of how large a buffer the caller hands us.
constexpr size_t AGC_SUB_BLOCK = 1024;
} // anonymous

void AGC::process(float* samples, size_t num_samples) {
    const float max_gain_lin = std::pow(10.f, cfg_.max_gain / 20.f);
    const float min_gain_lin = std::pow(10.f, cfg_.min_gain / 20.f);

    for (size_t off = 0; off < num_samples; off += AGC_SUB_BLOCK) {
        const size_t n = std::min(AGC_SUB_BLOCK, num_samples - off);
        float* blk = samples + off;

        double acc = 0.0;
        for (size_t i = 0; i < n; ++i)
            acc += static_cast<double>(blk[i]) * blk[i];
        const float block_power =
            static_cast<float>(acc / static_cast<double>(n));

        // Level smoothing with attack/release asymmetry, compounded over
        // the sub-block length so behavior is chunking-independent.
        const float alpha = compoundAlpha(
            (block_power > est_rms_ * est_rms_) ? alpha_attack_
                                                : alpha_release_, n);
        float est_power = est_rms_ * est_rms_;
        est_power = (1.f - alpha) * est_power + alpha * block_power;
        est_rms_ = std::sqrt(std::max(est_power, 1e-20f));

        const float g_start = std::clamp(gain_, min_gain_lin, max_gain_lin);
        if (est_rms_ > 1e-10f) {
            float desired = cfg_.target_rms / est_rms_;
            float g_alpha = compoundAlpha(
                (desired < gain_) ? alpha_attack_ : alpha_release_, n);
            gain_ = (1.f - g_alpha) * gain_ + g_alpha * desired;
        }
        gain_ = std::clamp(gain_, min_gain_lin, max_gain_lin);

        // Linear ramp across the sub-block: continuous at both ends.
        const float step = (gain_ - g_start) / static_cast<float>(n);
        float g = g_start;
        for (size_t i = 0; i < n; ++i) {
            g += step;
            blk[i] *= g;
        }
    }
}

void AGC::process(ComplexBuf& samples) {
    const size_t num_samples = samples.size();
    const float max_gain_lin = std::pow(10.f, cfg_.max_gain / 20.f);
    const float min_gain_lin = std::pow(10.f, cfg_.min_gain / 20.f);

    for (size_t off = 0; off < num_samples; off += AGC_SUB_BLOCK) {
        const size_t n = std::min(AGC_SUB_BLOCK, num_samples - off);
        ComplexSample* blk = samples.data() + off;

        double acc = 0.0;
        for (size_t i = 0; i < n; ++i)
            acc += static_cast<double>(std::norm(blk[i]));
        const float block_power =
            static_cast<float>(acc / static_cast<double>(n));

        const float alpha = compoundAlpha(
            (block_power > est_rms_ * est_rms_) ? alpha_attack_
                                                : alpha_release_, n);
        float est_power = est_rms_ * est_rms_;
        est_power = (1.f - alpha) * est_power + alpha * block_power;
        est_rms_ = std::sqrt(std::max(est_power, 1e-20f));

        const float g_start = std::clamp(gain_, min_gain_lin, max_gain_lin);
        if (est_rms_ > 1e-10f) {
            float desired = cfg_.target_rms / est_rms_;
            float g_alpha = compoundAlpha(
                (desired < gain_) ? alpha_attack_ : alpha_release_, n);
            gain_ = (1.f - g_alpha) * gain_ + g_alpha * desired;
        }
        gain_ = std::clamp(gain_, min_gain_lin, max_gain_lin);

        const float step = (gain_ - g_start) / static_cast<float>(n);
        float g = g_start;
        for (size_t i = 0; i < n; ++i) {
            g += step;
            blk[i] *= g;
        }
    }
}

float AGC::gainDB() const {
    return 20.f * std::log10(std::max(gain_, 1e-20f));
}

void AGC::reset() {
    gain_ = std::pow(10.f, cfg_.initial_gain / 20.f);
    est_rms_ = 0.f;
}

} // namespace dsca
