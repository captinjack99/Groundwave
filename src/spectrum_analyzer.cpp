/**
 * @file spectrum_analyzer.cpp
 * @brief Real-time power spectrum analyzer implementation
 */

#include "spectrum_analyzer.hpp"
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <memory>     // std::make_unique (not transitive under libc++)

namespace gw {

// =========================================================================
// Construction
// =========================================================================

SpectrumAnalyzer::SpectrumAnalyzer(const SpectrumConfig& cfg)
    : cfg_(cfg)
{
    if (cfg_.fft_size < 64)
        throw std::invalid_argument("SpectrumAnalyzer: fft_size must be >= 64");

    fft_     = std::make_unique<FFTEngine>(cfg_.fft_size);
    fft_in_  .resize(cfg_.fft_size, ComplexSample(0.f, 0.f));
    fft_out_ .resize(cfg_.fft_size, ComplexSample(0.f, 0.f));
    window_  .resize(cfg_.fft_size, 1.f);
    power_avg_.resize(SPECTRUM_BINS, 1e-10f);  // tiny floor
    peak_hold_.resize(SPECTRUM_BINS, -80.f);

    // Ring buffer: hold two FFT windows worth of samples
    sample_buf_ .resize(cfg_.fft_size * 2, 0.f);
    complex_buf_.resize(cfg_.fft_size * 2, ComplexSample(0.f, 0.f));

    computeWindow();
}

// =========================================================================
// Window Function (Hann)
// =========================================================================

void SpectrumAnalyzer::computeWindow() {
    const size_t N = cfg_.fft_size;
    for (size_t i = 0; i < N; ++i) {
        double w = 0.5 * (1.0 - std::cos(2.0 * M_PI * static_cast<double>(i)
                                          / static_cast<double>(N - 1)));
        window_[i] = static_cast<float>(w);
    }
}

// =========================================================================
// Input Push
// =========================================================================

void SpectrumAnalyzer::pushSamples(const float* samples, size_t count) {
    std::lock_guard<std::mutex> lock(mtx_);
    const size_t bufsz = sample_buf_.size();
    for (size_t i = 0; i < count; ++i) {
        sample_buf_[buf_write_pos_] = samples[i];
        buf_write_pos_ = (buf_write_pos_ + 1) % bufsz;
        if (buf_fill_ < bufsz) ++buf_fill_;
    }
    if (buf_fill_ >= cfg_.fft_size) {
        has_new_data_ = true;
    }
}

void SpectrumAnalyzer::pushComplex(const ComplexSample* samples, size_t count) {
    std::lock_guard<std::mutex> lock(mtx_);
    const size_t bufsz = complex_buf_.size();
    for (size_t i = 0; i < count; ++i) {
        complex_buf_[buf_write_pos_] = samples[i];
        buf_write_pos_ = (buf_write_pos_ + 1) % bufsz;
        if (buf_fill_ < bufsz) ++buf_fill_;
    }
    if (buf_fill_ >= cfg_.fft_size) {
        has_new_data_ = true;
    }
}

// =========================================================================
// FFT + Power
// =========================================================================

void SpectrumAnalyzer::runFFT() {
    const size_t N     = cfg_.fft_size;
    const size_t bufsz = sample_buf_.size();

    // Build windowed FFT input from ring buffer (most recent N samples)
    if (cfg_.complex_input) {
        // Complex baseband: full spectrum in bins 0..N-1
        size_t start = (buf_write_pos_ + bufsz - N) % bufsz;
        for (size_t i = 0; i < N; ++i) {
            ComplexSample s = complex_buf_[(start + i) % bufsz];
            fft_in_[i] = s * window_[i];
        }
    } else {
        // Real passband: use real part only → one-sided spectrum, bins 0..N/2
        size_t start = (buf_write_pos_ + bufsz - N) % bufsz;
        for (size_t i = 0; i < N; ++i) {
            float s = sample_buf_[(start + i) % bufsz];
            fft_in_[i] = ComplexSample(s * window_[i], 0.f);
        }
    }

    // Run forward FFT
    fft_->forward(fft_in_, fft_out_);

    // Compute power and bin into SPECTRUM_BINS display bins
    const float inv_N2 = 1.f / static_cast<float>(N * N);
    const float leak   = cfg_.avg_leak;
    const size_t output_bins = cfg_.complex_input ? N : N / 2;
    const float  bin_ratio   = static_cast<float>(output_bins)
                               / static_cast<float>(SPECTRUM_BINS);

    // Use center of mass for each display bin
    for (size_t d = 0; d < SPECTRUM_BINS; ++d) {
        size_t fft_start = static_cast<size_t>(static_cast<float>(d) * bin_ratio);
        size_t fft_end   = static_cast<size_t>(static_cast<float>(d + 1) * bin_ratio);
        if (fft_end <= fft_start) fft_end = fft_start + 1;
        if (fft_end > output_bins) fft_end = output_bins;

        float power = 0.f;
        for (size_t b = fft_start; b < fft_end; ++b) {
            float re = fft_out_[b].real();
            float im = fft_out_[b].imag();
            power += (re * re + im * im) * inv_N2;
        }
        power /= static_cast<float>(fft_end - fft_start);

        // Exponential average (leak=0: instant, leak=1: frozen)
        power_avg_[d] = leak * power_avg_[d] + (1.f - leak) * power;
    }
}

float SpectrumAnalyzer::binToDb(float linear_power) const {
    const float floor = 1e-12f;
    return 10.f * std::log10(std::max(linear_power, floor)) + cfg_.ref_db;
}

// =========================================================================
// Update (GUI thread)
// =========================================================================

bool SpectrumAnalyzer::update(SpectrumData& out_data) {
    bool new_data = false;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        if (!has_new_data_) return false;
        has_new_data_ = false;
        new_data = true;
        runFFT();
    }

    // Convert averaged power to dB and write to SpectrumData
    float peak = -80.f;
    float noise_accum = 0.f;
    size_t noise_count = 0;

    // Frequency axis
    const float sr = cfg_.sample_rate;
    const float hz_per_display_bin = (cfg_.complex_input)
        ? sr / static_cast<float>(SPECTRUM_BINS)
        : sr / (2.f * static_cast<float>(SPECTRUM_BINS));

    for (size_t i = 0; i < SPECTRUM_BINS; ++i) {
        out_data.freqs_hz[i] = static_cast<float>(i) * hz_per_display_bin;
        float db = binToDb(power_avg_[i]);
        out_data.power_db[i] = db;

        if (db > peak) peak = db;

        // Estimate noise floor from bottom 20% of bins (by power)
        if (db < -30.f) {
            noise_accum += db;
            ++noise_count;
        }
    }

    out_data.peak_db = peak;
    if (noise_count > 0)
        out_data.noise_floor = noise_accum / static_cast<float>(noise_count);

    // Advance the waterfall-row counter. The SpectrumWidget owns the actual
    // scrolling waterfall image and builds each row from `power_db`, so we no
    // longer maintain a separate full history buffer here — that was 256 KB of
    // never-rendered dead work refilled every frame (#49). The counter alone
    // tells consumers a fresh row is available.
    out_data.waterfall_write_row = (out_data.waterfall_write_row + 1) % WATERFALL_ROWS;

    return new_data;
}

// =========================================================================
// Reset
// =========================================================================

void SpectrumAnalyzer::reset() {
    std::lock_guard<std::mutex> lock(mtx_);
    std::fill(sample_buf_.begin(),  sample_buf_.end(),  0.f);
    std::fill(complex_buf_.begin(), complex_buf_.end(), ComplexSample(0.f, 0.f));
    std::fill(power_avg_.begin(),   power_avg_.end(),   1e-10f);
    std::fill(peak_hold_.begin(),   peak_hold_.end(),   -80.f);
    buf_write_pos_ = 0;
    buf_fill_      = 0;
    has_new_data_  = false;
}

} // namespace gw
