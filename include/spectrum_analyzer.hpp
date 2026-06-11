/**
 * @file spectrum_analyzer.hpp
 * @brief Real-time power spectrum analyzer for GUI display
 *
 * Takes audio samples (real passband or complex baseband) and produces
 * a smoothed power spectrum in dBFS for direct QPainter rendering.
 *
 * Features:
 *  - Configurable FFT size (independent of OFDM FFT)
 *  - Hann window function (sidelobe suppression)
 *  - Exponential averaging for smooth display (leak coefficient)
 *  - Peak hold with configurable decay
 *  - Waterfall ring buffer management
 *  - Thread-safe: DSP thread pushes, GUI thread pulls
 */

#pragma once

#include "types.hpp"
#include "fft_engine.hpp"
#include "app_state.hpp"
#include <vector>
#include <array>
#include <memory>
#include <mutex>
#include <cmath>
#include <numeric>
#include <algorithm>

namespace gw {

// =========================================================================
// Spectrum Analyzer
// =========================================================================

struct SpectrumConfig {
    size_t   fft_size      = SPECTRUM_BINS * 2; ///< Internal FFT (power of 2)
    float    sample_rate   = 48000.f;
    float    avg_leak      = 0.85f;  ///< Exponential avg coefficient (0=instant,1=frozen)
    float    ref_db        = 0.f;    ///< 0 dBFS reference
    bool     complex_input = false;  ///< true = complex baseband, false = real passband
};

class SpectrumAnalyzer {
public:
    explicit SpectrumAnalyzer(const SpectrumConfig& cfg = {});

    // ---- Input ----

    /** Push real passband samples for analysis.
     *  Thread-safe — safe to call from audio thread. */
    void pushSamples(const float* samples, size_t count);

    /** Push complex baseband samples for analysis.
     *  Thread-safe — safe to call from audio thread. */
    void pushComplex(const ComplexSample* samples, size_t count);
    void pushComplex(const ComplexBuf& buf) {
        pushComplex(buf.data(), buf.size());
    }

    // ---- Output (GUI thread) ----

    /** Compute spectrum from accumulated samples. Returns true if new data.
     *  @param out_data  SpectrumData to update in place */
    bool update(SpectrumData& out_data);

    /** Force a clear (e.g. on config change). */
    void reset();

    const SpectrumConfig& config() const { return cfg_; }

private:
    SpectrumConfig cfg_;
    std::unique_ptr<FFTEngine> fft_;

    // Input ring buffer
    std::vector<float> sample_buf_;   ///< Real input accumulator
    ComplexBuf         complex_buf_;  ///< Complex input accumulator
    size_t             buf_write_pos_ = 0;
    size_t             buf_fill_     = 0;

    // Analysis buffers
    std::vector<float>  window_;       ///< Hann window coefficients
    ComplexBuf          fft_in_;
    ComplexBuf          fft_out_;
    std::vector<float>  power_avg_;    ///< Averaged power spectrum (linear)
    std::vector<float>  peak_hold_;    ///< Peak hold (dB)

    mutable std::mutex  mtx_;
    bool                has_new_data_ = false;

    void computeWindow();
    void runFFT();
    float binToDb(float linear_power) const;
};

} // namespace gw
