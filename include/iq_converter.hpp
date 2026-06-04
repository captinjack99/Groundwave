/**
 * @file iq_converter.hpp
 * @brief I/Q upconversion, downconversion, and AGC for soundcard modem
 *
 * Upconversion (TX):
 *   Complex baseband → real passband at center frequency fc.
 *   y[n] = I[n]*cos(2π·fc·n/fs) - Q[n]*sin(2π·fc·n/fs)
 *
 * Downconversion (RX):
 *   Real passband → complex baseband.
 *   I[n] = 2 * x[n] * cos(2π·fc·n/fs)  → lowpass → decimate
 *   Q[n] = -2 * x[n] * sin(2π·fc·n/fs) → lowpass → decimate
 *   The factor of 2 compensates for the image rejection.
 *
 * The lowpass filter uses a windowed-sinc FIR (Blackman window) with
 * cutoff at the OFDM bandwidth edge.
 *
 * AGC:
 *   Exponential smoothing of signal power with attack/release asymmetry.
 *   Output normalized to target RMS level.
 */
#pragma once

#include "types.hpp"
#include "nco.hpp"
#include <vector>
#include <cstdint>

namespace dsca {

// =========================================================================
// I/Q Upconverter (TX: complex baseband → real passband)
// =========================================================================

class IQUpconverter {
public:
    /**
     * @param sample_rate   Audio sample rate (Hz)
     * @param center_freq   Center frequency (Hz) — must be < sample_rate/2
     */
    IQUpconverter(uint32_t sample_rate, float center_freq);

    /** Convert complex baseband to real passband.
     *  Output has same number of samples as input. */
    void upconvert(const ComplexBuf& baseband, std::vector<float>& passband);

    /** Enable a post-upconvert bandpass filter to tighten spectral mask
     *  compliance (FCC §73.319 adjacent-channel rejection). Implemented as
     *  a symmetric FIR with cutoff at center_freq ± signal_bw/2. Costs N
     *  multiply-adds per output sample; with num_taps=129 at 48 kHz this
     *  is ~6 MFLOPS, negligible on modern CPUs.
     *
     *  Pass num_taps=0 to disable filtering (back-compat / loopback paths).
     *  Pass an even number and it gets incremented to keep the filter
     *  symmetric (linear-phase).
     *
     *  @param signal_bw  One-sided signal bandwidth (Hz) — the LPF cutoff
     *                     applied to the baseband before the upconvert
     *                     (equivalent to bandpass filtering the passband).
     *  @param num_taps   FIR filter length. 129 is a sensible default;
     *                     65 is too short for sharp mask compliance,
     *                     257+ tightens the mask further at higher CPU
     *                     cost. Must be ≥ 3. */
    void setLPF(float signal_bw, size_t num_taps);

    /** Reset oscillator phase (e.g., for new transmission) */
    void reset();

    float centerFreq()  const { return fc_; }
    uint32_t sampleRate() const { return fs_; }
    bool     hasLPF()    const { return !lpf_taps_.empty(); }

private:
    void designLPF(float cutoff_hz, size_t num_taps);

    uint32_t fs_;
    float    fc_;
    // Table-based NCO replaces per-sample libm sin/cos in the hot loop (#67).
    // <1e-4 peak error → ~80 dBc, far above the IQ round-trip SNR thresholds.
    TableNCO nco_;

    // Optional baseband LPF applied before upconvert. We filter the
    // complex baseband (I and Q each through the same real-valued FIR)
    // rather than the passband, because filtering before the carrier
    // multiplication is half the work and yields the same result for a
    // mask-shaping filter whose cutoff is one-sided signal_bw.
    std::vector<float> lpf_taps_;
    std::vector<float> bb_delay_i_;
    std::vector<float> bb_delay_q_;
    size_t bb_delay_pos_ = 0;
};

// =========================================================================
// I/Q Downconverter (RX: real passband → complex baseband)
// =========================================================================

class IQDownconverter {
public:
    /**
     * @param sample_rate     Audio sample rate (Hz)
     * @param center_freq     Center frequency (Hz)
     * @param signal_bw       One-sided signal bandwidth (Hz) for LPF cutoff
     * @param filter_taps     Number of FIR filter taps (odd, higher = sharper)
     */
    IQDownconverter(uint32_t sample_rate, float center_freq,
                    float signal_bw, size_t filter_taps = 65);

    /** Downconvert real passband to complex baseband.
     *  Output has same number of samples as input. */
    void downconvert(const float* passband, size_t num_samples,
                     ComplexBuf& baseband);

    void reset();

    float centerFreq()  const { return fc_; }
    uint32_t sampleRate() const { return fs_; }

private:
    void designLPF(float cutoff_hz, size_t num_taps);
    void applyFilter(const std::vector<float>& in, std::vector<float>& out);

    uint32_t fs_;
    float    fc_;
    float    bw_;
    TableNCO nco_;   // table-based oscillator (#67)

    // Lowpass FIR filter
    std::vector<float> lpf_taps_;
    std::vector<float> delay_i_;   // I channel delay line
    std::vector<float> delay_q_;   // Q channel delay line
    size_t delay_pos_;
};

// =========================================================================
// Automatic Gain Control
// =========================================================================

struct AGCConfig {
    float target_rms    = 0.25f;   // Target output RMS level
    float attack_ms     = 5.0f;    // Attack time constant (ms)
    float release_ms    = 50.0f;   // Release time constant (ms)
    float max_gain      = 60.0f;   // Max gain (dB)
    float min_gain      = -20.0f;  // Min gain (dB)
    float initial_gain  = 0.0f;    // Initial gain (dB)
};

class AGC {
public:
    explicit AGC(uint32_t sample_rate, const AGCConfig& cfg = AGCConfig());

    /** Apply AGC to a block of real samples (in-place). */
    void process(float* samples, size_t num_samples);

    /** Apply AGC to a block of complex samples (in-place). */
    void process(ComplexBuf& samples);

    /** Get current gain in dB */
    float gainDB() const;

    /** Get estimated signal level (RMS) */
    float signalLevel() const { return est_rms_; }

    void reset();

private:
    uint32_t fs_;
    AGCConfig cfg_;
    float gain_;         // current linear gain
    float est_rms_;      // estimated signal RMS
    float alpha_attack_;
    float alpha_release_;
};

} // namespace dsca
