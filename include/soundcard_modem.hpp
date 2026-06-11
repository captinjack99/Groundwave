/**
 * @file soundcard_modem.hpp
 * @brief Soundcard-based modem: ties OFDM, I/Q conversion, and audio I/O
 *
 * The modem operates in two modes:
 *   1. Hardware mode: uses miniaudio for real soundcard I/O
 *   2. Loopback mode: TX output feeds directly to RX input (for testing)
 *
 * TX path:
 *   Complex baseband (from OFDM modulator)
 *   → I/Q upconvert to center frequency
 *   → real passband audio out (soundcard or loopback buffer)
 *
 * RX path:
 *   Real passband audio in (soundcard or loopback buffer)
 *   → AGC
 *   → I/Q downconvert to complex baseband
 *   → feed to OFDM demodulator
 *
 * Thread safety: the miniaudio callbacks run on audio threads. We use
 * lock-free ring buffers for TX/RX data exchange with the main thread.
 */
#pragma once

#include "types.hpp"
#include "iq_converter.hpp"
#include "dc_blocker.hpp"
#include "iq_balance.hpp"
#include "squelch.hpp"
#include "power_ramp.hpp"
#include "polyphase.hpp"
#include <vector>
#include <cstdint>
#include <functional>
#include <atomic>
#include <memory>
#include <mutex>

namespace gw {

// =========================================================================
// Ring Buffer (lock-free SPSC for audio thread communication)
// =========================================================================

class RingBuffer {
public:
    explicit RingBuffer(size_t capacity);

    /** Write samples. Returns number actually written. */
    size_t write(const float* data, size_t count);

    /** Read samples. Returns number actually read. */
    size_t read(float* data, size_t count);

    /** Number of samples available to read */
    size_t available() const;

    /** Number of samples that can be written */
    size_t space() const;

    void clear();

private:
    std::vector<float> buf_;
    std::atomic<size_t> read_pos_{0};
    std::atomic<size_t> write_pos_{0};
    size_t capacity_;
};

// =========================================================================
// Soundcard Modem Configuration
// =========================================================================

struct ModemConfig {
    uint32_t sample_rate   = 48000;   // Audio sample rate (Hz)
    float    center_freq   = 12000.f; // RF center frequency (Hz)
    float    signal_bw     = 0.f;     // Signal bandwidth (Hz), 0 = auto from OFDM params
    size_t   lpf_taps      = 129;     // Downconverter LPF FIR taps (was 65; 129 gives ~30 dB tighter mask)
    size_t   tx_lpf_taps   = 129;     // TX upconverter LPF taps for spectral-mask compliance; 0 = disable
    bool     enable_rs_outer = true;  // Reed-Solomon (16-byte parity) outer code wrapping LDPC info bytes
    bool     loopback      = true;    // Loopback mode (no hardware)
    bool     complex_loopback = false; // Complex baseband loopback (bypass IQ)
    AGCConfig agc;

    // Phase 1 primitives — feature-flagged so existing tests keep their behavior
    bool     enable_dc_blocker  = false;
    float    dc_blocker_cutoff_hz = 5.0f;     ///< Pre-AGC DC removal cutoff
    bool     enable_iq_balance  = false;      ///< Adaptive I/Q imbalance correction
    bool     enable_rx_squelch  = false;      ///< Energy-detector mute on idle
    SquelchConfig squelch;
    bool     enable_tx_ramp     = false;      ///< Raised-cosine TX keying
    uint32_t tx_ramp_samples    = 256;        ///< ~5 ms @ 48 kHz

    // Automatic Frequency Correction. When ON, the OFDMDemodulator's
    // 2nd-order phase-tracker PLL runs each frame and the SRO estimator
    // logs clock drift. Independent of `enable_iq_balance`.
    bool     enable_afc         = false;

    // Receive-side gain (dB) applied to the captured passband BEFORE the
    // AGC. Useful when the soundcard's analog mic-boost gain is too low
    // and you want a clean digital boost; pre-AGC so the AGC tracks it.
    float    rx_gain_db         = 0.f;

    // Optional sample-rate adaptation. If `ofdm_sample_rate != sample_rate`,
    // a polyphase rational resampler bridges the two domains: TX upsamples
    // OFDM-rate → audio-rate, RX downsamples audio-rate → OFDM-rate. Set
    // `enable_resampler` ON when feeding a fixed-rate hardware soundcard.
    bool     enable_resampler   = false;
    uint32_t ofdm_sample_rate   = 0;          ///< 0 = same as `sample_rate`

    // Per-direction sample rates. When non-zero, the TX path runs at
    // `tx_sample_rate` and the RX path at `rx_sample_rate`, with separate
    // resamplers bridging each to the OFDM rate. Useful when the playback
    // and capture devices support different rates (common on prosumer USB
    // interfaces). Zero falls back to the unified `sample_rate`.
    uint32_t tx_sample_rate     = 0;
    uint32_t rx_sample_rate     = 0;
};

// =========================================================================
// Soundcard Modem
// =========================================================================

class SoundcardModem {
public:
    SoundcardModem(const ModemConfig& cfg, const OFDMParams& ofdm_params);
    ~SoundcardModem();

    // Non-copyable
    SoundcardModem(const SoundcardModem&) = delete;
    SoundcardModem& operator=(const SoundcardModem&) = delete;

    // ----- TX (transmit) -----

    /** Queue complex baseband samples for transmission.
     *  Samples are upconverted to center_freq and sent to soundcard. */
    size_t transmit(const ComplexBuf& baseband);

    /** Queue raw real passband samples (already upconverted). */
    size_t transmitRaw(const float* samples, size_t count);

    /** Signal end-of-transmission (key-off). When the TX power ramp is
     *  enabled, this emits the held trailing samples with a raised-cosine
     *  fade to zero so the carrier ramps DOWN instead of being cut at full
     *  amplitude (which radiates broadband splatter). Must be called by the
     *  engine when TX transitions on→off. No-op when the ramp is disabled. */
    void endTransmission();

    // ----- RX (receive) -----

    /** Receive complex baseband samples.
     *  Reads from soundcard, AGC, downconverts from center_freq.
     *  @param max_samples  Max samples to return
     *  @return Received baseband samples (may be fewer than max) */
    ComplexBuf receive(size_t max_samples);

    /** Receive raw real passband samples (before downconversion). */
    size_t receiveRaw(float* samples, size_t max_samples);

    // ----- Loopback (for testing) -----

    /** In loopback mode: directly feed TX output to RX input.
     *  Call this to process queued TX data through the loopback path.
     *  In hardware mode this is a no-op. */
    void processLoopback();

    /** In loopback mode: feed TX output to RX with added AWGN.
     *  @param snr_db  Channel SNR in dB */
    void processLoopbackAWGN(float snr_db, uint32_t seed = 42);

    // ----- Status -----

    float agcGainDB()     const;
    float agcSignalLevel() const;
    bool  isLoopback()     const { return cfg_.loopback; }

    /// Live RX-gain update (applied in receive()) without a full modem rebuild,
    /// so dragging the Tuning panel's RX-gain slider is smooth instead of
    /// triggering a DSP teardown/reinit per step. Engine-thread only.
    void  setRxGainDb(float db) { cfg_.rx_gain_db = db; }

    void reset();

    const ModemConfig& config() const { return cfg_; }

    /** Access TX ring buffer (for hardware audio device to read from) */
    RingBuffer& txRing() { return tx_ring_; }

    /** Access RX ring buffer (for hardware audio device to write to) */
    RingBuffer& rxRing() { return rx_ring_; }

    /** Diagnostic accessor: bytes pending in the loopback ring. */
    size_t loopbackRingAvailable() const { return loopback_ring_.available(); }

private:
    ModemConfig cfg_;
    OFDMParams  ofdm_p_;

    IQUpconverter   upconv_;
    IQDownconverter downconv_;
    AGC             agc_;

    // Phase 1 live primitives — instantiated regardless; feature flags in
    // ModemConfig gate whether they actually mutate the signal.
    DCBlocker       dc_blocker_;
    IQBalance       iq_balance_;
    Squelch         squelch_;
    PowerRamp       tx_ramp_;
    // One-buffer-tail delay line for the TX ramp: the last tx_ramp_samples
    // of each transmitted buffer are held back so endTransmission() always
    // has real samples to fade down to zero (no duplication / no hard cut).
    // Only used when cfg_.enable_tx_ramp is set.
    std::vector<float> tx_hold_;

    // Rational resamplers between OFDM rate and audio rate. Allocated only
    // when `enable_resampler` is on and the rates differ.
    std::unique_ptr<RationalResampler> tx_resampler_;
    std::unique_ptr<RationalResampler> rx_resampler_;

    // Ring buffers for audio thread communication
    RingBuffer tx_ring_;    // TX: main thread writes, audio thread reads
    RingBuffer rx_ring_;    // RX: audio thread writes, main thread reads

    // Loopback buffer (TX passband → RX passband)
    RingBuffer loopback_ring_;

    // Complex baseband loopback buffer (bypasses IQ conversion)
    ComplexBuf complex_loopback_buf_;
};

} // namespace gw
