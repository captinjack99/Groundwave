/**
 * @file ofdm.hpp
 * @brief OFDM Modulator, Demodulator, and Synchronizer
 *
 * All components share the same subcarrier allocation algorithm,
 * same pilot PRNG seed, and same preamble definition.
 * Complex baseband throughout — no legacy real paths.
 *
 * Preamble design:
 *   Defined in the FREQUENCY DOMAIN as known BPSK symbols on all
 *   active subcarriers. TX IFFT's these to get time-domain samples.
 *   RX FFT's the received preamble and divides by the known
 *   frequency-domain reference. Correct by construction.
 */
#pragma once

#include "types.hpp"
#include "fft_engine.hpp"
#include "symbol_mapper.hpp"
#include "mmse_estimator.hpp"
#include "integer_cfo.hpp"
#include "phase_tracker.hpp"
#include "sample_rate_offset.hpp"
#include <memory>

namespace dsca {

class PAPRReducer;  // Forward declaration

// =========================================================================
// Subcarrier Allocation — computed identically by TX and RX
// =========================================================================

struct SubcarrierAllocation {
    std::vector<size_t> data_indices;   // FFT bin indices for data
    std::vector<size_t> pilot_indices;  // FFT bin indices for pilots
    ComplexBuf          pilot_values;   // known BPSK pilot symbols
    size_t              guard_left;
    size_t              guard_right;

    size_t dataCount()  const { return data_indices.size(); }
    size_t pilotCount() const { return pilot_indices.size(); }
};

/** Compute subcarrier allocation from OFDM parameters.
 *  This function is the SINGLE SOURCE OF TRUTH — used by
 *  OFDMModulator, OFDMDemodulator, and preamble generation. */
SubcarrierAllocation computeAllocation(const OFDMParams& p);

// =========================================================================
// Preamble — frequency-domain BPSK known symbols
// =========================================================================

/** Generate frequency-domain preamble reference (known BPSK on all active bins).
 *  Used directly by the demodulator for channel estimation. */
ComplexBuf generatePreambleFreqDomain(const OFDMParams& p);

/** Generate time-domain preamble: short training + long training w/ CP.
 *  This is what gets transmitted. */
ComplexBuf generatePreambleTimeDomain(const OFDMParams& p,
                                      size_t short_reps = 10,
                                      size_t long_syms  = 2);

// =========================================================================
// OFDMModulator
// =========================================================================

class OFDMModulator {
public:
    explicit OFDMModulator(const OFDMParams& params);
    ~OFDMModulator();

    /** Modulate bits → complex baseband OFDM samples (including CP) */
    size_t modulateBits(const uint8_t* bits, size_t num_bits, ComplexBuf& out);

    /** Modulate pre-mapped symbols → complex baseband */
    size_t modulateSymbols(const ComplexSample* syms, size_t n, ComplexBuf& out);

    /** Generate complete preamble (time domain, ready for TX) */
    ComplexBuf generatePreamble();

    const OFDMParams&         params()     const { return p_; }
    const SubcarrierAllocation& allocation() const { return alloc_; }
    size_t dataSubcarriers()  const { return alloc_.dataCount(); }
    size_t bitsPerOFDMSymbol() const { return alloc_.dataCount() * bitsPerSymbol(p_.modulation); }
    size_t samplesPerSymbol()  const { return p_.symbolLength(); }

    void reset();

    /** Set an optional PAPR reducer (applied after data+pilot placement, before IFFT).
     *  Caller retains ownership — must outlive this modulator. */
    void setPAPRReducer(PAPRReducer* reducer) { papr_ = reducer; }
    PAPRReducer* paprReducer() const { return papr_; }

private:
    OFDMParams          p_;
    SubcarrierAllocation alloc_;
    std::unique_ptr<FFTEngine>    fft_;
    std::unique_ptr<SymbolMapper> mapper_;
    ComplexBuf freq_buf_;
    ComplexBuf time_buf_;
    uint32_t   sym_counter_;
    PAPRReducer* papr_ = nullptr;  ///< Optional, not owned
};

// =========================================================================
// OFDMDemodulator
// =========================================================================

class OFDMDemodulator {
public:
    explicit OFDMDemodulator(const OFDMParams& params);
    ~OFDMDemodulator();

    /** Demodulate one OFDM symbol → equalized data symbols */
    bool demodulate(const ComplexBuf& samples, ComplexBuf& data_symbols);

    /** Demodulate with soft LLR output */
    bool demodulateSoft(const ComplexBuf& samples, std::vector<float>& llrs,
                        float noise_variance);

    /** Decision-directed soft demodulation. Runs the standard demodulate
     *  pass to get pilot-based channel estimates + equalized data, then
     *  uses the hard-decoded constellation points at the DATA positions
     *  as additional reference symbols, refines `ch_est_` at the data
     *  bins (confidence-weighted by how close each equalized symbol
     *  sits to its nearest constellation point), re-equalizes, and
     *  emits the refined LLRs.
     *
     *  Typical benefit: ~0.2–0.4 dB at the modcod waterfall cliff for
     *  the cost of one additional bin-divide + soft-demap pass.
     *
     *  NOTE: the refinement is INTRA-symbol only. The data-bin writes do
     *  NOT accumulate across OFDM symbols — both the pilot LS path
     *  (estimateChannelFromPilots) and dftDenoiseChannel rewrite the data
     *  bins on the next symbol, so symbol s+1 does not inherit symbol s's
     *  data-corrected channel. (A persistent decision-directed estimate
     *  would require a separate accumulator — see SOTA roadmap.) */
    bool demodulateSoftDD(const ComplexBuf& samples,
                           std::vector<float>& llrs,
                           float noise_variance,
                           bool use_pwl);

    /** Process preamble for initial channel estimation.
     *  @param long_sym_samples  Two long preamble symbols (each fft_size+cp samples)
     */
    bool processPreamble(const ComplexBuf& long_sym_samples);

    float snrEstimate()    const { return snr_db_; }
    float noiseVariance()  const { return noise_var_; }
    const ComplexBuf& channelEstimate() const { return ch_est_; }
    const SubcarrierAllocation& allocation() const { return alloc_; }

    /** Enable MMSE channel estimation (replaces LS+linear interpolation) */
    void enableMMSE(const MMSEConfig& cfg = MMSEConfig());

    /** Disable MMSE (revert to LS+linear interpolation) */
    void disableMMSE();

    /** Check if MMSE is active */
    bool isMMSEEnabled() const { return mmse_ != nullptr; }

    // -----------------------------------------------------------------
    // Sync hardening — feature-flagged, always safe to leave off.
    // -----------------------------------------------------------------

    /** Enable integer-CFO estimation during preamble processing.
     *  NOTE (accuracy): the detected bin-shift is currently REPORTED ONLY
     *  (via lastIntegerCFO() / the stats CFO readout) — it is NOT yet
     *  applied to re-align the frequency-domain reference before
     *  equalization, so it does not actually correct an integer offset.
     *  (And nothing in the shipping engine calls enableIntegerCFO, so
     *  int_cfo_ is null in production.) To make it corrective, circularly
     *  shift fft_out_/the reference by the detected bins before the
     *  per-bin divide. Left reported-only for now. */
    void enableIntegerCFO(size_t max_bin_shift = 16);
    void disableIntegerCFO();

    /** Enable per-symbol residual phase PLL on equalized pilots. */
    void enablePhaseTracker(const PhaseTrackerConfig& cfg = PhaseTrackerConfig());
    void disablePhaseTracker();

    /** Enable sample-rate offset (clock drift) tracking via pilot phase slope. */
    void enableSROTracking(const SROConfig& cfg = SROConfig());
    void disableSROTracking();

    /** Last-detected integer CFO in subcarrier bins (signed). */
    int  lastIntegerCFO() const { return last_int_cfo_; }
    /** Estimated PPM clock offset from SRO tracker (0 if disabled). */
    float clockPpm() const;
    /** Current PLL phase accumulator (radians). */
    float trackedPhaseRad() const;

    void reset();

private:
    void estimateChannelFromPilots(const ComplexBuf& freq);
    void interpolateChannel();

    /** Denoise the channel estimate via a time-domain truncation: take
     *  IFFT of ch_est_, zero out taps past `max_delay_taps`, then FFT
     *  back. The real channel impulse response is short (≤ CP length)
     *  while noise spreads uniformly in time, so truncating outside the
     *  expected delay-spread window kills noise without distorting the
     *  channel. Typically buys 1–2 dB at moderate SNR for negligible
     *  compute cost. */
    void dftDenoiseChannel();

    OFDMParams           p_;
    SubcarrierAllocation alloc_;
    ComplexBuf           preamble_freq_ref_; // known frequency-domain preamble
    std::unique_ptr<FFTEngine>    fft_;
    std::unique_ptr<SymbolMapper> mapper_;

    // Channel state
    ComplexBuf ch_est_;
    SampleBuf  ch_mag_;
    float      snr_db_;
    float      noise_var_;
    // Per-pilot smoothed LS estimate, kept SEPARATE from ch_est_ so the
    // temporal IIR smooths against the previous pilot LS rather than the
    // DFT-denoised estimate (which overwrites ch_est_ each symbol). (#37)
    ComplexBuf pilot_ls_prev_;

    // Work buffers
    ComplexBuf fft_in_;
    ComplexBuf fft_out_;

    // MMSE estimator (optional, nullptr = use LS+linear)
    std::unique_ptr<MMSEChannelEstimator> mmse_;

    // Sync hardening (all optional, nullptr = bypass)
    std::unique_ptr<IntegerCFOEstimator> int_cfo_;
    std::unique_ptr<PhaseTracker>        phase_tracker_;
    std::unique_ptr<SROEstimator>        sro_;
    int   last_int_cfo_ = 0;
};

// =========================================================================
// OFDMSynchronizer
// =========================================================================

struct SyncResult {
    bool  valid          = false;
    int   timing_offset  = 0;
    float timing_metric  = 0.f;
    float freq_offset_hz = 0.f;
};

class OFDMSynchronizer {
public:
    explicit OFDMSynchronizer(const OFDMParams& params);
    ~OFDMSynchronizer();

    /** Schmidl-Cox coarse synchronization on CP correlation */
    bool coarseSync(const ComplexBuf& samples, SyncResult& result);

    /** Fine sync via long preamble cross-correlation */
    bool fineSync(const ComplexBuf& samples, int coarse_offset,
                  SyncResult& result);

    /** CP-based timing tracking (returns sample adjustment: -1, 0, +1) */
    int trackTiming(const ComplexBuf& samples);

    void reset();

private:
    OFDMParams p_;
    ComplexBuf short_preamble_td_; // time-domain short preamble for detection
    ComplexBuf long_preamble_td_;  // time-domain long preamble for fine sync
    float      threshold_;
    float      accum_timing_;
};

} // namespace dsca
