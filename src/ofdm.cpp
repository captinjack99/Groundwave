/**
 * @file ofdm.cpp
 * @brief OFDM Modulator / Demodulator / Synchronizer implementation
 *
 * KEY DESIGN DECISIONS (fixing v1 bugs):
 *
 * 1. FREQUENCY-DOMAIN PREAMBLE:
 *    The preamble is defined as known BPSK symbols in the frequency domain.
 *    TX: IFFT → add CP → transmit.
 *    RX: remove CP → FFT → divide by known freq symbols → channel estimate.
 *    This avoids the v1 bug where time-domain preamble was divided against
 *    frequency-domain FFT output.
 *
 * 2. SINGLE ALLOCATION FUNCTION:
 *    computeAllocation() is the sole source of truth for which FFT bins
 *    are data vs pilot vs guard vs DC-null. Both TX and RX call it with
 *    the same OFDMParams.
 *
 * 3. COMPLEX BASEBAND ONLY:
 *    No legacy real-only paths. Everything is ComplexBuf.
 */

#include "ofdm.hpp"
#include "papr_reducer.hpp"
#include <cmath>
#include <algorithm>
#include <numeric>
#include <stdexcept>

namespace dsca {

// =========================================================================
// PRNG for deterministic pilot/preamble sequences (must match TX & RX)
// =========================================================================

namespace {

constexpr uint32_t PILOT_SEED    = 0xABCDEF01u;
constexpr uint32_t PREAMBLE_SEED = 0x12345678u;
constexpr uint32_t SHORT_SEED    = 0x87654321u;

// LCG: same as glibc
inline uint32_t lcg(uint32_t s) { return s * 1103515245u + 12345u; }

// Generate a BPSK value from PRNG state
inline ComplexSample bpsk_from_seed(uint32_t seed, float amp) {
    float sign = (seed & 0x80000000u) ? 1.0f : -1.0f;
    return ComplexSample(sign * amp, 0.0f);
}

} // anonymous

// =========================================================================
// Subcarrier Allocation (SINGLE SOURCE OF TRUTH)
// =========================================================================

SubcarrierAllocation computeAllocation(const OFDMParams& p) {
    SubcarrierAllocation a;
    const size_t N = p.fft_size;

    a.guard_left  = p.guardLeft();
    a.guard_right = p.guardRight();

    const size_t start = a.guard_left;
    const size_t end   = N - a.guard_right; // exclusive
    const size_t dc_bin = 0;

    float pilot_amp = std::pow(10.0f, p.pilot_boost_db / 20.0f);
    uint32_t seed = PILOT_SEED;
    size_t carrier_count = 0;

    for (size_t i = start; i < end; ++i) {
        if (p.dc_null && i == dc_bin) continue;

        if (carrier_count % p.pilot_spacing == 0) {
            a.pilot_indices.push_back(i);
            seed = lcg(seed);
            a.pilot_values.push_back(bpsk_from_seed(seed, pilot_amp));
        } else {
            a.data_indices.push_back(i);
        }
        ++carrier_count;
    }

    return a;
}

// =========================================================================
// Preamble Generation — FREQUENCY DOMAIN FIRST
// =========================================================================

ComplexBuf generatePreambleFreqDomain(const OFDMParams& p) {
    // Known BPSK symbols on ALL active subcarriers (data + pilot positions)
    ComplexBuf freq(p.fft_size, ComplexSample(0.f, 0.f));

    const size_t start = p.guardLeft();
    const size_t end   = p.fft_size - p.guardRight();

    uint32_t seed = PREAMBLE_SEED;
    for (size_t i = start; i < end; ++i) {
        if (p.dc_null && i == 0) continue;
        seed = lcg(seed);
        float sign = (seed & 0x80000000u) ? 1.0f : -1.0f;
        freq[i] = ComplexSample(sign, 0.0f);
    }

    return freq;
}

ComplexBuf generatePreambleTimeDomain(const OFDMParams& p,
                                      size_t short_reps,
                                      size_t long_syms) {
    ComplexBuf preamble;

    const size_t N  = p.fft_size;
    const size_t cp = p.cpLength();

    // --- Short preamble: repeated short sequence for AGC/detection ---
    // Use every 4th subcarrier → time domain repeats with period N/4
    {
        ComplexBuf short_freq(N, ComplexSample(0.f, 0.f));
        const size_t start = p.guardLeft();
        const size_t end   = N - p.guardRight();
        uint32_t seed = SHORT_SEED;
        for (size_t i = start; i < end; i += 4) {
            if (p.dc_null && i == 0) continue;
            seed = lcg(seed);
            float sign = (seed & 0x80000000u) ? 1.0f : -1.0f;
            short_freq[i] = ComplexSample(sign * 1.5f, 0.0f);
        }

        // IFFT to time domain
        FFTEngine fft(N);
        ComplexBuf short_td;
        fft.inverse(short_freq, short_td);

        // Extract one period (N/4 samples) and repeat
        size_t period = N / 4;
        for (size_t rep = 0; rep < short_reps; ++rep) {
            for (size_t i = 0; i < period; ++i) {
                preamble.push_back(short_td[i]);
            }
        }
    }

    // --- Long preamble: full OFDM symbols for channel estimation ---
    {
        ComplexBuf long_freq = generatePreambleFreqDomain(p);

        FFTEngine fft(N);
        ComplexBuf long_td;
        fft.inverse(long_freq, long_td);

        for (size_t s = 0; s < long_syms; ++s) {
            // Add cyclic prefix (last cp samples of symbol)
            for (size_t i = N - cp; i < N; ++i)
                preamble.push_back(long_td[i]);
            // Full symbol
            preamble.insert(preamble.end(), long_td.begin(), long_td.end());
        }
    }

    return preamble;
}

// =========================================================================
// OFDMModulator
// =========================================================================

OFDMModulator::OFDMModulator(const OFDMParams& params)
    : p_(params), sym_counter_(0)
{
    if (!FFTEngine::isValidSize(p_.fft_size)) {
        throw std::invalid_argument("FFT size must be power of 2, >= 4");
    }

    alloc_  = computeAllocation(p_);
    fft_    = std::make_unique<FFTEngine>(p_.fft_size);
    mapper_ = std::make_unique<SymbolMapper>(p_.modulation);

    freq_buf_.resize(p_.fft_size);
    time_buf_.resize(p_.fft_size);
}

OFDMModulator::~OFDMModulator() = default;

size_t OFDMModulator::modulateSymbols(const ComplexSample* syms, size_t n,
                                       ComplexBuf& out) {
    const size_t data_per_sym = alloc_.dataCount();
    const size_t num_ofdm = (n + data_per_sym - 1) / data_per_sym;
    const size_t sym_len  = p_.symbolLength();
    const size_t cp       = p_.cpLength();

    // CP windowing — Tukey/Hann taper at the symbol leading edge (first
    // W of CP, ramp 0→1) and trailing edge (last W of body, ramp 1→0).
    // Output length is unchanged so the receiver's FFT-window slicing
    // logic stays untouched. IMPORTANT (see types.hpp cp_window_taper_pct):
    // the trailing taper sits INSIDE the samples the RX feeds to its FFT,
    // so it convolves the subcarrier spectrum (inter-carrier leakage) — a
    // diagonal per-bin equalizer canNOT invert it. That residual ICI is
    // why this is DEFAULT-OFF (taper 0) and only opted into when
    // spectral-mask compliance outweighs the marginal-SNR cost on the
    // densest modcods. A true overlap-add (WOLA) window would move the
    // taper outside the FFT span and avoid the ICI — see SOTA roadmap.
    float taper_pct = p_.cp_window_taper_pct;
    if (taper_pct < 0.f)  taper_pct = 0.f;
    if (taper_pct > 25.f) taper_pct = 25.f;
    size_t W = static_cast<size_t>(static_cast<float>(cp) *
                                    (taper_pct / 100.f) + 0.5f);
    if (W > cp) W = cp;          // never exceed the CP itself
    const bool windowing = (W >= 2 && cp >= 2);

    // Pre-compute Tukey rise window (length W); fall is its reverse.
    std::vector<float> w_rise;
    if (windowing) {
        w_rise.resize(W);
        const float pi = static_cast<float>(M_PI);
        const float invW = 1.f / static_cast<float>(W);
        for (size_t k = 0; k < W; ++k) {
            // Hann-shaped rise: w[0]=0, w[W-1] just below 1; symmetric
            // so the matched fall window applied at the symbol tail
            // produces a smooth transition through 0 to the next
            // symbol's ramp-up.
            w_rise[k] = 0.5f * (1.f - std::cos(pi *
                                  static_cast<float>(k) * invW));
        }
    }

    out.clear();
    out.reserve(num_ofdm * sym_len);

    size_t sym_off = 0;

    for (size_t oi = 0; oi < num_ofdm; ++oi) {
        // Clear frequency domain
        std::fill(freq_buf_.begin(), freq_buf_.end(), ComplexSample(0.f, 0.f));

        // Insert data symbols
        for (size_t i = 0; i < data_per_sym && sym_off < n; ++i) {
            freq_buf_[alloc_.data_indices[i]] = syms[sym_off++];
        }

        // Insert pilots (fixed BPSK, no per-symbol rotation)
        for (size_t i = 0; i < alloc_.pilotCount(); ++i) {
            freq_buf_[alloc_.pilot_indices[i]] = alloc_.pilot_values[i];
        }

        // PAPR reduction via tone reservation (modifies only reserved bins)
        if (papr_) {
            papr_->reduce(freq_buf_, *fft_);
        }

        // IFFT → time domain
        fft_->inverse(freq_buf_, time_buf_);

        // Add cyclic prefix + symbol
        size_t base = out.size();
        out.resize(base + sym_len);
        // CP: copy last cp samples
        std::copy(time_buf_.end() - static_cast<ptrdiff_t>(cp),
                  time_buf_.end(),
                  out.begin() + static_cast<ptrdiff_t>(base));
        // Symbol body
        std::copy(time_buf_.begin(), time_buf_.end(),
                  out.begin() + static_cast<ptrdiff_t>(base + cp));

        // Apply CP-windowing taper in place
        if (windowing) {
            for (size_t k = 0; k < W; ++k) {
                out[base + k] *= w_rise[k];                  // leading rise
                out[base + sym_len - 1 - k] *= w_rise[k];    // trailing fall (mirror)
            }
        }

        ++sym_counter_;
    }

    return out.size();
}

size_t OFDMModulator::modulateBits(const uint8_t* bits, size_t num_bits,
                                    ComplexBuf& out) {
    ComplexBuf symbols;
    mapper_->mapBytes(bits, num_bits, symbols);
    return modulateSymbols(symbols.data(), symbols.size(), out);
}

ComplexBuf OFDMModulator::generatePreamble() {
    return generatePreambleTimeDomain(p_, 10, 2);
}

void OFDMModulator::reset() { sym_counter_ = 0; }

// =========================================================================
// OFDMDemodulator
// =========================================================================

OFDMDemodulator::OFDMDemodulator(const OFDMParams& params)
    : p_(params), snr_db_(0.f), noise_var_(0.1f)
{
    alloc_ = computeAllocation(p_);
    fft_   = std::make_unique<FFTEngine>(p_.fft_size);
    mapper_ = std::make_unique<SymbolMapper>(p_.modulation);

    // Frequency-domain preamble reference (for channel estimation)
    preamble_freq_ref_ = generatePreambleFreqDomain(p_);

    fft_in_.resize(p_.fft_size);
    fft_out_.resize(p_.fft_size);
    ch_est_.resize(p_.fft_size, ComplexSample(1.f, 0.f));
    ch_mag_.resize(p_.fft_size, 1.f);
}

OFDMDemodulator::~OFDMDemodulator() = default;

bool OFDMDemodulator::processPreamble(const ComplexBuf& long_sym_samples) {
    const size_t sym_len = p_.symbolLength();
    const size_t cp      = p_.cpLength();
    const size_t N       = p_.fft_size;

    if (long_sym_samples.size() < 2 * sym_len) return false;

    // Process two long preamble symbols for averaged channel estimate
    ComplexBuf h1(N), h2(N);

    // First long symbol: strip CP, FFT, divide by known freq reference
    std::copy(long_sym_samples.begin() + static_cast<ptrdiff_t>(cp),
              long_sym_samples.begin() + static_cast<ptrdiff_t>(cp + N),
              fft_in_.begin());

    // Integer CFO estimation BEFORE FFT-based channel estimate.
    // We measure on the pre-FFT time-domain symbol; if a bin shift is
    // detected, we rotate the channel estimate accordingly.
    if (int_cfo_) {
        last_int_cfo_ = int_cfo_->estimate(fft_in_);
    }

    fft_->forward(fft_in_, fft_out_);

    for (size_t i = 0; i < N; ++i) {
        if (std::abs(preamble_freq_ref_[i]) > 1e-6f) {
            h1[i] = fft_out_[i] / preamble_freq_ref_[i];
        } else {
            h1[i] = ComplexSample(0.f, 0.f); // guard/DC — no estimate
        }
    }

    // Second long symbol
    std::copy(long_sym_samples.begin() + static_cast<ptrdiff_t>(sym_len + cp),
              long_sym_samples.begin() + static_cast<ptrdiff_t>(sym_len + cp + N),
              fft_in_.begin());
    fft_->forward(fft_in_, fft_out_);

    for (size_t i = 0; i < N; ++i) {
        if (std::abs(preamble_freq_ref_[i]) > 1e-6f) {
            h2[i] = fft_out_[i] / preamble_freq_ref_[i];
        } else {
            h2[i] = ComplexSample(0.f, 0.f);
        }
    }

    // Average + estimate SNR
    float sig_power = 0.f, noise_power = 0.f;
    for (size_t i = 0; i < N; ++i) {
        ch_est_[i] = (h1[i] + h2[i]) * 0.5f;
        ch_mag_[i] = std::abs(ch_est_[i]);
        sig_power += std::norm(ch_est_[i]);
        noise_power += std::norm(h1[i] - h2[i]) * 0.5f;
    }

    if (noise_power > 0.f) {
        snr_db_ = 10.f * std::log10(sig_power / noise_power);
        noise_var_ = noise_power / static_cast<float>(N);
    }

    // Initialize MMSE estimator from preamble if enabled
    if (mmse_) {
        mmse_->initFromPreamble(h1, h2);
    }

    return true;
}

bool OFDMDemodulator::demodulate(const ComplexBuf& samples,
                                  ComplexBuf& data_symbols) {
    const size_t sym_len = p_.symbolLength();
    const size_t cp      = p_.cpLength();
    const size_t N       = p_.fft_size;

    if (samples.size() < sym_len) return false;

    // Strip CP
    std::copy(samples.begin() + static_cast<ptrdiff_t>(cp),
              samples.begin() + static_cast<ptrdiff_t>(cp + N),
              fft_in_.begin());

    // FFT
    fft_->forward(fft_in_, fft_out_);

    // Channel estimation: MMSE or LS+linear
    if (mmse_) {
        mmse_->update(fft_out_);
        // Copy MMSE results to member state
        const auto& h = mmse_->estimate();
        const auto& m = mmse_->magnitude();
        for (size_t i = 0; i < N; ++i) {
            ch_est_[i] = h[i];
            ch_mag_[i] = m[i];
        }
        snr_db_    = mmse_->snrDB();
        noise_var_ = mmse_->noiseVariance();
    } else {
        // Legacy LS + linear interpolation
        estimateChannelFromPilots(fft_out_);
    }

    // Extract & equalize data subcarriers
    data_symbols.resize(alloc_.dataCount());
    for (size_t i = 0; i < alloc_.dataCount(); ++i) {
        size_t idx = alloc_.data_indices[i];
        if (ch_mag_[idx] > 1e-6f) {
            data_symbols[i] = fft_out_[idx] / ch_est_[idx];
        } else {
            data_symbols[i] = fft_out_[idx];
        }
    }

    // ---- Sync hardening: residual phase + sample-rate offset tracking ----
    if (phase_tracker_ || sro_) {
        // Build vectors of equalized pilots and references for the trackers
        ComplexBuf eq_pilots, ref_pilots;
        std::vector<size_t>  pilot_idx;
        std::vector<float>   pilot_phases;
        eq_pilots.reserve(alloc_.pilotCount());
        ref_pilots.reserve(alloc_.pilotCount());
        pilot_idx.reserve(alloc_.pilotCount());
        pilot_phases.reserve(alloc_.pilotCount());
        for (size_t i = 0; i < alloc_.pilotCount(); ++i) {
            size_t idx = alloc_.pilot_indices[i];
            ComplexSample y = (ch_mag_[idx] > 1e-6f)
                              ? (fft_out_[idx] / ch_est_[idx])
                              : fft_out_[idx];
            eq_pilots.push_back(y);
            ref_pilots.push_back(alloc_.pilot_values[i]);
            pilot_idx.push_back(idx);
            // Phase of LS pilot estimate (pre-Wiener) for SRO regression
            ComplexSample h_ls = (std::abs(alloc_.pilot_values[i]) > 1e-9f)
                                 ? fft_out_[idx] / alloc_.pilot_values[i]
                                 : ComplexSample(0.f, 0.f);
            pilot_phases.push_back(std::arg(h_ls));
        }

        if (phase_tracker_) {
            float err = phase_tracker_->detectPhase(eq_pilots, ref_pilots);
            phase_tracker_->update(err);
            phase_tracker_->apply(data_symbols.data(), data_symbols.size());
        }
        if (sro_) {
            sro_->estimateSlope(pilot_idx, pilot_phases);
        }
    }

    return true;
}

void OFDMDemodulator::enableMMSE(const MMSEConfig& cfg) {
    mmse_ = std::make_unique<MMSEChannelEstimator>(
        p_.fft_size,
        alloc_.pilot_indices,
        alloc_.data_indices,
        alloc_.pilot_values,
        p_.sample_rate,
        cfg);
}

void OFDMDemodulator::disableMMSE() {
    mmse_.reset();
}

void OFDMDemodulator::enableIntegerCFO(size_t max_bin_shift) {
    int_cfo_ = std::make_unique<IntegerCFOEstimator>(
        p_.fft_size, preamble_freq_ref_, max_bin_shift);
}

void OFDMDemodulator::disableIntegerCFO() {
    int_cfo_.reset();
    last_int_cfo_ = 0;
}

void OFDMDemodulator::enablePhaseTracker(const PhaseTrackerConfig& cfg) {
    PhaseTrackerConfig c = cfg;
    if (c.symbol_rate <= 0.f) {
        c.symbol_rate = static_cast<float>(p_.sample_rate)
                      / static_cast<float>(p_.symbolLength());
    }
    phase_tracker_ = std::make_unique<PhaseTracker>(c);
}

void OFDMDemodulator::disablePhaseTracker() {
    phase_tracker_.reset();
}

void OFDMDemodulator::enableSROTracking(const SROConfig& cfg) {
    sro_ = std::make_unique<SROEstimator>(cfg);
}

void OFDMDemodulator::disableSROTracking() {
    sro_.reset();
}

float OFDMDemodulator::clockPpm() const {
    return sro_ ? sro_->slopePpm(p_.fft_size) : 0.f;
}

float OFDMDemodulator::trackedPhaseRad() const {
    return phase_tracker_ ? phase_tracker_->phaseRad() : 0.f;
}

bool OFDMDemodulator::demodulateSoft(const ComplexBuf& samples,
                                      std::vector<float>& llrs,
                                      float noise_variance) {
    ComplexBuf data_syms;
    if (!demodulate(samples, data_syms)) return false;
    mapper_->demapSoft(data_syms, noise_variance, llrs);
    return true;
}

bool OFDMDemodulator::demodulateSoftDD(const ComplexBuf& samples,
                                        std::vector<float>& llrs,
                                        float noise_variance,
                                        bool use_pwl) {
    ComplexBuf data_syms;
    if (!demodulate(samples, data_syms)) return false;

    // Build a vector of hard-decided constellation references for each
    // data subcarrier — these are our "virtual pilots". Track the
    // confidence per symbol as 1 / (1 + |error|²): well-decided symbols
    // get ~1, ambiguous symbols get ≪ 1 and contribute little to the
    // refinement.
    const auto& const_pts = mapper_->constellation();
    if (const_pts.empty()) {
        // BPSK with empty constellation array — fall back to plain LLR.
        if (use_pwl) mapper_->demapSoftPWL(data_syms, noise_variance, llrs);
        else         mapper_->demapSoft   (data_syms, noise_variance, llrs);
        return true;
    }

    const size_t Nd = alloc_.dataCount();
    std::vector<ComplexSample> refs(Nd);
    std::vector<float> conf(Nd, 0.f);
    for (size_t i = 0; i < Nd && i < data_syms.size(); ++i) {
        uint16_t idx = mapper_->demapHard(data_syms[i]);
        if (idx >= const_pts.size()) idx = 0;
        refs[i] = const_pts[idx];
        float err2 = std::norm(data_syms[i] - refs[i]);
        // Half-life around |err|² = noise_variance: confident when the
        // residual matches the noise, untrustworthy otherwise.
        conf[i] = 1.f / (1.f + err2 / std::max(noise_variance, 1e-6f));
    }

    // Phase-frame consistency (#36): demodulate() already rotated data_syms
    // (hence refs) by e^{-jφ} via the phase tracker. The raw fft_out_ is NOT
    // in that frame, so forming Y/X̂ from raw fft_out_ would bake φ into the
    // refined channel estimate, and re-applying the tracker afterwards
    // double-counted it. Put fft_out_ into the SAME φ-corrected frame here
    // (one rotation), and DO NOT re-apply the tracker below.
    ComplexSample derot(1.f, 0.f);
    if (phase_tracker_)
        derot = std::polar(1.f, -phase_tracker_->phaseRad());

    // Refine ch_est_ at each data subcarrier by exponentially blending
    // the LS estimate Y/X̂ (in the φ-corrected frame) with the pilot estimate.
    // The blend weight scales with confidence — at conf=1 we accept
    // 30 % of the new LS measurement; at conf→0 we leave the pilot
    // estimate unchanged.
    constexpr float MAX_BLEND = 0.30f;
    for (size_t i = 0; i < Nd; ++i) {
        size_t bin = alloc_.data_indices[i];
        if (std::abs(refs[i]) < 1e-6f) continue;
        ComplexSample h_ls = (fft_out_[bin] * derot) / refs[i];
        float w = MAX_BLEND * conf[i];
        ch_est_[bin] = (1.f - w) * ch_est_[bin] + w * h_ls;
        ch_mag_[bin] = std::abs(ch_est_[bin]);
    }

    // Re-equalize the data subcarriers with the refined channel, in the same
    // φ-corrected frame. The phase tracker is already folded into `derot`, so
    // it is NOT re-applied (that was the double-correction).
    ComplexBuf refined(Nd);
    for (size_t i = 0; i < Nd; ++i) {
        size_t bin = alloc_.data_indices[i];
        if (ch_mag_[bin] > 1e-6f) {
            refined[i] = (fft_out_[bin] * derot) / ch_est_[bin];
        } else {
            refined[i] = data_syms[i];
        }
    }

    if (use_pwl) mapper_->demapSoftPWL(refined, noise_variance, llrs);
    else         mapper_->demapSoft   (refined, noise_variance, llrs);
    return true;
}

void OFDMDemodulator::estimateChannelFromPilots(const ComplexBuf& freq) {
    if (pilot_ls_prev_.size() != alloc_.pilotCount())
        pilot_ls_prev_.assign(alloc_.pilotCount(), ComplexSample(1.f, 0.f));

    // LS estimation at pilot positions
    for (size_t i = 0; i < alloc_.pilotCount(); ++i) {
        size_t idx = alloc_.pilot_indices[i];
        ComplexSample ref = alloc_.pilot_values[i];
        if (std::abs(ref) > 1e-6f) {
            ComplexSample h_ls = freq[idx] / ref;
            // Temporal IIR on the pilot LS. The smoothing memory lives in
            // pilot_ls_prev_, NOT ch_est_ — the latter is overwritten every
            // symbol by dftDenoiseChannel() below, so feeding it back made the
            // IIR smooth against the denoised output instead of the previous
            // pilot measurement (fine on a static channel, unpredictable on a
            // time-varying one). (#37)
            pilot_ls_prev_[i] = 0.7f * h_ls + 0.3f * pilot_ls_prev_[i];
            ch_est_[idx] = pilot_ls_prev_[i];
            ch_mag_[idx] = std::abs(ch_est_[idx]);
        }
    }
    interpolateChannel();
}

void OFDMDemodulator::interpolateChannel() {
    // Linear interpolation between pilot positions
    auto& pi = alloc_.pilot_indices;
    if (pi.size() < 2) return;

    for (size_t p = 0; p < pi.size() - 1; ++p) {
        size_t i1 = pi[p], i2 = pi[p + 1];
        ComplexSample h1 = ch_est_[i1], h2 = ch_est_[i2];
        for (size_t i = i1 + 1; i < i2; ++i) {
            float alpha = static_cast<float>(i - i1) / static_cast<float>(i2 - i1);
            ch_est_[i] = h1 * (1.f - alpha) + h2 * alpha;
            ch_mag_[i] = std::abs(ch_est_[i]);
        }
    }

    // Extrapolate edges
    if (pi.size() >= 2) {
        // Before first pilot
        ComplexSample slope = (ch_est_[pi[1]] - ch_est_[pi[0]]) /
                              static_cast<float>(pi[1] - pi[0]);
        for (size_t i = 0; i < pi[0]; ++i) {
            ch_est_[i] = ch_est_[pi[0]] - slope * static_cast<float>(pi[0] - i);
            ch_mag_[i] = std::abs(ch_est_[i]);
        }
        // After last pilot
        size_t L = pi.size();
        slope = (ch_est_[pi[L-1]] - ch_est_[pi[L-2]]) /
                static_cast<float>(pi[L-1] - pi[L-2]);
        for (size_t i = pi[L-1] + 1; i < p_.fft_size; ++i) {
            ch_est_[i] = ch_est_[pi[L-1]] + slope * static_cast<float>(i - pi[L-1]);
            ch_mag_[i] = std::abs(ch_est_[i]);
        }
    }

    // DFT-based denoise: after pilot LS + linear interpolation, truncate
    // the time-domain impulse response to the expected delay-spread
    // window. This kills noise that spread across the full IFFT support
    // while the real channel impulse response is concentrated in the
    // first CP-length taps.
    dftDenoiseChannel();
}

void OFDMDemodulator::dftDenoiseChannel() {
    if (!fft_ || ch_est_.size() != p_.fft_size) return;
    // Expected delay spread: CP length is the upper bound (the OFDM
    // system can only tolerate channels with delay spread ≤ CP). Add a
    // small slack (×1.25) to accommodate sync timing offset.
    size_t cp_len = static_cast<size_t>(p_.fft_size *
                                         cpRatio(p_.cyclic_prefix));
    size_t max_taps = static_cast<size_t>(cp_len * 5 / 4);
    if (max_taps < 4)              max_taps = 4;
    if (max_taps >= p_.fft_size/2) return;  // delay spread too large; no benefit

    // IFFT channel estimate → channel impulse response in time domain.
    ComplexBuf cir(p_.fft_size);
    fft_->inverse(ch_est_, cir);

    // Zero out taps outside the [0, max_taps) ∪ [N-max_taps, N) window.
    // (Negative-time taps wrap to the end of the IFFT output.)
    for (size_t i = max_taps; i < p_.fft_size - max_taps; ++i) {
        cir[i] = ComplexSample(0.f, 0.f);
    }

    // FFT back to frequency domain. The denoised channel estimate has
    // the same energy at active subcarriers (real channel taps survive)
    // with the noise floor suppressed across all bins.
    fft_->forward(cir, ch_est_);
    for (size_t i = 0; i < p_.fft_size; ++i) {
        ch_mag_[i] = std::abs(ch_est_[i]);
    }
}

void OFDMDemodulator::reset() {
    std::fill(ch_est_.begin(), ch_est_.end(), ComplexSample(1.f, 0.f));
    std::fill(ch_mag_.begin(), ch_mag_.end(), 1.f);
    std::fill(pilot_ls_prev_.begin(), pilot_ls_prev_.end(), ComplexSample(1.f, 0.f));
    snr_db_ = 0.f;
    noise_var_ = 0.1f;
    if (mmse_) mmse_->reset();
    if (phase_tracker_) phase_tracker_->reset();
    if (sro_)           sro_->reset();
    last_int_cfo_ = 0;
}

// =========================================================================
// OFDMSynchronizer
// =========================================================================

OFDMSynchronizer::OFDMSynchronizer(const OFDMParams& params)
    : p_(params), threshold_(0.25f), accum_timing_(0.f)
{
    // Generate time-domain preambles for correlation
    auto preamble = generatePreambleTimeDomain(p_, 1, 1);

    // Short preamble: first N/4 samples
    size_t short_len = p_.fft_size / 4;
    short_preamble_td_.assign(preamble.begin(),
                               preamble.begin() + static_cast<ptrdiff_t>(short_len));

    // Long preamble: after short preamble, skip CP, take N samples
    size_t long_start = short_len + p_.cpLength();
    if (preamble.size() >= long_start + p_.fft_size) {
        long_preamble_td_.assign(
            preamble.begin() + static_cast<ptrdiff_t>(long_start),
            preamble.begin() + static_cast<ptrdiff_t>(long_start + p_.fft_size));
    }
}

OFDMSynchronizer::~OFDMSynchronizer() = default;

bool OFDMSynchronizer::coarseSync(const ComplexBuf& samples, SyncResult& result) {
    result = SyncResult{};

    size_t N  = p_.fft_size;
    size_t cp = p_.cpLength();

    if (samples.size() < 2 * (N + cp)) return false;

    // Schmidl-Cox: correlate CP region with end-of-symbol
    float best_metric = 0.f;
    size_t best_offset = 0;
    std::complex<float> best_corr(0.f, 0.f);

    for (size_t d = 0; d < N; ++d) {
        ComplexSample P(0.f, 0.f);
        float R = 0.f;
        for (size_t i = 0; i < cp; ++i) {
            P += samples[d + i] * std::conj(samples[d + i + N]);
            R += std::norm(samples[d + i + N]);
        }
        float metric = std::norm(P) / (R * R + 1e-10f);
        if (metric > best_metric) {
            best_metric = metric;
            best_offset = d;
            best_corr = P;
        }
    }

    if (best_metric > threshold_) {
        result.valid = true;
        result.timing_offset = static_cast<int>(best_offset);
        result.timing_metric = best_metric;
        float phase = std::arg(best_corr);
        result.freq_offset_hz = -phase * static_cast<float>(p_.sample_rate)
                                / (2.0f * static_cast<float>(M_PI) * static_cast<float>(N));
    }

    return result.valid;
}

bool OFDMSynchronizer::fineSync(const ComplexBuf& samples, int coarse_offset,
                                 SyncResult& result) {
    result = SyncResult{};

    size_t N = p_.fft_size;
    size_t range = N / 8;

    if (long_preamble_td_.empty() || long_preamble_td_.size() != N) return false;

    float preamble_energy = 0.f;
    for (auto& s : long_preamble_td_) preamble_energy += std::norm(s);

    float best_corr = 0.f;
    int best_off = coarse_offset;

    size_t lo = static_cast<size_t>(std::max(0, coarse_offset - static_cast<int>(range)));
    size_t hi = std::min(samples.size() - N,
                         static_cast<size_t>(coarse_offset + range));

    for (size_t d = lo; d < hi; ++d) {
        ComplexSample c(0.f, 0.f);
        float local_e = 0.f;
        for (size_t i = 0; i < N; ++i) {
            c += samples[d + i] * std::conj(long_preamble_td_[i]);
            local_e += std::norm(samples[d + i]);
        }
        float norm_f = std::sqrt(local_e * preamble_energy);
        float normalized = (norm_f > 1e-6f) ? std::abs(c) / norm_f : 0.f;

        if (normalized > best_corr) {
            best_corr = normalized;
            best_off = static_cast<int>(d);
        }
    }

    result.valid = (best_corr > threshold_);
    result.timing_offset = best_off;
    result.timing_metric = best_corr;
    return result.valid;
}

int OFDMSynchronizer::trackTiming(const ComplexBuf& samples) {
    size_t N  = p_.fft_size;
    size_t cp = p_.cpLength();

    if (samples.size() < N + cp + 2) return 0;

    auto cp_corr = [&](int offset) -> float {
        ComplexSample c(0.f, 0.f);
        for (size_t i = 0; i < cp; ++i) {
            // Signed index: offset=-1 with i=0 would underflow to SIZE_MAX,
            // and the old `ni + N >= size` guard wrapped and failed to
            // catch it → samples[SIZE_MAX] OOB read. Bound both ends in
            // signed arithmetic. (#35)
            long ni = static_cast<long>(i) + offset;
            if (ni < 0) continue;
            size_t nu = static_cast<size_t>(ni);
            if (nu + N >= samples.size()) break;
            c += samples[nu] * std::conj(samples[nu + N]);
        }
        return std::abs(c);
    };

    float early = cp_corr(-1);
    float late  = cp_corr(+1);
    float on    = cp_corr(0);
    float denom = early + on + late + 1e-6f;
    float ted   = (late - early) / denom;

    accum_timing_ += ted * 0.05f;
    if (accum_timing_ >  0.5f) { accum_timing_ -= 1.0f; return  1; }
    if (accum_timing_ < -0.5f) { accum_timing_ += 1.0f; return -1; }
    return 0;
}

void OFDMSynchronizer::reset() {
    accum_timing_ = 0.f;
}

} // namespace dsca
