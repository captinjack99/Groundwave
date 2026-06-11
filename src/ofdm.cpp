/**
 * @file ofdm.cpp
 * @brief OFDM Modulator / Demodulator / Synchronizer implementation
 *
 * KEY DESIGN DECISIONS (fixing v1 bugs):
 *
 * 1. FREQUENCY-DOMAIN PREAMBLE:
 *    The preamble is defined as known symbols in the frequency domain — a
 *    constant-amplitude Zadoff-Chu (CAZAC) sequence on the long preamble's
 *    active subcarriers (BPSK on the short preamble).
 *    TX: IFFT → add CP → transmit.
 *    RX: remove CP → FFT → divide by known freq symbols → channel estimate.
 *    This avoids the v1 bug where time-domain preamble was divided against
 *    frequency-domain FFT output. ZC's |X[k]|=1 keeps the Y/X channel-estimate
 *    divides well-conditioned and gives the time-domain long preamble a sharp
 *    autocorrelation peak for fine timing acquisition.
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
#include "zadoff_chu.hpp"
#include <cmath>
#include <algorithm>
#include <numeric>
#include <stdexcept>
#include <utility>

namespace gw {

// =========================================================================
// PRNG for deterministic pilot/preamble sequences (must match TX & RX)
// =========================================================================

namespace {

constexpr uint32_t PILOT_SEED    = 0xABCDEF01u;
// PREAMBLE_SEED retired: the long preamble's active-bin values are now a
// Zadoff-Chu (CAZAC) sequence (see generatePreambleFreqDomain), not LCG-BPSK.
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

    // The logical carrier index l walks the OCCUPIED BAND edge-to-edge in
    // ascending frequency, with DC at l = N/2. Its physical IFFT bin is
    //     phys = (l + N/2) mod N
    // (FFT natural order: DC = bin 0, +f in [1, N/2), -f in (N/2, N)).
    // Guards at the logical edges [0,start) and [end,N) therefore sit at
    // the PHYSICAL band edges (around ±Nyquist) — which is what the
    // guard / target_bw_hz math in OFDMParams promises, and what the
    // analog passband model (TX/RX LPF cutoffs at signal_bw/2, real-IF
    // up/downconversion at a center frequency) requires.
    //
    // Previously l was used as the physical bin DIRECTLY, which inverted
    // the layout: the guard hole sat uselessly around DC and the active
    // carriers ran out to ±Nyquist, so every passband path (HW audio,
    // vcable, the CLI's internal real-IF loopback) clipped most of the
    // signal in the LPFs and nothing decoded. The complex-baseband
    // loopback is bin-exact regardless of layout, which is why the GUI
    // and the test suite never noticed. This mapping also makes dc_null
    // real: bin 0 (the downconverter's DC/carrier-leak bin) now actually
    // carries no data or pilot.
    auto physBin = [N](size_t logical) { return (logical + N / 2) % N; };

    float pilot_amp = std::pow(10.0f, p.pilot_boost_db / 20.0f);
    uint32_t seed = PILOT_SEED;
    size_t carrier_count = 0;

    // Pilot values are tied to their bins; build (bin, value) pairs in
    // logical order (so the LCG sequence stays deterministic), then sort
    // by physical bin so downstream consumers that assume ascending
    // indices (pilot interpolation, PAPR carve "sorted ascending"
    // invariant) keep working.
    std::vector<std::pair<size_t, ComplexSample>> pilots;
    for (size_t i = start; i < end; ++i) {
        const size_t phys = physBin(i);
        if (p.dc_null && phys == 0) continue;

        if (carrier_count % p.pilot_spacing == 0) {
            seed = lcg(seed);
            pilots.emplace_back(phys, bpsk_from_seed(seed, pilot_amp));
        } else {
            a.data_indices.push_back(phys);
        }
        ++carrier_count;
    }
    std::sort(pilots.begin(), pilots.end(),
              [](const std::pair<size_t, ComplexSample>& x,
                 const std::pair<size_t, ComplexSample>& y) {
                  return x.first < y.first;
              });
    std::sort(a.data_indices.begin(), a.data_indices.end());
    a.pilot_indices.reserve(pilots.size());
    a.pilot_values.reserve(pilots.size());
    for (const auto& pv : pilots) {
        a.pilot_indices.push_back(pv.first);
        a.pilot_values.push_back(pv.second);
    }

    // PAPR tone reservation: carve uniformly-spaced DATA carriers out of the
    // data allocation into dedicated reserved (peak-reduction) tones. Done HERE
    // -- the single source of truth -- so TX and RX agree and neither maps nor
    // demaps data on reserved bins. Off (no carve) when papr_reserve_fraction
    // is 0, so the default allocation is byte-identical to before.
    if (p.papr_reserve_fraction > 0.f && a.data_indices.size() >= 2) {
        size_t total_active = a.data_indices.size() + a.pilot_indices.size();
        size_t n_reserve = static_cast<size_t>(std::ceil(
            p.papr_reserve_fraction * static_cast<float>(total_active)));
        n_reserve = std::min(n_reserve, a.data_indices.size() / 2);
        if (n_reserve == 0) n_reserve = 1;

        std::vector<bool> is_res(a.data_indices.size(), false);
        float step = static_cast<float>(a.data_indices.size()) /
                     static_cast<float>(n_reserve);
        for (size_t i = 0; i < n_reserve; ++i) {
            size_t di = static_cast<size_t>(static_cast<float>(i) * step + 0.5f);
            if (di >= a.data_indices.size()) di = a.data_indices.size() - 1;
            is_res[di] = true;
        }
        std::vector<size_t> keep;
        keep.reserve(a.data_indices.size());
        for (size_t di = 0; di < a.data_indices.size(); ++di) {
            if (is_res[di]) a.reserved_indices.push_back(a.data_indices[di]);
            else            keep.push_back(a.data_indices[di]);
        }
        a.data_indices.swap(keep);   // reserved/data both remain sorted ascending
    }

    return a;
}

// =========================================================================
// Preamble Generation — FREQUENCY DOMAIN FIRST
// =========================================================================

ComplexBuf generatePreambleFreqDomain(const OFDMParams& p) {
    // Known CAZAC (Zadoff-Chu) symbols on ALL active subcarriers (data + pilot
    // positions). ZC is constant-amplitude (|X[k]|=1) so channel estimation
    // (Y/X divides) and the long-preamble CFO estimator stay valid, while its
    // near-ideal periodic autocorrelation gives the time-domain long preamble a
    // sharp correlation peak for honest fineSync acquisition. TX generation and
    // RX reference both come from THIS function, so they stay bit-identical.
    ComplexBuf freq(p.fft_size, ComplexSample(0.f, 0.f));

    const size_t N     = p.fft_size;
    const size_t start = p.guardLeft();
    const size_t end   = N - p.guardRight();

    // Same logical→physical bin mapping as computeAllocation (band centered
    // on DC, guards at the physical band edges) so the preamble occupies
    // EXACTLY the band the data does and survives the same LPFs.
    auto physBin = [N](size_t logical) { return (logical + N / 2) % N; };

    // Count active bins (DC nulled when requested) so the ZC length matches
    // the number of bins we actually populate.
    size_t n_active = 0;
    for (size_t i = start; i < end; ++i) {
        if (p.dc_null && physBin(i) == 0) continue;
        ++n_active;
    }
    if (n_active == 0) return freq;

    // One length-n_active ZC sequence, optimal root (coprime to length, near
    // n_active/2 → best off-peak autocorrelation suppression). Mapped in
    // ascending-frequency (logical) order onto the active bins, so the ZC
    // phase ramp stays contiguous along the occupied band.
    const ComplexBuf zc = zadoffChu(n_active, zadoffChuOptimalRoot(n_active));

    size_t k = 0;
    for (size_t i = start; i < end; ++i) {
        const size_t phys = physBin(i);
        if (p.dc_null && phys == 0) continue;
        freq[phys] = zc[k++];
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
        // Same logical→physical mapping as computeAllocation: the comb
        // lives inside the occupied band (centered on DC), not out to
        // ±Nyquist, so AGC/detection sees the band the data will use.
        for (size_t i = start; i < end; i += 4) {
            const size_t phys = (i + N / 2) % N;
            if (p.dc_null && phys == 0) continue;
            seed = lcg(seed);
            float sign = (seed & 0x80000000u) ? 1.0f : -1.0f;
            short_freq[phys] = ComplexSample(sign * 1.5f, 0.0f);
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
    dd_persist_.resize(p_.fft_size, ComplexSample(1.f, 0.f));
    dd_conf_.resize(p_.fft_size, 0.f);
}

OFDMDemodulator::~OFDMDemodulator() = default;

bool OFDMDemodulator::processPreamble(const ComplexBuf& long_sym_samples) {
    const size_t sym_len = p_.symbolLength();
    const size_t cp      = p_.cpLength();
    const size_t N       = p_.fft_size;

    if (long_sym_samples.size() < 2 * sym_len) return false;

    // ---- Fractional CFO estimate (Moose) ----------------------------------
    // The two long preamble symbols are identical and sym_len samples apart.
    // With a carrier offset f, corresponding samples differ only by the
    // rotation e^{j·2π·f·sym_len/fs}, so the phase of their cross-correlation
    // IS the offset. Unambiguous over |f| < fs/(2·sym_len) (~83 Hz at
    // 48 kHz / N=256). This is the offset the engine previously discarded.
    {
        ComplexSample corr(0.f, 0.f);
        for (size_t i = 0; i < N; ++i) {
            corr += std::conj(long_sym_samples[cp + i]) *
                    long_sym_samples[sym_len + cp + i];
        }
        // arg(corr) = f·sym_len (rad) → per-sample offset = arg / sym_len.
        cfo_rad_per_sample_ = (std::norm(corr) > 0.f)
            ? std::arg(corr) / static_cast<float>(sym_len) : 0.f;

        // Dead-zone: below ~1% of the subcarrier spacing the estimate is
        // dominated by noise (a few tenths of a Hz at typical SNR), and the
        // per-symbol pilot tracker already absorbs that much. "Correcting" it
        // would only inject jitter, so snap it to zero — which also keeps a
        // genuinely CFO-free stream byte-for-byte unchanged.
        const float eps_norm = std::fabs(cfo_rad_per_sample_) *
                               static_cast<float>(N) / 6.283185307179586f;
        if (eps_norm < 0.008f) cfo_rad_per_sample_ = 0.f;
    }

    // Derotation referenced to long_sym_samples[0]. The payload stream
    // continues the SAME NCO from sample index 2·sym_len (rx_sample_pos_,
    // set below), so the channel estimate and the data live in one CFO-free
    // frame and the pilot IIR no longer chases a spinning phase.
    auto derot = [&](size_t k) -> ComplexSample {
        if (cfo_rad_per_sample_ == 0.f) return ComplexSample(1.f, 0.f);
        double ph = std::fmod(-static_cast<double>(cfo_rad_per_sample_) *
                              static_cast<double>(k), 2.0 * M_PI);
        return ComplexSample(static_cast<float>(std::cos(ph)),
                             static_cast<float>(std::sin(ph)));
    };

    // Process two long preamble symbols for averaged channel estimate
    ComplexBuf h1(N), h2(N);

    // First long symbol: strip CP + derotate, FFT, divide by known freq ref
    for (size_t i = 0; i < N; ++i)
        fft_in_[i] = long_sym_samples[cp + i] * derot(cp + i);

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

    // Second long symbol: strip CP + derotate (continuing the same NCO)
    for (size_t i = 0; i < N; ++i)
        fft_in_[i] = long_sym_samples[sym_len + cp + i] * derot(sym_len + cp + i);
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

    // When CFO correction is active, referencing the derotation at the long-
    // preamble start leaves a CONSTANT phase offset on the equalized data (the
    // carrier phase accumulated before the long preamble). It cancels in
    // equalization, but the per-symbol pilot IIR starts from (1,0) and would
    // take several symbols to converge to it — corrupting the first data
    // symbols of high-order QAM (the error count tracks that residual phase).
    // Seed the IIR from the preamble estimate so the constant is absorbed from
    // symbol 0. No-op when CFO is inactive, so CFO-free streams are untouched.
    if (cfo_rad_per_sample_ != 0.f) {
        if (pilot_ls_prev_.size() != alloc_.pilotCount())
            pilot_ls_prev_.assign(alloc_.pilotCount(), ComplexSample(1.f, 0.f));
        for (size_t i = 0; i < alloc_.pilotCount(); ++i)
            pilot_ls_prev_[i] = ch_est_[alloc_.pilot_indices[i]];
    }

    if (noise_power > 0.f) {
        snr_db_ = 10.f * std::log10(sig_power / noise_power);
        noise_var_ = noise_power / static_cast<float>(N);
    }

    // Seed the persistent decision-directed accumulator from the (clean,
    // CFO-corrected) preamble channel estimate so it starts coherent and in the
    // same frame as the data that follows. Confidence resets to 0 so the first
    // data symbols lean on the pilot estimate until the DD accumulator has seen
    // enough consistent decisions to be trusted. Inert unless persistent_dd_.
    if (persistent_dd_) {
        if (dd_persist_.size() != N) dd_persist_.assign(N, ComplexSample(1.f, 0.f));
        if (dd_conf_.size()    != N) dd_conf_.assign(N, 0.f);
        for (size_t i = 0; i < N; ++i) {
            dd_persist_[i] = ch_est_[i];
            dd_conf_[i]    = 0.f;
        }
        dd_seeded_ = true;
    }

    // Initialize MMSE estimator from preamble if enabled
    if (mmse_) {
        mmse_->initFromPreamble(h1, h2);
    }

    // The payload that follows starts right after the two long symbols. Keep
    // the derotation NCO continuous so the first data symbol picks up exactly
    // where the preamble left off.
    rx_sample_pos_ = 2 * sym_len;

    return true;
}

void OFDMDemodulator::stripAndDerotate(const ComplexBuf& samples) {
    const size_t sym_len = p_.symbolLength();
    const size_t cp      = p_.cpLength();
    const size_t N       = p_.fft_size;

    if (cfo_rad_per_sample_ == 0.f) {
        // No CFO estimated → plain CP strip, byte-for-byte identical to before.
        std::copy(samples.begin() + static_cast<ptrdiff_t>(cp),
                  samples.begin() + static_cast<ptrdiff_t>(cp + N),
                  fft_in_.begin());
        rx_sample_pos_ += sym_len;
        return;
    }

    // Derotate each kept sample by e^{-j·cfo·k}, where k is the ABSOLUTE sample
    // index since the preamble. Using the absolute index (rather than a
    // per-sample accumulator) keeps the payload phase-locked to the preamble's
    // channel estimate and immune to drift over long streams; fmod keeps the
    // cos/sin argument small for float accuracy.
    const double cfo = static_cast<double>(cfo_rad_per_sample_);
    for (size_t n = 0; n < N; ++n) {
        double k  = static_cast<double>(rx_sample_pos_ + cp + n);
        double ph = std::fmod(-cfo * k, 2.0 * M_PI);
        fft_in_[n] = samples[cp + n] *
            ComplexSample(static_cast<float>(std::cos(ph)),
                          static_cast<float>(std::sin(ph)));
    }
    rx_sample_pos_ += sym_len;
}

bool OFDMDemodulator::demodulate(const ComplexBuf& samples,
                                  ComplexBuf& data_symbols) {
    const size_t sym_len = p_.symbolLength();
    const size_t N       = p_.fft_size;

    if (samples.size() < sym_len) return false;

    // Strip CP + apply fractional-CFO derotation (a plain CP strip when no
    // CFO has been estimated, so a CFO-free stream is unchanged).
    stripAndDerotate(samples);

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

void OFDMDemodulator::buildDataBinNoise(float noise_variance, size_t n,
                                        std::vector<float>& nv) const {
    nv.resize(n);
    if (!per_bin_llr_) {                 // legacy: one scalar for all bins
        std::fill(nv.begin(), nv.end(), noise_variance);
        return;
    }
    for (size_t i = 0; i < n; ++i) {
        size_t idx = (i < alloc_.data_indices.size()) ? alloc_.data_indices[i] : 0;
        float h2 = (idx < ch_mag_.size()) ? ch_mag_[idx] * ch_mag_[idx] : 1.f;
        // Floor |H|^2 so a deep fade doesn't blow the noise to infinity (the
        // resulting LLRs would be ~0, which is the intended "no information"
        // outcome, but we keep it finite and bounded).
        if (h2 < 1e-3f) h2 = 1e-3f;
        nv[i] = noise_variance / h2;
    }
}

bool OFDMDemodulator::demodulateSoft(const ComplexBuf& samples,
                                      std::vector<float>& llrs,
                                      float noise_variance, bool use_pwl,
                                      ComplexBuf* eq_out) {
    ComplexBuf data_syms;
    if (!demodulate(samples, data_syms)) return false;
    if (eq_out) *eq_out = data_syms;
    // Per-subcarrier noise weighting: ZF enhances noise on faded bins by
    // 1/|H(k)|^2, so hand the demapper the true per-bin noise instead of one
    // scalar. This is the largest coding-gain win on any frequency-selective
    // channel; on a flat channel it reduces to the scalar case.
    std::vector<float> nv;
    buildDataBinNoise(noise_variance, data_syms.size(), nv);
    if (use_pwl) mapper_->demapSoftPWL(data_syms, nv, llrs);
    else         mapper_->demapSoft   (data_syms, nv, llrs);
    return true;
}

bool OFDMDemodulator::demodulateSoftDD(const ComplexBuf& samples,
                                        std::vector<float>& llrs,
                                        float noise_variance,
                                        bool use_pwl,
                                        ComplexBuf* eq_out) {
    ComplexBuf data_syms;
    if (!demodulate(samples, data_syms)) return false;

    // Build a vector of hard-decided constellation references for each
    // data subcarrier — these are our "virtual pilots". Track the
    // confidence per symbol as 1 / (1 + |error|²): well-decided symbols
    // get ~1, ambiguous symbols get ≪ 1 and contribute little to the
    // refinement.
    const auto& const_pts = mapper_->constellation();
    if (const_pts.empty()) {
        // BPSK with empty constellation array — fall back to plain LLR
        // (still per-bin weighted).
        if (eq_out) *eq_out = data_syms;
        std::vector<float> nv;
        buildDataBinNoise(noise_variance, data_syms.size(), nv);
        if (use_pwl) mapper_->demapSoftPWL(data_syms, nv, llrs);
        else         mapper_->demapSoft   (data_syms, nv, llrs);
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

    if (persistent_dd_) {
        // -------------------------------------------------------------------
        // PERSISTENT (cross-symbol) decision-directed tracking. (#29)
        //
        // DESIGN — track the RESIDUAL, leash it to the pilots.
        //   The per-symbol pilot estimate h_pilot (ch_est_, fresh every symbol)
        //   already follows the channel's gross / common motion at the pilot
        //   bins; what it does NOT capture is the frequency-selective structure
        //   BETWEEN pilots, which linear interpolation reconstructs imperfectly
        //   on a multipath channel. The decision-directed LS estimate h_ls at a
        //   DATA bin measures the TRUE channel there (modulo decision/noise
        //   error). So we accumulate the RESIDUAL  r = h_ls - h_pilot  with a
        //   slow per-bin IIR (dd_persist_ stores h_pilot + r, i.e. the absolute
        //   corrected estimate, but the IIR target is recomputed from the LIVE
        //   pilot each symbol). Averaging r across many symbols drives down its
        //   noise variance — the win intra-symbol DD (which blends 30 % of ONE
        //   noisy h_ls and throws it away) cannot get.
        //
        //   Tracking the residual rather than the absolute channel decouples the
        //   accumulator from the pilots' common-phase/gross motion: if the whole
        //   channel rotates, h_pilot rotates with it and r stays small, so the
        //   accumulator does NOT lag a moving channel (the failure mode of a
        //   naive absolute accumulator). A LEASH caps |corrected - h_pilot| to a
        //   fraction of |h_pilot|, so error propagation / confident-but-wrong
        //   runs can never pull a bin far from the live pilot estimate — the
        //   correction is bounded and self-healing.
        //
        //   trust (dd_conf_) is a slow low-pass of per-symbol confidence: the
        //   correction is applied in proportion to how consistently confident
        //   that bin's decisions have been. On a clean/static channel r → ~0 so
        //   the whole thing is a near-no-op; pilots keep a floor of influence.
        if (dd_persist_.size() != ch_est_.size() || !dd_seeded_) {
            dd_persist_ = ch_est_;          // lazy seed (no preamble yet)
            if (dd_conf_.size() != ch_est_.size())
                dd_conf_.assign(ch_est_.size(), 0.f);
            else
                std::fill(dd_conf_.begin(), dd_conf_.end(), 0.f);
            dd_seeded_ = true;
        }
        constexpr float RES_LP     = 0.12f; // residual IIR rate (per confident sym)
        constexpr float CONF_LP    = 0.08f; // confidence low-pass rate
        constexpr float TRUST_MAX  = 0.85f; // cap so pilots keep a floor
        constexpr float LEASH      = 0.50f; // max |corrected - pilot| / |pilot|
        for (size_t i = 0; i < Nd; ++i) {
            size_t bin = alloc_.data_indices[i];
            ComplexSample h_pilot = ch_est_[bin];   // LIVE per-symbol pilot estimate
            // Current accumulated residual relative to the live pilot.
            ComplexSample res = dd_persist_[bin] - h_pilot;
            if (std::abs(refs[i]) >= 1e-6f) {
                ComplexSample h_ls   = (fft_out_[bin] * derot) / refs[i];
                ComplexSample res_ls = h_ls - h_pilot;       // measured residual
                float step = RES_LP * conf[i];
                res = (1.f - step) * res + step * res_ls;     // average it down
                dd_conf_[bin] = (1.f - CONF_LP) * dd_conf_[bin] + CONF_LP * conf[i];
            }
            // Leash: never let the correction stray more than LEASH·|h_pilot|
            // from the live pilot estimate (bounds error propagation).
            float pil_mag = std::abs(h_pilot);
            float max_dev = LEASH * (pil_mag > 1e-6f ? pil_mag : 1e-6f);
            float res_mag = std::abs(res);
            if (res_mag > max_dev && res_mag > 1e-12f)
                res *= (max_dev / res_mag);
            dd_persist_[bin] = h_pilot + res;   // store absolute corrected estimate

            float trust = dd_conf_[bin];
            if (trust > TRUST_MAX) trust = TRUST_MAX;
            if (trust < 0.f)       trust = 0.f;
            // Apply the trusted fraction of the (leashed, averaged) residual.
            ComplexSample h_use = h_pilot + trust * res;
            ch_est_[bin] = h_use;
            ch_mag_[bin] = std::abs(h_use);
        }
    } else {
        // Refine ch_est_ at each data subcarrier by exponentially blending
        // the LS estimate Y/X̂ (in the φ-corrected frame) with the pilot estimate.
        // The blend weight scales with confidence — at conf=1 we accept
        // 30 % of the new LS measurement; at conf→0 we leave the pilot
        // estimate unchanged. INTRA-symbol only (discarded next symbol).
        constexpr float MAX_BLEND = 0.30f;
        for (size_t i = 0; i < Nd; ++i) {
            size_t bin = alloc_.data_indices[i];
            if (std::abs(refs[i]) < 1e-6f) continue;
            ComplexSample h_ls = (fft_out_[bin] * derot) / refs[i];
            float w = MAX_BLEND * conf[i];
            ch_est_[bin] = (1.f - w) * ch_est_[bin] + w * h_ls;
            ch_mag_[bin] = std::abs(ch_est_[bin]);
        }
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

    // Per-bin weighting uses the REFINED |H| (ch_mag_ was updated above).
    if (eq_out) *eq_out = refined;
    std::vector<float> nv;
    buildDataBinNoise(noise_variance, refined.size(), nv);
    if (use_pwl) mapper_->demapSoftPWL(refined, nv, llrs);
    else         mapper_->demapSoft   (refined, nv, llrs);
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
    std::fill(dd_persist_.begin(), dd_persist_.end(), ComplexSample(1.f, 0.f));
    std::fill(dd_conf_.begin(), dd_conf_.end(), 0.f);
    dd_seeded_ = false;
    snr_db_ = 0.f;
    noise_var_ = 0.1f;
    if (mmse_) mmse_->reset();
    if (phase_tracker_) phase_tracker_->reset();
    if (sro_)           sro_->reset();
    last_int_cfo_ = 0;
    cfo_rad_per_sample_ = 0.f;
    rx_sample_pos_      = 0;
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
                                 SyncResult& result, size_t search_range) {
    result = SyncResult{};

    size_t N = p_.fft_size;
    // search_range == 0 → default narrow refine window (±N/8): pulls a coarse,
    // CP-boundary-aligned offset onto the exact sample. A caller that has not
    // yet localized the preamble to within a symbol passes a wider range to
    // ACQUIRE (scan) for the long-symbol body. ZC's sharp autocorrelation keeps
    // the peak unambiguous even over a wide scan.
    size_t range = (search_range > 0) ? search_range : (N / 8);

    if (long_preamble_td_.empty() || long_preamble_td_.size() != N) return false;
    // Guard: with fewer samples than one body the search space is empty —
    // and `samples.size() - N` below would underflow into a huge scan that
    // reads out of bounds.
    if (samples.size() < N) return false;

    float preamble_energy = 0.f;
    for (auto& s : long_preamble_td_) preamble_energy += std::norm(s);

    float best_corr = 0.f;
    int best_off = coarse_offset;

    size_t lo = static_cast<size_t>(std::max(0, coarse_offset - static_cast<int>(range)));
    // +1: the window STARTING at size-N is valid; `d < hi` must include it.
    size_t hi = std::min(samples.size() - N + 1,
                         static_cast<size_t>(coarse_offset + range));
    if (hi <= lo) return false;

    std::vector<float> metric(hi - lo, 0.f);
    for (size_t d = lo; d < hi; ++d) {
        ComplexSample c(0.f, 0.f);
        float local_e = 0.f;
        for (size_t i = 0; i < N; ++i) {
            c += samples[d + i] * std::conj(long_preamble_td_[i]);
            local_e += std::norm(samples[d + i]);
        }
        float norm_f = std::sqrt(local_e * preamble_energy);
        float normalized = (norm_f > 1e-6f) ? std::abs(c) / norm_f : 0.f;
        metric[d - lo] = normalized;

        if (normalized > best_corr) {
            best_corr = normalized;
            best_off = static_cast<int>(d);
        }
    }

    // Earliest-peak tie-break. The long preamble is TWO identical ZC
    // symbols, so the cross-correlation has two equal peaks sym_len apart;
    // float rounding (or channel tilt) can hand the global max to the
    // SECOND body. Anchoring there mis-positions the channel estimate one
    // symbol late — its "second symbol" is payload/silence (h2 garbage,
    // SNR estimate pinned near -3 dB) and every payload slice shifts.
    // Take the earliest candidate within 5% of the maximum instead.
    if (best_corr > 0.f) {
        const float accept = 0.95f * best_corr;
        for (size_t k = 0; k < metric.size(); ++k) {
            if (metric[k] >= accept) {
                best_off  = static_cast<int>(lo + k);
                best_corr = metric[k];
                break;
            }
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

} // namespace gw
