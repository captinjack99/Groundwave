/**
 * @file soundcard_modem.cpp
 * @brief Soundcard modem implementation
 *
 * Currently implements loopback mode for testing. Hardware mode (miniaudio)
 * will be added when building with GW_ENABLE_AUDIO defined.
 */

#include "soundcard_modem.hpp"
#include <cmath>
#include <algorithm>
#include <random>
#include <stdexcept>
#include <cstdio>
#include <utility>

namespace gw {

// =========================================================================
// Ring Buffer
// =========================================================================

RingBuffer::RingBuffer(size_t capacity)
    : buf_(capacity + 1), capacity_(capacity + 1) {}

size_t RingBuffer::write(const float* data, size_t count) {
    size_t wp = write_pos_.load(std::memory_order_relaxed);
    size_t rp = read_pos_.load(std::memory_order_acquire);

    size_t avail_space;
    if (wp >= rp) {
        avail_space = capacity_ - 1 - (wp - rp);
    } else {
        avail_space = rp - wp - 1;
    }

    count = std::min(count, avail_space);

    for (size_t i = 0; i < count; ++i) {
        buf_[wp] = data[i];
        wp = (wp + 1) % capacity_;
    }

    write_pos_.store(wp, std::memory_order_release);
    return count;
}

size_t RingBuffer::read(float* data, size_t count) {
    size_t rp = read_pos_.load(std::memory_order_relaxed);
    size_t wp = write_pos_.load(std::memory_order_acquire);

    size_t avail;
    if (wp >= rp) {
        avail = wp - rp;
    } else {
        avail = capacity_ - rp + wp;
    }

    count = std::min(count, avail);

    for (size_t i = 0; i < count; ++i) {
        data[i] = buf_[rp];
        rp = (rp + 1) % capacity_;
    }

    read_pos_.store(rp, std::memory_order_release);
    return count;
}

size_t RingBuffer::available() const {
    size_t rp = read_pos_.load(std::memory_order_acquire);
    size_t wp = write_pos_.load(std::memory_order_acquire);
    if (wp >= rp) return wp - rp;
    return capacity_ - rp + wp;
}

size_t RingBuffer::space() const {
    size_t rp = read_pos_.load(std::memory_order_acquire);
    size_t wp = write_pos_.load(std::memory_order_acquire);
    size_t used;
    if (wp >= rp) used = wp - rp;
    else used = capacity_ - rp + wp;
    return capacity_ - 1 - used;
}

void RingBuffer::clear() {
    read_pos_.store(0, std::memory_order_release);
    write_pos_.store(0, std::memory_order_release);
}

// =========================================================================
// SoundcardModem
// =========================================================================

SoundcardModem::SoundcardModem(const ModemConfig& cfg,
                                const OFDMParams& ofdm_params)
    : cfg_(cfg), ofdm_p_(ofdm_params),
      upconv_(cfg.sample_rate,
              std::min(cfg.center_freq,
                       static_cast<float>(cfg.sample_rate) / 2.f - 1.f)),
      // ModemConfig.signal_bw represents the FULL signal bandwidth in Hz
      // (e.g. user enters 27 kHz to span 69-96 kHz at Fc=82.5 kHz).
      // IQDownconverter expects the ONE-SIDED LPF cutoff, so divide by 2.
      // 0 = "auto" → the OFDM allocation's actual occupied bandwidth, so
      // the LPF passes exactly the band the carriers live in. (The old
      // "80% of Nyquist" auto could disagree with the allocation in both
      // directions and clip edge carriers.)
      downconv_(cfg.sample_rate,
                std::min(cfg.center_freq,
                         static_cast<float>(cfg.sample_rate) / 2.f - 1.f),
                cfg.complex_loopback
                    ? cfg.center_freq * 0.5f  // safe dummy BW for complex loopback
                    : (cfg.signal_bw > 0.f
                        ? cfg.signal_bw
                        : static_cast<float>(
                              ofdm_params.occupiedBandwidthHz())) * 0.5f,
                cfg.lpf_taps),
      agc_(cfg.sample_rate, cfg.agc),
      dc_blocker_(),
      iq_balance_(cfg.enable_iq_balance ? IQBalanceConfig{}
                                         : IQBalanceConfig{0.f, 1e-6f, false}),
      squelch_(cfg.squelch),
      tx_ramp_(cfg.tx_ramp_samples),
      tx_ring_(cfg.sample_rate * 2),     // 2 seconds buffer
      rx_ring_(cfg.sample_rate * 2),
      loopback_ring_(cfg.sample_rate * 2)
{
    // Configure DC blocker pole based on requested cutoff
    if (cfg.enable_dc_blocker) {
        dc_blocker_.setCutoff(cfg.dc_blocker_cutoff_hz,
                              static_cast<float>(cfg.sample_rate));
    }

    // Build polyphase resamplers if the OFDM rate differs from the audio rate.
    // Per-direction sample rates are honored: tx_sample_rate / rx_sample_rate
    // override the unified `sample_rate` for the corresponding direction so a
    // playback device at 192 kHz and a capture device at 96 kHz can coexist.
    if (cfg.enable_resampler) {
        uint32_t ofdm_rate = cfg.ofdm_sample_rate
                             ? cfg.ofdm_sample_rate
                             : ofdm_params.sample_rate;
        uint32_t tx_rate   = cfg.tx_sample_rate
                             ? cfg.tx_sample_rate : cfg.sample_rate;
        uint32_t rx_rate   = cfg.rx_sample_rate
                             ? cfg.rx_sample_rate : cfg.sample_rate;
        auto gcd = [](uint32_t a, uint32_t b) {
            while (b) { uint32_t t = b; b = a % b; a = t; }
            return a;
        };
        // Bound L/M after GCD reduction. A near-but-coprime rate pair
        // (e.g. 48000/44101, gcd=1) would otherwise build an L≈44101-tap
        // polyphase filter and an n_in·L intermediate per call — an OOM /
        // stall. Approximate the ratio with a continued-fraction rational
        // whose terms are capped, logging the residual rate error. (The
        // GUI file-load path already guards this; the modem ctor did not.) (#17)
        constexpr uint32_t LM_CAP = 2048;
        auto boundRatio = [](uint32_t num, uint32_t den,
                             uint32_t cap) -> std::pair<uint32_t,uint32_t> {
            if (num <= cap && den <= cap) return {num, den};
            double x = static_cast<double>(num) / static_cast<double>(den);
            uint32_t p0 = 0, q0 = 1, p1 = 1, q1 = 0;  // CF convergents
            double v = x;
            for (int it = 0; it < 64; ++it) {
                double fl = std::floor(v);
                uint32_t a = static_cast<uint32_t>(fl);
                uint32_t p2 = a * p1 + p0, q2 = a * q1 + q0;
                if (p2 > cap || q2 > cap || p2 == 0) break;
                p0 = p1; q0 = q1; p1 = p2; q1 = q2;
                double frac = v - fl;
                if (frac < 1e-9) break;
                v = 1.0 / frac;
            }
            if (p1 == 0 || q1 == 0) return { std::min(num, cap), std::min(den, cap) };
            return {p1, q1};
        };
        auto makeResampler = [&](uint32_t up, uint32_t down, const char* dir)
                -> std::unique_ptr<RationalResampler> {
            uint32_t g = gcd(up, down);
            uint32_t L = up / g, M = down / g;
            if (L > LM_CAP || M > LM_CAP) {
                auto [L2, M2] = boundRatio(L, M, LM_CAP);
                std::fprintf(stderr,
                    "[modem] %s resampler L/M (%u/%u) exceeds cap %u; using "
                    "bounded approximation %u/%u (residual rate error ~%.3f%%)\n",
                    dir, L, M, LM_CAP, L2, M2,
                    100.0 * std::fabs((double)L2 / M2 - (double)L / M) /
                        ((double)L / M));
                L = L2; M = M2;
            }
            return std::make_unique<RationalResampler>(L, M);
        };
        if (ofdm_rate > 0 && tx_rate != ofdm_rate) {
            tx_resampler_ = makeResampler(tx_rate, ofdm_rate, "TX");
        }
        if (ofdm_rate > 0 && rx_rate != ofdm_rate) {
            rx_resampler_ = makeResampler(ofdm_rate, rx_rate, "RX");
        }
    }
    // Skip IQ path validation in complex baseband loopback mode
    if (!cfg.complex_loopback) {
        // Validate center frequency
        float nyquist = static_cast<float>(cfg.sample_rate) / 2.0f;
        if (cfg.center_freq >= nyquist) {
            throw std::invalid_argument("Center freq must be < Nyquist");
        }

        // Signal occupies [fc - bw/2, fc + bw/2] — both edges must be inside
        // (0, Nyquist). 0-default for signal_bw means "use the OFDM
        // allocation's actual occupied bandwidth" so validation and the
        // LPFs agree with where the carriers really are.
        float sig_bw = cfg.signal_bw > 0.f ? cfg.signal_bw :
                       static_cast<float>(ofdm_params.occupiedBandwidthHz());
        float half_bw = sig_bw * 0.5f;
        if (cfg.center_freq - half_bw < 0.f ||
            cfg.center_freq + half_bw > nyquist) {
            throw std::invalid_argument(
                "Signal bandwidth exceeds available passband around center freq");
        }

        // Configure the TX-side LPF for spectral-mask compliance. Skipped
        // for complex loopback paths because no real passband is generated
        // there (no spectrum to shape). 0 taps = disabled.
        if (cfg.tx_lpf_taps > 0) {
            upconv_.setLPF(half_bw, cfg.tx_lpf_taps);
        }
    }
}

SoundcardModem::~SoundcardModem() = default;

size_t SoundcardModem::transmit(const ComplexBuf& baseband) {
    // Complex baseband loopback: store complex samples directly
    if (cfg_.complex_loopback) {
        complex_loopback_buf_.insert(complex_loopback_buf_.end(),
                                      baseband.begin(), baseband.end());
        // Cap growth: if RX under-drains (TX outpaces receive()), drop the
        // oldest so this can't grow without bound (the passband loopback uses
        // a fixed RingBuffer; the complex path had no equivalent guard).
        constexpr size_t CPLX_LOOPBACK_CAP = 1u << 20;   // ~1M samples
        if (complex_loopback_buf_.size() > CPLX_LOOPBACK_CAP) {
            complex_loopback_buf_.erase(
                complex_loopback_buf_.begin(),
                complex_loopback_buf_.begin() +
                static_cast<ptrdiff_t>(complex_loopback_buf_.size() - CPLX_LOOPBACK_CAP));
        }
        return baseband.size();
    }

    // Upconvert to real passband
    std::vector<float> passband;
    upconv_.upconvert(baseband, passband);

    // Resample OFDM rate → audio rate if needed.
    if (tx_resampler_) {
        size_t L = tx_resampler_->L();
        size_t M = tx_resampler_->M();
        std::vector<float> resampled(passband.size() * L / M + L + 16);
        size_t n = tx_resampler_->process(passband.data(), passband.size(),
                                          resampled.data());
        resampled.resize(n);
        passband.swap(resampled);
    }

    // TX power ramp: shapes the on/off transitions to reduce splatter.
    // Keyed on while transmit() is being called; the leading edge ramps
    // 0→1 over the first tx_ramp_samples. To also ramp DOWN cleanly at
    // key-off (endTransmission), we run a one-buffer-tail delay line: the
    // last tx_ramp_samples of each buffer are held back so there are always
    // real samples available to fade to zero. This avoids both the hard
    // full-amplitude cut (broadband splatter) and any sample duplication.
    if (cfg_.enable_tx_ramp) {
        tx_ramp_.setKey(true);
        size_t hold = std::min<size_t>(cfg_.tx_ramp_samples, passband.size());
        std::vector<float> out;
        out.reserve(tx_hold_.size() + passband.size() - hold);
        out.insert(out.end(), tx_hold_.begin(), tx_hold_.end());
        out.insert(out.end(), passband.begin(),
                   passband.end() - static_cast<ptrdiff_t>(hold));
        tx_hold_.assign(passband.end() - static_cast<ptrdiff_t>(hold),
                        passband.end());
        tx_ramp_.apply(out.data(), out.size());
        passband.swap(out);
    }

    if (cfg_.loopback) {
        return loopback_ring_.write(passband.data(), passband.size());
    } else {
        return tx_ring_.write(passband.data(), passband.size());
    }
}

void SoundcardModem::endTransmission() {
    if (!cfg_.enable_tx_ramp) return;
    tx_ramp_.setKey(false);
    if (!tx_hold_.empty()) {
        // Fade the held tail from the current envelope (≈1) down to 0.
        std::vector<float> fade = tx_hold_;
        tx_ramp_.apply(fade.data(), fade.size());
        if (cfg_.loopback) loopback_ring_.write(fade.data(), fade.size());
        else               tx_ring_.write(fade.data(), fade.size());
        tx_hold_.clear();
    }
    tx_ramp_.reset(false);   // ready for the next key-up (ramps from 0)
}

size_t SoundcardModem::transmitRaw(const float* samples, size_t count) {
    if (cfg_.loopback) {
        return loopback_ring_.write(samples, count);
    } else {
        return tx_ring_.write(samples, count);
    }
}

ComplexBuf SoundcardModem::receive(size_t max_samples) {
    // Complex baseband loopback: return complex samples directly
    if (cfg_.complex_loopback) {
        size_t n = std::min(max_samples, complex_loopback_buf_.size());
        ComplexBuf result(complex_loopback_buf_.begin(),
                          complex_loopback_buf_.begin() + static_cast<ptrdiff_t>(n));
        complex_loopback_buf_.erase(complex_loopback_buf_.begin(),
                                     complex_loopback_buf_.begin() + static_cast<ptrdiff_t>(n));
        return result;
    }

    // Read real passband from RX ring
    std::vector<float> passband(max_samples);
    size_t got;

    if (cfg_.loopback) {
        got = loopback_ring_.read(passband.data(), max_samples);
    } else {
        got = rx_ring_.read(passband.data(), max_samples);
    }

    if (got == 0) return {};
    passband.resize(got);

    // RX chain: rx_gain → DC block → AGC → squelch → resample → downconvert.
    // rx_gain is applied first so the AGC sees a properly-leveled input
    // and adapts; useful when the OS-level mic boost is too low.
    if (cfg_.rx_gain_db != 0.f) {
        float g = std::pow(10.f, cfg_.rx_gain_db / 20.f);
        for (auto& s : passband) s *= g;
    }
    if (cfg_.enable_dc_blocker) {
        dc_blocker_.process(passband.data(), passband.size());
    }

    agc_.process(passband.data(), passband.size());

    if (cfg_.enable_rx_squelch) {
        squelch_.apply(passband.data(), passband.size());
    }

    // Resample audio rate → OFDM rate if needed (RX downsample direction).
    if (rx_resampler_) {
        size_t L = rx_resampler_->L();
        size_t M = rx_resampler_->M();
        std::vector<float> resampled(passband.size() * L / M + L + 16);
        size_t n = rx_resampler_->process(passband.data(), passband.size(),
                                          resampled.data());
        resampled.resize(n);
        passband.swap(resampled);
    }

    // Downconvert to complex baseband
    ComplexBuf baseband;
    downconv_.downconvert(passband.data(), passband.size(), baseband);

    // I/Q imbalance correction (post-downconvert, pre-FFT)
    if (cfg_.enable_iq_balance) {
        iq_balance_.process(baseband.data(), baseband.size());
    }

    return baseband;
}

size_t SoundcardModem::receiveRaw(float* samples, size_t max_samples) {
    if (cfg_.loopback) {
        return loopback_ring_.read(samples, max_samples);
    } else {
        return rx_ring_.read(samples, max_samples);
    }
}

void SoundcardModem::processLoopback() {
    // In loopback mode, data is written directly to loopback_ring_
    // by transmit(), and read from loopback_ring_ by receive().
    // No extra processing needed for perfect loopback.
}

void SoundcardModem::processLoopbackAWGN(float snr_db, uint32_t seed) {
    if (cfg_.complex_loopback) {
        // Add AWGN directly to complex baseband buffer
        if (complex_loopback_buf_.empty()) return;

        float sig_power = 0.f;
        for (auto& s : complex_loopback_buf_) sig_power += std::norm(s);
        sig_power /= static_cast<float>(complex_loopback_buf_.size());

        float snr_lin = std::pow(10.f, snr_db / 10.f);
        float sigma = std::sqrt(sig_power / (2.f * snr_lin));

        std::mt19937 rng(seed);
        std::normal_distribution<float> dist(0.f, sigma);
        for (auto& s : complex_loopback_buf_) {
            s += ComplexSample(dist(rng), dist(rng));
        }
        return;
    }

    if (!cfg_.loopback) return;

    // Read all available TX data from loopback ring
    size_t avail = loopback_ring_.available();
    if (avail == 0) return;

    std::vector<float> samples(avail);
    size_t got = loopback_ring_.read(samples.data(), avail);
    samples.resize(got);

    // Measure signal power
    float sig_power = 0.f;
    for (auto s : samples) sig_power += s * s;
    sig_power /= static_cast<float>(samples.size());

    // Add AWGN
    float snr_lin = std::pow(10.f, snr_db / 10.f);
    float noise_sigma = std::sqrt(sig_power / snr_lin);

    std::mt19937 rng(seed);
    std::normal_distribution<float> dist(0.f, noise_sigma);
    for (auto& s : samples) {
        s += dist(rng);
    }

    // Write back to loopback ring for RX to pick up
    loopback_ring_.write(samples.data(), samples.size());
}

float SoundcardModem::agcGainDB() const {
    return agc_.gainDB();
}

float SoundcardModem::agcSignalLevel() const {
    return agc_.signalLevel();
}

void SoundcardModem::reset() {
    upconv_.reset();
    downconv_.reset();
    agc_.reset();
    dc_blocker_.reset();
    iq_balance_.reset();
    squelch_.reset();
    tx_ramp_.reset(/*key_state=*/false);
    tx_hold_.clear();
    if (tx_resampler_) tx_resampler_->reset();
    if (rx_resampler_) rx_resampler_->reset();
    tx_ring_.clear();
    rx_ring_.clear();
    loopback_ring_.clear();
    complex_loopback_buf_.clear();
}

} // namespace gw
