/**
 * @file types.hpp
 * @brief Core type definitions for DSCA-NG v2
 *
 * Single source of truth for all types, enums, constants, and
 * configuration structures used throughout the system.
 */

#pragma once

#ifdef _WIN32
    #ifndef _USE_MATH_DEFINES
        #define _USE_MATH_DEFINES
    #endif
    #ifndef NOMINMAX
        #define NOMINMAX
    #endif
#endif

#include <cstdint>
#include <cmath>
#include <complex>
#include <vector>
#include <array>
#include <string>
#include <functional>

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

namespace dsca {

// =========================================================================
// Fundamental Types
// =========================================================================

using Sample        = float;
using ComplexSample = std::complex<float>;
using SampleBuf     = std::vector<Sample>;
using ComplexBuf    = std::vector<ComplexSample>;
using ByteVec       = std::vector<uint8_t>;

// =========================================================================
// Portable Bit Utilities (no __builtin required)
// =========================================================================

inline uint32_t popcount32(uint32_t x) {
    x = x - ((x >> 1) & 0x55555555u);
    x = (x & 0x33333333u) + ((x >> 2) & 0x33333333u);
    return (((x + (x >> 4)) & 0x0F0F0F0Fu) * 0x01010101u) >> 24;
}

inline uint32_t popcount8(uint8_t x) {
    return popcount32(static_cast<uint32_t>(x));
}

// =========================================================================
// System Constants
// =========================================================================

namespace constants {
    constexpr size_t   MAX_STREAMS    = 8;
    constexpr size_t   MAX_FFT_SIZE   = 16384;
    constexpr size_t   MIN_FFT_SIZE   = 64;
    constexpr uint64_t SYNC_PATTERN   = 0xACBDCEDF12345678ULL;
    constexpr size_t   SYNC_BYTES     = 8;
    constexpr size_t   HEADER_BYTES   = 12;
    constexpr size_t   CRC_BYTES      = 4;
    constexpr size_t   FRAME_OVERHEAD = SYNC_BYTES + HEADER_BYTES + CRC_BYTES; // 24
}

// =========================================================================
// Enumerations
// =========================================================================

/** Subcarrier modulation schemes */
enum class Modulation : uint8_t {
    BPSK    = 0,   // 1 bit/sym,  SNR > 3 dB
    QPSK    = 1,   // 2 bit/sym,  SNR > 6 dB
    QAM16   = 2,   // 4 bit/sym,  SNR > 12 dB
    QAM64   = 3,   // 6 bit/sym,  SNR > 18 dB
    QAM256  = 4,   // 8 bit/sym,  SNR > 24 dB
    QAM1024 = 5,   // 10 bit/sym, SNR > 30 dB
    QAM4096 = 6,   // 12 bit/sym, SNR > 36 dB
};

inline constexpr uint8_t bitsPerSymbol(Modulation m) {
    switch (m) {
        case Modulation::BPSK:    return 1;
        case Modulation::QPSK:    return 2;
        case Modulation::QAM16:   return 4;
        case Modulation::QAM64:   return 6;
        case Modulation::QAM256:  return 8;
        case Modulation::QAM1024: return 10;
        case Modulation::QAM4096: return 12;
        default: return 2;
    }
}

/** FEC code rates */
enum class FECRate : uint8_t {
    Rate_1_4  = 0,
    Rate_1_3  = 1,
    Rate_2_5  = 2,
    Rate_1_2  = 3,
    Rate_3_5  = 4,
    Rate_2_3  = 5,
    Rate_3_4  = 6,
    Rate_4_5  = 7,
    Rate_5_6  = 8,
    Rate_8_9  = 9,
    Rate_9_10 = 10,
    None      = 255,
};

inline constexpr float codeRateValue(FECRate r) {
    switch (r) {
        case FECRate::Rate_1_4:  return 0.25f;
        case FECRate::Rate_1_3:  return 1.f/3.f;
        case FECRate::Rate_2_5:  return 0.4f;
        case FECRate::Rate_1_2:  return 0.5f;
        case FECRate::Rate_3_5:  return 0.6f;
        case FECRate::Rate_2_3:  return 2.f/3.f;
        case FECRate::Rate_3_4:  return 0.75f;
        case FECRate::Rate_4_5:  return 0.8f;
        case FECRate::Rate_5_6:  return 5.f/6.f;
        case FECRate::Rate_8_9:  return 8.f/9.f;
        case FECRate::Rate_9_10: return 0.9f;
        case FECRate::None:      return 1.0f;
        default: return 0.5f;
    }
}

/** Cyclic prefix ratios */
enum class CyclicPrefix : uint8_t {
    CP_1_4  = 0,
    CP_1_8  = 1,
    CP_1_16 = 2,
    CP_1_32 = 3,
};

inline constexpr float cpRatio(CyclicPrefix cp) {
    switch (cp) {
        case CyclicPrefix::CP_1_4:  return 0.25f;
        case CyclicPrefix::CP_1_8:  return 0.125f;
        case CyclicPrefix::CP_1_16: return 0.0625f;
        case CyclicPrefix::CP_1_32: return 0.03125f;
        default: return 0.125f;
    }
}

/** Receiver synchronization state */
enum class SyncState : uint8_t {
    Searching = 0,
    Acquiring = 1,
    Locked    = 2,
    Tracking  = 3,
    Lost      = 4,
};

// =========================================================================
// Configuration Structures
// =========================================================================

/** OFDM parameters — shared identically by TX and RX */
struct OFDMParams {
    uint16_t     fft_size       = 256;
    CyclicPrefix cyclic_prefix  = CyclicPrefix::CP_1_8;
    Modulation   modulation     = Modulation::QAM16;
    uint8_t      pilot_spacing  = 8;     // pilot every N active subcarriers
    float        pilot_boost_db = 3.0f;  // pilot power boost
    bool         dc_null        = true;  // null DC subcarrier
    uint32_t     sample_rate    = 48000; // Hz

    // Guard bands: 0 = auto (5% of SR per side, or constrained by
    // target_bw_hz when set)
    uint16_t guard_left  = 0;
    uint16_t guard_right = 0;

    // Target OFDM-occupied bandwidth (Hz). When > 0, the auto guard
    // calculation widens guards as needed to keep the active subcarrier
    // region within this bandwidth. Set by the engine from
    // ModemConfig.signal_bw so OFDM allocation tracks the configured
    // LPF cutoff — without this the TX LPF clips edge subcarriers and
    // corrupts data. Zero = no target constraint (use SR-relative
    // 5%-per-side default).
    float    target_bw_hz   = 0.f;

    // CP windowing taper width as percent of CP length. When > 0, the
    // TX modulator applies a raised-cosine (Hann-shaped) rise window
    // over the FIRST W samples of each symbol's CP and a complementary
    // fall window over the LAST W samples of the symbol body, where
    // W = round(cp * pct / 100). Smooths the time-domain transitions
    // at symbol boundaries, knocking ~3–6 dB off the OFDM spectral
    // side-lobes at modest tapers (12 %) and more at higher tapers —
    // helpful for FCC §73.319 spectral-mask compliance.
    //
    // The receiver does not need to know about the windowing (FFT
    // window stays unchanged). HOWEVER the last W samples of body are
    // attenuated by the fall window, which introduces a small amount
    // of inter-carrier leakage. For high-density modcods (QAM256 at
    // FEC 9/10) operating at the SNR cliff, even this tiny leakage
    // can push the post-FEC BER above zero on otherwise-clean channels.
    // Therefore the default is 0 (disabled, legacy rectangular CP)
    // and the user opts in (e.g., 8–15 %) when spectral-mask
    // compliance matters more than the marginal SNR on the highest
    // modcods. Range: 0 to 25.
    float    cp_window_taper_pct = 0.f;

    // Derived helpers
    size_t cpLength() const {
        return static_cast<size_t>(fft_size * cpRatio(cyclic_prefix));
    }
    size_t symbolLength() const { return fft_size + cpLength(); }
    double subcarrierSpacing() const {
        return static_cast<double>(sample_rate) / fft_size;
    }
    double symbolDuration() const {
        return static_cast<double>(symbolLength()) / sample_rate;
    }
    /** Guard subcarriers on the lower band edge. When set explicitly
     *  (guard_left > 0) that value is returned; otherwise auto-default
     *  is the LARGER of:
     *    (a) ~5% of sample_rate per side (physical adjacent-channel
     *        protection)
     *    (b) enough guard to keep active_subcarriers × subcarrier_spacing
     *        ≤ target_bw_hz (when target_bw_hz > 0)
     *
     *  Constraint (b) ensures the OFDM-occupied bandwidth never exceeds
     *  the configured signal bandwidth (modem.signal_bw). Without this,
     *  the TX LPF at signal_bw × 1.1 would clip outer subcarriers and
     *  cause data loss. */
    size_t guardLeft() const {
        if (guard_left > 0) return guard_left;
        if (fft_size == 0)  return 0;
        double sc_spacing = subcarrierSpacing();
        if (sc_spacing <= 0.0) return fft_size / 10;
        // (a) Sample-rate-relative floor: 5% per side, ~2.4 kHz at SR=48k.
        double sr_guard_hz = 0.05 * static_cast<double>(sample_rate);
        size_t n_sr = static_cast<size_t>(std::ceil(sr_guard_hz / sc_spacing));
        // (b) Target-bandwidth constraint: each side must hold
        //     (fft_size - target_bw/spacing) / 2 guard bins (symmetric).
        size_t n_target = 0;
        if (target_bw_hz > 0.f) {
            double target_active = static_cast<double>(target_bw_hz) /
                                    sc_spacing;
            double dc_bin = dc_null ? 1.0 : 0.0;
            double extra  = (static_cast<double>(fft_size) - dc_bin -
                              target_active) * 0.5;
            if (extra > 0.0) {
                n_target = static_cast<size_t>(std::ceil(extra));
            }
        }
        size_t n = (n_target > n_sr) ? n_target : n_sr;
        // Floor at the SR-relative value and ceiling at fft/4 so a tiny
        // signal_bw doesn't reduce active to zero.
        size_t max_guard = fft_size / 4;
        return (n < max_guard) ? n : max_guard;
    }
    size_t guardRight() const {
        if (guard_right > 0) return guard_right;
        return guardLeft();  // symmetric auto-default
    }
};

/** Frame-level parameters */
struct FrameParams {
    FECRate  fec_rate = FECRate::Rate_1_2;
    uint32_t preamble_interval = 50; // frames between preambles
};

// =========================================================================
// CRC-32/BZIP2  (poly 0x04C11DB7, init 0xFFFFFFFF, NON-reflected in/out,
// final XOR 0xFFFFFFFF). Known-answer: crc32("123456789") == 0xFC891918.
//
// NOTE: this is NOT the reflected CRC-32/ISO-HDLC (V.42, zlib/PNG) — the
// label was previously wrong. It is internally self-consistent (TX and RX
// use this same routine, so frame CRCs round-trip correctly); the only
// hazard is interop with an external party expecting reflected CRC-32.
// A KAT guards it in frame_test.
// =========================================================================

inline uint32_t crc32(const uint8_t* data, size_t len) {
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint32_t>(data[i]) << 24;
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x80000000u) ? (crc << 1) ^ 0x04C11DB7u
                                       : (crc << 1);
        }
    }
    return crc ^ 0xFFFFFFFFu;
}

// =========================================================================
// Status / Statistics
// =========================================================================

struct ModemStats {
    SyncState  sync_state    = SyncState::Searching;
    float      snr_db        = 0.f;
    float      evm_percent   = 100.f;
    float      freq_offset   = 0.f;       ///< Fractional CFO from preamble (Hz)
    float      ber_estimate  = 1.f;       ///< Post-FEC frame error rate
    uint64_t   frames_tx     = 0;
    uint64_t   frames_rx     = 0;
    uint64_t   frames_ok     = 0;
    uint64_t   frames_bad    = 0;
    uint64_t   bits_total    = 0;
    uint64_t   bit_errors    = 0;

    // Extended diagnostics (RF/DSP engineer view)
    float      cfo_total_hz    = 0.f;     ///< Fractional + integer CFO combined
    int        integer_cfo_bins = 0;      ///< Last detected integer bin shift
    float      clock_ppm       = 0.f;     ///< Estimated SR offset (parts per million)
    float      pre_fec_ber     = 1.f;     ///< Uncoded BER from LDPC syndrome
    float      papr_tx_db      = 0.f;     ///< Last computed TX PAPR
    float      agc_ripple_db   = 0.f;     ///< AGC pumping detector (peak-to-peak)
    float      rt_latency_ms   = 0.f;     ///< TX→RX round-trip in ms (loopback)
    float      tx_peak_dbfs    = -60.f;   ///< TX peak amplitude (dBFS, peak-held)
    bool       tx_clipping     = false;   ///< TX clipped this frame

    // Engine health
    float      tick_max_ms     = 0.f;     ///< Worst tick wall-clock over recent window
    float      tick_avg_ms     = 0.f;     ///< Mean tick wall-clock over recent window

    // Hierarchical-modulation layer status. Populated only when the engine
    // is running in M/S hierarchical mode (`ms_mode_active`). Allows the
    // GUI to show graceful-degradation state: HP locked + LP locked =
    // stereo; HP locked + LP failed = mono fallback; HP failed = no
    // useful audio.
    bool       hier_active        = false; ///< true when M/S hierarchical chain is wired
    uint64_t   hp_frames_ok       = 0;     ///< HP-layer (Mid) frames passing CRC
    uint64_t   hp_frames_bad      = 0;     ///< HP-layer frames failing CRC
    uint64_t   lp_frames_ok       = 0;     ///< LP-layer (Side) frames passing CRC
    uint64_t   lp_frames_bad      = 0;     ///< LP-layer frames failing CRC
    float      hp_avg_llr_mag     = 0.f;   ///< Mean |posterior LLR| from HP LDPC decoder
    float      lp_avg_llr_mag     = 0.f;   ///< Mean |posterior LLR| from LP LDPC decoder
    bool       hp_locked          = false; ///< Recent HP frame succeeded
    bool       lp_locked          = false; ///< Recent LP frame succeeded
};

} // namespace dsca
