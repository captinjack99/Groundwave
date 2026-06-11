/**
 * @file mimo.hpp
 * @brief 2×N MIMO encoders/decoders: Alamouti STBC, MRC, ZF, MMSE.
 *
 * v2 mostly targets soundcard / single-channel paths, but the framework
 * supports MIMO for SDR or multi-channel hardware setups. All operations
 * are per-subcarrier (post-OFDM-FFT), since the channel is flat per bin.
 *
 * Coordinate conventions:
 *   nT = number of TX antennas
 *   nR = number of RX antennas
 *   H is nR × nT complex matrix (H[r,t] = channel from TX t to RX r)
 *   x is nT × 1 transmit vector
 *   y = H·x + n (n: AWGN per RX)
 */
#pragma once

#include "types.hpp"
#include <vector>
#include <array>
#include <complex>
#include <cmath>

namespace gw {

// =========================================================================
// Alamouti 2×1 / 2×2 Space-Time Block Coding
// =========================================================================

/** Two consecutive symbols (s0, s1) are transmitted in two time slots
 *  from two antennas as:
 *    Slot 0:  ant0=s0,        ant1=s1
 *    Slot 1:  ant0=-conj(s1), ant1=conj(s0)
 *  Achieves rate-1 with full diversity in 2x1 / 2x2 systems. */
struct AlamoutiPair {
    std::array<ComplexSample, 2> slot0; ///< [ant0, ant1] in slot 0
    std::array<ComplexSample, 2> slot1; ///< [ant0, ant1] in slot 1
};

inline AlamoutiPair alamoutiEncode(ComplexSample s0, ComplexSample s1) {
    AlamoutiPair p;
    p.slot0[0] = s0;
    p.slot0[1] = s1;
    p.slot1[0] = -std::conj(s1);
    p.slot1[1] =  std::conj(s0);
    return p;
}

/** 2×1 Alamouti combining: given two RX samples y0 (slot 0) and y1 (slot 1)
 *  on a single RX antenna with channel h0 (from ant0) and h1 (from ant1),
 *  recover the soft estimates of s0, s1. */
inline void alamoutiDecode2x1(ComplexSample y0, ComplexSample y1,
                              ComplexSample h0, ComplexSample h1,
                              ComplexSample& s0_out, ComplexSample& s1_out) {
    float h_norm = std::norm(h0) + std::norm(h1);
    if (h_norm < 1e-12f) {
        s0_out = ComplexSample(0.f, 0.f);
        s1_out = ComplexSample(0.f, 0.f);
        return;
    }
    s0_out = (std::conj(h0) * y0 + h1 * std::conj(y1)) / h_norm;
    s1_out = (std::conj(h1) * y0 - h0 * std::conj(y1)) / h_norm;
}

/** 2×2 Alamouti combining: y0/y1 are 2-vectors (one per RX antenna),
 *  H is 2×2 (h[r,t]). */
inline void alamoutiDecode2x2(const std::array<ComplexSample, 2>& y0,
                              const std::array<ComplexSample, 2>& y1,
                              const std::array<std::array<ComplexSample, 2>, 2>& H,
                              ComplexSample& s0_out,
                              ComplexSample& s1_out) {
    // s0 = Σ_r ( conj(h_r0)·y0_r + h_r1·conj(y1_r) ) / Σ_r (|h_r0|² + |h_r1|²)
    float h_norm = 0.f;
    ComplexSample num0(0.f, 0.f), num1(0.f, 0.f);
    for (size_t r = 0; r < 2; ++r) {
        ComplexSample h0 = H[r][0], h1 = H[r][1];
        h_norm += std::norm(h0) + std::norm(h1);
        num0 += std::conj(h0) * y0[r] + h1 * std::conj(y1[r]);
        num1 += std::conj(h1) * y0[r] - h0 * std::conj(y1[r]);
    }
    if (h_norm < 1e-12f) {
        s0_out = ComplexSample(0.f, 0.f);
        s1_out = ComplexSample(0.f, 0.f);
        return;
    }
    s0_out = num0 / h_norm;
    s1_out = num1 / h_norm;
}

// =========================================================================
// Maximal Ratio Combining (1×nR SIMO)
// =========================================================================

/** MRC: y_eq = Σ conj(h_r)·y_r / Σ |h_r|². Optimal SIMO combiner in AWGN. */
inline ComplexSample mrcCombine(const ComplexSample* y, const ComplexSample* h, size_t nR) {
    float power = 0.f;
    ComplexSample acc(0.f, 0.f);
    for (size_t r = 0; r < nR; ++r) {
        acc += std::conj(h[r]) * y[r];
        power += std::norm(h[r]);
    }
    if (power < 1e-12f) return ComplexSample(0.f, 0.f);
    return acc / power;
}

// =========================================================================
// Spatial multiplexing detectors (nT > 1)
// =========================================================================

/** Zero-Forcing detector for 2×2 spatial multiplexing.
 *  x_hat = H^{-1} · y (uses inverse for nT=nR=2; see ZF for general case). */
inline std::array<ComplexSample, 2> zfDetect2x2(
    const std::array<ComplexSample, 2>& y,
    const std::array<std::array<ComplexSample, 2>, 2>& H)
{
    // H^{-1} = (1/det) * [[h11, -h01], [-h10, h00]]
    ComplexSample det = H[0][0] * H[1][1] - H[0][1] * H[1][0];
    std::array<ComplexSample, 2> x;
    if (std::abs(det) < 1e-9f) {
        x[0] = ComplexSample(0.f, 0.f);
        x[1] = ComplexSample(0.f, 0.f);
        return x;
    }
    ComplexSample inv00 =  H[1][1] / det;
    ComplexSample inv01 = -H[0][1] / det;
    ComplexSample inv10 = -H[1][0] / det;
    ComplexSample inv11 =  H[0][0] / det;
    x[0] = inv00 * y[0] + inv01 * y[1];
    x[1] = inv10 * y[0] + inv11 * y[1];
    return x;
}

/** MMSE detector for 2×2 spatial multiplexing.
 *  x_hat = (H^H·H + σ²·I)^{-1} · H^H · y */
inline std::array<ComplexSample, 2> mmseDetect2x2(
    const std::array<ComplexSample, 2>& y,
    const std::array<std::array<ComplexSample, 2>, 2>& H,
    float noise_var)
{
    // A = H^H·H + σ²·I (2x2 Hermitian)
    ComplexSample a00 = std::conj(H[0][0])*H[0][0] + std::conj(H[1][0])*H[1][0]
                        + ComplexSample(noise_var, 0.f);
    ComplexSample a01 = std::conj(H[0][0])*H[0][1] + std::conj(H[1][0])*H[1][1];
    ComplexSample a10 = std::conj(a01);
    ComplexSample a11 = std::conj(H[0][1])*H[0][1] + std::conj(H[1][1])*H[1][1]
                        + ComplexSample(noise_var, 0.f);

    // b = H^H · y
    ComplexSample b0 = std::conj(H[0][0])*y[0] + std::conj(H[1][0])*y[1];
    ComplexSample b1 = std::conj(H[0][1])*y[0] + std::conj(H[1][1])*y[1];

    // Solve 2x2 A·x = b
    ComplexSample det = a00 * a11 - a01 * a10;
    std::array<ComplexSample, 2> x;
    if (std::abs(det) < 1e-9f) {
        x[0] = ComplexSample(0.f, 0.f);
        x[1] = ComplexSample(0.f, 0.f);
        return x;
    }
    x[0] = (a11 * b0 - a01 * b1) / det;
    x[1] = (a00 * b1 - a10 * b0) / det;
    return x;
}

} // namespace gw
