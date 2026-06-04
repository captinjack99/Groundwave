/**
 * @file zadoff_chu.hpp
 * @brief Zadoff–Chu sequences for preamble / sync.
 *
 * ZC sequences have constant amplitude and zero cyclic autocorrelation, which
 * makes them ideal for OFDM preamble synchronization. For prime length N
 * and root q coprime to N:
 *
 *   x[k] = exp(-j · π · q · k · (k + N mod 2) / N),  k = 0..N-1
 *
 * For non-prime N you lose the zero-autocorrelation property at some lags,
 * but ZC is still useful when N is a power of two (with care).
 *
 * The two forms below produce length-N sequences for any N (the standard
 * even/odd-length variants of the formula).
 */
#pragma once

#include "types.hpp"
#include <cmath>
#include <vector>
#include <numeric>

namespace dsca {

inline ComplexBuf zadoffChu(size_t N, size_t root = 1) {
    if (N == 0) return {};
    ComplexBuf out(N);
    double pi = M_PI;
    bool even = (N % 2 == 0);
    for (size_t k = 0; k < N; ++k) {
        double phase;
        double qk = static_cast<double>(root) * static_cast<double>(k);
        if (even) {
            // x[k] = exp(-j · π · q · k² / N)
            phase = -pi * qk * static_cast<double>(k) / static_cast<double>(N);
        } else {
            // x[k] = exp(-j · π · q · k · (k+1) / N)
            phase = -pi * qk * static_cast<double>(k + 1) / static_cast<double>(N);
        }
        out[k] = ComplexSample(static_cast<float>(std::cos(phase)),
                                static_cast<float>(std::sin(phase)));
    }
    return out;
}

/** Compute the optimal ZC root for a given N, picking a root coprime to N
 *  closest to N/2 (best off-peak suppression in the autocorrelation). */
inline size_t zadoffChuOptimalRoot(size_t N) {
    if (N <= 1) return 1;
    size_t target = N / 2;
    for (size_t step = 0; step < N; ++step) {
        size_t up = target + step;
        size_t dn = (target > step) ? target - step : 0;
        if (up < N && std::gcd(up, N) == 1) return up;
        if (dn > 0 && dn < N && std::gcd(dn, N) == 1) return dn;
    }
    return 1;
}

} // namespace dsca
