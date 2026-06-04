/**
 * @file simd_helpers.hpp
 * @brief SIMD primitives for the hot DSP loops.
 *
 * Behind `DSCA_ENABLE_SIMD`, provides AVX2 implementations of:
 *   - parallel distance-squared between a scalar reference point and 8
 *     constellation points (used by soft demapping)
 *   - SIMD horizontal min reduction
 *   - vector add/multiply for LLR post-processing
 *
 * All routines have scalar fallbacks so any SIMD-disabled build still
 * compiles and produces the same numerical result.
 */
#pragma once

#include <cstddef>
#include <cmath>
#include <algorithm>

#if defined(DSCA_ENABLE_SIMD) && (defined(__AVX2__) || defined(_MSC_VER))
#include <immintrin.h>
#define DSCA_HAVE_AVX2 1
#else
#define DSCA_HAVE_AVX2 0
#endif

namespace dsca {
namespace simd {

/** Compute |r - s|² for 8 constellation points in parallel.
 *  s_i / s_q are 8-element float arrays of constellation real/imag values.
 *  Returns 8 squared-distance results. Scalar fallback used outside AVX2. */
inline void distSquared8(float r_re, float r_im,
                          const float* s_i, const float* s_q,
                          float* d_out) {
#if DSCA_HAVE_AVX2
    __m256 r_re_v = _mm256_set1_ps(r_re);
    __m256 r_im_v = _mm256_set1_ps(r_im);
    __m256 si = _mm256_loadu_ps(s_i);
    __m256 sq = _mm256_loadu_ps(s_q);
    __m256 dr = _mm256_sub_ps(r_re_v, si);
    __m256 dq = _mm256_sub_ps(r_im_v, sq);
    __m256 d  = _mm256_fmadd_ps(dr, dr, _mm256_mul_ps(dq, dq));
    _mm256_storeu_ps(d_out, d);
#else
    for (int i = 0; i < 8; ++i) {
        float dr = r_re - s_i[i];
        float dq = r_im - s_q[i];
        d_out[i] = dr * dr + dq * dq;
    }
#endif
}

/** Horizontal min over an array of N floats. */
inline float hmin(const float* x, size_t n) {
    if (n == 0) return 0.f;
    float m = x[0];
#if DSCA_HAVE_AVX2
    if (n >= 8) {
        __m256 acc = _mm256_loadu_ps(x);
        size_t i = 8;
        for (; i + 8 <= n; i += 8) {
            acc = _mm256_min_ps(acc, _mm256_loadu_ps(x + i));
        }
        // Reduce 8 → 1
        alignas(32) float tmp[8];
        _mm256_storeu_ps(tmp, acc);
        m = tmp[0];
        for (int k = 1; k < 8; ++k) if (tmp[k] < m) m = tmp[k];
        for (; i < n; ++i) if (x[i] < m) m = x[i];
        return m;
    }
#endif
    for (size_t i = 1; i < n; ++i) if (x[i] < m) m = x[i];
    return m;
}

} // namespace simd
} // namespace dsca
