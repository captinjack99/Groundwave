/**
 * @file fft_engine.cpp
 * @brief FFT engine — built-in radix-2 DIT Cooley-Tukey
 *
 * Clean implementation with precomputed twiddle factors.
 * Can be swapped for FFTW3/Kiss FFT later for SIMD performance.
 */

#include "fft_engine.hpp"
#include <cmath>
#include <stdexcept>
#include <algorithm>

#ifdef GW_USE_FFTW3
#include <fftw3.h>
#endif

namespace gw {

#ifdef GW_USE_FFTW3

// =========================================================================
// Impl — FFTW3 single-precision backend
// =========================================================================

struct FFTEngine::Impl {
    size_t N;
    fftwf_plan plan_fwd;
    fftwf_plan plan_inv;
    fftwf_complex* in;
    fftwf_complex* out;

    explicit Impl(size_t n) : N(n) {
        in  = fftwf_alloc_complex(N);
        out = fftwf_alloc_complex(N);
        plan_fwd = fftwf_plan_dft_1d(static_cast<int>(N), in, out,
                                     FFTW_FORWARD, FFTW_MEASURE);
        plan_inv = fftwf_plan_dft_1d(static_cast<int>(N), in, out,
                                     FFTW_BACKWARD, FFTW_MEASURE);
    }
    ~Impl() {
        fftwf_destroy_plan(plan_fwd);
        fftwf_destroy_plan(plan_inv);
        fftwf_free(in);
        fftwf_free(out);
    }

    void transform(const ComplexBuf& input, ComplexBuf& output,
                   bool inverse, float scale) {
        output.resize(N);
        for (size_t i = 0; i < N; ++i) {
            in[i][0] = input[i].real();
            in[i][1] = input[i].imag();
        }
        fftwf_execute_dft(inverse ? plan_inv : plan_fwd, in, out);
        for (size_t i = 0; i < N; ++i) {
            output[i] = ComplexSample(out[i][0] * scale, out[i][1] * scale);
        }
    }
};

#else

// =========================================================================
// Impl — precomputed twiddle factors for radix-2 DIT
// =========================================================================

struct FFTEngine::Impl {
    size_t N;
    ComplexBuf twiddles;     // W_N^k for forward FFT
    ComplexBuf twiddles_inv; // conjugate for inverse
    std::vector<size_t> bit_rev; // bit-reversal permutation

    explicit Impl(size_t n) : N(n) {
        // Precompute twiddle factors: W_N^k = exp(-j*2*pi*k/N)
        twiddles.resize(N / 2);
        twiddles_inv.resize(N / 2);
        for (size_t k = 0; k < N / 2; ++k) {
            float angle = -2.0f * static_cast<float>(M_PI) * static_cast<float>(k) / static_cast<float>(N);
            twiddles[k] = ComplexSample(std::cos(angle), std::sin(angle));
            twiddles_inv[k] = std::conj(twiddles[k]);
        }

        // Precompute bit-reversal permutation
        size_t log2n = 0;
        for (size_t tmp = N; tmp > 1; tmp >>= 1) ++log2n;

        bit_rev.resize(N);
        for (size_t i = 0; i < N; ++i) {
            size_t rev = 0;
            size_t val = i;
            for (size_t b = 0; b < log2n; ++b) {
                rev = (rev << 1) | (val & 1);
                val >>= 1;
            }
            bit_rev[i] = rev;
        }
    }

    void transform(const ComplexBuf& input, ComplexBuf& output,
                   const ComplexBuf& tw, float scale) {
        output.resize(N);

        // Bit-reversal copy
        for (size_t i = 0; i < N; ++i) {
            output[bit_rev[i]] = input[i];
        }

        // Butterfly stages
        for (size_t half = 1; half < N; half <<= 1) {
            size_t step = half << 1;
            size_t tw_step = N / step; // twiddle index stride

            for (size_t group = 0; group < N; group += step) {
                for (size_t k = 0; k < half; ++k) {
                    ComplexSample w = tw[k * tw_step];
                    ComplexSample u = output[group + k];
                    ComplexSample t = w * output[group + k + half];
                    output[group + k]        = u + t;
                    output[group + k + half] = u - t;
                }
            }
        }

        // Apply scaling
        if (scale != 1.0f) {
            for (size_t i = 0; i < N; ++i) {
                output[i] *= scale;
            }
        }
    }
};

#endif // GW_USE_FFTW3

// =========================================================================
// Public API
// =========================================================================

FFTEngine::FFTEngine(size_t size) : size_(size) {
    if (!isValidSize(size)) {
        throw std::invalid_argument("FFT size must be power of 2, >= 4");
    }
    impl_ = std::make_unique<Impl>(size);
}

FFTEngine::~FFTEngine() = default;
FFTEngine::FFTEngine(FFTEngine&&) noexcept = default;
FFTEngine& FFTEngine::operator=(FFTEngine&&) noexcept = default;

void FFTEngine::forward(const ComplexBuf& input, ComplexBuf& output) {
    if (input.size() < size_) {
        throw std::invalid_argument("Input buffer too small for FFT");
    }
#ifdef GW_USE_FFTW3
    impl_->transform(input, output, /*inverse=*/false, 1.0f);
#else
    impl_->transform(input, output, impl_->twiddles, 1.0f);
#endif
}

void FFTEngine::inverse(const ComplexBuf& input, ComplexBuf& output) {
    if (input.size() < size_) {
        throw std::invalid_argument("Input buffer too small for IFFT");
    }
#ifdef GW_USE_FFTW3
    impl_->transform(input, output, /*inverse=*/true,
                     1.0f / static_cast<float>(size_));
#else
    impl_->transform(input, output, impl_->twiddles_inv,
                     1.0f / static_cast<float>(size_));
#endif
}

bool FFTEngine::isValidSize(size_t n) {
    return n >= 4 && (n & (n - 1)) == 0;
}

} // namespace gw
