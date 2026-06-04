/**
 * @file fft_engine.hpp
 * @brief FFT engine using bundled Kiss FFT
 */
#pragma once

#include "types.hpp"
#include <memory>

namespace dsca {

class FFTEngine {
public:
    explicit FFTEngine(size_t size);
    ~FFTEngine();

    FFTEngine(FFTEngine&&) noexcept;
    FFTEngine& operator=(FFTEngine&&) noexcept;
    FFTEngine(const FFTEngine&) = delete;
    FFTEngine& operator=(const FFTEngine&) = delete;

    /** Forward FFT: time → frequency */
    void forward(const ComplexBuf& input, ComplexBuf& output);

    /** Inverse FFT: frequency → time (normalized by 1/N) */
    void inverse(const ComplexBuf& input, ComplexBuf& output);

    size_t size() const { return size_; }

    static bool isValidSize(size_t n);

private:
    size_t size_;
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace dsca
