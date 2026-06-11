/**
 * @file iq_io.hpp
 * @brief IQ recording and playback to/from interleaved float WAV files.
 *
 * Records the post-downconvert complex baseband as a 2-channel float32 WAV:
 *   channel 0 = I (real part)
 *   channel 1 = Q (imag part)
 *
 * This format is widely interoperable: GNU Radio, Inspectrum, SDR# etc. all
 * understand 2-channel float WAV as IQ. Header is the standard RIFF/WAVE.
 *
 * Use IQRecorder to dump the receive baseband for offline analysis.
 * Use IQPlayer to replay a recorded WAV as if it were live RX (drives the
 * loopback ring or a custom callback).
 */
#pragma once

#include "types.hpp"
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>
#include <mutex>

namespace gw {

class IQRecorder {
public:
    IQRecorder() = default;
    ~IQRecorder() { close(); }

    /** Open a WAV file for writing. Returns true on success.
     *  @param path        File path
     *  @param sample_rate Audio sample rate (Hz)  */
    bool open(const std::string& path, uint32_t sample_rate) {
        std::lock_guard<std::mutex> lk(mtx_);
        close_locked();
        f_ = std::fopen(path.c_str(), "wb");
        if (!f_) return false;
        sample_rate_ = sample_rate;
        // Write a placeholder header; we'll patch sizes on close.
        writeHeader(0);
        path_ = path;
        return true;
    }

    /** Append a chunk of complex baseband samples. */
    void write(const ComplexSample* samples, size_t n) {
        std::lock_guard<std::mutex> lk(mtx_);
        if (!f_ || n == 0) return;
        // Interleave I/Q into a temporary buffer
        std::vector<float> buf(2 * n);
        for (size_t i = 0; i < n; ++i) {
            buf[2*i + 0] = samples[i].real();
            buf[2*i + 1] = samples[i].imag();
        }
        std::fwrite(buf.data(), sizeof(float), buf.size(), f_);
        frame_count_ += n;
    }

    /** Close the file, patching the RIFF/data sizes in the header. */
    void close() {
        std::lock_guard<std::mutex> lk(mtx_);
        close_locked();
    }

    bool isOpen() const {
        std::lock_guard<std::mutex> lk(mtx_);
        return f_ != nullptr;
    }

    uint64_t frameCount() const {
        std::lock_guard<std::mutex> lk(mtx_);
        return frame_count_;
    }

private:
    void writeHeader(uint32_t data_bytes) {
        if (!f_) return;
        const uint16_t channels = 2;
        const uint16_t bits     = 32;
        const uint16_t fmt_code = 3;   // IEEE float
        const uint32_t byte_rate = sample_rate_ * channels * (bits / 8);
        const uint16_t block_align = channels * (bits / 8);

        std::fseek(f_, 0, SEEK_SET);
        std::fwrite("RIFF", 1, 4, f_);
        uint32_t riff_size = 36 + data_bytes;
        std::fwrite(&riff_size, 4, 1, f_);
        std::fwrite("WAVEfmt ", 1, 8, f_);
        uint32_t fmt_size = 16;
        std::fwrite(&fmt_size, 4, 1, f_);
        std::fwrite(&fmt_code, 2, 1, f_);
        std::fwrite(&channels, 2, 1, f_);
        std::fwrite(&sample_rate_, 4, 1, f_);
        std::fwrite(&byte_rate, 4, 1, f_);
        std::fwrite(&block_align, 2, 1, f_);
        std::fwrite(&bits, 2, 1, f_);
        std::fwrite("data", 1, 4, f_);
        std::fwrite(&data_bytes, 4, 1, f_);
    }

    void close_locked() {
        if (!f_) return;
        // Patch sizes
        uint32_t data_bytes = static_cast<uint32_t>(
            frame_count_ * 2 * sizeof(float));
        writeHeader(data_bytes);
        std::fclose(f_);
        f_ = nullptr;
        path_.clear();
        frame_count_ = 0;
    }

    mutable std::mutex mtx_;
    std::FILE* f_       = nullptr;
    uint32_t   sample_rate_ = 48000;
    std::string path_;
    uint64_t   frame_count_ = 0;
};

class IQPlayer {
public:
    /** Read an entire IQ WAV into memory. Returns false on error. */
    bool open(const std::string& path) {
        FILE* f = std::fopen(path.c_str(), "rb");
        if (!f) return false;
        uint8_t hdr[44];
        if (std::fread(hdr, 1, 44, f) != 44) { std::fclose(f); return false; }
        // Validate RIFF/WAVE
        if (std::memcmp(hdr, "RIFF", 4) != 0 ||
            std::memcmp(hdr + 8, "WAVE", 4) != 0) {
            std::fclose(f); return false;
        }
        uint16_t channels = static_cast<uint16_t>(hdr[22] | (hdr[23] << 8));
        uint32_t sr       = static_cast<uint32_t>(
            hdr[24] | (hdr[25] << 8) | (hdr[26] << 16) | (hdr[27] << 24));
        uint16_t bits     = static_cast<uint16_t>(hdr[34] | (hdr[35] << 8));
        // We only support 2ch float32
        if (channels != 2 || bits != 32) { std::fclose(f); return false; }
        // Find data chunk size
        uint32_t data_bytes = static_cast<uint32_t>(
            hdr[40] | (hdr[41] << 8) | (hdr[42] << 16) | (hdr[43] << 24));
        size_t total_samples = data_bytes / sizeof(float);
        std::vector<float> raw(total_samples);
        std::fread(raw.data(), sizeof(float), total_samples, f);
        std::fclose(f);

        samples_.resize(total_samples / 2);
        for (size_t i = 0; i < samples_.size(); ++i) {
            samples_[i] = ComplexSample(raw[2*i + 0], raw[2*i + 1]);
        }
        sample_rate_ = sr;
        pos_ = 0;
        return true;
    }

    /** Read up to n complex samples into out. Returns count read. */
    size_t read(ComplexSample* out, size_t n) {
        if (pos_ >= samples_.size()) return 0;
        size_t k = std::min(n, samples_.size() - pos_);
        for (size_t i = 0; i < k; ++i) out[i] = samples_[pos_ + i];
        pos_ += k;
        return k;
    }

    void rewind() { pos_ = 0; }
    bool atEnd() const { return pos_ >= samples_.size(); }
    uint32_t sampleRate() const { return sample_rate_; }
    size_t   total() const { return samples_.size(); }

private:
    std::vector<ComplexSample> samples_;
    uint32_t sample_rate_ = 48000;
    size_t   pos_         = 0;
};

} // namespace gw
