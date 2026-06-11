/**
 * @file wav_io.hpp
 * @brief Minimal WAV file reader/writer (PCM 16-bit / float32).
 *
 * Header-only, no external dependencies. Used by CLI tools to ingest and
 * emit WAV files without pulling in libsndfile.
 */
#pragma once

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace gw {
namespace wav {

struct WavInfo {
    uint32_t sample_rate = 0;
    uint16_t channels    = 0;
    uint16_t bits        = 0;     ///< 16 or 32
    uint32_t num_frames  = 0;     ///< Frames (samples per channel)
    bool     is_float    = false;
};

inline uint32_t le32(const uint8_t* p) {
    return uint32_t(p[0]) | (uint32_t(p[1]) << 8) |
           (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24);
}

inline uint16_t le16(const uint8_t* p) {
    return uint16_t(p[0]) | (uint16_t(p[1]) << 8);
}

inline void put32(std::vector<uint8_t>& v, uint32_t x) {
    v.push_back(uint8_t(x));
    v.push_back(uint8_t(x >> 8));
    v.push_back(uint8_t(x >> 16));
    v.push_back(uint8_t(x >> 24));
}

inline void put16(std::vector<uint8_t>& v, uint16_t x) {
    v.push_back(uint8_t(x));
    v.push_back(uint8_t(x >> 8));
}

/** Read a WAV file as interleaved float samples in [-1, 1].
 *  Supports PCM 16-bit and IEEE float 32-bit. */
inline bool readFloat(const std::string& path, std::vector<float>& out, WavInfo& info) {
    FILE* f = std::fopen(path.c_str(), "rb");
    if (!f) return false;
    std::fseek(f, 0, SEEK_END);
    long fsize = std::ftell(f);
    std::fseek(f, 0, SEEK_SET);
    if (fsize < 44) { std::fclose(f); return false; }
    std::vector<uint8_t> buf(static_cast<size_t>(fsize));
    if (std::fread(buf.data(), 1, buf.size(), f) != buf.size()) {
        std::fclose(f); return false;
    }
    std::fclose(f);
    if (std::memcmp(buf.data(), "RIFF", 4) != 0) return false;
    if (std::memcmp(buf.data() + 8, "WAVE", 4) != 0) return false;

    size_t pos = 12;
    bool fmt_seen = false;
    uint16_t fmt_code = 0;
    size_t data_off = 0, data_len = 0;
    while (pos + 8 <= buf.size()) {
        uint32_t cklen = le32(&buf[pos + 4]);
        if (std::memcmp(&buf[pos], "fmt ", 4) == 0) {
            // Need at least 24 bytes from `pos` to read through the
            // bits-per-sample field; bail on a truncated fmt chunk rather
            // than reading past the buffer.
            if (pos + 24 > buf.size()) return false;
            fmt_code      = le16(&buf[pos + 8]);
            info.channels = le16(&buf[pos + 10]);
            info.sample_rate = le32(&buf[pos + 12]);
            info.bits     = le16(&buf[pos + 22]);
            info.is_float = (fmt_code == 3);
            fmt_seen = true;
        } else if (std::memcmp(&buf[pos], "data", 4) == 0) {
            data_off = pos + 8;
            // CRITICAL: clamp the chunk-declared length to what the file
            // actually contains. A malformed/truncated WAV can declare a
            // huge `data` length; trusting it drives an out-of-bounds heap
            // read in the sample-conversion loop below.
            size_t avail = (data_off <= buf.size()) ? buf.size() - data_off : 0;
            data_len = std::min<size_t>(cklen, avail);
            break;
        }
        // Advance by the chunk — RIFF chunks are WORD-aligned, so an
        // odd-sized chunk is followed by one pad byte (legal WAVs with an
        // odd-length LIST/bext chunk before `data` misparsed without it).
        // Guard against an overflowing/garbage cklen that would wrap or
        // stall the walk.
        size_t next = pos + 8 + static_cast<size_t>(cklen) + (cklen & 1u);
        if (next <= pos) break;           // overflow / zero-progress guard
        pos = next;
    }
    if (!fmt_seen || data_off == 0) return false;

    size_t bytes_per_sample = info.bits / 8;
    if (bytes_per_sample == 0) return false;
    size_t total_samples = data_len / bytes_per_sample;
    info.num_frames = static_cast<uint32_t>(total_samples / std::max<uint16_t>(1, info.channels));
    out.resize(total_samples);

    if (fmt_code == 1 && info.bits == 16) {
        const int16_t* src = reinterpret_cast<const int16_t*>(buf.data() + data_off);
        const float scale = 1.0f / 32768.0f;
        for (size_t i = 0; i < total_samples; ++i) out[i] = static_cast<float>(src[i]) * scale;
    } else if (fmt_code == 3 && info.bits == 32) {
        const float* src = reinterpret_cast<const float*>(buf.data() + data_off);
        std::memcpy(out.data(), src, total_samples * sizeof(float));
    } else {
        return false;
    }
    return true;
}

/** Write float samples [-1, 1] as 16-bit PCM WAV. */
inline bool writeFloat16(const std::string& path, const float* pcm,
                         size_t num_samples, uint32_t sample_rate, uint16_t channels) {
    std::vector<uint8_t> hdr;
    hdr.reserve(44);
    uint32_t data_bytes = static_cast<uint32_t>(num_samples * sizeof(int16_t));
    uint32_t fmt_size = 16;
    uint32_t total_size = 4 + (8 + fmt_size) + (8 + data_bytes);
    hdr.insert(hdr.end(), {'R','I','F','F'});
    put32(hdr, total_size);
    hdr.insert(hdr.end(), {'W','A','V','E'});
    hdr.insert(hdr.end(), {'f','m','t',' '});
    put32(hdr, fmt_size);
    put16(hdr, 1);                              // PCM
    put16(hdr, channels);
    put32(hdr, sample_rate);
    uint16_t block_align = channels * sizeof(int16_t);
    put32(hdr, sample_rate * block_align);
    put16(hdr, block_align);
    put16(hdr, 16);
    hdr.insert(hdr.end(), {'d','a','t','a'});
    put32(hdr, data_bytes);

    FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) return false;
    std::fwrite(hdr.data(), 1, hdr.size(), f);
    std::vector<int16_t> i16(num_samples);
    for (size_t i = 0; i < num_samples; ++i) {
        float v = std::max(-1.f, std::min(1.f, pcm[i]));
        i16[i] = static_cast<int16_t>(v * 32767.f);
    }
    std::fwrite(i16.data(), sizeof(int16_t), num_samples, f);
    std::fclose(f);
    return true;
}

} // namespace wav
} // namespace gw
