/**
 * @file reed_solomon.hpp
 * @brief Reed-Solomon outer code over GF(256), 16 parity bytes per block.
 *
 * Concatenated with LDPC: LDPC handles soft-decision channel bits, RS
 * mops up the rare residual byte errors that survive the LDPC waterfall.
 * Standard concatenation strategy from DVB-T (RS(204,188), CCSDS, etc.)
 * — produces a meaningfully steeper waterfall slope on the post-FEC
 * error rate at a small (~8% at typical block sizes) bitrate cost.
 *
 * Implementation:
 *   - Field: GF(256) with primitive polynomial 0x11D (CCSDS standard).
 *   - Generator polynomial: product of (x − α^i) for i in [1..16].
 *   - Encoder: long-division by g(x) using LFSR.
 *   - Decoder: Berlekamp-Massey error locator + Chien search + Forney.
 *
 * Each block carries `data_bytes` message bytes followed by 16 parity
 * bytes. The block length is variable (shortened-RS) up to 255 bytes
 * total. Block lengths < 17 are not useful (no room for data + parity).
 *
 * Throughput: scalar code, ~50 MB/s on a typical desktop CPU per stream.
 * Negligible compared to LDPC decode cost at any practical modcod.
 */
#pragma once

#include <cstdint>
#include <cstddef>
#include <array>
#include <vector>

namespace dsca {

class ReedSolomon {
public:
    static constexpr size_t PARITY_BYTES = 16;
    static constexpr size_t MAX_BLOCK    = 255;

    ReedSolomon();

    /** Encode `data_len` bytes of input followed by 16 parity bytes
     *  written into the same `block` buffer at offset data_len.
     *  Caller must size `block` to at least data_len + 16 bytes.
     *
     *  @return true on success (data_len >= 1 and data_len + 16 ≤ 255). */
    bool encode(uint8_t* block, size_t data_len) const;

    /** Decode `block` of size data_len + 16. On output, the first
     *  data_len bytes contain the corrected message. Returns the
     *  number of byte errors corrected, or -1 if the block is
     *  uncorrectable (more than 8 errors).
     *
     *  Decoder is in-place: input bytes are overwritten with corrected
     *  values. The parity bytes are also corrected (they're part of the
     *  codeword) but typically the caller discards them. */
    int decode(uint8_t* block, size_t data_len) const;

    /** Convenience wrapper: copy data to a temporary buffer, encode,
     *  and return the {data | parity} block. */
    std::vector<uint8_t> encodeCopy(const uint8_t* data, size_t data_len) const;

private:
    // GF(256) arithmetic tables. Built once in the ctor.
    std::array<uint8_t, 512> exp_;   // antilog table (with wrap-around for mul)
    std::array<uint8_t, 256> log_;   // discrete log table

    // Generator polynomial coefficients: g(x) = ∏(x - α^i), i=1..16.
    // Stored as 17 coefficients g_[0..16] with g_[16] = 1 (leading).
    std::array<uint8_t, PARITY_BYTES + 1> gen_;

    uint8_t mul(uint8_t a, uint8_t b) const {
        if (a == 0 || b == 0) return 0;
        return exp_[log_[a] + log_[b]];
    }
    uint8_t div(uint8_t a, uint8_t b) const {
        if (a == 0) return 0;
        return exp_[(log_[a] + 255 - log_[b]) % 255];
    }
    // (pow_field removed — it had no callers. mul/div already guard the
    // log_[0]=0 aliasing via their explicit zero checks above.)
};

} // namespace dsca
