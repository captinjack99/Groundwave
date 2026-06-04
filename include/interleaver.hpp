/**
 * @file interleaver.hpp
 * @brief Block bit interleaver for spreading burst errors
 *
 * Writes bits column-wise into a matrix, reads row-wise (or vice versa).
 * This spreads consecutive bit errors (from fading or QAM decision errors)
 * across different positions in the LDPC codeword, where the decoder
 * can correct them more effectively.
 */
#pragma once

#include "types.hpp"
#include <vector>

namespace dsca {

class BitInterleaver {
public:
    /** Create interleaver for given block size.
     *  @param num_bits  Number of bits in one interleaving block
     *  @param num_cols  Number of columns in the interleaving matrix
     *                   (rows = ceil(num_bits / num_cols)) */
    BitInterleaver(size_t num_bits, size_t num_cols = 0);

    /** Interleave: write column-wise, read row-wise */
    void interleave(const uint8_t* in, uint8_t* out) const;

    /** De-interleave: inverse permutation */
    void deinterleave(const uint8_t* in, uint8_t* out) const;

    /** Interleave soft values (LLRs) */
    void interleave(const float* in, float* out) const;

    /** De-interleave soft values (LLRs) */
    void deinterleave(const float* in, float* out) const;

    size_t numBits() const { return n_; }
    size_t rows() const { return rows_; }
    size_t cols() const { return cols_; }

private:
    size_t n_      = 0;
    size_t rows_   = 0;
    size_t cols_   = 0;
    std::vector<size_t> perm_;     // forward permutation
    std::vector<size_t> inv_perm_; // inverse permutation
};

} // namespace dsca
