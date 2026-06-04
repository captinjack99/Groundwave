/**
 * @file orbgrand.hpp
 * @brief ORBGRAND (Ordered Reliability Bits GRAND) decoder for DSCA-NG v2
 *
 * Near-ML universal decoder for any linear block code, designed as a drop-in
 * replacement for the normalized min-sum BP decoder at short block lengths.
 *
 * Reference:
 *   Duffy, Solomon, Médard — "Ordered Reliability Bits GRAND"
 *   (IEEE Trans. Signal Processing, 2022)
 *
 * Algorithm:
 *   1. Hard decision from channel LLRs
 *   2. Sort bit positions by |LLR| ascending (least reliable first)
 *   3. Compute syndrome of hard decision
 *   4. Enumerate candidate noise patterns in logistic-weight order
 *      (sum of reliability-sorted indices, lower Hamming weight first)
 *   5. For each candidate: XOR relevant syndrome columns, check == 0
 *   6. First valid codeword is returned (near-ML guarantee)
 *
 * Advantages over BP at n=2160:
 *   - Near-ML performance (~1-2 dB gain over normalized min-sum)
 *   - No Tanner graph cycles to worry about
 *   - Deterministic worst-case runtime (hard query cap)
 *   - Works with ANY linear code — encoder is unchanged
 *
 * Complexity:
 *   - Per candidate: O(max_column_weight) for syndrome update
 *   - Total: O(max_queries × max_column_weight)
 *   - At high SNR (few errors), converges very quickly
 */
#pragma once

#include "types.hpp"
#include "ldpc.hpp"
#include <vector>
#include <memory>

namespace dsca {

// =========================================================================
// ORBGRAND Decoder
// =========================================================================

struct ORBGRANDConfig {
    size_t max_queries   = 5000;  ///< Hard cap on syndrome queries (RT guarantee)
    size_t max_weight    = 4;     ///< Max Hamming weight of error patterns
    size_t max_lw        = 0;     ///< Max logistic weight (0 = auto from max_queries)
};

class ORBGRANDDecoder {
public:
    ORBGRANDDecoder(FECRate rate, LDPCBlockSize blk,
                    const ORBGRANDConfig& cfg = {});

    /** Decode from channel LLRs, returning info bits.
     *  @param llr_in   Input LLRs (n values, positive = bit 0 more likely)
     *  @param info_out Decoded info bits as packed bytes, ceil(k/8) bytes
     *  @return Decode result (converged, iterations=queries used) */
    LDPCDecodeResult decode(const float* llr_in, uint8_t* info_out);

    /** Decode from channel LLRs, returning full codeword.
     *  @param llr_in    Input LLRs (n values)
     *  @param hard_out  Decoded codeword as packed bytes, ceil(n/8) bytes */
    LDPCDecodeResult decodeFull(const float* llr_in, uint8_t* hard_out);

    size_t infoBits()     const { return H_->k; }
    size_t codewordBits() const { return H_->n; }
    size_t parityChecks() const { return H_->m; }

    const LDPCMatrix& matrix() const { return *H_; }

private:
    // Syndrome helpers
    void computeSyndrome(const std::vector<uint8_t>& hard,
                         std::vector<uint8_t>& syndrome,
                         size_t& unsatisfied) const;

    void flipBit(std::vector<uint8_t>& hard, size_t bit_idx,
                 std::vector<uint8_t>& syndrome,
                 size_t& unsatisfied) const;

    // Pattern enumeration
    bool enumeratePatterns(std::vector<uint8_t>& hard,
                           std::vector<uint8_t>& syndrome,
                           size_t& unsatisfied,
                           LDPCDecodeResult& result);

    // Weight-specific pattern generators (return true if codeword found)
    bool tryWeight1(std::vector<uint8_t>& hard,
                    std::vector<uint8_t>& syndrome,
                    size_t& unsatisfied,
                    size_t lw, size_t& queries);

    bool tryWeight2(std::vector<uint8_t>& hard,
                    std::vector<uint8_t>& syndrome,
                    size_t& unsatisfied,
                    size_t lw, size_t& queries);

    bool tryWeight3(std::vector<uint8_t>& hard,
                    std::vector<uint8_t>& syndrome,
                    size_t& unsatisfied,
                    size_t lw, size_t& queries);

    bool tryWeight4(std::vector<uint8_t>& hard,
                    std::vector<uint8_t>& syndrome,
                    size_t& unsatisfied,
                    size_t lw, size_t& queries);

    std::shared_ptr<LDPCMatrix> H_;
    ORBGRANDConfig cfg_;

    // Pre-allocated working storage
    std::vector<size_t> sorted_indices_;   ///< Reliability-sorted: index into original bits
    std::vector<float>  sorted_absllr_;    ///< |LLR| for sorted positions
    std::vector<uint8_t> hard_bits_;       ///< Working hard decision (n bits packed)
    std::vector<uint8_t> syndrome_;        ///< Working syndrome (m bits)
};

} // namespace dsca
