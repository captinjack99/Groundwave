/**
 * @file grand.hpp
 * @brief Basic GRAND (Guessing Random Additive Noise Decoding).
 *
 * Plain GRAND enumerates error patterns in order of decreasing a-priori
 * likelihood (Hamming weight, then natural ordering) and tests whether
 * received_word XOR error is a valid codeword. Unlike ORBGRAND, it does
 * not use soft information — pure hard-decision input.
 *
 * Use ORBGRAND when soft LLRs are available (much higher capacity).
 * Use plain GRAND for hard-decision inputs (saturation channels, simple
 * sliders, BICM with quantized LLRs).
 */
#pragma once

#include "types.hpp"
#include "ldpc.hpp"
#include "orbgrand.hpp"  // for LDPCDecodeResult & matrix builder
#include <vector>
#include <memory>

namespace dsca {

struct GRANDConfig {
    size_t max_weight  = 3;       ///< Max Hamming weight of error pattern
    size_t max_queries = 50000;   ///< Computational budget
};

class GRANDDecoder {
public:
    GRANDDecoder(FECRate rate, LDPCBlockSize blk,
                 const GRANDConfig& cfg = GRANDConfig())
        : cfg_(cfg)
    {
        H_ = buildLDPCMatrix(rate, blk);
        const size_t n = H_->n;
        hard_.resize((n + 7) / 8);
        syndrome_.resize(H_->m);
    }

    /** Decode hard-decision input (1 bit per byte input, MSB-first packed). */
    LDPCDecodeResult decodeHard(const uint8_t* hard_in, uint8_t* hard_out) {
        LDPCDecodeResult result;
        const size_t n = H_->n;
        const size_t m = H_->m;
        size_t n_bytes = (n + 7) / 8;
        std::memcpy(hard_.data(), hard_in, n_bytes);

        // Initial syndrome
        size_t unsat = computeSyndrome();
        size_t queries = 0;

        if (unsat == 0) {
            result.converged = true;
            std::memcpy(hard_out, hard_.data(), n_bytes);
            return result;
        }

        // Enumerate error patterns by Hamming weight
        for (size_t w = 1; w <= cfg_.max_weight && queries < cfg_.max_queries; ++w) {
            std::vector<size_t> idx(w, 0);
            for (size_t i = 0; i < w; ++i) idx[i] = i;
            while (true) {
                if (queries >= cfg_.max_queries) break;
                // Apply pattern
                for (size_t v : idx) flipBit(v);
                ++queries;
                if (computeSyndrome() == 0) {
                    result.converged = true;
                    result.iterations = queries;
                    std::memcpy(hard_out, hard_.data(), n_bytes);
                    return result;
                }
                // Unflip
                for (size_t v : idx) flipBit(v);
                // Advance pattern (lex order)
                size_t pos = w;
                while (pos > 0) {
                    --pos;
                    if (idx[pos] < n - (w - pos)) {
                        ++idx[pos];
                        for (size_t k = pos + 1; k < w; ++k) idx[k] = idx[k - 1] + 1;
                        break;
                    }
                    if (pos == 0) goto next_weight;
                }
            }
            next_weight:;
        }
        result.iterations = queries;
        std::memcpy(hard_out, hard_.data(), n_bytes);
        return result;
    }

    size_t infoBits() const { return H_->k; }
    size_t codewordBits() const { return H_->n; }

private:
    size_t computeSyndrome() {
        const size_t m = H_->m;
        size_t unsat = 0;
        for (size_t c = 0; c < m; ++c) {
            uint8_t s = 0;
            for (uint32_t v : H_->rows[c]) {
                s ^= (hard_[v >> 3] >> (7 - (v & 7))) & 1;
            }
            syndrome_[c] = s;
            if (s) ++unsat;
        }
        return unsat;
    }

    void flipBit(size_t v) {
        hard_[v >> 3] ^= static_cast<uint8_t>(1u << (7 - (v & 7)));
    }

    GRANDConfig cfg_;
    std::shared_ptr<LDPCMatrix> H_;
    std::vector<uint8_t> hard_;
    std::vector<uint8_t> syndrome_;
};

} // namespace dsca
