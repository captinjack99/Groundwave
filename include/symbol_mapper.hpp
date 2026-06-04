/**
 * @file symbol_mapper.hpp
 * @brief QAM constellation mapping / demapping with soft LLR output
 */
#pragma once

#include "types.hpp"
#include <vector>

namespace dsca {

class SymbolMapper {
public:
    explicit SymbolMapper(Modulation mod);

    /** Map packed bytes → complex symbols (MSB first) */
    void mapBytes(const uint8_t* bytes, size_t num_bits, ComplexBuf& symbols) const;

    /** Hard-demap a single symbol → bit index */
    uint16_t demapHard(ComplexSample sym) const;

    /** Soft-demap: compute LLRs for all bits of one symbol (max-log-MAP) */
    void demapSoft(ComplexSample sym, float noise_var,
                   std::vector<float>& llrs) const;

    /** Batch soft-demap */
    void demapSoft(const ComplexBuf& symbols, float noise_var,
                   std::vector<float>& llrs) const;

    /** Piecewise-linear soft-demap: O(bps) per symbol instead of O(M*bps).
     *  Exploits Gray-coded rectangular QAM structure.
     *  For BPSK/QPSK: identical to demapSoft (already O(1)).
     *  For M-QAM (M>=16): uses precomputed breakpoints for each bit. */
    void demapSoftPWL(ComplexSample sym, float noise_var,
                      std::vector<float>& llrs) const;

    /** Batch piecewise-linear soft-demap */
    void demapSoftPWL(const ComplexBuf& symbols, float noise_var,
                      std::vector<float>& llrs) const;

    Modulation  modulation()     const { return mod_; }
    size_t      bitsPerSymbol()  const { return bps_; }
    float       normalization()  const { return norm_; }
    const ComplexBuf& constellation() const { return const_; }

private:
    void initBPSK();
    void initQPSK();
    void initSquareQAM(size_t order);
    void initPWLTables();

    Modulation mod_;
    size_t     bps_;   // bits per symbol
    float      norm_;  // normalization factor (spacing = 2*norm_)
    ComplexBuf const_; // constellation points indexed by bit pattern

    // Precomputed bit patterns for exact LLR calculation
    std::vector<std::vector<uint8_t>> bit_patterns_;

    // Piecewise-linear LLR data for each bit in each dimension
    // For square QAM: bps/2 bits in I, bps/2 bits in Q
    // Each bit has sorted breakpoints where LLR slope changes
    struct PWLSegment {
        float start;     // coordinate value where segment begins
        float slope;     // LLR per unit coordinate (multiply by 1/σ²)
        float intercept; // LLR value at start (multiply by 1/σ²)
    };
    // pwl_tables_[dim][bit] = sorted segments for that bit in that dimension
    // dim 0 = I (real), dim 1 = Q (imag)
    std::vector<std::vector<std::vector<PWLSegment>>> pwl_tables_;
    size_t grid_size_ = 0; // sqrt(M) for square QAM, 0 for BPSK/QPSK
};

} // namespace dsca
