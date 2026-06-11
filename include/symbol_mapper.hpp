/**
 * @file symbol_mapper.hpp
 * @brief QAM constellation mapping / demapping with soft LLR output
 */
#pragma once

#include "types.hpp"
#include <vector>

namespace gw {

/** Bit-pattern <-> constellation-point labeling for square QAM.
 *
 *  Gray (default): adjacent constellation points differ in exactly one bit.
 *    Minimizes uncoded BER (a symbol error usually flips one bit) and makes
 *    each bit's max-log LLR depend on a single I/Q coordinate. That same
 *    separability flattens the BICM-ID demapper EXIT curve: feeding back the
 *    other bits' a-priori information cannot change a bit's max-log decision,
 *    so the iterative loop has nothing to exchange (I_E ~ const vs I_A).
 *
 *  AntiGray: a labeling whose Euclidean-nearest neighbors differ in MANY bits
 *    (maximizes the average Hamming distance between adjacent points). The
 *    first (prior-free) pass is worse, but every bit now COUPLES to the others
 *    through the constellation, giving the prior-aware demapper a positive-slope
 *    EXIT curve — the textbook BICM-ID-friendly labeling. Geometry, energy
 *    normalization and all LLR machinery are unchanged; only the level<->label
 *    permutation differs. BPSK/QPSK (1-2 bits) are identical to Gray.
 */
enum class Labeling : uint8_t { Gray = 0, AntiGray = 1 };

class SymbolMapper {
public:
    explicit SymbolMapper(Modulation mod, Labeling labeling = Labeling::Gray);

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

    /** Batch soft-demap with PER-SUBCARRIER noise variance. After ZF
     *  equalization each bin's effective noise is sigma^2/|H(k)|^2, so faded
     *  subcarriers must get a larger noise (smaller-magnitude LLRs). Pass one
     *  noise value per symbol; the LLR clamp is then applied at the correct
     *  per-bin scale. noise_var.size() must equal symbols.size(). */
    void demapSoft(const ComplexBuf& symbols,
                   const std::vector<float>& noise_var,
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

    /** Batch PWL soft-demap with PER-SUBCARRIER noise variance (see the
     *  demapSoft per-bin overload). noise_var.size() must equal symbols.size(). */
    void demapSoftPWL(const ComplexBuf& symbols,
                      const std::vector<float>& noise_var,
                      std::vector<float>& llrs) const;

    Modulation  modulation()     const { return mod_; }
    Labeling    labeling()       const { return labeling_; }
    size_t      bitsPerSymbol()  const { return bps_; }
    float       normalization()  const { return norm_; }
    const ComplexBuf& constellation() const { return const_; }

private:
    void initBPSK();
    void initQPSK();
    void initSquareQAM(size_t order);
    void initPWLTables();

    // Per-dimension labeling permutation for square QAM. label2level_[g] gives
    // the I/Q LEVEL index (0 = most positive coordinate) carried by the
    // bits_dim-bit label g; level2label_ is its inverse. For Gray these are the
    // standard Gray decode/encode; for AntiGray they are a fixed anti-Gray
    // permutation. Everything else (geometry, LLR math, PWL tables) reads the
    // labeling exclusively through these tables, so the rest of the code is
    // labeling-agnostic.
    void buildDimLabeling(size_t bits_dim);

    Modulation mod_;
    Labeling   labeling_;
    size_t     bps_;   // bits per symbol
    float      norm_;  // normalization factor (spacing = 2*norm_)
    ComplexBuf const_; // constellation points indexed by bit pattern
    std::vector<size_t> label2level_; // size grid: label g -> level k
    std::vector<size_t> level2label_; // size grid: level k -> label g

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

} // namespace gw
