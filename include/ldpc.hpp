/**
 * @file ldpc.hpp
 * @brief LDPC Forward Error Correction for DSCA-NG v2
 *
 * Quasi-Cyclic LDPC with IRA (Irregular Repeat-Accumulate) structure:
 *   H = [ H_info | H_parity ]
 * where H_parity has a staircase (dual-diagonal) structure enabling
 * linear-time systematic encoding.
 *
 * Block sizes chosen for soundcard modem latency:
 *   Short:  n=2160 bits (270 bytes) — ~3 OFDM symbols @ QAM16/FFT256
 *   Normal: n=8640 bits (1080 bytes) — ~11 OFDM symbols
 *
 * Decoder: Normalized min-sum belief propagation.
 */
#pragma once

#include "types.hpp"
#include <vector>
#include <memory>

namespace dsca {

// =========================================================================
// Block Size
// =========================================================================

enum class LDPCBlockSize : uint8_t {
    Short  = 0,  // n=2160  (q=90)
    Normal = 1,  // n=8640  (q=360)
};

// =========================================================================
// Code-construction method
// =========================================================================
//
// Legacy:     the original LCG-random base-matrix generator. For the
//             high rates (8/9, 9/10) the base has only m_base=3 check rows,
//             so every info column connects all 3 rows — a dense,
//             repetition-like protograph riddled with unavoidable lifted
//             4-cycles → high error floor (audit #11 / SOTA-1).
//
// Protograph: a girth-conditioned QC-LDPC construction. For the degenerate
//             high rates it uses a TALLER base (more check rows, smaller
//             circulant q) carrying the SAME lifted dimensions (n, k, m are
//             bit-identical to Legacy for every rate) so nothing downstream
//             changes, while the finer base gives the shift-assignment room
//             to eliminate 4-cycles (girth >= 6). The IRA staircase parity
//             is unchanged, so the systematic encoder is identical and
//             Hc^T = 0 holds by construction.
enum class LDPCConstruction : uint8_t {
    Legacy     = 0,
    Protograph = 1,
};

// =========================================================================
// LDPC Matrix (sparse H)
// =========================================================================

struct LDPCMatrix {
    size_t n;  // codeword bits
    size_t k;  // information bits
    size_t m;  // parity bits (n - k)
    size_t q;  // circulant expansion factor

    // Sparse H: rows[check_idx] = list of variable indices
    std::vector<std::vector<uint32_t>> rows;
    // Sparse H: cols[var_idx] = list of check indices
    std::vector<std::vector<uint32_t>> cols;
};

/** Build LDPC matrix for given rate and block size.
 *  Deterministic — always produces the same matrix for the same params.
 *  @param construction  Legacy or girth-conditioned Protograph (see enum).
 *         All encoder/decoder paths default to the same value so a frame's
 *         encoder and every decoder always share one H. */
std::shared_ptr<LDPCMatrix> buildLDPCMatrix(
    FECRate rate, LDPCBlockSize blk,
    LDPCConstruction construction = LDPCConstruction::Protograph);

/** Count the number of length-4 cycles (girth-4 events) in the lifted
 *  Tanner graph of H. A 4-cycle is a pair of variable nodes sharing two
 *  common check nodes; the count is sum over variable-pairs of
 *  C(common_checks, 2). Zero means girth >= 6. Diagnostic / test use. */
size_t countLDPCShortCycles(const LDPCMatrix& H);

// =========================================================================
// LDPC Encoder
// =========================================================================

class LDPCEncoder {
public:
    LDPCEncoder(FECRate rate, LDPCBlockSize blk,
                LDPCConstruction construction = LDPCConstruction::Protograph);

    /** Systematic encode: first k bits = info, remaining m bits = parity.
     *  @param info  Packed bytes, at least ceil(k/8) bytes
     *  @param cw    Output packed bytes, at least ceil(n/8) bytes
     *  @return true on success */
    bool encode(const uint8_t* info, uint8_t* cw) const;

    size_t infoBits()      const { return H_->k; }
    size_t codewordBits()  const { return H_->n; }
    size_t parityBits()    const { return H_->m; }
    size_t infoBytes()     const { return (H_->k + 7) / 8; }
    size_t codewordBytes() const { return (H_->n + 7) / 8; }
    float  codeRate()      const { return float(H_->k) / float(H_->n); }

    const LDPCMatrix& matrix() const { return *H_; }

private:
    std::shared_ptr<LDPCMatrix> H_;
};

// =========================================================================
// LDPC Decoder (Normalized Min-Sum BP)
// =========================================================================

struct LDPCDecodeResult {
    bool   converged  = false;  // all parity checks satisfied
    size_t iterations = 0;
    float  avg_magnitude = 0.f; // average |LLR| after decoding (confidence)
};

class LDPCDecoder {
public:
    LDPCDecoder(FECRate rate, LDPCBlockSize blk, size_t max_iter = 50,
                LDPCConstruction construction = LDPCConstruction::Protograph);

    /** Decode from channel LLRs.
     *  @param llr_in   Input LLRs (n values, positive = bit 0 more likely)
     *  @param info_out Decoded info bits as packed bytes, ceil(k/8) bytes
     *  @return Decode result (converged, iterations) */
    LDPCDecodeResult decode(const float* llr_in, uint8_t* info_out);

    /** Decode from channel LLRs, returning hard bits for full codeword.
     *  @param llr_in    Input LLRs (n values)
     *  @param hard_out  Decoded codeword as packed bytes, ceil(n/8) bytes */
    LDPCDecodeResult decodeFull(const float* llr_in, uint8_t* hard_out);

    /** Decode and return full per-bit POSTERIOR LLRs (size n).
     *  Used by BICM-ID to compute extrinsic feedback to the demapper.
     *  Posterior[i] = channel_llr[i] + sum(check-to-variable messages incoming to var i).
     *
     *  @param llr_in   Input LLRs (n values)
     *  @param post_out Output posterior LLRs (size n; resized as needed)
     *  @return Decode result with converged/iterations/avg_magnitude. */
    LDPCDecodeResult decodePosterior(const float* llr_in,
                                      std::vector<float>& post_out);

    size_t infoBits()     const { return H_->k; }
    size_t codewordBits() const { return H_->n; }

    const LDPCMatrix& matrix() const { return *H_; }

private:
    bool checkParity(const std::vector<uint8_t>& hard) const;
    void hardDecision(const std::vector<float>& llr,
                      std::vector<uint8_t>& hard) const;

    std::shared_ptr<LDPCMatrix> H_;
    size_t max_iter_;
    float  norm_factor_; // min-sum normalization (0.75 typical)

    // Working storage (pre-allocated)
    std::vector<float>         posterior_;      // n
    std::vector<std::vector<float>> msg_c2v_;   // per check → variables
    std::vector<uint8_t>       hard_bits_;      // n packed bytes
};

} // namespace dsca
