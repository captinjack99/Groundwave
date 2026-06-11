/**
 * @file soft_decoder.hpp
 * @brief Minimal soft-input/soft-output decoder interface for BICM-ID.
 *
 * BICM-ID (see bicm.hpp) only ever calls three methods on its inner decoder:
 *   - decodePosterior() — per-bit a-posteriori LLRs for extrinsic feedback
 *   - codewordBits()    — n, the full codeword length
 *   - infoBits()        — k, the systematic info length
 *
 * Both the normalized min-sum BP decoder (LDPCDecoder) and the soft-output
 * GRAND decoder (ORBGRANDDecoder) implement exactly this contract, so the
 * orchestrator can hold an ISoftDecoder* and use either interchangeably.
 *
 * The interface is intentionally tiny; these are per-codeword calls, so the
 * virtual-dispatch overhead is negligible.
 */
#pragma once

#include "types.hpp"
#include <vector>

namespace gw {

struct LDPCDecodeResult;  // defined in ldpc.hpp

/** Soft-input/soft-output decoder contract consumed by BICMDecoder. */
class ISoftDecoder {
public:
    virtual ~ISoftDecoder() = default;

    /** Decode and return full per-bit POSTERIOR LLRs (size n).
     *  post_out[i] > 0  =>  bit i more likely 0.
     *  @param llr_in   Input channel LLRs (n values, positive = bit 0).
     *  @param post_out Output posterior LLRs (resized to n as needed).
     *  @return Decode result (converged / iterations / avg_magnitude). */
    virtual LDPCDecodeResult decodePosterior(const float* llr_in,
                                             std::vector<float>& post_out) = 0;

    /** Number of systematic information bits (k). */
    virtual size_t infoBits() const = 0;

    /** Number of codeword bits (n). */
    virtual size_t codewordBits() const = 0;
};

} // namespace gw
