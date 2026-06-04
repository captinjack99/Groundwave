/**
 * @file bicm.hpp
 * @brief Bit-Interleaved Coded Modulation with iterative decoding (BICM-ID).
 *
 * Standard BICM transmit chain:
 *
 *   info → LDPC encode → bit interleaver → symbol mapper → channel
 *
 * Standard BICM receive (no iteration):
 *
 *   channel → soft demap → de-interleaver → LDPC decode
 *
 * BICM-ID adds an iteration loop:
 *
 *   1. Soft demap with prior = 0 (or last LDPC posterior)
 *   2. De-interleave LLRs
 *   3. LDPC decode (don't take final hard decision yet)
 *   4. Interleave LLRs back
 *   5. Soft demap again with prior LLRs as a-priori bit probabilities
 *   6. Loop or exit on convergence
 *
 * The gain comes from the demapper using prior bit information to
 * sharpen LLRs on bits that share a symbol with already-known bits.
 * Typical gain: 0.5–1.5 dB on QAM16+ at low-to-mid SNR.
 *
 * This module wraps the existing SymbolMapper + BitInterleaver + LDPCDecoder
 * to provide a clean orchestrated decode path.
 */
#pragma once

#include "types.hpp"
#include "symbol_mapper.hpp"
#include "interleaver.hpp"
#include "ldpc.hpp"
#include <vector>
#include <memory>
#include <cstring>
#include <algorithm>

namespace dsca {

struct BICMConfig {
    size_t outer_iterations = 1;     ///< 1 = standard BICM, >1 = iterative
    size_t ldpc_inner_iter  = 30;    ///< LDPC iterations per outer pass
    bool   use_extrinsic    = true;  ///< Subtract prior from LDPC posterior before feedback
};

/** Iterative-demapping BICM decoder.
 *  Holds borrowed pointers to mapper / interleaver / decoder. None are owned. */
class BICMDecoder {
public:
    BICMDecoder(SymbolMapper*   mapper,
                BitInterleaver* interleaver,
                LDPCDecoder*    decoder,
                const BICMConfig& cfg = BICMConfig())
        : mapper_(mapper)
        , interleaver_(interleaver)
        , decoder_(decoder)
        , cfg_(cfg) {}

    /** One-shot decode (no iteration) — equivalent to standard BICM. */
    LDPCDecodeResult decode(const ComplexBuf& syms, float noise_var,
                            uint8_t* info_out) {
        BICMConfig saved = cfg_;
        cfg_.outer_iterations = 1;
        auto r = decodeIterative(syms, noise_var, info_out);
        cfg_ = saved;
        return r;
    }

    /** Iterative BICM-ID decode.
     *  @param syms      Equalized symbols from OFDMDemodulator
     *  @param noise_var Noise variance estimate
     *  @param info_out  Output info bytes (ceil(k/8) bytes) */
    LDPCDecodeResult decodeIterative(const ComplexBuf& syms,
                                     float noise_var,
                                     uint8_t* info_out) {
        LDPCDecodeResult result;
        if (!mapper_ || !interleaver_ || !decoder_) return result;

        const size_t bps = mapper_->bitsPerSymbol();
        const size_t n_bits_chan = syms.size() * bps;
        const size_t n_cw = decoder_->codewordBits();
        if (n_bits_chan < n_cw) return result;

        // Buffers
        ch_llr_.assign(n_cw, 0.f);
        deint_llr_.assign(n_cw, 0.f);
        post_llr_.assign(n_cw, 0.f);
        prior_llr_.assign(n_bits_chan, 0.f);  // prior on channel-bit positions

        // ---- Initial soft demap (no prior) ----
        std::vector<float> demap_buf;
        mapper_->demapSoftPWL(syms, noise_var, demap_buf);
        std::copy_n(demap_buf.begin(), n_cw, ch_llr_.begin());

        // De-interleave channel LLRs to LDPC bit order
        interleaver_->deinterleave(ch_llr_.data(), deint_llr_.data());

        // ---- Outer iteration loop ----
        std::vector<float> posterior_llr;
        for (size_t it = 0; it < cfg_.outer_iterations; ++it) {
            // LDPC decode — get full per-bit posterior LLRs
            result = decoder_->decodePosterior(deint_llr_.data(), posterior_llr);

            // Snapshot hard decisions
            cw_bytes_.assign((n_cw + 7) / 8, 0);
            for (size_t b = 0; b < n_cw; ++b) {
                if (b < posterior_llr.size() && posterior_llr[b] < 0.f) {
                    cw_bytes_[b >> 3] |= static_cast<uint8_t>(1u << (7 - (b & 7)));
                }
            }

            if (result.converged || it + 1 == cfg_.outer_iterations) break;

            // True extrinsic feedback: posterior - input_to_decoder, in
            // de-interleaved (LDPC) bit order. Then re-interleave back to
            // channel-bit order for the demapper's prior.
            if (cfg_.use_extrinsic) {
                for (size_t b = 0; b < n_cw; ++b) {
                    post_llr_[b] = posterior_llr[b] - deint_llr_[b];
                }
            } else {
                std::copy_n(posterior_llr.begin(), n_cw, post_llr_.begin());
            }

            interleaver_->interleave(post_llr_.data(), prior_llr_.data());

            // Re-demap with the a-priori LLRs folded into the symbol-level
            // marginalization (true BICM-ID). Simply ADDING the prior to the
            // channel LLR — the previous approach — is a no-op for Gray maps
            // because each bit's max-log metric is unaffected by the other
            // bits' priors under independent per-bit decisions, so it never
            // delivered the advertised iterative gain. demapWithPriors
            // recomputes each bit's extrinsic LLR as
            //   max_{x:b_k=0}[-|y-x|²/2σ² + Σ_{j≠k} prior_j·(b_j?-½:+½)]
            //   − max_{x:b_k=1}[ … ],
            // which genuinely couples the layers through the constellation.
            demapWithPriors(syms, noise_var, prior_llr_, demap_buf);
            std::copy_n(demap_buf.begin(), n_cw, ch_llr_.begin());
            interleaver_->deinterleave(ch_llr_.data(), deint_llr_.data());
        }

        // Extract systematic info bits
        size_t info_bits = decoder_->infoBits();
        size_t info_bytes = (info_bits + 7) / 8;
        std::memcpy(info_out, cw_bytes_.data(), info_bytes);
        return result;
    }

    void setConfig(const BICMConfig& cfg) { cfg_ = cfg; }
    const BICMConfig& config() const { return cfg_; }

private:
    /** Prior-aware max-log-APP soft demap (the heart of BICM-ID). For each
     *  bit it marginalizes over the full constellation, folding in the
     *  a-priori LLRs of the OTHER bits in the same symbol. Output LLRs are
     *  extrinsic-style (the bit's own prior is excluded), positive ⇒ bit 0.
     *  O(M·bps) per symbol via a precomputed per-candidate base metric. */
    void demapWithPriors(const ComplexBuf& syms, float noise_var,
                         const std::vector<float>& prior,
                         std::vector<float>& out) {
        const ComplexBuf& C = mapper_->constellation();
        const size_t bps = mapper_->bitsPerSymbol();
        const size_t M   = C.size();
        out.assign(syms.size() * bps, 0.f);
        if (M == 0 || bps == 0) {
            // No constellation table (e.g. BPSK) — fall back to plain demap.
            mapper_->demapSoftPWL(syms, noise_var, out);
            return;
        }
        const float inv2s2 = 1.0f / (2.0f * std::max(noise_var, 1e-6f));
        base_.resize(M);
        for (size_t s = 0; s < syms.size(); ++s) {
            const ComplexSample y = syms[s];
            // base[idx] = -|y-x|²/2σ² + Σ_j prior_j contribution.
            // A-priori contribution to log P(symbol) for bit j:
            //   bit 0 → +L_j/2,  bit 1 → −L_j/2  (common term cancels in LLR).
            for (size_t idx = 0; idx < M; ++idx) {
                float m = -std::norm(y - C[idx]) * inv2s2;
                for (size_t j = 0; j < bps; ++j) {
                    float Lj = prior[s * bps + j];
                    bool  bj = (idx >> (bps - 1 - j)) & 1u;
                    m += bj ? (-0.5f * Lj) : (0.5f * Lj);
                }
                base_[idx] = m;
            }
            // Per bit: extrinsic LLR excludes the bit's own prior so we add
            // it back into the side it belongs to before maxing.
            for (size_t k = 0; k < bps; ++k) {
                float Lk = prior[s * bps + k];
                float max0 = -1e30f, max1 = -1e30f;
                for (size_t idx = 0; idx < M; ++idx) {
                    bool bk = (idx >> (bps - 1 - k)) & 1u;
                    // Remove bit-k's own prior contribution from base[idx]
                    // (extrinsic output convention).
                    float m = base_[idx] - (bk ? (-0.5f * Lk) : (0.5f * Lk));
                    if (bk) { if (m > max1) max1 = m; }
                    else    { if (m > max0) max0 = m; }
                }
                out[s * bps + k] = max0 - max1;   // positive ⇒ bit 0
            }
        }
    }

    SymbolMapper*   mapper_;
    BitInterleaver* interleaver_;
    LDPCDecoder*    decoder_;
    BICMConfig      cfg_;

    std::vector<float>   ch_llr_;
    std::vector<float>   deint_llr_;
    std::vector<float>   post_llr_;
    std::vector<float>   prior_llr_;
    std::vector<uint8_t> cw_bytes_;
    std::vector<float>   base_;       ///< per-candidate metric scratch (demapWithPriors)
};

} // namespace dsca
