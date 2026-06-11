/**
 * @file orbgrand.cpp
 * @brief ORBGRAND decoder implementation
 *
 * Pattern enumeration strategy (logistic weight ordering):
 *   For logistic weight LW = 0, 1, 2, ...:
 *     For Hamming weight w = 1, 2, 3, ... (up to max_weight):
 *       Enumerate all w-element index sets from the reliability-sorted
 *       bit positions whose indices sum to LW.
 *
 * Syndrome update optimization:
 *   Instead of recomputing H·c^T from scratch for each candidate, we
 *   maintain a running syndrome and toggle it when bits are flipped.
 *   Flipping bit j toggles syndrome[c] for all checks c in H->cols[j].
 *   We track the count of unsatisfied checks for O(1) convergence test.
 */

#include "orbgrand.hpp"
#include <algorithm>
#include <numeric>
#include <cstring>
#include <cmath>

namespace gw {

// =========================================================================
// Bit manipulation (same convention as ldpc.cpp)
// =========================================================================

namespace {

inline bool getBit(const uint8_t* d, size_t i) {
    return (d[i >> 3] >> (7 - (i & 7))) & 1;
}

inline void setBit(uint8_t* d, size_t i, bool v) {
    if (v) d[i >> 3] |=  (uint8_t(1) << (7 - (i & 7)));
    else   d[i >> 3] &= ~(uint8_t(1) << (7 - (i & 7)));
}

inline void toggleBit(uint8_t* d, size_t i) {
    d[i >> 3] ^= (uint8_t(1) << (7 - (i & 7)));
}

} // anonymous

// =========================================================================
// Constructor
// =========================================================================

ORBGRANDDecoder::ORBGRANDDecoder(FECRate rate, LDPCBlockSize blk,
                                   const ORBGRANDConfig& cfg)
    : cfg_(cfg)
{
    H_ = buildLDPCMatrix(rate, blk);

    const size_t n = H_->n;
    const size_t m = H_->m;

    sorted_indices_.resize(n);
    sorted_absllr_.resize(n);
    hard_bits_.resize((n + 7) / 8);
    syndrome_.resize(m);

    // Auto-compute max logistic weight if not set
    if (cfg_.max_lw == 0) {
        // Heuristic: for max_queries Q and max_weight W,
        // the number of patterns up to LW=L is roughly O(L^W / W!).
        // We want that to be around max_queries.
        // For W=4, Q=5000: L ≈ (5000 * 24)^(1/4) ≈ 18
        // For W=3, Q=5000: L ≈ (5000 * 6)^(1/3)  ≈ 31
        // Conservative: set to n/4 (capped by query budget anyway)
        cfg_.max_lw = n / 4;
    }
}

// =========================================================================
// Syndrome computation
// =========================================================================

void ORBGRANDDecoder::computeSyndrome(const std::vector<uint8_t>& hard,
                                       std::vector<uint8_t>& syndrome,
                                       size_t& unsatisfied) const {
    const size_t m = H_->m;
    unsatisfied = 0;

    for (size_t c = 0; c < m; ++c) {
        uint8_t s = 0;
        for (uint32_t v : H_->rows[c]) {
            s ^= getBit(hard.data(), v) ? 1 : 0;
        }
        syndrome[c] = s;
        if (s) ++unsatisfied;
    }
}

void ORBGRANDDecoder::flipBit(std::vector<uint8_t>& hard, size_t bit_idx,
                                std::vector<uint8_t>& syndrome,
                                size_t& unsatisfied) const {
    // Toggle the bit in the hard decision
    toggleBit(hard.data(), bit_idx);

    // Update syndrome: toggle every check that includes this bit
    for (uint32_t c : H_->cols[bit_idx]) {
        if (syndrome[c]) {
            syndrome[c] = 0;
            --unsatisfied;
        } else {
            syndrome[c] = 1;
            ++unsatisfied;
        }
    }
}

// =========================================================================
// Weight-specific pattern generators
//
// Each returns true if a valid codeword was found (unsatisfied == 0).
// They flip bits IN PLACE and unflip them before returning false,
// so the hard/syndrome state is restored on failure.
// =========================================================================

bool ORBGRANDDecoder::tryWeight1(std::vector<uint8_t>& hard,
                                  std::vector<uint8_t>& syndrome,
                                  size_t& unsatisfied,
                                  size_t lw, size_t& queries) {
    // Weight 1, index sum = lw → single index = lw
    const size_t n = H_->n;
    if (lw >= n) return false;
    if (queries >= cfg_.max_queries) return false;

    size_t real_idx = sorted_indices_[lw];
    flipBit(hard, real_idx, syndrome, unsatisfied);
    ++queries;

    if (unsatisfied == 0) return true;

    // Unflip
    flipBit(hard, real_idx, syndrome, unsatisfied);
    return false;
}

bool ORBGRANDDecoder::tryWeight2(std::vector<uint8_t>& hard,
                                  std::vector<uint8_t>& syndrome,
                                  size_t& unsatisfied,
                                  size_t lw, size_t& queries) {
    // Weight 2: pairs (a, b) where a < b, a + b = lw
    // Minimum lw for weight 2: 0 + 1 = 1
    const size_t n = H_->n;
    if (lw < 1) return false;

    // a ranges from 0 to (lw-1)/2, b = lw - a, need b < n and a < b
    size_t a_max = (lw - 1) / 2;

    for (size_t a = 0; a <= a_max; ++a) {
        if (queries >= cfg_.max_queries) return false;

        size_t b = lw - a;
        if (b >= n) continue;

        size_t real_a = sorted_indices_[a];
        size_t real_b = sorted_indices_[b];

        flipBit(hard, real_a, syndrome, unsatisfied);
        flipBit(hard, real_b, syndrome, unsatisfied);
        ++queries;

        if (unsatisfied == 0) return true;

        // Unflip
        flipBit(hard, real_b, syndrome, unsatisfied);
        flipBit(hard, real_a, syndrome, unsatisfied);
    }

    return false;
}

bool ORBGRANDDecoder::tryWeight3(std::vector<uint8_t>& hard,
                                  std::vector<uint8_t>& syndrome,
                                  size_t& unsatisfied,
                                  size_t lw, size_t& queries) {
    // Weight 3: triples (a, b, c) where a < b < c, a + b + c = lw
    // Minimum lw for weight 3: 0 + 1 + 2 = 3
    const size_t n = H_->n;
    if (lw < 3) return false;

    for (size_t a = 0; a * 3 + 3 <= lw; ++a) {
        // b + c = lw - a, b > a, b < c → b ranges from a+1 to (lw-a-1)/2
        size_t rem = lw - a;
        if (a + 1 > rem / 2) break;  // can't form valid b < c

        for (size_t b = a + 1; b * 2 < rem; ++b) {
            if (queries >= cfg_.max_queries) return false;

            size_t c = rem - b;
            if (c <= b) break;   // need c > b
            if (c >= n) continue;

            size_t real_a = sorted_indices_[a];
            size_t real_b = sorted_indices_[b];
            size_t real_c = sorted_indices_[c];

            flipBit(hard, real_a, syndrome, unsatisfied);
            flipBit(hard, real_b, syndrome, unsatisfied);
            flipBit(hard, real_c, syndrome, unsatisfied);
            ++queries;

            if (unsatisfied == 0) return true;

            // Unflip (reverse order)
            flipBit(hard, real_c, syndrome, unsatisfied);
            flipBit(hard, real_b, syndrome, unsatisfied);
            flipBit(hard, real_a, syndrome, unsatisfied);
        }
    }

    return false;
}

bool ORBGRANDDecoder::tryWeight4(std::vector<uint8_t>& hard,
                                  std::vector<uint8_t>& syndrome,
                                  size_t& unsatisfied,
                                  size_t lw, size_t& queries) {
    // Weight 4: (a, b, c, d) where a < b < c < d, sum = lw
    // Minimum lw for weight 4: 0 + 1 + 2 + 3 = 6
    const size_t n = H_->n;
    if (lw < 6) return false;

    for (size_t a = 0; a * 4 + 6 <= lw; ++a) {
        size_t rem3 = lw - a;

        for (size_t b = a + 1; b * 3 + 3 <= rem3; ++b) {
            size_t rem2 = rem3 - b;
            if (b + 1 > rem2 / 2) break;

            for (size_t c = b + 1; c * 2 < rem2; ++c) {
                if (queries >= cfg_.max_queries) return false;

                size_t d = rem2 - c;
                if (d <= c) break;
                if (d >= n) continue;

                size_t real_a = sorted_indices_[a];
                size_t real_b = sorted_indices_[b];
                size_t real_c = sorted_indices_[c];
                size_t real_d = sorted_indices_[d];

                flipBit(hard, real_a, syndrome, unsatisfied);
                flipBit(hard, real_b, syndrome, unsatisfied);
                flipBit(hard, real_c, syndrome, unsatisfied);
                flipBit(hard, real_d, syndrome, unsatisfied);
                ++queries;

                if (unsatisfied == 0) return true;

                flipBit(hard, real_d, syndrome, unsatisfied);
                flipBit(hard, real_c, syndrome, unsatisfied);
                flipBit(hard, real_b, syndrome, unsatisfied);
                flipBit(hard, real_a, syndrome, unsatisfied);
            }
        }
    }

    return false;
}

// =========================================================================
// Main pattern enumeration (logistic weight ordering)
// =========================================================================

bool ORBGRANDDecoder::enumeratePatterns(std::vector<uint8_t>& hard,
                                         std::vector<uint8_t>& syndrome,
                                         size_t& unsatisfied,
                                         LDPCDecodeResult& result) {
    size_t queries = 0;

    for (size_t lw = 0; lw <= cfg_.max_lw; ++lw) {
        if (queries >= cfg_.max_queries) break;

        // Weight 1 (if lw < n)
        if (cfg_.max_weight >= 1 &&
            tryWeight1(hard, syndrome, unsatisfied, lw, queries)) {
            result.converged = true;
            result.iterations = queries;
            return true;
        }

        // Weight 2 (if lw >= 1)
        if (cfg_.max_weight >= 2 &&
            tryWeight2(hard, syndrome, unsatisfied, lw, queries)) {
            result.converged = true;
            result.iterations = queries;
            return true;
        }

        // Weight 3 (if lw >= 3)
        if (cfg_.max_weight >= 3 &&
            tryWeight3(hard, syndrome, unsatisfied, lw, queries)) {
            result.converged = true;
            result.iterations = queries;
            return true;
        }

        // Weight 4 (if lw >= 6)
        if (cfg_.max_weight >= 4 &&
            tryWeight4(hard, syndrome, unsatisfied, lw, queries)) {
            result.converged = true;
            result.iterations = queries;
            return true;
        }
    }

    result.iterations = queries;
    return false;
}

// =========================================================================
// Public decode interfaces
// =========================================================================

LDPCDecodeResult ORBGRANDDecoder::decodeFull(const float* llr_in,
                                               uint8_t* hard_out) {
    LDPCDecodeResult result;
    const size_t n = H_->n;

    // 1. Hard decision from channel LLRs
    size_t n_bytes = (n + 7) / 8;
    std::fill(hard_bits_.begin(), hard_bits_.end(), 0);
    for (size_t i = 0; i < n; ++i) {
        if (llr_in[i] < 0.f) {
            setBit(hard_bits_.data(), i, true);
        }
    }

    // 2. Sort bit positions by |LLR| ascending (least reliable first)
    std::iota(sorted_indices_.begin(), sorted_indices_.end(), 0);
    for (size_t i = 0; i < n; ++i) {
        sorted_absllr_[i] = std::abs(llr_in[i]);
    }

    std::sort(sorted_indices_.begin(), sorted_indices_.end(),
              [this](size_t a, size_t b) {
                  return sorted_absllr_[a] < sorted_absllr_[b];
              });

    // 3. Compute syndrome of hard decision
    size_t unsatisfied = 0;
    computeSyndrome(hard_bits_, syndrome_, unsatisfied);

    // 4. Check if hard decision already valid (common at high SNR)
    if (unsatisfied == 0) {
        result.converged = true;
        result.iterations = 0;
        std::memcpy(hard_out, hard_bits_.data(), n_bytes);

        // Average confidence
        float sum_mag = 0.f;
        for (size_t i = 0; i < n; ++i) sum_mag += std::abs(llr_in[i]);
        result.avg_magnitude = sum_mag / static_cast<float>(n);
        return result;
    }

    // 5. Enumerate candidate error patterns (ORBGRAND ordering)
    bool found = enumeratePatterns(hard_bits_, syndrome_, unsatisfied, result);

    if (!found) {
        // Fell through without finding a codeword — return hard decision
        // (best effort, but not a valid codeword)
        result.converged = false;
    }

    // Copy result
    std::memcpy(hard_out, hard_bits_.data(), n_bytes);

    // Average confidence
    float sum_mag = 0.f;
    for (size_t i = 0; i < n; ++i) sum_mag += std::abs(llr_in[i]);
    result.avg_magnitude = sum_mag / static_cast<float>(n);

    return result;
}

LDPCDecodeResult ORBGRANDDecoder::decode(const float* llr_in,
                                           uint8_t* info_out) {
    size_t n_bytes = (H_->n + 7) / 8;
    std::vector<uint8_t> full_cw(n_bytes, 0);

    auto result = decodeFull(llr_in, full_cw.data());

    // Extract systematic info bits (first k bits)
    size_t k_bytes = (H_->k + 7) / 8;
    std::memcpy(info_out, full_cw.data(), k_bytes);

    return result;
}

// =========================================================================
// List-GRAND: cost-ranked candidate collection
//
// Mirror of the hard-output tryWeightX, but on `unsatisfied == 0` we RECORD
// a snapshot of `hard` plus the pattern cost (= sum of sorted_absllr_ over
// the flipped sorted positions, which are exactly the loop variables) and
// then UNFLIP and KEEP SEARCHING. Each collectWeightX returns true to signal
// the caller to STOP (list full or query budget exhausted).
//
// INVARIANT: after returning, every flip applied inside has been undone, so
// `hard` / `syndrome` / `unsatisfied` are exactly restored — the outer
// enumeration over logistic weight can continue correctly.
// =========================================================================

void ORBGRANDDecoder::ListCollector::offer(const std::vector<uint8_t>& hard,
                                            float cost) {
    if (cap == 0) return;
    // Reject if list is full and this cost is no better than the worst kept.
    if (items.size() >= cap && cost >= items.back().cost) return;

    GRANDCandidate cand;
    cand.bits.assign(hard.begin(), hard.begin() +
                     static_cast<std::ptrdiff_t>(n_bytes));
    cand.cost = cost;

    // Insert keeping ascending-cost order.
    auto pos = std::upper_bound(
        items.begin(), items.end(), cost,
        [](float c, const GRANDCandidate& g) { return c < g.cost; });
    items.insert(pos, std::move(cand));

    if (items.size() > cap) items.pop_back();
}

bool ORBGRANDDecoder::collectWeight1(std::vector<uint8_t>& hard,
                                      std::vector<uint8_t>& syndrome,
                                      size_t& unsatisfied,
                                      size_t lw, size_t& queries,
                                      ListCollector& col) {
    const size_t n = H_->n;
    if (lw >= n) return false;
    if (queries >= cfg_.max_queries) return true;

    size_t real_idx = sorted_indices_[lw];
    flipBit(hard, real_idx, syndrome, unsatisfied);
    ++queries;

    if (unsatisfied == 0) {
        float cost = sorted_absllr_[real_idx];
        col.offer(hard, cost);
    }

    // Always unflip and keep searching.
    flipBit(hard, real_idx, syndrome, unsatisfied);
    return col.full();
}

bool ORBGRANDDecoder::collectWeight2(std::vector<uint8_t>& hard,
                                      std::vector<uint8_t>& syndrome,
                                      size_t& unsatisfied,
                                      size_t lw, size_t& queries,
                                      ListCollector& col) {
    const size_t n = H_->n;
    if (lw < 1) return false;

    size_t a_max = (lw - 1) / 2;
    for (size_t a = 0; a <= a_max; ++a) {
        if (queries >= cfg_.max_queries) return true;

        size_t b = lw - a;
        if (b >= n) continue;

        size_t real_a = sorted_indices_[a];
        size_t real_b = sorted_indices_[b];

        flipBit(hard, real_a, syndrome, unsatisfied);
        flipBit(hard, real_b, syndrome, unsatisfied);
        ++queries;

        if (unsatisfied == 0) {
            float cost = sorted_absllr_[real_a] + sorted_absllr_[real_b];
            col.offer(hard, cost);
        }

        flipBit(hard, real_b, syndrome, unsatisfied);
        flipBit(hard, real_a, syndrome, unsatisfied);

        if (col.full()) return true;
    }
    return false;
}

bool ORBGRANDDecoder::collectWeight3(std::vector<uint8_t>& hard,
                                      std::vector<uint8_t>& syndrome,
                                      size_t& unsatisfied,
                                      size_t lw, size_t& queries,
                                      ListCollector& col) {
    const size_t n = H_->n;
    if (lw < 3) return false;

    for (size_t a = 0; a * 3 + 3 <= lw; ++a) {
        size_t rem = lw - a;
        if (a + 1 > rem / 2) break;

        for (size_t b = a + 1; b * 2 < rem; ++b) {
            if (queries >= cfg_.max_queries) return true;

            size_t c = rem - b;
            if (c <= b) break;
            if (c >= n) continue;

            size_t real_a = sorted_indices_[a];
            size_t real_b = sorted_indices_[b];
            size_t real_c = sorted_indices_[c];

            flipBit(hard, real_a, syndrome, unsatisfied);
            flipBit(hard, real_b, syndrome, unsatisfied);
            flipBit(hard, real_c, syndrome, unsatisfied);
            ++queries;

            if (unsatisfied == 0) {
                float cost = sorted_absllr_[real_a] + sorted_absllr_[real_b]
                           + sorted_absllr_[real_c];
                col.offer(hard, cost);
            }

            flipBit(hard, real_c, syndrome, unsatisfied);
            flipBit(hard, real_b, syndrome, unsatisfied);
            flipBit(hard, real_a, syndrome, unsatisfied);

            if (col.full()) return true;
        }
    }
    return false;
}

bool ORBGRANDDecoder::collectWeight4(std::vector<uint8_t>& hard,
                                      std::vector<uint8_t>& syndrome,
                                      size_t& unsatisfied,
                                      size_t lw, size_t& queries,
                                      ListCollector& col) {
    const size_t n = H_->n;
    if (lw < 6) return false;

    for (size_t a = 0; a * 4 + 6 <= lw; ++a) {
        size_t rem3 = lw - a;

        for (size_t b = a + 1; b * 3 + 3 <= rem3; ++b) {
            size_t rem2 = rem3 - b;
            if (b + 1 > rem2 / 2) break;

            for (size_t c = b + 1; c * 2 < rem2; ++c) {
                if (queries >= cfg_.max_queries) return true;

                size_t d = rem2 - c;
                if (d <= c) break;
                if (d >= n) continue;

                size_t real_a = sorted_indices_[a];
                size_t real_b = sorted_indices_[b];
                size_t real_c = sorted_indices_[c];
                size_t real_d = sorted_indices_[d];

                flipBit(hard, real_a, syndrome, unsatisfied);
                flipBit(hard, real_b, syndrome, unsatisfied);
                flipBit(hard, real_c, syndrome, unsatisfied);
                flipBit(hard, real_d, syndrome, unsatisfied);
                ++queries;

                if (unsatisfied == 0) {
                    float cost = sorted_absllr_[real_a] + sorted_absllr_[real_b]
                               + sorted_absllr_[real_c] + sorted_absllr_[real_d];
                    col.offer(hard, cost);
                }

                flipBit(hard, real_d, syndrome, unsatisfied);
                flipBit(hard, real_c, syndrome, unsatisfied);
                flipBit(hard, real_b, syndrome, unsatisfied);
                flipBit(hard, real_a, syndrome, unsatisfied);

                if (col.full()) return true;
            }
        }
    }
    return false;
}

GRANDListResult ORBGRANDDecoder::decodeList(const float* llr_in, size_t L) {
    GRANDListResult out;
    const size_t n = H_->n;
    const size_t n_bytes = (n + 7) / 8;

    if (L == 0) {
        out.base.converged = false;
        return out;
    }

    // 1. Hard decision from channel LLRs
    std::fill(hard_bits_.begin(), hard_bits_.end(), 0);
    for (size_t i = 0; i < n; ++i) {
        if (llr_in[i] < 0.f) setBit(hard_bits_.data(), i, true);
    }

    // 2. Reliability sort (least reliable first)
    std::iota(sorted_indices_.begin(), sorted_indices_.end(), 0);
    for (size_t i = 0; i < n; ++i) sorted_absllr_[i] = std::abs(llr_in[i]);
    std::sort(sorted_indices_.begin(), sorted_indices_.end(),
              [this](size_t a, size_t b) {
                  return sorted_absllr_[a] < sorted_absllr_[b];
              });

    // 3. Syndrome of the hard decision
    size_t unsatisfied = 0;
    computeSyndrome(hard_bits_, syndrome_, unsatisfied);

    ListCollector col;
    col.cap     = L;
    col.n_bytes = n_bytes;

    // 4. Zero-error candidate (hard decision already a codeword): cost 0.
    if (unsatisfied == 0) {
        col.offer(hard_bits_, 0.f);
    }

    // 5. Enumerate error patterns in logistic-weight order, collecting up
    //    to L valid codewords ranked by ascending cost.
    size_t queries = 0;
    bool stop = col.full();
    for (size_t lw = 0; !stop && lw <= cfg_.max_lw; ++lw) {
        if (queries >= cfg_.max_queries) break;

        if (cfg_.max_weight >= 1 &&
            collectWeight1(hard_bits_, syndrome_, unsatisfied, lw, queries, col))
            { stop = col.full() || queries >= cfg_.max_queries; }
        if (!stop && cfg_.max_weight >= 2 &&
            collectWeight2(hard_bits_, syndrome_, unsatisfied, lw, queries, col))
            { stop = col.full() || queries >= cfg_.max_queries; }
        if (!stop && cfg_.max_weight >= 3 &&
            collectWeight3(hard_bits_, syndrome_, unsatisfied, lw, queries, col))
            { stop = col.full() || queries >= cfg_.max_queries; }
        if (!stop && cfg_.max_weight >= 4 &&
            collectWeight4(hard_bits_, syndrome_, unsatisfied, lw, queries, col))
            { stop = col.full() || queries >= cfg_.max_queries; }

        if (col.full() || queries >= cfg_.max_queries) stop = true;
    }

    out.list = std::move(col.items);
    out.base.converged   = !out.list.empty();
    out.base.iterations  = queries;

    float sum_mag = 0.f;
    for (size_t i = 0; i < n; ++i) sum_mag += std::abs(llr_in[i]);
    out.base.avg_magnitude = sum_mag / static_cast<float>(n);

    return out;
}

// =========================================================================
// SOGRAND: per-bit max-log a-posteriori LLRs from a List-GRAND search
// =========================================================================

LDPCDecodeResult ORBGRANDDecoder::decodePosterior(const float* llr_in,
                                                   std::vector<float>& post_out) {
    const size_t n = H_->n;
    constexpr float kClamp = 20.f;  // matches the demapper LLR clamp

    // Run a List-GRAND search (modest list depth is plenty for SO).
    GRANDListResult lr = decodeList(llr_in, 8);

    post_out.assign(n, 0.f);

    if (lr.list.empty()) {
        // No codeword found — fall back to the (clamped) channel LLR.
        for (size_t i = 0; i < n; ++i) {
            post_out[i] = std::max(-kClamp, std::min(kClamp, llr_in[i]));
        }
        LDPCDecodeResult r = lr.base;
        r.converged = false;
        return r;
    }

    // For each bit i: max-log APP LLR = (min cost over members with bit i=0)
    //                                 - (min cost over members with bit i=1).
    // The list is ascending in cost, so the FIRST member with a given bit
    // value is its minimum-cost representative — track first-seen per side.
    constexpr float kInf = 1e30f;
    for (size_t i = 0; i < n; ++i) {
        float c0 = kInf, c1 = kInf;
        for (const auto& cand : lr.list) {
            bool bit = getBit(cand.bits.data(), i);
            if (bit) { if (cand.cost < c1) c1 = cand.cost; }
            else     { if (cand.cost < c0) c0 = cand.cost; }
        }

        if (c0 < kInf && c1 < kInf) {
            // Both hypotheses represented in the list.
            float l = c1 - c0;  // post = (cost|bit1) - (cost|bit0); >0 => bit0
            post_out[i] = std::max(-kClamp, std::min(kClamp, l));
        } else {
            // Whole list agrees on this bit (the common case for strong
            // bits). Fall back to the channel LLR, clamped, but keep the
            // sign consistent with the agreed value.
            float ch = std::max(-kClamp, std::min(kClamp, llr_in[i]));
            bool agreed = (c1 < kInf);  // true => list says bit 1
            float mag = std::abs(ch);
            if (mag < 1e-3f) mag = kClamp;  // unreliable channel bit, list decided
            post_out[i] = agreed ? -mag : mag;  // bit1 => negative LLR
        }
    }

    LDPCDecodeResult r = lr.base;
    r.converged = !lr.list.empty();
    return r;
}

} // namespace gw
