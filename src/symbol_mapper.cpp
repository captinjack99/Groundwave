/**
 * @file symbol_mapper.cpp
 * @brief QAM symbol mapping / demapping
 *
 * All constellations normalized to unit average power.
 * Gray coding applied to I/Q dimensions independently.
 * Soft demapper uses piecewise-linear approximation for M>=16.
 */

#include "symbol_mapper.hpp"
#include "simd_helpers.hpp"
#include <cmath>
#include <algorithm>
#include <functional>
#include <limits>
#include <stdexcept>
#include <vector>

namespace dsca {

// =========================================================================
// Construction
// =========================================================================

namespace {

// Population count (Hamming weight) of a small integer.
inline int popcount_u(size_t v) {
    int c = 0;
    while (v) { c += static_cast<int>(v & 1u); v >>= 1; }
    return c;
}

// Build a per-dimension ANTI-GRAY label permutation: label2level[g] = level.
//
// Goal: assign one bits_dim-bit label to each of the G = 2^bits_dim equispaced
// levels so that Euclidean-adjacent levels (k, k+1) carry labels of LARGE
// Hamming distance. This is the per-dimension half of an anti-Gray square-QAM
// labeling; because square QAM is separable, doing it independently on I and Q
// yields a 2D map whose nearest neighbors differ in many bits.
//
// Construction (deterministic, optimal for the small G used here): exact
// branch-and-bound that maximizes the total adjacent Hamming distance
// Sum_{k} popcount(label[k] XOR label[k+1]). For G <= 16 (square QAM up to
// 256) this is tiny. label[0] is pinned to 0 to break the global-symmetry
// degeneracy and keep the result reproducible. The optimum places nearest
// neighbors at Hamming distance ~bits_dim, the hallmark of anti-Gray.
std::vector<size_t> buildAntiGrayLevels(size_t bits_dim) {
    const size_t G = static_cast<size_t>(1) << bits_dim;

    // Precompute pairwise Hamming distances between candidate labels.
    std::vector<std::vector<int>> hd(G, std::vector<int>(G, 0));
    for (size_t a = 0; a < G; ++a)
        for (size_t b = 0; b < G; ++b)
            hd[a][b] = popcount_u(a ^ b);

    std::vector<size_t> label(G);        // label[level]
    std::vector<char>   used(G, 0);
    std::vector<size_t> best(G);
    int best_cost = -1;

    // Upper bound on the extra cost achievable from the remaining levels: each
    // future adjacency contributes at most bits_dim. Used to prune.
    std::function<void(size_t, int)> dfs = [&](size_t pos, int cost) {
        if (pos == G) {
            if (cost > best_cost) { best_cost = cost; best = label; }
            return;
        }
        int remaining_edges = static_cast<int>(G - pos);   // edges still to add
        if (cost + remaining_edges * static_cast<int>(bits_dim) <= best_cost)
            return;                                          // cannot beat best
        for (size_t g = 0; g < G; ++g) {
            if (used[g]) continue;
            int add = (pos == 0) ? 0 : hd[label[pos - 1]][g];
            used[g] = 1;
            label[pos] = g;
            dfs(pos + 1, cost + add);
            used[g] = 0;
        }
    };
    // Pin label[0] = 0 to remove the trivial relabeling symmetry.
    used[0] = 1; label[0] = 0;
    dfs(1, 0);

    return best;   // best[level] = label
}

} // anonymous namespace

void SymbolMapper::buildDimLabeling(size_t bits_dim) {
    const size_t G = static_cast<size_t>(1) << bits_dim;
    label2level_.assign(G, 0);
    level2label_.assign(G, 0);

    if (labeling_ == Labeling::Gray) {
        // Standard binary-reflected Gray: level k carries label gray(k).
        auto natToGray = [](size_t n) -> size_t { return n ^ (n >> 1); };
        for (size_t k = 0; k < G; ++k) {
            size_t g = natToGray(k);
            level2label_[k] = g;
            label2level_[g] = k;
        }
    } else {
        // AntiGray: level k carries label best[k] from the optimizer above.
        std::vector<size_t> lvl2lab = buildAntiGrayLevels(bits_dim);
        for (size_t k = 0; k < G; ++k) {
            size_t g = lvl2lab[k];
            level2label_[k] = g;
            label2level_[g] = k;
        }
    }
}

SymbolMapper::SymbolMapper(Modulation mod, Labeling labeling)
    : mod_(mod), labeling_(labeling) {
    switch (mod) {
        case Modulation::BPSK:    initBPSK(); break;
        case Modulation::QPSK:    initQPSK(); break;
        case Modulation::QAM16:   initSquareQAM(16);   break;
        case Modulation::QAM64:   initSquareQAM(64);   break;
        case Modulation::QAM256:  initSquareQAM(256);  break;
        case Modulation::QAM1024: initSquareQAM(1024); break;
        case Modulation::QAM4096: initSquareQAM(4096); break;
        default: throw std::invalid_argument("Unsupported modulation");
    }
    bps_ = dsca::bitsPerSymbol(mod);

    // Precompute bit patterns
    size_t M = const_.size();
    bit_patterns_.resize(M);
    for (size_t i = 0; i < M; ++i) {
        bit_patterns_[i].resize(bps_);
        for (size_t b = 0; b < bps_; ++b) {
            bit_patterns_[i][b] = static_cast<uint8_t>(
                (i >> (bps_ - 1 - b)) & 1);
        }
    }

    // Build piecewise-linear LLR tables for square QAM
    initPWLTables();
}

// =========================================================================
// Constellation Init
// =========================================================================

void SymbolMapper::initBPSK() {
    const_.resize(2);
    const_[0] = ComplexSample( 1.0f, 0.0f); // bit 0
    const_[1] = ComplexSample(-1.0f, 0.0f); // bit 1
    norm_ = 1.0f;
}

void SymbolMapper::initQPSK() {
    const_.resize(4);
    float s = 1.0f / std::sqrt(2.0f);
    // Bit 0 → I dimension: 0=+s, 1=-s
    // Bit 1 → Q dimension: 0=+s, 1=-s
    // Matches soft demapper: LLR[0] = sc*Re, LLR[1] = sc*Im
    const_[0] = ComplexSample( s,  s); // 00
    const_[1] = ComplexSample( s, -s); // 01
    const_[2] = ComplexSample(-s,  s); // 10
    const_[3] = ComplexSample(-s, -s); // 11
    norm_ = s;
}

void SymbolMapper::initSquareQAM(size_t order) {
    size_t grid = static_cast<size_t>(std::round(std::sqrt(static_cast<double>(order))));
    if (grid * grid != order) {
        throw std::invalid_argument("QAM order must be perfect square");
    }

    size_t bits_dim = 0;
    { size_t g = grid; while (g > 1) { g >>= 1; bits_dim++; } }

    // Normalization: E[|s|^2] = 1
    // For MxM QAM with levels -(M-1), -(M-3), ..., (M-3), (M-1):
    //   E[|s|^2] = (2/3)(M^2 - 1)
    float m = static_cast<float>(grid);
    float avg_power = (2.0f / 3.0f) * (m * m - 1.0f);
    norm_ = 1.0f / std::sqrt(avg_power);

    // Build the per-dimension label<->level permutation (Gray or AntiGray).
    // label2level_[g] = level carried by the bits_dim-bit label g.
    buildDimLabeling(bits_dim);

    // Build the constellation.
    // Symbol index bits: [I-bits(high) | Q-bits(low)]
    // For each dimension: the label's level index gives the coordinate.
    // Level 0 = most positive coordinate, matching soft demapper convention
    // where (Gray) MSB=0 → positive coordinate → positive LLR.
    //
    // Coordinate = ((grid-1) - 2*level) * norm_
    //   level 0 → +(grid-1), level 1 → +(grid-3), ..., level grid-1 → -(grid-1)
    const_.resize(order);
    for (size_t idx = 0; idx < order; ++idx) {
        size_t i_lab = idx >> bits_dim;          // high bits → I dimension label
        size_t q_lab = idx & (grid - 1);         // low bits  → Q dimension label

        size_t i_level = label2level_[i_lab];
        size_t q_level = label2level_[q_lab];

        float i_coord = (static_cast<float>(grid - 1) - 2.0f * static_cast<float>(i_level));
        float q_coord = (static_cast<float>(grid - 1) - 2.0f * static_cast<float>(q_level));

        const_[idx] = ComplexSample(i_coord * norm_, q_coord * norm_);
    }
}

// =========================================================================
// Mapping
// =========================================================================

void SymbolMapper::mapBytes(const uint8_t* bytes, size_t num_bits,
                            ComplexBuf& symbols) const {
    size_t num_sym = num_bits / bps_;
    symbols.resize(num_sym);

    size_t bit_idx = 0;
    for (size_t s = 0; s < num_sym; ++s) {
        uint16_t val = 0;
        for (size_t b = 0; b < bps_; ++b) {
            size_t byte_i = bit_idx / 8;
            size_t bit_pos = 7 - (bit_idx % 8); // MSB first
            uint8_t bit = (bytes[byte_i] >> bit_pos) & 1;
            val = static_cast<uint16_t>((val << 1) | bit);
            ++bit_idx;
        }
        symbols[s] = const_[val & (static_cast<uint16_t>(const_.size()) - 1)];
    }
}

// =========================================================================
// Hard Demapping
// =========================================================================

uint16_t SymbolMapper::demapHard(ComplexSample sym) const {
    size_t best = 0;
    float best_d = std::numeric_limits<float>::max();

    // Zone-based for high-order QAM (M >= 256)
    if (bps_ >= 8) {
        size_t grid = static_cast<size_t>(1) << (bps_ / 2);
        size_t bits_dim = bps_ / 2;
        float spacing = norm_ * 2.0f;

        // New constellation: coord = (grid-1 - 2*level) * norm_
        // So level = (grid-1)/2 - coord/(2*norm_) = (grid-1)/2 - coord/spacing
        int i_level = static_cast<int>(std::round(
            static_cast<float>(grid - 1) / 2.0f - sym.real() / spacing));
        int q_level = static_cast<int>(std::round(
            static_cast<float>(grid - 1) / 2.0f - sym.imag() / spacing));

        i_level = std::clamp(i_level, 0, static_cast<int>(grid - 1));
        q_level = std::clamp(q_level, 0, static_cast<int>(grid - 1));

        // Map level -> label via the active labeling permutation.
        for (int di = -2; di <= 2; ++di) {
            for (int dq = -2; dq <= 2; ++dq) {
                int il = i_level + di;
                int ql = q_level + dq;
                if (il < 0 || il >= static_cast<int>(grid) ||
                    ql < 0 || ql >= static_cast<int>(grid)) continue;
                size_t lab_i = level2label_[static_cast<size_t>(il)];
                size_t lab_q = level2label_[static_cast<size_t>(ql)];
                size_t idx = (lab_i << bits_dim) | lab_q;
                float dr = sym.real() - const_[idx].real();
                float di2 = sym.imag() - const_[idx].imag();
                float d = dr*dr + di2*di2;
                if (d < best_d) { best_d = d; best = idx; }
            }
        }
    } else {
        for (size_t i = 0; i < const_.size(); ++i) {
            float dr = sym.real() - const_[i].real();
            float di2 = sym.imag() - const_[i].imag();
            float d = dr*dr + di2*di2;
            if (d < best_d) { best_d = d; best = i; }
        }
    }
    return static_cast<uint16_t>(best);
}

// =========================================================================
// Soft Demapping (LLR)
// =========================================================================

void SymbolMapper::demapSoft(ComplexSample sym, float noise_var,
                             std::vector<float>& llrs) const {
    llrs.resize(bps_);
    float nv = std::max(noise_var, 1e-10f);

    if (bps_ == 1) {
        // BPSK: LLR = 4*Re(r)/σ²
        llrs[0] = std::clamp(4.0f * sym.real() / nv, -20.0f, 20.0f);
        return;
    }
    if (bps_ == 2) {
        // QPSK: independent I/Q
        float sc = 4.0f * norm_ / nv;
        llrs[0] = std::clamp(sc * sym.real(), -20.0f, 20.0f);
        llrs[1] = std::clamp(sc * sym.imag(), -20.0f, 20.0f);
        return;
    }

    // For M >= 4: exact max-log-MAP, AVX2-accelerated when available.
    // LLR(b) = (min |r-s|² over s with bit_b=1) − (min |r-s|² over s with bit_b=0)
    // divided by σ². Positive LLR → bit=0 more likely (LDPC convention).
    const size_t M = const_.size();

    // Pack constellation into separate I/Q arrays for SIMD-friendly access.
    static thread_local std::vector<float> s_i_cache;
    static thread_local std::vector<float> s_q_cache;
    static thread_local size_t cached_M = 0;
    static thread_local const SymbolMapper* cache_owner = nullptr;
    if (cache_owner != this || cached_M != M) {
        s_i_cache.resize(M + 8);
        s_q_cache.resize(M + 8);
        for (size_t i = 0; i < M; ++i) {
            s_i_cache[i] = const_[i].real();
            s_q_cache[i] = const_[i].imag();
        }
        // Pad with safe values so the SIMD load past M is harmless
        for (size_t i = M; i < M + 8; ++i) {
            s_i_cache[i] = 1e9f;
            s_q_cache[i] = 1e9f;
        }
        cached_M = M;
        cache_owner = this;
    }

    // First pass: compute all M squared distances using SIMD chunks of 8.
    static thread_local std::vector<float> d_buf;
    d_buf.resize(M);
    float dout[8];
    for (size_t i = 0; i < M; i += 8) {
        simd::distSquared8(sym.real(), sym.imag(),
                            &s_i_cache[i], &s_q_cache[i], dout);
        size_t lim = std::min<size_t>(8, M - i);
        for (size_t k = 0; k < lim; ++k) d_buf[i + k] = dout[k];
    }

    // Second pass: per-bit min over indices that have that bit value.
    for (size_t b = 0; b < bps_; ++b) {
        float min_d0 = std::numeric_limits<float>::max();
        float min_d1 = std::numeric_limits<float>::max();
        for (size_t i = 0; i < M; ++i) {
            float d = d_buf[i];
            if (bit_patterns_[i][b]) { if (d < min_d1) min_d1 = d; }
            else                      { if (d < min_d0) min_d0 = d; }
        }
        llrs[b] = std::clamp((min_d1 - min_d0) / nv, -20.0f, 20.0f);
    }
}

void SymbolMapper::demapSoft(const ComplexBuf& symbols, float noise_var,
                             std::vector<float>& llrs) const {
    llrs.resize(symbols.size() * bps_);
    std::vector<float> tmp;
    for (size_t s = 0; s < symbols.size(); ++s) {
        demapSoft(symbols[s], noise_var, tmp);
        for (size_t b = 0; b < bps_; ++b) {
            llrs[s * bps_ + b] = tmp[b];
        }
    }
}

void SymbolMapper::demapSoft(const ComplexBuf& symbols,
                             const std::vector<float>& noise_var,
                             std::vector<float>& llrs) const {
    llrs.resize(symbols.size() * bps_);
    std::vector<float> tmp;
    for (size_t s = 0; s < symbols.size(); ++s) {
        float nv = (s < noise_var.size()) ? noise_var[s] : 1e-3f;
        demapSoft(symbols[s], nv, tmp);
        for (size_t b = 0; b < bps_; ++b) {
            llrs[s * bps_ + b] = tmp[b];
        }
    }
}

// =========================================================================
// Piecewise-Linear LLR Tables
// =========================================================================
//
// For Gray-coded square QAM, each bit's max-log LLR depends on only one
// dimension (I or Q). Between adjacent decision boundaries the LLR is
// exactly linear in the received coordinate:
//
//   LLR(b|r) = [d²(r, nearest bit=1) - d²(r, nearest bit=0)] / σ²
//
// Since d²(r,s) = (r-s)², the difference (r-s₁)²-(r-s₀)² = 2r(s₀-s₁)+(s₁²-s₀²)
// is linear in r. Breakpoints occur at midpoints between constellation levels
// (where nearest-neighbor assignments change).
//
// This gives O(bps) per symbol instead of O(M·bps) for the exact method.
// =========================================================================

void SymbolMapper::initPWLTables() {
    // Only for square QAM (bps >= 4, even)
    if (bps_ < 4 || (bps_ & 1) != 0) {
        grid_size_ = 0;
        return;
    }

    size_t G = static_cast<size_t>(1) << (bps_ / 2); // grid dimension
    grid_size_ = G;
    size_t bits_dim = bps_ / 2;

    // Constellation levels in one dimension (descending: most positive first)
    // level[k] = (G-1-2k) * norm_  for k = 0..G-1
    std::vector<float> levels(G);
    for (size_t k = 0; k < G; ++k) {
        levels[k] = (static_cast<float>(G - 1) - 2.0f * static_cast<float>(k)) * norm_;
    }

    // Decision boundaries = midpoints between adjacent levels
    // boundary[k] = (levels[k] + levels[k+1]) / 2 for k = 0..G-2
    std::vector<float> boundaries(G - 1);
    for (size_t k = 0; k + 1 < G; ++k) {
        boundaries[k] = (levels[k] + levels[k + 1]) * 0.5f;
    }

    // For each bit in this dimension, determine which levels have bit=0 vs bit=1
    // Bit patterns: for dimension's bits (bits_dim bits), level k carries label
    // level2label_[k]; bit b of that label determines if level has bit b = 0/1.
    // This makes the PWL demapper exact for ANY labeling (Gray or AntiGray):
    // it still derives each bit's nearest-0/nearest-1 levels from the true
    // label assignment, just over a possibly non-monotone level->label map.

    // Build tables for 2 dimensions (I and Q are structurally identical)
    // pwl_tables_[dim][bit_within_dim] = segments
    pwl_tables_.resize(2);
    for (size_t dim = 0; dim < 2; ++dim) {
        pwl_tables_[dim].resize(bits_dim);

        for (size_t b = 0; b < bits_dim; ++b) {
            auto& segs = pwl_tables_[dim][b];
            segs.clear();

            // For each level, determine bit value
            std::vector<uint8_t> level_bit(G);
            for (size_t k = 0; k < G; ++k) {
                size_t label = level2label_[k];
                level_bit[k] = static_cast<uint8_t>(
                    (label >> (bits_dim - 1 - b)) & 1);
            }

            // Build segments. Regions are:
            //   (-inf, boundary[0]], (boundary[0], boundary[1]], ..., (boundary[G-2], +inf)
            // In each region, find nearest level with bit=0 and bit=1
            // Region i corresponds to "nearest level is level[i]" but we need
            // nearest-with-specific-bit, which may be a neighboring level.

            // We'll use the analytical approach: evaluate segment endpoints
            // Total G+1 regions bounded by -inf, boundary[0..G-2], +inf
            // But we'll clip to a reasonable range: -(G+1)*norm_ to +(G+1)*norm_
            float lo = -(static_cast<float>(G) + 1.0f) * norm_;
            float hi =  (static_cast<float>(G) + 1.0f) * norm_;

            // Collect all breakpoints, sorted ascending.
            //
            // For GRAY the level<->label map is monotone, so each bit's
            // nearest-bit-0 and nearest-bit-1 levels change only at adjacent-
            // level midpoints: the original boundary set is sufficient AND we
            // keep it EXACTLY as before so the Gray PWL output is byte-for-byte
            // identical to the legacy code (no roundoff perturbation of any
            // existing test).
            //
            // For ANTIGRAY the map is non-monotone, so the nearest same-bit
            // level can switch at midpoints between NON-adjacent levels too.
            // We therefore add the midpoint of every pair of levels; the LLR is
            // then exactly linear on each resulting interval. (These extra knots
            // are only added for anti-Gray, so Gray is untouched.)
            std::vector<float> bkpts;
            bkpts.push_back(lo);
            bkpts.push_back(hi);
            for (auto bp : boundaries) bkpts.push_back(bp);
            if (labeling_ != Labeling::Gray) {
                for (size_t a = 0; a < G; ++a)
                    for (size_t c = a + 1; c < G; ++c)
                        bkpts.push_back((levels[a] + levels[c]) * 0.5f);
            }
            std::sort(bkpts.begin(), bkpts.end());
            bkpts.erase(std::unique(bkpts.begin(), bkpts.end(),
                        [](float x, float y){ return std::abs(x - y) < 1e-6f; }),
                        bkpts.end());

            for (size_t r = 0; r + 1 < bkpts.size(); ++r) {
                float seg_start = bkpts[r];
                float seg_mid = (bkpts[r] + bkpts[r + 1]) * 0.5f;

                // Find nearest level with bit=0 and bit=1 at seg_mid
                float min_d0 = std::numeric_limits<float>::max();
                float min_d1 = std::numeric_limits<float>::max();
                float best_s0 = 0.f, best_s1 = 0.f;

                for (size_t k = 0; k < G; ++k) {
                    float dist = std::abs(seg_mid - levels[k]);
                    if (level_bit[k] == 0) {
                        if (dist < min_d0) { min_d0 = dist; best_s0 = levels[k]; }
                    } else {
                        if (dist < min_d1) { min_d1 = dist; best_s1 = levels[k]; }
                    }
                }

                // LLR(r) = [(r-best_s1)² - (r-best_s0)²] / σ²
                //         = [2r(best_s0 - best_s1) + (best_s1² - best_s0²)] / σ²
                // slope     = 2(best_s0 - best_s1) / σ²
                // intercept at seg_start:
                //   LLR(seg_start) = [2*seg_start*(best_s0-best_s1) + best_s1²-best_s0²] / σ²
                //
                // We store unnormalized (multiply by 1/σ² at eval time):
                float slope_un = 2.0f * (best_s0 - best_s1);
                float intercept_un = 2.0f * seg_start * (best_s0 - best_s1)
                                   + (best_s1 * best_s1 - best_s0 * best_s0);

                segs.push_back({seg_start, slope_un, intercept_un});
            }
        }
    }
}

// =========================================================================
// Piecewise-Linear Soft Demapping
// =========================================================================

void SymbolMapper::demapSoftPWL(ComplexSample sym, float noise_var,
                                std::vector<float>& llrs) const {
    llrs.resize(bps_);
    float nv = std::max(noise_var, 1e-10f);
    float inv_nv = 1.0f / nv;

    // BPSK / QPSK: same as exact (already O(1) per bit)
    if (bps_ == 1) {
        llrs[0] = std::clamp(4.0f * sym.real() * inv_nv, -20.0f, 20.0f);
        return;
    }
    if (bps_ == 2) {
        float sc = 4.0f * norm_ * inv_nv;
        llrs[0] = std::clamp(sc * sym.real(), -20.0f, 20.0f);
        llrs[1] = std::clamp(sc * sym.imag(), -20.0f, 20.0f);
        return;
    }

    // Square QAM with PWL tables
    if (pwl_tables_.empty() || grid_size_ == 0) {
        // Fallback to exact
        demapSoft(sym, noise_var, llrs);
        return;
    }

    size_t bits_dim = bps_ / 2;
    float coords[2] = { sym.real(), sym.imag() };

    for (size_t dim = 0; dim < 2; ++dim) {
        float r = coords[dim];
        for (size_t b = 0; b < bits_dim; ++b) {
            size_t bit_idx = dim * bits_dim + b;
            auto& segs = pwl_tables_[dim][b];

            // Binary search for the right segment
            // Segments are sorted by start coordinate
            size_t lo = 0, hi = segs.size();
            while (lo + 1 < hi) {
                size_t mid = (lo + hi) / 2;
                if (segs[mid].start <= r) lo = mid;
                else hi = mid;
            }

            // Evaluate: LLR = (slope * r + (intercept - slope*start)) / σ²
            // Actually: intercept already includes seg_start evaluation
            // LLR(r) = [slope_un * (r - start) + intercept_un] / σ²
            // Wait, let me recalculate. intercept_un = LLR(start) * σ²
            // LLR(r) = [intercept_un + slope_un * (r - start)] / σ²

            float raw = (segs[lo].intercept + segs[lo].slope * (r - segs[lo].start)) * inv_nv;
            llrs[bit_idx] = std::clamp(raw, -20.0f, 20.0f);
        }
    }
}

void SymbolMapper::demapSoftPWL(const ComplexBuf& symbols, float noise_var,
                                std::vector<float>& llrs) const {
    llrs.resize(symbols.size() * bps_);
    std::vector<float> tmp;
    for (size_t s = 0; s < symbols.size(); ++s) {
        demapSoftPWL(symbols[s], noise_var, tmp);
        for (size_t b = 0; b < bps_; ++b) {
            llrs[s * bps_ + b] = tmp[b];
        }
    }
}

void SymbolMapper::demapSoftPWL(const ComplexBuf& symbols,
                                const std::vector<float>& noise_var,
                                std::vector<float>& llrs) const {
    llrs.resize(symbols.size() * bps_);
    std::vector<float> tmp;
    for (size_t s = 0; s < symbols.size(); ++s) {
        float nv = (s < noise_var.size()) ? noise_var[s] : 1e-3f;
        demapSoftPWL(symbols[s], nv, tmp);
        for (size_t b = 0; b < bps_; ++b) {
            llrs[s * bps_ + b] = tmp[b];
        }
    }
}

} // namespace dsca
