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
#include <limits>
#include <stdexcept>
#include <vector>

namespace dsca {

// =========================================================================
// Construction
// =========================================================================

SymbolMapper::SymbolMapper(Modulation mod) : mod_(mod) {
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

    // Gray decoding helper: Gray code → natural binary
    auto grayDecode = [](size_t g) -> size_t {
        size_t n = g;
        while (g >>= 1) n ^= g;
        return n;
    };

    // Build Gray-coded constellation.
    // Symbol index bits: [I-bits(high) | Q-bits(low)]
    // For each dimension: Gray-decoded value gives level index.
    // Level 0 = most positive coordinate, matching soft demapper convention
    // where MSB=0 → positive coordinate → positive LLR.
    //
    // Coordinate = ((grid-1) - 2*level) * norm_
    //   level 0 → +(grid-1), level 1 → +(grid-3), ..., level grid-1 → -(grid-1)
    const_.resize(order);
    for (size_t idx = 0; idx < order; ++idx) {
        size_t i_gray = idx >> bits_dim;         // high bits → I dimension
        size_t q_gray = idx & (grid - 1);        // low bits  → Q dimension

        size_t i_level = grayDecode(i_gray);
        size_t q_level = grayDecode(q_gray);

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

        // Natural-to-Gray: g = n ^ (n >> 1)
        auto natToGray = [](size_t n) -> size_t { return n ^ (n >> 1); };

        for (int di = -2; di <= 2; ++di) {
            for (int dq = -2; dq <= 2; ++dq) {
                int il = i_level + di;
                int ql = q_level + dq;
                if (il < 0 || il >= static_cast<int>(grid) ||
                    ql < 0 || ql >= static_cast<int>(grid)) continue;
                size_t gray_i = natToGray(static_cast<size_t>(il));
                size_t gray_q = natToGray(static_cast<size_t>(ql));
                size_t idx = (gray_i << bits_dim) | gray_q;
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

    // Gray code: natural-to-Gray
    auto natToGray = [](size_t n) -> size_t { return n ^ (n >> 1); };

    // For each bit in this dimension, determine which levels have bit=0 vs bit=1
    // Bit patterns: for dimension's bits (bits_dim bits), level k maps to Gray(k)
    // bit b of Gray(k) determines if this level has bit b = 0 or 1

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
                size_t gray = natToGray(k);
                level_bit[k] = static_cast<uint8_t>(
                    (gray >> (bits_dim - 1 - b)) & 1);
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

            // Collect all breakpoints (boundaries + lo/hi), sorted ascending
            std::vector<float> bkpts;
            bkpts.push_back(lo);
            for (auto bp : boundaries) bkpts.push_back(bp);
            bkpts.push_back(hi);
            std::sort(bkpts.begin(), bkpts.end());

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

} // namespace dsca
