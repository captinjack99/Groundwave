/**
 * @file interleaver.cpp
 * @brief Block bit interleaver
 *
 * Matrix interleaving: bits are written into an R×C matrix column-by-column,
 * then read row-by-row. This maps consecutive input positions to positions
 * spaced C apart in the output, spreading burst errors.
 */

#include "interleaver.hpp"
#include <cmath>
#include <algorithm>

namespace dsca {

namespace {

inline bool getBit(const uint8_t* d, size_t i) {
    return (d[i >> 3] >> (7 - (i & 7))) & 1;
}

inline void setBit(uint8_t* d, size_t i, bool v) {
    if (v) d[i >> 3] |=  (uint8_t(1) << (7 - (i & 7)));
    else   d[i >> 3] &= ~(uint8_t(1) << (7 - (i & 7)));
}

} // anonymous

BitInterleaver::BitInterleaver(size_t num_bits, size_t num_cols) : n_(num_bits) {
    if (n_ == 0) return;

    // Default columns: sqrt(n) rounded to a nice multiple
    if (num_cols == 0) {
        num_cols = static_cast<size_t>(std::ceil(std::sqrt(static_cast<double>(n_))));
        num_cols = ((num_cols + 7) / 8) * 8;
        if (num_cols == 0) num_cols = 1;
    }

    // The matrix interleaver only produces a true bijection when C divides n_.
    // If the requested C isn't a divisor of n_, snap to the nearest divisor
    // (preference: divisors close to the user's requested value, biased high
    // for byte alignment).
    auto adjustColsToDivisor = [](size_t n, size_t requested) -> size_t {
        if (n == 0) return 1;
        if (n % requested == 0) return requested;
        size_t best = 1;
        size_t best_dist = (requested > 1) ? requested - 1 : 0;
        for (size_t c = 1; c * c <= n; ++c) {
            if (n % c != 0) continue;
            size_t c2 = n / c;
            for (size_t cand : {c, c2}) {
                size_t d = (cand > requested) ? cand - requested : requested - cand;
                if (d < best_dist || (d == best_dist && cand > best)) {
                    best_dist = d;
                    best = cand;
                }
            }
        }
        return best;
    };

    size_t C = adjustColsToDivisor(n_, num_cols);

    // If the only available divisor is 1 (n_ is prime), the matrix
    // interleaver collapses to the identity — no interleaving at all,
    // which defeats the point (adjacent coded bits stay adjacent and a
    // burst hits one LDPC neighborhood). All current modcods use composite
    // sizes (n_base·q) so this is unreachable today, but it's a latent
    // trap for a future block size. Fall back to a coprime-stride
    // permutation perm[i] = (i·step) mod n_, which is bijective for any
    // step coprime to n_ and spreads neighbors apart. (#41)
    if (C <= 1 && n_ > 1) {
        auto gcd = [](size_t a, size_t b) {
            while (b) { size_t t = a % b; a = b; b = t; }
            return a;
        };
        size_t step = static_cast<size_t>(
            std::sqrt(static_cast<double>(n_))) | size_t(1);
        if (step < 2) step = 2;
        while (step < n_ && gcd(step, n_) != 1) ++step;
        if (step >= n_) step = 1;   // degenerate guard (n_ <= 2)
        cols_ = 0; rows_ = 0;       // not a matrix interleaver
        perm_.resize(n_);
        inv_perm_.resize(n_);
        for (size_t i = 0; i < n_; ++i) {
            size_t out_pos = (i * step) % n_;
            perm_[i] = out_pos;
            inv_perm_[out_pos] = i;
        }
        return;
    }

    size_t R = n_ / C;  // exact since C | n_

    cols_ = C;
    rows_ = R;

    // Build permutation: write column-by-column, read row-by-row.
    // Input position i is at (row = i % R, col = i / R) in the matrix.
    // Output position is row * C + col.
    perm_.resize(n_);
    inv_perm_.resize(n_);

    for (size_t i = 0; i < n_; ++i) {
        size_t col = i / R;
        size_t row = i % R;
        size_t out_pos = row * C + col;
        perm_[i] = out_pos;
        inv_perm_[out_pos] = i;
    }
}

void BitInterleaver::interleave(const uint8_t* in, uint8_t* out) const {
    size_t n_bytes = (n_ + 7) / 8;
    std::fill(out, out + n_bytes, uint8_t(0));
    for (size_t i = 0; i < n_; ++i) {
        setBit(out, perm_[i], getBit(in, i));
    }
}

void BitInterleaver::deinterleave(const uint8_t* in, uint8_t* out) const {
    size_t n_bytes = (n_ + 7) / 8;
    std::fill(out, out + n_bytes, uint8_t(0));
    for (size_t i = 0; i < n_; ++i) {
        setBit(out, inv_perm_[i], getBit(in, i));
    }
}

void BitInterleaver::interleave(const float* in, float* out) const {
    for (size_t i = 0; i < n_; ++i) {
        out[perm_[i]] = in[i];
    }
}

void BitInterleaver::deinterleave(const float* in, float* out) const {
    for (size_t i = 0; i < n_; ++i) {
        out[inv_perm_[i]] = in[i];
    }
}

} // namespace dsca
