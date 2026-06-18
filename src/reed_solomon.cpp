/**
 * @file reed_solomon.cpp
 * @brief RS(N, N-16) over GF(256) implementation.
 *
 * Algorithm notes:
 *   - GF(256) field with primitive poly 0x11D (CCSDS / DVB).
 *   - Generator g(x) = ∏_{i=1..16} (x − α^i), so syndromes evaluate the
 *     received polynomial at α^1..α^16.
 *   - Decoder: classical Berlekamp-Massey to find the error locator
 *     polynomial Λ(x), then Chien search to find roots (= error
 *     positions), then Forney to compute magnitudes via the error
 *     evaluator polynomial Ω(x) = (S(x) · Λ(x)) mod x^16.
 *
 * No external dependencies. Tested round-trip against the unit test
 * in tests/reed_solomon_test.cpp.
 */
#include "reed_solomon.hpp"
#include <algorithm>
#include <cstring>

namespace gw {

namespace {
constexpr uint16_t PRIM_POLY = 0x11D;   // standard GF(256) primitive
constexpr uint8_t  ALPHA     = 0x02;    // generator element
} // anonymous

ReedSolomon::ReedSolomon() {
    // Build log/antilog tables. exp_[i] = α^i for i in [0..510]; the
    // doubled-size table avoids a modulo for multiplication.
    uint16_t x = 1;
    for (int i = 0; i < 255; ++i) {
        exp_[i] = static_cast<uint8_t>(x);
        log_[exp_[i]] = static_cast<uint8_t>(i);
        x <<= 1;
        if (x & 0x100) x ^= PRIM_POLY;
    }
    for (int i = 255; i < 512; ++i) exp_[i] = exp_[i - 255];
    log_[0] = 0;  // convention; not used (we check for 0 before)

    // Build generator polynomial g(x) = ∏(x − α^i), i=1..16.
    // Start with g(x) = 1 (a single coefficient), then multiply by each
    // (x − α^i) which lengthens the polynomial by 1 each iteration.
    std::array<uint8_t, PARITY_BYTES + 1> g{};
    g[0] = 1;
    size_t g_len = 1;
    for (int i = 1; i <= static_cast<int>(PARITY_BYTES); ++i) {
        // Multiply current g by (x − α^i). New length = g_len + 1.
        // (x − α^i) coefficients: [−α^i, 1] = [α^i, 1] in GF(256)
        // since subtraction = addition (XOR).
        std::array<uint8_t, PARITY_BYTES + 1> next{};
        for (size_t j = 0; j < g_len; ++j) {
            next[j]     ^= mul(g[j], exp_[i]);
            next[j + 1] ^= g[j];
        }
        g = next;
        ++g_len;
    }
    // g_len is now PARITY_BYTES + 1 = 17. Coefficients g[0..16] with
    // g[16] = 1 (leading).
    gen_ = g;
}

bool ReedSolomon::encode(uint8_t* block, size_t data_len) const {
    if (data_len < 1) return false;
    if (data_len + PARITY_BYTES > MAX_BLOCK) return false;

    // Long division by g(x). Implemented as an LFSR over the data bytes.
    // The parity bytes are the remainder, stored at block[data_len..].
    std::memset(block + data_len, 0, PARITY_BYTES);
    for (size_t i = 0; i < data_len; ++i) {
        uint8_t fb = block[i] ^ block[data_len];
        if (fb != 0) {
            for (size_t j = 0; j < PARITY_BYTES - 1; ++j) {
                block[data_len + j] = block[data_len + j + 1] ^
                                       mul(gen_[PARITY_BYTES - 1 - j], fb);
            }
            block[data_len + PARITY_BYTES - 1] = mul(gen_[0], fb);
        } else {
            for (size_t j = 0; j < PARITY_BYTES - 1; ++j) {
                block[data_len + j] = block[data_len + j + 1];
            }
            block[data_len + PARITY_BYTES - 1] = 0;
        }
    }
    return true;
}

int ReedSolomon::decode(uint8_t* block, size_t data_len) const {
    if (data_len < 1) return -1;
    size_t total = data_len + PARITY_BYTES;
    if (total > MAX_BLOCK) return -1;

    // 1. Syndromes: S_k = R(α^(k+1)) for k = 0..15. Evaluated via
    // Horner's method on the array [block[0], ..., block[total-1]]
    // treated as a polynomial R(x) = block[0]·x^(total-1) + ... +
    // block[total-1].
    std::array<uint8_t, PARITY_BYTES> S{};
    bool any_nonzero = false;
    for (size_t k = 0; k < PARITY_BYTES; ++k) {
        uint8_t a_k = exp_[k + 1];  // α^(k+1)
        uint8_t acc = 0;
        for (size_t i = 0; i < total; ++i) {
            acc = static_cast<uint8_t>(mul(acc, a_k) ^ block[i]);
        }
        S[k] = acc;
        if (acc != 0) any_nonzero = true;
    }
    if (!any_nonzero) return 0;  // no errors

    // 2. Berlekamp-Massey: find error locator Λ(x) of minimal degree
    // such that Λ(x)·S(x) ≡ 0 (mod x^16).
    std::array<uint8_t, PARITY_BYTES + 1> Lambda{}, B{};
    Lambda[0] = 1;
    B[0]      = 1;
    int L = 0;
    int m = 1;
    uint8_t b = 1;
    for (int n = 0; n < static_cast<int>(PARITY_BYTES); ++n) {
        // Discrepancy: d_n = S_n + Σ_{i=1..L} Λ_i · S_{n-i}
        uint8_t d = S[n];
        for (int i = 1; i <= L; ++i) {
            if (n - i >= 0) d ^= mul(Lambda[i], S[n - i]);
        }
        if (d == 0) {
            ++m;
        } else if (2 * L <= n) {
            std::array<uint8_t, PARITY_BYTES + 1> T = Lambda;
            uint8_t coef = div(d, b);
            for (size_t i = 0; i < PARITY_BYTES + 1 - static_cast<size_t>(m); ++i) {
                Lambda[i + m] ^= mul(coef, B[i]);
            }
            L = n + 1 - L;
            B = T;
            b = d;
            m = 1;
        } else {
            uint8_t coef = div(d, b);
            for (size_t i = 0; i < PARITY_BYTES + 1 - static_cast<size_t>(m); ++i) {
                Lambda[i + m] ^= mul(coef, B[i]);
            }
            ++m;
        }
    }
    if (L < 1 || L > static_cast<int>(PARITY_BYTES / 2)) {
        // Too many errors — uncorrectable.
        return -1;
    }

    // 3. Chien search: find roots of Λ(x). For each codeword position
    // i in [0..total-1], compute Λ(α^(-i)) — if zero, position i is in
    // error. We test for x = α^(-i), where i is the byte index from
    // the END of the codeword (per RS convention), so the position in
    // our array is total-1-i_from_end.
    std::vector<int> err_pos;
    err_pos.reserve(PARITY_BYTES);
    for (int i = 0; i < static_cast<int>(total); ++i) {
        // Test root α^(-(total-1-i)) — codeword index `i` corresponds
        // to power exponent (total-1-i). The locator's root is the
        // inverse of α^(exponent).
        int exp_idx = static_cast<int>(total) - 1 - i;
        // x = α^(-exp_idx). pow_x[k] = x^k.
        uint8_t acc = 0;
        uint8_t pow_x = 1;
        // x = α^(255 - exp_idx mod 255)
        int neg_exp = (255 - (exp_idx % 255)) % 255;
        uint8_t x = exp_[neg_exp];
        for (int k = 0; k <= L; ++k) {
            acc = static_cast<uint8_t>(acc ^ mul(Lambda[k], pow_x));
            pow_x = mul(pow_x, x);
        }
        if (acc == 0) {
            err_pos.push_back(i);
            if (static_cast<int>(err_pos.size()) > L) return -1;  // sanity
        }
    }
    if (static_cast<int>(err_pos.size()) != L) {
        // Locator degree disagrees with root count — uncorrectable.
        return -1;
    }

    // 4. Forney: error magnitudes via Ω(x) = (S(x) · Λ(x)) mod x^16
    // and the formal derivative Λ'(x). e_j = -Ω(x_j^-1) / Λ'(x_j^-1).
    // We compute Ω first, then evaluate at each error position.
    std::array<uint8_t, PARITY_BYTES> Omega{};
    for (size_t i = 0; i < PARITY_BYTES; ++i) {
        uint8_t acc = 0;
        for (size_t j = 0; j <= static_cast<size_t>(L) && j <= i; ++j) {
            acc ^= mul(Lambda[j], S[i - j]);
        }
        Omega[i] = acc;
    }
    // Λ'(x): formal derivative. In GF(2) characteristic, Λ'(x) =
    // Σ i·Λ_i·x^(i-1) where i·Λ_i = Λ_i if i is odd, 0 if i is even.
    std::array<uint8_t, PARITY_BYTES + 1> Lambda_prime{};
    for (int i = 1; i <= L; ++i) {
        if (i & 1) Lambda_prime[i - 1] = Lambda[i];
    }

    for (int j = 0; j < L; ++j) {
        int i = err_pos[static_cast<size_t>(j)];
        int exp_idx = static_cast<int>(total) - 1 - i;
        int neg_exp = (255 - (exp_idx % 255)) % 255;
        uint8_t x_inv = exp_[neg_exp];

        // Evaluate Ω(x_inv) and Λ'(x_inv).
        uint8_t omega_v = 0, prime_v = 0;
        uint8_t pw = 1;
        for (size_t k = 0; k < PARITY_BYTES; ++k) {
            omega_v ^= mul(Omega[k], pw);
            pw = mul(pw, x_inv);
        }
        pw = 1;
        for (size_t k = 0; k < PARITY_BYTES; ++k) {
            prime_v ^= mul(Lambda_prime[k], pw);
            pw = mul(pw, x_inv);
        }
        if (prime_v == 0) return -1;
        // Forney for generator g(x) = ∏(x − α^i), i=1..16 (b=1):
        //   e_j = Ω(X_j^{-1}) / Λ'(X_j^{-1})
        // No extra multiplier needed in GF(256) since X_j^(1-b) = 1.
        uint8_t mag = div(omega_v, prime_v);
        block[i] ^= mag;
    }

    // 5. Post-correction syndrome recheck (defense-in-depth). A valid
    // correction drives ALL syndromes to zero; if any remain nonzero the
    // block was beyond the correction radius and Forney produced a
    // spurious "correction" — report uncorrectable rather than passing
    // corrupted data downstream. (#40)
    for (size_t k = 0; k < PARITY_BYTES; ++k) {
        uint8_t a_k = exp_[k + 1];
        uint8_t acc = 0;
        for (size_t i = 0; i < total; ++i) {
            acc = static_cast<uint8_t>(mul(acc, a_k) ^ block[i]);
        }
        if (acc != 0) return -1;
    }

    return L;
}

std::vector<uint8_t> ReedSolomon::encodeCopy(const uint8_t* data,
                                              size_t data_len) const {
    // Range-construct the data prefix, then grow for parity — avoids a
    // raw memcpy whose size GCC's -O3 _FORTIFY stringop-overflow pass
    // mis-bounds (it derives data_len could be up to SIZE_MAX-16 from the
    // vector-sizing no-overflow constraint and flags it against
    // PTRDIFF_MAX, a known false positive; the copy is always in-bounds).
    std::vector<uint8_t> block(data, data + data_len);
    block.resize(data_len + PARITY_BYTES, 0);
    encode(block.data(), data_len);
    return block;
}

} // namespace gw
