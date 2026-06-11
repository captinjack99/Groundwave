/**
 * @file reed_solomon_test.cpp
 * @brief Reed-Solomon round-trip and error-correction validation.
 *
 * Verifies:
 *   1. Encode+decode round-trips with no errors → message recovered, 0 errors corrected.
 *   2. Single-byte errors are corrected.
 *   3. Multi-byte errors up to 8 are corrected.
 *   4. 9 errors → uncorrectable (return -1).
 *   5. Variable block sizes work (shortened RS).
 */
#include <algorithm>
#include "reed_solomon.hpp"
#include <cstdio>
#include <cstring>
#include <random>
#include <vector>

using namespace gw;

namespace {

int g_passed = 0, g_failed = 0;

#define CHECK(cond, msg)                                                       \
    do {                                                                       \
        if (cond) { ++g_passed; std::printf("  [PASS] %s\n", msg); }            \
        else      { ++g_failed; std::printf("  [FAIL] %s\n", msg); }            \
    } while (0)

void test_clean_roundtrip() {
    std::printf("\n=== RS clean round-trip ===\n");
    ReedSolomon rs;
    std::mt19937 rng(0xC0DEC0DE);
    for (size_t data_len : { static_cast<size_t>(64),
                              static_cast<size_t>(128),
                              static_cast<size_t>(200),
                              static_cast<size_t>(239) }) {
        std::vector<uint8_t> block(data_len + ReedSolomon::PARITY_BYTES);
        for (size_t i = 0; i < data_len; ++i) {
            block[i] = static_cast<uint8_t>(rng() & 0xFF);
        }
        std::vector<uint8_t> orig(block.begin(), block.begin() + data_len);
        bool ok = rs.encode(block.data(), data_len);
        int n_err = rs.decode(block.data(), data_len);
        bool match = std::memcmp(block.data(), orig.data(), data_len) == 0;
        char label[64];
        std::snprintf(label, sizeof(label),
                       "data_len=%zu round-trip (decoded n_err=%d)",
                       data_len, n_err);
        CHECK(ok && n_err == 0 && match, label);
    }
}

void test_single_error() {
    std::printf("\n=== RS single-byte error correction ===\n");
    ReedSolomon rs;
    std::mt19937 rng(0x12345);
    size_t data_len = 200;
    for (int trial = 0; trial < 5; ++trial) {
        std::vector<uint8_t> block(data_len + ReedSolomon::PARITY_BYTES);
        for (size_t i = 0; i < data_len; ++i) {
            block[i] = static_cast<uint8_t>(rng() & 0xFF);
        }
        std::vector<uint8_t> orig(block.begin(), block.begin() + data_len);
        rs.encode(block.data(), data_len);
        size_t total = data_len + ReedSolomon::PARITY_BYTES;
        size_t err_pos = rng() % total;
        uint8_t err_val = static_cast<uint8_t>((rng() & 0xFE) | 1);  // nonzero
        block[err_pos] ^= err_val;
        int n_err = rs.decode(block.data(), data_len);
        bool match = std::memcmp(block.data(), orig.data(), data_len) == 0;
        char label[80];
        std::snprintf(label, sizeof(label),
                       "single error at pos %zu (n_err=%d, match=%d)",
                       err_pos, n_err, match ? 1 : 0);
        CHECK(n_err == 1 && match, label);
    }
}

void test_multi_errors() {
    std::printf("\n=== RS multi-byte error correction ===\n");
    ReedSolomon rs;
    std::mt19937 rng(0xABCDEF);
    size_t data_len = 200;
    for (int n_errs : {2, 4, 6, 8}) {
        std::vector<uint8_t> block(data_len + ReedSolomon::PARITY_BYTES);
        for (size_t i = 0; i < data_len; ++i) {
            block[i] = static_cast<uint8_t>(rng() & 0xFF);
        }
        std::vector<uint8_t> orig(block.begin(), block.begin() + data_len);
        rs.encode(block.data(), data_len);
        size_t total = data_len + ReedSolomon::PARITY_BYTES;
        // Sprinkle n_errs distinct error positions
        std::vector<size_t> positions;
        while (static_cast<int>(positions.size()) < n_errs) {
            size_t p = rng() % total;
            if (std::find(positions.begin(), positions.end(), p) == positions.end())
                positions.push_back(p);
        }
        for (size_t p : positions) {
            block[p] ^= static_cast<uint8_t>((rng() & 0xFE) | 1);
        }
        int n_corrected = rs.decode(block.data(), data_len);
        bool match = std::memcmp(block.data(), orig.data(), data_len) == 0;
        char label[80];
        std::snprintf(label, sizeof(label),
                       "%d errors corrected (n_corrected=%d, match=%d)",
                       n_errs, n_corrected, match ? 1 : 0);
        CHECK(n_corrected == n_errs && match, label);
    }
}

void test_uncorrectable() {
    std::printf("\n=== RS uncorrectable (> 8 errors) ===\n");
    ReedSolomon rs;
    std::mt19937 rng(0xDEAD);
    size_t data_len = 200;
    std::vector<uint8_t> block(data_len + ReedSolomon::PARITY_BYTES);
    for (size_t i = 0; i < data_len; ++i) {
        block[i] = static_cast<uint8_t>(rng() & 0xFF);
    }
    rs.encode(block.data(), data_len);
    // Inject 10 errors — beyond correction capacity.
    size_t total = data_len + ReedSolomon::PARITY_BYTES;
    for (int i = 0; i < 10; ++i) {
        block[(i * 17) % total] ^= 0x5A;
    }
    std::vector<uint8_t> corrupted = block;   // keep the over-capacity block
    int n = rs.decode(block.data(), data_len);
    // With the post-correction syndrome recheck (#40), a > t-error block
    // MUST be reported uncorrectable (n == -1) — never a spurious
    // "success" that silently rewrites bytes. Assert the strong property.
    char label[80];
    std::snprintf(label, sizeof(label),
                   "10 errors → uncorrectable, n==-1 (got n=%d)", n);
    CHECK(n == -1, label);
    // And on failure the decoder must not have mangled the data into a
    // *different valid-looking* codeword: the block is unchanged or
    // flagged — we require the returned data region to equal the
    // corrupted input (no partial spurious correction applied).
    bool unchanged = (n == -1) &&
                     std::equal(block.begin(), block.begin() + static_cast<ptrdiff_t>(data_len),
                                corrupted.begin());
    CHECK(unchanged, "uncorrectable block left unmodified (no spurious correction)");

    // Fuzz the #40 invariant: a *reported* success must always yield a
    // valid codeword (zero residual syndromes), never a partially-applied
    // / inconsistent "correction". We can't read the syndromes directly,
    // but a valid codeword re-decodes to itself with 0 corrections — so
    // for every over-capacity block where decode() reports n>=0, decoding
    // the result again MUST return 0. (Note: decode() landing on a *wrong*
    // valid codeword for >t errors is inherent RS behavior, not a defect —
    // that is why we test "result is a valid codeword", not "always -1".)
    int invalid_success = 0;
    for (int trial = 0; trial < 3000; ++trial) {
        std::vector<uint8_t> b(data_len + ReedSolomon::PARITY_BYTES);
        for (size_t i = 0; i < data_len; ++i) b[i] = static_cast<uint8_t>(rng() & 0xFF);
        rs.encode(b.data(), data_len);
        size_t tot = data_len + ReedSolomon::PARITY_BYTES;
        for (int e = 0; e < 9 + (trial % 4); ++e) {   // 9..12 errors (> t=8)
            size_t p = rng() % tot;
            uint8_t v; do { v = static_cast<uint8_t>(rng() & 0xFF); } while (v == 0);
            b[p] ^= v;
        }
        int rr = rs.decode(b.data(), data_len);
        if (rr >= 0) {
            // Re-decode: a genuine valid codeword needs 0 further fixes.
            int rr2 = rs.decode(b.data(), data_len);
            if (rr2 != 0) ++invalid_success;
        }
    }
    char fz[96];
    std::snprintf(fz, sizeof(fz),
                   "3000 over-cap fuzz: %d reported-success-but-invalid-codeword (want 0)",
                   invalid_success);
    CHECK(invalid_success == 0, fz);
}

} // anonymous

int main() {
    std::printf("=== Groundwave Reed-Solomon Test Suite ===\n");
    test_clean_roundtrip();
    test_single_error();
    test_multi_errors();
    test_uncorrectable();
    std::printf("\n=== Result: %d passed, %d failed ===\n",
                 g_passed, g_failed);
    return g_failed == 0 ? 0 : 1;
}
