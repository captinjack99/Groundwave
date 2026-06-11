/**
 * @file hierarchical_test.cpp
 * @brief Round-trip tests for asymmetric hierarchical modulation.
 *
 * Covers two real bugs the audit found that had NO test coverage:
 *
 *   #2 — Asymmetric TX/RX bit-misalignment: the engine splits one codeword
 *        into a contiguous HP partition (bits [0, hp_total_bits)) followed
 *        by a contiguous LP partition (bits [hp_total_bits, …)). The RX
 *        reconstructs HP-then-LP LLRs with NO byte padding, so the TX must
 *        feed the LP layer the bits starting at the exact bit hp_total_bits.
 *        We reproduce that exact bit layout and assert a clean-channel
 *        round-trip recovers every HP and LP bit. Any byte-alignment of the
 *        LP partition (the old bug) corrupts the LP layer whenever
 *        hp_total_bits % 8 != 0 (e.g. QPSK/QAM256, hp_total_bits=540).
 *
 *   #3 — extractBits / constellation-index truncation to 8 bits: a single
 *        layer can carry up to 10 bits (QPSK/QAM4096 LP=10, QAM1024/QAM4096
 *        HP=10). A uint8_t index silently dropped the top bits. The wide
 *        modes below would fail the round-trip if that truncation returned.
 */
#include "hierarchical_mod.hpp"
#include <cstdio>
#include <random>
#include <vector>
#include <string>

using namespace gw;

static int tests_passed = 0;
static int tests_failed = 0;

#define TEST(name) std::printf("  %-58s", name)
#define PASS()     do { std::printf("[PASS]\n"); ++tests_passed; } while (0)
#define FAIL(msg)  do { std::printf("[FAIL] %s\n", msg); ++tests_failed; } while (0)

namespace {

inline bool getBit(const uint8_t* d, size_t i) {
    return (d[i >> 3] >> (7 - (i & 7))) & 1;
}
inline void setBit(uint8_t* d, size_t i, bool v) {
    if (v) d[i >> 3] |=  static_cast<uint8_t>(1u << (7 - (i & 7)));
    else   d[i >> 3] &= ~static_cast<uint8_t>(1u << (7 - (i & 7)));
}

// Mirror the engine's asymmetric TX/RX bit handling exactly, isolating the
// HierarchicalMapper. Returns the number of (HP+LP) partition bit errors on
// a clean (near-noiseless) channel — must be zero.
int roundTripErrors(HierarchicalMode mode, size_t num_syms, uint32_t seed) {
    HierarchicalConfig cfg;
    cfg.mode    = mode;
    cfg.alpha   = 2.0f;
    cfg.enabled = true;
    HierarchicalMapper mapper(cfg);
    if (!mapper.isEnabled()) return -1;

    const uint8_t hp_bps = mapper.hpBPS();
    const uint8_t lp_bps = mapper.lpBPS();
    const size_t  hp_total_bits = num_syms * hp_bps;
    const size_t  lp_total_bits = num_syms * lp_bps;
    const size_t  cw_bits = hp_total_bits + lp_total_bits;

    // Random "interleaved codeword": HP partition then LP partition,
    // contiguous (exactly the engine's layout).
    std::mt19937 rng(seed);
    std::vector<uint8_t> cw((cw_bits + 7) / 8, 0);
    for (auto& b : cw) b = static_cast<uint8_t>(rng() & 0xFF);

    // ---- TX: HP starts at bit 0; LP partition repacked byte-aligned from
    //          bit hp_total_bits (matches the fixed engine TX path). ----
    std::vector<uint8_t> lp_packed((lp_total_bits + 7) / 8, 0);
    for (size_t j = 0; j < lp_total_bits; ++j)
        setBit(lp_packed.data(), j, getBit(cw.data(), hp_total_bits + j));

    ComplexBuf syms;
    mapper.map(cw.data(), hp_total_bits, lp_packed.data(), lp_total_bits, syms);
    if (syms.size() != num_syms) return -2;

    // ---- RX (clean channel): soft-demap HP and LP, hard-decide, then
    //      concatenate HP-then-LP exactly as the engine RX branch does. ----
    std::vector<float> hp_llrs, lp_llrs;
    mapper.demapSoftHP(syms, /*noise_var=*/0.001f, hp_llrs);
    mapper.demapSoftLP(syms, /*noise_var=*/0.001f, lp_llrs);

    std::vector<uint8_t> rx((cw_bits + 7) / 8, 0);
    // Convention: positive LLR ⇒ bit 0.
    for (size_t i = 0; i < hp_total_bits && i < hp_llrs.size(); ++i)
        setBit(rx.data(), i, hp_llrs[i] < 0.f);
    for (size_t i = 0; i < lp_total_bits && i < lp_llrs.size(); ++i)
        setBit(rx.data(), hp_total_bits + i, lp_llrs[i] < 0.f);

    int errs = 0;
    for (size_t i = 0; i < cw_bits; ++i)
        if (getBit(cw.data(), i) != getBit(rx.data(), i)) ++errs;
    return errs;
}

// Hard-demap round-trip (exercises extractBits + demapHP/demapLP integer
// paths, where the 8-bit truncation bug lived).
int hardRoundTripErrors(HierarchicalMode mode, size_t num_syms, uint32_t seed) {
    HierarchicalConfig cfg;
    cfg.mode = mode; cfg.alpha = 2.0f; cfg.enabled = true;
    HierarchicalMapper mapper(cfg);
    if (!mapper.isEnabled()) return -1;
    const uint8_t hp_bps = mapper.hpBPS();
    const uint8_t lp_bps = mapper.lpBPS();
    const size_t  hp_bits = num_syms * hp_bps;
    const size_t  lp_bits = num_syms * lp_bps;

    std::mt19937 rng(seed);
    std::vector<uint8_t> hp_src((hp_bits + 7) / 8, 0);
    std::vector<uint8_t> lp_src((lp_bits + 7) / 8, 0);
    for (auto& b : hp_src) b = static_cast<uint8_t>(rng() & 0xFF);
    for (auto& b : lp_src) b = static_cast<uint8_t>(rng() & 0xFF);

    ComplexBuf syms;
    mapper.map(hp_src.data(), hp_bits, lp_src.data(), lp_bits, syms);

    std::vector<uint8_t> hp_out, lp_out;
    mapper.demapHP(syms, hp_out);
    mapper.demapLP(syms, lp_out);

    int errs = 0;
    for (size_t i = 0; i < hp_bits; ++i)
        if (getBit(hp_src.data(), i) != getBit(hp_out.data(), i)) ++errs;
    for (size_t i = 0; i < lp_bits; ++i)
        if (getBit(lp_src.data(), i) != getBit(lp_out.data(), i)) ++errs;
    return errs;
}

struct Case { const char* name; HierarchicalMode mode; size_t num_syms; };

} // anonymous

int main() {
    std::printf("\n=== Asymmetric hierarchical modulation round-trip ===\n");

    // num_syms chosen so hp_total_bits is NOT a multiple of 8 where possible,
    // which is exactly the case the old byte-aligned TX corrupted.
    //   QPSK/QAM256: hp=2 → 270 syms → hp_total_bits=540 (540 % 8 = 4)
    //   QAM64/QAM256: hp=6 → 270 syms → 1620 (% 8 = 4)
    //   QPSK/QAM4096: lp=10 (wide LP, #3)   → 201 syms → hp_total=402 (% 8 = 2)
    //   QAM1024/QAM4096: hp=10 (wide HP, #3)→ 201 syms → hp_total=2010 (% 8 = 2)
    //   QAM64/QAM4096: 6+6 wide symmetric   → 173 syms → 1038 (% 8 = 6)
    Case cases[] = {
        { "QPSK/QAM256  (2+6, non-byte-aligned HP)", HierarchicalMode::QPSK_QAM256,   270 },
        { "QAM64/QAM256 (6+2, non-byte-aligned HP)", HierarchicalMode::QAM64_QAM256,  270 },
        { "QPSK/QAM4096 (2+10, wide LP)",            HierarchicalMode::QPSK_QAM4096,  201 },
        { "QAM1024/QAM4096 (10+2, wide HP)",         HierarchicalMode::QAM1024_QAM4096, 201 },
        { "QAM64/QAM4096 (6+6, wide symmetric)",     HierarchicalMode::QAM64_QAM4096, 173 },
    };

    for (const auto& c : cases) {
        TEST((std::string("soft round-trip: ") + c.name).c_str());
        int e = roundTripErrors(c.mode, c.num_syms, 0x1E12345u);
        if (e == 0) PASS();
        else {
            char buf[64];
            std::snprintf(buf, sizeof(buf), "%d bit errors (clean channel)", e);
            FAIL(buf);
        }
    }

    for (const auto& c : cases) {
        TEST((std::string("hard round-trip: ") + c.name).c_str());
        int e = hardRoundTripErrors(c.mode, c.num_syms, 0x5EED0001u);
        if (e == 0) PASS();
        else {
            char buf[64];
            std::snprintf(buf, sizeof(buf), "%d bit errors", e);
            FAIL(buf);
        }
    }

    std::printf("\n%d passed, %d failed\n", tests_passed, tests_failed);
    return tests_failed == 0 ? 0 : 1;
}
