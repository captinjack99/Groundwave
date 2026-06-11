/**
 * @file ldpc_protograph_test.cpp
 * @brief A/B validation of the girth-conditioned QC-LDPC protograph
 *        construction vs the legacy LCG-random base matrices (#11 / SOTA-1).
 *
 * Three things are checked, for the Short block size:
 *
 *   1. Encoder validity — both constructions must produce true codewords
 *      (H·c^T = 0) for every rate. The protograph must not break the IRA
 *      staircase encoder.
 *
 *   2. Girth — the number of length-4 cycles in the lifted Tanner graph.
 *      The legacy high-rate matrices (m_base = 3) are riddled with
 *      unavoidable 4-cycles; the protograph must drive this to (near) zero
 *      while never being WORSE than legacy at any rate.
 *
 *   3. Coding gain — a paired AWGN BER sweep (same info + same noise fed to
 *      both codes) at the high rates. The protograph must not lose to legacy
 *      on total info-bit errors; the degenerate legacy code's error floor
 *      should show the protograph winning.
 *
 * Exits non-zero if any hard assertion fails.
 */
#include "ldpc.hpp"

#include <cstdio>
#include <cmath>
#include <random>
#include <vector>

using namespace gw;

namespace {

int g_fails = 0;
void check(bool ok, const char* msg) {
    std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", msg);
    if (!ok) ++g_fails;
}

inline bool getBit(const uint8_t* d, size_t i) {
    return (d[i >> 3] >> (7 - (i & 7))) & 1;
}

const char* rateName(FECRate r) {
    switch (r) {
        case FECRate::Rate_1_4:  return "1/4";
        case FECRate::Rate_1_3:  return "1/3";
        case FECRate::Rate_2_5:  return "2/5";
        case FECRate::Rate_1_2:  return "1/2";
        case FECRate::Rate_3_5:  return "3/5";
        case FECRate::Rate_2_3:  return "2/3";
        case FECRate::Rate_3_4:  return "3/4";
        case FECRate::Rate_4_5:  return "4/5";
        case FECRate::Rate_5_6:  return "5/6";
        case FECRate::Rate_8_9:  return "8/9";
        case FECRate::Rate_9_10: return "9/10";
        default: return "??";
    }
}

const FECRate kAllRates[] = {
    FECRate::Rate_1_4, FECRate::Rate_1_3, FECRate::Rate_2_5,
    FECRate::Rate_1_2, FECRate::Rate_3_5, FECRate::Rate_2_3,
    FECRate::Rate_3_4, FECRate::Rate_4_5, FECRate::Rate_5_6,
    FECRate::Rate_8_9, FECRate::Rate_9_10
};

// The rates whose legacy base is too short (m_base <= 5) and which the
// protograph lifts to a taller base — the ones that must measurably improve.
bool isLiftedRate(FECRate r) {
    return r == FECRate::Rate_9_10 || r == FECRate::Rate_8_9 ||
           r == FECRate::Rate_5_6  || r == FECRate::Rate_4_5;
}

int unsatisfiedChecks(FECRate r, LDPCConstruction cons, uint32_t seed) {
    LDPCEncoder enc(r, LDPCBlockSize::Short, cons);
    const auto& H = enc.matrix();
    std::mt19937 rng(seed ^ static_cast<uint32_t>(r));
    std::vector<uint8_t> info(enc.infoBytes(), 0);
    std::vector<uint8_t> cw(enc.codewordBytes(), 0);
    for (auto& b : info) b = static_cast<uint8_t>(rng() & 0xFF);
    enc.encode(info.data(), cw.data());
    int unsat = 0;
    for (size_t c = 0; c < H.m; ++c) {
        uint8_t s = 0;
        for (uint32_t v : H.rows[c]) s ^= getBit(cw.data(), v) ? 1 : 0;
        if (s) ++unsat;
    }
    return unsat;
}

// Paired AWGN comparison: identical info bits AND identical channel noise
// fed to both the legacy and protograph code at the same Es/N0. Returns
// total info-bit errors for each over `trials` codewords.
struct Paired { long leg_err = 0; long pro_err = 0; long info_bits = 0; };

Paired runPaired(FECRate rate, double esno_db, int trials, uint32_t seed) {
    LDPCEncoder encL(rate, LDPCBlockSize::Short, LDPCConstruction::Legacy);
    LDPCEncoder encP(rate, LDPCBlockSize::Short, LDPCConstruction::Protograph);
    LDPCDecoder decL(rate, LDPCBlockSize::Short, 50, LDPCConstruction::Legacy);
    LDPCDecoder decP(rate, LDPCBlockSize::Short, 50, LDPCConstruction::Protograph);

    const size_t n = encL.codewordBits();   // identical for both
    const size_t k = encL.infoBits();
    const size_t kb = encL.infoBytes();
    const size_t nbL = encL.codewordBytes();
    const size_t nbP = encP.codewordBytes();

    const double esno = std::pow(10.0, esno_db / 10.0);
    const double sigma = std::sqrt(1.0 / (2.0 * esno));  // BPSK, Es=1
    const float inv_sig2 = static_cast<float>(1.0 / (sigma * sigma));

    std::mt19937 rng(seed);
    std::normal_distribution<float> gauss(0.f, static_cast<float>(sigma));
    std::uniform_int_distribution<int> byte(0, 255);

    Paired r;
    std::vector<uint8_t> info(kb), cwL(nbL), cwP(nbP), outL(kb), outP(kb);
    std::vector<float> noise(n), llrL(n), llrP(n);

    for (int t = 0; t < trials; ++t) {
        for (auto& b : info) b = static_cast<uint8_t>(byte(rng));
        for (size_t i = 0; i < n; ++i) noise[i] = gauss(rng);

        encL.encode(info.data(), cwL.data());
        encP.encode(info.data(), cwP.data());
        for (size_t i = 0; i < n; ++i) {
            float yL = (getBit(cwL.data(), i) ? -1.f : 1.f) + noise[i];
            float yP = (getBit(cwP.data(), i) ? -1.f : 1.f) + noise[i];
            llrL[i] = 2.f * yL * inv_sig2;
            llrP[i] = 2.f * yP * inv_sig2;
        }
        decL.decode(llrL.data(), outL.data());
        decP.decode(llrP.data(), outP.data());

        for (size_t i = 0; i < k; ++i) {
            if (getBit(info.data(), i) != getBit(outL.data(), i)) ++r.leg_err;
            if (getBit(info.data(), i) != getBit(outP.data(), i)) ++r.pro_err;
        }
        r.info_bits += static_cast<long>(k);
    }
    return r;
}

} // anonymous

int main() {
    // -------------------------------------------------------------------
    // 1. Encoder validity (both constructions, every rate)
    // -------------------------------------------------------------------
    std::printf("=== 1. Encoder validity (H.c^T = 0) ===\n");
    for (auto r : kAllRates) {
        int uL = unsatisfiedChecks(r, LDPCConstruction::Legacy,     0x12345678u);
        int uP = unsatisfiedChecks(r, LDPCConstruction::Protograph, 0x12345678u);
        char msg[96];
        std::snprintf(msg, sizeof(msg),
                      "rate %-5s  legacy unsat=%d  protograph unsat=%d",
                      rateName(r), uL, uP);
        check(uL == 0 && uP == 0, msg);
    }

    // -------------------------------------------------------------------
    // 2. Girth — 4-cycle count (lower is better; 0 == girth >= 6)
    // -------------------------------------------------------------------
    std::printf("\n=== 2. 4-cycle (girth-4) count, legacy vs protograph ===\n");
    for (auto r : kAllRates) {
        auto HL = buildLDPCMatrix(r, LDPCBlockSize::Short, LDPCConstruction::Legacy);
        auto HP = buildLDPCMatrix(r, LDPCBlockSize::Short, LDPCConstruction::Protograph);
        size_t cL = countLDPCShortCycles(*HL);
        size_t cP = countLDPCShortCycles(*HP);
        std::printf("  rate %-5s  legacy=%-8zu  protograph=%-8zu  (n=%zu k=%zu)\n",
                    rateName(r), cL, cP, HP->n, HP->k);
        char msg[128];
        // Protograph must never be worse than legacy on girth.
        std::snprintf(msg, sizeof(msg),
                      "rate %-5s protograph 4-cycles (%zu) <= legacy (%zu)",
                      rateName(r), cP, cL);
        check(cP <= cL, msg);
        // The lifted high rates must improve dramatically AND reach girth >= 6.
        if (isLiftedRate(r)) {
            char m2[128];
            std::snprintf(m2, sizeof(m2),
                          "rate %-5s protograph reaches girth>=6 (0 four-cycles)",
                          rateName(r));
            check(cP == 0, m2);
        }
    }

    // -------------------------------------------------------------------
    // 3. Coding gain — paired AWGN BER sweep at the high rates
    // -------------------------------------------------------------------
    std::printf("\n=== 3. Paired AWGN BER (same info+noise both codes) ===\n");
    // Measure in the WATERFALL/floor region (BER ~1e-3..1e-5) where coding
    // gain is defined. Below threshold (Es/N0 <= 3.0 dB here, BER > 1e-2)
    // both codes catastrophically fail and the comparison is pure noise —
    // that is not where a girth improvement helps, by construction.
    const double esno_pts[] = { 3.5, 4.0, 4.5, 5.0 };
    const int trials = 200;
    FECRate ber_rates[] = { FECRate::Rate_8_9, FECRate::Rate_9_10 };
    for (auto r : ber_rates) {
        long leg_tot = 0, pro_tot = 0, bits_tot = 0;
        std::printf("  --- rate %s ---\n", rateName(r));
        for (double es : esno_pts) {
            Paired p = runPaired(r, es, trials, 0xC0FFEEu + static_cast<uint32_t>(es * 10));
            double berL = double(p.leg_err) / double(p.info_bits);
            double berP = double(p.pro_err) / double(p.info_bits);
            std::printf("    Es/N0=%.1f dB  legacy BER=%.2e (%ld)  "
                        "protograph BER=%.2e (%ld)\n",
                        es, berL, p.leg_err, berP, p.pro_err);
            leg_tot += p.leg_err; pro_tot += p.pro_err; bits_tot += p.info_bits;
        }
        std::printf("    TOTAL  legacy=%ld  protograph=%ld  info_bits=%ld\n",
                    leg_tot, pro_tot, bits_tot);
        char msg[128];
        std::snprintf(msg, sizeof(msg),
                      "rate %-5s protograph total errors (%ld) <= legacy (%ld)",
                      rateName(r), pro_tot, leg_tot);
        check(pro_tot <= leg_tot, msg);
    }

    std::printf("\n%s (%d failure%s)\n",
                g_fails == 0 ? "ALL PASS" : "FAILURES",
                g_fails, g_fails == 1 ? "" : "s");
    return g_fails == 0 ? 0 : 1;
}
