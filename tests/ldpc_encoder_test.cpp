/**
 * @file ldpc_encoder_test.cpp
 * @brief Verify the LDPC encoder produces valid codewords for every rate.
 *
 * The most direct test of encoder correctness:
 *   1. Encode random info bits.
 *   2. Compute the syndrome H·c^T over GF(2).
 *   3. Assert all checks evaluate to zero.
 *
 * If this fails, the encoder is producing bit-strings that aren't members
 * of the code — explaining why even a clean-channel decode produces 1-3
 * bit errors (the decoder finds the nearest TRUE codeword, which differs
 * from what we sent by exactly the encoder's mistakes).
 */
#include "ldpc.hpp"

#include <cstdio>
#include <random>
#include <vector>

using namespace dsca;

namespace {
inline bool getBit(const uint8_t* d, size_t i) {
    return (d[i >> 3] >> (7 - (i & 7))) & 1;
}

struct RateResult {
    FECRate rate;
    int     unsatisfied_checks;
    size_t  m;
    bool    ok() const { return unsatisfied_checks == 0; }
};

RateResult verifyRate(FECRate r) {
    RateResult res{r, -1, 0};
    LDPCEncoder enc(r, LDPCBlockSize::Short);
    const auto& H = enc.matrix();
    res.m = H.m;

    std::mt19937 rng(0x12345678 ^ static_cast<uint32_t>(r));
    std::vector<uint8_t> info((H.k + 7) / 8, 0);
    std::vector<uint8_t> cw  ((H.n + 7) / 8, 0);
    for (auto& b : info) b = static_cast<uint8_t>(rng() & 0xFF);
    if (!enc.encode(info.data(), cw.data())) return res;

    int unsat = 0;
    for (size_t check = 0; check < H.m; ++check) {
        uint8_t s = 0;
        for (uint32_t v : H.rows[check]) s ^= getBit(cw.data(), v) ? 1 : 0;
        if (s) ++unsat;
    }
    res.unsatisfied_checks = unsat;
    return res;
}
} // anonymous

// Pure-FEC round-trip: encode → hard-decision LLRs → decode → compare.
// Bypasses the OFDM mod/demod entirely so any error is decoder-side.
int decoderPureRoundTrip(FECRate r) {
    LDPCEncoder enc(r, LDPCBlockSize::Short);
    LDPCDecoder dec(r, LDPCBlockSize::Short, 50);

    std::mt19937 rng(0xABC ^ static_cast<uint32_t>(r));
    std::vector<uint8_t> info((enc.infoBits() + 7) / 8, 0);
    for (auto& b : info) b = static_cast<uint8_t>(rng() & 0xFF);

    std::vector<uint8_t> cw((enc.codewordBits() + 7) / 8, 0);
    enc.encode(info.data(), cw.data());

    // Build large-magnitude hard-decision LLRs from the codeword.
    // Convention: positive LLR ⇒ bit 0, negative ⇒ bit 1
    std::vector<float> llrs(enc.codewordBits());
    for (size_t i = 0; i < enc.codewordBits(); ++i) {
        bool b = getBit(cw.data(), i);
        llrs[i] = b ? -10.f : 10.f;
    }
    std::vector<uint8_t> out((enc.infoBits() + 7) / 8, 0);
    auto result = dec.decode(llrs.data(), out.data());
    (void)result;

    int errs = 0;
    size_t k_bytes = (enc.infoBits() + 7) / 8;
    // Compare bit-by-bit only over the actual info-bit count (k may not be
    // a multiple of 8; trailing bits in the last byte are unspecified).
    for (size_t i = 0; i < enc.infoBits(); ++i) {
        bool a = getBit(info.data(), i);
        bool b = getBit(out.data(),  i);
        if (a != b) ++errs;
    }
    (void)k_bytes;
    return errs;
}

int main() {
    std::printf("=== LDPC encoder validity ===\n");
    FECRate rates[] = {
        FECRate::Rate_1_4, FECRate::Rate_1_3, FECRate::Rate_2_5,
        FECRate::Rate_1_2, FECRate::Rate_3_5, FECRate::Rate_2_3,
        FECRate::Rate_3_4, FECRate::Rate_4_5, FECRate::Rate_5_6,
        FECRate::Rate_8_9, FECRate::Rate_9_10
    };
    auto rateName = [](FECRate r) -> const char* {
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
    };
    int fails = 0;
    for (auto r : rates) {
        auto res = verifyRate(r);
        std::printf("  encoder rate %-5s: %d unsatisfied  %s\n",
                    rateName(r), res.unsatisfied_checks,
                    res.ok() ? "OK" : "FAIL");
        if (!res.ok()) ++fails;
    }

    std::printf("\n=== Pure-FEC decoder round-trip (no OFDM) ===\n");
    for (auto r : rates) {
        int errs = decoderPureRoundTrip(r);
        std::printf("  decoder rate %-5s: %d info-bit errors  %s\n",
                    rateName(r), errs, errs == 0 ? "OK" : "FAIL");
        if (errs != 0) ++fails;
    }
    return fails == 0 ? 0 : 1;
}
