/**
 * @file pls_test.cpp
 * @brief PLS bootstrap robustness (#26): the Reed-Muller + soft-combine
 *        codec must (a) round-trip every ModCod and (b) decode at SNRs where
 *        the legacy CRC-8 + 2x-repetition scheme fails outright.
 */
#include "pls.hpp"
#include "ofdm.hpp"

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

const Modulation kMods[] = {
    Modulation::BPSK, Modulation::QPSK, Modulation::QAM16, Modulation::QAM64,
    Modulation::QAM256, Modulation::QAM1024, Modulation::QAM4096
};
const FECRate kRates[] = {
    FECRate::Rate_1_4, FECRate::Rate_1_2, FECRate::Rate_2_3,
    FECRate::Rate_3_4, FECRate::Rate_8_9, FECRate::Rate_9_10
};

PLSBlock makePLS(Modulation m, FECRate f, bool vcm, uint8_t slot, uint8_t total) {
    PLSBlock p;
    p.modulation = m; p.fec_rate = f;
    p.vcm_active = vcm; p.vcm_slot = slot; p.vcm_total = total;
    return p;
}

// --- Legacy codec success over an AWGN BPSK channel (64 raw bits, hard) ---
bool legacyTrial(const PLSBlock& tx, double sigma, std::mt19937& rng,
                 std::normal_distribution<float>& g) {
    std::vector<uint8_t> enc;
    encodePLS(tx, enc);                 // 8 bytes = 64 bits, 2 copies
    std::vector<uint8_t> bytes(8, 0);
    for (int i = 0; i < 64; ++i) {
        int bit = (enc[i >> 3] >> (7 - (i & 7))) & 1;
        float y = (bit ? -1.f : 1.f) + g(rng);
        if (y < 0.f) bytes[i >> 3] |= static_cast<uint8_t>(1 << (7 - (i & 7)));
    }
    (void)sigma;
    PLSBlock rx;
    return decodePLS(bytes, rx) && rx.modulation == tx.modulation &&
           rx.fec_rate == tx.fec_rate;
}

// --- New codec success: RM-coded, BPSK, n_copies, soft-combined ---
bool codedTrial(const PLSBlock& tx, int n_copies, std::mt19937& rng,
                std::normal_distribution<float>& g) {
    std::vector<uint8_t> coded;
    encodePLSCoded(tx, coded);          // PLS_CODED_BITS = 112
    std::vector<float> soft(coded.size() * static_cast<size_t>(n_copies));
    for (int c = 0; c < n_copies; ++c)
        for (size_t i = 0; i < coded.size(); ++i)
            soft[c * coded.size() + i] = (coded[i] ? -1.f : 1.f) + g(rng);
    PLSBlock rx;
    return decodePLSSoft(soft, rx) && rx.modulation == tx.modulation &&
           rx.fec_rate == tx.fec_rate;
}

// Full OFDM TX→RX of the coded PLS, exactly as the engine does it: coded
// bits → genuine BPSK symbols → modulateSymbols → preamble + demodulate →
// real-part soft values → decodePLSSoft. `data_mod` is the DATA tier's
// modulation; the PLS must decode regardless of it (the genuine-BPSK fix).
bool ofdmPlsRoundTrip(Modulation data_mod, FECRate data_fec, uint16_t fft,
                      float snr_db, uint32_t seed) {
    OFDMParams p;
    p.fft_size = fft; p.modulation = data_mod; p.sample_rate = 48000;
    p.cyclic_prefix = CyclicPrefix::CP_1_8;
    OFDMModulator tx(p);
    OFDMDemodulator rx(p);

    PLSBlock pls = makePLS(data_mod, data_fec, false, 0, 1);
    std::vector<uint8_t> coded;
    encodePLSCoded(pls, coded);
    size_t dc = computeAllocation(p).dataCount();
    if (dc < coded.size()) return false;             // FFT too small for PLS
    size_t n_copies = dc / coded.size();
    ComplexBuf syms;
    for (size_t c = 0; c < n_copies; ++c)
        for (uint8_t b : coded) syms.emplace_back(b ? -1.f : 1.f, 0.f);

    ComplexBuf pls_bb;
    tx.modulateSymbols(syms.data(), syms.size(), pls_bb);
    ComplexBuf stream = tx.generatePreamble();
    size_t pre_len = stream.size();
    stream.insert(stream.end(), pls_bb.begin(), pls_bb.end());

    if (snr_db >= 0.f) {
        float sig = 0.f; for (auto& s : stream) sig += std::norm(s);
        sig /= static_cast<float>(stream.size());
        float nv = sig / std::pow(10.f, snr_db / 10.f);
        std::mt19937 rng(seed);
        std::normal_distribution<float> g(0.f, std::sqrt(nv * 0.5f));
        for (auto& s : stream) s += ComplexSample(g(rng), g(rng));
    }

    size_t sym_len = p.symbolLength();
    size_t short_total = 10 * (fft / 4);
    if (stream.size() < short_total + 2 * sym_len) return false;
    ComplexBuf longs(stream.begin() + static_cast<ptrdiff_t>(short_total),
                     stream.begin() + static_cast<ptrdiff_t>(short_total + 2 * sym_len));
    rx.processPreamble(longs);

    ComplexBuf pls_sym(stream.begin() + static_cast<ptrdiff_t>(pre_len),
                       stream.begin() + static_cast<ptrdiff_t>(pre_len + sym_len));
    ComplexBuf dsyms;
    if (!rx.demodulate(pls_sym, dsyms)) return false;
    std::vector<float> soft;
    soft.reserve(dsyms.size());
    for (auto& s : dsyms) soft.push_back(s.real());

    PLSBlock rxpls;
    return decodePLSSoft(soft, rxpls) && rxpls.modulation == data_mod &&
           rxpls.fec_rate == data_fec;
}

double successRate(int which, int n_copies, double esno_db, int trials) {
    double esno = std::pow(10.0, esno_db / 10.0);
    double sigma = std::sqrt(1.0 / (2.0 * esno));   // BPSK, Es=1
    std::mt19937 rng(0x5A17u ^ static_cast<uint32_t>(esno_db * 7 + which * 131));
    std::normal_distribution<float> g(0.f, static_cast<float>(sigma));
    std::uniform_int_distribution<int> mi(0, 6), ri(0, 5);
    int ok = 0;
    for (int t = 0; t < trials; ++t) {
        PLSBlock tx = makePLS(kMods[mi(rng)], kRates[ri(rng)], false, 0, 1);
        bool s = (which == 0) ? legacyTrial(tx, sigma, rng, g)
                              : codedTrial(tx, n_copies, rng, g);
        if (s) ++ok;
    }
    return double(ok) / trials;
}

} // anonymous

int main() {
    // -------------------------------------------------------------------
    // 1. Clean round-trip — every ModCod (and a VCM case)
    // -------------------------------------------------------------------
    std::printf("=== 1. Coded PLS round-trip (no noise) ===\n");
    int rt_fail = 0;
    for (auto m : kMods) {
        for (auto f : kRates) {
            PLSBlock tx = makePLS(m, f, true, 2, 4);
            std::vector<uint8_t> coded;
            encodePLSCoded(tx, coded);
            if (coded.size() != PLS_CODED_BITS) { ++rt_fail; continue; }
            std::vector<float> soft(coded.size());
            for (size_t i = 0; i < coded.size(); ++i)
                soft[i] = coded[i] ? -1.f : 1.f;
            PLSBlock rx;
            if (!decodePLSSoft(soft, rx) || rx.modulation != m ||
                rx.fec_rate != f || rx.vcm_slot != 2 || rx.vcm_total != 4)
                ++rt_fail;
        }
    }
    char rt[80];
    std::snprintf(rt, sizeof(rt), "all ModCods round-trip clean (%d failures)", rt_fail);
    check(rt_fail == 0, rt);

    // -------------------------------------------------------------------
    // 2. AWGN: coded codec decodes where legacy fails
    // -------------------------------------------------------------------
    std::printf("\n=== 2. AWGN success rate: legacy vs RM-coded ===\n");
    const int N = 600;
    double esno_pts[] = { 0.0, 2.0, 3.0, 4.0, 6.0 };
    std::printf("   Es/N0    legacy   RM(1cpy)  RM(2cpy)\n");
    for (double es : esno_pts) {
        double L  = successRate(0, 1, es, N);
        double C1 = successRate(1, 1, es, N);
        double C2 = successRate(1, 2, es, N);
        std::printf("   %4.1f dB   %5.1f%%   %5.1f%%    %5.1f%%\n",
                    es, 100*L, 100*C1, 100*C2);
    }

    // Hard assertions at a representative cliff point.
    double L3  = successRate(0, 1, 3.0, N);
    double C1_3 = successRate(1, 1, 3.0, N);
    double C2_3 = successRate(1, 2, 3.0, N);
    char m1[140];
    std::snprintf(m1, sizeof(m1),
        "@3 dB: RM 1-copy (%.0f%%) far exceeds legacy (%.0f%%)", 100*C1_3, 100*L3);
    check(C1_3 > L3 + 0.25 && C1_3 > 0.80, m1);

    char m2[140];
    std::snprintf(m2, sizeof(m2),
        "@3 dB: soft-combining 2 copies (%.0f%%) >= 1 copy (%.0f%%)",
        100*C2_3, 100*C1_3);
    check(C2_3 >= C1_3 - 0.02, m2);

    double C1_0 = successRate(1, 1, 0.0, N), L0 = successRate(0, 1, 0.0, N);
    double C2_0 = successRate(1, 2, 0.0, N);
    char m3[160];
    std::snprintf(m3, sizeof(m3),
        "@0 dB (deep cliff): legacy collapses (%.0f%%); combining helps (1cpy %.0f%% -> 2cpy %.0f%%)",
        100*L0, 100*C1_0, 100*C2_0);
    check(C2_0 > C1_0 && C2_0 > L0, m3);

    // -------------------------------------------------------------------
    // 3. Full OFDM TX→RX (the engine path): genuine BPSK PLS decodes at any
    //    data modcod — including QAM256, where the old real-sign demap of a
    //    data-modulated PLS produced garbage.
    // -------------------------------------------------------------------
    std::printf("\n=== 3. OFDM PLS round-trip (genuine BPSK, any data modcod) ===\n");
    struct OC { Modulation m; FECRate f; const char* name; };
    OC ocs[] = {
        { Modulation::BPSK,   FECRate::Rate_1_2,  "BPSK"   },
        { Modulation::QPSK,   FECRate::Rate_2_3,  "QPSK"   },
        { Modulation::QAM256, FECRate::Rate_9_10, "QAM256" },
    };
    int clean_fail = 0;
    for (auto& o : ocs) {
        bool ok = ofdmPlsRoundTrip(o.m, o.f, 256, /*clean*/ -1.f, 0);
        std::printf("   data=%-6s FFT256 clean: PLS decode %s\n",
                    o.name, ok ? "OK" : "FAIL");
        if (!ok) ++clean_fail;
    }
    check(clean_fail == 0,
          "coded PLS round-trips through OFDM at BPSK/QPSK/QAM256 data modcods");

    // Noisy channel: PLS still decodes at a low SNR (QAM256 data tier).
    int noisy_ok = 0;
    for (int s = 0; s < 10; ++s)
        if (ofdmPlsRoundTrip(Modulation::QAM256, FECRate::Rate_9_10, 256, 3.0f,
                             0x1000u + static_cast<uint32_t>(s)))
            ++noisy_ok;
    char mo[120];
    std::snprintf(mo, sizeof(mo),
                  "coded PLS survives a noisy channel (3 dB): %d/10 decodes", noisy_ok);
    check(noisy_ok >= 9, mo);

    std::printf("\n%s (%d failure%s)\n",
                g_fails == 0 ? "ALL PASS" : "FAILURES",
                g_fails, g_fails == 1 ? "" : "s");
    return g_fails == 0 ? 0 : 1;
}
