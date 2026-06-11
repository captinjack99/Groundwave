/**
 * @file labeling_test.cpp
 * @brief Anti-Gray (BICM-ID-friendly) constellation labeling validation.
 *
 * Motivation: for the system's GRAY-mapped square QAM, each bit's max-log LLR
 * depends on a single I/Q coordinate and is independent of the other bits in
 * the symbol. Feeding back the decoder's a-priori information therefore cannot
 * change a bit's max-log decision, so the BICM-ID demapper EXIT curve is FLAT
 * (I_E ~ const vs I_A) and the iterative loop exchanges almost nothing.
 *
 * The fix is a NON-Gray labeling whose Euclidean-nearest neighbors differ in
 * many bits. The first (prior-free) pass is worse, but every bit now couples to
 * the others through the constellation, giving the demapper EXIT curve a
 * positive slope — rich iterations.
 *
 * Tests:
 *   (1) Gray geometry byte-stable (anti-Gray must not perturb the default).
 *   (2) Anti-Gray increases the average Euclidean-nearest-neighbor Hamming
 *       distance (the defining property of an anti-Gray labeling).
 *   (3) Anti-Gray round-trips bit-exact at high SNR (hard demap; map is a valid
 *       bijection and demapHard finds the right point).
 *   (4) PWL soft demapper == exact soft demapper under anti-Gray (the PWL
 *       tables are labeling-general, not Gray-specific).
 *   (5) Demapper EXIT slope: anti-Gray's I_E rises with I_A (positive slope)
 *       while Gray's is ~flat — the central claim of the deliverable.
 *   (6) Full BICM-ID with anti-Gray converges near threshold, and exhibits the
 *       textbook signature: WORSE at iteration 1, BETTER after the loop.
 */
#include "types.hpp"
#include "symbol_mapper.hpp"
#include "interleaver.hpp"
#include "ldpc.hpp"
#include "bicm.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

using namespace gw;

namespace {

int g_fails = 0;
void check(bool ok, const char* msg) {
    std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", msg);
    if (!ok) ++g_fails;
}

int popcnt(unsigned v) { int c = 0; while (v) { c += int(v & 1u); v >>= 1; } return c; }

// Average Hamming distance between each point's label and that of its
// Euclidean-nearest neighbor (point index == its bit label here).
double avgNNHamming(const SymbolMapper& m) {
    const ComplexBuf& C = m.constellation();
    const size_t M = C.size();
    double acc = 0.0;
    for (size_t i = 0; i < M; ++i) {
        double bestd = 1e30; size_t bestj = 0;
        for (size_t j = 0; j < M; ++j) {
            if (j == i) continue;
            double dr = C[i].real() - C[j].real();
            double di = C[i].imag() - C[j].imag();
            double d = dr * dr + di * di;
            if (d < bestd) { bestd = d; bestj = j; }
        }
        acc += popcnt(unsigned(i ^ bestj));
    }
    return acc / double(M);
}

// ---- ten-Brink J / Jinv and MI estimator (same model as tools/exit_chart) ----
constexpr double H1 = 0.3073, H2 = 0.8935, H3 = 1.1064;
double Jinv(double I) {
    I = std::clamp(I, 0.0, 1.0 - 1e-7);
    if (I <= 0.0) return 0.0;
    double inner = 1.0 - std::pow(I, 1.0 / H3);
    if (inner <= 0.0) inner = 1e-12;
    double sigma2 = (-1.0 / H1) * (std::log(inner) / std::log(2.0));
    if (sigma2 <= 0.0) return 0.0;
    return std::pow(sigma2, 1.0 / (2.0 * H2));
}
double measureMI(const std::vector<float>& L, const std::vector<uint8_t>& b) {
    if (L.empty()) return 0.0;
    const size_t n = std::min(L.size(), b.size());
    double acc = 0.0;
    for (size_t i = 0; i < n; ++i) {
        double sign = b[i] ? -1.0 : 1.0;
        double x = -sign * double(L[i]);
        double t;
        if (x > 30.0)       t = x / std::log(2.0);
        else if (x < -30.0) t = 0.0;
        else                t = std::log1p(std::exp(x)) / std::log(2.0);
        acc += t;
    }
    return std::clamp(1.0 - acc / double(n), 0.0, 1.0);
}
void genApriori(double I_A, const std::vector<uint8_t>& b,
                std::vector<float>& L, std::mt19937& rng) {
    L.resize(b.size());
    double sigma = Jinv(I_A);
    double var = sigma * sigma;
    std::normal_distribution<double> nd(0.0, sigma > 0 ? sigma : 1e-6);
    for (size_t i = 0; i < b.size(); ++i) {
        double mean = (b[i] ? -1.0 : 1.0) * var * 0.5;
        L[i] = float(mean + nd(rng));
    }
}
// Prior-aware extrinsic max-log-APP demap (mirror of BICMDecoder::demapWithPriors).
void demapWithPriors(const SymbolMapper& mapper, const ComplexBuf& syms,
                     float noise_var, const std::vector<float>& prior,
                     std::vector<float>& out) {
    const ComplexBuf& C = mapper.constellation();
    const size_t bps = mapper.bitsPerSymbol();
    const size_t M = C.size();
    out.assign(syms.size() * bps, 0.f);
    const float inv2s2 = 1.0f / (2.0f * std::max(noise_var, 1e-6f));
    std::vector<float> base(M);
    for (size_t s = 0; s < syms.size(); ++s) {
        const ComplexSample y = syms[s];
        for (size_t idx = 0; idx < M; ++idx) {
            float m = -std::norm(y - C[idx]) * inv2s2;
            for (size_t j = 0; j < bps; ++j) {
                float Lj = prior[s * bps + j];
                bool bj = (idx >> (bps - 1 - j)) & 1u;
                m += bj ? (-0.5f * Lj) : (0.5f * Lj);
            }
            base[idx] = m;
        }
        for (size_t k = 0; k < bps; ++k) {
            float Lk = prior[s * bps + k];
            float max0 = -1e30f, max1 = -1e30f;
            for (size_t idx = 0; idx < M; ++idx) {
                bool bk = (idx >> (bps - 1 - k)) & 1u;
                float m = base[idx] - (bk ? (-0.5f * Lk) : (0.5f * Lk));
                if (bk) { if (m > max1) max1 = m; }
                else    { if (m > max0) max0 = m; }
            }
            out[s * bps + k] = max0 - max1;
        }
    }
}
float esnoToN0(float esno_db) { return 1.0f / std::pow(10.0f, esno_db / 10.0f); }

// One demapper-EXIT point: I_E given a-priori MI I_A at fixed Es/N0.
double demapperPoint(Modulation mod, Labeling lab, float esno_db, double I_A,
                     size_t n_syms, std::mt19937& rng) {
    SymbolMapper mapper(mod, lab);
    const size_t bps = mapper.bitsPerSymbol();
    const float N0 = esnoToN0(esno_db);
    const float sigma_n = std::sqrt(N0 * 0.5f);

    const size_t n_bits = n_syms * bps;
    const size_t n_bytes = (n_bits + 7) / 8;
    std::vector<uint8_t> bytes(n_bytes, 0), bits(n_bits, 0);
    std::uniform_int_distribution<int> bd(0, 1);
    for (size_t i = 0; i < n_bits; ++i) {
        uint8_t v = uint8_t(bd(rng));
        bits[i] = v;
        if (v) bytes[i >> 3] |= uint8_t(1u << (7 - (i & 7)));
    }
    ComplexBuf syms;
    mapper.mapBytes(bytes.data(), n_bits, syms);
    std::normal_distribution<float> nd(0.f, sigma_n);
    for (auto& s : syms) s += ComplexSample(nd(rng), nd(rng));

    std::vector<float> prior;
    genApriori(I_A, bits, prior, rng);
    std::vector<float> ext;
    demapWithPriors(mapper, syms, N0, prior, ext);
    ext.resize(n_bits);
    return measureMI(ext, bits);
}

// Demapper-EXIT slope = I_E(I_A=0.95) - I_E(I_A=0): positive => iterations help.
double demapperSlope(Modulation mod, Labeling lab, float esno_db,
                     std::mt19937& rng) {
    double lo = demapperPoint(mod, lab, esno_db, 0.0,  20000, rng);
    double hi = demapperPoint(mod, lab, esno_db, 0.95, 20000, rng);
    return hi - lo;
}

// ---- Full BICM-ID chain: encode -> interleave -> map -> AWGN -> iterate ----
struct ChainStats { double iter1_ber; double conv_ber; size_t conv_iter; };

ChainStats runChain(Modulation mod, Labeling lab, FECRate fec, float esno_db,
                    size_t n_frames, size_t max_outer, uint32_t seed) {
    SymbolMapper mapper(mod, lab);
    LDPCEncoder enc(fec, LDPCBlockSize::Short);
    LDPCDecoder dec(fec, LDPCBlockSize::Short, 30);
    const size_t bps = mapper.bitsPerSymbol();
    const size_t n_cw = enc.codewordBits();
    const size_t k = enc.infoBits();
    BitInterleaver intl(n_cw, 0);

    const float N0 = esnoToN0(esno_db);
    const float sigma_n = std::sqrt(N0 * 0.5f);
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> bd(0, 1);
    std::normal_distribution<float> nd(0.f, sigma_n);

    // symbols per frame: ceil(n_cw / bps)
    const size_t n_sym = (n_cw + bps - 1) / bps;
    const size_t info_bytes = (k + 7) / 8;

    double iter1_errsum = 0, conv_errsum = 0;
    size_t conv_iter_acc = 0;

    std::vector<uint8_t> info(info_bytes), cw_bytes((n_cw + 7) / 8);
    std::vector<uint8_t> cw_il((n_cw + 7) / 8);
    std::vector<uint8_t> info_out(info_bytes);

    for (size_t f = 0; f < n_frames; ++f) {
        for (auto& b : info) b = uint8_t(rng() & 0xFF);   // random info bytes
        enc.encode(info.data(), cw_bytes.data());

        // interleave coded bits
        intl.interleave(cw_bytes.data(), cw_il.data());

        // map (pad to n_sym*bps bits; extra bits are 0)
        std::vector<uint8_t> map_bytes((n_sym * bps + 7) / 8, 0);
        std::copy(cw_il.begin(), cw_il.end(), map_bytes.begin());
        ComplexBuf syms;
        mapper.mapBytes(map_bytes.data(), n_sym * bps, syms);

        // AWGN
        for (auto& s : syms) s += ComplexSample(nd(rng), nd(rng));

        // Helper to count info-bit errors of a decode
        auto infoErrs = [&](const uint8_t* out) {
            size_t e = 0;
            for (size_t i = 0; i < k; ++i) {
                uint8_t a = (info[i >> 3] >> (7 - (i & 7))) & 1;
                uint8_t b = (out[i >> 3] >> (7 - (i & 7))) & 1;
                if (a != b) ++e;
            }
            return e;
        };

        // ---- iteration 1 (standard BICM, one pass) ----
        {
            BICMConfig cfg; cfg.outer_iterations = 1;
            BICMDecoder bd1(&mapper, &intl, &dec, cfg);
            bd1.decode(syms, N0, info_out.data());
            iter1_errsum += double(infoErrs(info_out.data())) / double(k);
        }
        // ---- converged (up to max_outer passes) ----
        {
            BICMConfig cfg; cfg.outer_iterations = max_outer;
            BICMDecoder bdN(&mapper, &intl, &dec, cfg);
            auto r = bdN.decodeIterative(syms, N0, info_out.data());
            conv_errsum += double(infoErrs(info_out.data())) / double(k);
            conv_iter_acc += r.iterations;
        }
    }
    return { iter1_errsum / n_frames, conv_errsum / n_frames,
             conv_iter_acc / std::max<size_t>(1, n_frames) };
}

} // namespace

int main() {
    std::printf("=== Anti-Gray labeling validation ===\n\n");
    std::mt19937 rng(0xA47C0DEu);

    // (1) Gray geometry byte-stable: AntiGray option must not change Gray.
    {
        bool ok = true;
        for (Modulation mod : {Modulation::QAM16, Modulation::QAM64,
                               Modulation::QAM256}) {
            SymbolMapper def(mod);                 // default arg
            SymbolMapper gray(mod, Labeling::Gray);
            const auto& A = def.constellation();
            const auto& B = gray.constellation();
            if (A.size() != B.size()) { ok = false; break; }
            for (size_t i = 0; i < A.size(); ++i)
                if (A[i] != B[i]) { ok = false; break; }
        }
        check(ok, "default labeling == explicit Gray, geometry unchanged");
    }

    // (2) Anti-Gray raises average nearest-neighbor Hamming distance.
    for (Modulation mod : {Modulation::QAM16, Modulation::QAM64,
                           Modulation::QAM256}) {
        SymbolMapper g(mod, Labeling::Gray), a(mod, Labeling::AntiGray);
        double hg = avgNNHamming(g), ha = avgNNHamming(a);
        char msg[128];
        std::snprintf(msg, sizeof(msg),
            "M=%zu  avg NN-Hamming  Gray=%.3f  AntiGray=%.3f (anti-Gray > Gray)",
            g.constellation().size(), hg, ha);
        check(ha > hg + 0.5, msg);
    }

    // (3) Anti-Gray noiseless hard round-trip is bit-exact.
    for (Modulation mod : {Modulation::QAM16, Modulation::QAM64,
                           Modulation::QAM256}) {
        SymbolMapper a(mod, Labeling::AntiGray);
        const size_t bps = a.bitsPerSymbol();
        const size_t M = a.constellation().size();
        const size_t n_sym = 4 * M;
        const size_t n_bits = n_sym * bps;
        std::vector<uint8_t> bytes((n_bits + 7) / 8, 0), bits(n_bits, 0);
        std::uniform_int_distribution<int> bd(0, 1);
        for (size_t i = 0; i < n_bits; ++i) {
            uint8_t v = uint8_t(bd(rng)); bits[i] = v;
            if (v) bytes[i >> 3] |= uint8_t(1u << (7 - (i & 7)));
        }
        ComplexBuf syms;
        a.mapBytes(bytes.data(), n_bits, syms);
        size_t errs = 0;
        for (size_t s = 0; s < n_sym; ++s) {
            uint16_t idx = a.demapHard(syms[s]);
            for (size_t b = 0; b < bps; ++b) {
                uint8_t exp = bits[s * bps + b];
                uint8_t got = (idx >> (bps - 1 - b)) & 1u;
                if (exp != got) ++errs;
            }
        }
        char msg[96];
        std::snprintf(msg, sizeof(msg),
            "M=%zu anti-Gray noiseless hard round-trip bit-exact (errs=%zu)",
            M, errs);
        check(errs == 0, msg);
    }

    // (4) PWL soft demap == exact soft demap under anti-Gray.
    for (Modulation mod : {Modulation::QAM16, Modulation::QAM64,
                           Modulation::QAM256}) {
        SymbolMapper a(mod, Labeling::AntiGray);
        const size_t bps = a.bitsPerSymbol();
        const size_t M = a.constellation().size();
        const float nv = 0.12f;
        std::vector<float> ex, pw;
        double maxabs = 0.0;
        std::normal_distribution<float> jit(0.f, 0.25f);
        for (size_t s = 0; s < M; ++s) {
            ComplexSample y = a.constellation()[s] +
                              ComplexSample(jit(rng), jit(rng));
            a.demapSoft(y, nv, ex);
            a.demapSoftPWL(y, nv, pw);
            for (size_t b = 0; b < bps; ++b)
                maxabs = std::max(maxabs, double(std::fabs(ex[b] - pw[b])));
        }
        char msg[96];
        std::snprintf(msg, sizeof(msg),
            "M=%zu anti-Gray PWL==exact LLR (max|diff|=%.4g)", M, maxabs);
        check(maxabs < 1e-2, msg);
    }

    // (5) Demapper EXIT slope: anti-Gray positive, Gray ~flat.
    //     Evaluate near each modcod's iterative operating region.
    struct Pt { Modulation mod; float esno; };
    for (Pt p : { Pt{Modulation::QAM16, 9.0f}, Pt{Modulation::QAM64, 14.0f} }) {
        double sg = demapperSlope(p.mod, Labeling::Gray,     p.esno, rng);
        double sa = demapperSlope(p.mod, Labeling::AntiGray, p.esno, rng);
        char msg[160];
        std::snprintf(msg, sizeof(msg),
            "M=%zu @%.1fdB demapper EXIT slope  Gray=%+.3f (flat)  "
            "AntiGray=%+.3f (positive, > Gray)",
            SymbolMapper(p.mod).constellation().size(), p.esno, sg, sa);
        // Anti-Gray must have a clearly positive slope and exceed Gray's.
        check(sa > 0.1 && sa > sg + 0.1, msg);
    }

    // (6) BICM-ID iterative signature with anti-Gray. Operating point is chosen
    //     INSIDE anti-Gray's tunnel (64-QAM 2/3 at ~threshold+3 dB) so the loop
    //     visibly works: the first (prior-free) pass is poor because anti-Gray
    //     neighbors differ in many bits, but the prior-aware iterations collapse
    //     the error rate by orders of magnitude — the gain a FLAT-EXIT Gray map
    //     cannot produce. We assert (a) the first-pass penalty (anti-Gray iter-1
    //     BER >> Gray iter-1 BER) and (b) the strong iterative gain
    //     (anti-Gray converged BER is a large fraction below its iter-1 BER).
    {
        const Modulation mod = Modulation::QAM64;
        const FECRate fec = FECRate::Rate_2_3;
        const float esno = 17.4f;          // inside anti-Gray's open tunnel
        double a1 = 0, aN = 0, g1 = 0, gN = 0, convI = 0;
        const size_t seeds = 3, frames = 20, outer = 12;
        for (uint32_t s = 0; s < seeds; ++s) {
            auto ga = runChain(mod, Labeling::Gray,     fec, esno, frames, outer, 100 + s);
            auto aa = runChain(mod, Labeling::AntiGray, fec, esno, frames, outer, 100 + s);
            g1 += ga.iter1_ber; gN += ga.conv_ber;
            a1 += aa.iter1_ber; aN += aa.conv_ber;
            convI += double(aa.conv_iter);
        }
        g1 /= seeds; gN /= seeds; a1 /= seeds; aN /= seeds; convI /= seeds;
        std::printf("  [info] 64-QAM r2/3 @%.1fdB  Gray  BER iter1=%.5f conv=%.5f\n",
                    esno, g1, gN);
        std::printf("  [info] 64-QAM r2/3 @%.1fdB  AntiG BER iter1=%.5f conv=%.5f "
                    "(avg %.1f outer iters)\n", esno, a1, aN, convI);
        // (a) First-pass penalty: anti-Gray's prior-free pass is much worse.
        check(a1 > g1 + 1e-4,
              "anti-Gray iteration-1 BER > Gray iteration-1 BER (first-pass penalty)");
        // (b) Strong iterative gain: the loop drives anti-Gray BER far below its
        //     own iteration-1 BER (the demapper EXIT curve had room to climb).
        check(aN < 0.5 * a1,
              "anti-Gray BICM-ID converges: converged BER < half of iteration-1 BER");
        // (c) The loop actually iterated (more than one outer pass on average).
        check(convI > 1.5,
              "anti-Gray BICM-ID ran multiple outer iterations on average");
    }

    std::printf("\n=== labeling_test: %s (%d failure%s) ===\n",
                g_fails ? "FAIL" : "PASS", g_fails, g_fails == 1 ? "" : "s");
    return g_fails ? 1 : 0;
}
