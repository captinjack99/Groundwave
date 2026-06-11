/**
 * @file exit_chart.cpp
 * @brief EXIT (EXtrinsic Information Transfer) chart for the DSCA-NG BICM-ID
 *        receiver. Computes the two transfer curves whose relative position
 *        predicts whether iterative demapping+decoding converges:
 *
 *   (1) DEMAPPER curve  I_E_dem(I_A) — at a FIXED channel Es/N0, how much
 *       extrinsic mutual information the prior-aware soft demapper
 *       (BICMDecoder::demapWithPriors, mirrored here) produces about the coded
 *       bits given a-priori information of MI I_A fed back from the decoder.
 *
 *   (2) DECODER curve   I_E_dec(I_A) — how much extrinsic MI ONE LDPC posterior
 *       pass (LDPCDecoder::decodePosterior) produces about the codeword bits
 *       given a-priori information of MI I_A on its inputs.
 *
 * The convergence "tunnel" is open when the demapper curve stays strictly above
 * the decoder curve plotted on SWAPPED axes (I_A_dec = I_E_dem). If the two
 * curves touch before (1,1) the iteration pinches there and BICM-ID stalls.
 *
 * METHOD (standard ten-Brink semi-analytic EXIT analysis):
 *   - J(sigma): MI between a coded bit and a consistent-Gaussian LLR of
 *     variance sigma^2 (mean = ±sigma^2/2). Jinv is its inverse. Both use the
 *     standard two-piece polynomial approximations.
 *   - To GENERATE a-priori LLRs of target MI I_A on KNOWN bits b:
 *         sigma = Jinv(I_A);  L_a ~ N( (1-2b)*sigma^2/2 , sigma^2 ).
 *   - To MEASURE MI from a set of LLRs L on KNOWN bits b (histogram-free,
 *     Monte-Carlo "ergodic" estimator):
 *         I = 1 - mean( log2(1 + exp(-(1-2b)*L)) ).
 *
 * LLR sign convention matches the modem: POSITIVE LLR => bit 0 more likely,
 * so a known bit b contributes (1-2b) (i.e. +1 for b=0, -1 for b=1).
 *
 * This is a MODEL-BASED prediction (Gaussian-LLR a-priori assumption); it
 * predicts convergence behavior, it is not a frame-by-frame BER simulation.
 *
 * Usage:
 *   ./exit_chart                 64-QAM 2/3 at threshold+1 dB (defaults)
 *   ./exit_chart --mod 16 --fec 3_4 --esno 9.5
 *   ./exit_chart --mod 16 --labeling antigray   anti-Gray demapper EXIT curve
 *   ./exit_chart --selftest      J/Jinv round-trip + MI-range + monotonicity
 *                                + anti-Gray-vs-Gray demapper slope
 *                                (fast ctest gate; exit 0 on pass)
 */
#include "types.hpp"
#include "symbol_mapper.hpp"
#include "ldpc.hpp"
#include "snr_calculator.hpp"   // modulationName / fecRateName / computeThreshold

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>
#include <string>
#include <vector>

using namespace dsca;

namespace {

// =========================================================================
// ten-Brink J-function and its inverse (standard polynomial approximation,
// Brink/Kramer/Ashikhmin 2004). J(sigma) = MI of a consistent-Gaussian LLR
// with variance sigma^2 about the bit it describes. Defined on sigma in
// [0, ~), returning MI in [0,1).
// =========================================================================
constexpr double H1 = 0.3073, H2 = 0.8935, H3 = 1.1064;

double Jfunc(double sigma) {
    if (sigma <= 0.0) return 0.0;
    if (sigma > 10.0) return 1.0;
    // J(sigma) = ( 1 - 2^( -H1 * sigma^(2*H2) ) )^H3
    double v = 1.0 - std::pow(2.0, -H1 * std::pow(sigma, 2.0 * H2));
    if (v <= 0.0) return 0.0;
    double j = std::pow(v, H3);
    return std::clamp(j, 0.0, 1.0);
}

// Inverse: sigma such that J(sigma) = I. Closed-form companion to the above.
double Jinv(double I) {
    I = std::clamp(I, 0.0, 1.0 - 1e-7);
    if (I <= 0.0) return 0.0;
    // sigma = ( -1/H1 * log2( 1 - I^(1/H3) ) )^(1/(2*H2))
    double inner = 1.0 - std::pow(I, 1.0 / H3);
    if (inner <= 0.0) inner = 1e-12;
    double sigma2 = (-1.0 / H1) * (std::log(inner) / std::log(2.0));
    if (sigma2 <= 0.0) return 0.0;
    return std::pow(sigma2, 1.0 / (2.0 * H2));
}

// =========================================================================
// MI estimator from a set of LLRs L on KNOWN bits b (Monte-Carlo, eq. above).
// b==0 -> sign +1, b==1 -> sign -1. Clamp the exponent to avoid overflow.
// =========================================================================
double measureMI(const std::vector<float>& L, const std::vector<uint8_t>& b) {
    if (L.empty()) return 0.0;
    const size_t n = std::min(L.size(), b.size());
    double acc = 0.0;
    for (size_t i = 0; i < n; ++i) {
        double sign = b[i] ? -1.0 : 1.0;           // (1-2b)
        double x = -sign * static_cast<double>(L[i]);
        // log2(1 + exp(x)) with overflow guards.
        double t;
        if (x > 30.0)       t = x / std::log(2.0);             // 1+e^x ~ e^x
        else if (x < -30.0) t = 0.0;                            // 1+e^x ~ 1
        else                t = std::log1p(std::exp(x)) / std::log(2.0);
        acc += t;
    }
    double I = 1.0 - acc / static_cast<double>(n);
    return std::clamp(I, 0.0, 1.0);
}

// Generate a-priori LLRs of target MI I_A on KNOWN bits b:
//   sigma = Jinv(I_A);  L ~ N( (1-2b)*sigma^2/2, sigma^2 ).
void genApriori(double I_A, const std::vector<uint8_t>& b,
                std::vector<float>& L, std::mt19937& rng) {
    L.resize(b.size());
    double sigma = Jinv(I_A);
    double var   = sigma * sigma;
    std::normal_distribution<double> nd(0.0, sigma > 0 ? sigma : 1e-6);
    for (size_t i = 0; i < b.size(); ++i) {
        double mean = (b[i] ? -1.0 : 1.0) * var * 0.5;  // (1-2b)*sigma^2/2
        L[i] = static_cast<float>(mean + nd(rng));
    }
}

// =========================================================================
// Prior-aware max-log-APP soft demapper — a faithful mirror of
// BICMDecoder::demapWithPriors (include/bicm.hpp ~160-204). For each bit it
// marginalizes over the full constellation, folding the OTHER bits' a-priori
// LLRs into each candidate's log-probability, and emits the EXTRINSIC LLR
// (the bit's own prior is excluded). Positive => bit 0.
// =========================================================================
void demapWithPriors(const SymbolMapper& mapper, const ComplexBuf& syms,
                     float noise_var, const std::vector<float>& prior,
                     std::vector<float>& out) {
    const ComplexBuf& C = mapper.constellation();
    const size_t bps = mapper.bitsPerSymbol();
    const size_t M   = C.size();
    out.assign(syms.size() * bps, 0.f);
    if (M == 0 || bps == 0) {           // BPSK has no table -> plain demap
        mapper.demapSoftPWL(syms, noise_var, out);
        return;
    }
    const float inv2s2 = 1.0f / (2.0f * std::max(noise_var, 1e-6f));
    std::vector<float> base(M);
    for (size_t s = 0; s < syms.size(); ++s) {
        const ComplexSample y = syms[s];
        for (size_t idx = 0; idx < M; ++idx) {
            float m = -std::norm(y - C[idx]) * inv2s2;
            for (size_t j = 0; j < bps; ++j) {
                float Lj = prior[s * bps + j];
                bool  bj = (idx >> (bps - 1 - j)) & 1u;
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
            out[s * bps + k] = max0 - max1;   // extrinsic, positive => bit 0
        }
    }
}

// =========================================================================
// Es/N0 (dB) -> complex AWGN variance N0 for a UNIT-average-energy symbol set
// (the SymbolMapper normalizes constellations to unit average energy).
// demapSoft's noise_var argument is this complex variance N0.
// =========================================================================
float esnoToN0(float esno_db) {
    return 1.0f / std::pow(10.0f, esno_db / 10.0f);
}

// One point of the DEMAPPER curve: I_E_dem given a-priori MI I_A at fixed Es/N0.
double demapperPoint(Modulation mod, Labeling lab, float esno_db, double I_A,
                     size_t n_syms, std::mt19937& rng) {
    SymbolMapper mapper(mod, lab);
    const size_t bps = mapper.bitsPerSymbol();
    const float N0   = esnoToN0(esno_db);
    const float sigma_n = std::sqrt(N0 * 0.5f);   // per real dimension

    // Random coded bits -> packed bytes -> symbols.
    const size_t n_bits  = n_syms * bps;
    const size_t n_bytes = (n_bits + 7) / 8;
    std::vector<uint8_t> bytes(n_bytes, 0);
    std::vector<uint8_t> bits(n_bits, 0);
    std::uniform_int_distribution<int> bd(0, 1);
    for (size_t i = 0; i < n_bits; ++i) {
        uint8_t v = static_cast<uint8_t>(bd(rng));
        bits[i] = v;
        if (v) bytes[i >> 3] |= static_cast<uint8_t>(1u << (7 - (i & 7)));
    }

    ComplexBuf syms;
    mapper.mapBytes(bytes.data(), n_bits, syms);

    // AWGN at the channel Es/N0.
    std::normal_distribution<float> nd(0.f, sigma_n);
    for (auto& s : syms) s += ComplexSample(nd(rng), nd(rng));

    // A-priori LLRs of MI I_A on the (known) coded bits.
    std::vector<float> prior;
    genApriori(I_A, bits, prior, rng);

    // Prior-aware extrinsic demap.
    std::vector<float> ext;
    demapWithPriors(mapper, syms, N0, prior, ext);
    ext.resize(n_bits);

    return measureMI(ext, bits);
}

// One point of the DECODER curve: I_E_dec given a-priori MI I_A on the codeword
// bits of the (known) all-zero codeword. Runs ONE decodePosterior pass and
// measures extrinsic = posterior - input.
double decoderPoint(FECRate fec, double I_A, std::mt19937& rng) {
    // One decodePosterior CALL = one BICM-ID outer pass. Inside it the LDPC runs
    // its normal inner BP iterations (BICMConfig::ldpc_inner_iter default = 30),
    // so use that here — a single BP iteration would badly understate the
    // decoder's transfer curve.
    LDPCDecoder dec(fec, LDPCBlockSize::Short, /*max_iter*/ 30);
    const size_t n = dec.codewordBits();
    std::vector<uint8_t> zeros(n, 0);          // all-zero codeword (always valid)

    std::vector<float> in;
    genApriori(I_A, zeros, in, rng);

    std::vector<float> post;
    dec.decodePosterior(in.data(), post);
    post.resize(n, 0.f);

    std::vector<float> ext(n);
    for (size_t i = 0; i < n; ++i) ext[i] = post[i] - in[i];

    return measureMI(ext, zeros);
}

// =========================================================================
// CLI parsing for modulation / FEC.
// =========================================================================
bool parseMod(const char* s, Modulation& m) {
    if (!std::strcmp(s, "bpsk") || !std::strcmp(s, "1"))   { m = Modulation::BPSK;   return true; }
    if (!std::strcmp(s, "qpsk") || !std::strcmp(s, "4") || !std::strcmp(s, "2")) { m = Modulation::QPSK; return true; }
    if (!std::strcmp(s, "16")   || !std::strcmp(s, "16qam")) { m = Modulation::QAM16;  return true; }
    if (!std::strcmp(s, "64")   || !std::strcmp(s, "64qam")) { m = Modulation::QAM64;  return true; }
    if (!std::strcmp(s, "256")  || !std::strcmp(s, "256qam")){ m = Modulation::QAM256; return true; }
    return false;
}
bool parseFec(const char* s, FECRate& f) {
    if (!std::strcmp(s, "1_2") || !std::strcmp(s, "1/2")) { f = FECRate::Rate_1_2; return true; }
    if (!std::strcmp(s, "2_3") || !std::strcmp(s, "2/3")) { f = FECRate::Rate_2_3; return true; }
    if (!std::strcmp(s, "3_4") || !std::strcmp(s, "3/4")) { f = FECRate::Rate_3_4; return true; }
    if (!std::strcmp(s, "5_6") || !std::strcmp(s, "5/6")) { f = FECRate::Rate_5_6; return true; }
    return false;
}

// =========================================================================
// Self-test: J/Jinv round-trip, MI-range, demapper monotonicity. Fast ctest.
// =========================================================================
int g_pass = 0, g_fail = 0;
void check(bool cond, const char* msg) {
    if (cond) { ++g_pass; std::printf("  [PASS] %s\n", msg); }
    else      { ++g_fail; std::printf("  [FAIL] %s\n", msg); }
}

int runSelftest() {
    std::printf("=== EXIT-chart self-test (J/Jinv, MI range, monotonicity) ===\n");

    // (1) Jinv(J(sigma)) ~ sigma over a representative range.
    bool rt_ok = true;
    double max_err = 0.0;
    for (double sigma = 0.2; sigma <= 6.0; sigma += 0.2) {
        double back = Jinv(Jfunc(sigma));
        double err = std::fabs(back - sigma);
        max_err = std::max(max_err, err);
        if (err > 0.15 + 0.05 * sigma) rt_ok = false;   // loose tol (poly approx)
    }
    char b1[96];
    std::snprintf(b1, sizeof(b1), "Jinv(J(sigma)) ~ sigma (max abs err %.3f)", max_err);
    check(rt_ok, b1);

    // (2) J maps into [0,1] and is monotone increasing.
    bool jmono = true, jrange = true;
    double prev = -1.0;
    for (double sigma = 0.0; sigma <= 8.0; sigma += 0.25) {
        double j = Jfunc(sigma);
        if (j < 0.0 || j > 1.0) jrange = false;
        if (j < prev - 1e-9) jmono = false;
        prev = j;
    }
    check(jrange, "J(sigma) in [0,1] over sweep");
    check(jmono,  "J(sigma) monotone increasing");

    // (3) MI estimator returns values in [0,1] for generated a-priori LLRs,
    //     and tracks the requested I_A (within Monte-Carlo tolerance).
    std::mt19937 rng(0xE1A7u);
    std::vector<uint8_t> bits(20000);
    std::uniform_int_distribution<int> bd(0, 1);
    for (auto& b : bits) b = static_cast<uint8_t>(bd(rng));
    bool mi_range = true, mi_track = true;
    for (double ia = 0.0; ia <= 1.0001; ia += 0.1) {
        std::vector<float> L;
        genApriori(ia, bits, L, rng);
        double mi = measureMI(L, bits);
        if (mi < 0.0 || mi > 1.0) mi_range = false;
        if (std::fabs(mi - ia) > 0.08) mi_track = false;
    }
    check(mi_range, "measureMI output in [0,1]");
    check(mi_track, "measureMI(genApriori(I_A)) ~ I_A");

    // (4) Demapper EXIT curve is monotone non-decreasing in I_A and in [0,1].
    Modulation mod = Modulation::QAM64;
    float esno = computeThreshold(mod, FECRate::Rate_2_3).threshold_db + 1.0f;
    bool dem_mono = true, dem_range = true;
    double prevIE = -1.0;
    for (double ia = 0.0; ia <= 1.0001; ia += 0.2) {
        double ie = demapperPoint(mod, Labeling::Gray, esno, ia, 4000, rng);
        if (ie < 0.0 || ie > 1.0) dem_range = false;
        if (ie < prevIE - 0.03) dem_mono = false;   // small slack for MC noise
        prevIE = ie;
    }
    check(dem_range, "demapper I_E in [0,1]");
    check(dem_mono,  "demapper I_E monotone non-decreasing in I_A");

    // (5) Anti-Gray demapper EXIT curve has a strictly POSITIVE slope (the
    //     whole point of the labeling), unlike Gray's near-flat curve.
    {
        float es = computeThreshold(Modulation::QAM16, FECRate::Rate_1_2)
                       .threshold_db + 1.0f;
        double g0 = demapperPoint(Modulation::QAM16, Labeling::Gray,     es, 0.0,  6000, rng);
        double g1 = demapperPoint(Modulation::QAM16, Labeling::Gray,     es, 0.95, 6000, rng);
        double a0 = demapperPoint(Modulation::QAM16, Labeling::AntiGray, es, 0.0,  6000, rng);
        double a1 = demapperPoint(Modulation::QAM16, Labeling::AntiGray, es, 0.95, 6000, rng);
        char b[128];
        std::snprintf(b, sizeof(b),
            "anti-Gray slope %+.3f > Gray slope %+.3f (16-QAM)",
            a1 - a0, g1 - g0);
        check((a1 - a0) > 0.1 && (a1 - a0) > (g1 - g0) + 0.1, b);
    }

    std::printf("\n=== Self-test: %d passed, %d failed ===\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}

// =========================================================================
// Full run: tabulate both curves and judge the tunnel.
// =========================================================================
int runChart(Modulation mod, FECRate fec, float esno_db, Labeling lab) {
    const size_t n_syms_dem = 60000;   // symbols/point for a stable demapper MI
    std::mt19937 rng(0x1C0DEu);

    const float thr = computeThreshold(mod, fec).threshold_db;
    std::printf("=== DSCA-NG BICM-ID EXIT chart ===\n");
    std::printf("modcod = %s %s   labeling = %s   channel Es/N0 = %.2f dB   "
                "(computeThreshold = %.2f dB, margin %+.2f dB)\n",
                modulationName(mod), fecRateName(fec),
                lab == Labeling::AntiGray ? "anti-Gray" : "Gray",
                esno_db, thr, esno_db - thr);
    std::printf("Demapper: prior-aware max-log-APP (mirrors "
                "BICMDecoder::demapWithPriors).\n");
    std::printf("Decoder : one LDPCDecoder::decodePosterior pass, Short block "
                "(n=%zu).\n\n", LDPCDecoder(fec, LDPCBlockSize::Short).codewordBits());

    // Grid I_A = 0 .. 1 step 0.1.
    std::vector<double> grid;
    for (int i = 0; i <= 10; ++i) grid.push_back(i * 0.1);

    std::vector<double> ie_dem(grid.size()), ie_dec(grid.size());
    std::printf("   I_A   | I_E_demap | I_E_decode\n");
    std::printf("  -------+-----------+-----------\n");
    for (size_t i = 0; i < grid.size(); ++i) {
        ie_dem[i] = demapperPoint(mod, lab, esno_db, grid[i], n_syms_dem, rng);
        ie_dec[i] = decoderPoint(fec, grid[i], rng);
        std::printf("  %5.2f  |   %5.3f   |   %5.3f\n",
                    grid[i], ie_dem[i], ie_dec[i]);
    }

    // Demapper EXIT slope: I_E_dem(I_A=1) - I_E_dem(I_A=0). Flat (~0) for Gray;
    // strongly positive for an anti-Gray labeling — the signature that the
    // iterative loop has information to exchange.
    double dem_slope = ie_dem.back() - ie_dem.front();
    std::printf("\nDemapper EXIT slope (I_E@I_A=1 - I_E@I_A=0) = %+.3f  [%s]\n",
                dem_slope,
                dem_slope > 0.1 ? "positive: iterations help (anti-Gray-like)"
                                : "flat: little iterative gain (Gray-like)");

    // -------- Tunnel test --------
    // Demapper curve runs (I_A -> I_E_dem). Decoder curve runs (I_A -> I_E_dec);
    // plotted on swapped axes it is the inverse map I_E_dec -> I_A, i.e. the
    // decoder's required INPUT MI to produce a given output. The tunnel is open
    // when, at every operating point, the demapper output exceeds the input the
    // decoder needs to make progress. Equivalently, sampling the decoder curve
    // as (x=I_E_dec, y=I_A_dec) and the demapper as (x=I_A, y=I_E_dem), the
    // demapper must lie above the swapped decoder everywhere up to (1,1).
    //
    // We evaluate the gap on a fine MI abscissa u in [0,1):
    //   demapper height at u            = interp(grid -> ie_dem)(u)
    //   decoder (swapped) height at u   = interp(ie_dec -> grid)(u)
    // and report the minimum gap and where it occurs.
    auto interp = [](const std::vector<double>& xs,
                     const std::vector<double>& ys, double u) -> double {
        // xs assumed non-decreasing; clamp at ends.
        if (u <= xs.front()) return ys.front();
        if (u >= xs.back())  return ys.back();
        for (size_t i = 1; i < xs.size(); ++i) {
            if (u <= xs[i]) {
                double dx = xs[i] - xs[i - 1];
                double t = (dx > 1e-12) ? (u - xs[i - 1]) / dx : 0.0;
                return ys[i - 1] + t * (ys[i] - ys[i - 1]);
            }
        }
        return ys.back();
    };

    double min_gap = 1e9;
    double pinch_u = 0.0;
    bool open = true;
    for (double u = 0.0; u <= 0.95 + 1e-9; u += 0.02) {
        double dem_y = interp(grid, ie_dem, u);        // demapper output at I_A=u
        double dec_x = interp(ie_dec, grid, u);        // decoder input needed to
                                                       // emit output MI = u
        double gap = dem_y - dec_x;
        if (gap < min_gap) { min_gap = gap; pinch_u = u; }
        if (gap <= 0.0) open = false;
    }

    std::printf("\nTunnel analysis (demapper output vs decoder-required input):\n");
    std::printf("  minimum gap = %+.3f at I_A ~ %.2f\n", min_gap, pinch_u);
    if (open && min_gap > 0.02) {
        std::printf("  VERDICT: tunnel OPEN -> BICM-ID converges toward (1,1) "
                    "at %.2f dB.\n", esno_db);
    } else if (open) {
        std::printf("  VERDICT: tunnel BARELY open (min gap %.3f at I_A~%.2f) "
                    "-> slow convergence near %.2f dB.\n",
                    min_gap, pinch_u, esno_db);
    } else {
        std::printf("  VERDICT: tunnel PINCHES at I_A ~ %.2f (gap %.3f) -> "
                    "BICM-ID stalls at this SNR.\n", pinch_u, min_gap);
    }
    std::printf("\nNote: EXIT MI is a model-based (consistent-Gaussian a-priori)"
                " prediction of\nconvergence, not a frame BER simulation.\n");
    return 0;
}

} // namespace

int main(int argc, char** argv) {
    bool selftest = false;
    Modulation mod = Modulation::QAM64;
    FECRate    fec = FECRate::Rate_2_3;
    Labeling   lab = Labeling::Gray;
    bool have_esno = false;
    float esno_db = 0.f;

    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--selftest")) selftest = true;
        else if (!std::strcmp(argv[i], "--labeling") && i + 1 < argc) {
            const char* s = argv[++i];
            if (!std::strcmp(s, "gray"))            lab = Labeling::Gray;
            else if (!std::strcmp(s, "antigray") ||
                     !std::strcmp(s, "anti-gray"))  lab = Labeling::AntiGray;
            else { std::fprintf(stderr, "bad --labeling (gray|antigray)\n"); return 2; }
        }
        else if (!std::strcmp(argv[i], "--mod") && i + 1 < argc) {
            if (!parseMod(argv[++i], mod)) {
                std::fprintf(stderr, "bad --mod (bpsk|qpsk|16|64|256)\n"); return 2;
            }
        } else if (!std::strcmp(argv[i], "--fec") && i + 1 < argc) {
            if (!parseFec(argv[++i], fec)) {
                std::fprintf(stderr, "bad --fec (1_2|2_3|3_4|5_6)\n"); return 2;
            }
        } else if (!std::strcmp(argv[i], "--esno") && i + 1 < argc) {
            esno_db = static_cast<float>(std::atof(argv[++i]));
            have_esno = true;
        }
    }

    if (selftest) return runSelftest();

    // Default operating point: ~1 dB above the modcod's modeled threshold.
    if (!have_esno) esno_db = computeThreshold(mod, fec).threshold_db + 1.0f;
    return runChart(mod, fec, esno_db, lab);
}
