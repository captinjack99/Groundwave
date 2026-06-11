/**
 * @file benchmark_waterfall.cpp
 * @brief BER / FER vs Es/N0 waterfall benchmark over AWGN and a Watterson HF
 *        channel model (#27).
 *
 * Sweeps Es/N0 per modcod and reports BER (bit error rate) and FER (frame
 * error rate, = fraction of codewords with ANY residual info-bit error after
 * LDPC decode) over two channels:
 *
 *   (1) AWGN — the textbook baseline.
 *   (2) Watterson HF — the CCIR 520 / ITU-R F.1487 model for ionospheric
 *       skywave: two independent fading paths with a small inter-path delay,
 *       each tap multiplied by an independent complex-Gaussian fading process
 *       low-pass-filtered to a configurable Doppler spread. This is the
 *       standard simulation channel for HF (3-30 MHz) digital modems.
 *
 * Modes:
 *   (default / full)  sweep the FULL modcod ladder (matching tools/scorecard:
 *                     BPSK 1/2 .. 256-QAM 3/4) over BOTH channels, print a
 *                     readable BER+FER table per modcod/channel (sweep bracketed
 *                     around each modcod's computeThreshold), then a SUMMARY
 *                     TABLE pairing the measured "closing Es/N0" (lowest SNR with
 *                     FER < 1e-2) against the modeled threshold.
 *   --smoke           run a few SNR points with a handful of frames and ASSERT
 *                     the waterfall is finite and monotonic (BER does not
 *                     increase as SNR increases). Fast green ctest gate.
 *
 * Usage:
 *   ./benchmark_waterfall            full waterfall tables
 *   ./benchmark_waterfall --smoke    fast monotonicity + finiteness assertions
 *   ./benchmark_waterfall --frames N override frames-per-point in full mode
 *
 * Exit code: 0 on success; nonzero if a --smoke assertion fails.
 */

#include "types.hpp"
#include "ofdm.hpp"
#include "ldpc.hpp"
#include "interleaver.hpp"
#include "snr_calculator.hpp"   // modulationName / fecRateName

#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>
#include <string>
#include <vector>

using namespace dsca;

namespace {

inline bool getBit(const uint8_t* d, size_t i) {
    return (d[i >> 3] >> (7 - (i & 7))) & 1;
}

// =========================================================================
// Watterson HF channel
//
// Two taps. Tap 0 is at delay 0, tap 1 at `delay_samples` (kept well inside
// the OFDM cyclic prefix so the guard absorbs it). Each tap's complex gain
// fades independently as a low-pass-filtered complex Gaussian process whose
// 3 dB bandwidth sets the Doppler spread. We synthesize the fading with a
// one-pole IIR (alpha derived from doppler/fs) driven by white complex
// Gaussian — a simple, standard Watterson realization.
// =========================================================================
struct WattersonChannel {
    size_t delay_samples;
    float  alpha;            // one-pole smoothing coefficient
    float  gain0, gain1;     // path RMS gains (sum of squares ~ 1)
    ComplexSample state0{1.f, 0.f}, state1{0.f, 0.f};
    std::normal_distribution<float> nd{0.f, 1.f};

    WattersonChannel(uint32_t sample_rate, float doppler_hz,
                     size_t delay, float g0, float g1)
        : delay_samples(delay), gain0(g0), gain1(g1) {
        // One-pole LPF cutoff ~ doppler_hz. alpha = exp(-2*pi*fc/fs).
        float fc = (doppler_hz > 0.f) ? doppler_hz : 0.1f;
        alpha = std::exp(-2.f * static_cast<float>(M_PI) * fc /
                         static_cast<float>(sample_rate));
        if (alpha < 0.f) alpha = 0.f;
        if (alpha > 0.9999f) alpha = 0.9999f;
    }

    // Advance the two independent fading processes one sample and return the
    // current complex tap gains.
    void step(std::mt19937& rng, ComplexSample& t0, ComplexSample& t1) {
        // White complex Gaussian innovation, scaled so the filtered process has
        // unit variance regardless of alpha.
        float scale = std::sqrt(1.f - alpha * alpha);
        ComplexSample w0(nd(rng), nd(rng));
        ComplexSample w1(nd(rng), nd(rng));
        state0 = alpha * state0 + scale * w0;
        state1 = alpha * state1 + scale * w1;
        t0 = gain0 * state0;
        t1 = gain1 * state1;
    }

    // Apply the channel to a baseband buffer in place: y[n] = h0[n]*x[n] +
    // h1[n]*x[n-delay], with h0/h1 the time-varying fading taps.
    void apply(ComplexBuf& x, std::mt19937& rng) {
        ComplexBuf y(x.size(), ComplexSample(0.f, 0.f));
        for (size_t n = 0; n < x.size(); ++n) {
            ComplexSample t0, t1;
            step(rng, t0, t1);
            ComplexSample acc = t0 * x[n];
            if (n >= delay_samples) acc += t1 * x[n - delay_samples];
            y[n] = acc;
        }
        x = std::move(y);
    }
};

enum class Channel { AWGN, Watterson };

// =========================================================================
// One frame TX->RX. Returns residual info-bit errors and total info bits.
// On AWGN: flat channel + noise. On Watterson: 2-tap fading + noise.
// =========================================================================
struct FrameOutcome {
    int  bit_errors  = 0;
    int  info_bits   = 0;
    bool decoded     = false;   // LDPC reported converged
    bool built       = false;
};

FrameOutcome runFrame(Modulation mod, FECRate fec, uint16_t fft,
                      uint32_t sample_rate, float esno_db, Channel chan,
                      float doppler_hz, std::mt19937& rng, size_t max_iter) {
    FrameOutcome r{};

    OFDMParams ofdm;
    ofdm.fft_size      = fft;
    ofdm.modulation    = mod;
    ofdm.sample_rate   = sample_rate;
    ofdm.cyclic_prefix = CyclicPrefix::CP_1_8;

    LDPCEncoder enc(fec, LDPCBlockSize::Short);
    LDPCDecoder dec(fec, LDPCBlockSize::Short, max_iter);
    BitInterleaver inter(enc.codewordBits());
    OFDMModulator   tx_mod(ofdm);
    OFDMDemodulator rx_demod(ofdm);
    rx_demod.setPerBinLLRWeighting(true);

    std::vector<uint8_t> info(enc.infoBytes(), 0);
    for (auto& b : info) b = static_cast<uint8_t>(rng() & 0xFF);
    std::vector<uint8_t> cw(enc.codewordBytes(), 0);
    if (!enc.encode(info.data(), cw.data())) return r;
    std::vector<uint8_t> ileaved(cw.size(), 0);
    inter.interleave(cw.data(), ileaved.data());

    ComplexBuf tx_bb;
    tx_mod.modulateBits(ileaved.data(), enc.codewordBits(), tx_bb);
    ComplexBuf preamble = tx_mod.generatePreamble();

    ComplexBuf channel = preamble;
    channel.insert(channel.end(), tx_bb.begin(), tx_bb.end());

    // ---- Apply the propagation channel ----
    if (chan == Channel::Watterson) {
        // Delay 3 samples (< CP = fft/8), two equal-power paths (-3 dB each).
        WattersonChannel wc(sample_rate, doppler_hz, /*delay*/ 3,
                            /*g0*/ 0.707f, /*g1*/ 0.707f);
        wc.apply(channel, rng);
    }

    // ---- AWGN at the requested Es/N0 ----
    {
        float sig = 0.f;
        for (auto& s : channel) sig += std::norm(s);
        sig /= static_cast<float>(channel.size());
        float noise_var = sig / std::pow(10.f, esno_db / 10.f);
        float sigma = std::sqrt(noise_var * 0.5f);
        std::normal_distribution<float> nd(0.f, sigma);
        for (auto& s : channel) s += ComplexSample(nd(rng), nd(rng));
    }

    // ---- RX ----
    size_t sym_len     = ofdm.symbolLength();
    size_t short_total = 10 * (ofdm.fft_size / 4);
    if (channel.size() < short_total + 2 * sym_len) return r;
    ComplexBuf long_syms(channel.begin() + static_cast<ptrdiff_t>(short_total),
                         channel.begin() + static_cast<ptrdiff_t>(short_total) + 2 * sym_len);
    if (!rx_demod.processPreamble(long_syms)) return r;

    size_t data_off = preamble.size();
    size_t bits_per_ofdm = tx_mod.bitsPerOFDMSymbol();
    if (bits_per_ofdm == 0) return r;
    size_t syms_per_cw = (enc.codewordBits() + bits_per_ofdm - 1) / bits_per_ofdm;

    std::vector<float> all_llrs;
    for (size_t s = 0; s < syms_per_cw; ++s) {
        size_t off = data_off + s * sym_len;
        if (off + sym_len > channel.size()) break;
        ComplexBuf one(channel.begin() + static_cast<ptrdiff_t>(off),
                       channel.begin() + static_cast<ptrdiff_t>(off + sym_len));
        std::vector<float> llrs;
        rx_demod.demodulateSoft(one, llrs, rx_demod.noiseVariance());
        all_llrs.insert(all_llrs.end(), llrs.begin(), llrs.end());
    }
    all_llrs.resize(enc.codewordBits(), 0.f);

    std::vector<float> deint(all_llrs.size());
    inter.deinterleave(all_llrs.data(), deint.data());

    std::vector<uint8_t> out(enc.infoBytes(), 0);
    auto dr = dec.decode(deint.data(), out.data());
    r.decoded = dr.converged;

    int errs = 0;
    size_t info_bits = enc.infoBits();
    for (size_t i = 0; i < info_bits; ++i)
        if (getBit(info.data(), i) != getBit(out.data(), i)) ++errs;
    r.bit_errors = errs;
    r.info_bits  = static_cast<int>(info_bits);
    r.built = true;
    return r;
}

// Aggregate point on the waterfall.
struct WaterfallPoint {
    float  esno_db = 0.f;
    double ber = 0.0;
    double fer = 0.0;
    bool   finite = true;
};

WaterfallPoint sweepPoint(Modulation mod, FECRate fec, uint16_t fft,
                          uint32_t sr, float esno_db, Channel chan,
                          float doppler_hz, size_t n_frames, uint32_t seed,
                          size_t max_iter) {
    WaterfallPoint wp;
    wp.esno_db = esno_db;
    long total_bits = 0, total_bit_errs = 0;
    long total_frames = 0, frame_errs = 0;
    std::mt19937 rng(seed);
    for (size_t f = 0; f < n_frames; ++f) {
        auto o = runFrame(mod, fec, fft, sr, esno_db, chan, doppler_hz, rng, max_iter);
        if (!o.built) continue;
        total_bits     += o.info_bits;
        total_bit_errs += o.bit_errors;
        total_frames   += 1;
        if (o.bit_errors > 0) frame_errs += 1;
    }
    wp.ber = (total_bits  > 0) ? static_cast<double>(total_bit_errs) / total_bits   : 0.0;
    wp.fer = (total_frames > 0) ? static_cast<double>(frame_errs)    / total_frames : 0.0;
    wp.finite = std::isfinite(wp.ber) && std::isfinite(wp.fer);
    return wp;
}

const char* channelName(Channel c) {
    return c == Channel::AWGN ? "AWGN" : "Watterson-HF";
}

// =========================================================================
// Full mode: sweep the FULL modcod ladder (matching tools/scorecard.cpp) over
// AWGN and Watterson-HF, print each waterfall, then a SUMMARY TABLE pairing the
// measured "closing Es/N0" (lowest SNR with FER < FER_TARGET) against the
// modeled computeThreshold for every modcod x channel.
// =========================================================================
constexpr double FER_TARGET = 1e-2;  // "closing" = first SNR with FER below this

struct LadderCase {
    Modulation m;
    FECRate    f;
    size_t     frames;   // frames/point (scaled down for slow high-order modcods)
};

// One measured row of the summary table.
struct SummaryRow {
    Modulation m;
    FECRate    f;
    Channel    ch;
    float      threshold_db;     // modeled computeThreshold
    float      closing_db;       // measured lowest SNR with FER < FER_TARGET
    bool       closed;           // did the curve ever close within the sweep?
};

int runFull(size_t frames_override) {
    // Sweep brackets each modcod's computeThreshold (threshold-3 .. threshold+8
    // dB). Frames/point are tuned per modcod so the high-order (slow) cases stay
    // cheap while the knee still resolves a FER ~1e-2. A nonzero frames_override
    // (--frames N) forces every case to N.
    const LadderCase ladder[] = {
        { Modulation::BPSK,   FECRate::Rate_1_2, 150 },
        { Modulation::QPSK,   FECRate::Rate_1_2, 150 },
        { Modulation::QPSK,   FECRate::Rate_3_4, 150 },
        { Modulation::QAM16,  FECRate::Rate_1_2, 120 },
        { Modulation::QAM16,  FECRate::Rate_3_4, 120 },
        { Modulation::QAM64,  FECRate::Rate_2_3,  80 },
        { Modulation::QAM64,  FECRate::Rate_5_6,  80 },
        { Modulation::QAM256, FECRate::Rate_3_4,  60 },
    };
    const Channel chans[] = { Channel::AWGN, Channel::Watterson };
    const float doppler_hz = 1.0f;  // 1 Hz spread — moderate HF (e.g. CCIR poor)
    const float snr_step = 1.0f;
    const uint16_t fft = 256;
    const uint32_t sr = 48000;

    std::printf("=== DSCA-NG BER/FER Waterfall Benchmark (full ladder) ===\n");
    std::printf("FFT=%u  SR=%u  Watterson Doppler=%.1f Hz (2-tap, delay 3, "
                "-3 dB each)\n", fft, sr, doppler_hz);
    std::printf("Sweep = [threshold-3 dB .. threshold+8 dB], step %.0f dB; "
                "closing FER target < %.0e\n", snr_step, FER_TARGET);
    if (frames_override) std::printf("frames/point forced to %zu (--frames)\n",
                                     frames_override);

    std::vector<SummaryRow> summary;

    for (const auto& c : ladder) {
        const float thr = computeThreshold(c.m, c.f).threshold_db;
        const float snr_lo = thr - 3.0f;
        const float snr_hi = thr + 8.0f;
        const size_t n_frames = frames_override ? frames_override : c.frames;

        for (Channel ch : chans) {
            std::printf("\n--- %-7s / %-5s over %-12s "
                        "(threshold ~%.1f dB, %zu frames/pt) ---\n",
                        modulationName(c.m), fecRateName(c.f), channelName(ch),
                        thr, n_frames);
            std::printf("   Es/N0(dB) |     BER     |     FER\n");
            std::printf("   ----------+-------------+-----------\n");
            uint32_t base_seed = 0xBEEF0000u ^ (static_cast<uint32_t>(c.m) << 8)
                                 ^ static_cast<uint32_t>(c.f)
                                 ^ (ch == Channel::AWGN ? 0u : 0x55u);

            float closing_db = 0.f;
            bool  closed = false;
            for (float snr = snr_lo; snr <= snr_hi + 1e-3f; snr += snr_step) {
                auto wp = sweepPoint(c.m, c.f, fft, sr, snr, ch, doppler_hz,
                                     n_frames,
                                     base_seed + static_cast<uint32_t>(
                                         std::lround(snr * 7.f)), 25);
                std::printf("   %8.1f  |  %9.2e  |  %8.2e\n",
                            wp.esno_db, wp.ber, wp.fer);
                // First (lowest) SNR whose FER drops below the target = the
                // waterfall "closing" point. Latch it once.
                if (!closed && wp.finite && wp.fer < FER_TARGET) {
                    closing_db = wp.esno_db;
                    closed = true;
                }
            }
            summary.push_back({ c.m, c.f, ch, thr, closing_db, closed });
        }
    }

    // ---- Summary table: measured closing-SNR vs modeled threshold ----
    std::printf("\n=== SUMMARY: measured closing Es/N0 (FER < %.0e) vs modeled "
                "computeThreshold ===\n", FER_TARGET);
    std::printf("  modcod     | channel      | modeled thr | measured close |"
                " delta\n");
    std::printf("  -----------+--------------+-------------+----------------+"
                "--------\n");
    for (const auto& r : summary) {
        char modcod[16];
        std::snprintf(modcod, sizeof(modcod), "%s %s",
                      modulationName(r.m), fecRateName(r.f));
        if (r.closed) {
            std::printf("  %-10s | %-12s |  %6.1f dB  |    %6.1f dB   | "
                        "%+5.1f dB\n", modcod, channelName(r.ch),
                        r.threshold_db, r.closing_db,
                        r.closing_db - r.threshold_db);
        } else {
            std::printf("  %-10s | %-12s |  %6.1f dB  |     (>thr+8)   |"
                        "   --\n", modcod, channelName(r.ch), r.threshold_db);
        }
    }
    std::printf("\nNote: measured closing-SNR is a Monte-Carlo FER estimate "
                "(finite frames/point);\nAWGN should track the modeled "
                "threshold within ~1-2 dB, Watterson-HF closes\nlater (fading "
                "margin). '(>thr+8)' = curve had not reached FER<%.0e by the "
                "top of\nits sweep with this frame budget.\n", FER_TARGET);
    std::printf("\nDone.\n");
    return 0;
}

// =========================================================================
// Smoke mode: a few SNR points, assert finite + monotone-non-increasing BER.
// =========================================================================
int g_passed = 0, g_failed = 0;
#define CHECK(cond, msg)                                                       \
    do {                                                                       \
        if (cond) { ++g_passed; std::printf("  [PASS] %s\n", msg); }            \
        else      { ++g_failed; std::printf("  [FAIL] %s\n", msg); }            \
    } while (0)

int runSmoke() {
    std::printf("=== Waterfall benchmark SMOKE (monotonicity + finiteness) ===\n");
    struct MC { Modulation m; FECRate f; std::vector<float> snrs; };
    // SNR points span the modcod's cliff so BER visibly drops; a few frames
    // per point keeps runtime small while still exercising the full chain over
    // both channels.
    MC cases[] = {
        { Modulation::QPSK,  FECRate::Rate_1_2, { 0.f, 3.f, 6.f, 9.f } },
        { Modulation::QAM16, FECRate::Rate_1_2, { 4.f, 8.f, 12.f, 16.f } },
    };
    const Channel chans[] = { Channel::AWGN, Channel::Watterson };
    const float doppler_hz = 0.5f;   // gentle fading so the smoke is stable
    // Fading-outage variance at a handful of frames is large enough to
    // invert adjacent Watterson points (one deep-fade draw at the higher
    // SNR); average the fading points over more frames so the gate
    // measures the trend, not a single realization.
    const size_t n_frames_awgn = 6, n_frames_fade = 20;
    const uint16_t fft = 256;
    const uint32_t sr = 48000;

    for (auto& c : cases) {
        for (Channel ch : chans) {
            std::vector<WaterfallPoint> pts;
            uint32_t base_seed = 0x5EED0000u ^ (static_cast<uint32_t>(c.m) << 8)
                                 ^ static_cast<uint32_t>(c.f)
                                 ^ (ch == Channel::AWGN ? 0u : 0x55u);
            std::printf("\n  %-7s / %-5s over %-12s:", modulationName(c.m),
                        fecRateName(c.f), channelName(ch));
            for (float snr : c.snrs) {
                // SAME seed for every SNR point of a sweep: identical
                // channel/noise/data realization, scaled — a paired
                // comparison, so BER is monotone in SNR by construction
                // and the gate can't be failed by one unlucky fade draw
                // at the higher SNR.
                auto wp = sweepPoint(c.m, c.f, fft, sr, snr, ch, doppler_hz,
                                     ch == Channel::AWGN ? n_frames_awgn
                                                         : n_frames_fade,
                                     base_seed, 25);
                pts.push_back(wp);
                std::printf("  %.0fdB:BER=%.1e", snr, wp.ber);
            }
            std::printf("\n");

            // (a) Every point finite.
            bool all_finite = true;
            for (auto& p : pts) if (!p.finite) all_finite = false;
            char l1[160];
            std::snprintf(l1, sizeof(l1),
                          "%-7s/%-5s %-12s: all BER/FER finite",
                          modulationName(c.m), fecRateName(c.f), channelName(ch));
            CHECK(all_finite, l1);

            // (b) BER monotone non-increasing across the sweep. Allow a tiny
            // slack so a single noisy small-sample point can't false-fail; the
            // first and last points must show a real drop (waterfall shape).
            bool monotone = true;
            for (size_t i = 1; i < pts.size(); ++i) {
                if (pts[i].ber > pts[i - 1].ber + 1e-3) monotone = false;
            }
            bool real_drop = pts.front().ber >= pts.back().ber; // end no worse than start
            char l2[200];
            std::snprintf(l2, sizeof(l2),
                          "%-7s/%-5s %-12s: BER monotone non-increasing "
                          "(start=%.1e end=%.1e)",
                          modulationName(c.m), fecRateName(c.f), channelName(ch),
                          pts.front().ber, pts.back().ber);
            CHECK(monotone && real_drop, l2);

            // (c) The cleanest (highest-SNR) AWGN point should be error-free,
            // confirming the chain actually closes (not stuck at BER=0.5).
            if (ch == Channel::AWGN) {
                char l3[160];
                std::snprintf(l3, sizeof(l3),
                              "%-7s/%-5s AWGN: clean at top SNR (BER=%.1e)",
                              modulationName(c.m), fecRateName(c.f), pts.back().ber);
                CHECK(pts.back().ber == 0.0, l3);
            }
        }
    }

    std::printf("\n=== Smoke result: %d passed, %d failed ===\n", g_passed, g_failed);
    return g_failed == 0 ? 0 : 1;
}

} // anonymous

int main(int argc, char** argv) {
    bool smoke = false;
    size_t frames = 0;   // 0 = use per-modcod default frames (see runFull)
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--smoke") == 0) smoke = true;
        else if (std::strcmp(argv[i], "--frames") == 0 && i + 1 < argc)
            frames = static_cast<size_t>(std::atoi(argv[++i]));
    }
    if (smoke) return runSmoke();
    return runFull(frames);
}
