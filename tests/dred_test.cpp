/**
 * @file dred_test.cpp
 * @brief SOTA-5: Opus Deep REDundancy (DRED) decoder-side burst recovery.
 *
 * Encodes speech-like audio with DRED enabled, then simulates a multi-frame
 * burst loss and recovers the lost frames two ways:
 *   - OpusAudioDecoder::decodeFromDRED() (the new neural-FEC path), vs
 *   - packet-loss concealment (decodePLC) as the baseline.
 * Asserts DRED reconstructs the burst materially closer to the clean
 * (no-loss) decode than PLC does. Skips cleanly (pass) if the linked libopus
 * was built without DRED.
 */
#include "opus_codec.hpp"

#include <cstdio>
#include <cmath>
#include <vector>

using namespace gw;

namespace {
int g_fails = 0;
void check(bool ok, const char* msg) {
    std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", msg);
    if (!ok) ++g_fails;
}

// One frame of speech-like audio: pitched buzz (harmonics) + vibrato + AM.
void genFrame(std::vector<float>& f, int idx, const OpusConfig& cfg) {
    const int n = (int)cfg.frameSamples();
    f.resize(n);
    const double fs = cfg.sample_rate;
    for (int i = 0; i < n; ++i) {
        double t = double(idx * n + i) / fs;
        double pitch = 130.0 + 10.0 * std::sin(2.0 * M_PI * 4.0 * t);   // vibrato
        double s = 0.0;
        for (int h = 1; h <= 6; ++h) s += std::sin(2.0 * M_PI * pitch * h * t) / h;
        double am = 0.6 + 0.4 * std::sin(2.0 * M_PI * 3.0 * t);          // syllable AM
        f[i] = (float)(0.25 * am * s);
    }
}

double energy(const std::vector<float>& v) {
    double e = 0; for (float s : v) e += double(s) * s; return e;
}

// Energy of `est` relative to `ref`. Neural recovery (DRED) sustains the
// signal energy through a burst; PLC fades toward silence — so this cleanly
// separates them without depending on sample-exact phase.
double energyRatio(const std::vector<float>& est, const std::vector<float>& ref) {
    double p = energy(ref);
    return p > 1e-12 ? energy(est) / p : 0.0;
}

// Best normalized cross-correlation over a small lag search — phase-tolerant
// content match (a periodic signal still aligns at its pitch lag). DRED
// reconstructs the right content; faded PLC does not.
double maxXcorr(const std::vector<float>& a, const std::vector<float>& b, int maxlag) {
    double na = energy(a), nb = energy(b);
    if (na < 1e-12 || nb < 1e-12) return 0.0;
    double denom = std::sqrt(na * nb);
    int n = (int)std::min(a.size(), b.size());
    double best = -1.0;
    for (int lag = -maxlag; lag <= maxlag; ++lag) {
        double s = 0;
        for (int i = 0; i < n; ++i) {
            int j = i + lag;
            if (j >= 0 && j < (int)b.size()) s += double(a[i]) * b[j];
        }
        double c = s / denom;
        if (c > best) best = c;
    }
    return best;
}
} // anonymous

int main() {
    std::printf("=== DRED decoder-side burst recovery (SOTA-5) ===\n");

    OpusConfig cfg;
    cfg.sample_rate = 48000;
    cfg.channels    = 1;
    cfg.bitrate     = 48000;
    cfg.frame_ms    = 10.0f;       // 10 ms → more DRED frame coverage
    cfg.application = OpusApplication::VoIP;
    cfg.inband_fec  = true;
    cfg.expected_loss_perc = 20;
    cfg.dred_frames = 50;          // request DRED (forces voice signal)

    // Build availability gate from a throwaway decoder.
    {
        OpusAudioDecoder probe(cfg);
        if (!probe.dredAvailable()) {
            std::printf("  [SKIP] linked libopus has no DRED — nothing to test, "
                        "treating as pass.\n");
            return 0;
        }
    }

    const int N = 140;             // total frames (~1.4 s @ 10 ms)
    const int PRIME = 90;          // frames decoded before the loss
    const int BURST = 4;           // consecutive lost frames

    OpusAudioEncoder enc(cfg);
    std::vector<std::vector<uint8_t>> packets;
    std::vector<std::vector<float>>   orig;
    for (int i = 0; i < N; ++i) {
        std::vector<float> f; genFrame(f, i, cfg);
        std::vector<uint8_t> pkt;
        if (enc.encode(f.data(), pkt) && !pkt.empty()) {
            packets.push_back(pkt);
            orig.push_back(f);
        }
    }
    check((int)packets.size() == N, "encoded all frames with DRED");

    // Reference: a clean decoder that never loses anything — the best the
    // decoder can do, against which recovery quality is judged.
    OpusAudioDecoder ref_dec(cfg);
    std::vector<std::vector<float>> clean(N);
    for (int i = 0; i < N; ++i)
        ref_dec.decode(packets[i].data(), packets[i].size(), clean[i]);

    // --- Path A: DRED recovery ---
    OpusAudioDecoder dred_dec(cfg);
    std::vector<float> sink;
    for (int i = 0; i < PRIME; ++i)            // prime
        dred_dec.decode(packets[i].data(), packets[i].size(), sink);
    // Lose frames PRIME..PRIME+BURST-1; receive packet PRIME+BURST.
    std::vector<float> dred_pcm;
    size_t dred_n = dred_dec.decodeFromDRED(
        packets[PRIME + BURST].data(), packets[PRIME + BURST].size(),
        BURST, dred_pcm);
    std::printf("  DRED reconstructed %zu/%d burst frames\n", dred_n, BURST);

    // --- Path B: PLC baseline ---
    OpusAudioDecoder plc_dec(cfg);
    for (int i = 0; i < PRIME; ++i)
        plc_dec.decode(packets[i].data(), packets[i].size(), sink);
    std::vector<float> plc_pcm;
    for (int k = 0; k < BURST; ++k) {
        std::vector<float> f; plc_dec.decodePLC(f);
        plc_pcm.insert(plc_pcm.end(), f.begin(), f.end());
    }

    // Build the clean reference for the burst window.
    std::vector<float> ref_burst;
    for (int k = 0; k < BURST; ++k)
        ref_burst.insert(ref_burst.end(), clean[PRIME + k].begin(), clean[PRIME + k].end());

    const int maxlag = (int)cfg.frameSamples();   // ~one frame of lag slack
    double dred_er = energyRatio(dred_pcm, ref_burst);
    double plc_er  = energyRatio(plc_pcm,  ref_burst);
    double dred_xc = maxXcorr(dred_pcm, ref_burst, maxlag);
    double plc_xc  = maxXcorr(plc_pcm,  ref_burst, maxlag);
    std::printf("  energy ratio (1.0=full):   DRED=%.2f   PLC=%.2f\n", dred_er, plc_er);
    std::printf("  content xcorr (1.0=ideal): DRED=%.2f   PLC=%.2f\n", dred_xc, plc_xc);

    char m1[140];
    std::snprintf(m1, sizeof(m1),
                  "DRED embedded + reconstructed the burst (%zu/%d frames)",
                  dred_n, BURST);
    check(dred_n >= 1, m1);

    char m2[176];
    std::snprintf(m2, sizeof(m2),
                  "DRED sustains signal energy through the burst where PLC fades "
                  "(%.2f >= 1.5x PLC %.2f, and not negligible)", dred_er, plc_er);
    check(dred_er > 1.5 * plc_er && dred_er > 0.10, m2);

    char m3[160];
    std::snprintf(m3, sizeof(m3),
                  "DRED content matches the lost audio better than PLC "
                  "(xcorr %.2f vs %.2f)", dred_xc, plc_xc);
    check(dred_xc > plc_xc && dred_xc > 0.3, m3);

    std::printf("\n%s (%d failure%s)\n",
                g_fails == 0 ? "ALL PASS" : "FAILURES",
                g_fails, g_fails == 1 ? "" : "s");
    return g_fails == 0 ? 0 : 1;
}
