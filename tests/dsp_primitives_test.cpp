/**
 * @file dsp_primitives_test.cpp
 * @brief Unit tests for Phase 1-5 additions: DC blocker, IQ balance, NCO,
 *        polyphase, CIC, power ramp, squelch, Zadoff-Chu, integer CFO,
 *        phase tracker, SRO, sync FSM, AMC, MIMO, multi-stream.
 *
 * No external test framework — uses simple assert-and-print pattern to
 * stay consistent with the rest of v2's tests.
 */
#include "dc_blocker.hpp"
#include "iq_balance.hpp"
#include "nco.hpp"
#include "polyphase.hpp"
#include "cic.hpp"
#include "power_ramp.hpp"
#include "squelch.hpp"
#include "zadoff_chu.hpp"
#include "integer_cfo.hpp"
#include "phase_tracker.hpp"
#include "sample_rate_offset.hpp"
#include "sync_fsm.hpp"
#include "amc.hpp"
#include "mimo.hpp"
#include "mimo_pipeline.hpp"
#include "multi_stream.hpp"
#ifdef DSCA_ENABLE_AUDIO
#include "hw_audio.hpp"
#endif

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <vector>

using namespace dsca;

namespace {

int g_passed = 0;
int g_failed = 0;

#define CHECK(cond, msg) \
    do { \
        if (cond) { ++g_passed; std::printf("  PASS %s\n", msg); } \
        else      { ++g_failed; std::printf("  FAIL %s\n", msg); } \
    } while (0)

#define NEAR(a, b, eps) (std::fabs((a) - (b)) <= (eps))

void test_dc_blocker() {
    std::printf("[dc_blocker]\n");
    DCBlocker bl;
    bl.setCutoff(50.f, 48000.f);
    // Apply DC offset of +1.0 to a sine — output should converge near zero mean
    std::vector<float> buf(48000);
    for (size_t i = 0; i < buf.size(); ++i)
        buf[i] = 1.0f + std::sin(2.f * static_cast<float>(M_PI) * 1000.f * static_cast<float>(i) / 48000.f);
    bl.process(buf.data(), buf.size());
    float mean = 0.f;
    for (size_t i = 24000; i < buf.size(); ++i) mean += buf[i];
    mean /= 24000.f;
    CHECK(NEAR(mean, 0.f, 0.05f), "DC blocker removes DC offset");
}

void test_nco() {
    std::printf("[nco]\n");
    TableNCO osc;
    // Use 4 kHz at 48 kHz so a quarter cycle is exactly 3 samples — keeps the
    // expected phase a small power-of-two multiple of inc and avoids the 1000/48 kHz
    // rounding that loses bits in 32-bit phase accumulator math.
    osc.setFrequency(4000.0, 48000.0);
    float c, s;
    // step() returns the value AT current phase, then advances. So the first
    // call returns cos(0)=1, sin(0)=0; the 4th call returns cos(3·inc) = cos(π/2).
    osc.stepRealImag(c, s);
    CHECK(NEAR(c, 1.f, 0.01f), "NCO cos(0) = 1");
    CHECK(NEAR(s, 0.f, 0.01f), "NCO sin(0) = 0");
    osc.stepRealImag(c, s);  // phase = 1·inc (30°)
    osc.stepRealImag(c, s);  // phase = 2·inc (60°)
    osc.stepRealImag(c, s);  // phase = 3·inc (90°) — read here
    CHECK(NEAR(c, 0.f, 0.05f) && NEAR(s, 1.f, 0.05f), "NCO sin(π/2) ≈ 1");
}

void test_iq_balance() {
    std::printf("[iq_balance]\n");
    // Generate skewed I/Q (ε amplitude error, no phase error) and verify
    // balance estimate converges in the right direction.
    IQBalanceConfig cfg;
    cfg.ema_alpha = 1e-3f;  // faster for test
    IQBalance bal(cfg);
    std::vector<ComplexSample> sig(20000);
    for (size_t i = 0; i < sig.size(); ++i) {
        float t = static_cast<float>(i) / 48000.f;
        const float TWO_PI_F = 2.f * static_cast<float>(M_PI);
        sig[i] = ComplexSample(1.5f * std::cos(TWO_PI_F * 1000.f * t),
                                1.0f * std::sin(TWO_PI_F * 1000.f * t));
    }
    bal.process(sig.data(), sig.size());
    CHECK(std::fabs(bal.amplitudeImbalanceDb()) > 0.1f,
          "IQ balance detects amplitude skew");
}

void test_polyphase_decimator() {
    std::printf("[polyphase_decimator]\n");
    PolyphaseDecimator dec(4);
    std::vector<float> in(4096), out(4096 / 4 + 32);
    for (size_t i = 0; i < in.size(); ++i)
        in[i] = std::sin(2.f * static_cast<float>(M_PI) * 0.1f *
                         static_cast<float>(i));  // 0.1 normalized freq
    size_t n = dec.process(in.data(), in.size(), out.data());
    CHECK(n == 1024, "Polyphase decimator produces N/M outputs");
}

void test_cic_decimator() {
    std::printf("[cic_decimator]\n");
    CICDecimator cic(8, 3, 1);
    std::vector<float> in(2048), out(256 + 16);
    for (size_t i = 0; i < in.size(); ++i) in[i] = 1.0f;  // DC
    size_t n = cic.process(in.data(), in.size(), out.data());
    CHECK(n == 256, "CIC decimator produces N/R outputs");
    // DC should pass at unit gain
    CHECK(NEAR(out[100], 1.0f, 0.05f), "CIC unity DC gain");
}

void test_power_ramp() {
    std::printf("[power_ramp]\n");
    PowerRamp r(64);
    r.setKey(true);
    float prev = 0.f;
    bool monotone = true;
    for (int i = 0; i < 100; ++i) {
        float g = r.step();
        if (g < prev - 1e-5f) monotone = false;
        prev = g;
    }
    CHECK(monotone, "Power ramp envelope is monotone non-decreasing");
    CHECK(r.isOn(), "Power ramp reaches ON");

    // Ramp-DOWN: the key-off path was unreachable in production (issue #5).
    // After key-off the envelope must fall monotonically and reach 0.
    r.setKey(false);
    prev = 1.f;
    bool down_monotone = true;
    for (int i = 0; i < 100; ++i) {
        float g = r.step();
        if (g > prev + 1e-5f) down_monotone = false;
        prev = g;
    }
    CHECK(down_monotone, "Power ramp envelope is monotone non-increasing after key-off");
    CHECK(r.isOff(), "Power ramp reaches OFF (carrier fully ramped down)");
}

void test_squelch() {
    std::printf("[squelch]\n");
    SquelchConfig cfg;
    cfg.attack_samples = 32;
    cfg.hold_samples   = 32;
    Squelch sq(cfg);
    // Strong signal opens
    for (int i = 0; i < 200; ++i) sq.processSample(0.5f);
    CHECK(sq.isOpen(), "Squelch opens on strong signal");
    // Silence closes
    for (int i = 0; i < 1000; ++i) sq.processSample(1e-6f);
    CHECK(!sq.isOpen(), "Squelch closes on silence");
}

void test_zadoff_chu() {
    std::printf("[zadoff_chu]\n");
    auto seq = zadoffChu(64, 25);
    // Constant amplitude
    bool flat = true;
    for (auto& s : seq) if (!NEAR(std::abs(s), 1.0f, 1e-4f)) flat = false;
    CHECK(flat, "ZC has constant amplitude");
    CHECK(seq.size() == 64, "ZC has correct length");
}

void test_integer_cfo() {
    std::printf("[integer_cfo]\n");
    const size_t N = 64;
    ComplexBuf ref(N, ComplexSample(0.f, 0.f));
    for (size_t i = 8; i < N - 8; ++i) ref[i] = ComplexSample(1.f, 0.f);
    IntegerCFOEstimator est(N, ref, 8);
    // Generate a perfectly aligned IFFT of ref
    FFTEngine fft(N);
    ComplexBuf td;
    fft.inverse(ref, td);
    int d = est.estimate(td);
    CHECK(d == 0, "Integer CFO = 0 for aligned signal");
}

void test_phase_tracker() {
    std::printf("[phase_tracker]\n");
    PhaseTrackerConfig cfg;
    cfg.symbol_rate = 1000.f;
    cfg.loop_bw_hz  = 100.f;
    PhaseTracker pt(cfg);
    // Realistic feedback: a constant target phase, the PLL drives its output
    // toward it. Each iteration the detector sees (target - pt.phaseRad()).
    float target = 30.f * static_cast<float>(M_PI) / 180.f;
    for (int i = 0; i < 500; ++i) {
        float err = target - pt.phaseRad();
        // Wrap err into [-π, π]
        while (err >  static_cast<float>(M_PI)) err -= 2.f * static_cast<float>(M_PI);
        while (err < -static_cast<float>(M_PI)) err += 2.f * static_cast<float>(M_PI);
        pt.update(err);
    }
    CHECK(std::fabs(pt.phaseRad() - target) < 0.05f, "PLL converges toward target phase");
}

void test_sro() {
    std::printf("[sro]\n");
    SROConfig cfg;
    cfg.ema_alpha = 1.0f;
    SROEstimator sro(cfg);
    // Simulate a linear phase ramp across 8 pilots: slope = 0.05 rad/bin
    std::vector<size_t> idx{4, 8, 12, 16, 20, 24, 28, 32};
    std::vector<float> phases(idx.size());
    for (size_t i = 0; i < idx.size(); ++i)
        phases[i] = 0.05f * static_cast<float>(idx[i]);
    float slope = sro.estimateSlope(idx, phases);
    CHECK(NEAR(slope, 0.05f, 0.01f), "SRO slope estimate matches input");
}

void test_sync_fsm() {
    std::printf("[sync_fsm]\n");
    SyncFSM fsm;
    CHECK(fsm.state() == SyncState::Searching, "FSM starts in Searching");
    for (int i = 0; i < 10; ++i) fsm.feed(true, 0.9f, 0.001f);
    CHECK(fsm.state() == SyncState::Locked, "FSM reaches Locked with good frames");
    for (int i = 0; i < 20; ++i) fsm.feed(false, 0.0f, 1.0f);
    CHECK(fsm.state() != SyncState::Locked, "FSM leaves Locked on bad frames");
}

void test_amc() {
    std::printf("[amc]\n");
    AMCConfig acfg;
    acfg.ema_alpha_snr = 1.0f;          // no smoothing — react instantly for the test
    acfg.min_dwell_frames = 0;
    AMCSelector amc(acfg);
    CHECK(!amc.table().empty(), "AMC builds threshold table");

    // Drive several iterations so EMA stabilizes and dwell counter clears
    AMCEntry rec{};
    for (int i = 0; i < 4; ++i)
        rec = amc.recommend(25.f, Modulation::QPSK, FECRate::Rate_1_2);
    CHECK(rec.spectral_eff_bps_hz > 1.0f, "AMC recommends high-SE mode at high SNR");

    for (int i = 0; i < 4; ++i)
        rec = amc.recommend(-3.f, Modulation::QAM64, FECRate::Rate_3_4);
    CHECK(true, "AMC accepts low-SNR input");

    // #27: the recommendation must be the MAX spectral-efficiency feasible
    // mode, not merely the highest-threshold one (threshold and SE aren't
    // co-monotonic). Verify against an independent scan of the table.
    {
        const float snr = 14.f;
        AMCEntry r{};
        for (int i = 0; i < 4; ++i)
            r = amc.recommend(snr, Modulation::QPSK, FECRate::Rate_1_2);
        float max_se = 0.f;
        for (const auto& e : amc.table())
            if (e.threshold_db + acfg.up_margin_db <= snr)
                max_se = std::max(max_se, e.spectral_eff_bps_hz);
        CHECK(std::abs(r.spectral_eff_bps_hz - max_se) < 1e-3f,
              "AMC picks the max-throughput feasible mode (not max-threshold)");
    }

    // #4 (SOTA): OLLA bias rises on failures, falls on successes.
    {
        AMCConfig oc;
        oc.olla_enabled = true;
        oc.olla_target_bler = 0.05f;
        oc.olla_step_db = 0.2f;
        AMCSelector olla(oc);
        for (int i = 0; i < 10; ++i) olla.reportFrameResult(false);
        float after_fail = olla.ollaBiasDb();
        CHECK(after_fail > 0.5f, "OLLA bias rises after repeated failures");
        for (int i = 0; i < 500; ++i) olla.reportFrameResult(true);
        CHECK(olla.ollaBiasDb() < after_fail,
              "OLLA bias falls back after sustained successes");
    }
}

// Recovery tests for the MIMO detectors that previously had ZERO coverage
// (#64/#65): 2×2 Alamouti STBC, MRC, ZF and MMSE spatial-mux detectors.
void test_mimo_detectors() {
    std::printf("[mimo_detectors]\n");
    auto close = [](ComplexSample a, ComplexSample b, float tol) {
        return std::abs(a - b) < tol;
    };

    // --- 2×2 Alamouti STBC: transmit (s0,s1) over a non-trivial 2×2 H,
    //     form the received vectors per the Alamouti scheme, decode. ---
    {
        ComplexSample s0(0.7071f, 0.7071f), s1(-0.7071f, 0.7071f);
        std::array<std::array<ComplexSample, 2>, 2> H = {{
            {{ ComplexSample(0.8f, 0.2f), ComplexSample(0.3f, -0.5f) }},
            {{ ComplexSample(-0.4f, 0.6f), ComplexSample(0.9f, 0.1f) }}
        }};
        // Alamouti over two symbol times, per RX antenna r:
        //   t0: y0_r = h_r0·s0 + h_r1·s1
        //   t1: y1_r = -h_r0·conj(s1) + h_r1·conj(s0)
        std::array<ComplexSample, 2> y0, y1;
        for (size_t r = 0; r < 2; ++r) {
            y0[r] = H[r][0]*s0 + H[r][1]*s1;
            y1[r] = -H[r][0]*std::conj(s1) + H[r][1]*std::conj(s0);
        }
        ComplexSample d0, d1;
        alamoutiDecode2x2(y0, y1, H, d0, d1);
        CHECK(close(d0, s0, 1e-4f) && close(d1, s1, 1e-4f),
              "Alamouti 2x2 recovers both symbols over non-trivial H");
    }

    // --- MRC (1×3 SIMO): y_r = h_r·s; combiner must recover s. ---
    {
        ComplexSample s(0.5f, -0.5f);
        ComplexSample h[3] = { {0.6f,0.1f}, {-0.2f,0.7f}, {0.4f,-0.3f} };
        ComplexSample y[3];
        for (int r = 0; r < 3; ++r) y[r] = h[r]*s;
        CHECK(close(mrcCombine(y, h, 3), s, 1e-4f),
              "MRC 1x3 recovers the transmitted symbol");
    }

    // --- ZF 2×2 spatial mux on a well-conditioned H. ---
    std::array<std::array<ComplexSample, 2>, 2> H = {{
        {{ ComplexSample(1.0f, 0.0f), ComplexSample(0.3f, 0.2f) }},
        {{ ComplexSample(0.1f, -0.4f), ComplexSample(0.9f, 0.1f) }}
    }};
    std::array<ComplexSample, 2> x = {{ ComplexSample(1.f, 0.f),
                                        ComplexSample(-1.f, 0.5f) }};
    std::array<ComplexSample, 2> y;
    y[0] = H[0][0]*x[0] + H[0][1]*x[1];
    y[1] = H[1][0]*x[0] + H[1][1]*x[1];
    {
        auto xh = zfDetect2x2(y, H);
        CHECK(close(xh[0], x[0], 1e-3f) && close(xh[1], x[1], 1e-3f),
              "ZF 2x2 recovers both streams (well-conditioned H)");
    }
    // --- MMSE 2×2: at ~0 noise it should match ZF closely. ---
    {
        auto xh = mmseDetect2x2(y, H, 1e-6f);
        CHECK(close(xh[0], x[0], 1e-2f) && close(xh[1], x[1], 1e-2f),
              "MMSE 2x2 ≈ ZF at near-zero noise");
    }
    // --- Singular H: both detectors must return zeros, not NaN/inf. ---
    {
        std::array<std::array<ComplexSample, 2>, 2> Hs = {{
            {{ ComplexSample(1.f,0.f), ComplexSample(2.f,0.f) }},
            {{ ComplexSample(2.f,0.f), ComplexSample(4.f,0.f) }}  // rows linearly dependent
        }};
        std::array<ComplexSample, 2> ys = {{ ComplexSample(1.f,0.f), ComplexSample(2.f,0.f) }};
        auto zf = zfDetect2x2(ys, Hs);
        bool finite = std::isfinite(zf[0].real()) && std::isfinite(zf[1].real());
        CHECK(finite, "ZF 2x2 singular-H guard returns finite (no NaN/inf)");
    }
}

void test_alamouti() {
    std::printf("[alamouti]\n");
    ComplexSample s0(0.7071f, 0.7071f);
    ComplexSample s1(-0.7071f, 0.7071f);
    auto p = alamoutiEncode(s0, s1);
    // Channel: pure passes
    ComplexSample h0(1.f, 0.f), h1(1.f, 0.f);
    ComplexSample y0 = h0 * p.slot0[0] + h1 * p.slot0[1];
    ComplexSample y1 = h0 * p.slot1[0] + h1 * p.slot1[1];
    ComplexSample s0r, s1r;
    alamoutiDecode2x1(y0, y1, h0, h1, s0r, s1r);
    CHECK(NEAR(s0r.real(), s0.real(), 0.01f) && NEAR(s0r.imag(), s0.imag(), 0.01f),
          "Alamouti recovers s0");
    CHECK(NEAR(s1r.real(), s1.real(), 0.01f) && NEAR(s1r.imag(), s1.imag(), 0.01f),
          "Alamouti recovers s1");
}

void test_mimo_pipeline() {
    std::printf("[mimo_pipeline]\n");
    OFDMParams p;
    p.fft_size = 64;
    p.modulation = Modulation::QPSK;
    p.sample_rate = 48000;

    MIMOMod tx(p);
    std::vector<uint8_t> bits(64, 0xA5);
    std::array<ComplexBuf, 2> ant_streams;
    tx.modulate(bits.data(), bits.size() * 8, ant_streams);
    CHECK(ant_streams[0].size() > 0 && ant_streams[1].size() > 0,
          "MIMO modulator emits two equal-length streams");
    CHECK(ant_streams[0].size() == ant_streams[1].size(),
          "Antenna streams are time-aligned");

    // Build a trivial channel: H[0][0]=1, H[0][1]=1, H[1][0]=1, H[1][1]=1
    // So both antennas combine constructively at the single RX.
    auto alloc = computeAllocation(p);
    ComplexBuf flat_h(p.fft_size, ComplexSample(1.0f, 0.f));
    std::vector<std::array<ComplexBuf, 2>> H_ant(1);
    H_ant[0][0] = flat_h;
    H_ant[0][1] = flat_h;

    // Pass through perfect channel: y_r[t] = H[r][0]·x0[t] + H[r][1]·x1[t]
    size_t sym_len = p.symbolLength();
    std::vector<ComplexBuf> y0_ant(1), y1_ant(1);
    y0_ant[0].assign(sym_len, ComplexSample(0.f, 0.f));
    y1_ant[0].assign(sym_len, ComplexSample(0.f, 0.f));
    for (size_t i = 0; i < sym_len; ++i) {
        y0_ant[0][i] = ant_streams[0][i] + ant_streams[1][i];
        y1_ant[0][i] = ant_streams[0][i + sym_len] + ant_streams[1][i + sym_len];
    }

    MIMODemod rx(p);
    ComplexBuf recovered;
    bool ok = rx.demodulatePair(y0_ant, y1_ant, H_ant, recovered);
    CHECK(ok && recovered.size() == 2 * alloc.dataCount(),
          "MIMO demodulator produces 2× data symbols");
}

#ifdef DSCA_ENABLE_AUDIO
void test_hw_audio_query() {
    std::printf("[hw_audio_query]\n");
    HWAudioDevice dev;
    if (!dev.init()) {
        std::printf("  SKIP no audio context (CI/headless)\n");
        return;
    }
    // Exercise the same code path the device dialog uses on Apply,
    // including out-of-range and "default" indices. None should crash.
    auto rates_default = dev.supportedSampleRates(-1, -1);
    CHECK(true, "supportedSampleRates(-1,-1) returns without crash");

    auto rates_out_of_range = dev.supportedSampleRates(99, 99);
    CHECK(rates_out_of_range.empty(),
          "supportedSampleRates with bad indices returns empty");

    if (!dev.playbackDevices().empty() && !dev.captureDevices().empty()) {
        auto r = dev.supportedSampleRates(0, 0);
        CHECK(true, "supportedSampleRates(0,0) returns without crash");
        (void)r;
    } else {
        std::printf("  no real devices; skipping (0,0) probe\n");
    }
}
#endif

void test_multi_stream() {
    std::printf("[multi_stream]\n");
    MultiStreamCoordinator coord;
    StreamConfig cfg;
    cfg.enabled     = true;
    cfg.sample_rate = 48000;
    cfg.channels    = 1;
    cfg.bitrate_bps = 24000;
    cfg.frame_ms    = 20;
    coord.configureStream(0, cfg);
    coord.configureStream(1, cfg);
    CHECK(coord.activeCount() == 2, "Two streams enabled");

    // Push some PCM, encode into a frame, ensure packets are queued
    std::vector<float> pcm(960, 0.1f);
    coord.pushTX(0, pcm.data(), pcm.size());
    coord.pushTX(1, pcm.data(), pcm.size());
    FrameBuilder fb(2048);
    coord.encodeIntoFrame(fb);
    CHECK(fb.usedBytes() > 0, "Multi-stream produces packets");
}

// Exercises the previously-untested RX half of MultiStreamCoordinator
// (onParsedFrames, Mid/Side recombine rings, Opus FEC recovery) and locks
// in the fix for the Mid/Side ring-desync family: for a stereo stream the
// Mid and Side output rings must stay sample-for-sample frame-locked
// regardless of whether the transmitted Side is present, missing, or the
// Mid frame was FEC-recovered.
void test_multi_stream_rx() {
    std::printf("[multi_stream_rx]\n");
    MultiStreamCoordinator coord;
    StreamConfig cfg;
    cfg.enabled     = true;
    cfg.sample_rate = 48000;
    cfg.channels    = 2;          // stereo → Mid/Side split
    cfg.bitrate_bps = 64000;
    cfg.frame_ms    = 20;
    coord.configureStream(0, cfg);

    const size_t frame_samples = 48000 * 20 / 1000; // 960

    // Distinct, non-trivial L/R so Side ≠ 0 and DTX doesn't suppress the
    // packet (a constant/silent frame would be dropped by Opus DTX).
    auto pushStereoFrame = [&]() {
        std::vector<float> lr(frame_samples * 2);
        for (size_t i = 0; i < frame_samples; ++i) {
            float t = static_cast<float>(i);
            lr[2 * i + 0] = 0.3f * std::sin(0.05f * t);             // L
            lr[2 * i + 1] = 0.2f * std::sin(0.09f * t + 0.5f);      // R
        }
        coord.pushTX(0, lr.data(), lr.size());
    };

    auto parse = [](const ByteVec& bytes, ParsedFrame& pf) {
        FrameParser::parse(bytes, pf);
    };

    // --- Case 1: Mid + Side both present → rings frame-locked ---
    {
        pushStereoFrame();
        FrameBuilder mfb(4096), sfb(4096);
        coord.encodeIntoFrames(mfb, &sfb, /*hier_active=*/true);
        ParsedFrame mpf, spf;
        parse(mfb.build(1, FECRate::Rate_1_2, Modulation::QPSK), mpf);
        parse(sfb.build(1, FECRate::Rate_1_2, Modulation::QPSK), spf);
        coord.onParsedFrames(mpf, &spf);
        CHECK(coord.rxAvailable(0) > 0, "RX: Mid ring received samples");
        CHECK(coord.rxAvailable(0) == coord.sideRxAvailable(0),
              "RX: Mid+Side present → rings frame-locked");
    }

    // --- Case 2: Mid present, Side absent → silence-filled, still locked ---
    {
        pushStereoFrame();
        FrameBuilder mfb(4096);
        coord.encodeIntoFrame(mfb);   // Mid only (no Side frame)
        ParsedFrame mpf;
        parse(mfb.build(2, FECRate::Rate_1_2, Modulation::QPSK), mpf);
        coord.onParsedFrames(mpf, nullptr);   // no Side at all
        CHECK(coord.rxAvailable(0) == coord.sideRxAvailable(0),
              "RX: Side-missing → rings stay frame-locked (silence fill)");
    }

    // --- Case 3: Mid loss then FEC recovery on the next frame ---
    {
        coord.onFrameLost();          // mark stream lost
        pushStereoFrame();
        FrameBuilder mfb(4096), sfb(4096);
        coord.encodeIntoFrames(mfb, &sfb, /*hier_active=*/true);
        ParsedFrame mpf, spf;
        parse(mfb.build(3, FECRate::Rate_1_2, Modulation::QPSK), mpf);
        parse(sfb.build(3, FECRate::Rate_1_2, Modulation::QPSK), spf);
        coord.onParsedFrames(mpf, &spf);
        CHECK(coord.rxAvailable(0) == coord.sideRxAvailable(0),
              "RX: FEC-recovered Mid frame → rings stay frame-locked");
    }
}

} // anonymous

int main() {
    std::printf("\n=== DSP Primitives Test (Phase 1-5) ===\n");

    test_dc_blocker();
    test_nco();
    test_iq_balance();
    test_polyphase_decimator();
    test_cic_decimator();
    test_power_ramp();
    test_squelch();
    test_zadoff_chu();
    test_integer_cfo();
    test_phase_tracker();
    test_sro();
    test_sync_fsm();
    test_amc();
    test_alamouti();
    test_mimo_detectors();
    test_mimo_pipeline();
#ifdef DSCA_ENABLE_AUDIO
    test_hw_audio_query();
#endif
    test_multi_stream();
    test_multi_stream_rx();

    std::printf("\n%d passed, %d failed\n", g_passed, g_failed);
    return g_failed == 0 ? 0 : 1;
}
