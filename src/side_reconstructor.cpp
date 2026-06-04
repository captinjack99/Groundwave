/**
 * @file side_reconstructor.cpp
 * @brief All-pass-cascade Side-channel synthesizer for hierarchical
 *        graceful-degradation recovery.
 */
#include "side_reconstructor.hpp"
#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace dsca {

SideReconstructor::SideReconstructor(uint32_t sample_rate)
    : sample_rate_(sample_rate)
{
    // All-pass stages: 6 log-spaced center frequencies from 250 Hz to
    // 6 kHz. The choice covers the perceptually-critical stereo-image
    // band (300 Hz — 4 kHz) plus a low-end stage to handle voice and a
    // high-end stage to dress up the upper formants.
    static constexpr float kFreqs[NUM_AP_STAGES] = {
        250.0f, 500.0f, 1000.0f, 2000.0f, 3500.0f, 6000.0f
    };
    static constexpr float kQ = 0.7f;
    for (size_t i = 0; i < NUM_AP_STAGES; ++i) {
        designAllPass(ap_stages_[i], kFreqs[i], kQ);
    }
    designHighpass(hp_, 80.0f);
    designLowpass(lp_,  12000.0f);
}

void SideReconstructor::process(const float* mid_pcm, size_t n_samples,
                                 std::vector<float>& side_out) {
    side_out.resize(n_samples);
    for (size_t i = 0; i < n_samples; ++i) {
        float x = mid_pcm[i];
        // Band-limit input to the perceptual stereo-relevant band.
        x = hp_.process(x);
        // Cascade through all-pass stages — phase decorrelation only.
        for (auto& ap : ap_stages_) x = ap.process(x);
        // Final low-pass to suppress any high-frequency artifacts from
        // the all-pass cascade and band-limit synthesized stereo.
        x = lp_.process(x);
        side_out[i] = x;
    }
}

void SideReconstructor::reset() {
    for (auto& ap : ap_stages_) ap.reset();
    hp_.reset();
    lp_.reset();
}

float SideReconstructor::confidenceFromLLR(float avg_llr_mag) {
    // Tuned for the engine's 25-iter normalized min-sum BP decoder:
    //   |LLR| < 2.0  → decoder genuinely unsure, fall back to synth.
    //   |LLR| > 8.0  → decoder very confident, transmitted Side trusted.
    //   In between, smooth linear taper.
    constexpr float LLR_LOW  = 2.0f;
    constexpr float LLR_HIGH = 8.0f;
    if (avg_llr_mag <= LLR_LOW)  return 0.0f;
    if (avg_llr_mag >= LLR_HIGH) return 1.0f;
    return (avg_llr_mag - LLR_LOW) / (LLR_HIGH - LLR_LOW);
}

// =========================================================================
// Biquad coefficient design — RBJ Audio EQ Cookbook formulas.
// =========================================================================

void SideReconstructor::designAllPass(Biquad& bq, float f0_hz, float q) const {
    float fs = static_cast<float>(sample_rate_);
    if (fs <= 0.f) fs = 48000.f;
    float w0 = 2.0f * static_cast<float>(M_PI) * f0_hz / fs;
    float cw = std::cos(w0);
    float sw = std::sin(w0);
    float alpha = sw / (2.0f * q);
    float a0 = 1.0f + alpha;
    bq.b0 = (1.0f - alpha) / a0;
    bq.b1 = -2.0f * cw / a0;
    bq.b2 = 1.0f;          // (1 + α) / (1 + α) = 1 after normalization
    bq.a1 = bq.b1;          // all-pass: a coefs = reverse of b coefs
    bq.a2 = bq.b0;
    bq.reset();
}

void SideReconstructor::designHighpass(Biquad& bq, float f0_hz) const {
    float fs = static_cast<float>(sample_rate_);
    if (fs <= 0.f) fs = 48000.f;
    float w0 = 2.0f * static_cast<float>(M_PI) * f0_hz / fs;
    float cw = std::cos(w0);
    float sw = std::sin(w0);
    float q = 0.707f;          // Butterworth = sqrt(2)/2
    float alpha = sw / (2.0f * q);
    float a0 = 1.0f + alpha;
    bq.b0 =  (1.0f + cw) / (2.0f * a0);
    bq.b1 = -(1.0f + cw) / a0;
    bq.b2 =  (1.0f + cw) / (2.0f * a0);
    bq.a1 = -2.0f * cw / a0;
    bq.a2 = (1.0f - alpha) / a0;
    bq.reset();
}

void SideReconstructor::designLowpass(Biquad& bq, float f0_hz) const {
    float fs = static_cast<float>(sample_rate_);
    if (fs <= 0.f) fs = 48000.f;
    float w0 = 2.0f * static_cast<float>(M_PI) * f0_hz / fs;
    float cw = std::cos(w0);
    float sw = std::sin(w0);
    float q = 0.707f;
    float alpha = sw / (2.0f * q);
    float a0 = 1.0f + alpha;
    bq.b0 = (1.0f - cw) / (2.0f * a0);
    bq.b1 = (1.0f - cw) / a0;
    bq.b2 = (1.0f - cw) / (2.0f * a0);
    bq.a1 = -2.0f * cw / a0;
    bq.a2 = (1.0f - alpha) / a0;
    bq.reset();
}

} // namespace dsca
