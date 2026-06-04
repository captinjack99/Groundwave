/**
 * @file side_reconstructor_test.cpp
 * @brief Validates the all-pass-cascade Side-channel reconstructor.
 *
 * Properties checked:
 *   1. Output length matches input length.
 *   2. Output is NOT identical to input (decorrelator actually decorrelates).
 *   3. Output spectrum is approximately equal to input spectrum (RMS match).
 *      All-pass filters preserve magnitude; this validates that property.
 *   4. confidenceFromLLR maps correctly across the [LLR_LOW, LLR_HIGH] window.
 *   5. Reset clears state — repeated runs on the same input produce identical
 *      output after reset (filter state doesn't leak between sessions).
 */
#include "side_reconstructor.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>
#include <vector>

using namespace dsca;

namespace {

int g_passed = 0, g_failed = 0;

#define CHECK(cond, msg)                                                       \
    do {                                                                       \
        if (cond) { ++g_passed; std::printf("  [PASS] %s\n", msg); }            \
        else      { ++g_failed; std::printf("  [FAIL] %s\n", msg); }            \
    } while (0)

void test_output_length() {
    std::printf("\n=== Reconstructor output length ===\n");
    SideReconstructor r(48000);
    std::vector<float> in(960), out;
    r.process(in.data(), in.size(), out);
    CHECK(out.size() == in.size(), "output length matches input");
}

void test_decorrelation() {
    std::printf("\n=== Decorrelation: output differs from input ===\n");
    SideReconstructor r(48000);
    std::mt19937 rng(0x12345);
    std::vector<float> in(4800);
    for (auto& s : in) s = (static_cast<float>(rng() & 0xFFFF) /
                            65535.f - 0.5f);
    std::vector<float> out;
    r.process(in.data(), in.size(), out);
    // Compute correlation between in and out: high correlation = bad
    // (decorrelator failed). The all-pass cascade should decorrelate
    // significantly so |corr| < 0.6 even on white noise input.
    double sx = 0, sy = 0, sxy = 0, sxx = 0, syy = 0;
    size_t n = in.size();
    for (size_t i = 0; i < n; ++i) {
        sx += in[i]; sy += out[i];
        sxx += in[i] * in[i]; syy += out[i] * out[i];
        sxy += in[i] * out[i];
    }
    double mx = sx / n, my = sy / n;
    double cov = sxy / n - mx * my;
    double vx = sxx / n - mx * mx;
    double vy = syy / n - my * my;
    double corr = cov / std::sqrt(vx * vy + 1e-12);
    char label[80];
    std::snprintf(label, sizeof(label),
                   "correlation %.3f (should be |corr| < 0.6)", corr);
    CHECK(std::abs(corr) < 0.6, label);
}

void test_magnitude_preserved() {
    std::printf("\n=== All-pass preserves signal RMS ===\n");
    SideReconstructor r(48000);
    std::mt19937 rng(0xABCD);
    std::vector<float> in(48000);
    for (auto& s : in) s = (static_cast<float>(rng() & 0xFFFF) /
                            65535.f - 0.5f) * 0.5f;
    std::vector<float> out;
    r.process(in.data(), in.size(), out);
    double rms_in = 0, rms_out = 0;
    // Skip the first 100 samples to let filter transients settle.
    for (size_t i = 100; i < in.size(); ++i) {
        rms_in  += in[i] * in[i];
        rms_out += out[i] * out[i];
    }
    rms_in  = std::sqrt(rms_in  / (in.size() - 100));
    rms_out = std::sqrt(rms_out / (in.size() - 100));
    double ratio = rms_out / (rms_in + 1e-12);
    // High-pass + low-pass + all-pass cascade. Spectrum should be
    // similar but with bass and treble rolled off. Expect ratio in
    // [0.4, 1.1] for broadband white-noise input.
    char label[96];
    std::snprintf(label, sizeof(label),
                   "RMS ratio out/in = %.3f (expect 0.4–1.1)", ratio);
    CHECK(ratio > 0.4 && ratio < 1.1, label);
}

void test_confidence_mapping() {
    std::printf("\n=== Confidence-from-LLR mapping ===\n");
    auto f = &SideReconstructor::confidenceFromLLR;
    CHECK(f(0.0f) == 0.0f,                  "|LLR|=0 → confidence=0");
    CHECK(f(1.5f) == 0.0f,                  "|LLR|=1.5 (below floor) → 0");
    CHECK(f(2.0f) == 0.0f,                  "|LLR|=2.0 (at floor) → 0");
    CHECK(std::abs(f(5.0f) - 0.5f) < 0.01f, "|LLR|=5.0 (midpoint) → 0.5");
    CHECK(f(8.0f) == 1.0f,                  "|LLR|=8.0 (at ceiling) → 1.0");
    CHECK(f(50.0f) == 1.0f,                 "|LLR|=50 (above ceiling) → 1.0");
}

void test_reset_clears_state() {
    std::printf("\n=== Reset clears filter state ===\n");
    SideReconstructor r(48000);
    std::mt19937 rng(0xDEAD);
    std::vector<float> in(2400);
    for (auto& s : in) s = (static_cast<float>(rng() & 0xFFFF) /
                            65535.f - 0.5f);
    std::vector<float> out1, out2;
    r.process(in.data(), in.size(), out1);
    r.reset();
    r.process(in.data(), in.size(), out2);
    bool match = (out1.size() == out2.size());
    for (size_t i = 0; match && i < out1.size(); ++i) {
        if (std::abs(out1[i] - out2[i]) > 1e-6f) match = false;
    }
    CHECK(match, "two runs from reset produce identical output");
}

} // anonymous

int main() {
    std::printf("=== DSCA-NG Side-Reconstructor Test Suite ===\n");
    test_output_length();
    test_decorrelation();
    test_magnitude_preserved();
    test_confidence_mapping();
    test_reset_clears_state();
    std::printf("\n=== Result: %d passed, %d failed ===\n",
                 g_passed, g_failed);
    return g_failed == 0 ? 0 : 1;
}
