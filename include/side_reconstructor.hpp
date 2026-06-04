/**
 * @file side_reconstructor.hpp
 * @brief Predictive Side-channel reconstruction from Mid for hierarchical
 *        graceful-degradation.
 *
 * In M/S over hierarchical modulation, the LP layer carries Side and the
 * HP layer carries Mid. When LP fails (LDPC decoder doesn't converge or
 * CRC bad) we lose the Side audio for that frame. Naive recovery is to
 * play silence on the Side channel — the listener hears stereo collapse
 * abruptly to mono. This module synthesizes a plausible Side signal from
 * the (still-decoded) Mid, so the stereo image degrades smoothly to a
 * decorrelated-but-coherent fake-stereo rather than cliff-falling to mono.
 *
 * Algorithm:
 *
 *   1. Cascaded 2nd-order all-pass biquads. Each all-pass has unit
 *      magnitude response (preserves spectrum exactly) but applies a
 *      frequency-dependent phase shift. Six stages log-spaced over
 *      250 Hz — 6 kHz at Q=0.7 produce a Mid-derived signal that is
 *      spectrally indistinguishable from Mid but temporally decorrelated.
 *      The ear interprets this as natural stereo width.
 *
 *   2. Perceptual band-limiting via Butterworth HP at 80 Hz and LP at
 *      12 kHz. Below 80 Hz, stereo information is perceptually invisible
 *      (the head can't localize bass); above 12 kHz, the codec rarely
 *      carries meaningful content. Both edges keep the synthesized Side
 *      from sounding artificial.
 *
 *   3. LP-LLR-confidence-driven crossfade is applied by the caller, not
 *      by this class. This class only produces the synthesized Side; the
 *      AudioEngine mixes it with the transmitted Side using the LDPC
 *      decoder's `avg_magnitude` as the confidence weight.
 *
 * Each instance keeps per-cascade filter state across calls (necessary
 * for IIR filters to behave coherently across frame boundaries). The
 * caller must keep the same instance alive across consecutive frames.
 */
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace dsca {

class SideReconstructor {
public:
    static constexpr size_t NUM_AP_STAGES = 6;

    explicit SideReconstructor(uint32_t sample_rate);

    /** Run one frame of Mid PCM through the all-pass cascade plus
     *  band-limiting filters to produce a synthesized Side. Output is
     *  resized to match input length. Filter state persists across
     *  calls (IIR filters require continuity for clean response). */
    void process(const float* mid_pcm, size_t n_samples,
                 std::vector<float>& side_out);

    /** Reset all filter state to zero. Call when the stream restarts
     *  (preset change, sync loss recovery) to avoid leftover transients
     *  from previous program material. */
    void reset();

    /** Convert an LDPC posterior LLR magnitude into a [0, 1] confidence
     *  weight for the transmitted-Side path. Output:
     *    - confidence ≈ 1 when |LLR| is high (LP decoder very sure) →
     *      transmitted Side plays unmodified.
     *    - confidence ≈ 0 when |LLR| is low or LP failed → synthesized
     *      Side takes over.
     *    - Smooth taper across the [LLR_LOW, LLR_HIGH] window so there's
     *      no audible click at the handoff.
     *
     *  Map: confidence(|LLR|) = clamp((|LLR| − LLR_LOW) /
     *                                  (LLR_HIGH − LLR_LOW), 0, 1).
     *  Default window is tuned for the engine's max_iter=25 BP decoder. */
    static float confidenceFromLLR(float avg_llr_mag);

private:
    // 2nd-order biquad in Direct Form II Transposed. For an all-pass
    // with center frequency f0 and Q:
    //   ω0 = 2π·f0/fs,  α = sin(ω0)/(2Q)
    //   b0 = (1−α)/(1+α),  b1 = −2cos(ω0)/(1+α),  b2 = 1
    //   a1 = b1,           a2 = b0
    struct Biquad {
        float b0 = 1.f, b1 = 0.f, b2 = 0.f;
        float a1 = 0.f, a2 = 0.f;
        float z1 = 0.f, z2 = 0.f;

        inline float process(float x) {
            float y  = b0 * x + z1;
            z1 = b1 * x + z2 - a1 * y;
            z2 = b2 * x       - a2 * y;
            return y;
        }
        void reset() { z1 = 0.f; z2 = 0.f; }
    };

    /** Design a 2nd-order all-pass biquad at the given center frequency
     *  and Q. Writes coefficients into `bq`. */
    void designAllPass(Biquad& bq, float f0_hz, float q) const;

    /** Design a Butterworth 2nd-order high-pass at f0_hz. */
    void designHighpass(Biquad& bq, float f0_hz) const;

    /** Design a Butterworth 2nd-order low-pass at f0_hz. */
    void designLowpass(Biquad& bq, float f0_hz) const;

    uint32_t sample_rate_;

    // Cascaded all-pass stages — the decorrelator core.
    std::array<Biquad, NUM_AP_STAGES> ap_stages_;

    // Band-limit filters before and after the all-pass cascade.
    Biquad hp_;  // 80 Hz high-pass (kill bass — stereo invisible there)
    Biquad lp_;  // 12 kHz low-pass (band-limit synthetic stereo)
};

} // namespace dsca
