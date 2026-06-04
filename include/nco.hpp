/**
 * @file nco.hpp
 * @brief Numerically Controlled Oscillator with table-based sine/cosine.
 *
 * Single-precision NCO using a 4096-entry quarter-wave sine table with
 * linear interpolation. ~25–60× faster than std::sin/std::cos depending on
 * the platform, with peak error < 1e-4 over the unit circle. Phase is held
 * as uint32_t (30.32 fixed point on the unit circle), giving exact wrap.
 *
 * Use TableNCO for tight inner loops where you would have called sin/cos.
 * For non-real-time uses (e.g. setup), std::sin is fine.
 */
#pragma once

#include "types.hpp"
#include <array>
#include <cmath>
#include <cstdint>

namespace dsca {

namespace nco_detail {

// Quarter-wave table built once at namespace-init time. Storing only one
// quadrant + symmetry gives 4× memory win versus a full-period table.
constexpr size_t TABLE_BITS = 12;
constexpr size_t TABLE_SIZE = 1u << TABLE_BITS;       // 4096
constexpr size_t TABLE_MASK = TABLE_SIZE - 1;

struct SineTable {
    std::array<float, TABLE_SIZE + 1> data;
    SineTable() {
        // Quarter-wave: [0, π/2] mapped to [0, TABLE_SIZE]
        for (size_t i = 0; i <= TABLE_SIZE; ++i) {
            double t = static_cast<double>(i) / static_cast<double>(TABLE_SIZE);
            data[i] = static_cast<float>(std::sin(t * 0.5 * M_PI));
        }
    }
};

inline const SineTable& table() {
    static SineTable t;
    return t;
}

// sin(2π * (phase / 2³²)) using quarter-wave symmetry
inline float sinFromPhase32(uint32_t phase) {
    const auto& t = table();
    // Top 2 bits = quadrant; next TABLE_BITS bits = index into quarter-wave.
    uint32_t quadrant = phase >> 30;
    uint32_t inner    = (phase >> (30 - TABLE_BITS)) & TABLE_MASK;
    uint32_t frac_bits = 30 - TABLE_BITS;
    uint32_t frac     = phase & ((1u << frac_bits) - 1);
    float f = static_cast<float>(frac) / static_cast<float>(1u << frac_bits);

    float s0, s1;
    switch (quadrant) {
        case 0: s0 = t.data[inner];                 s1 = t.data[inner + 1];                 break;
        case 1: s0 = t.data[TABLE_SIZE - inner];    s1 = t.data[TABLE_SIZE - inner - 1];    break;
        case 2: s0 = -t.data[inner];                s1 = -t.data[inner + 1];                break;
        default:s0 = -t.data[TABLE_SIZE - inner];   s1 = -t.data[TABLE_SIZE - inner - 1];   break;
    }
    return s0 + (s1 - s0) * f;
}

inline float cosFromPhase32(uint32_t phase) {
    return sinFromPhase32(phase + 0x40000000u); // +π/2
}

} // namespace nco_detail

class TableNCO {
public:
    TableNCO() = default;

    /** Set frequency in Hz given sample rate. */
    void setFrequency(double freq_hz, double sample_rate) {
        double normalized = freq_hz / sample_rate;
        // Wrap into [-0.5, 0.5)
        normalized -= std::floor(normalized + 0.5);
        // 2³² phase units per cycle
        double inc_d = normalized * 4294967296.0;
        phase_inc_ = static_cast<int32_t>(inc_d);
    }

    /** Set phase in radians. */
    void setPhase(double phase_rad) {
        double t = phase_rad / (2.0 * M_PI);
        t -= std::floor(t);
        phase_ = static_cast<uint32_t>(t * 4294967296.0);
    }

    void reset() { phase_ = 0; }

    /** Generate next complex exponential sample exp(j·θ). */
    ComplexSample step() {
        ComplexSample s(nco_detail::cosFromPhase32(phase_),
                        nco_detail::sinFromPhase32(phase_));
        phase_ += static_cast<uint32_t>(phase_inc_);
        return s;
    }

    /** Generate next pair (cos, sin). */
    void stepRealImag(float& c, float& s) {
        c = nco_detail::cosFromPhase32(phase_);
        s = nco_detail::sinFromPhase32(phase_);
        phase_ += static_cast<uint32_t>(phase_inc_);
    }

    /** Mix a complex baseband signal up to passband (real output).
     *  out[n] = re(in[n] · exp(j·θ_n)). */
    void mixUp(const ComplexSample* in, float* out, size_t n) {
        for (size_t i = 0; i < n; ++i) {
            float c, s;
            stepRealImag(c, s);
            out[i] = c * in[i].real() - s * in[i].imag();
        }
    }

    /** Mix a passband signal down to complex baseband.
     *  out[n] = in[n] · exp(-j·θ_n) = in[n] · (cos - j·sin). */
    void mixDown(const float* in, ComplexSample* out, size_t n) {
        for (size_t i = 0; i < n; ++i) {
            float c, s;
            stepRealImag(c, s);
            out[i] = ComplexSample(in[i] * c, -in[i] * s);
        }
    }

    uint32_t phase() const { return phase_; }
    int32_t  phaseIncrement() const { return phase_inc_; }

private:
    uint32_t phase_     = 0;
    int32_t  phase_inc_ = 0;
};

} // namespace dsca
