/**
 * @file cic.hpp
 * @brief Cascaded Integrator-Comb (CIC) decimator and interpolator.
 *
 * Multiplier-free filter for very high (≥ 8) decimation/interpolation ratios.
 * Stages: N integrators at high rate, rate change R, N comb filters at low
 * rate with delay M (typically 1 or 2).
 *
 * DC gain of decimator = (R·M)^N — divide by this to keep unit gain.
 *
 * Use a small FIR compensator (CICCompensator) afterward to flatten the
 * passband droop if you need flatness within the passband.
 */
#pragma once

#include "types.hpp"
#include <vector>
#include <cmath>
#include <cstdint>
#include <algorithm>
#include <memory>

namespace dsca {

class CICDecimator {
public:
    /** @param R   Decimation factor (≥ 1).
     *  @param N   Number of stages (typically 3–5).
     *  @param M   Differential delay (typically 1 or 2). */
    CICDecimator(uint32_t R, uint32_t N = 3, uint32_t M = 1)
        : R_(R ? R : 1), N_(N ? N : 1), M_(M ? M : 1)
    {
        integrators_.assign(N_, 0);
        comb_history_.assign(N_, std::vector<int64_t>(M_, 0));
        gain_ = std::pow(static_cast<double>(R_ * M_), static_cast<double>(N_));
    }

    size_t process(const float* in, size_t n_in, float* out) {
        size_t n_out = 0;
        const float scale = static_cast<float>(1.0 / gain_);
        for (size_t i = 0; i < n_in; ++i) {
            // Convert to integer Q15 to keep the math integer-only
            int64_t s = static_cast<int64_t>(in[i] * 32768.0f);
            // N integrators
            for (uint32_t k = 0; k < N_; ++k) {
                integrators_[k] += s;
                s = integrators_[k];
            }
            // Decimate by R
            ++count_;
            if (count_ == R_) {
                count_ = 0;
                int64_t v = integrators_[N_ - 1];
                // N comb filters with delay M
                for (uint32_t k = 0; k < N_; ++k) {
                    int64_t past = comb_history_[k][comb_idx_];
                    int64_t cur  = v;
                    v = cur - past;
                    comb_history_[k][comb_idx_] = cur;
                }
                comb_idx_ = (comb_idx_ + 1) % M_;
                out[n_out++] = (static_cast<float>(v) / 32768.0f) * scale;
            }
        }
        return n_out;
    }

    size_t process(const ComplexSample* in, size_t n_in, ComplexSample* out) {
        // Run two parallel decimators on the I/Q components
        std::vector<float> ti(n_in), tq(n_in);
        for (size_t i = 0; i < n_in; ++i) { ti[i] = in[i].real(); tq[i] = in[i].imag(); }
        std::vector<float> oi(n_in / R_ + 2), oq(n_in / R_ + 2);
        size_t ni = process(ti.data(), n_in, oi.data());
        // Save state, restart for Q. Since CICDecimator holds state, we'd corrupt it.
        // Use a side instance for Q to keep states independent.
        if (!q_partner_) q_partner_ = std::make_unique<CICDecimator>(R_, N_, M_);
        size_t nq = q_partner_->process(tq.data(), n_in, oq.data());
        size_t n = std::min(ni, nq);
        for (size_t i = 0; i < n; ++i) out[i] = ComplexSample(oi[i], oq[i]);
        return n;
    }

    void reset() {
        std::fill(integrators_.begin(), integrators_.end(), 0);
        for (auto& h : comb_history_) std::fill(h.begin(), h.end(), 0);
        comb_idx_ = 0;
        count_ = 0;
        if (q_partner_) q_partner_->reset();
    }

    uint32_t R() const { return R_; }
    uint32_t N() const { return N_; }
    uint32_t M() const { return M_; }

private:
    uint32_t R_, N_, M_;
    std::vector<int64_t> integrators_;
    std::vector<std::vector<int64_t>> comb_history_;
    uint32_t comb_idx_ = 0;
    uint32_t count_    = 0;
    double gain_       = 1.0;
    std::unique_ptr<CICDecimator> q_partner_; ///< For complex-input dual-decimation
};

} // namespace dsca
