/**
 * @file mimo_pipeline.hpp
 * @brief 2×N MIMO OFDM modulator and demodulator using Alamouti STBC.
 *
 * Provides a thin wrapper around the existing OFDMModulator / OFDMDemodulator
 * that turns each pair of consecutive OFDM-mapped data symbols into an
 * Alamouti pair across two virtual TX antennas. Two independent OFDM
 * symbol streams are generated (one per antenna).
 *
 * Receive side: given two received OFDM symbols on each RX antenna and a
 * 2×nR channel estimate per subcarrier, recovers the original symbol pair
 * via Alamouti combining. Achieves rate-1 with full transmit diversity.
 *
 * Loopback support: in `complex_loopback` mode the soundcard modem skips
 * IQ conversion. We can simulate a 2-antenna channel in software by just
 * applying two channel coefficients per subcarrier and adding the streams.
 * This makes the pipeline testable without multi-channel hardware.
 */
#pragma once

#include "types.hpp"
#include "ofdm.hpp"
#include "mimo.hpp"
#include <array>
#include <memory>
#include <vector>

namespace gw {

class MIMOMod {
public:
    explicit MIMOMod(const OFDMParams& params)
        : p_(params)
        , ofdm_a_(std::make_unique<OFDMModulator>(params))
        , ofdm_b_(std::make_unique<OFDMModulator>(params))
        , mapper_(params.modulation) {}

    /** Encode `bits` for two-antenna transmission. Output: passband samples
     *  for each antenna (`out[0]` = ant0, `out[1]` = ant1). Each antenna's
     *  samples have the same length and are time-aligned. */
    void modulate(const uint8_t* bits, size_t num_bits,
                  std::array<ComplexBuf, 2>& out) {
        // 1. Map bits to symbols
        ComplexBuf syms;
        mapper_.mapBytes(bits, num_bits, syms);
        if (syms.size() % 2 != 0) syms.push_back(ComplexSample(0.f, 0.f));

        // 2. Build two Alamouti streams: stream A gets s0/-conj(s1), stream B
        //    gets s1/conj(s0) at consecutive symbol slots.
        ComplexBuf stream_a, stream_b;
        stream_a.reserve(syms.size());
        stream_b.reserve(syms.size());
        for (size_t i = 0; i + 1 < syms.size(); i += 2) {
            ComplexSample s0 = syms[i];
            ComplexSample s1 = syms[i + 1];
            // Slot t:    A=s0,        B=s1
            // Slot t+1:  A=-conj(s1), B=conj(s0)
            stream_a.push_back(s0);
            stream_b.push_back(s1);
            stream_a.push_back(-std::conj(s1));
            stream_b.push_back(std::conj(s0));
        }

        // 3. Run each stream through its own OFDMModulator
        ofdm_a_->modulateSymbols(stream_a.data(), stream_a.size(), out[0]);
        ofdm_b_->modulateSymbols(stream_b.data(), stream_b.size(), out[1]);
    }

    OFDMModulator& streamA() { return *ofdm_a_; }
    OFDMModulator& streamB() { return *ofdm_b_; }

    void reset() {
        ofdm_a_->reset();
        ofdm_b_->reset();
    }

    const OFDMParams& params() const { return p_; }

private:
    OFDMParams                       p_;
    std::unique_ptr<OFDMModulator>   ofdm_a_;
    std::unique_ptr<OFDMModulator>   ofdm_b_;
    SymbolMapper                     mapper_;
};

/** 2×nR Alamouti receiver. Operates per-subcarrier on the FFT outputs of
 *  pairs of consecutive OFDM symbols received on each RX antenna. */
class MIMODemod {
public:
    explicit MIMODemod(const OFDMParams& params)
        : p_(params)
        , ofdm_a_(std::make_unique<OFDMDemodulator>(params))
        , ofdm_b_(std::make_unique<OFDMDemodulator>(params))
        , mapper_(params.modulation) {}

    /** Demodulate one pair of OFDM symbols received simultaneously on two
     *  RX antennas (anywhere from 1×2 SIMO to 2×2 MIMO).
     *
     *  @param y0_ant   Received samples for slot 0 — one entry per RX antenna,
     *                  each of length symbolLength().
     *  @param y1_ant   Received samples for slot 1.
     *  @param H_ant    Per-antenna channel estimate matrix.
     *                  H_ant[r][t][k] = H from TX antenna t to RX antenna r at subcarrier k.
     *                  Outer dim = nR, inner = nT (always 2 for Alamouti).
     *  @param out      Recovered data symbols (size = 2 × dataCount()).
     *  @return true on success.
     */
    bool demodulatePair(const std::vector<ComplexBuf>& y0_ant,
                        const std::vector<ComplexBuf>& y1_ant,
                        const std::vector<std::array<ComplexBuf, 2>>& H_ant,
                        ComplexBuf& out) {
        if (y0_ant.size() != y1_ant.size() || y0_ant.size() != H_ant.size())
            return false;
        if (y0_ant.empty()) return false;
        size_t nR = y0_ant.size();

        // FFT each antenna's samples for both slots
        size_t N = p_.fft_size;
        size_t cp = p_.cpLength();
        FFTEngine fft(N);

        std::vector<ComplexBuf> Y0(nR, ComplexBuf(N));
        std::vector<ComplexBuf> Y1(nR, ComplexBuf(N));
        for (size_t r = 0; r < nR; ++r) {
            ComplexBuf in0(y0_ant[r].begin() + static_cast<ptrdiff_t>(cp),
                           y0_ant[r].begin() + static_cast<ptrdiff_t>(cp + N));
            ComplexBuf in1(y1_ant[r].begin() + static_cast<ptrdiff_t>(cp),
                           y1_ant[r].begin() + static_cast<ptrdiff_t>(cp + N));
            fft.forward(in0, Y0[r]);
            fft.forward(in1, Y1[r]);
        }

        // Build allocation to know which subcarriers carry data
        auto alloc = computeAllocation(p_);
        out.clear();
        out.reserve(alloc.dataCount() * 2);

        // For each data subcarrier, run Alamouti combining
        for (size_t i = 0; i < alloc.dataCount(); ++i) {
            size_t k = alloc.data_indices[i];
            std::array<ComplexSample, 2> y0v, y1v;
            std::array<std::array<ComplexSample, 2>, 2> H;
            // Use up to 2 RX antennas for the standard 2x2 path; if nR=1, use 2x1.
            if (nR >= 2) {
                for (size_t r = 0; r < 2; ++r) {
                    y0v[r] = Y0[r][k];
                    y1v[r] = Y1[r][k];
                    H[r][0] = H_ant[r][0][k];
                    H[r][1] = H_ant[r][1][k];
                }
                ComplexSample s0, s1;
                alamoutiDecode2x2(y0v, y1v, H, s0, s1);
                out.push_back(s0);
                out.push_back(s1);
            } else {
                // 2×1 SIMO/Alamouti: use single RX antenna
                ComplexSample s0, s1;
                alamoutiDecode2x1(Y0[0][k], Y1[0][k],
                                  H_ant[0][0][k], H_ant[0][1][k],
                                  s0, s1);
                out.push_back(s0);
                out.push_back(s1);
            }
        }
        return true;
    }

    /** Soft-demap recovered symbols to LLRs using the configured modulation. */
    void demapSoft(const ComplexBuf& syms, float noise_var,
                   std::vector<float>& llrs) const {
        mapper_.demapSoft(syms, noise_var, llrs);
    }

    OFDMDemodulator& streamA() { return *ofdm_a_; }
    OFDMDemodulator& streamB() { return *ofdm_b_; }
    const OFDMParams& params() const { return p_; }

private:
    OFDMParams                          p_;
    std::unique_ptr<OFDMDemodulator>    ofdm_a_;
    std::unique_ptr<OFDMDemodulator>    ofdm_b_;
    SymbolMapper                        mapper_;
};

} // namespace gw
