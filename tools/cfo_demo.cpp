/**
 * @file cfo_demo.cpp
 * @brief Carrier-frequency-offset (CFO) before/after demonstration for the
 *        Groundwave receive path. Sweeps a set of CFO values and reports info-bit
 *        errors for two receiver configurations on an otherwise clean channel:
 *
 *   UNCORRECTED — the historical engine that estimated the fractional CFO but
 *       discarded it. Reproduced faithfully here by applying the carrier
 *       rotation only to the PAYLOAD samples, leaving the two long preamble
 *       symbols offset-free. The Moose estimator then measures ~0 Hz and
 *       performs no derotation, so the payload constellation spins symbol to
 *       symbol exactly as it did before the fix.
 *
 *   CORRECTED  — the current default path. The carrier rotation is applied to
 *       the WHOLE stream (preamble included). The two identical long-preamble
 *       symbols let the Moose estimator recover the offset and the baseband is
 *       derotated before every FFT, so the payload is decoded error-free across
 *       the fractional-CFO capture range.
 *
 * Both curves are MEASURED end-to-end through the same encoder/mapper/OFDM/
 * demapper/LDPC pipeline used by the integration suite (Test 11). This mirrors
 * that test but additionally emits the uncorrected reference so the regression
 * is quantified, not merely asserted to be zero.
 *
 * Usage:
 *   ./cfo_demo                 64-QAM 2/3 default sweep
 *   ./cfo_demo --mod 16qam     16-QAM 1/2
 *   ./cfo_demo --mod qpsk      QPSK 1/2
 */
#include "types.hpp"
#include "ofdm.hpp"
#include "ldpc.hpp"
#include "interleaver.hpp"
#include "frame.hpp"
#include "snr_calculator.hpp"

#include <cmath>
#include <complex>
#include <cstdio>
#include <cstring>
#include <random>
#include <string>
#include <vector>

using namespace gw;

namespace {

// Run one encode->channel->decode cycle and return info-bit errors.
// If preamble_offset is false the CFO rotation is applied to the payload
// samples only (preamble left clean) -> reproduces the discarded-offset path.
int runCFO(Modulation mod, FECRate fec, float cfo_hz, bool preamble_offset,
           uint32_t seed = 0x1234u) {
    const uint16_t fft_size = 256;
    const uint32_t sample_rate = 48000;

    OFDMParams ofdm;
    ofdm.fft_size = fft_size;
    ofdm.modulation = mod;
    ofdm.sample_rate = sample_rate;
    ofdm.cyclic_prefix = CyclicPrefix::CP_1_8;

    LDPCEncoder ldpc_enc(fec, LDPCBlockSize::Short);
    LDPCDecoder ldpc_dec(fec, LDPCBlockSize::Short, 25);
    BitInterleaver inter(ldpc_enc.codewordBits());

    OFDMModulator tx_mod(ofdm);
    OFDMDemodulator rx_demod(ofdm);
    rx_demod.setPerBinLLRWeighting(true);

    const size_t k_bytes = ldpc_enc.infoBytes();
    const size_t n_bytes = ldpc_enc.codewordBytes();

    std::mt19937 rng(seed);
    const size_t cap = (k_bytes > constants::FRAME_OVERHEAD)
                       ? k_bytes - constants::FRAME_OVERHEAD : 0;
    if (cap < 8) return -1;
    FrameBuilder fb(cap);
    std::vector<uint8_t> packet_data(cap - 4, 0);
    for (auto& b : packet_data) b = static_cast<uint8_t>(rng() & 0xFF);
    fb.addPacket(0, packet_data.data(), packet_data.size());
    auto frame_bytes = fb.build(/*frame_no*/ 1, fec, mod);
    frame_bytes.resize(k_bytes, 0);

    std::vector<uint8_t> codeword(n_bytes, 0);
    if (!ldpc_enc.encode(frame_bytes.data(), codeword.data())) return -1;
    std::vector<uint8_t> ileaved(n_bytes, 0);
    inter.interleave(codeword.data(), ileaved.data());

    ComplexBuf tx_bb;
    tx_mod.modulateBits(ileaved.data(), ldpc_enc.codewordBits(), tx_bb);
    ComplexBuf preamble = tx_mod.generatePreamble();

    // Channel = preamble + payload.
    ComplexBuf channel = preamble;
    channel.insert(channel.end(), tx_bb.begin(), tx_bb.end());

    // Apply the carrier rotation. When preamble_offset is true the whole stream
    // rotates and Moose recovers it; when false only the payload rotates so the
    // estimator sees a clean preamble and leaves the spin uncorrected.
    if (cfo_hz != 0.f) {
        const float w = 2.f * static_cast<float>(M_PI) * cfo_hz /
                        static_cast<float>(sample_rate);
        const size_t pre_n = preamble.size();
        for (size_t n = 0; n < channel.size(); ++n) {
            if (!preamble_offset && n < pre_n) continue;
            // Phase continuous in n so the payload sees the same NCO it would on
            // a real offset carrier.
            channel[n] *= std::polar(1.f, w * static_cast<float>(n));
        }
    }

    // RX: preamble then per-symbol soft demap.
    const size_t sym_len = ofdm.symbolLength();
    const size_t short_total = 10 * (ofdm.fft_size / 4);
    const size_t pre_long_off = short_total;
    if (channel.size() < pre_long_off + 2 * sym_len) return -1;
    ComplexBuf long_syms(channel.begin() + pre_long_off,
                         channel.begin() + pre_long_off + 2 * sym_len);
    if (!rx_demod.processPreamble(long_syms)) return -1;

    const size_t data_off = preamble.size();
    const size_t bits_per_ofdm = tx_mod.bitsPerOFDMSymbol();
    if (bits_per_ofdm == 0) return -1;
    const size_t syms_per_cw =
        (ldpc_enc.codewordBits() + bits_per_ofdm - 1) / bits_per_ofdm;

    std::vector<float> all_llrs;
    for (size_t s = 0; s < syms_per_cw; ++s) {
        const size_t off = data_off + s * sym_len;
        if (off + sym_len > channel.size()) break;
        ComplexBuf one(channel.begin() + off, channel.begin() + off + sym_len);
        std::vector<float> llrs;
        rx_demod.demodulateSoft(one, llrs, rx_demod.noiseVariance());
        all_llrs.insert(all_llrs.end(), llrs.begin(), llrs.end());
    }
    all_llrs.resize(ldpc_enc.codewordBits(), 0.f);

    std::vector<float> deint(all_llrs.size());
    inter.deinterleave(all_llrs.data(), deint.data());

    std::vector<uint8_t> decoded(k_bytes, 0);
    ldpc_dec.decode(deint.data(), decoded.data());

    int bit_errs = 0;
    const size_t info_bits = ldpc_enc.infoBits();
    for (size_t i = 0; i < info_bits; ++i) {
        bool a = (decoded[i >> 3] >> (7 - (i & 7))) & 1;
        bool b = (frame_bytes[i >> 3] >> (7 - (i & 7))) & 1;
        if (a != b) ++bit_errs;
    }
    return bit_errs;
}

} // namespace

int main(int argc, char** argv) {
    Modulation mod = Modulation::QAM64;
    FECRate fec = FECRate::Rate_2_3;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--mod") == 0 && i + 1 < argc) {
            std::string m = argv[++i];
            if (m == "qpsk") { mod = Modulation::QPSK; fec = FECRate::Rate_1_2; }
            else if (m == "16qam") { mod = Modulation::QAM16; fec = FECRate::Rate_1_2; }
            else if (m == "64qam") { mod = Modulation::QAM64; fec = FECRate::Rate_2_3; }
        }
    }

    const float cfos[] = {0.f, 5.f, 10.f, 20.f, 40.f, 60.f, 80.f};

    std::printf("=== Groundwave CFO before/after (clean channel) ===\n");
    std::printf("mod = %s  fec = %s   subcarrier spacing = 187.5 Hz\n",
                modulationName(mod), fecRateName(fec));
    std::printf("info-bit errors vs carrier-frequency offset\n\n");
    std::printf("   CFO(Hz) | uncorrected | corrected\n");
    std::printf("   --------+-------------+----------\n");
    for (float c : cfos) {
        int un = runCFO(mod, fec, c, /*preamble_offset=*/false);
        int co = runCFO(mod, fec, c, /*preamble_offset=*/true);
        std::printf("   %6.0f  | %11d | %9d\n", c, un, co);
    }
    std::printf("\nDone.\n");
    return 0;
}
