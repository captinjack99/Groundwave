/**
 * @file ofdm_chain_test.cpp
 * @brief Bisect the OFDM transmit/receive chain to find where bit errors
 *        come from on a clean channel.
 *
 * Stages, each tested independently:
 *
 *   A. Interleaver round-trip (bytes in == bytes out)
 *   B. Symbol mapper hard-demap round-trip (bits → symbols → hard bits)
 *   C. Symbol mapper soft-demap → hard threshold round-trip
 *   D. OFDM modulator → demodulator round-trip on clean channel
 *
 * Once we know which stage drops bits, the fix is targeted.
 */
#include "interleaver.hpp"
#include "symbol_mapper.hpp"
#include "ofdm.hpp"
#include "ldpc.hpp"
#include "snr_calculator.hpp"   // modulationName

#include <cstdio>
#include <random>
#include <vector>
#include <algorithm>

using namespace dsca;

namespace {
inline bool getBit(const uint8_t* d, size_t i) {
    return (d[i >> 3] >> (7 - (i & 7))) & 1;
}
inline void setBit(uint8_t* d, size_t i, bool v) {
    if (v) d[i >> 3] |=  static_cast<uint8_t>(1u << (7 - (i & 7)));
    else   d[i >> 3] &= ~static_cast<uint8_t>(1u << (7 - (i & 7)));
}

int countBitDiff(const uint8_t* a, const uint8_t* b, size_t bits) {
    int errs = 0;
    for (size_t i = 0; i < bits; ++i) {
        if (getBit(a, i) != getBit(b, i)) ++errs;
    }
    return errs;
}

// =========================================================================
// Stage A: Interleaver
// =========================================================================
int testInterleaver(size_t bits) {
    std::mt19937 rng(0x100 ^ static_cast<uint32_t>(bits));
    std::vector<uint8_t> in((bits + 7) / 8, 0);
    for (auto& b : in) b = static_cast<uint8_t>(rng() & 0xFF);
    BitInterleaver inter(bits);
    std::vector<uint8_t> mid(in.size(), 0);
    inter.interleave(in.data(), mid.data());
    std::vector<uint8_t> out(in.size(), 0);
    inter.deinterleave(mid.data(), out.data());
    return countBitDiff(in.data(), out.data(), bits);
}

// =========================================================================
// Stage B: Symbol mapper hard round-trip
// =========================================================================
int testHardMapper(Modulation mod, size_t bits) {
    std::mt19937 rng(0x200 ^ static_cast<uint32_t>(mod));
    SymbolMapper m(mod);
    size_t bps = m.bitsPerSymbol();
    bits = (bits / bps) * bps;        // align to symbol
    std::vector<uint8_t> in((bits + 7) / 8, 0);
    for (auto& b : in) b = static_cast<uint8_t>(rng() & 0xFF);

    ComplexBuf syms;
    m.mapBytes(in.data(), bits, syms);

    std::vector<uint8_t> out((bits + 7) / 8, 0);
    for (size_t s = 0; s < syms.size(); ++s) {
        uint16_t idx = m.demapHard(syms[s]);
        for (size_t b = 0; b < bps; ++b) {
            bool v = (idx >> (bps - 1 - b)) & 1;
            setBit(out.data(), s * bps + b, v);
        }
    }
    return countBitDiff(in.data(), out.data(), bits);
}

// =========================================================================
// Stage C: Symbol mapper soft round-trip (LLR → hard threshold)
// =========================================================================
int testSoftMapper(Modulation mod, size_t bits) {
    std::mt19937 rng(0x300 ^ static_cast<uint32_t>(mod));
    SymbolMapper m(mod);
    size_t bps = m.bitsPerSymbol();
    bits = (bits / bps) * bps;
    std::vector<uint8_t> in((bits + 7) / 8, 0);
    for (auto& b : in) b = static_cast<uint8_t>(rng() & 0xFF);
    ComplexBuf syms;
    m.mapBytes(in.data(), bits, syms);

    std::vector<float> llrs;
    m.demapSoft(syms, /*noise_var=*/0.01f, llrs);
    // Convention: positive LLR ⇒ bit 0
    std::vector<uint8_t> out((bits + 7) / 8, 0);
    for (size_t i = 0; i < bits && i < llrs.size(); ++i) {
        setBit(out.data(), i, llrs[i] < 0.f);
    }
    return countBitDiff(in.data(), out.data(), bits);
}

// =========================================================================
// Stage D: OFDM mod → demod round-trip (no preamble, no AWGN)
// =========================================================================
int testOFDM(Modulation mod, uint16_t fft, uint32_t sr, size_t info_bytes) {
    OFDMParams p;
    p.fft_size = fft;
    p.modulation = mod;
    p.sample_rate = sr;
    p.cyclic_prefix = CyclicPrefix::CP_1_8;

    OFDMModulator txm(p);
    OFDMDemodulator rxm(p);

    size_t bps = bitsPerSymbol(mod);
    size_t bits_per_ofdm = txm.bitsPerOFDMSymbol();
    if (bits_per_ofdm == 0) return -1;

    std::mt19937 rng(0x400 ^ static_cast<uint32_t>(mod) ^ fft);
    std::vector<uint8_t> in(info_bytes, 0);
    for (auto& b : in) b = static_cast<uint8_t>(rng() & 0xFF);
    // Round bit count down to a multiple of bps so mapBytes doesn't drop the
    // trailing partial-symbol bits (otherwise we'd compare against bits that
    // were never modulated and falsely report errors).
    size_t total_bits = (info_bytes * 8 / bps) * bps;

    // Send through preamble first so the demod has a channel estimate.
    ComplexBuf preamble = txm.generatePreamble();
    ComplexBuf payload;
    txm.modulateBits(in.data(), total_bits, payload);

    // Process preamble: extract two long syms after the short-training block
    size_t short_total = 10 * (fft / 4);
    size_t sym_len = p.symbolLength();
    if (preamble.size() < short_total + 2 * sym_len) return -1;
    ComplexBuf long_syms(preamble.begin() + static_cast<ptrdiff_t>(short_total),
                          preamble.begin() + static_cast<ptrdiff_t>(short_total) + 2 * sym_len);
    if (!rxm.processPreamble(long_syms)) return -1;

    // Demod each OFDM symbol of payload, then hard-threshold to bits.
    std::vector<uint8_t> out((total_bits + 7) / 8, 0);
    size_t bit_idx = 0;
    for (size_t off = 0; off + sym_len <= payload.size() && bit_idx < total_bits;
         off += sym_len) {
        ComplexBuf one(payload.begin() + static_cast<ptrdiff_t>(off),
                        payload.begin() + static_cast<ptrdiff_t>(off + sym_len));
        ComplexBuf eq;
        if (!rxm.demodulate(one, eq)) break;
        SymbolMapper sm(mod);
        for (auto& s : eq) {
            uint16_t idx = sm.demapHard(s);
            for (size_t b = 0; b < bps && bit_idx < total_bits; ++b) {
                bool v = (idx >> (bps - 1 - b)) & 1;
                setBit(out.data(), bit_idx++, v);
            }
        }
    }
    return countBitDiff(in.data(), out.data(), total_bits);
}

} // anonymous

// =========================================================================
// Stage E: Full TX→RX with LDPC + OFDM, measure clean-channel BER
// (mirrors integration_test::runCycle but with intermediate diagnostics)
// =========================================================================
struct StageEResult {
    int bit_errors;
    int unsat_pre_decode;   // unsatisfied parity checks BEFORE LDPC decode
    float noise_var_estimate;
    bool decoder_converged;
};

StageEResult testFullChain(Modulation mod, FECRate fec, uint16_t fft, uint32_t sr) {
    StageEResult r{0, 0, 0.f, false};

    OFDMParams p;
    p.fft_size = fft;
    p.modulation = mod;
    p.sample_rate = sr;
    p.cyclic_prefix = CyclicPrefix::CP_1_8;

    LDPCEncoder enc(fec, LDPCBlockSize::Short);
    LDPCDecoder dec(fec, LDPCBlockSize::Short, 50);
    BitInterleaver inter(enc.codewordBits());
    OFDMModulator txm(p);
    OFDMDemodulator rxm(p);

    std::mt19937 rng(0x500 ^ static_cast<uint32_t>(mod) ^ static_cast<uint32_t>(fec));
    std::vector<uint8_t> info((enc.infoBits() + 7) / 8, 0);
    for (auto& b : info) b = static_cast<uint8_t>(rng() & 0xFF);
    std::vector<uint8_t> cw((enc.codewordBits() + 7) / 8, 0);
    enc.encode(info.data(), cw.data());
    std::vector<uint8_t> ileaved(cw.size(), 0);
    inter.interleave(cw.data(), ileaved.data());

    ComplexBuf preamble = txm.generatePreamble();
    ComplexBuf payload;
    txm.modulateBits(ileaved.data(), enc.codewordBits(), payload);

    // Process preamble
    size_t short_total = 10 * (fft / 4);
    size_t sym_len = p.symbolLength();
    ComplexBuf long_syms(preamble.begin() + static_cast<ptrdiff_t>(short_total),
                          preamble.begin() + static_cast<ptrdiff_t>(short_total) + 2 * sym_len);
    rxm.processPreamble(long_syms);
    r.noise_var_estimate = rxm.noiseVariance();

    // Demod payload
    size_t bits_per_ofdm = txm.bitsPerOFDMSymbol();
    size_t syms_per_cw = (enc.codewordBits() + bits_per_ofdm - 1) / bits_per_ofdm;
    std::vector<float> all_llrs;
    for (size_t s = 0; s < syms_per_cw; ++s) {
        size_t off = s * sym_len;
        if (off + sym_len > payload.size()) break;
        ComplexBuf one(payload.begin() + static_cast<ptrdiff_t>(off),
                        payload.begin() + static_cast<ptrdiff_t>(off + sym_len));
        std::vector<float> llrs;
        rxm.demodulateSoft(one, llrs, r.noise_var_estimate);
        all_llrs.insert(all_llrs.end(), llrs.begin(), llrs.end());
    }
    all_llrs.resize(enc.codewordBits(), 0.f);

    std::vector<float> deint(all_llrs.size());
    inter.deinterleave(all_llrs.data(), deint.data());

    // Pre-decode hard decisions: how many bits already mismatch the original codeword?
    std::vector<uint8_t> hard((enc.codewordBits() + 7) / 8, 0);
    for (size_t i = 0; i < enc.codewordBits(); ++i) {
        setBit(hard.data(), i, deint[i] < 0.f);
    }
    int pre_bit_errs = 0;
    for (size_t i = 0; i < enc.codewordBits(); ++i) {
        if (getBit(hard.data(), i) != getBit(cw.data(), i)) ++pre_bit_errs;
    }
    r.unsat_pre_decode = pre_bit_errs;

    // Decode
    std::vector<uint8_t> out((enc.infoBits() + 7) / 8, 0);
    auto dr = dec.decode(deint.data(), out.data());
    r.decoder_converged = dr.converged;
    for (size_t i = 0; i < enc.infoBits(); ++i) {
        if (getBit(info.data(), i) != getBit(out.data(), i)) ++r.bit_errors;
    }
    return r;
}

int main() {
    std::printf("=== A: interleaver round-trip ===\n");
    for (size_t b : {64u, 270u, 540u, 1080u, 2160u, 2700u}) {
        int e = testInterleaver(b);
        std::printf("  %5zu bits: %d errors  %s\n", b, e, e == 0 ? "OK" : "FAIL");
    }

    std::printf("\n=== B: symbol mapper hard round-trip ===\n");
    for (auto m : {Modulation::BPSK, Modulation::QPSK, Modulation::QAM16,
                    Modulation::QAM64, Modulation::QAM256}) {
        int e = testHardMapper(m, 2160);
        std::printf("  %-7s: %d errors  %s\n", modulationName(m), e,
                    e == 0 ? "OK" : "FAIL");
    }

    std::printf("\n=== C: symbol mapper soft → hard round-trip ===\n");
    for (auto m : {Modulation::BPSK, Modulation::QPSK, Modulation::QAM16,
                    Modulation::QAM64, Modulation::QAM256}) {
        int e = testSoftMapper(m, 2160);
        std::printf("  %-7s: %d errors  %s\n", modulationName(m), e,
                    e == 0 ? "OK" : "FAIL");
    }

    std::printf("\n=== D: OFDM mod → demod round-trip ===\n");
    for (auto m : {Modulation::BPSK, Modulation::QPSK, Modulation::QAM16,
                    Modulation::QAM64, Modulation::QAM256}) {
        int e = testOFDM(m, 256, 48000, 64);
        std::printf("  %-7s FFT=256: %d errors  %s\n", modulationName(m), e,
                    e == 0 ? "OK" : "FAIL");
    }

    std::printf("\n=== E: full chain (FEC + interleave + OFDM) ===\n");
    Modulation mods[] = { Modulation::BPSK, Modulation::QPSK, Modulation::QAM16,
                           Modulation::QAM64, Modulation::QAM256 };
    FECRate fecs[] = { FECRate::Rate_1_4, FECRate::Rate_1_2,
                        FECRate::Rate_3_4, FECRate::Rate_9_10 };
    for (auto m : mods) {
        for (auto f : fecs) {
            auto rs = testFullChain(m, f, 256, 48000);
            std::printf("  %-7s / %-5s : pre-decode err=%d  post-decode err=%d  "
                        "noise_var=%.2e  conv=%d  %s\n",
                        modulationName(m),
                        [&]{ switch(f) {
                            case FECRate::Rate_1_4: return "1/4";
                            case FECRate::Rate_1_2: return "1/2";
                            case FECRate::Rate_3_4: return "3/4";
                            case FECRate::Rate_9_10: return "9/10";
                            default: return "??"; } }(),
                        rs.unsat_pre_decode, rs.bit_errors,
                        rs.noise_var_estimate, rs.decoder_converged ? 1 : 0,
                        rs.bit_errors == 0 ? "OK" : "FAIL");
        }
    }
    return 0;
}
