/**
 * @file fec_test.cpp
 * @brief FEC (LDPC + Interleaver) tests
 *
 * Tests:
 *  1. LDPC encode/decode round-trip (clean)
 *  2. LDPC error correction capability
 *  3. Interleaver round-trip (bits and soft values)
 *  4. LDPC + interleaver + OFDM integrated chain
 *  5. Coding gain measurement: uncoded vs coded BER at multiple SNRs
 */

#include "types.hpp"
#include "ldpc.hpp"
#include "orbgrand.hpp"
#include "soft_decoder.hpp"
#include "bicm.hpp"
#include "symbol_mapper.hpp"
#include "interleaver.hpp"
#include "ofdm.hpp"
#include <cstdio>
#include <cstring>
#include <cmath>
#include <random>
#include <vector>
#include <algorithm>
#include <numeric>

using namespace gw;

static int tests_passed = 0;
static int tests_failed = 0;

#define TEST(name) printf("  %-60s", name)
#define PASS() do { printf("[PASS]\n"); tests_passed++; } while(0)
#define FAIL(msg) do { printf("[FAIL] %s\n", msg); tests_failed++; } while(0)

namespace {

inline bool getBit(const uint8_t* d, size_t i) {
    return (d[i >> 3] >> (7 - (i & 7))) & 1;
}

size_t countBitDiffs(const uint8_t* a, const uint8_t* b, size_t num_bits) {
    size_t errors = 0;
    for (size_t i = 0; i < num_bits; ++i) {
        if (getBit(a, i) != getBit(b, i)) errors++;
    }
    return errors;
}

void flipRandomBits(uint8_t* data, size_t num_bits, size_t num_flips,
                    std::mt19937& rng) {
    std::uniform_int_distribution<size_t> dist(0, num_bits - 1);
    for (size_t i = 0; i < num_flips; ++i) {
        size_t pos = dist(rng);
        data[pos >> 3] ^= (uint8_t(1) << (7 - (pos & 7)));
    }
}

void addAWGN(ComplexBuf& samples, float snr_db, std::mt19937& rng) {
    float sig_power = 0.f;
    for (auto& s : samples) sig_power += std::norm(s);
    sig_power /= static_cast<float>(samples.size());

    float snr_lin = std::pow(10.f, snr_db / 10.f);
    float sigma = std::sqrt(sig_power / (2.f * snr_lin));

    std::normal_distribution<float> dist(0.f, sigma);
    for (auto& s : samples) {
        s += ComplexSample(dist(rng), dist(rng));
    }
}

} // anonymous

int main() {
    printf("=== Groundwave v2 FEC Test Suite ===\n\n");

    // =====================================================================
    // Test 1: LDPC encode/decode round-trip (no errors)
    // =====================================================================
    for (auto rate : {FECRate::Rate_1_2, FECRate::Rate_2_3, FECRate::Rate_3_4}) {
        const char* names[] = {"1/2", "2/3", "3/4"};
        int ri = (rate == FECRate::Rate_1_2) ? 0 :
                 (rate == FECRate::Rate_2_3) ? 1 : 2;
        char name[64];
        snprintf(name, sizeof(name), "LDPC R=%s short: clean round-trip", names[ri]);
        TEST(name);

        LDPCEncoder enc(rate, LDPCBlockSize::Short);
        LDPCDecoder dec(rate, LDPCBlockSize::Short, 50);

        size_t k = enc.infoBits();
        size_t n = enc.codewordBits();
        size_t k_bytes = (k + 7) / 8;
        size_t n_bytes = (n + 7) / 8;

        // Random info bits
        std::mt19937 rng(42 + ri);
        std::vector<uint8_t> info(k_bytes, 0);
        for (auto& b : info) b = static_cast<uint8_t>(rng() & 0xFF);

        // Encode
        std::vector<uint8_t> codeword(n_bytes, 0);
        enc.encode(info.data(), codeword.data());

        // Convert to LLRs (perfect channel: bit 0 → +1.0, bit 1 → -1.0)
        std::vector<float> llr(n);
        for (size_t i = 0; i < n; ++i) {
            llr[i] = getBit(codeword.data(), i) ? -1.0f : 1.0f;
        }

        // Decode
        std::vector<uint8_t> decoded(k_bytes, 0);
        auto result = dec.decode(llr.data(), decoded.data());

        size_t bit_errs = countBitDiffs(info.data(), decoded.data(), k);

        if (bit_errs == 0) {
            PASS();
        } else {
            char msg[64];
            snprintf(msg, sizeof(msg), "%zu bit errors (converged=%d, iter=%zu)",
                     bit_errs, result.converged ? 1 : 0, result.iterations);
            FAIL(msg);
        }
    }

    // =====================================================================
    // Test 2: LDPC error correction (inject bit errors)
    // =====================================================================
    TEST("LDPC R=1/2 short: correct 30 random bit errors");
    {
        LDPCEncoder enc(FECRate::Rate_1_2, LDPCBlockSize::Short);
        LDPCDecoder dec(FECRate::Rate_1_2, LDPCBlockSize::Short, 100);

        size_t k = enc.infoBits();
        size_t n = enc.codewordBits();
        size_t k_bytes = (k + 7) / 8;
        size_t n_bytes = (n + 7) / 8;

        std::mt19937 rng(123);
        std::vector<uint8_t> info(k_bytes, 0);
        for (auto& b : info) b = static_cast<uint8_t>(rng() & 0xFF);

        std::vector<uint8_t> codeword(n_bytes, 0);
        enc.encode(info.data(), codeword.data());

        // Inject errors
        std::vector<uint8_t> noisy = codeword;
        flipRandomBits(noisy.data(), n, 30, rng);

        // Verify errors were injected
        size_t injected = countBitDiffs(codeword.data(), noisy.data(), n);

        // Convert to LLRs
        std::vector<float> llr(n);
        for (size_t i = 0; i < n; ++i) {
            llr[i] = getBit(noisy.data(), i) ? -2.0f : 2.0f;
        }

        // Decode
        std::vector<uint8_t> decoded(k_bytes, 0);
        auto result = dec.decode(llr.data(), decoded.data());

        size_t remaining = countBitDiffs(info.data(), decoded.data(), k);

        if (remaining == 0 && result.converged) {
            PASS();
        } else {
            char msg[96];
            snprintf(msg, sizeof(msg),
                     "%zu injected, %zu remaining after %zu iter (converged=%d)",
                     injected, remaining, result.iterations,
                     result.converged ? 1 : 0);
            FAIL(msg);
        }
    }

    // =====================================================================
    // Test 3: Interleaver round-trip
    // =====================================================================
    TEST("Interleaver: bit round-trip");
    {
        size_t n = 2160;
        BitInterleaver intlv(n);

        std::mt19937 rng(42);
        size_t n_bytes = (n + 7) / 8;
        std::vector<uint8_t> orig(n_bytes);
        for (auto& b : orig) b = static_cast<uint8_t>(rng() & 0xFF);

        std::vector<uint8_t> interleaved(n_bytes, 0);
        std::vector<uint8_t> recovered(n_bytes, 0);

        intlv.interleave(orig.data(), interleaved.data());
        intlv.deinterleave(interleaved.data(), recovered.data());

        if (countBitDiffs(orig.data(), recovered.data(), n) == 0) {
            PASS();
        } else {
            FAIL("bits changed after interleave + deinterleave");
        }
    }

    TEST("Interleaver: LLR round-trip");
    {
        size_t n = 2160;
        BitInterleaver intlv(n);

        std::mt19937 rng(99);
        std::normal_distribution<float> dist(0.f, 5.f);

        std::vector<float> orig(n);
        for (auto& v : orig) v = dist(rng);

        std::vector<float> interleaved(n);
        std::vector<float> recovered(n);

        intlv.interleave(orig.data(), interleaved.data());
        intlv.deinterleave(interleaved.data(), recovered.data());

        float max_err = 0.f;
        for (size_t i = 0; i < n; ++i) {
            max_err = std::max(max_err, std::abs(orig[i] - recovered[i]));
        }

        if (max_err < 1e-6f) {
            PASS();
        } else {
            char msg[64];
            snprintf(msg, sizeof(msg), "max LLR error = %e", max_err);
            FAIL(msg);
        }
    }

    TEST("Interleaver: actually permutes (not identity)");
    {
        size_t n = 2160;
        BitInterleaver intlv(n);

        std::vector<float> orig(n);
        for (size_t i = 0; i < n; ++i) orig[i] = static_cast<float>(i);

        std::vector<float> perm(n);
        intlv.interleave(orig.data(), perm.data());

        // Check that at least 50% of positions moved
        size_t moved = 0;
        for (size_t i = 0; i < n; ++i) {
            if (std::abs(perm[i] - orig[i]) > 0.5f) moved++;
        }

        if (moved > n / 2) {
            PASS();
        } else {
            char msg[64];
            snprintf(msg, sizeof(msg), "only %zu/%zu positions moved", moved, n);
            FAIL(msg);
        }
    }

    // =====================================================================
    // Test 4: Full chain: LDPC + Interleaver + OFDM (perfect channel)
    // =====================================================================
    TEST("Full chain: LDPC+OFDM perfect channel");
    {
        // Setup
        FECRate rate = FECRate::Rate_1_2;
        LDPCBlockSize blk = LDPCBlockSize::Short;

        LDPCEncoder enc(rate, blk);
        LDPCDecoder dec(rate, blk, 50);
        BitInterleaver intlv(enc.codewordBits());

        OFDMParams ofdm_p;
        ofdm_p.fft_size = 256;
        ofdm_p.modulation = Modulation::QAM16;
        ofdm_p.sample_rate = 48000;

        OFDMModulator   mod(ofdm_p);
        OFDMDemodulator demod(ofdm_p);

        // Process preamble
        ComplexBuf preamble = mod.generatePreamble();
        size_t short_total = 10 * (ofdm_p.fft_size / 4);
        size_t long_total  = 2 * ofdm_p.symbolLength();
        if (preamble.size() >= short_total + long_total) {
            ComplexBuf long_syms(
                preamble.begin() + static_cast<ptrdiff_t>(short_total),
                preamble.begin() + static_cast<ptrdiff_t>(short_total + long_total));
            demod.processPreamble(long_syms);
        }

        // Random info
        size_t k = enc.infoBits();
        size_t n = enc.codewordBits();
        size_t k_bytes = (k + 7) / 8;
        size_t n_bytes = (n + 7) / 8;

        std::mt19937 rng(77);
        std::vector<uint8_t> info(k_bytes, 0);
        for (auto& b : info) b = static_cast<uint8_t>(rng() & 0xFF);

        // TX: encode → interleave → OFDM modulate
        std::vector<uint8_t> codeword(n_bytes, 0);
        enc.encode(info.data(), codeword.data());

        std::vector<uint8_t> interleaved(n_bytes, 0);
        intlv.interleave(codeword.data(), interleaved.data());

        ComplexBuf tx_samples;
        mod.modulateBits(interleaved.data(), n, tx_samples);

        // RX: OFDM demodulate → soft demap → deinterleave → LDPC decode
        SymbolMapper mapper(ofdm_p.modulation);
        [[maybe_unused]] size_t bps = bitsPerSymbol(ofdm_p.modulation);
        size_t sym_len = ofdm_p.symbolLength();

        // Collect all LLRs
        std::vector<float> all_llrs;
        for (size_t base = 0; base + sym_len <= tx_samples.size(); base += sym_len) {
            ComplexBuf one(tx_samples.begin() + static_cast<ptrdiff_t>(base),
                          tx_samples.begin() + static_cast<ptrdiff_t>(base + sym_len));

            std::vector<float> sym_llrs;
            demod.demodulateSoft(one, sym_llrs, 0.01f); // low noise variance for perfect ch
            all_llrs.insert(all_llrs.end(), sym_llrs.begin(), sym_llrs.end());
        }

        // Pad or truncate to n LLRs
        all_llrs.resize(n, 0.f);

        // Deinterleave LLRs
        std::vector<float> deintlv_llrs(n);
        intlv.deinterleave(all_llrs.data(), deintlv_llrs.data());

        // LDPC decode
        std::vector<uint8_t> decoded(k_bytes, 0);
        auto result = dec.decode(deintlv_llrs.data(), decoded.data());

        size_t bit_errs = countBitDiffs(info.data(), decoded.data(), k);

        if (bit_errs == 0) {
            PASS();
        } else {
            char msg[96];
            snprintf(msg, sizeof(msg), "%zu errors, converged=%d, iter=%zu",
                     bit_errs, result.converged ? 1 : 0, result.iterations);
            FAIL(msg);
        }
    }

    // =====================================================================
    // Test 5: Coding gain — LDPC+OFDM at various SNR levels
    // =====================================================================
    printf("\n  --- Coding Gain Measurement (R=1/2 QPSK) ---\n");
    {
        FECRate rate = FECRate::Rate_1_2;
        LDPCBlockSize blk = LDPCBlockSize::Short;

        LDPCEncoder enc(rate, blk);
        LDPCDecoder dec(rate, blk, 50);
        BitInterleaver intlv(enc.codewordBits());

        OFDMParams ofdm_p;
        ofdm_p.fft_size = 256;
        ofdm_p.modulation = Modulation::QPSK;
        ofdm_p.sample_rate = 48000;

        OFDMModulator   omod(ofdm_p);
        OFDMDemodulator odemod(ofdm_p);
        SymbolMapper mapper(ofdm_p.modulation);

        size_t k = enc.infoBits();
        size_t n = enc.codewordBits();
        size_t k_bytes = (k + 7) / 8;
        size_t n_bytes = (n + 7) / 8;
        [[maybe_unused]] size_t bps = bitsPerSymbol(ofdm_p.modulation);
        size_t sym_len = ofdm_p.symbolLength();

        float snrs[] = {2.f, 4.f, 6.f, 8.f, 10.f};
        size_t num_trials = 20;

        for (float snr_db : snrs) {
            size_t total_info_bits = 0;
            size_t coded_errors = 0;
            size_t uncoded_errors = 0;

            for (size_t trial = 0; trial < num_trials; ++trial) {
                std::mt19937 rng(static_cast<unsigned>(snr_db * 100 + trial));

                // Preamble
                ComplexBuf preamble = omod.generatePreamble();
                size_t short_total = 10 * (ofdm_p.fft_size / 4);
                size_t long_total  = 2 * ofdm_p.symbolLength();
                ComplexBuf pre_noisy = preamble;
                addAWGN(pre_noisy, snr_db + 3.f, rng); // preamble gets slight boost
                if (pre_noisy.size() >= short_total + long_total) {
                    ComplexBuf lsyms(
                        pre_noisy.begin() + static_cast<ptrdiff_t>(short_total),
                        pre_noisy.begin() + static_cast<ptrdiff_t>(short_total + long_total));
                    odemod.processPreamble(lsyms);
                }

                // Random info
                std::vector<uint8_t> info(k_bytes, 0);
                for (auto& b : info) b = static_cast<uint8_t>(rng() & 0xFF);

                // TX: encode + interleave + modulate
                std::vector<uint8_t> codeword(n_bytes, 0);
                enc.encode(info.data(), codeword.data());

                std::vector<uint8_t> interleaved(n_bytes, 0);
                intlv.interleave(codeword.data(), interleaved.data());

                ComplexBuf tx_samp;
                omod.modulateBits(interleaved.data(), n, tx_samp);

                // Channel: AWGN
                addAWGN(tx_samp, snr_db, rng);

                // RX: demod + soft demap
                std::vector<float> all_llrs;
                for (size_t base = 0; base + sym_len <= tx_samp.size(); base += sym_len) {
                    ComplexBuf one(tx_samp.begin() + static_cast<ptrdiff_t>(base),
                                  tx_samp.begin() + static_cast<ptrdiff_t>(base + sym_len));

                    float noise_est = std::pow(10.f, -snr_db / 10.f);
                    std::vector<float> llrs;
                    odemod.demodulateSoft(one, llrs, noise_est);
                    all_llrs.insert(all_llrs.end(), llrs.begin(), llrs.end());
                }
                all_llrs.resize(n, 0.f);

                // Uncoded BER: hard decisions on raw LLRs, compare first k bits
                for (size_t i = 0; i < k && i < all_llrs.size(); ++i) {
                    bool rx_bit = (all_llrs[i] < 0.f);
                    bool tx_bit = getBit(interleaved.data(), i);
                    if (rx_bit != tx_bit) uncoded_errors++;
                }

                // Deinterleave + LDPC decode
                std::vector<float> deintlv(n);
                intlv.deinterleave(all_llrs.data(), deintlv.data());

                std::vector<uint8_t> decoded(k_bytes, 0);
                dec.decode(deintlv.data(), decoded.data());

                coded_errors += countBitDiffs(info.data(), decoded.data(), k);
                total_info_bits += k;
            }

            float coded_ber = static_cast<float>(coded_errors) /
                              static_cast<float>(total_info_bits);
            float uncoded_ber = static_cast<float>(uncoded_errors) /
                                static_cast<float>(total_info_bits);

            char line[100];
            snprintf(line, sizeof(line),
                     "  SNR=%4.1f dB: uncoded BER=%.2e, coded BER=%.2e %s",
                     snr_db, uncoded_ber, coded_ber,
                     (coded_ber < uncoded_ber) ? "(gain!)" :
                     (coded_ber == 0.f) ? "(CLEAN)" : "");
            printf("%s\n", line);
        }
    }

    // =====================================================================
    // Test 6: ORBGRAND clean round-trip (no errors)
    // =====================================================================
    for (auto rate : {FECRate::Rate_1_2, FECRate::Rate_2_3, FECRate::Rate_3_4}) {
        const char* names[] = {"1/2", "2/3", "3/4"};
        int ri = (rate == FECRate::Rate_1_2) ? 0 :
                 (rate == FECRate::Rate_2_3) ? 1 : 2;
        char name[80];
        snprintf(name, sizeof(name),
                 "ORBGRAND R=%s short: clean round-trip", names[ri]);
        TEST(name);

        LDPCEncoder enc(rate, LDPCBlockSize::Short);
        ORBGRANDDecoder dec(rate, LDPCBlockSize::Short);

        size_t k = enc.infoBits();
        size_t n = enc.codewordBits();
        size_t k_bytes = (k + 7) / 8;
        size_t n_bytes = (n + 7) / 8;

        std::mt19937 rng(200 + ri);
        std::vector<uint8_t> info(k_bytes, 0);
        for (auto& b : info) b = static_cast<uint8_t>(rng() & 0xFF);

        std::vector<uint8_t> codeword(n_bytes, 0);
        enc.encode(info.data(), codeword.data());

        // Perfect channel LLRs
        std::vector<float> llr(n);
        for (size_t i = 0; i < n; ++i) {
            llr[i] = getBit(codeword.data(), i) ? -4.0f : 4.0f;
        }

        std::vector<uint8_t> decoded(k_bytes, 0);
        auto result = dec.decode(llr.data(), decoded.data());

        size_t bit_errs = countBitDiffs(info.data(), decoded.data(), k);

        if (bit_errs == 0 && result.converged && result.iterations == 0) {
            PASS();
        } else {
            char msg[96];
            snprintf(msg, sizeof(msg),
                     "%zu bit errors, converged=%d, queries=%zu",
                     bit_errs, result.converged ? 1 : 0, result.iterations);
            FAIL(msg);
        }
    }

    // =====================================================================
    // Test 7: ORBGRAND error correction (soft AWGN LLRs at moderate SNR)
    //
    // NOTE: ORBGRAND relies on accurate LLR magnitudes for reliability
    // sorting. Unlike BP which can correct many errors via message passing,
    // ORBGRAND corrects few errors (≤ max_weight) but finds the ML codeword.
    // Its advantage is at the error floor (medium-high SNR), not waterfall.
    // =====================================================================
    TEST("ORBGRAND R=1/2 short: AWGN correction at 8 dB");
    {
        LDPCEncoder enc(FECRate::Rate_1_2, LDPCBlockSize::Short);
        ORBGRANDConfig cfg;
        cfg.max_queries = 10000;
        cfg.max_weight  = 4;
        ORBGRANDDecoder dec(FECRate::Rate_1_2, LDPCBlockSize::Short, cfg);

        size_t k = enc.infoBits();
        size_t n = enc.codewordBits();
        size_t k_bytes = (k + 7) / 8;
        size_t n_bytes = (n + 7) / 8;

        // Run multiple trials at SNR=8dB (expect ~5-10 raw bit errors)
        size_t total_bits = 0;
        size_t total_errors = 0;
        size_t num_converged = 0;
        size_t num_trials = 20;

        for (size_t trial = 0; trial < num_trials; ++trial) {
            std::mt19937 rng(300 + static_cast<unsigned>(trial));

            std::vector<uint8_t> info(k_bytes, 0);
            for (auto& b : info) b = static_cast<uint8_t>(rng() & 0xFF);

            std::vector<uint8_t> codeword(n_bytes, 0);
            enc.encode(info.data(), codeword.data());

            // BPSK + AWGN at 8 dB → exact LLR = 2*rx/sigma^2
            float snr_lin = std::pow(10.f, 8.f / 10.f);
            float sigma = std::sqrt(1.f / (2.f * snr_lin));
            float noise_var = sigma * sigma;
            std::normal_distribution<float> noise(0.f, sigma);

            std::vector<float> llr(n);
            for (size_t i = 0; i < n; ++i) {
                float tx = getBit(codeword.data(), i) ? -1.f : 1.f;
                float rx = tx + noise(rng);
                llr[i] = 2.f * rx / noise_var;
            }

            std::vector<uint8_t> decoded(k_bytes, 0);
            auto result = dec.decode(llr.data(), decoded.data());

            total_errors += countBitDiffs(info.data(), decoded.data(), k);
            total_bits += k;
            if (result.converged) num_converged++;
        }

        float ber = static_cast<float>(total_errors) /
                    static_cast<float>(total_bits);

        // At 8dB with R=1/2, ORBGRAND should converge most trials
        if (num_converged >= num_trials * 3 / 4 && ber < 1e-3f) {
            PASS();
        } else {
            char msg[96];
            snprintf(msg, sizeof(msg),
                     "BER=%.2e, converged=%zu/%zu",
                     ber, num_converged, num_trials);
            FAIL(msg);
        }
    }

    // =====================================================================
    // Test 8: ORBGRAND query budget (verify RT guarantee)
    // =====================================================================
    TEST("ORBGRAND: respects max_queries budget");
    {
        ORBGRANDConfig cfg;
        cfg.max_queries = 500;
        cfg.max_weight  = 4;
        ORBGRANDDecoder dec(FECRate::Rate_1_2, LDPCBlockSize::Short, cfg);
        LDPCEncoder enc(FECRate::Rate_1_2, LDPCBlockSize::Short);

        size_t n = enc.codewordBits();
        size_t n_bytes = (n + 7) / 8;

        // Feed garbage LLRs — should not converge but must respect budget
        std::mt19937 rng(999);
        std::vector<float> llr(n);
        std::normal_distribution<float> dist(0.f, 0.1f);
        for (auto& v : llr) v = dist(rng); // near-zero LLRs = pure noise

        std::vector<uint8_t> out(n_bytes, 0);
        auto result = dec.decodeFull(llr.data(), out.data());

        if (!result.converged && result.iterations <= 500) {
            PASS();
        } else {
            char msg[96];
            snprintf(msg, sizeof(msg),
                     "converged=%d, queries=%zu (expected <= 500, not converged)",
                     result.converged ? 1 : 0, result.iterations);
            FAIL(msg);
        }
    }

    // =====================================================================
    // Test 9: ORBGRAND vs BP comparative coding gain (R=1/2 QPSK)
    // =====================================================================
    printf("\n  --- ORBGRAND vs BP Coding Gain (R=1/2 QPSK, Short block) ---\n");
    {
        FECRate rate = FECRate::Rate_1_2;
        LDPCBlockSize blk = LDPCBlockSize::Short;

        LDPCEncoder enc(rate, blk);
        LDPCDecoder bp_dec(rate, blk, 50);
        ORBGRANDConfig orbcfg;
        orbcfg.max_queries = 5000;
        orbcfg.max_weight  = 4;
        ORBGRANDDecoder orb_dec(rate, blk, orbcfg);

        size_t k = enc.infoBits();
        size_t n = enc.codewordBits();
        size_t k_bytes = (k + 7) / 8;
        size_t n_bytes = (n + 7) / 8;

        float snrs[] = {4.f, 6.f, 8.f, 10.f, 12.f};
        size_t num_trials = 50;

        for (float snr_db : snrs) {
            size_t total_bits = 0;
            size_t bp_errors = 0;
            size_t orb_errors = 0;
            size_t bp_converged = 0;
            size_t orb_converged = 0;

            for (size_t trial = 0; trial < num_trials; ++trial) {
                std::mt19937 rng(
                    static_cast<unsigned>(snr_db * 1000 + trial + 500));

                // Random info
                std::vector<uint8_t> info(k_bytes, 0);
                for (auto& b : info) b = static_cast<uint8_t>(rng() & 0xFF);

                // Encode
                std::vector<uint8_t> codeword(n_bytes, 0);
                enc.encode(info.data(), codeword.data());

                // Convert to BPSK symbols + AWGN
                float snr_lin = std::pow(10.f, snr_db / 10.f);
                float sigma = std::sqrt(1.f / (2.f * snr_lin));
                std::normal_distribution<float> noise(0.f, sigma);

                std::vector<float> llr(n);
                float noise_var = sigma * sigma;
                for (size_t i = 0; i < n; ++i) {
                    float tx = getBit(codeword.data(), i) ? -1.f : 1.f;
                    float rx = tx + noise(rng);
                    // Exact LLR for BPSK: 2*rx / sigma^2
                    llr[i] = 2.f * rx / noise_var;
                }

                // BP decode
                std::vector<uint8_t> bp_out(k_bytes, 0);
                auto bp_res = bp_dec.decode(llr.data(), bp_out.data());
                bp_errors += countBitDiffs(info.data(), bp_out.data(), k);
                if (bp_res.converged) bp_converged++;

                // ORBGRAND decode
                std::vector<uint8_t> orb_out(k_bytes, 0);
                auto orb_res = orb_dec.decode(llr.data(), orb_out.data());
                orb_errors += countBitDiffs(info.data(), orb_out.data(), k);
                if (orb_res.converged) orb_converged++;

                total_bits += k;
            }

            float bp_ber  = static_cast<float>(bp_errors) /
                            static_cast<float>(total_bits);
            float orb_ber = static_cast<float>(orb_errors) /
                            static_cast<float>(total_bits);

            char line[120];
            snprintf(line, sizeof(line),
                "  SNR=%4.1f dB: BP BER=%.2e (%zu/%zu conv), "
                "ORBGRAND BER=%.2e (%zu/%zu conv) %s",
                snr_db, bp_ber, bp_converged, num_trials,
                orb_ber, orb_converged, num_trials,
                (orb_ber < bp_ber) ? "<-- ORBGRAND wins" :
                (orb_ber == bp_ber && orb_ber == 0.f) ? "(both CLEAN)" : "");
            printf("%s\n", line);
        }
    }

    // =====================================================================
    // Test 10: ORBGRAND Normal block (n=8640, higher rate)
    // =====================================================================
    TEST("ORBGRAND R=3/4 normal: clean round-trip");
    {
        LDPCEncoder enc(FECRate::Rate_3_4, LDPCBlockSize::Normal);
        ORBGRANDDecoder dec(FECRate::Rate_3_4, LDPCBlockSize::Normal);

        size_t k = enc.infoBits();
        size_t n = enc.codewordBits();
        size_t k_bytes = (k + 7) / 8;
        size_t n_bytes = (n + 7) / 8;

        std::mt19937 rng(777);
        std::vector<uint8_t> info(k_bytes, 0);
        for (auto& b : info) b = static_cast<uint8_t>(rng() & 0xFF);

        std::vector<uint8_t> codeword(n_bytes, 0);
        enc.encode(info.data(), codeword.data());

        // Perfect channel
        std::vector<float> llr(n);
        for (size_t i = 0; i < n; ++i) {
            llr[i] = getBit(codeword.data(), i) ? -5.0f : 5.0f;
        }

        std::vector<uint8_t> decoded(k_bytes, 0);
        auto result = dec.decode(llr.data(), decoded.data());

        size_t bit_errs = countBitDiffs(info.data(), decoded.data(), k);

        if (bit_errs == 0 && result.converged) {
            PASS();
        } else {
            char msg[96];
            snprintf(msg, sizeof(msg),
                     "%zu bit errors, converged=%d, queries=%zu",
                     bit_errs, result.converged ? 1 : 0, result.iterations);
            FAIL(msg);
        }
    }

    // =====================================================================
    // Test 11: List-GRAND — non-empty, non-decreasing costs, first == hard
    // =====================================================================
    TEST("List-GRAND R=1/2 short: list valid, first == decodeFull");
    {
        LDPCEncoder enc(FECRate::Rate_1_2, LDPCBlockSize::Short);
        ORBGRANDConfig cfg;
        cfg.max_queries = 10000;
        cfg.max_weight  = 4;
        ORBGRANDDecoder dec(FECRate::Rate_1_2, LDPCBlockSize::Short, cfg);

        size_t k = enc.infoBits();
        size_t n = enc.codewordBits();
        size_t k_bytes = (k + 7) / 8;
        size_t n_bytes = (n + 7) / 8;

        std::mt19937 rng(400);
        std::vector<uint8_t> info(k_bytes, 0);
        for (auto& b : info) b = static_cast<uint8_t>(rng() & 0xFF);

        std::vector<uint8_t> codeword(n_bytes, 0);
        enc.encode(info.data(), codeword.data());

        // Low-noise channel: a couple of soft errors so the list has depth
        // beyond the zero-cost member.
        float snr_lin = std::pow(10.f, 9.f / 10.f);
        float sigma = std::sqrt(1.f / (2.f * snr_lin));
        float noise_var = sigma * sigma;
        std::normal_distribution<float> noise(0.f, sigma);

        std::vector<float> llr(n);
        for (size_t i = 0; i < n; ++i) {
            float tx = getBit(codeword.data(), i) ? -1.f : 1.f;
            float rx = tx + noise(rng);
            llr[i] = 2.f * rx / noise_var;
        }

        // Hard-output reference (must remain UNCHANGED behavior).
        std::vector<uint8_t> hard_cw(n_bytes, 0);
        auto hres = dec.decodeFull(llr.data(), hard_cw.data());

        auto lr = dec.decodeList(llr.data(), 8);

        bool ok = !lr.list.empty() && lr.base.converged;
        // Costs non-decreasing
        for (size_t i = 1; ok && i < lr.list.size(); ++i) {
            if (lr.list[i].cost + 1e-4f < lr.list[i - 1].cost) ok = false;
        }
        // First list entry == hard decodeFull() codeword (both ML at low noise)
        if (ok && hres.converged) {
            if (countBitDiffs(lr.list.front().bits.data(),
                              hard_cw.data(), n) != 0) ok = false;
        }
        // Every listed candidate must be a true codeword (== original here,
        // since at low noise the ML decode recovers the transmitted word).
        if (ok) {
            if (countBitDiffs(lr.list.front().bits.data(),
                              codeword.data(), n) != 0) ok = false;
        }

        if (ok) {
            PASS();
        } else {
            char msg[120];
            snprintf(msg, sizeof(msg),
                     "list=%zu conv=%d firstDiff=%zu",
                     lr.list.size(), lr.base.converged ? 1 : 0,
                     lr.list.empty() ? 0
                       : countBitDiffs(lr.list.front().bits.data(),
                                       codeword.data(), n));
            FAIL(msg);
        }
    }

    // =====================================================================
    // Test 12: SOGRAND posterior — signs match TX bits; FER vs hard ORBGRAND
    // =====================================================================
    TEST("SOGRAND R=1/2 short: posterior signs match TX @ 8 dB");
    {
        LDPCEncoder enc(FECRate::Rate_1_2, LDPCBlockSize::Short);
        ORBGRANDConfig cfg;
        cfg.max_queries = 10000;
        cfg.max_weight  = 4;
        ORBGRANDDecoder dec(FECRate::Rate_1_2, LDPCBlockSize::Short, cfg);

        size_t k = enc.infoBits();
        size_t n = enc.codewordBits();
        size_t k_bytes = (k + 7) / 8;
        size_t n_bytes = (n + 7) / 8;

        size_t num_trials = 20;
        size_t post_frame_err = 0;   // SOGRAND posterior-sign frame errors
        size_t hard_frame_err = 0;   // hard ORBGRAND frame errors
        size_t post_bit_err   = 0;
        size_t total_cw_bits  = 0;

        for (size_t trial = 0; trial < num_trials; ++trial) {
            std::mt19937 rng(600 + static_cast<unsigned>(trial));

            std::vector<uint8_t> info(k_bytes, 0);
            for (auto& b : info) b = static_cast<uint8_t>(rng() & 0xFF);

            std::vector<uint8_t> codeword(n_bytes, 0);
            enc.encode(info.data(), codeword.data());

            float snr_lin = std::pow(10.f, 8.f / 10.f);
            float sigma = std::sqrt(1.f / (2.f * snr_lin));
            float noise_var = sigma * sigma;
            std::normal_distribution<float> noise(0.f, sigma);

            std::vector<float> llr(n);
            for (size_t i = 0; i < n; ++i) {
                float tx = getBit(codeword.data(), i) ? -1.f : 1.f;
                float rx = tx + noise(rng);
                llr[i] = 2.f * rx / noise_var;
            }

            // SOGRAND posterior
            std::vector<float> post;
            auto pres = dec.decodePosterior(llr.data(), post);

            // Posterior signs → hard bits (post>0 => bit 0)
            size_t frame_bad = 0;
            for (size_t i = 0; i < n; ++i) {
                bool pbit = (post[i] < 0.f);
                bool tbit = getBit(codeword.data(), i);
                if (pbit != tbit) { ++post_bit_err; ++frame_bad; }
            }
            if (frame_bad) ++post_frame_err;
            total_cw_bits += n;

            // Hard ORBGRAND for comparison
            std::vector<uint8_t> hard_cw(n_bytes, 0);
            dec.decodeFull(llr.data(), hard_cw.data());
            if (countBitDiffs(hard_cw.data(), codeword.data(), n) != 0)
                ++hard_frame_err;

            (void)pres;
        }

        float post_ber = static_cast<float>(post_bit_err) /
                         static_cast<float>(total_cw_bits);

        // SOGRAND posterior signs should recover the codeword on the vast
        // majority of frames, with FER no worse than hard ORBGRAND.
        if (post_ber < 1e-3f && post_frame_err <= hard_frame_err) {
            PASS();
        } else {
            char msg[120];
            snprintf(msg, sizeof(msg),
                     "postBER=%.2e postFER=%zu/%zu hardFER=%zu/%zu",
                     post_ber, post_frame_err, num_trials,
                     hard_frame_err, num_trials);
            FAIL(msg);
        }
    }

    // =====================================================================
    // Test 13: ISoftDecoder polymorphism — BICMDecoder with ORBGRAND inner
    // =====================================================================
    TEST("ISoftDecoder: BICM over ORBGRAND clean round-trip");
    {
        FECRate rate = FECRate::Rate_1_2;
        LDPCBlockSize blk = LDPCBlockSize::Short;

        LDPCEncoder enc(rate, blk);
        ORBGRANDConfig ocfg;
        ocfg.max_queries = 10000;
        ocfg.max_weight  = 4;
        ORBGRANDDecoder orb(rate, blk, ocfg);

        Modulation mod = Modulation::QPSK;
        SymbolMapper mapper(mod);
        BitInterleaver inter(enc.codewordBits());

        BICMConfig bc;
        bc.outer_iterations = 1;
        bc.use_extrinsic    = true;
        // ISoftDecoder* binding: pass ORBGRANDDecoder* where the ctor
        // expects ISoftDecoder*.
        ISoftDecoder* soft = &orb;
        BICMDecoder bicm(&mapper, &inter, soft, bc);

        size_t k = enc.infoBits();
        size_t n = enc.codewordBits();
        size_t k_bytes = (k + 7) / 8;
        size_t n_bytes = (n + 7) / 8;
        size_t bps = bitsPerSymbol(mod);

        std::mt19937 rng(0x50F6);
        std::vector<uint8_t> info(k_bytes, 0);
        for (auto& b : info) b = static_cast<uint8_t>(rng() & 0xFF);

        std::vector<uint8_t> codeword(n_bytes, 0);
        enc.encode(info.data(), codeword.data());

        std::vector<uint8_t> interleaved(n_bytes, 0);
        inter.interleave(codeword.data(), interleaved.data());

        // Map interleaved bits to clean QPSK symbols.
        size_t n_syms = (n + bps - 1) / bps;
        ComplexBuf syms(n_syms);
        for (size_t s = 0; s < n_syms; ++s) {
            uint32_t idx = 0;
            for (size_t j = 0; j < bps; ++j) {
                size_t bit_pos = s * bps + j;
                bool bit = (bit_pos < n) && getBit(interleaved.data(), bit_pos);
                idx = (idx << 1) | (bit ? 1u : 0u);
            }
            syms[s] = mapper.constellation()[idx];
        }

        std::vector<uint8_t> decoded(k_bytes, 0);
        auto r = bicm.decode(syms, 0.01f, decoded.data());

        size_t bit_errs = countBitDiffs(info.data(), decoded.data(), k);
        if (bit_errs == 0 && r.converged) {
            PASS();
        } else {
            char msg[96];
            snprintf(msg, sizeof(msg),
                     "%zu bit errors, converged=%d",
                     bit_errs, r.converged ? 1 : 0);
            FAIL(msg);
        }
    }

    // =====================================================================
    // Summary
    // =====================================================================
    printf("\n=== Results: %d passed, %d failed ===\n",
           tests_passed, tests_failed);

    if (tests_failed == 0) {
        printf("\n>>> ALL FEC TESTS PASSED <<<\n");
    }

    return tests_failed > 0 ? 1 : 0;
}
