/**
 * @file loopback_test.cpp
 * @brief End-to-end OFDM loopback test
 *
 * Tests the complete signal chain:
 *   Bits → QAM Map → OFDM Mod → [channel] → OFDM Demod → QAM Demap → Bits
 *
 * Verifies:
 *   1. Subcarrier allocation matches between TX and RX
 *   2. Preamble-based channel estimation works (frequency-domain)
 *   3. Data bits survive round-trip through AWGN channel
 *   4. Frame builder/parser round-trip
 */

#include "types.hpp"
#include "ofdm.hpp"
#include "frame.hpp"
#include <cstdio>
#include <cstring>
#include <cmath>
#include <random>
#include <algorithm>
#include <numeric>

using namespace dsca;

static int tests_passed = 0;
static int tests_failed = 0;

#define TEST(name) printf("  %-55s", name)
#define PASS() do { printf("[PASS]\n"); tests_passed++; } while(0)
#define FAIL(msg) do { printf("[FAIL] %s\n", msg); tests_failed++; } while(0)

// Add AWGN noise to complex samples
static void addNoise(ComplexBuf& samples, float snr_db, std::mt19937& rng) {
    // Measure signal power
    float sig_power = 0.f;
    for (auto& s : samples) sig_power += std::norm(s);
    sig_power /= static_cast<float>(samples.size());

    float snr_lin = std::pow(10.f, snr_db / 10.f);
    float noise_power = sig_power / snr_lin;
    float sigma = std::sqrt(noise_power / 2.f); // per-component

    std::normal_distribution<float> dist(0.f, sigma);
    for (auto& s : samples) {
        s += ComplexSample(dist(rng), dist(rng));
    }
}

int main() {
    printf("=== DSCA-NG v2 Loopback Test ===\n\n");

    // =====================================================================
    // Test 1: Subcarrier allocation consistency
    // =====================================================================
    TEST("Allocation consistency (TX == RX)");
    {
        OFDMParams p;
        p.fft_size = 256;
        p.pilot_spacing = 8;
        p.dc_null = true;

        auto a1 = computeAllocation(p);
        auto a2 = computeAllocation(p); // second call must be identical

        bool ok = (a1.data_indices == a2.data_indices) &&
                  (a1.pilot_indices == a2.pilot_indices) &&
                  (a1.pilot_values.size() == a2.pilot_values.size());

        if (ok && a1.dataCount() > 0 && a1.pilotCount() > 0) {
            PASS();
        } else {
            FAIL("allocations differ or empty");
        }
    }

    // =====================================================================
    // Test 2: Modulator/Demodulator dimensions
    // =====================================================================
    TEST("Mod/demod dimensions match");
    {
        OFDMParams p;
        p.fft_size = 256;
        p.modulation = Modulation::QAM16;

        OFDMModulator  mod(p);
        OFDMDemodulator demod(p);

        if (mod.dataSubcarriers() == demod.allocation().dataCount() &&
            mod.allocation().pilot_indices == demod.allocation().pilot_indices) {
            PASS();
        } else {
            char msg[128];
            snprintf(msg, sizeof(msg), "TX data=%zu, RX data=%zu",
                     mod.dataSubcarriers(), demod.allocation().dataCount());
            FAIL(msg);
        }
    }

    // =====================================================================
    // Test 3: Perfect channel loopback (no noise)
    // =====================================================================
    TEST("Perfect channel: zero BER");
    {
        OFDMParams p;
        p.fft_size = 256;
        p.modulation = Modulation::QAM16;
        p.cyclic_prefix = CyclicPrefix::CP_1_8;
        p.sample_rate = 48000;

        OFDMModulator   mod(p);
        OFDMDemodulator demod(p);

        // Generate preamble, pass to demodulator
        ComplexBuf preamble_td = mod.generatePreamble();

        // Extract the two long preamble symbols from the end
        // Short preamble: 10 reps × (N/4) samples
        // Long preamble: 2 × (CP + N) samples
        size_t short_total = 10 * (p.fft_size / 4);
        size_t long_total  = 2 * p.symbolLength();

        if (preamble_td.size() >= short_total + long_total) {
            ComplexBuf long_syms(preamble_td.begin() + static_cast<ptrdiff_t>(short_total),
                                 preamble_td.begin() + static_cast<ptrdiff_t>(short_total + long_total));
            demod.processPreamble(long_syms);
        }

        // Generate test data
        size_t bits_per_sym = mod.bitsPerOFDMSymbol();
        size_t test_bytes = (bits_per_sym + 7) / 8; // enough for 1 OFDM symbol
        ByteVec test_data(test_bytes);
        std::mt19937 rng(12345);
        for (auto& b : test_data) b = static_cast<uint8_t>(rng() & 0xFF);

        // Modulate
        ComplexBuf tx_samples;
        mod.modulateBits(test_data.data(), bits_per_sym, tx_samples);

        // Demodulate (perfect channel = identity)
        ComplexBuf rx_symbols;
        bool ok = demod.demodulate(tx_samples, rx_symbols);

        if (!ok) { FAIL("demodulate returned false"); }
        else {
            // Hard-demap and compare bits
            SymbolMapper mapper(p.modulation);
            size_t bps = bitsPerSymbol(p.modulation);
            size_t errors = 0;
            size_t total_bits = 0;

            for (size_t s = 0; s < rx_symbols.size() && s < mod.dataSubcarriers(); ++s) {
                uint16_t rx_idx = mapper.demapHard(rx_symbols[s]);
                // Extract original bits for this symbol
                uint16_t tx_idx = 0;
                for (size_t b = 0; b < bps; ++b) {
                    size_t bit_pos = s * bps + b;
                    size_t byte_i = bit_pos / 8;
                    size_t bit_i  = 7 - (bit_pos % 8);
                    if (byte_i < test_data.size()) {
                        uint8_t bit = (test_data[byte_i] >> bit_i) & 1;
                        tx_idx = static_cast<uint16_t>((tx_idx << 1) | bit);
                    }
                }
                // Count bit errors
                uint16_t diff = rx_idx ^ tx_idx;
                for (int b = 0; b < 16; ++b) {
                    if ((diff >> b) & 1) errors++;
                }
                total_bits += bps;
            }

            if (errors == 0) {
                PASS();
            } else {
                char msg[64];
                snprintf(msg, sizeof(msg), "%zu bit errors in %zu bits",
                         errors, total_bits);
                FAIL(msg);
            }
        }
    }

    // =====================================================================
    // Test 4: AWGN channel with preamble-based channel estimation
    // =====================================================================
    TEST("AWGN 30dB SNR: BER < 1e-3 (QAM16)");
    {
        OFDMParams p;
        p.fft_size = 512;
        p.modulation = Modulation::QAM16;
        p.cyclic_prefix = CyclicPrefix::CP_1_8;
        p.sample_rate = 48000;

        OFDMModulator   mod(p);
        OFDMDemodulator demod(p);
        std::mt19937 rng(42);

        // Generate and process preamble through noisy channel
        ComplexBuf preamble_td = mod.generatePreamble();
        addNoise(preamble_td, 30.f, rng);

        size_t short_total = 10 * (p.fft_size / 4);
        size_t long_total  = 2 * p.symbolLength();
        if (preamble_td.size() >= short_total + long_total) {
            ComplexBuf long_syms(preamble_td.begin() + static_cast<ptrdiff_t>(short_total),
                                 preamble_td.begin() + static_cast<ptrdiff_t>(short_total + long_total));
            demod.processPreamble(long_syms);
        }

        // Multiple OFDM symbols worth of data
        size_t num_ofdm = 10;
        size_t bits_per = mod.bitsPerOFDMSymbol();
        size_t total_bits = bits_per * num_ofdm;
        size_t total_bytes = (total_bits + 7) / 8;
        ByteVec test_data(total_bytes);
        for (auto& b : test_data) b = static_cast<uint8_t>(rng() & 0xFF);

        ComplexBuf tx_samples;
        mod.modulateBits(test_data.data(), total_bits, tx_samples);

        // Add noise
        addNoise(tx_samples, 30.f, rng);

        // Demodulate symbol by symbol
        SymbolMapper mapper(p.modulation);
        size_t bps = bitsPerSymbol(p.modulation);
        size_t sym_len = p.symbolLength();
        size_t errors = 0;
        size_t counted_bits = 0;
        size_t global_sym = 0;

        for (size_t oi = 0; oi < num_ofdm; ++oi) {
            size_t base = oi * sym_len;
            if (base + sym_len > tx_samples.size()) break;

            ComplexBuf one_sym(tx_samples.begin() + static_cast<ptrdiff_t>(base),
                               tx_samples.begin() + static_cast<ptrdiff_t>(base + sym_len));

            ComplexBuf data_syms;
            if (!demod.demodulate(one_sym, data_syms)) continue;

            for (size_t s = 0; s < data_syms.size(); ++s) {
                uint16_t rx_idx = mapper.demapHard(data_syms[s]);
                uint16_t tx_idx = 0;
                for (size_t b = 0; b < bps; ++b) {
                    size_t bit_pos = global_sym * bps + b;
                    size_t byte_i = bit_pos / 8;
                    size_t bit_i  = 7 - (bit_pos % 8);
                    if (byte_i < test_data.size()) {
                        uint8_t bit = (test_data[byte_i] >> bit_i) & 1;
                        tx_idx = static_cast<uint16_t>((tx_idx << 1) | bit);
                    }
                }
                uint16_t diff = rx_idx ^ tx_idx;
                for (int b = 0; b < 16; ++b) {
                    if ((diff >> b) & 1) errors++;
                }
                counted_bits += bps;
                global_sym++;
            }
        }

        float ber = (counted_bits > 0)
                  ? static_cast<float>(errors) / static_cast<float>(counted_bits)
                  : 1.f;

        if (ber < 1e-3f) {
            PASS();
        } else {
            char msg[64];
            snprintf(msg, sizeof(msg), "BER=%.4e (%zu/%zu)", ber, errors, counted_bits);
            FAIL(msg);
        }
    }

    // =====================================================================
    // Test 5: Full chain with frame builder/parser
    // =====================================================================
    TEST("Full chain: Frame build → OFDM → Frame parse");
    {
        OFDMParams p;
        p.fft_size = 256;
        p.modulation = Modulation::QPSK; // robust for this test
        p.cyclic_prefix = CyclicPrefix::CP_1_8;
        p.sample_rate = 48000;

        OFDMModulator   mod(p);
        OFDMDemodulator demod(p);

        // Process preamble (perfect channel)
        ComplexBuf preamble = mod.generatePreamble();
        size_t short_total = 10 * (p.fft_size / 4);
        size_t long_total  = 2 * p.symbolLength();
        if (preamble.size() >= short_total + long_total) {
            ComplexBuf long_syms(preamble.begin() + static_cast<ptrdiff_t>(short_total),
                                 preamble.begin() + static_cast<ptrdiff_t>(short_total + long_total));
            demod.processPreamble(long_syms);
        }

        // Build frame with test payload
        uint8_t payload[] = "Hello DSCA-NG v2!";
        size_t payload_len = sizeof(payload) - 1; // exclude null terminator

        // Calculate capacity: enough for our frame through OFDM
        size_t frame_capacity = 128;
        FrameBuilder builder(frame_capacity);
        builder.addPacket(0, payload, payload_len);
        ByteVec frame_bytes = builder.build(1, FECRate::Rate_1_2, Modulation::QPSK);

        // Modulate frame bytes as bits
        size_t frame_bits = frame_bytes.size() * 8;
        ComplexBuf tx_samples;
        mod.modulateBits(frame_bytes.data(), frame_bits, tx_samples);

        // Demodulate (perfect channel)
        SymbolMapper mapper(p.modulation);
        size_t bps = bitsPerSymbol(p.modulation);
        size_t sym_len = p.symbolLength();

        // Collect all demodulated bits
        ByteVec rx_bytes(frame_bytes.size(), 0);
        size_t bit_pos = 0;

        for (size_t base = 0; base + sym_len <= tx_samples.size(); base += sym_len) {
            ComplexBuf one_sym(tx_samples.begin() + static_cast<ptrdiff_t>(base),
                               tx_samples.begin() + static_cast<ptrdiff_t>(base + sym_len));

            ComplexBuf data_syms;
            if (!demod.demodulate(one_sym, data_syms)) continue;

            for (size_t s = 0; s < data_syms.size(); ++s) {
                uint16_t idx = mapper.demapHard(data_syms[s]);
                for (size_t b = 0; b < bps; ++b) {
                    size_t byte_i = bit_pos / 8;
                    size_t bit_i  = 7 - (bit_pos % 8);
                    if (byte_i < rx_bytes.size()) {
                        uint8_t bit = static_cast<uint8_t>((idx >> (bps - 1 - b)) & 1);
                        rx_bytes[byte_i] |= (bit << bit_i);
                    }
                    bit_pos++;
                }
            }
        }

        // Parse received frame
        ParsedFrame parsed;
        bool ok = FrameParser::parse(rx_bytes, parsed);

        if (!ok || !parsed.valid) {
            FAIL("frame parse failed");
        } else if (!parsed.crc_ok) {
            FAIL("CRC mismatch");
        } else if (parsed.packets.empty()) {
            FAIL("no packets recovered");
        } else if (parsed.packets[0].data.size() != payload_len) {
            char msg[64];
            snprintf(msg, sizeof(msg), "payload len %zu, expected %zu",
                     parsed.packets[0].data.size(), payload_len);
            FAIL(msg);
        } else if (memcmp(parsed.packets[0].data.data(), payload, payload_len) != 0) {
            FAIL("payload content mismatch");
        } else {
            PASS();
        }
    }

    // =====================================================================
    // Test 6: Multiple modulation schemes
    // =====================================================================
    for (auto mod_type : {Modulation::BPSK, Modulation::QPSK, Modulation::QAM16,
                          Modulation::QAM64, Modulation::QAM256}) {
        char name[64];
        const char* mod_names[] = {"BPSK", "QPSK", "QAM16", "QAM64", "QAM256"};
        snprintf(name, sizeof(name), "Perfect channel: %s",
                 mod_names[static_cast<int>(mod_type)]);
        TEST(name);

        OFDMParams p;
        p.fft_size = 256;
        p.modulation = mod_type;
        p.sample_rate = 48000;

        OFDMModulator   tx(p);
        OFDMDemodulator rx(p);

        // Process preamble
        ComplexBuf preamble = tx.generatePreamble();
        size_t short_total = 10 * (p.fft_size / 4);
        size_t long_total  = 2 * p.symbolLength();
        if (preamble.size() >= short_total + long_total) {
            ComplexBuf long_syms(preamble.begin() + static_cast<ptrdiff_t>(short_total),
                                 preamble.begin() + static_cast<ptrdiff_t>(short_total + long_total));
            rx.processPreamble(long_syms);
        }

        // Single OFDM symbol of random data
        size_t bits = tx.bitsPerOFDMSymbol();
        size_t bytes = (bits + 7) / 8;
        ByteVec data(bytes);
        std::mt19937 rng(static_cast<unsigned>(mod_type));
        for (auto& b : data) b = static_cast<uint8_t>(rng() & 0xFF);

        ComplexBuf samples;
        tx.modulateBits(data.data(), bits, samples);

        ComplexBuf rx_syms;
        rx.demodulate(samples, rx_syms);

        SymbolMapper mapper(mod_type);
        size_t bps = bitsPerSymbol(mod_type);
        size_t errors = 0;
        size_t total = 0;

        for (size_t s = 0; s < rx_syms.size() && s < tx.dataSubcarriers(); ++s) {
            uint16_t rx_idx = mapper.demapHard(rx_syms[s]);
            uint16_t tx_idx = 0;
            for (size_t b = 0; b < bps; ++b) {
                size_t bp = s * bps + b;
                size_t bi = bp / 8;
                size_t bb = 7 - (bp % 8);
                if (bi < data.size()) {
                    uint8_t bit = (data[bi] >> bb) & 1;
                    tx_idx = static_cast<uint16_t>((tx_idx << 1) | bit);
                }
            }
            uint16_t diff = rx_idx ^ tx_idx;
            for (int b = 0; b < 16; ++b) {
                if ((diff >> b) & 1) errors++;
            }
            total += bps;
        }

        if (errors == 0) {
            PASS();
        } else {
            char msg[64];
            snprintf(msg, sizeof(msg), "%zu errors in %zu bits", errors, total);
            FAIL(msg);
        }
    }

    // =====================================================================
    // Summary
    // =====================================================================
    printf("\n=== Results: %d passed, %d failed ===\n",
           tests_passed, tests_failed);

    if (tests_failed == 0) {
        printf("\n>>> ALL TESTS PASSED — Core DSP chain is solid! <<<\n");
    }

    return tests_failed > 0 ? 1 : 0;
}
