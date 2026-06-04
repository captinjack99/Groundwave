/**
 * @file codec_test.cpp
 * @brief Opus codec + full chain integration tests
 *
 * Tests:
 *  1. Opus encode/decode round-trip (sine wave, delay-compensated SNR)
 *  2. Opus encode/decode round-trip (stereo)
 *  3. Packet loss concealment
 *  4. Bitrate adaptation
 *  5. Full TX/RX chain: PCM → Opus → Frame → LDPC → OFDM → ... → PCM
 *  6. Full chain with AWGN at multiple SNR levels
 */

#include "types.hpp"
#include "opus_codec.hpp"
#include "ldpc.hpp"
#include "interleaver.hpp"
#include "ofdm.hpp"
#include "frame.hpp"
#include <cstdio>
#include <cstring>
#include <cmath>
#include <random>
#include <vector>
#include <algorithm>
#include <numeric>

using namespace dsca;

static int tests_passed = 0;
static int tests_failed = 0;

#define TEST(name) printf("  %-60s", name)
#define PASS() do { printf("[PASS]\n"); tests_passed++; } while(0)
#define FAIL(msg) do { printf("[FAIL] %s\n", msg); tests_failed++; } while(0)

namespace {

std::vector<float> generateSine(float freq, float sample_rate,
                                 size_t num_samples, float amplitude = 0.8f) {
    std::vector<float> pcm(num_samples);
    for (size_t i = 0; i < num_samples; ++i) {
        pcm[i] = amplitude * std::sin(2.0f * 3.14159265f * freq *
                 static_cast<float>(i) / sample_rate);
    }
    return pcm;
}

std::vector<float> generateStereoSine(float freq_l, float freq_r,
                                       float sample_rate, size_t num_samples,
                                       float amplitude = 0.8f) {
    std::vector<float> pcm(num_samples * 2);
    for (size_t i = 0; i < num_samples; ++i) {
        pcm[i * 2]     = amplitude * std::sin(2.0f * 3.14159265f * freq_l *
                          static_cast<float>(i) / sample_rate);
        pcm[i * 2 + 1] = amplitude * std::sin(2.0f * 3.14159265f * freq_r *
                          static_cast<float>(i) / sample_rate);
    }
    return pcm;
}

/**
 * Find delay between two signals via cross-correlation, then compute SNR
 * on the aligned overlap. This handles Opus's algorithmic lookahead (~6.5ms).
 */
float computeAlignedSNR(const float* orig, size_t orig_n,
                         const float* recon, size_t recon_n,
                         int max_delay = 1024) {
    // Find delay by cross-correlation
    size_t min_n = std::min(orig_n, recon_n);
    int best_d = 0;
    float best_corr = -1e30f;

    for (int d = 0; d <= max_delay && d < static_cast<int>(min_n / 2); ++d) {
        float corr = 0.f;
        size_t count = min_n - static_cast<size_t>(d);
        for (size_t i = 0; i < count; ++i) {
            corr += orig[i] * recon[i + static_cast<size_t>(d)];
        }
        if (corr > best_corr) { best_corr = corr; best_d = d; }
    }

    // Compute SNR on aligned region (skip edges)
    size_t margin = 480; // skip half a frame at each end
    size_t start = static_cast<size_t>(best_d) + margin;
    size_t end = min_n - margin;
    if (start >= end || end - start < 480) return 0.f;

    float sig_power = 0.f, noise_power = 0.f;
    for (size_t i = start; i < end; ++i) {
        size_t oi = i - static_cast<size_t>(best_d);
        if (oi >= orig_n) break;
        sig_power += orig[oi] * orig[oi];
        float err = orig[oi] - recon[i];
        noise_power += err * err;
    }
    if (noise_power < 1e-20f) return 120.0f;
    return 10.0f * std::log10(sig_power / noise_power);
}

inline bool getBit(const uint8_t* d, size_t i) {
    return (d[i >> 3] >> (7 - (i & 7))) & 1;
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
    printf("=== DSCA-NG v2 Codec Test Suite ===\n\n");

    // =====================================================================
    // Test 1: Opus mono encode/decode round-trip (delay-compensated)
    // =====================================================================
    TEST("Opus mono: 1kHz sine (delay-compensated SNR)");
    {
        OpusConfig cfg;
        cfg.sample_rate = 48000;
        cfg.channels = 1;
        cfg.bitrate = 64000;
        cfg.frame_ms = 20.0f;

        OpusAudioEncoder enc(cfg);
        OpusAudioDecoder dec(cfg);

        size_t frame_samples = cfg.frameSamples(); // 960
        size_t num_frames = 20;
        auto pcm = generateSine(1000.0f, 48000.0f, frame_samples * num_frames);

        // Encode all, decode all, collect into contiguous buffer
        std::vector<float> all_decoded;
        for (size_t f = 0; f < num_frames; ++f) {
            std::vector<uint8_t> packet;
            enc.encode(pcm.data() + f * frame_samples, packet);
            std::vector<float> decoded;
            dec.decode(packet.data(), packet.size(), decoded);
            all_decoded.insert(all_decoded.end(), decoded.begin(), decoded.end());
        }

        float snr = computeAlignedSNR(pcm.data(), pcm.size(),
                                        all_decoded.data(), all_decoded.size());
        if (snr > 25.0f) {
            PASS();
        } else {
            char msg[64];
            snprintf(msg, sizeof(msg), "SNR=%.1f dB (expected >25)", snr);
            FAIL(msg);
        }
    }

    // =====================================================================
    // Test 2: Opus stereo encode/decode
    // =====================================================================
    TEST("Opus stereo: L=440Hz R=880Hz (delay-compensated)");
    {
        OpusConfig cfg;
        cfg.sample_rate = 48000;
        cfg.channels = 2;
        cfg.bitrate = 96000;
        cfg.frame_ms = 20.0f;

        OpusAudioEncoder enc(cfg);
        OpusAudioDecoder dec(cfg);

        size_t frame_samples = cfg.frameSamples();
        size_t num_frames = 20;
        auto pcm = generateStereoSine(440.0f, 880.0f, 48000.0f,
                                       frame_samples * num_frames);

        std::vector<float> all_decoded;
        for (size_t f = 0; f < num_frames; ++f) {
            std::vector<uint8_t> packet;
            enc.encode(pcm.data() + f * frame_samples * 2, packet);
            std::vector<float> decoded;
            dec.decode(packet.data(), packet.size(), decoded);
            all_decoded.insert(all_decoded.end(), decoded.begin(), decoded.end());
        }

        // Measure on interleaved stereo signal
        float snr = computeAlignedSNR(pcm.data(), pcm.size(),
                                        all_decoded.data(), all_decoded.size());
        if (snr > 18.0f) {
            PASS();
        } else {
            char msg[64];
            snprintf(msg, sizeof(msg), "SNR=%.1f dB (expected >18)", snr);
            FAIL(msg);
        }
    }

    // =====================================================================
    // Test 3: Packet loss concealment
    // =====================================================================
    TEST("Opus PLC: decode after 1 lost packet");
    {
        OpusConfig cfg;
        cfg.sample_rate = 48000;
        cfg.channels = 1;
        cfg.bitrate = 64000;
        cfg.frame_ms = 20.0f;

        OpusAudioEncoder enc(cfg);
        OpusAudioDecoder dec(cfg);

        size_t frame_samples = cfg.frameSamples();
        auto pcm = generateSine(1000.0f, 48000.0f, frame_samples * 5);

        // Encode and decode first 3 frames normally
        for (size_t f = 0; f < 3; ++f) {
            std::vector<uint8_t> packet;
            enc.encode(pcm.data() + f * frame_samples, packet);
            std::vector<float> decoded;
            dec.decode(packet.data(), packet.size(), decoded);
        }

        // Simulate packet loss on frame 3
        std::vector<float> plc_out;
        size_t plc_n = dec.decodePLC(plc_out);

        // Decode frame 4 normally
        std::vector<uint8_t> packet;
        enc.encode(pcm.data() + 4 * frame_samples, packet);
        std::vector<float> decoded;
        size_t n = dec.decode(packet.data(), packet.size(), decoded);

        if (plc_n == frame_samples && n == frame_samples) {
            float plc_power = 0.f;
            for (size_t i = 0; i < plc_out.size(); ++i)
                plc_power += plc_out[i] * plc_out[i];
            plc_power /= static_cast<float>(plc_out.size());

            if (plc_power > 0.001f) {
                PASS();
            } else {
                FAIL("PLC produced near-silence");
            }
        } else {
            char msg[64];
            snprintf(msg, sizeof(msg), "plc=%zu decode=%zu (expected %zu)",
                     plc_n, n, frame_samples);
            FAIL(msg);
        }
    }

    // =====================================================================
    // Test 4: Dynamic bitrate change
    // =====================================================================
    TEST("Opus: dynamic bitrate change 64k -> 32k -> 128k");
    {
        OpusConfig cfg;
        cfg.sample_rate = 48000;
        cfg.channels = 1;
        cfg.bitrate = 64000;
        cfg.frame_ms = 20.0f;

        OpusAudioEncoder enc(cfg);
        OpusAudioDecoder dec(cfg);

        size_t frame_samples = cfg.frameSamples();
        auto pcm = generateSine(1000.0f, 48000.0f, frame_samples * 6);

        size_t sizes[3] = {0, 0, 0};
        uint32_t rates[] = {64000, 32000, 128000};

        for (int phase = 0; phase < 3; ++phase) {
            enc.setBitrate(rates[phase]);
            for (int f = 0; f < 2; ++f) {
                size_t idx = static_cast<size_t>(phase * 2 + f);
                std::vector<uint8_t> packet;
                enc.encode(pcm.data() + idx * frame_samples, packet);
                sizes[phase] += packet.size();
                std::vector<float> decoded;
                dec.decode(packet.data(), packet.size(), decoded);
            }
        }

        if (sizes[1] < sizes[0] && sizes[2] > sizes[0]) {
            PASS();
        } else {
            char msg[96];
            snprintf(msg, sizeof(msg), "64k=%zu, 32k=%zu, 128k=%zu bytes",
                     sizes[0], sizes[1], sizes[2]);
            FAIL(msg);
        }
    }

    // =====================================================================
    // Test 5: Full TX/RX chain — PCM → Opus → Frame → LDPC → OFDM (perfect)
    // =====================================================================
    TEST("Full chain: PCM->Opus->LDPC->OFDM->...->PCM (perfect)");
    {
        // Opus config: mono, 32kbps CBR to fit in LDPC short block
        OpusConfig ocfg;
        ocfg.sample_rate = 48000;
        ocfg.channels = 1;
        ocfg.bitrate = 32000;
        ocfg.frame_ms = 20.0f;
        ocfg.vbr = false;

        OpusAudioEncoder oenc(ocfg);
        OpusAudioDecoder odec(ocfg);

        // LDPC R=1/2 short: k=1080 bits = 135 bytes total info
        // Frame overhead = 24 bytes (sync+header+CRC)
        // Payload capacity = 135 - 24 = 111 bytes
        // Minus 3 bytes packet header = 108 bytes for Opus data
        // At 32kbps CBR, 20ms → ~80 bytes per packet. Fits.
        FECRate fec_rate = FECRate::Rate_1_2;
        LDPCBlockSize blk = LDPCBlockSize::Short;
        LDPCEncoder ldpc_enc(fec_rate, blk);
        LDPCDecoder ldpc_dec(fec_rate, blk, 50);
        BitInterleaver intlv(ldpc_enc.codewordBits());

        size_t k_bytes = ldpc_enc.infoBytes();
        size_t n = ldpc_enc.codewordBits();
        size_t n_bytes = ldpc_enc.codewordBytes();

        // Frame capacity = k_bytes - overhead, so built frame = k_bytes exactly
        size_t frame_capacity = k_bytes - constants::FRAME_OVERHEAD;

        OFDMParams ofdm_p;
        ofdm_p.fft_size = 256;
        ofdm_p.modulation = Modulation::QPSK;
        ofdm_p.sample_rate = 48000;

        OFDMModulator   omod(ofdm_p);
        OFDMDemodulator odemod(ofdm_p);

        // Preamble channel estimation
        ComplexBuf preamble = omod.generatePreamble();
        size_t short_total = 10 * (ofdm_p.fft_size / 4);
        size_t long_total  = 2 * ofdm_p.symbolLength();
        if (preamble.size() >= short_total + long_total) {
            ComplexBuf lsyms(
                preamble.begin() + static_cast<ptrdiff_t>(short_total),
                preamble.begin() + static_cast<ptrdiff_t>(short_total + long_total));
            odemod.processPreamble(lsyms);
        }

        size_t audio_frame_samples = ocfg.frameSamples();
        size_t sym_len = ofdm_p.symbolLength();

        // Generate test audio
        size_t num_frames = 10;
        auto pcm_orig = generateSine(1000.0f, 48000.0f,
                                      audio_frame_samples * num_frames);

        std::vector<float> all_decoded_audio;
        bool chain_ok = true;

        for (size_t f = 0; f < num_frames; ++f) {
            // === TX ===
            // 1. Opus encode
            std::vector<uint8_t> opus_pkt;
            if (!oenc.encode(pcm_orig.data() + f * audio_frame_samples, opus_pkt)) {
                FAIL("Opus encode failed"); chain_ok = false; break;
            }

            // 2. Build frame (capacity sized so total = k_bytes)
            FrameBuilder fb(frame_capacity);
            if (!fb.addPacket(0, opus_pkt.data(), opus_pkt.size())) {
                char msg[96];
                snprintf(msg, sizeof(msg), "Packet too large: %zu bytes, capacity=%zu",
                         opus_pkt.size(), frame_capacity);
                FAIL(msg); chain_ok = false; break;
            }
            auto frame_data = fb.build(static_cast<uint32_t>(f), fec_rate,
                                       ofdm_p.modulation);

            // Frame should be exactly k_bytes
            if (frame_data.size() != k_bytes) {
                // Pad if smaller (shouldn't happen with correct capacity)
                frame_data.resize(k_bytes, 0);
            }

            // 3. LDPC encode
            std::vector<uint8_t> codeword(n_bytes, 0);
            ldpc_enc.encode(frame_data.data(), codeword.data());

            // 4. Interleave
            std::vector<uint8_t> interleaved(n_bytes, 0);
            intlv.interleave(codeword.data(), interleaved.data());

            // 5. OFDM modulate
            ComplexBuf tx_samples;
            omod.modulateBits(interleaved.data(), n, tx_samples);

            // === PERFECT CHANNEL ===

            // === RX ===
            // 6. OFDM demodulate + soft demap
            std::vector<float> all_llrs;
            for (size_t base = 0; base + sym_len <= tx_samples.size();
                 base += sym_len) {
                ComplexBuf one(
                    tx_samples.begin() + static_cast<ptrdiff_t>(base),
                    tx_samples.begin() + static_cast<ptrdiff_t>(base + sym_len));
                std::vector<float> llrs;
                odemod.demodulateSoft(one, llrs, 0.01f);
                all_llrs.insert(all_llrs.end(), llrs.begin(), llrs.end());
            }
            all_llrs.resize(n, 0.f);

            // 7. Deinterleave
            std::vector<float> deintlv(n);
            intlv.deinterleave(all_llrs.data(), deintlv.data());

            // 8. LDPC decode
            std::vector<uint8_t> decoded_frame(k_bytes, 0);
            auto ldpc_res = ldpc_dec.decode(deintlv.data(), decoded_frame.data());

            if (!ldpc_res.converged) {
                FAIL("LDPC did not converge"); chain_ok = false; break;
            }

            // 9. Parse frame
            ParsedFrame parsed;
            FrameParser::parse(decoded_frame, parsed);
            if (!parsed.valid || !parsed.crc_ok || parsed.packets.empty()) {
                char msg[96];
                snprintf(msg, sizeof(msg),
                         "Frame parse fail: valid=%d crc=%d pkts=%zu",
                         parsed.valid ? 1 : 0, parsed.crc_ok ? 1 : 0,
                         parsed.packets.size());
                FAIL(msg); chain_ok = false; break;
            }

            // 10. Opus decode
            std::vector<float> pcm_out;
            size_t dec_n = odec.decode(parsed.packets[0].data.data(),
                                        parsed.packets[0].data.size(), pcm_out);
            if (dec_n != audio_frame_samples) {
                char msg[64];
                snprintf(msg, sizeof(msg), "Opus decoded %zu, expected %zu",
                         dec_n, audio_frame_samples);
                FAIL(msg); chain_ok = false; break;
            }

            all_decoded_audio.insert(all_decoded_audio.end(),
                                      pcm_out.begin(), pcm_out.end());
        }

        if (chain_ok) {
            float snr = computeAlignedSNR(pcm_orig.data(), pcm_orig.size(),
                                           all_decoded_audio.data(),
                                           all_decoded_audio.size());
            if (snr > 20.0f) {
                PASS();
            } else {
                char msg[64];
                snprintf(msg, sizeof(msg), "Audio SNR=%.1f dB (expected >20)", snr);
                FAIL(msg);
            }
        }
    }

    // =====================================================================
    // Test 6: Full chain with AWGN at multiple SNR levels
    // =====================================================================
    printf("\n  --- Full Chain Audio Quality vs Channel SNR ---\n");
    {
        OpusConfig ocfg;
        ocfg.sample_rate = 48000;
        ocfg.channels = 1;
        ocfg.bitrate = 32000;
        ocfg.frame_ms = 20.0f;
        ocfg.vbr = false;

        FECRate fec_rate = FECRate::Rate_1_2;
        LDPCBlockSize blk = LDPCBlockSize::Short;

        OFDMParams ofdm_p;
        ofdm_p.fft_size = 256;
        ofdm_p.modulation = Modulation::QPSK;
        ofdm_p.sample_rate = 48000;

        float snrs[] = {4.f, 6.f, 8.f, 10.f, 15.f};

        for (float snr_db : snrs) {
            OpusAudioEncoder oenc(ocfg);
            OpusAudioDecoder odec(ocfg);
            LDPCEncoder ldpc_enc(fec_rate, blk);
            LDPCDecoder ldpc_dec(fec_rate, blk, 50);
            BitInterleaver intlv(ldpc_enc.codewordBits());
            OFDMModulator   omod(ofdm_p);
            OFDMDemodulator odemod(ofdm_p);

            size_t audio_frame_samples = ocfg.frameSamples();
            size_t n = ldpc_enc.codewordBits();
            size_t k_bytes = ldpc_enc.infoBytes();
            size_t n_bytes = ldpc_enc.codewordBytes();
            size_t frame_capacity = k_bytes - constants::FRAME_OVERHEAD;
            size_t sym_len = ofdm_p.symbolLength();

            std::mt19937 rng(static_cast<unsigned>(snr_db * 100));

            // Preamble with AWGN
            ComplexBuf preamble = omod.generatePreamble();
            size_t short_total = 10 * (ofdm_p.fft_size / 4);
            size_t long_total  = 2 * ofdm_p.symbolLength();
            ComplexBuf pre_noisy = preamble;
            addAWGN(pre_noisy, snr_db + 3.f, rng);
            if (pre_noisy.size() >= short_total + long_total) {
                ComplexBuf lsyms(
                    pre_noisy.begin() + static_cast<ptrdiff_t>(short_total),
                    pre_noisy.begin() + static_cast<ptrdiff_t>(short_total + long_total));
                odemod.processPreamble(lsyms);
            }

            size_t num_frames = 12;
            auto pcm_orig = generateSine(1000.0f, 48000.0f,
                                          audio_frame_samples * num_frames);

            std::vector<float> all_decoded_audio;
            size_t frames_ok = 0;
            size_t frames_failed = 0;

            for (size_t f = 0; f < num_frames; ++f) {
                // TX
                std::vector<uint8_t> opus_pkt;
                oenc.encode(pcm_orig.data() + f * audio_frame_samples, opus_pkt);

                FrameBuilder fb(frame_capacity);
                fb.addPacket(0, opus_pkt.data(), opus_pkt.size());
                auto frame_data = fb.build(static_cast<uint32_t>(f), fec_rate,
                                           ofdm_p.modulation);
                frame_data.resize(k_bytes, 0);

                std::vector<uint8_t> codeword(n_bytes, 0);
                ldpc_enc.encode(frame_data.data(), codeword.data());

                std::vector<uint8_t> interleaved(n_bytes, 0);
                intlv.interleave(codeword.data(), interleaved.data());

                ComplexBuf tx_samples;
                omod.modulateBits(interleaved.data(), n, tx_samples);

                // Channel: AWGN
                addAWGN(tx_samples, snr_db, rng);

                // RX
                std::vector<float> all_llrs;
                float noise_est = std::pow(10.f, -snr_db / 10.f);
                for (size_t base = 0; base + sym_len <= tx_samples.size();
                     base += sym_len) {
                    ComplexBuf one(
                        tx_samples.begin() + static_cast<ptrdiff_t>(base),
                        tx_samples.begin() + static_cast<ptrdiff_t>(base + sym_len));
                    std::vector<float> llrs;
                    odemod.demodulateSoft(one, llrs, noise_est);
                    all_llrs.insert(all_llrs.end(), llrs.begin(), llrs.end());
                }
                all_llrs.resize(n, 0.f);

                std::vector<float> deintlv(n);
                intlv.deinterleave(all_llrs.data(), deintlv.data());

                std::vector<uint8_t> decoded_frame(k_bytes, 0);
                auto ldpc_res = ldpc_dec.decode(deintlv.data(),
                                                 decoded_frame.data());

                if (!ldpc_res.converged) {
                    frames_failed++;
                    std::vector<float> plc;
                    odec.decodePLC(plc);
                    all_decoded_audio.insert(all_decoded_audio.end(),
                                              plc.begin(), plc.end());
                    continue;
                }

                ParsedFrame parsed;
                FrameParser::parse(decoded_frame, parsed);
                if (!parsed.valid || !parsed.crc_ok || parsed.packets.empty()) {
                    frames_failed++;
                    std::vector<float> plc;
                    odec.decodePLC(plc);
                    all_decoded_audio.insert(all_decoded_audio.end(),
                                              plc.begin(), plc.end());
                    continue;
                }

                std::vector<float> pcm_out;
                size_t dec_n = odec.decode(parsed.packets[0].data.data(),
                                            parsed.packets[0].data.size(), pcm_out);

                if (dec_n == audio_frame_samples) {
                    all_decoded_audio.insert(all_decoded_audio.end(),
                                              pcm_out.begin(), pcm_out.end());
                    frames_ok++;
                } else {
                    frames_failed++;
                    std::vector<float> plc;
                    odec.decodePLC(plc);
                    all_decoded_audio.insert(all_decoded_audio.end(),
                                              plc.begin(), plc.end());
                }
            }

            float audio_snr = 0.f;
            if (frames_ok > 0) {
                audio_snr = computeAlignedSNR(
                    pcm_orig.data(), pcm_orig.size(),
                    all_decoded_audio.data(), all_decoded_audio.size());
            }

            char line[120];
            snprintf(line, sizeof(line),
                     "  Ch SNR=%4.1f dB: %zu/%zu frames OK, audio SNR=%.1f dB %s",
                     snr_db, frames_ok, num_frames, audio_snr,
                     (frames_ok == num_frames) ? "(all clean)" : "");
            printf("%s\n", line);
        }
    }

    // =====================================================================
    // Summary
    // =====================================================================
    printf("\n=== Results: %d passed, %d failed ===\n",
           tests_passed, tests_failed);

    if (tests_failed == 0) {
        printf("\n>>> ALL CODEC TESTS PASSED <<<\n");
    }

    return tests_failed > 0 ? 1 : 0;
}
