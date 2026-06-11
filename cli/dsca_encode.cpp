/**
 * @file dsca_encode.cpp
 * @brief CLI: WAV audio → DSCA baseband file (complex float32 IQ).
 *
 * Reads a WAV input, encodes via Opus → frame builder → LDPC → interleaver
 * → symbol mapper → OFDM modulator → IQ upconverter, and writes the
 * resulting passband float samples to a `.raw` file.
 *
 * Usage:
 *   dsca_encode -i input.wav -o output.raw [options]
 *
 * Options:
 *   -m <bpsk|qpsk|qam16|qam64|qam256|qam1024|qam4096>
 *   -f <fec rate, e.g. 1/2, 3/4>
 *   -F <fft_size>
 *   -s <sample_rate>
 *   -b <opus bitrate bps>
 *   -c <center freq Hz>
 *   --headless  : do not print progress
 */
#include "wav_io.hpp"
#include <algorithm>
#include "types.hpp"
#include "ofdm.hpp"
#include "frame.hpp"
#include "ldpc.hpp"
#include "interleaver.hpp"
#include "opus_codec.hpp"
#include "iq_converter.hpp"

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include <cstdlib>

using namespace dsca;

namespace {

void usage() {
    std::fprintf(stderr,
        "dsca_encode -i in.wav -o out.raw [-m mod] [-f rate] [-F fft] [-s sr] [-b bps] [-c freq]\n"
        "  -m: bpsk|qpsk|qam16|qam64|qam256|qam1024|qam4096 (default: qpsk)\n"
        "  -f: 1/4 1/3 2/5 1/2 3/5 2/3 3/4 4/5 5/6 8/9 9/10 (default: 1/2)\n"
        "  -F: fft size, power of two (default: 256)\n"
        "  -s: sample rate Hz (default: 48000)\n"
        "  -b: opus bitrate bps (default: 24000)\n"
        "  -c: center frequency Hz for IQ upconvert (default: 12000)\n");
}

Modulation parseMod(const std::string& s, bool& ok) {
    ok = true;
    if (s == "bpsk") return Modulation::BPSK;
    if (s == "qpsk") return Modulation::QPSK;
    if (s == "qam16") return Modulation::QAM16;
    if (s == "qam64") return Modulation::QAM64;
    if (s == "qam256") return Modulation::QAM256;
    if (s == "qam1024") return Modulation::QAM1024;
    if (s == "qam4096") return Modulation::QAM4096;
    ok = false;
    return Modulation::QPSK;
}

FECRate parseFec(const std::string& s, bool& ok) {
    ok = true;
    if (s == "1/4")  return FECRate::Rate_1_4;
    if (s == "1/3")  return FECRate::Rate_1_3;
    if (s == "2/5")  return FECRate::Rate_2_5;
    if (s == "1/2")  return FECRate::Rate_1_2;
    if (s == "3/5")  return FECRate::Rate_3_5;
    if (s == "2/3")  return FECRate::Rate_2_3;
    if (s == "3/4")  return FECRate::Rate_3_4;
    if (s == "4/5")  return FECRate::Rate_4_5;
    if (s == "5/6")  return FECRate::Rate_5_6;
    if (s == "8/9")  return FECRate::Rate_8_9;
    if (s == "9/10") return FECRate::Rate_9_10;
    ok = false;
    return FECRate::Rate_1_2;
}

inline bool isPow2InRange(uint32_t n) {
    return n >= 64 && n <= 16384 && (n & (n - 1)) == 0;
}
inline bool isOpusRate(uint32_t r) {
    return r == 8000 || r == 12000 || r == 16000 || r == 24000 || r == 48000;
}

} // anonymous

int main(int argc, char** argv) {
    std::string in_path, out_path;
    Modulation mod = Modulation::QPSK;
    FECRate    fec = FECRate::Rate_1_2;
    uint32_t   fft_size = 256;
    uint32_t   sample_rate = 48000;
    uint32_t   opus_bps = 24000;
    float      center_hz = 12000.f;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "-i" && i + 1 < argc)      in_path = argv[++i];
        else if (a == "-o" && i + 1 < argc) out_path = argv[++i];
        else if (a == "-m" && i + 1 < argc) {
            bool ok; mod = parseMod(argv[++i], ok);
            if (!ok) { std::fprintf(stderr, "Error: unknown modulation '%s'\n", argv[i]); return 1; }
        }
        else if (a == "-f" && i + 1 < argc) {
            bool ok; fec = parseFec(argv[++i], ok);
            if (!ok) { std::fprintf(stderr, "Error: unknown FEC rate '%s'\n", argv[i]); return 1; }
        }
        else if (a == "-F" && i + 1 < argc) fft_size = static_cast<uint32_t>(std::atoi(argv[++i]));
        else if (a == "-s" && i + 1 < argc) sample_rate = static_cast<uint32_t>(std::atoi(argv[++i]));
        else if (a == "-b" && i + 1 < argc) opus_bps = static_cast<uint32_t>(std::atoi(argv[++i]));
        else if (a == "-c" && i + 1 < argc) center_hz = static_cast<float>(std::atof(argv[++i]));
        else if (a == "-h" || a == "--help") { usage(); return 0; }
    }
    if (in_path.empty() || out_path.empty()) { usage(); return 1; }

    // Validate numeric args BEFORE constructing the DSP objects, which
    // throw on invalid input (uncaught → std::terminate). Fail cleanly.
    if (!isPow2InRange(fft_size)) {
        std::fprintf(stderr, "Error: FFT size %u must be a power of two in [64, 16384].\n",
                     fft_size);
        return 1;
    }
    if (sample_rate < 8000 || sample_rate > 768000) {
        std::fprintf(stderr, "Error: sample rate %u Hz out of range [8000, 768000].\n",
                     sample_rate);
        return 1;
    }
    if (opus_bps < 6000 || opus_bps > 510000) {
        std::fprintf(stderr, "Error: opus bitrate %u out of range [6000, 510000].\n",
                     opus_bps);
        return 1;
    }
    // Center frequency must sit inside (0, Nyquist) — the IQUpconverter
    // ctor throws otherwise (uncaught → terminate, no message).
    const float nyquist = static_cast<float>(sample_rate) / 2.f;
    if (!(center_hz > 0.f && center_hz < nyquist)) {
        std::fprintf(stderr, "Error: center freq %.0f Hz must be inside "
                     "(0, %.0f) at this sample rate.\n",
                     static_cast<double>(center_hz),
                     static_cast<double>(nyquist));
        return 1;
    }

    // Read WAV
    std::vector<float> pcm;
    wav::WavInfo info{};
    if (!wav::readFloat(in_path, pcm, info)) {
        std::fprintf(stderr, "Failed to read WAV: %s\n", in_path.c_str());
        return 2;
    }
    std::fprintf(stderr, "Input: %u Hz, %u channels, %u frames, %u-bit %s\n",
                 info.sample_rate, info.channels, info.num_frames, info.bits,
                 info.is_float ? "float" : "PCM");

    // Opus only supports a fixed set of sample rates and 1-2 channels;
    // the OpusAudioEncoder ctor THROWS otherwise (uncaught → terminate).
    if (!isOpusRate(info.sample_rate)) {
        std::fprintf(stderr,
            "Error: input sample rate %u Hz is not supported by Opus "
            "(need 8000/12000/16000/24000/48000). Resample the WAV first.\n",
            info.sample_rate);
        return 2;
    }
    if (info.channels < 1 || info.channels > 2) {
        std::fprintf(stderr, "Error: input has %u channels; Opus needs 1 or 2.\n",
                     info.channels);
        return 2;
    }

    // Set up Opus
    OpusConfig oc;
    oc.sample_rate = info.sample_rate;
    oc.channels    = static_cast<uint8_t>(info.channels);
    oc.bitrate     = opus_bps;
    oc.frame_ms    = 20.f;
    OpusAudioEncoder enc(oc);
    size_t opus_frame_samples = oc.frameSamplesTotal();

    // Set up modem chain. target_bw_hz keeps the active subcarriers inside
    // the passband around center_hz (same derivation as dsca_decode, so the
    // two tools' allocations agree by default).
    OFDMParams ofdm;
    ofdm.fft_size    = static_cast<uint16_t>(fft_size);
    ofdm.modulation  = mod;
    ofdm.sample_rate = sample_rate;
    ofdm.target_bw_hz = 2.f * 0.8f * std::min(center_hz, nyquist - center_hz);
    OFDMModulator mod_engine(ofdm);

    LDPCEncoder ldpc(fec, LDPCBlockSize::Short);
    BitInterleaver inter(ldpc.codewordBits());

    // Up-convert real passband at center_hz
    IQUpconverter upconv(sample_rate, center_hz);

    FILE* fout = std::fopen(out_path.c_str(), "wb");
    if (!fout) { std::fprintf(stderr, "Cannot open %s\n", out_path.c_str()); return 3; }

    size_t pcm_pos = 0;
    uint32_t frame_no = 0;
    while (pcm_pos + opus_frame_samples <= pcm.size()) {
        // Encode one Opus frame
        std::vector<uint8_t> opus_packet;
        if (!enc.encode(pcm.data() + pcm_pos, opus_packet)) {
            std::fprintf(stderr, "Opus encode failed at frame %u\n", frame_no);
            break;
        }
        pcm_pos += opus_frame_samples;

        // Build frame, encode with LDPC. The FrameBuilder capacity is the
        // PAYLOAD size — build() wraps it in FRAME_OVERHEAD (sync + header
        // + CRC) more bytes. Passing infoBytes() as the capacity made every
        // built frame 24 bytes LONGER than the LDPC info block, and the
        // resize below chopped the tail off — including the CRC, so no
        // frame this tool ever produced could pass the decoder's check.
        const size_t payload_cap =
            ldpc.infoBytes() > constants::FRAME_OVERHEAD
                ? ldpc.infoBytes() - constants::FRAME_OVERHEAD : 0;
        FrameBuilder fb(payload_cap);
        fb.addPacket(0, opus_packet.data(), opus_packet.size());
        ByteVec raw = fb.build(frame_no++, fec, mod);
        raw.resize(ldpc.infoBytes(), 0);

        std::vector<uint8_t> codeword(ldpc.codewordBytes(), 0);
        ldpc.encode(raw.data(), codeword.data());

        std::vector<uint8_t> interleaved(ldpc.codewordBytes(), 0);
        inter.interleave(codeword.data(), interleaved.data());

        // OFDM modulate
        ComplexBuf bb;
        mod_engine.modulateBits(interleaved.data(), ldpc.codewordBits(), bb);

        // IQ upconvert to passband
        std::vector<float> passband;
        upconv.upconvert(bb, passband);
        std::fwrite(passband.data(), sizeof(float), passband.size(), fout);
    }

    std::fclose(fout);
    std::fprintf(stderr, "Encoded %u frames → %s\n", frame_no, out_path.c_str());
    return 0;
}
