/**
 * @file gw_decode.cpp
 * @brief CLI: Groundwave baseband float file → WAV audio.
 *
 * Inverse of gw_encode. Reads passband floats, downconverts to baseband,
 * runs OFDM demodulation, soft demap, deinterleave, LDPC decode, frame
 * parse, Opus decode, and writes WAV.
 *
 * Usage:
 *   gw_decode -i input.raw -o output.wav [options]
 */
#include "wav_io.hpp"
#include "types.hpp"
#include "ofdm.hpp"
#include "frame.hpp"
#include "ldpc.hpp"
#include "interleaver.hpp"
#include "opus_codec.hpp"
#include "iq_converter.hpp"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

using namespace gw;

namespace {

void usage() {
    std::fprintf(stderr,
        "gw_decode -i in.raw -o out.wav [-m mod] [-f rate] [-F fft] [-s sr] [-c freq]\n"
        "Note: parameters must match those used at encode time.\n");
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

// MUST match gw_encode's parseFec exactly — the previous decoder table
// listed only 5 of the 11 rates and silently fell back to 1/2, so a stream
// encoded at e.g. 2/3 decoded at the wrong rate and every frame failed CRC
// with no warning.
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

} // anonymous

int main(int argc, char** argv) {
    std::string in_path, out_path;
    Modulation mod = Modulation::QPSK;
    FECRate    fec = FECRate::Rate_1_2;
    uint32_t   fft_size = 256;
    uint32_t   sample_rate = 48000;
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
        else if (a == "-c" && i + 1 < argc) center_hz = static_cast<float>(std::atof(argv[++i]));
        else if (a == "-h" || a == "--help") { usage(); return 0; }
    }
    if (in_path.empty() || out_path.empty()) { usage(); return 1; }

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
    // Center frequency must sit inside (0, Nyquist) — the IQ converter
    // ctor throws otherwise (uncaught → terminate, no message).
    const float nyquist = static_cast<float>(sample_rate) / 2.f;
    if (!(center_hz > 0.f && center_hz < nyquist)) {
        std::fprintf(stderr, "Error: center freq %.0f Hz must be inside "
                     "(0, %.0f) at this sample rate.\n",
                     static_cast<double>(center_hz),
                     static_cast<double>(nyquist));
        return 1;
    }

    // Slurp passband file (64-bit safe; check ftell and the short-read).
    FILE* fin = std::fopen(in_path.c_str(), "rb");
    if (!fin) { std::fprintf(stderr, "Cannot open %s\n", in_path.c_str()); return 2; }
    std::fseek(fin, 0, SEEK_END);
    long long fsize = static_cast<long long>(
#ifdef _WIN32
        _ftelli64(fin)
#else
        ftello(fin)
#endif
    );
    std::fseek(fin, 0, SEEK_SET);
    if (fsize < 0) {
        std::fprintf(stderr, "Error: cannot determine size of %s\n", in_path.c_str());
        std::fclose(fin);
        return 2;
    }
    std::vector<float> passband(static_cast<size_t>(fsize) / sizeof(float));
    size_t got = std::fread(passband.data(), sizeof(float), passband.size(), fin);
    std::fclose(fin);
    // Resize to what was actually read so a short read doesn't feed a
    // zero-padded tail into the demodulator.
    passband.resize(got);

    // IQ downconvert. Mirror gw_encode's bandwidth derivation so the
    // OFDM allocation (target_bw_hz) and the LPF agree on both ends.
    const float signal_bw = 2.f * 0.8f * std::min(center_hz, nyquist - center_hz);
    constexpr size_t DN_TAPS = 65;
    IQDownconverter dn(sample_rate, center_hz, signal_bw * 0.5f, DN_TAPS);
    ComplexBuf bb;
    dn.downconvert(passband.data(), passband.size(), bb);

    // The linear-phase FIR delays the stream by (taps-1)/2 samples; the
    // encoder writes symbol-aligned passband with no TX filter, so skip
    // exactly the group delay to restore symbol alignment.
    constexpr size_t GROUP_DELAY = (DN_TAPS - 1) / 2;
    if (bb.size() > GROUP_DELAY)
        bb.erase(bb.begin(), bb.begin() + static_cast<ptrdiff_t>(GROUP_DELAY));

    // OFDM demodulate, run frame chain. Note: this is a simplified offline
    // pipeline assuming sample-aligned frames; a real receiver would do
    // sync first via OFDMSynchronizer. For loopback verification this is fine.
    OFDMParams ofdm;
    ofdm.fft_size    = static_cast<uint16_t>(fft_size);
    ofdm.modulation  = mod;
    ofdm.sample_rate = sample_rate;
    ofdm.target_bw_hz = signal_bw;
    OFDMDemodulator demod(ofdm);
    OFDMModulator   mod_ref(ofdm);   // for bitsPerOFDMSymbol (TX geometry)

    LDPCDecoder ldpc(fec, LDPCBlockSize::Short);
    BitInterleaver inter(ldpc.codewordBits());

    // Opus decodes to any legal Opus rate regardless of the encoder's
    // input rate; use 48 kHz so the output WAV is always well-formed
    // (the RF sample rate is unrelated to the audio rate).
    constexpr uint32_t AUDIO_RATE = 48000;
    OpusConfig oc;
    oc.sample_rate = AUDIO_RATE;
    oc.channels    = 1;
    oc.bitrate     = 32000;
    oc.frame_ms    = 20.f;
    OpusAudioDecoder dec(oc);

    // Each codeword spans ceil(codewordBits / bitsPerOFDMSymbol) whole OFDM
    // symbols (modulateBits zero-fills the last). Aggregate the LLRs across
    // that span before deinterleave + LDPC decode. (The previous loop
    // demodulated ONE symbol, found llrs.size() < codewordBits, and skipped
    // it — every symbol, every time: the tool could never decode anything.)
    const size_t cw_bits     = ldpc.codewordBits();
    const size_t bps_per_sym = mod_ref.bitsPerOFDMSymbol();
    if (bps_per_sym == 0) {
        std::fprintf(stderr, "Error: degenerate OFDM config (no data bins)\n");
        return 1;
    }
    const size_t syms_per_cw    = (cw_bits + bps_per_sym - 1) / bps_per_sym;
    const size_t sym_len        = ofdm.symbolLength();
    const size_t samples_per_cw = syms_per_cw * sym_len;

    std::vector<float> all_pcm;
    size_t frames_ok = 0, frames_bad = 0;
    size_t pos = 0;
    while (pos + samples_per_cw <= bb.size()) {
        std::vector<float> all_llrs;
        all_llrs.reserve(syms_per_cw * bps_per_sym);
        for (size_t s = 0; s < syms_per_cw; ++s) {
            ComplexBuf sym(
                bb.begin() + static_cast<ptrdiff_t>(pos + s * sym_len),
                bb.begin() + static_cast<ptrdiff_t>(pos + (s + 1) * sym_len));
            std::vector<float> llrs;
            demod.demodulateSoft(sym, llrs, demod.noiseVariance());
            all_llrs.insert(all_llrs.end(), llrs.begin(), llrs.end());
        }
        pos += samples_per_cw;

        all_llrs.resize(cw_bits, 0.f);
        std::vector<float> deint(cw_bits);
        inter.deinterleave(all_llrs.data(), deint.data());

        std::vector<uint8_t> info((ldpc.infoBits() + 7) / 8);
        ldpc.decode(deint.data(), info.data());

        // Parse frame and decode Opus
        ParsedFrame pf;
        if (FrameParser::parse(info.data(), info.size(), pf) && pf.crc_ok) {
            ++frames_ok;
            for (const auto& pkt : pf.packets) {
                std::vector<float> pcm;
                size_t n = dec.decode(pkt.data.data(), pkt.data.size(), pcm);
                if (n > 0) all_pcm.insert(all_pcm.end(), pcm.begin(), pcm.end());
            }
        } else {
            ++frames_bad;
        }
    }

    std::fprintf(stderr, "Frames: %zu ok, %zu failed CRC\n",
                 frames_ok, frames_bad);
    if (all_pcm.empty()) {
        std::fprintf(stderr, "No frames decoded successfully\n");
        return 3;
    }
    if (!wav::writeFloat16(out_path, all_pcm.data(), all_pcm.size(),
                           AUDIO_RATE, 1)) {
        std::fprintf(stderr, "Failed to write WAV\n");
        return 4;
    }
    std::fprintf(stderr, "Decoded %zu samples → %s\n", all_pcm.size(), out_path.c_str());
    return 0;
}
