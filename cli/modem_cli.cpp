/**
 * @file modem_cli.cpp
 * @brief CLI: live soundcard modem with virtual-cable loopback support.
 *
 * Modes:
 *   internal — software loopback (no audio hardware), TX feeds RX in memory.
 *   tx       — modulate test tone, push to playback device.
 *   rx       — capture from input device, demodulate, decode.
 *   vcable   — full TX→audio-out→audio-in→RX loop using HW devices. Wire
 *              your virtual cable's "playback" to its "recording" mate
 *              (e.g. VB-Audio CABLE Output → CABLE Input). Then pick the
 *              cable as both playback and capture device with --pb / --cap.
 *
 * Examples:
 *   List devices:
 *     dsca_modem --list
 *   Software loopback BER test (no hardware):
 *     dsca_modem --mode internal -d 10 -m qpsk
 *   Virtual cable end-to-end:
 *     dsca_modem --mode vcable --pb 5 --cap 3 -d 30 -m qam16
 */
#include "types.hpp"
#include "soundcard_modem.hpp"
#include "ofdm.hpp"
#include "frame.hpp"
#include "ldpc.hpp"
#include "interleaver.hpp"
#include "opus_codec.hpp"
#include "snr_calculator.hpp"   // modulationName / fecRateName

#ifdef DSCA_ENABLE_AUDIO
#include "hw_audio.hpp"
#endif

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <atomic>

using namespace dsca;

namespace {

void usage() {
    std::fprintf(stderr,
        "dsca_modem [--mode internal|tx|rx|vcable] [options]\n"
        "  --list             List available audio devices and exit\n"
        "  --mode <mode>      internal | tx | rx | vcable  (default: internal)\n"
        "  --pb <id>          Playback device index (HW modes)\n"
        "  --cap <id>         Capture  device index (HW modes)\n"
        "  -d N               Duration in seconds (default 10)\n"
        "  -c freq            Center freq Hz (default 12000)\n"
        "  -s sr              Sample rate Hz (default 48000)\n"
        "  -F fft             FFT size (default 256)\n"
        "  -m mod             bpsk|qpsk|qam16|qam64|qam256|qam1024|qam4096\n"
        "  -f fec             1/4 1/3 2/5 1/2 3/5 2/3 3/4 4/5 5/6 8/9 9/10 (default 1/2)\n"
        "  --bw hz            Occupied signal bandwidth (default: auto-fit to\n"
        "                     80%% of the passband around the center freq)\n"
        "  --tone hz          TX test-tone frequency (default 1000)\n"
        "  --snr db           Add AWGN to internal loopback (default: clean)\n");
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

void listDevices() {
#ifdef DSCA_ENABLE_AUDIO
    HWAudioDevice dev;
    if (!dev.init()) {
        std::fprintf(stderr, "Failed to init audio context\n");
        return;
    }
    std::printf("\nPlayback devices (use with --pb):\n");
    for (size_t i = 0; i < dev.playbackDevices().size(); ++i) {
        const auto& d = dev.playbackDevices()[i];
        std::printf("  [%zu] %s%s\n", i, d.name.c_str(),
                    d.is_default ? " (default)" : "");
    }
    std::printf("\nCapture devices (use with --cap):\n");
    for (size_t i = 0; i < dev.captureDevices().size(); ++i) {
        const auto& d = dev.captureDevices()[i];
        std::printf("  [%zu] %s%s\n", i, d.name.c_str(),
                    d.is_default ? " (default)" : "");
    }
    std::printf("\nFor a virtual-cable test, install VB-CABLE (Windows) or\n"
                "create a 'pcm.loopback' alsa-loopback device (Linux), then\n"
                "use the same cable as both pb and cap.\n");
#else
    std::fprintf(stderr,
        "Hardware audio support not built in.\n"
        "Re-configure with -DDSCA_ENABLE_AUDIO=ON.\n");
#endif
}

// Generate a steady test tone on the TX side (mono float)
void fillTone(float* pcm, size_t n, float freq_hz, float fs, float& phase) {
    const float two_pi = 2.f * static_cast<float>(M_PI);
    const float dphi = two_pi * freq_hz / fs;
    for (size_t i = 0; i < n; ++i) {
        pcm[i] = 0.5f * std::sin(phase);
        phase += dphi;
        if (phase > two_pi) phase -= two_pi;
    }
}

} // anonymous

int main(int argc, char** argv) {
    std::string mode = "internal";
    int duration_sec = 10;
    uint32_t sample_rate = 48000;
    float    center_hz   = 12000.f;
    uint32_t fft_size    = 256;
    Modulation mod  = Modulation::QPSK;
    FECRate    fec  = FECRate::Rate_1_2;
    int  pb_dev   = -1;
    int  cap_dev  = -1;
    float tone_hz = 1000.f;
    // Negative SNRs are legitimate test points (BPSK 1/4 closes below
    // 0 dB), so "off" needs a separate flag rather than a -1 sentinel.
    bool  awgn_enabled = false;
    float awgn_snr_db  = 0.f;
    float signal_bw_hz = 0.f;   // 0 = auto-fit to the passband
    bool list_only = false;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--list")              { list_only = true; }
        else if (a == "--mode" && i + 1 < argc)  mode = argv[++i];
        else if (a == "--pb"   && i + 1 < argc)  pb_dev = std::atoi(argv[++i]);
        else if (a == "--cap"  && i + 1 < argc)  cap_dev = std::atoi(argv[++i]);
        else if (a == "--bw"   && i + 1 < argc)  signal_bw_hz = static_cast<float>(std::atof(argv[++i]));
        else if (a == "--tone" && i + 1 < argc)  tone_hz = static_cast<float>(std::atof(argv[++i]));
        else if (a == "--snr"  && i + 1 < argc) {
            awgn_snr_db  = static_cast<float>(std::atof(argv[++i]));
            awgn_enabled = true;
        }
        else if (a == "-d" && i + 1 < argc) duration_sec = std::atoi(argv[++i]);
        else if (a == "-c" && i + 1 < argc) center_hz = static_cast<float>(std::atof(argv[++i]));
        else if (a == "-s" && i + 1 < argc) sample_rate = static_cast<uint32_t>(std::atoi(argv[++i]));
        else if (a == "-F" && i + 1 < argc) fft_size = static_cast<uint32_t>(std::atoi(argv[++i]));
        else if (a == "-m" && i + 1 < argc) {
            bool ok; mod = parseMod(argv[++i], ok);
            if (!ok) { std::fprintf(stderr, "Error: unknown modulation '%s'\n", argv[i]); return 1; }
        }
        else if (a == "-f" && i + 1 < argc) {
            bool ok; fec = parseFec(argv[++i], ok);
            if (!ok) { std::fprintf(stderr, "Error: unknown FEC rate '%s'\n", argv[i]); return 1; }
        }
        else if (a == "-h" || a == "--help") { usage(); return 0; }
    }

    if (list_only) {
        listDevices();
        return 0;
    }

    // Validate the mode before doing anything: an unknown mode otherwise
    // falls through every branch and prints a bogus "RX OK 0 (0.0%)".
    if (mode != "internal" && mode != "tx" && mode != "rx" && mode != "vcable") {
        std::fprintf(stderr, "Error: unknown --mode '%s' "
                     "(expected internal|tx|rx|vcable)\n", mode.c_str());
        usage();
        return 1;
    }
    // The DSP / Opus constructors throw on invalid params (uncaught →
    // terminate). modem_cli couples the Opus rate to the modem rate, so the
    // sample rate must be one Opus accepts.
    if (!(fft_size >= 64 && fft_size <= 16384 && (fft_size & (fft_size - 1)) == 0)) {
        std::fprintf(stderr, "Error: FFT size %u must be a power of two in [64, 16384].\n",
                     fft_size);
        return 1;
    }
    if (!(sample_rate == 8000 || sample_rate == 12000 || sample_rate == 16000 ||
          sample_rate == 24000 || sample_rate == 48000)) {
        std::fprintf(stderr, "Error: sample rate %u Hz must be an Opus rate "
                     "(8000/12000/16000/24000/48000).\n", sample_rate);
        return 1;
    }

    // Occupied bandwidth must fit inside (0, Nyquist) around the center
    // frequency, and the OFDM allocation must be told about it
    // (target_bw_hz) so the active subcarriers stay inside the up/down-
    // converter LPFs. This mirrors the contract the GUI engine honors
    // (audio_engine sets ofdm.target_bw_hz = modem.signal_bw); without it
    // the default 5%-per-side guards span far more than the passband and
    // the loopback LPF destroys the signal — 0% decode on a clean channel.
    const float nyquist = static_cast<float>(sample_rate) / 2.f;
    if (!(center_hz > 0.f && center_hz < nyquist)) {
        std::fprintf(stderr, "Error: center freq %.0f Hz must be inside "
                     "(0, %.0f) at this sample rate.\n",
                     static_cast<double>(center_hz),
                     static_cast<double>(nyquist));
        return 1;
    }
    if (signal_bw_hz <= 0.f) {   // auto: 80% of the symmetric fit
        signal_bw_hz = 2.f * 0.8f * std::min(center_hz, nyquist - center_hz);
    }
    if (center_hz - signal_bw_hz * 0.5f <= 0.f ||
        center_hz + signal_bw_hz * 0.5f >= nyquist) {
        std::fprintf(stderr, "Error: bandwidth %.0f Hz does not fit in "
                     "(0, %.0f) around center %.0f Hz.\n",
                     static_cast<double>(signal_bw_hz),
                     static_cast<double>(nyquist),
                     static_cast<double>(center_hz));
        return 1;
    }

    OFDMParams ofdm;
    ofdm.fft_size    = static_cast<uint16_t>(fft_size);
    ofdm.modulation  = mod;
    ofdm.sample_rate = sample_rate;
    ofdm.target_bw_hz = signal_bw_hz;

    ModemConfig mcfg;
    mcfg.sample_rate = sample_rate;
    mcfg.center_freq = center_hz;
    mcfg.signal_bw   = signal_bw_hz;
    mcfg.loopback    = (mode == "internal");

    SoundcardModem modem(mcfg, ofdm);

    LDPCEncoder ldpc(fec, LDPCBlockSize::Short);
    LDPCDecoder ldpc_dec(fec, LDPCBlockSize::Short, 50);
    BitInterleaver inter(ldpc.codewordBits());

    OpusConfig oc;
    oc.sample_rate = sample_rate;
    oc.channels = 1;
    oc.bitrate = 24000;
    oc.frame_ms = 20.f;
    OpusAudioEncoder opus_enc(oc);

    OFDMModulator ofdm_mod(ofdm);
    OFDMDemodulator ofdm_demod(ofdm);
    OFDMSynchronizer ofdm_sync(ofdm);

    // Hardware audio: needed for tx, rx, vcable modes
#ifdef DSCA_ENABLE_AUDIO
    HWAudioDevice hw;
    bool hw_started = false;
    if (mode == "tx" || mode == "rx" || mode == "vcable") {
        if (!hw.init()) {
            std::fprintf(stderr, "Failed to init audio (run with --list)\n");
            return 1;
        }
        HWAudioConfig hcfg;
        hcfg.sample_rate     = sample_rate;
        hcfg.buffer_frames   = 1024;
        hcfg.playback_device = pb_dev;
        hcfg.capture_device  = cap_dev;
        if (!hw.start(modem.txRing(), modem.rxRing(), hcfg)) {
            std::fprintf(stderr, "Failed to start audio device\n");
            return 1;
        }
        hw_started = true;
        std::fprintf(stderr,
            "HW audio: pb=%s cap=%s @ %u Hz, buffer=%u frames\n",
            pb_dev < 0 ? "default" : std::to_string(pb_dev).c_str(),
            cap_dev < 0 ? "default" : std::to_string(cap_dev).c_str(),
            sample_rate, hcfg.buffer_frames);
    }
#else
    if (mode != "internal") {
        std::fprintf(stderr,
            "Mode '%s' requires hardware audio. Re-build with -DDSCA_ENABLE_AUDIO=ON.\n",
            mode.c_str());
        return 1;
    }
#endif

    std::fprintf(stderr,
        "DSCA-NG modem CLI\n"
        "  mode      : %s\n"
        "  duration  : %d s\n"
        "  modulation: %s\n"
        "  fec       : %s\n"
        "  fft size  : %u\n"
        "  center    : %.0f Hz @ %u Hz sample rate\n"
        "  bandwidth : %.0f Hz\n",
        mode.c_str(), duration_sec, modulationName(mod), fecRateName(fec),
        fft_size, static_cast<double>(center_hz), sample_rate,
        static_cast<double>(signal_bw_hz));

    auto start = std::chrono::steady_clock::now();
    uint32_t frame_no = 0;
    uint64_t frames_tx = 0, frames_rx = 0, frames_ok = 0;
    bool preamble_sent = false;
    bool preamble_rx = false;
    ComplexBuf rx_accum;
    float tone_phase = 0.f;
    size_t frame_samples = oc.frameSamples();

    while (true) {
        auto elapsed_s = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - start).count();
        if (elapsed_s >= duration_sec) break;

        // ---- TX path (all modes except pure rx) ----
        if (mode == "tx" || mode == "vcable" || mode == "internal") {
            if (!preamble_sent) {
                modem.transmit(ofdm_mod.generatePreamble());
                preamble_sent = true;
            }

            std::vector<float> pcm(frame_samples);
            fillTone(pcm.data(), pcm.size(), tone_hz,
                     static_cast<float>(sample_rate), tone_phase);

            std::vector<uint8_t> opus_pkt;
            if (!opus_enc.encode(pcm.data(), opus_pkt)) continue;

            FrameBuilder fb(ldpc.infoBytes() > 32 ? ldpc.infoBytes() - 32 : 0);
            fb.addPacket(0, opus_pkt.data(), opus_pkt.size());
            auto frame = fb.build(frame_no++, fec, mod);
            frame.resize(ldpc.infoBytes(), 0);

            std::vector<uint8_t> cw(ldpc.codewordBytes(), 0);
            ldpc.encode(frame.data(), cw.data());
            std::vector<uint8_t> ileaved(cw.size(), 0);
            inter.interleave(cw.data(), ileaved.data());

            ComplexBuf bb;
            ofdm_mod.modulateBits(ileaved.data(), ldpc.codewordBits(), bb);
            modem.transmit(bb);
            frames_tx++;
        }

        // ---- Internal-loopback: inject AWGN if requested ----
        if (mode == "internal" && awgn_enabled) {
            modem.processLoopbackAWGN(awgn_snr_db, frame_no);
        }

        // ---- RX path ----
        if (mode == "rx" || mode == "vcable" || mode == "internal") {
            ComplexBuf rx = modem.receive(sample_rate / 5);
            if (!rx.empty()) {
                rx_accum.insert(rx_accum.end(), rx.begin(), rx.end());

                // Preamble acquisition. The received stream is NOT front-
                // aligned: the TX and RX LPFs (129 taps each by default)
                // contribute ~2x64 samples of group delay, so the preamble
                // sits an arbitrary-but-fixed offset into the stream. Mirror
                // the engine's ACQUIRE path: cross-correlate against the known
                // Zadoff-Chu long-preamble body over the whole buffer
                // (fineSync wide scan), then back out the preamble start
                // (body - CP - short preamble) and hand processPreamble the
                // two long symbols at their true position.
                if (!preamble_rx) {
                    const size_t cp          = ofdm.cpLength();
                    const size_t short_total = 10 * (ofdm.fft_size / 4);
                    const size_t long_total  = 2 * ofdm.symbolLength();
                    const size_t need        = short_total + long_total;
                    // A little slack beyond the nominal length so the wide
                    // scan window actually contains the delayed preamble.
                    if (rx_accum.size() >= need + ofdm.symbolLength()) {
                        SyncResult fr;
                        int centre = static_cast<int>(rx_accum.size() / 2);
                        if (ofdm_sync.fineSync(rx_accum, centre, fr,
                                               rx_accum.size()) && fr.valid) {
                            long ps = static_cast<long>(fr.timing_offset)
                                    - static_cast<long>(cp)
                                    - static_cast<long>(short_total);
                            if (ps < 0) ps = 0;
                            if (static_cast<size_t>(ps) + need > rx_accum.size())
                                ps = static_cast<long>(rx_accum.size() - need);
                            const size_t pre_start = static_cast<size_t>(ps);
                            ComplexBuf longp(
                                rx_accum.begin() + static_cast<ptrdiff_t>(
                                    pre_start + short_total),
                                rx_accum.begin() + static_cast<ptrdiff_t>(
                                    pre_start + need));
                            if (ofdm_demod.processPreamble(longp)) {
                                preamble_rx = true;
                                rx_accum.erase(rx_accum.begin(),
                                    rx_accum.begin() + static_cast<ptrdiff_t>(
                                        pre_start + need));
                            }
                        }
                    }
                }

                if (preamble_rx) {
                    size_t bps_per_sym = ofdm_mod.bitsPerOFDMSymbol();
                    if (bps_per_sym > 0) {
                        size_t syms_per_cw =
                            (ldpc.codewordBits() + bps_per_sym - 1) / bps_per_sym;
                        size_t samples_per_cw = syms_per_cw * ofdm.symbolLength();
                        while (rx_accum.size() >= samples_per_cw) {
                            std::vector<float> all_llrs;
                            for (size_t b = 0;
                                 b + ofdm.symbolLength() <= samples_per_cw;
                                 b += ofdm.symbolLength()) {
                                ComplexBuf one(rx_accum.begin() + static_cast<ptrdiff_t>(b),
                                    rx_accum.begin() + static_cast<ptrdiff_t>(b + ofdm.symbolLength()));
                                std::vector<float> llrs;
                                ofdm_demod.demodulateSoft(one, llrs,
                                    ofdm_demod.noiseVariance());
                                all_llrs.insert(all_llrs.end(),
                                    llrs.begin(), llrs.end());
                            }
                            all_llrs.resize(ldpc.codewordBits(), 0.f);
                            std::vector<float> deint(all_llrs.size());
                            inter.deinterleave(all_llrs.data(), deint.data());
                            std::vector<uint8_t> info(ldpc.infoBytes(), 0);
                            auto r = ldpc_dec.decode(deint.data(), info.data());
                            frames_rx++;
                            if (r.converged) frames_ok++;
                            rx_accum.erase(rx_accum.begin(),
                                rx_accum.begin() + static_cast<ptrdiff_t>(samples_per_cw));
                        }
                    }
                }
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(20));

        // Status line
        if ((frames_tx & 7) == 0) {
            std::fprintf(stderr,
                "\r[%3llds] TX=%llu RX=%llu OK=%llu  SNR=%.1f dB  AGC=%.1f dB    ",
                static_cast<long long>(elapsed_s),
                static_cast<unsigned long long>(frames_tx),
                static_cast<unsigned long long>(frames_rx),
                static_cast<unsigned long long>(frames_ok),
                static_cast<double>(ofdm_demod.snrEstimate()),
                static_cast<double>(modem.agcGainDB()));
            std::fflush(stderr);
        }
    }

#ifdef DSCA_ENABLE_AUDIO
    if (hw_started) hw.stop();
#endif

    std::fprintf(stderr,
        "\n--- Summary ---\n"
        "  TX frames : %llu\n"
        "  RX frames : %llu\n"
        "  RX OK     : %llu (%.1f%%)\n"
        "  Final SNR : %.1f dB\n",
        static_cast<unsigned long long>(frames_tx),
        static_cast<unsigned long long>(frames_rx),
        static_cast<unsigned long long>(frames_ok),
        frames_rx > 0
            ? 100.0 * static_cast<double>(frames_ok)
                    / static_cast<double>(frames_rx)
            : 0.0,
        static_cast<double>(ofdm_demod.snrEstimate()));

    return 0;
}
