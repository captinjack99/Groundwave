/**
 * @file opus_codec.cpp
 * @brief Opus audio codec wrapper implementation
 *
 * Uses the libopus C API with RAII wrappers and proper error handling.
 * All encoder/decoder state is heap-allocated by libopus; we just manage
 * the lifetime and translate between C++ and C calling conventions.
 */

#include "opus_codec.hpp"
#include <opus/opus.h>
#include <stdexcept>
#include <cstring>
#include <cstdio>

namespace gw {

// =========================================================================
// Helpers
// =========================================================================

namespace {

int toOpusApplication(OpusApplication app) {
    switch (app) {
        case OpusApplication::Audio:    return OPUS_APPLICATION_AUDIO;
        case OpusApplication::VoIP:     return OPUS_APPLICATION_VOIP;
        case OpusApplication::LowDelay: return OPUS_APPLICATION_RESTRICTED_LOWDELAY;
        default: return OPUS_APPLICATION_AUDIO;
    }
}

void validateConfig(const OpusConfig& cfg) {
    // Opus supports these sample rates
    if (cfg.sample_rate != 8000  && cfg.sample_rate != 12000 &&
        cfg.sample_rate != 16000 && cfg.sample_rate != 24000 &&
        cfg.sample_rate != 48000) {
        throw std::invalid_argument("Opus sample rate must be 8000/12000/16000/24000/48000");
    }
    if (cfg.channels < 1 || cfg.channels > 2) {
        throw std::invalid_argument("Opus channels must be 1 or 2");
    }
    if (cfg.bitrate < 6000 || cfg.bitrate > 510000) {
        throw std::invalid_argument("Opus bitrate must be 6000..510000 bps");
    }

    // Validate frame size: Opus supports 2.5, 5, 10, 20, 40, 60 ms
    float valid_ms[] = {2.5f, 5.0f, 10.0f, 20.0f, 40.0f, 60.0f};
    bool valid = false;
    for (float ms : valid_ms) {
        if (std::abs(cfg.frame_ms - ms) < 0.01f) { valid = true; break; }
    }
    if (!valid) {
        throw std::invalid_argument("Opus frame_ms must be 2.5/5/10/20/40/60");
    }
}

} // anonymous

// =========================================================================
// OpusAudioEncoder
// =========================================================================

OpusAudioEncoder::OpusAudioEncoder(const OpusConfig& cfg) : cfg_(cfg) {
    validateConfig(cfg);

    int error = 0;
    enc_ = opus_encoder_create(
        static_cast<opus_int32>(cfg_.sample_rate),
        cfg_.channels,
        toOpusApplication(cfg_.application),
        &error);

    if (error != OPUS_OK || !enc_) {
        throw std::runtime_error(
            std::string("opus_encoder_create failed: ") + opus_strerror(error));
    }

    // Configure encoder. Check the correctness-critical ctls (the ones
    // whose silent failure would change the on-air behavior — bitrate and
    // the FEC settings) and warn rather than claiming "proper error
    // handling" while ignoring every return. (#70)
    int cerr = OPUS_OK, e;
    e = opus_encoder_ctl(enc_, OPUS_SET_BITRATE(static_cast<opus_int32>(cfg_.bitrate))); if (e != OPUS_OK) cerr = e;
    opus_encoder_ctl(enc_, OPUS_SET_COMPLEXITY(cfg_.complexity));
    opus_encoder_ctl(enc_, OPUS_SET_DTX(cfg_.dtx ? 1 : 0));
    opus_encoder_ctl(enc_, OPUS_SET_VBR(cfg_.vbr ? 1 : 0));
    // Inband FEC: each packet carries a degraded copy of the previous
    // frame so the decoder can recover a lost packet from the next one.
    // OPUS_SET_PACKET_LOSS_PERC tunes how much of that frame's bitrate
    // budget is spent on the FEC copy (Opus reduces normal-coding quality
    // proportionally). 5% is a reasonable default for a channel where
    // occasional CRC failures are expected.
    e = opus_encoder_ctl(enc_, OPUS_SET_INBAND_FEC(cfg_.inband_fec ? 1 : 0)); if (e != OPUS_OK) cerr = e;
    int loss_perc = cfg_.expected_loss_perc;
    if (loss_perc < 0)   loss_perc = 0;
    if (loss_perc > 100) loss_perc = 100;
    e = opus_encoder_ctl(enc_, OPUS_SET_PACKET_LOSS_PERC(loss_perc)); if (e != OPUS_OK) cerr = e;
    // Signal type drives SILK (speech) vs CELT (music) mode. CELT/MUSIC
    // handles tones, music, and mixed content cleanly — the broadcast-audio
    // default. BUT DRED rides the SILK/speech path, so a DRED-enabled stream
    // MUST use the voice signal: with OPUS_SIGNAL_MUSIC the encoder embeds no
    // DRED at all (this previously made the DRED plumbing inert). (SOTA-5)
    bool want_dred = false;
#ifdef OPUS_SET_DRED_DURATION_REQUEST
    want_dred = (cfg_.dred_frames > 0);
#endif
    opus_encoder_ctl(enc_, OPUS_SET_SIGNAL(
        want_dred ? OPUS_SIGNAL_VOICE : OPUS_SIGNAL_MUSIC));
    if (cerr != OPUS_OK) {
        std::fprintf(stderr,
            "[opus] warning: encoder ctl failed (%s) — bitrate/FEC settings "
            "may not have applied\n", opus_strerror(cerr));
    }

    // Opus 1.5+ Deep REDundancy: neural-FEC reconstruction data for
    // multi-frame recovery (vs INBAND_FEC's single previous frame). Requires
    // SILK mode (voice signal, set above) + a non-zero PACKET_LOSS_PERC (set
    // above) to actually embed. The ctl is absent on pre-1.5 Opus → guarded.
#ifdef OPUS_SET_DRED_DURATION_REQUEST
    if (cfg_.dred_frames > 0) {
        int dred = cfg_.dred_frames;
        if (dred > 104) dred = 104;  // ~1 s upper bound per Opus docs
        opus_encoder_ctl(enc_, OPUS_SET_DRED_DURATION(dred));
    }
#endif
}

OpusAudioEncoder::~OpusAudioEncoder() {
    if (enc_) opus_encoder_destroy(enc_);
}

OpusAudioEncoder::OpusAudioEncoder(OpusAudioEncoder&& other) noexcept
    : cfg_(other.cfg_), enc_(other.enc_) {
    other.enc_ = nullptr;
}

OpusAudioEncoder& OpusAudioEncoder::operator=(OpusAudioEncoder&& other) noexcept {
    if (this != &other) {
        if (enc_) opus_encoder_destroy(enc_);
        cfg_ = other.cfg_;
        enc_ = other.enc_;
        other.enc_ = nullptr;
    }
    return *this;
}

bool OpusAudioEncoder::encode(const float* pcm, std::vector<uint8_t>& packet) {
    if (!enc_) return false;

    packet.resize(OpusConfig::maxPacketBytes());

    opus_int32 ret = opus_encode_float(
        enc_,
        pcm,
        static_cast<int>(cfg_.frameSamples()),
        packet.data(),
        static_cast<opus_int32>(packet.size()));

    if (ret < 0) {
        packet.clear();
        return false;
    }

    packet.resize(static_cast<size_t>(ret));
    return true;
}

bool OpusAudioEncoder::setBitrate(uint32_t bitrate_bps) {
    if (!enc_ || bitrate_bps < 6000 || bitrate_bps > 510000) return false;
    cfg_.bitrate = bitrate_bps;
    return opus_encoder_ctl(enc_, OPUS_SET_BITRATE(
        static_cast<opus_int32>(bitrate_bps))) == OPUS_OK;
}

uint32_t OpusAudioEncoder::actualBitrate() const {
    if (!enc_) return 0;
    opus_int32 br = 0;
    opus_encoder_ctl(enc_, OPUS_GET_BITRATE(&br));
    return static_cast<uint32_t>(br);
}

// =========================================================================
// OpusAudioDecoder
// =========================================================================

OpusAudioDecoder::OpusAudioDecoder(const OpusConfig& cfg) : cfg_(cfg) {
    validateConfig(cfg);

    int error = 0;
    dec_ = opus_decoder_create(
        static_cast<opus_int32>(cfg_.sample_rate),
        cfg_.channels,
        &error);

    if (error != OPUS_OK || !dec_) {
        throw std::runtime_error(
            std::string("opus_decoder_create failed: ") + opus_strerror(error));
    }

    // Allocate Deep REDundancy decode state (Opus 1.5+). On a libopus built
    // without deep-PLC these return null/unimplemented — we just leave the
    // pointers null and decodeFromDRED() becomes a no-op. (SOTA-5)
#ifdef OPUS_SET_DRED_DURATION_REQUEST
    int dred_err = 0;
    dred_dec_ = opus_dred_decoder_create(&dred_err);
    if (dred_err != OPUS_OK) dred_dec_ = nullptr;
    dred_ = opus_dred_alloc(&dred_err);
    if (dred_err != OPUS_OK) dred_ = nullptr;
#endif
}

OpusAudioDecoder::~OpusAudioDecoder() {
    if (dec_) opus_decoder_destroy(dec_);
#ifdef OPUS_SET_DRED_DURATION_REQUEST
    if (dred_)     opus_dred_free(dred_);
    if (dred_dec_) opus_dred_decoder_destroy(dred_dec_);
#endif
}

OpusAudioDecoder::OpusAudioDecoder(OpusAudioDecoder&& other) noexcept
    : cfg_(other.cfg_), dec_(other.dec_),
      dred_dec_(other.dred_dec_), dred_(other.dred_) {
    other.dec_ = nullptr;
    other.dred_dec_ = nullptr;
    other.dred_ = nullptr;
}

OpusAudioDecoder& OpusAudioDecoder::operator=(OpusAudioDecoder&& other) noexcept {
    if (this != &other) {
        if (dec_) opus_decoder_destroy(dec_);
#ifdef OPUS_SET_DRED_DURATION_REQUEST
        if (dred_)     opus_dred_free(dred_);
        if (dred_dec_) opus_dred_decoder_destroy(dred_dec_);
#endif
        cfg_ = other.cfg_;
        dec_ = other.dec_;
        dred_dec_ = other.dred_dec_;
        dred_ = other.dred_;
        other.dec_ = nullptr;
        other.dred_dec_ = nullptr;
        other.dred_ = nullptr;
    }
    return *this;
}

size_t OpusAudioDecoder::decode(const uint8_t* packet, size_t packet_len,
                                 std::vector<float>& pcm) {
    if (!dec_) return 0;

    // Max output: frame_samples * channels
    size_t max_samples = cfg_.frameSamples();
    pcm.resize(max_samples * cfg_.channels);

    int ret = opus_decode_float(
        dec_,
        packet,
        static_cast<opus_int32>(packet_len),
        pcm.data(),
        static_cast<int>(max_samples),
        0 /* no FEC */);

    if (ret < 0) {
        pcm.clear();
        return 0;
    }

    pcm.resize(static_cast<size_t>(ret) * cfg_.channels);
    return static_cast<size_t>(ret);
}

size_t OpusAudioDecoder::decodeFromFEC(const uint8_t* next_packet,
                                        size_t next_packet_len,
                                        std::vector<float>& pcm) {
    if (!dec_ || !next_packet || next_packet_len == 0) {
        return decodePLC(pcm);
    }
    size_t max_samples = cfg_.frameSamples();
    pcm.resize(max_samples * cfg_.channels);
    // decode_fec = 1 tells Opus to produce the PREVIOUS frame's audio
    // from this packet's embedded FEC info. If the packet has no FEC
    // (encoder didn't set inband_fec), Opus falls back to PLC.
    int ret = opus_decode_float(
        dec_,
        next_packet,
        static_cast<opus_int32>(next_packet_len),
        pcm.data(),
        static_cast<int>(max_samples),
        1 /* decode_fec */);
    if (ret < 0) {
        pcm.clear();
        return 0;
    }
    pcm.resize(static_cast<size_t>(ret) * cfg_.channels);
    return static_cast<size_t>(ret);
}

size_t OpusAudioDecoder::decodePLC(std::vector<float>& pcm) {
    if (!dec_) return 0;

    size_t max_samples = cfg_.frameSamples();
    pcm.resize(max_samples * cfg_.channels);

    // NULL packet = packet loss concealment
    int ret = opus_decode_float(
        dec_,
        nullptr,
        0,
        pcm.data(),
        static_cast<int>(max_samples),
        0);

    if (ret < 0) {
        pcm.clear();
        return 0;
    }

    pcm.resize(static_cast<size_t>(ret) * cfg_.channels);
    return static_cast<size_t>(ret);
}

size_t OpusAudioDecoder::decodeFromDRED(const uint8_t* recv_packet, size_t recv_len,
                                        int frames_lost, std::vector<float>& pcm) {
    pcm.clear();
#ifdef OPUS_SET_DRED_DURATION_REQUEST
    if (!dec_ || !dred_dec_ || !dred_ || !recv_packet || recv_len == 0 ||
        frames_lost <= 0) {
        return 0;
    }
    const int frame = static_cast<int>(cfg_.frameSamples());
    const int chans = cfg_.channels;

    // Parse the received packet's DRED. The return value is the offset (in
    // samples before this packet's audio) of the OLDEST recoverable DRED
    // sample; 0 means the packet carries no DRED. We ask for at most the
    // burst length, since that's all we need to fill.
    int dred_end = 0;
    int first_off = opus_dred_parse(
        dred_dec_, dred_, recv_packet, static_cast<opus_int32>(recv_len),
        static_cast<opus_int32>(frames_lost * frame),
        static_cast<opus_int32>(cfg_.sample_rate), &dred_end, 0);
    if (first_off <= 0) return 0;

    pcm.assign(static_cast<size_t>(frames_lost) * frame * chans, 0.f);
    size_t dred_recovered = 0;
    // Reconstruct each lost frame in time order (oldest first). The frame
    // (recv − k) sits k frames (k*frame samples) before recv's audio.
    for (int k = frames_lost; k >= 1; --k) {
        const int off = k * frame;
        float* dst = pcm.data() +
                     static_cast<size_t>(frames_lost - k) * frame * chans;
        if (off <= first_off) {
            int dr = opus_decoder_dred_decode_float(dec_, dred_, off, dst, frame);
            if (dr > 0) ++dred_recovered;
        } else {
            // Older than the DRED depth → packet-loss concealment, so the
            // decoder still advances one frame and the gap is concealed
            // rather than left as a hole. (wires decodePLC's underlying call)
            opus_decode_float(dec_, nullptr, 0, dst, frame, 0);
        }
    }
    return dred_recovered;
#else
    (void)recv_packet; (void)recv_len; (void)frames_lost;
    return 0;
#endif
}

bool OpusAudioDecoder::dredAvailable() const {
#ifdef OPUS_SET_DRED_DURATION_REQUEST
    return dred_dec_ != nullptr && dred_ != nullptr;
#else
    return false;
#endif
}

void OpusAudioDecoder::reset() {
    if (dec_) {
        opus_decoder_ctl(dec_, OPUS_RESET_STATE);
    }
}

} // namespace gw
