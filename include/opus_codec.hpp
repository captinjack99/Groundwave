/**
 * @file opus_codec.hpp
 * @brief Opus audio codec wrapper for Groundwave v2
 *
 * Wraps the libopus encoder/decoder with a clean C++ interface.
 * Opus is used because it handles the full audio bandwidth range
 * (narrowband to fullband), supports mono and stereo, and is
 * designed for low-latency streaming — ideal for a soundcard modem.
 *
 * Typical usage in the TX/RX chain:
 *   TX: PCM audio → OpusEncoder::encode() → packets → FrameBuilder
 *       → LDPC encode → interleave → OFDM modulate
 *   RX: OFDM demod → deinterleave → LDPC decode → FrameParser
 *       → packets → OpusDecoder::decode() → PCM audio
 */
#pragma once

#include "types.hpp"
#include <vector>
#include <memory>
#include <cstdint>

// Forward-declare opaque libopus types (avoid exposing opus.h to all users)
struct OpusEncoder;
struct OpusDecoder;
struct OpusDREDDecoder;   // Opus 1.5+ Deep REDundancy decoder state
struct OpusDRED;          // Opus 1.5+ parsed-DRED holder

namespace gw {

// =========================================================================
// Configuration
// =========================================================================

enum class OpusApplication : uint8_t {
    Audio = 0,       // OPUS_APPLICATION_AUDIO — music, mixed content
    VoIP  = 1,       // OPUS_APPLICATION_VOIP — speech optimization
    LowDelay = 2,    // OPUS_APPLICATION_RESTRICTED_LOWDELAY — minimum latency
};

struct OpusConfig {
    uint32_t sample_rate = 48000;    // Must be 8000, 12000, 16000, 24000, or 48000
    uint8_t  channels    = 1;        // 1 (mono) or 2 (stereo)
    uint32_t bitrate     = 64000;    // Target bitrate in bps (6000..510000)
    float    frame_ms    = 20.0f;    // Frame duration: 2.5, 5, 10, 20, 40, or 60 ms
    OpusApplication application = OpusApplication::Audio;
    bool     dtx         = true;     // Discontinuous transmission (silence detection)
    int      complexity  = 10;       // Encoder complexity 0-10 (10 = best quality)
    bool     vbr         = true;     // Variable bitrate (vs CBR)

    // Forward error correction. When enabled, each packet contains a low-
    // bitrate FEC copy of the *previous* packet's contents that the
    // decoder can recover when the previous packet is lost. Costs about
    // 10% bitrate at expected_loss_perc=10. Tells Opus how much
    // resilience to encode against random per-packet loss.
    bool     inband_fec  = true;     // OPUS_SET_INBAND_FEC
    int      expected_loss_perc = 5; // OPUS_SET_PACKET_LOSS_PERC (0..100)

    // Deep REDundancy (Opus 1.5+). When > 0, the encoder embeds up to
    // `dred_frames × 10 ms` of neural-FEC reconstruction data in each
    // packet so the decoder can recover the contents of multiple lost
    // packets. 0 disables DRED (passes OPUS_SET_DRED_DURATION(0)).
    // Recommended for streaming radio links where bursty losses span
    // multiple frames. Costs ~2–4 kbps of the packet's bitrate budget
    // per 10 ms of DRED depending on content. Older Opus versions
    // silently ignore the ctl call.
    int      dred_frames = 0;

    /** Samples per frame per channel */
    size_t frameSamples() const {
        return static_cast<size_t>(sample_rate * frame_ms / 1000.0f);
    }

    /** Total samples per frame (all channels) */
    size_t frameSamplesTotal() const {
        return frameSamples() * channels;
    }

    /** Max Opus packet size (bytes) — worst case for encoder output */
    static constexpr size_t maxPacketBytes() { return 1500; }
};

// =========================================================================
// Opus Encoder
// =========================================================================

class OpusAudioEncoder {
public:
    explicit OpusAudioEncoder(const OpusConfig& cfg);
    ~OpusAudioEncoder();

    // Non-copyable, movable
    OpusAudioEncoder(const OpusAudioEncoder&) = delete;
    OpusAudioEncoder& operator=(const OpusAudioEncoder&) = delete;
    OpusAudioEncoder(OpusAudioEncoder&& other) noexcept;
    OpusAudioEncoder& operator=(OpusAudioEncoder&& other) noexcept;

    /** Encode one frame of PCM audio.
     *  @param pcm  Interleaved float samples, cfg.frameSamplesTotal() values
     *              Range: [-1.0, 1.0]
     *  @param packet  Output: encoded Opus packet (variable length)
     *  @return true on success */
    bool encode(const float* pcm, std::vector<uint8_t>& packet);

    /** Set bitrate dynamically (for adaptive ModCod) */
    bool setBitrate(uint32_t bitrate_bps);

    /** Get current configuration */
    const OpusConfig& config() const { return cfg_; }

    /** Get actual bitrate (may differ from target in VBR mode) */
    uint32_t actualBitrate() const;

private:
    OpusConfig cfg_;
    ::OpusEncoder* enc_ = nullptr;
};

// =========================================================================
// Opus Decoder
// =========================================================================

class OpusAudioDecoder {
public:
    explicit OpusAudioDecoder(const OpusConfig& cfg);
    ~OpusAudioDecoder();

    // Non-copyable, movable
    OpusAudioDecoder(const OpusAudioDecoder&) = delete;
    OpusAudioDecoder& operator=(const OpusAudioDecoder&) = delete;
    OpusAudioDecoder(OpusAudioDecoder&& other) noexcept;
    OpusAudioDecoder& operator=(OpusAudioDecoder&& other) noexcept;

    /** Decode one Opus packet to PCM audio.
     *  @param packet      Encoded Opus data
     *  @param packet_len  Packet length in bytes
     *  @param pcm         Output: interleaved float samples
     *  @return Number of samples decoded per channel, or 0 on error */
    size_t decode(const uint8_t* packet, size_t packet_len,
                  std::vector<float>& pcm);

    /** Packet loss concealment: generate one frame of interpolated audio.
     *  Call this when a packet is lost (not received / CRC failed).
     *  @param pcm  Output: concealed audio samples
     *  @return Number of samples per channel */
    size_t decodePLC(std::vector<float>& pcm);

    /** Recover a lost frame from the NEXT received packet's inband FEC
     *  copy. Use this when packet N is missing but packet N+1 is in hand
     *  and was encoded with inband FEC enabled — the next packet carries
     *  a degraded version of N which this method extracts.
     *
     *  After calling this for the recovered frame, call decode() on the
     *  same `next_packet` to emit the normal frame. The two-call pattern
     *  produces TWO frames of audio from one received packet (the
     *  recovered one and the current one).
     *
     *  Falls back to PLC behavior when the next packet doesn't actually
     *  carry FEC info (encoder was configured without it).
     *
     *  @param next_packet      Bytes of the packet immediately after the lost one
     *  @param next_packet_len  Length of next_packet
     *  @param pcm              Output: recovered audio samples
     *  @return Samples per channel, 0 on error */
    size_t decodeFromFEC(const uint8_t* next_packet, size_t next_packet_len,
                          std::vector<float>& pcm);

    /** Recover a burst of lost frames from a RECEIVED packet's Deep
     *  REDundancy (DRED) data (Opus 1.5+). Where INBAND_FEC covers only the
     *  single previous frame, DRED reconstructs multiple frames back. The
     *  decoder must already be primed (have decoded prior frames) for the
     *  neural reconstruction to produce real audio.
     *
     *  Fills `frames_lost` contiguous frames in time order (oldest first):
     *  DRED reconstruction where the packet's DRED reaches, packet-loss
     *  concealment (PLC) for any frames older than the DRED depth. After this,
     *  call decode() on the SAME `recv_packet` to emit the current frame —
     *  the decoder state is left ready for it.
     *
     *  @param recv_packet   The packet received after the loss burst.
     *  @param recv_len      Its length in bytes.
     *  @param frames_lost   Number of consecutive lost frames before recv_packet.
     *  @param pcm           Output: frames_lost × frameSamplesTotal() values.
     *  @return Number of frames DRED actually RECONSTRUCTED (excludes PLC
     *          fills); 0 if the packet carries no DRED or DRED is unavailable. */
    size_t decodeFromDRED(const uint8_t* recv_packet, size_t recv_len,
                          int frames_lost, std::vector<float>& pcm);

    /** True if the linked libopus implements DRED and this decoder allocated
     *  its DRED state. */
    bool dredAvailable() const;

    /** Reset decoder state (e.g., after seeking or stream restart) */
    void reset();

    const OpusConfig& config() const { return cfg_; }

private:
    OpusConfig cfg_;
    ::OpusDecoder* dec_ = nullptr;
    // Deep REDundancy decode state (Opus 1.5+). Null on builds without DRED.
    ::OpusDREDDecoder* dred_dec_ = nullptr;
    ::OpusDRED*        dred_     = nullptr;
};

} // namespace gw
