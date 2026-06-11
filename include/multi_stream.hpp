/**
 * @file multi_stream.hpp
 * @brief Multi-stream audio coordinator (up to 8 simultaneous streams).
 *
 * The frame format already carries packets tagged with `stream_id` (0..7),
 * so the wire layer supports multi-stream natively. This module adds:
 *
 *   - One OpusEncoder per active TX stream
 *   - Per-stream bitrate, frame size, mono/stereo, application
 *   - One OpusDecoder per active RX stream (created on first packet seen)
 *   - Per-stream playback ring buffers
 *   - Stream enable/disable without tearing down others
 *
 * Total bandwidth per OFDM frame is divided among active streams. By
 * default each enabled stream gets an equal share; setBitrateWeight()
 * can bias the split.
 */
#pragma once

#include "types.hpp"
#include "opus_codec.hpp"
#include "frame.hpp"
#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <vector>
#include <algorithm>

namespace gw {

static constexpr size_t MAX_STREAMS = 8;

/** Per-stream audio source. Determines what fills the TX ring on each
 *  frame: a programmable test tone, system silence (digital zero), or the
 *  default microphone capture device routed via miniaudio. */
enum class StreamAudioSource : uint8_t {
    TestTone   = 0,   ///< Sine at `tone_freq_hz` (different per stream so streams are audibly distinguishable)
    Silence    = 1,   ///< Digital zero — useful for measuring noise floor without distraction
    Microphone = 2,   ///< Capture default mic; miniaudio writes into the per-stream input ring
    File       = 3,   ///< Read PCM from `file_path` (looped)
};

/** Per-stream configuration. */
struct StreamConfig {
    bool   enabled        = false;
    uint32_t bitrate_bps  = 32000;   ///< Total stream bitrate budget
                                     ///  (split internally into Mid/Side
                                     ///   when channels==2 + hier active).
    float  weight         = 1.f;     ///< Bandwidth share weight (relative)
    uint8_t channels      = 1;       ///< 1 = mono, 2 = stereo
    OpusApplication app   = OpusApplication::Audio;
    uint32_t frame_ms     = 20;      ///< Opus frame duration (2.5/5/10/20/40/60)
    uint32_t sample_rate  = 48000;
    char   name[32]       = {0};

    StreamAudioSource source = StreamAudioSource::TestTone;
    float    tone_freq_hz   = 1000.f;   ///< Used when source == TestTone
    float    tone_amplitude = 0.5f;
    char     file_path[256] = {0};      ///< Used when source == File

    /** Per-stream input device index. -1 = system default. When two
     *  streams use the same `input_device` value, they share one open
     *  capture device (and one capture ring) inside the engine. Used
     *  only when source == Microphone. */
    int      input_device   = -1;

    /** Mid/Side bitrate split for stereo streams under hier mod.
     *  β = bits assigned to Mid as a fraction of total. Side gets (1−β).
     *  Default 0.62 — Mid carries main mono image (higher info density),
     *  Side carries stereo width (lower info density). Ignored for
     *  mono streams or when hier mod is off. */
    float    mid_side_split = 0.62f;

    /** Opus 1.5+ Deep REDundancy frame count. 0 = off (legacy Opus
     *  behavior — only OPUS_SET_INBAND_FEC's single-frame recovery).
     *  Values up to ~104 add neural-FEC for multi-frame recovery in
     *  exchange for bitrate. Pass-through to OpusConfig.dred_frames. */
    int      opus_dred_frames = 0;
};

/** Lock-free SPSC ring buffer for PCM audio (per-stream playback). */
class FloatRingBuffer {
public:
    explicit FloatRingBuffer(size_t capacity_samples) : buf_(capacity_samples + 1), cap_(capacity_samples + 1) {}

    size_t write(const float* in, size_t n) {
        size_t wp = wp_.load(std::memory_order_relaxed);
        size_t rp = rp_.load(std::memory_order_acquire);
        size_t avail = (cap_ - 1) - ((wp - rp + cap_) % cap_);
        size_t k = std::min(n, avail);
        for (size_t i = 0; i < k; ++i) {
            buf_[wp] = in[i];
            wp = (wp + 1) % cap_;
        }
        wp_.store(wp, std::memory_order_release);
        return k;
    }

    size_t read(float* out, size_t n) {
        size_t rp = rp_.load(std::memory_order_relaxed);
        size_t wp = wp_.load(std::memory_order_acquire);
        size_t avail = (wp - rp + cap_) % cap_;
        size_t k = std::min(n, avail);
        for (size_t i = 0; i < k; ++i) {
            out[i] = buf_[rp];
            rp = (rp + 1) % cap_;
        }
        rp_.store(rp, std::memory_order_release);
        return k;
    }

    size_t available() const {
        size_t rp = rp_.load(std::memory_order_acquire);
        size_t wp = wp_.load(std::memory_order_acquire);
        return (wp - rp + cap_) % cap_;
    }

    void clear() {
        rp_.store(0); wp_.store(0);
    }

    size_t capacity() const { return cap_ - 1; }

private:
    std::vector<float> buf_;
    size_t cap_;
    std::atomic<size_t> rp_{0};
    std::atomic<size_t> wp_{0};
};

class MultiStreamCoordinator {
public:
    MultiStreamCoordinator() {
        for (size_t i = 0; i < MAX_STREAMS; ++i) {
            tx_input_[i]    = std::make_unique<FloatRingBuffer>(48000);
            rx_output_[i]   = std::make_unique<FloatRingBuffer>(48000);
            side_output_[i] = std::make_unique<FloatRingBuffer>(48000);
        }
    }

    // -----------------------------------------------------------------
    // Configuration
    // -----------------------------------------------------------------

    /** Configure or enable/disable a stream. Re-creates the codecs on
     *  change. For stereo streams (channels=2), creates BOTH a Mid and a
     *  Side encoder (each mono, at split bitrates determined by
     *  `mid_side_split`); these feed the Mid (HP) and Side (LP)
     *  hierarchical-modulation layers respectively. For mono streams
     *  only the Mid encoder exists. */
    void configureStream(size_t id, const StreamConfig& cfg) {
        if (id >= MAX_STREAMS) return;
        std::lock_guard<std::mutex> lk(mtx_);
        cfg_[id] = cfg;
        // Reset prior state — handles disable, channel-count change,
        // bitrate change, etc.
        tx_mid_[id].reset();
        tx_side_[id].reset();
        rx_mid_[id].reset();
        rx_side_[id].reset();

        // Flush per-stream RX state on (re)configure. Without this, a live
        // reconfigure (channel-count or bitrate change) could leave the
        // Mid and Side rings at different fill levels — permanently
        // desyncing the L=Mid+Side / R=Mid−Side recombination — or carry a
        // stale lost-run counter into the new codec instance.
        lost_run_[id] = 0;
        latest_side_[id].clear();
        if (rx_output_[id])   rx_output_[id]->clear();
        if (side_output_[id]) side_output_[id]->clear();

        if (!cfg.enabled) return;

        // Compute Mid / Side bitrates. Mono streams get the whole
        // bitrate on Mid; stereo streams split per `mid_side_split`.
        // A stereo stream needs at least 2×6000 = 12000 bps total so each
        // of the two mono encoders clears Opus's 6000 floor. Below that,
        // the independent [6000,510000] clamps would DOUBLE the real
        // encoder budget (e.g. 6000 total → 6000+6000 = 12000 actual),
        // overshooting the frame and tripping FrameBuilder's full-frame
        // `break` that halts every remaining stream. So fall back to mono
        // (Mid only, full rate) for such low-rate streams. (#71)
        const bool stereo = (cfg.channels == 2) && (cfg.bitrate_bps >= 12000u);
        const float beta  = std::clamp(cfg.mid_side_split, 0.10f, 0.90f);
        uint32_t mid_bps  = stereo
            ? static_cast<uint32_t>(static_cast<float>(cfg.bitrate_bps) * beta)
            : cfg.bitrate_bps;
        uint32_t side_bps = stereo
            ? static_cast<uint32_t>(static_cast<float>(cfg.bitrate_bps)
                                     * (1.f - beta))
            : 0u;
        // Clamp to Opus's valid range (6000..510000).
        auto clamp_bps = [](uint32_t b) {
            if (b < 6000u) return 6000u;
            if (b > 510000u) return 510000u;
            return b;
        };
        mid_bps  = clamp_bps(mid_bps);
        if (side_bps > 0) side_bps = clamp_bps(side_bps);

        // Both Mid and Side encoders run mono. Each carries a
        // mathematically distinct signal: Mid = (L+R)/2, Side = (L-R)/2.
        // Using two mono encoders is the SOTA layered-audio + UEP
        // pattern — Opus's internal joint-stereo coupling would NOT
        // give us the HP/LP separation we need for hier mod.
        OpusConfig mid_cfg{};
        mid_cfg.sample_rate = cfg.sample_rate;
        mid_cfg.channels    = 1;
        mid_cfg.bitrate     = mid_bps;
        mid_cfg.frame_ms    = static_cast<float>(cfg.frame_ms);
        mid_cfg.application = cfg.app;
        mid_cfg.dred_frames = cfg.opus_dred_frames;
        try {
            tx_mid_[id] = std::make_unique<OpusAudioEncoder>(mid_cfg);
            rx_mid_[id] = std::make_unique<OpusAudioDecoder>(mid_cfg);
        } catch (...) {
            tx_mid_[id].reset();
            rx_mid_[id].reset();
            cfg_[id].enabled = false;
            return;
        }
        if (stereo) {
            OpusConfig side_cfg = mid_cfg;
            side_cfg.bitrate = side_bps;
            try {
                tx_side_[id] = std::make_unique<OpusAudioEncoder>(side_cfg);
                rx_side_[id] = std::make_unique<OpusAudioDecoder>(side_cfg);
            } catch (...) {
                // Side failure isn't fatal — fall back to mono-only
                // (the user loses graceful-degradation benefit on this
                // stream but Mid still plays).
                tx_side_[id].reset();
                rx_side_[id].reset();
            }
        }
    }

    StreamConfig streamConfig(size_t id) const {
        if (id >= MAX_STREAMS) return {};
        std::lock_guard<std::mutex> lk(mtx_);
        return cfg_[id];
    }

    /** Active stream count. */
    size_t activeCount() const {
        std::lock_guard<std::mutex> lk(mtx_);
        size_t c = 0;
        for (auto& s : cfg_) if (s.enabled) ++c;
        return c;
    }

    // -----------------------------------------------------------------
    // TX path: encode all enabled streams into Mid + Side frames
    // -----------------------------------------------------------------

    /** Pull audio from input ring buffers, split stereo streams into
     *  Mid/Side, encode each, and add the resulting packets to two
     *  FrameBuilders.
     *
     *  @param mid_fb        Receives a packet per enabled stream
     *                       (Mid component for stereo, full audio for mono).
     *  @param side_fb       Receives a packet per ENABLED STEREO stream.
     *                       Pass nullptr (or unused) if hierarchical mod
     *                       is OFF — mono streams ride mid_fb only and
     *                       stereo streams degrade to Mid-only playback.
     *  @param hier_active   When false, only mid_fb is populated. */
    void encodeIntoFrames(FrameBuilder& mid_fb, FrameBuilder* side_fb,
                          bool hier_active) {
        std::lock_guard<std::mutex> lk(mtx_);
        for (uint8_t id = 0; id < MAX_STREAMS; ++id) {
            if (!cfg_[id].enabled || !tx_mid_[id]) continue;
            const bool stereo = (cfg_[id].channels == 2);
            size_t frame_samples = tx_mid_[id]->config().frameSamples();
            // Input ring layout: interleaved L,R for stereo, mono samples
            // for mono. Required input count = frame_samples × channels.
            size_t needed = frame_samples * cfg_[id].channels;
            if (tx_input_[id]->available() < needed) continue;

            scratch_.resize(needed);
            tx_input_[id]->read(scratch_.data(), needed);

            // Split stereo into Mid/Side. Mono streams pass through to
            // mid_pcm_ directly.
            mid_pcm_.resize(frame_samples);
            if (stereo) {
                side_pcm_.resize(frame_samples);
                for (size_t i = 0; i < frame_samples; ++i) {
                    float L = scratch_[2 * i + 0];
                    float R = scratch_[2 * i + 1];
                    mid_pcm_[i]  = 0.5f * (L + R);
                    side_pcm_[i] = 0.5f * (L - R);
                }
            } else {
                std::memcpy(mid_pcm_.data(), scratch_.data(),
                            frame_samples * sizeof(float));
            }

            // Encode Mid → mid_fb (always)
            ByteVec mid_pkt;
            if (tx_mid_[id]->encode(mid_pcm_.data(), mid_pkt)) {
                if (!mid_fb.addPacket(id, mid_pkt.data(), mid_pkt.size())) {
                    // Mid frame full — bail; Side path would just waste work
                    break;
                }
            } else {
                continue;
            }

            // Encode Side → side_fb (only stereo streams under hier mod)
            if (stereo && hier_active && side_fb && tx_side_[id]) {
                ByteVec side_pkt;
                if (tx_side_[id]->encode(side_pcm_.data(), side_pkt)) {
                    if (!side_fb->addPacket(id, side_pkt.data(),
                                             side_pkt.size())) {
                        // Side frame full — stream's Side info dropped
                        // this frame, but Mid was already added. The RX
                        // will see no Side packet → falls back to
                        // predictive synthesis.
                    }
                }
            }
        }
    }

    /** Backward-compat single-frame encoder. Encodes all streams as
     *  Mid only (no Side) into one FrameBuilder. Used when hier mod is
     *  OFF — stereo streams play as mono. */
    void encodeIntoFrame(FrameBuilder& fb) {
        encodeIntoFrames(fb, nullptr, /*hier_active=*/false);
    }

    /** Push raw PCM into a stream's TX input buffer. Returns samples
     *  accepted. PCM is interleaved L,R for stereo (channels=2) or mono
     *  samples for mono streams. The caller must respect the stream's
     *  configured channel count. */
    size_t pushTX(size_t id, const float* pcm, size_t n_samples) {
        if (id >= MAX_STREAMS) return 0;
        return tx_input_[id]->write(pcm, n_samples);
    }

    // -----------------------------------------------------------------
    // RX path: decode incoming packets, route to per-stream output rings
    // -----------------------------------------------------------------

    /** Dispatch Mid and Side frame packets to per-stream decoders.
     *  For each stream's Mid packet, decode to mid_pcm. For each
     *  matching Side packet, decode to side_pcm. Recombine to
     *  effective playback PCM and write to the stream's output ring.
     *
     *  Side packet may be missing because:
     *    - The stream is mono (channels=1) — no Side ever transmitted.
     *    - Side frame decode failed at the engine level.
     *    - Specific stream's Side packet was lost (frame full).
     *
     *  When Side is missing, the caller (engine) may pass a
     *  reconstructor function that synthesizes a plausible Side from
     *  the decoded Mid. The output is then L = Mid + Side_synth and
     *  R = Mid − Side_synth — graceful stereo recovery.
     *
     *  Currently the output ring carries the Mid PCM only (mono);
     *  full L/R routing is a future-iteration concern. The Side info
     *  is still decoded and made available via `latestSide(id)` for
     *  recording or future stereo output channels. */
    void onParsedFrames(const ParsedFrame& mid_frame,
                        const ParsedFrame* side_frame_opt) {
        std::lock_guard<std::mutex> lk(mtx_);
        std::array<bool, MAX_STREAMS> seen_mid{};
        std::array<bool, MAX_STREAMS> seen_side{};

        // Build a stream_id → side-packet lookup for O(1) match below.
        std::array<const ParsedFrame::Packet*, MAX_STREAMS> side_pkt_by_id{};
        if (side_frame_opt && side_frame_opt->crc_ok) {
            for (const auto& pkt : side_frame_opt->packets) {
                if (pkt.stream_id < MAX_STREAMS) {
                    side_pkt_by_id[pkt.stream_id] = &pkt;
                }
            }
        }

        // Write `n` samples of silence to a stream's Side ring, keeping it
        // sample-locked to the Mid ring. Used whenever a stereo stream
        // produces Mid output but the transmitted Side is absent (no Side
        // packet, Side decode failed, or FEC-recovered Mid frame). The
        // engine's drain crossfades to a synthesized Side via the LP-LLR
        // confidence weight in those cases, so these silence samples carry
        // weight 0 — their only job is to keep the two rings aligned so the
        // L=Mid+Side / R=Mid−Side recombination never drifts.
        auto writeSilenceSide = [&](size_t id, size_t n) {
            if (n == 0 || !side_output_[id]) return;
            std::vector<float> zeros(n, 0.f);
            side_output_[id]->write(zeros.data(), n);
        };

        if (mid_frame.crc_ok) {
            for (const auto& pkt : mid_frame.packets) {
                if (pkt.stream_id >= MAX_STREAMS) continue;
                auto& mid_dec = rx_mid_[pkt.stream_id];
                if (!mid_dec) continue;
                seen_mid[pkt.stream_id] = true;
                const bool stereo = (cfg_[pkt.stream_id].channels == 2) &&
                                    rx_side_[pkt.stream_id] &&
                                    side_output_[pkt.stream_id];

                // Loss recovery on Mid for the run of previously-lost frames.
                // DRED (Opus 1.5+) reconstructs the WHOLE burst from this
                // packet's neural-FEC data; if the packet carries no DRED (or
                // the build lacks it) we fall back to inband FEC, which only
                // recovers the single most-recent frame. Recovered frames have
                // no Side packet, so the Side ring is kept locked with matching
                // silence. (SOTA-5)
                if (lost_run_[pkt.stream_id] > 0) {
                    const int lost = lost_run_[pkt.stream_id];
                    bool recovered = false;
                    if (mid_dec->dredAvailable()) {
                        dred_buf_.clear();
                        size_t dred_n = mid_dec->decodeFromDRED(
                            pkt.data.data(), pkt.data.size(), lost, dred_buf_);
                        if (dred_n > 0 && !dred_buf_.empty()) {
                            rx_output_[pkt.stream_id]->write(dred_buf_.data(),
                                                              dred_buf_.size());
                            if (stereo) writeSilenceSide(pkt.stream_id,
                                                         dred_buf_.size());
                            recovered = true;
                        }
                    }
                    if (!recovered) {
                        // Inband FEC reconstructs only the single most-recent
                        // lost frame from this packet. For a multi-frame loss
                        // run, PLC-fill the older lost frames first so the
                        // output ring stays time-locked with wall-clock —
                        // otherwise it underfills by (lost-1) frames and the
                        // decoded audio drifts faster than real time. (SOTA-5)
                        for (int k = 0; k < lost - 1; ++k) {
                            decode_buf_.clear();
                            size_t plc_n = mid_dec->decodePLC(decode_buf_);
                            if (plc_n > 0) {
                                rx_output_[pkt.stream_id]->write(
                                    decode_buf_.data(), decode_buf_.size());
                                if (stereo) writeSilenceSide(pkt.stream_id,
                                                             decode_buf_.size());
                            }
                        }
                        decode_buf_.clear();
                        size_t fec_n = mid_dec->decodeFromFEC(
                            pkt.data.data(), pkt.data.size(), decode_buf_);
                        if (fec_n > 0) {
                            rx_output_[pkt.stream_id]->write(decode_buf_.data(),
                                                              decode_buf_.size());
                            if (stereo) writeSilenceSide(pkt.stream_id,
                                                         decode_buf_.size());
                        }
                    }
                    lost_run_[pkt.stream_id] = 0;
                }

                decode_buf_.clear();
                size_t mid_n = mid_dec->decode(pkt.data.data(), pkt.data.size(),
                                                decode_buf_);
                if (mid_n == 0) continue;
                rx_output_[pkt.stream_id]->write(decode_buf_.data(),
                                                  decode_buf_.size());
                const size_t mid_written = decode_buf_.size();

                // Stereo: write exactly one matching Side run for this Mid
                // frame — decoded transmitted Side if present & valid, else
                // silence — so the rings stay frame-locked unconditionally.
                if (stereo) {
                    bool wrote_side = false;
                    if (side_pkt_by_id[pkt.stream_id]) {
                        const auto* spkt = side_pkt_by_id[pkt.stream_id];
                        side_decode_buf_.clear();
                        size_t side_n = rx_side_[pkt.stream_id]->decode(
                            spkt->data.data(), spkt->data.size(),
                            side_decode_buf_);
                        if (side_n > 0 && !side_decode_buf_.empty()) {
                            seen_side[pkt.stream_id] = true;
                            latest_side_[pkt.stream_id] = side_decode_buf_;
                            // Match Mid length exactly (Opus Mid/Side share
                            // the frame size; guard defensively anyway).
                            side_decode_buf_.resize(mid_written, 0.f);
                            side_output_[pkt.stream_id]->write(
                                side_decode_buf_.data(),
                                side_decode_buf_.size());
                            wrote_side = true;
                        }
                    }
                    if (!wrote_side) {
                        latest_side_[pkt.stream_id].clear();
                        writeSilenceSide(pkt.stream_id, mid_written);
                    }
                }
            }
        }

        // Mark missing-Mid streams as lost (extend the run) so the next
        // received packet attempts DRED / FEC recovery.
        for (size_t id = 0; id < MAX_STREAMS; ++id) {
            if (cfg_[id].enabled && rx_mid_[id] && !seen_mid[id] &&
                lost_run_[id] < MAX_RECOVER_FRAMES) {
                ++lost_run_[id];
            }
        }
    }

    /** Backward-compat single-frame dispatch — no Side info, used when
     *  hier mod is OFF. Calls onParsedFrames with nullptr for side. */
    void onParsedFrame(const ParsedFrame& frame) {
        onParsedFrames(frame, nullptr);
    }

    /** Access the most-recently-decoded Side PCM for a stereo stream.
     *  Empty if the stream is mono, Side is missing this frame, or
     *  decode failed. Used by the engine to feed recorders or future
     *  stereo output channels. */
    std::vector<float> latestSide(size_t id) const {
        if (id >= MAX_STREAMS) return {};
        std::lock_guard<std::mutex> lk(mtx_);
        return latest_side_[id];
    }

    /** Signal that an entire Mid frame (codeword) was lost. Extends each
     *  stream's lost-run so the next received Mid packet attempts DRED
     *  (multi-frame) or inband-FEC (single-frame) recovery. */
    void onFrameLost() {
        std::lock_guard<std::mutex> lk(mtx_);
        for (size_t id = 0; id < MAX_STREAMS; ++id) {
            if (cfg_[id].enabled && rx_mid_[id] &&
                lost_run_[id] < MAX_RECOVER_FRAMES) {
                ++lost_run_[id];
            }
        }
    }

    /** Pull decoded PCM out of a stream's playback ring. */
    size_t pullRX(size_t id, float* pcm, size_t max_samples) {
        if (id >= MAX_STREAMS) return 0;
        return rx_output_[id]->read(pcm, max_samples);
    }

    /** Number of decoded samples buffered for a stream. */
    size_t rxAvailable(size_t id) const {
        if (id >= MAX_STREAMS) return 0;
        return rx_output_[id]->available();
    }

    /** Pull decoded Side PCM for a stereo stream. Returns 0 for mono
     *  streams (which never produce Side) and when no Side has been
     *  decoded since the last drain. The engine's drain stage pairs
     *  this with `pullRX(id)` to recombine L = Mid + Side / R =
     *  Mid − Side per stream. */
    size_t pullSideRX(size_t id, float* pcm, size_t max_samples) {
        if (id >= MAX_STREAMS || !side_output_[id]) return 0;
        return side_output_[id]->read(pcm, max_samples);
    }

    /** Number of decoded Side samples buffered for a stream. */
    size_t sideRxAvailable(size_t id) const {
        if (id >= MAX_STREAMS || !side_output_[id]) return 0;
        return side_output_[id]->available();
    }

    // -----------------------------------------------------------------
    // Bitrate budget allocation
    // -----------------------------------------------------------------

    /** Distribute a total bps budget proportionally across enabled streams
     *  using their configured weights. Updates the per-stream bitrates and
     *  re-creates encoders. */
    void allocateBudget(uint32_t total_bps) {
        std::lock_guard<std::mutex> lk(mtx_);
        float total_weight = 0.f;
        for (auto& s : cfg_) if (s.enabled) total_weight += std::max(0.f, s.weight);
        if (total_weight <= 0.f) return;
        for (size_t id = 0; id < MAX_STREAMS; ++id) {
            if (!cfg_[id].enabled) continue;
            float share = std::max(0.f, cfg_[id].weight) / total_weight;
            uint32_t bps = static_cast<uint32_t>(static_cast<float>(total_bps) * share);
            // Clamp total to Opus valid range (per encoder, not total)
            if (bps < 6000) bps = 6000;
            if (bps > 1020000) bps = 1020000;
            if (cfg_[id].bitrate_bps != bps) {
                cfg_[id].bitrate_bps = bps;
                // Split between Mid and Side encoders for stereo streams.
                const bool stereo = (cfg_[id].channels == 2);
                const float beta  = std::clamp(cfg_[id].mid_side_split,
                                                0.10f, 0.90f);
                uint32_t mid_bps  = stereo
                    ? static_cast<uint32_t>(static_cast<float>(bps) * beta)
                    : bps;
                uint32_t side_bps = stereo
                    ? static_cast<uint32_t>(static_cast<float>(bps) *
                                              (1.f - beta))
                    : 0u;
                auto clamp_bps = [](uint32_t b) {
                    if (b < 6000u) return 6000u;
                    if (b > 510000u) return 510000u;
                    return b;
                };
                mid_bps = clamp_bps(mid_bps);
                if (tx_mid_[id])  tx_mid_[id]->setBitrate(mid_bps);
                if (stereo && tx_side_[id]) {
                    tx_side_[id]->setBitrate(clamp_bps(side_bps));
                }
            }
        }
    }

private:
    mutable std::mutex mtx_;
    std::array<StreamConfig, MAX_STREAMS> cfg_;
    // Per-stream Mid encoder/decoder (mono). Always present for enabled
    // streams; carries (L+R)/2 for stereo or the raw audio for mono.
    std::array<std::unique_ptr<OpusAudioEncoder>, MAX_STREAMS> tx_mid_;
    std::array<std::unique_ptr<OpusAudioDecoder>, MAX_STREAMS> rx_mid_;
    // Per-stream Side encoder/decoder (mono). Present only for stereo
    // streams; carries (L−R)/2. nullptr for mono streams.
    std::array<std::unique_ptr<OpusAudioEncoder>, MAX_STREAMS> tx_side_;
    std::array<std::unique_ptr<OpusAudioDecoder>, MAX_STREAMS> rx_side_;
    // TX input ring: interleaved L,R for stereo streams, mono for mono.
    std::array<std::unique_ptr<FloatRingBuffer>, MAX_STREAMS> tx_input_;
    // RX output ring: mono Mid PCM (live playback). Decoded Side is
    // streamed in parallel via side_output_ for L/R recombination.
    std::array<std::unique_ptr<FloatRingBuffer>, MAX_STREAMS> rx_output_;
    // RX side ring: mono Side PCM for stereo streams, fed by the LP
    // decoder. Stays empty for mono streams. Drained by the engine to
    // recombine L = Mid + Side / R = Mid − Side per stream.
    std::array<std::unique_ptr<FloatRingBuffer>, MAX_STREAMS> side_output_;
    // Most recently decoded Side PCM per stream — kept for recorders
    // and snapshot consumers. Empty when Side missing this frame.
    std::array<std::vector<float>, MAX_STREAMS> latest_side_;
    // Consecutive lost Mid frames per stream (0 = none). Drives loss
    // recovery on the next received packet: DRED reconstructs the whole burst
    // (multi-frame), inband FEC the single previous frame. Capped so a long
    // outage can't trigger a pathological recovery burst.
    std::array<int, MAX_STREAMS> lost_run_{};
    static constexpr int MAX_RECOVER_FRAMES = 16;
    std::vector<float> scratch_;     // PCM read from tx_input_
    std::vector<float> mid_pcm_;     // M = (L+R)/2 buffer
    std::vector<float> side_pcm_;    // S = (L-R)/2 buffer
    std::vector<float> decode_buf_;  // Mid decoder output
    std::vector<float> side_decode_buf_;  // Side decoder output
    std::vector<float> dred_buf_;    // DRED burst-recovery output
};

} // namespace gw
