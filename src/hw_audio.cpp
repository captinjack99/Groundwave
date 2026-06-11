/**
 * @file hw_audio.cpp
 * @brief Hardware audio I/O implementation using miniaudio
 *
 * When GW_ENABLE_AUDIO is defined:
 *   - Includes miniaudio.h with implementation
 *   - Creates duplex device (playback + capture)
 *   - Callbacks read from tx_ring, write to rx_ring
 *
 * When not defined: stub file compiles to nothing.
 */

#ifdef GW_ENABLE_AUDIO

#define MINIAUDIO_IMPLEMENTATION
// Vendored single-header: suppress the int/size_t narrowing warnings it
// trips under /W4 (we removed those from the global suppression set so the
// DSP core gets them — see CMakeLists #25 — but we can't fix third-party
// source). Scoped to just this include via pragma push/pop so our own code
// below is still checked.
#ifdef _MSC_VER
#  pragma warning(push)
#  pragma warning(disable: 4244 4267)
#endif
#include "../external/miniaudio.h"
#ifdef _MSC_VER
#  pragma warning(pop)
#endif
#include "hw_audio.hpp"
#include <algorithm>
#include <array>
#include <atomic>
#include <cstring>

namespace gw {

// =========================================================================
// Pimpl Implementation (hides miniaudio types from header)
// =========================================================================

struct HWAudioDevice::Impl {
    ::ma_context context;
    ::ma_device  device;
    bool         context_inited = false;
    bool         device_inited  = false;

    // Ring buffer references (set during start())
    RingBuffer* tx_ring = nullptr;
    RingBuffer* rx_ring = nullptr;

    // Per-channel routing for multi-channel playback. When ch_count > 0,
    // the audio callback interleaves these rings into the device output
    // instead of broadcasting tx_ring across all channels. ch_rings[i]
    // feeds output channel i (wrapping with modulo if the device negotiated
    // fewer channels than rings were provided).
    static constexpr size_t MAX_CH_RINGS = 16;
    std::array<RingBuffer*, MAX_CH_RINGS> ch_rings{};
    size_t                                ch_count = 0;

    // Interleave scratch for the multi-channel playback callback. Lives
    // here (not on the realtime callback's stack) — the old code
    // value-initialized a 16×4096×4 B = 256 KB array on EVERY callback,
    // a large frame on the ~1 MB WASAPI audio thread and a pointless
    // per-call zero-init (every element used is written before read).
    // miniaudio invokes the data callback non-reentrantly per device, so
    // a shared member is safe. (#47)
    static constexpr size_t CB_SCRATCH = 4096;
    std::array<std::array<float, CB_SCRATCH>, MAX_CH_RINGS> cb_scratch;

    // Set by the miniaudio notification callback when the device stops
    // unexpectedly (USB unplug, audio-service restart, format change).
    // isRunning() reports false once this is set so the engine's
    // recovery path (recoverAudioDevicesIfNeeded) actually fires — the
    // previous `running_` flag only tracked "ever started" and never went
    // false on device loss, so recovery was dead code.
    std::atomic<bool> device_lost{false};
};

// =========================================================================
// Miniaudio Callback
// =========================================================================

// Duplex callback: reads from tx_ring for output, writes to rx_ring from input.
// Must be at file scope (NOT inside namespace gw) so its type matches the
// ma_device_data_proc function-pointer signature exactly.
} // namespace gw

static void audio_callback(::ma_device* pDevice, void* pOutput, const void* pInput,
                           ma_uint32 frameCount) {
    auto* impl = static_cast<gw::HWAudioDevice::Impl*>(pDevice->pUserData);
    if (!impl) return;

    auto* out = static_cast<float*>(pOutput);
    const auto* in = static_cast<const float*>(pInput);
    size_t frames = static_cast<size_t>(frameCount);

    // The device may have been initialized with 2 channels (some virtual
    // cables refuse mono). Our DSP rings are mono float, so convert here:
    //   - Playback: replicate each ring sample into all output channels.
    //   - Capture:  collapse all input channels to mono via sum/N average.
    ::ma_uint32 pb_ch  = pDevice->playback.channels;
    ::ma_uint32 cap_ch = pDevice->capture.channels;

    // Playback (multi-channel routing): interleave per-stream rings into
    // the device output. Channel i takes its samples from ch_rings[i]; if
    // the device negotiated fewer channels than rings were provided, the
    // surplus stream rings wrap around modulo pb_ch so they still mix into
    // the available channels rather than being silently dropped.
    if (impl->ch_count > 0 && out && pb_ch > 0) {
        constexpr size_t SCRATCH = gw::HWAudioDevice::Impl::CB_SCRATCH;
        // Per-channel temporaries (Impl-resident, not stack — see #47).
        // Pulling N samples per ring per chunk amortizes the atomic load
        // cost in RingBuffer::read.
        auto& ch_scratch = impl->cb_scratch;
        size_t pending = frames;
        size_t out_off = 0;
        while (pending > 0) {
            size_t take = std::min(pending, SCRATCH);
            // Pull `take` samples from each provided ring.
            for (size_t c = 0; c < impl->ch_count; ++c) {
                if (impl->ch_rings[c]) {
                    size_t got = impl->ch_rings[c]->read(ch_scratch[c].data(),
                                                          take);
                    if (got < take) {
                        std::memset(ch_scratch[c].data() + got, 0,
                                    (take - got) * sizeof(float));
                    }
                } else {
                    std::memset(ch_scratch[c].data(), 0,
                                take * sizeof(float));
                }
            }
            // Interleave into the device output frames.
            // pb_ch >= 1 is guaranteed by the surrounding `if`.
            for (size_t f = 0; f < take; ++f) {
                // Zero all output channels first, then sum stream rings
                // into their assigned channel (wrapping modulo pb_ch
                // when ch_count > pb_ch so excess streams still mix in).
                for (::ma_uint32 c = 0; c < pb_ch; ++c) {
                    out[(out_off + f) * pb_ch + c] = 0.f;
                }
                for (size_t s = 0; s < impl->ch_count; ++s) {
                    ::ma_uint32 dst = static_cast<::ma_uint32>(s % pb_ch);
                    out[(out_off + f) * pb_ch + dst] += ch_scratch[s][f];
                }
                // Soft clip the sum so multiple streams stacked into the
                // same channel can't drive output past full-scale.
                for (::ma_uint32 c = 0; c < pb_ch; ++c) {
                    float& v = out[(out_off + f) * pb_ch + c];
                    if      (v >  1.f) v =  1.f;
                    else if (v < -1.f) v = -1.f;
                }
            }
            out_off += take;
            pending -= take;
        }
    }
    // Playback (mono broadcast): legacy path. Reads mono samples from
    // tx_ring and broadcasts across all device channels.
    else if (impl->tx_ring && out && pb_ch > 0) {
        if (pb_ch == 1) {
            size_t got = impl->tx_ring->read(out, frames);
            if (got < frames) {
                std::memset(out + got, 0, (frames - got) * sizeof(float));
            }
        } else {
            // Read mono into a stack scratch (pull frames worth)
            constexpr size_t SCRATCH = 4096;
            float mono[SCRATCH];
            size_t pending = frames;
            size_t out_off = 0;
            while (pending > 0) {
                size_t want = std::min(pending, SCRATCH);
                size_t got  = impl->tx_ring->read(mono, want);
                if (got == 0) {
                    // Underrun — zero the rest of this output block
                    std::memset(out + out_off * pb_ch, 0,
                                pending * pb_ch * sizeof(float));
                    break;
                }
                for (size_t f = 0; f < got; ++f) {
                    float v = mono[f];
                    for (::ma_uint32 c = 0; c < pb_ch; ++c) {
                        out[(out_off + f) * pb_ch + c] = v;
                    }
                }
                out_off += got;
                pending -= got;
                if (got < want) {
                    std::memset(out + out_off * pb_ch, 0,
                                pending * pb_ch * sizeof(float));
                    break;
                }
            }
        }
    }

    // Capture: collapse cap_ch channels into mono, write to RX ring.
    if (impl->rx_ring && in && cap_ch > 0) {
        if (cap_ch == 1) {
            impl->rx_ring->write(in, frames);
        } else {
            // Heap allocation here is rare (only when cap_ch > 1 AND
            // frames > 4096); typical case stays on stack.
            constexpr size_t SCRATCH = 4096;
            float mono[SCRATCH];
            size_t in_off = 0;
            size_t pending = frames;
            float inv_ch = 1.f / static_cast<float>(cap_ch);
            while (pending > 0) {
                size_t take = std::min(pending, SCRATCH);
                for (size_t f = 0; f < take; ++f) {
                    float sum = 0.f;
                    for (::ma_uint32 c = 0; c < cap_ch; ++c) {
                        sum += in[(in_off + f) * cap_ch + c];
                    }
                    mono[f] = sum * inv_ch;
                }
                impl->rx_ring->write(mono, take);
                in_off  += take;
                pending -= take;
            }
        }
    }
}

// Device-state notification (file scope so the type matches
// ma_device_notification_proc). Fires for started / stopped / rerouted /
// interruption. We flag device_lost on an unexpected stop or interruption
// so HWAudioDevice::isRunning() reports false and the engine rebuilds the
// device. (A stop we initiate ourselves also fires this, but stop() has
// already cleared running_, and the next start() resets device_lost, so
// the self-stop case is harmless.)
static void device_notification_callback(
        const ::ma_device_notification* pNotification) {
    if (!pNotification || !pNotification->pDevice) return;
    auto* impl = static_cast<gw::HWAudioDevice::Impl*>(
        pNotification->pDevice->pUserData);
    if (!impl) return;
    if (pNotification->type == ma_device_notification_type_stopped ||
        pNotification->type == ma_device_notification_type_interruption_began) {
        impl->device_lost.store(true, std::memory_order_relaxed);
    }
}

namespace gw {

// =========================================================================
// HWAudioDevice Implementation
// =========================================================================

HWAudioDevice::HWAudioDevice() {
    impl_ = new Impl();
}

HWAudioDevice::~HWAudioDevice() {
    stop();
    if (impl_) {
        if (impl_->context_inited) {
            ::ma_context_uninit(&impl_->context);
        }
        delete impl_;
    }
}

bool HWAudioDevice::init() {
    if (impl_->context_inited) return true;

    ::ma_result result = ::ma_context_init(nullptr, 0, nullptr, &impl_->context);
    if (result != MA_SUCCESS) return false;
    impl_->context_inited = true;

    // Enumerate playback devices
    ::ma_device_info* pb_infos = nullptr;
    ::ma_uint32 pb_count = 0;
    ::ma_device_info* cap_infos = nullptr;
    ::ma_uint32 cap_count = 0;

    result = ::ma_context_get_devices(&impl_->context,
                                     &pb_infos, &pb_count,
                                     &cap_infos, &cap_count);
    if (result == MA_SUCCESS) {
        pb_devices_.clear();
        for (::ma_uint32 i = 0; i < pb_count; ++i) {
            AudioDeviceInfo info;
            info.name = pb_infos[i].name;
            info.id = i;
            info.is_default = pb_infos[i].isDefault != 0;
            pb_devices_.push_back(std::move(info));
        }

        cap_devices_.clear();
        for (::ma_uint32 i = 0; i < cap_count; ++i) {
            AudioDeviceInfo info;
            info.name = cap_infos[i].name;
            info.id = i;
            info.is_default = cap_infos[i].isDefault != 0;
            cap_devices_.push_back(std::move(info));
        }
    }

    return true;
}

bool HWAudioDevice::start(RingBuffer& tx_ring, RingBuffer& rx_ring,
                           const HWAudioConfig& cfg) {
    if (running_) stop();
    if (!impl_->context_inited && !init()) return false;

    impl_->tx_ring = &tx_ring;
    impl_->rx_ring = &rx_ring;

    // Resolve device IDs
    ::ma_device_info* pb_infos = nullptr;
    ::ma_uint32 pb_count = 0;
    ::ma_device_info* cap_infos = nullptr;
    ::ma_uint32 cap_count = 0;
    if (::ma_context_get_devices(&impl_->context,
                                  &pb_infos, &pb_count,
                                  &cap_infos, &cap_count) != MA_SUCCESS) {
        return false;
    }

    ::ma_device_id* pb_id = nullptr;
    ::ma_device_id* cap_id = nullptr;
    if (cfg.playback_device >= 0 &&
        static_cast<::ma_uint32>(cfg.playback_device) < pb_count) {
        pb_id = &pb_infos[cfg.playback_device].id;
    }
    if (cfg.capture_device >= 0 &&
        static_cast<::ma_uint32>(cfg.capture_device) < cap_count) {
        cap_id = &cap_infos[cfg.capture_device].id;
    }

    // Try a sequence of channel-count combinations. Some virtual-cable
    // drivers (e.g. VB-CABLE 16ch) refuse mono and require ≥ 2 channels.
    // miniaudio will down-mix to mono inside the callback so our DSP
    // still sees 1-channel data — but the OS device must accept the
    // negotiated channel count first. (Channel counts are explicitly
    // requested in the config; miniaudio's auto-conversion handles the
    // rest in user-space when device formats differ.)
    struct ChannelTry { ::ma_uint32 pb_ch, cap_ch; };
    const ChannelTry attempts[] = {
        {1, 1},   // ideal: mono in/out
        {2, 1},   // VAC 16ch playback often refuses mono
        {1, 2},   // some interfaces capture-only-stereo
        {2, 2},
        {0, 0},   // 0 = "use device's native channel count"
    };

    ::ma_result last_err = MA_ERROR;
    for (const auto& a : attempts) {
        ::ma_device_config config =
            ::ma_device_config_init(::ma_device_type_duplex);
        config.sampleRate         = cfg.sample_rate;
        config.playback.format    = ::ma_format_f32;
        config.playback.channels  = a.pb_ch;
        config.capture.format     = ::ma_format_f32;
        config.capture.channels   = a.cap_ch;
        config.periodSizeInFrames = cfg.buffer_frames;
        config.dataCallback         = audio_callback;
        config.notificationCallback = device_notification_callback;
        config.pUserData          = impl_;
        if (pb_id)  config.playback.pDeviceID = pb_id;
        if (cap_id) config.capture.pDeviceID  = cap_id;

        last_err = ::ma_device_init(&impl_->context, &config, &impl_->device);
        if (last_err == MA_SUCCESS) {
            impl_->device_inited = true;
            last_err = ::ma_device_start(&impl_->device);
            if (last_err == MA_SUCCESS) {
                running_ = true;
                impl_->device_lost.store(false, std::memory_order_relaxed);
                return true;
            }
            ::ma_device_uninit(&impl_->device);
            impl_->device_inited = false;
        }
    }
    return false;
}

void HWAudioDevice::stop() {
    if (impl_->device_inited) {
        ::ma_device_stop(&impl_->device);
        ::ma_device_uninit(&impl_->device);
        impl_->device_inited = false;
    }
    impl_->tx_ring = nullptr;
    impl_->rx_ring = nullptr;
    impl_->ch_rings.fill(nullptr);
    impl_->ch_count = 0;
    running_ = false;
}

bool HWAudioDevice::startPlaybackOnly(RingBuffer& playback_ring,
                                       const HWAudioConfig& cfg) {
    if (running_) stop();
    if (!impl_->context_inited && !init()) return false;

    // tx_ring drives the audio_callback's playback path; we leave rx_ring
    // null so the capture branch is skipped naturally.
    impl_->tx_ring = &playback_ring;
    impl_->rx_ring = nullptr;

    // Negotiate channels: try mono, fall back to stereo (mono → stereo
    // dup happens in audio_callback).
    ::ma_uint32 ch_attempts[] = {1, 2, 0};
    for (::ma_uint32 ch : ch_attempts) {
        ::ma_device_config config =
            ::ma_device_config_init(::ma_device_type_playback);
        config.sampleRate         = cfg.sample_rate;
        config.playback.format    = ::ma_format_f32;
        config.playback.channels  = ch;
        config.periodSizeInFrames = cfg.buffer_frames;
        config.dataCallback         = audio_callback;
        config.notificationCallback = device_notification_callback;
        config.pUserData          = impl_;

        if (::ma_device_init(&impl_->context, &config, &impl_->device) == MA_SUCCESS) {
            impl_->device_inited = true;
            if (::ma_device_start(&impl_->device) == MA_SUCCESS) {
                running_ = true;
                impl_->device_lost.store(false, std::memory_order_relaxed);
                return true;
            }
            ::ma_device_uninit(&impl_->device);
            impl_->device_inited = false;
        }
    }
    return false;
}

bool HWAudioDevice::startMultiChannelPlayback(
        const std::vector<RingBuffer*>& channel_rings,
        const HWAudioConfig& cfg)
{
    if (running_) stop();
    if (!impl_->context_inited && !init()) return false;
    if (channel_rings.empty()) return false;

    // Stash the rings into the Impl so audio_callback can find them.
    size_t n = std::min<size_t>(channel_rings.size(),
                                 gw::HWAudioDevice::Impl::MAX_CH_RINGS);
    impl_->ch_rings.fill(nullptr);
    for (size_t i = 0; i < n; ++i) impl_->ch_rings[i] = channel_rings[i];
    impl_->ch_count = n;
    impl_->tx_ring = nullptr;
    impl_->rx_ring = nullptr;

    ::ma_device_id* pb_id = nullptr;
    if (cfg.playback_device >= 0) {
        ::ma_device_info* pb_infos = nullptr;
        ::ma_uint32 pb_count = 0;
        ::ma_device_info* cap_infos = nullptr;
        ::ma_uint32 cap_count = 0;
        if (::ma_context_get_devices(&impl_->context,
                                      &pb_infos, &pb_count,
                                      &cap_infos, &cap_count) == MA_SUCCESS) {
            if (static_cast<::ma_uint32>(cfg.playback_device) < pb_count) {
                pb_id = &pb_infos[cfg.playback_device].id;
            }
        }
    }

    // Channel-count negotiation: try the full set first, then progressively
    // smaller standard layouts. The callback handles wrap-around when the
    // device gives us fewer channels than rings were provided.
    ::ma_uint32 desired = static_cast<::ma_uint32>(n);
    ::ma_uint32 attempts[5];
    int a = 0;
    attempts[a++] = desired;
    if (desired > 2) attempts[a++] = 2;
    if (desired != 1) attempts[a++] = 1;
    attempts[a++] = 0;  // native
    int attempt_count = a;

    for (int i = 0; i < attempt_count; ++i) {
        ::ma_uint32 ch = attempts[i];
        // Skip duplicates introduced by clamping (e.g., desired == 2).
        bool dup = false;
        for (int j = 0; j < i; ++j) {
            if (attempts[j] == ch) { dup = true; break; }
        }
        if (dup) continue;

        ::ma_device_config config =
            ::ma_device_config_init(::ma_device_type_playback);
        config.sampleRate         = cfg.sample_rate;
        config.playback.format    = ::ma_format_f32;
        config.playback.channels  = ch;
        config.periodSizeInFrames = cfg.buffer_frames;
        config.dataCallback         = audio_callback;
        config.notificationCallback = device_notification_callback;
        config.pUserData          = impl_;
        if (pb_id) config.playback.pDeviceID = pb_id;

        if (::ma_device_init(&impl_->context, &config, &impl_->device) == MA_SUCCESS) {
            impl_->device_inited = true;
            if (::ma_device_start(&impl_->device) == MA_SUCCESS) {
                running_ = true;
                impl_->device_lost.store(false, std::memory_order_relaxed);
                return true;
            }
            ::ma_device_uninit(&impl_->device);
            impl_->device_inited = false;
        }
    }

    // Failed — clear the routing so a later call to start() works cleanly.
    impl_->ch_rings.fill(nullptr);
    impl_->ch_count = 0;
    return false;
}

uint32_t HWAudioDevice::actualPlaybackChannels() const {
    if (!impl_ || !impl_->device_inited) return 0;
    return static_cast<uint32_t>(impl_->device.playback.channels);
}

bool HWAudioDevice::isRunning() const {
    // running_ tracks whether we started; device_lost is set by the
    // miniaudio notification callback on an unexpected stop. Both must hold
    // for the device to be considered live.
    return running_ && impl_ && !impl_->device_lost.load(std::memory_order_relaxed);
}

bool HWAudioDevice::startCaptureOnly(RingBuffer& capture_ring,
                                      const HWAudioConfig& cfg) {
    if (running_) stop();
    if (!impl_->context_inited && !init()) return false;

    impl_->tx_ring = nullptr;
    impl_->rx_ring = &capture_ring;

    ::ma_uint32 ch_attempts[] = {1, 2, 0};
    for (::ma_uint32 ch : ch_attempts) {
        ::ma_device_config config =
            ::ma_device_config_init(::ma_device_type_capture);
        config.sampleRate         = cfg.sample_rate;
        config.capture.format     = ::ma_format_f32;
        config.capture.channels   = ch;
        config.periodSizeInFrames = cfg.buffer_frames;
        config.dataCallback         = audio_callback;
        config.notificationCallback = device_notification_callback;
        config.pUserData          = impl_;

        if (::ma_device_init(&impl_->context, &config, &impl_->device) == MA_SUCCESS) {
            impl_->device_inited = true;
            if (::ma_device_start(&impl_->device) == MA_SUCCESS) {
                running_ = true;
                impl_->device_lost.store(false, std::memory_order_relaxed);
                return true;
            }
            ::ma_device_uninit(&impl_->device);
            impl_->device_inited = false;
        }
    }
    return false;
}

std::vector<uint32_t> HWAudioDevice::supportedSampleRates(
        int playback_device, int capture_device) const
{
    std::vector<uint32_t> result;
    if (!impl_ || !impl_->context_inited) return result;

    // Take a snapshot of the device list. The pointers returned by
    // ma_context_get_devices are owned by the context; we must NOT mutate
    // the structs in-place when calling ma_context_get_device_info — that
    // call writes back into the supplied buffer, so we use a fresh local.
    ::ma_device_info* pb_infos = nullptr;
    ::ma_uint32 pb_count = 0;
    ::ma_device_info* cap_infos = nullptr;
    ::ma_uint32 cap_count = 0;
    if (::ma_context_get_devices(
            const_cast<::ma_context*>(&impl_->context),
            &pb_infos, &pb_count, &cap_infos, &cap_count) != MA_SUCCESS) {
        return result;
    }

    auto collectFromInfo = [&](::ma_device_info* infos, ::ma_uint32 count,
                                int idx, ::ma_device_type kind,
                                std::vector<uint32_t>& sink) {
        if (idx < 0 || static_cast<::ma_uint32>(idx) >= count) return;
        // Copy the id (only the id field is reliably stable across the
        // returned pointer's lifetime), then query into a LOCAL info struct.
        ::ma_device_info detailed{};
        detailed.id = infos[idx].id;
        if (::ma_context_get_device_info(
                const_cast<::ma_context*>(&impl_->context),
                kind, &detailed.id, &detailed) != MA_SUCCESS) {
            return;
        }
        for (::ma_uint32 i = 0; i < detailed.nativeDataFormatCount; ++i) {
            ::ma_uint32 sr = detailed.nativeDataFormats[i].sampleRate;
            if (sr > 0) sink.push_back(sr);
        }
    };

    std::vector<uint32_t> pb_rates;
    std::vector<uint32_t> cap_rates;
    collectFromInfo(pb_infos,  pb_count,  playback_device,
                    ::ma_device_type_playback, pb_rates);
    collectFromInfo(cap_infos, cap_count, capture_device,
                    ::ma_device_type_capture,  cap_rates);

    // Intersection: rates supported by BOTH devices (so duplex works).
    if (pb_rates.empty() && cap_rates.empty()) {
        // Nothing usable from either device — fall back to the standard
        // soundcard rates so the SR combo still works.
        return result;
    }
    if (pb_rates.empty()) result = cap_rates;
    else if (cap_rates.empty()) result = pb_rates;
    else {
        for (uint32_t r : pb_rates) {
            for (uint32_t c : cap_rates) {
                if (r == c) { result.push_back(r); break; }
            }
        }
    }
    // De-dup + sort
    std::sort(result.begin(), result.end());
    result.erase(std::unique(result.begin(), result.end()), result.end());
    return result;
}

} // namespace gw

#endif // GW_ENABLE_AUDIO
