/**
 * @file hw_audio.hpp
 * @brief Hardware audio I/O using miniaudio
 *
 * Wraps miniaudio device API to connect real soundcard I/O to
 * SoundcardModem's ring buffers. Provides:
 *   - Playback device: reads from TX ring buffer → soundcard output
 *   - Capture device: soundcard input → writes to RX ring buffer
 *   - Device enumeration for GUI device selection
 *
 * Enable with GW_ENABLE_AUDIO in CMake. Without it, this header
 * provides stubs that always return false (loopback-only mode).
 *
 * Thread model: miniaudio callbacks run on dedicated audio threads.
 * They interact with RingBuffer which is lock-free SPSC, so no
 * additional synchronization needed.
 */
#pragma once

#include "types.hpp"
#include "soundcard_modem.hpp"
#include <string>
#include <vector>
#include <cstdint>

namespace gw {

struct AudioDeviceInfo {
    std::string name;
    uint32_t    id;        ///< opaque device index
    bool        is_default;
};

struct HWAudioConfig {
    uint32_t sample_rate     = 48000;
    uint32_t buffer_frames   = 512;   ///< Per-callback buffer size
    int      playback_device = -1;    ///< -1 = system default
    int      capture_device  = -1;    ///< -1 = system default
};

#ifdef GW_ENABLE_AUDIO

// Note: miniaudio's ma_device/ma_context are global-scope C structs. We
// don't forward-declare them here because doing so inside `namespace gw`
// would create incompatible `gw::ma_context` shadows that conflict with
// the real ones at link/use time. The pimpl `Impl` lives in hw_audio.cpp
// and references the global types directly via `::ma_context`.

class HWAudioDevice {
public:
    HWAudioDevice();
    ~HWAudioDevice();

    // Non-copyable
    HWAudioDevice(const HWAudioDevice&) = delete;
    HWAudioDevice& operator=(const HWAudioDevice&) = delete;

    /** Initialize audio context and enumerate devices */
    bool init();

    /** Start playback + capture connected to the modem's ring buffers.
     *  @param tx_ring  Ring buffer to read TX samples from (modem writes, soundcard reads)
     *  @param rx_ring  Ring buffer to write RX samples to (soundcard writes, modem reads)
     */
    bool start(RingBuffer& tx_ring, RingBuffer& rx_ring,
               const HWAudioConfig& cfg = HWAudioConfig());

    /** Start a PLAYBACK-ONLY device fed by `playback_ring`. Use this for the
     *  monitor channel that plays decoded RX audio out the system speakers
     *  even when the modem TX/RX path is purely software-loopback. */
    bool startPlaybackOnly(RingBuffer& playback_ring,
                           const HWAudioConfig& cfg = HWAudioConfig());

    /** Start a PLAYBACK-ONLY device with per-channel routing. Each entry in
     *  `channel_rings` feeds one output channel (channel `i` ← rings[i]).
     *  Null entries produce silence on that channel. The device is opened
     *  with `channel_rings.size()` channels and falls back to fewer if the
     *  OS refuses; surplus stream rings then wrap with modulo so they still
     *  produce audible mixed output (rather than silently dropping).
     *
     *  Use this to route per-stream decoded audio to distinct channels of a
     *  multi-channel virtual audio cable (VAC, VoiceMeeter) or to L/R of a
     *  stereo speaker pair. */
    bool startMultiChannelPlayback(const std::vector<RingBuffer*>& channel_rings,
                                   const HWAudioConfig& cfg = HWAudioConfig());

    /** Number of playback channels actually negotiated with the device.
     *  Valid only while running; 0 otherwise. */
    uint32_t actualPlaybackChannels() const;

    /** Start a CAPTURE-ONLY device that writes mic samples into `capture_ring`.
     *  Used when a stream's audio source is set to Microphone — the mic
     *  callback feeds the per-stream input ring directly. */
    bool startCaptureOnly(RingBuffer& capture_ring,
                          const HWAudioConfig& cfg = HWAudioConfig());

    /** Stop audio devices */
    void stop();

    /** Check if devices are running. Returns false if the device stopped
     *  unexpectedly (USB unplug / audio-service restart), detected via the
     *  miniaudio notification callback — this is what drives the engine's
     *  auto-recovery. */
    bool isRunning() const;

    /** Get available playback devices */
    const std::vector<AudioDeviceInfo>& playbackDevices() const { return pb_devices_; }

    /** Get available capture devices */
    const std::vector<AudioDeviceInfo>& captureDevices() const { return cap_devices_; }

    /** Query the supported sample rates of a device. Returns common SRs
     *  the device's native data formats can serve. Empty on error. */
    std::vector<uint32_t> supportedSampleRates(int playback_device,
                                                int capture_device) const;

    // Pimpl is public so the file-scope miniaudio data callback (which must
    // sit outside `namespace gw` to match ma_device_data_proc's signature)
    // can access Impl members. The struct definition is private to the cpp.
    struct Impl;

private:
    Impl* impl_ = nullptr;  ///< Pimpl to hide miniaudio types
    bool running_ = false;

    std::vector<AudioDeviceInfo> pb_devices_;
    std::vector<AudioDeviceInfo> cap_devices_;
};

#else // !GW_ENABLE_AUDIO

// Stub when hardware audio is not compiled in
class HWAudioDevice {
public:
    bool init() { return false; }
    bool start(RingBuffer& /*tx*/, RingBuffer& /*rx*/,
               const HWAudioConfig& /*cfg*/ = HWAudioConfig()) { return false; }
    bool startPlaybackOnly(RingBuffer& /*pb*/,
               const HWAudioConfig& /*cfg*/ = HWAudioConfig()) { return false; }
    bool startMultiChannelPlayback(const std::vector<RingBuffer*>& /*rings*/,
               const HWAudioConfig& /*cfg*/ = HWAudioConfig()) { return false; }
    uint32_t actualPlaybackChannels() const { return 0; }
    bool startCaptureOnly(RingBuffer& /*cap*/,
               const HWAudioConfig& /*cfg*/ = HWAudioConfig()) { return false; }
    void stop() {}
    bool isRunning() const { return false; }
    const std::vector<AudioDeviceInfo>& playbackDevices() const {
        static std::vector<AudioDeviceInfo> empty;
        return empty;
    }
    const std::vector<AudioDeviceInfo>& captureDevices() const {
        static std::vector<AudioDeviceInfo> empty;
        return empty;
    }
    std::vector<uint32_t> supportedSampleRates(int, int) const {
        return {};
    }
};

#endif // GW_ENABLE_AUDIO

} // namespace gw
