/**
 * @file app_state.hpp
 * @brief Application state — shared data between DSP engine and GUI
 *
 * AppState is the single source of truth for all runtime configuration,
 * live measurements, and persistent preset data. It is designed to be
 * updated by the DSP/audio thread and read by the GUI thread; callers
 * must hold the mutex when modifying shared fields.
 *
 * Preset system: 8 named slots, each storing a complete modem config.
 * Alarm system:  per-parameter thresholds; updateAlarms() checks stats.
 */

#pragma once

#include "types.hpp"
#include "soundcard_modem.hpp"
#include "multi_stream.hpp"
#include "hierarchical_mod.hpp"
#include <array>
#include <string>
#include <vector>
#include <mutex>
#include <chrono>
#include <cstring>
#include <cmath>

namespace dsca {

// =========================================================================
// Preset Configuration
// =========================================================================

static constexpr size_t NUM_PRESETS = 8;

struct PresetConfig {
    char        name[64]     = "Untitled";
    bool        valid        = false;
    OFDMParams  ofdm;
    FrameParams frame;
    ModemConfig modem;

    void setName(const char* n) {
        std::strncpy(name, n, sizeof(name) - 1);
        name[sizeof(name)-1] = '\0';
    }
    std::string getName() const { return std::string(name); }
};

/** Built-in factory presets */
inline void initFactoryPresets(std::array<PresetConfig, NUM_PRESETS>& presets) {
    // 0 — Robust (narrowband, BPSK, strong FEC)
    {
        auto& p = presets[0];
        p.setName("Robust");
        p.valid = true;
        p.ofdm.fft_size       = 256;
        p.ofdm.modulation     = Modulation::BPSK;
        p.ofdm.cyclic_prefix  = CyclicPrefix::CP_1_4;
        p.frame.fec_rate      = FECRate::Rate_1_4;
        p.modem.center_freq   = 12000.f;
        p.modem.sample_rate   = 48000;
    }
    // 1 — Standard (QPSK 1/2, default params)
    {
        auto& p = presets[1];
        p.setName("Standard");
        p.valid = true;
        p.ofdm.fft_size       = 256;
        p.ofdm.modulation     = Modulation::QPSK;
        p.ofdm.cyclic_prefix  = CyclicPrefix::CP_1_8;
        p.frame.fec_rate      = FECRate::Rate_1_2;
        p.modem.center_freq   = 12000.f;
        p.modem.sample_rate   = 48000;
    }
    // 2 — HD Audio (QAM16 3/4)
    {
        auto& p = presets[2];
        p.setName("HD Audio");
        p.valid = true;
        p.ofdm.fft_size       = 512;
        p.ofdm.modulation     = Modulation::QAM16;
        p.ofdm.cyclic_prefix  = CyclicPrefix::CP_1_8;
        p.frame.fec_rate      = FECRate::Rate_3_4;
        p.modem.center_freq   = 12000.f;
        p.modem.sample_rate   = 48000;
    }
    // 3 — High Capacity (QAM64 4/5)
    {
        auto& p = presets[3];
        p.setName("High Capacity");
        p.valid = true;
        p.ofdm.fft_size       = 1024;
        p.ofdm.modulation     = Modulation::QAM64;
        p.ofdm.cyclic_prefix  = CyclicPrefix::CP_1_16;
        p.frame.fec_rate      = FECRate::Rate_4_5;
        p.modem.center_freq   = 12000.f;
        p.modem.sample_rate   = 48000;
    }
    // 4 — Ultra HD (QAM256 8/9, 96kHz)
    {
        auto& p = presets[4];
        p.setName("Ultra HD");
        p.valid = true;
        p.ofdm.fft_size       = 2048;
        p.ofdm.modulation     = Modulation::QAM256;
        p.ofdm.cyclic_prefix  = CyclicPrefix::CP_1_32;
        p.ofdm.sample_rate    = 96000;
        p.frame.fec_rate      = FECRate::Rate_8_9;
        p.modem.sample_rate   = 96000;
        p.modem.center_freq   = 24000.f;
    }
    // 5 — Broadcast Studio (QAM1024, 192kHz, 9/10 FEC)
    {
        auto& p = presets[5];
        p.setName("Broadcast Studio");
        p.valid = true;
        p.ofdm.fft_size       = 4096;
        p.ofdm.modulation     = Modulation::QAM1024;
        p.ofdm.cyclic_prefix  = CyclicPrefix::CP_1_32;
        p.ofdm.sample_rate    = 192000;
        p.frame.fec_rate      = FECRate::Rate_9_10;
        p.modem.sample_rate   = 192000;
        p.modem.center_freq   = 48000.f;
    }
    // 6 — Emergency (BPSK 1/4, max robustness)
    {
        auto& p = presets[6];
        p.setName("Emergency");
        p.valid = true;
        p.ofdm.fft_size       = 128;
        p.ofdm.modulation     = Modulation::BPSK;
        p.ofdm.cyclic_prefix  = CyclicPrefix::CP_1_4;
        p.frame.fec_rate      = FECRate::Rate_1_4;
        p.modem.center_freq   = 12000.f;
        p.modem.sample_rate   = 48000;
    }
    // 7 — Custom (empty, user-editable)
    {
        auto& p = presets[7];
        p.setName("Custom");
        p.valid = false;
    }
}

// =========================================================================
// Alarm System
// =========================================================================

struct AlarmThresholds {
    float snr_low_db    =  6.0f;  ///< Alarm when SNR falls below this
    float ber_high      =  1e-3f; ///< Alarm when BER exceeds this
    float evm_high_pct  = 30.0f;  ///< Alarm when EVM% exceeds this
    float level_low_db  = -40.0f; ///< Alarm when RX level below this
    float level_high_db =  -3.0f; ///< Alarm when RX level above this (clip)
    float agc_ripple_db =  6.0f;  ///< Alarm when AGC ripple exceeds this (pumping)
    bool  alarm_sync_loss   = true;
    bool  alarm_audio_clip  = true;
    bool  alarm_agc_pump    = true;  ///< Enable AGC-pumping alarm
};

struct AlarmEvent {
    using Clock = std::chrono::steady_clock;
    enum class Type : uint8_t {
        SNR_LOW, BER_HIGH, EVM_HIGH, SYNC_LOST, AUDIO_CLIP,
        LEVEL_LOW, LEVEL_HIGH, ALARM_CLEAR
    };
    Type        type;
    float       value;
    Clock::time_point timestamp;
    bool        active;

    const char* typeName() const {
        switch (type) {
            case Type::SNR_LOW:    return "SNR Low";
            case Type::BER_HIGH:   return "BER High";
            case Type::EVM_HIGH:   return "EVM High";
            case Type::SYNC_LOST:  return "Sync Lost";
            case Type::AUDIO_CLIP: return "Audio Clip";
            case Type::LEVEL_LOW:  return "Level Low";
            case Type::LEVEL_HIGH: return "Level High";
            case Type::ALARM_CLEAR:return "Alarm Clear";
            default: return "Unknown";
        }
    }
};

struct AlarmStatus {
    bool snr_low      = false;
    bool ber_high     = false;
    bool evm_high     = false;
    bool sync_lost    = false;
    bool audio_clipped = false;
    bool level_low    = false;
    bool level_high   = false;
    bool agc_pumping  = false;   ///< AGC ripple exceeds threshold
    bool muted        = false;   ///< Alarm output muted (but still tracked)

    bool anyActive() const {
        return snr_low || ber_high || evm_high ||
               sync_lost || audio_clipped || level_low || level_high ||
               agc_pumping;
    }

    /** Overall alarm severity: 0=ok, 1=warning, 2=critical */
    int severity() const {
        if (sync_lost || audio_clipped || level_high) return 2;
        if (snr_low || ber_high || evm_high || level_low ||
            agc_pumping) return 1;
        return 0;
    }
};

static constexpr size_t ALARM_LOG_SIZE = 64;

// =========================================================================
// Spectrum / Waterfall Data
// =========================================================================

static constexpr size_t SPECTRUM_BINS    = 512;  ///< Display resolution
static constexpr size_t WATERFALL_ROWS   = 128;  ///< History lines

struct SpectrumData {
    std::array<float, SPECTRUM_BINS> freqs_hz  = {};
    std::array<float, SPECTRUM_BINS> power_db  = {};
    // Count of analyzer updates (= waterfall rows pushed to the display). The
    // SpectrumWidget owns the actual scrolling waterfall image and builds each
    // row from `power_db`, so no full history buffer is kept here — the old
    // SPECTRUM_BINS×WATERFALL_ROWS array was 256 KB of never-rendered dead
    // state refilled every frame (#49). This counter is the one signal
    // consumers (and tests) need: that fresh spectrum frames are arriving.
    size_t  waterfall_write_row = 0;
    float   peak_db      = -80.f;
    float   noise_floor  = -80.f;
    float   sample_rate  = 48000.f;

    void initFreqs(float sr) {
        sample_rate = sr;
        const float bin_hz = sr / (2.f * static_cast<float>(SPECTRUM_BINS));
        for (size_t i = 0; i < SPECTRUM_BINS; ++i) {
            freqs_hz[i] = static_cast<float>(i) * bin_hz;
        }
    }

    void clear() {
        power_db.fill(-80.f);
        waterfall_write_row = 0;
        peak_db = -80.f;
        noise_floor = -80.f;
    }
};

// =========================================================================
// Constellation Data
// =========================================================================

static constexpr size_t CONSTELLATION_MAX = 2048;

struct ConstellationData {
    std::vector<float> i_vals; ///< In-phase values
    std::vector<float> q_vals; ///< Quadrature values
    size_t write_pos = 0;
    bool   full      = false;

    ConstellationData() {
        i_vals.resize(CONSTELLATION_MAX, 0.f);
        q_vals.resize(CONSTELLATION_MAX, 0.f);
    }

    void push(float i, float q) {
        i_vals[write_pos] = i;
        q_vals[write_pos] = q;
        write_pos = (write_pos + 1) % CONSTELLATION_MAX;
        if (write_pos == 0) full = true;
    }

    void push(const ComplexBuf& syms) {
        for (const auto& s : syms) push(s.real(), s.imag());
    }

    size_t count() const {
        return full ? CONSTELLATION_MAX : write_pos;
    }

    void clear() {
        write_pos = 0;
        full = false;
        std::fill(i_vals.begin(), i_vals.end(), 0.f);
        std::fill(q_vals.begin(), q_vals.end(), 0.f);
    }
};

// =========================================================================
// Level Meters
// =========================================================================

struct LevelMeter {
    static constexpr float PEAK_HOLD_SECS = 2.0f;
    static constexpr float DECAY_DB_PER_SEC = 20.0f;

    float   rms_db      = -60.f;
    float   peak_db     = -60.f;
    float   clip_db     = 0.f;   ///< Full-scale reference
    bool    clipping    = false;
    using Clock = std::chrono::steady_clock;
    Clock::time_point peak_hold_time = Clock::now();

    void update(float new_rms_db, float dt_sec) {
        rms_db = new_rms_db;
        clipping = (new_rms_db >= -0.5f);
        if (new_rms_db > peak_db) {
            peak_db = new_rms_db;
            peak_hold_time = Clock::now();
        } else {
            auto elapsed = std::chrono::duration<float>(
                Clock::now() - peak_hold_time).count();
            if (elapsed > PEAK_HOLD_SECS) {
                peak_db -= DECAY_DB_PER_SEC * dt_sec;
                if (peak_db < -60.f) peak_db = -60.f;
            }
        }
    }

    /** 0.0 = -60 dBFS, 1.0 = 0 dBFS */
    float normalizedLevel() const {
        return std::max(0.f, (rms_db + 60.f) / 60.f);
    }
    float normalizedPeak() const {
        return std::max(0.f, (peak_db + 60.f) / 60.f);
    }
};

// =========================================================================
// Computed Modem Parameters (derived, not stored)
// =========================================================================

struct ComputedParams {
    size_t active_subcarriers   = 0;
    size_t data_subcarriers     = 0;
    size_t pilot_subcarriers    = 0;
    size_t bits_per_ofdm_sym    = 0;
    float  symbol_duration_ms   = 0.f;
    float  subcarrier_spacing_hz = 0.f;

    // Bit-rate breakdown — what the user pays, where each cost goes.
    //   gross_bitrate_bps  = data_subcarriers × bps / symbol_duration
    //                        (raw OFDM payload before any coding)
    //   fec_coded_bps      = gross × FEC code rate
    //                        (after LDPC parity overhead)
    //   net_bitrate_bps    = fec_coded × (1 − rs_overhead_fraction)
    //                        (after Reed-Solomon parity overhead if enabled)
    //
    // The 10–20% drop from gross → fec_coded is FEC overhead (1/4 .. 9/10).
    // The 6–24% drop from fec_coded → net is RS overhead (block-size
    // dependent: 16 parity bytes per LDPC info block).
    float  gross_bitrate_bps    = 0.f;
    float  fec_coded_bitrate_bps = 0.f;
    float  net_bitrate_bps      = 0.f;

    float  spectral_eff_bps_hz  = 0.f;
    float  signal_bandwidth_hz  = 0.f;     ///< OFDM-OCCUPIED bandwidth
                                           ///  (active_subcarriers × spacing)
    float  cp_overhead_pct      = 0.f;     ///< CP airtime cost (1/4..1/32)
    float  pilot_overhead_pct   = 0.f;     ///< Pilot subcarrier fraction
    float  fec_overhead_pct     = 0.f;     ///< 1 − code_rate
    float  rs_overhead_pct      = 0.f;     ///< 16 / k_bytes (0 if RS off)

    /** @param rs_enabled  Whether the Reed-Solomon outer code is active.
     *                     When true, net bitrate accounts for the 16-byte
     *                     RS parity per LDPC info block (block-size-
     *                     dependent overhead). When false, net == fec_coded. */
    static ComputedParams compute(const OFDMParams& o, const FrameParams& f,
                                   bool rs_enabled = true) {
        ComputedParams c;
        c.subcarrier_spacing_hz = static_cast<float>(
            static_cast<double>(o.sample_rate) / o.fft_size);
        c.symbol_duration_ms = 1000.f * static_cast<float>(o.symbolDuration());

        // Count data / pilot subcarriers EXACTLY as the modem does
        // (computeAllocation() in ofdm.cpp is the single source of truth): the
        // active region is the FULL FFT span [guardLeft, fft_size - guardRight);
        // DC (bin 0) is skipped when nulled; every pilot_spacing-th active
        // carrier is a pilot, the rest data.
        //
        // This replaces a divergent hand-rolled formula that (a) used
        // fft_size/2 (half the real count) and (b) UNDERFLOWED — `usable` is a
        // size_t, and when the auto-guard maxed out at fft_size/4 per side,
        // fft_size/2 - guardLeft - guardRight - dc went negative and wrapped to
        // ~1.8e19, sending the displayed bitrate to absurd values. Because the
        // guard size depends on the subcarrier spacing (= sample_rate/fft_size),
        // changing the sample rate is exactly what tripped it.
        size_t gl  = o.guardLeft();
        size_t gr  = o.guardRight();
        size_t end   = (o.fft_size > gr) ? (o.fft_size - gr) : 0;
        size_t start = (gl < end) ? gl : end;
        size_t psp   = (o.pilot_spacing > 0) ? o.pilot_spacing : 1;
        size_t pilots = 0, data = 0, carrier_count = 0;
        for (size_t i = start; i < end; ++i) {
            // Mirror computeAllocation's logical→physical mapping: logical
            // index i = fft_size/2 lands on physical bin 0 (DC), which the
            // dc_null option keeps empty. (The old `i == 0` check could
            // never fire — start >= 1 — and over-counted by one carrier.)
            if (o.dc_null && i == o.fft_size / 2) continue; // DC bin
            if (carrier_count % psp == 0) ++pilots; else ++data;
            ++carrier_count;
        }
        size_t active = data + pilots;
        c.pilot_subcarriers  = pilots;
        c.data_subcarriers   = data;
        c.active_subcarriers = active;
        c.pilot_overhead_pct = active > 0
            ? 100.f * static_cast<float>(pilots) / static_cast<float>(active)
            : 0.f;

        uint8_t bps = bitsPerSymbol(o.modulation);
        c.bits_per_ofdm_sym  = c.data_subcarriers * bps;

        float sym_dur_s = static_cast<float>(o.symbolDuration());
        // CP overhead: 1 − (fft_size / symbol_length).
        if (sym_dur_s > 0.f) {
            c.gross_bitrate_bps = static_cast<float>(c.bits_per_ofdm_sym) / sym_dur_s;
            float useful_dur = static_cast<float>(o.fft_size) /
                               static_cast<float>(o.sample_rate);
            c.cp_overhead_pct = 100.f * (1.f - useful_dur / sym_dur_s);
        }
        float cr = codeRateValue(f.fec_rate);
        c.fec_coded_bitrate_bps = c.gross_bitrate_bps * cr;
        c.fec_overhead_pct      = 100.f * (1.f - cr);

        // RS overhead: 16 parity bytes per LDPC info block. The fraction
        // depends on the block size. Approximate using the rate to map
        // codeword bytes back to info bytes: for short LDPC (n=2160 bits =
        // 270 bytes), k = 270 × code_rate. RS overhead = 16 / k.
        c.rs_overhead_pct = 0.f;
        if (rs_enabled) {
            constexpr int N_BYTES_SHORT = 270;
            float k_bytes = static_cast<float>(N_BYTES_SHORT) * cr;
            if (k_bytes > 16.f) {
                c.rs_overhead_pct = 100.f * (16.f / k_bytes);
            }
        }
        c.net_bitrate_bps = c.fec_coded_bitrate_bps *
                            (1.f - c.rs_overhead_pct / 100.f);

        c.signal_bandwidth_hz = c.active_subcarriers * c.subcarrier_spacing_hz;
        if (c.signal_bandwidth_hz > 0.f) {
            c.spectral_eff_bps_hz = c.net_bitrate_bps / c.signal_bandwidth_hz;
        }
        return c;
    }
};

// =========================================================================
// Link-budget panel inputs
// =========================================================================

// User-entered link-budget parameters. Owned by AppState so they persist
// across restarts (serialized to config) rather than living only in the
// panel's spinboxes. Defaults mirror the panel's initial widget values.
struct LinkBudgetInputs {
    float tx_power_w   = 1.0f;
    float tx_gain_db   = 0.0f;
    float rx_gain_db   = 0.0f;
    float freq_mhz     = 98.0f;
    float tx_height_m  = 30.0f;
    float rx_height_m  = 2.0f;
    int   terrain_idx  = 2;
    float nf_db        = 8.0f;
    float margin_db    = 3.0f;
};

// =========================================================================
// AppState — master application state
// =========================================================================

struct AppState {
    // ---- Configuration (live) ----
    OFDMParams  ofdm;
    FrameParams frame;
    ModemConfig modem;

    // ---- Presets ----
    std::array<PresetConfig, NUM_PRESETS> presets;
    int  active_preset_slot = -1;  ///< -1 = custom (not from preset)
    bool preset_modified    = false;

    // ---- Alarm system ----
    AlarmThresholds  alarm_thresh;
    AlarmStatus      alarm_status;
    std::vector<AlarmEvent> alarm_log; ///< Circular log
    /// Monotonic count of events ever appended to alarm_log. The panel's
    /// change detection compares against this — comparing the list-widget
    /// count to alarm_log.size() froze the log display permanently once
    /// the circular buffer filled (size pinned at ALARM_LOG_SIZE).
    uint64_t alarm_log_revision = 0;

    // ---- Live measurements ----
    ModemStats      stats;
    LevelMeter      tx_meter;
    LevelMeter      rx_meter;
    float           agc_gain_db  = 0.f;

    // ---- Visualization ----
    SpectrumData    spectrum;
    ConstellationData constellation;

    // ---- TX control ----
    bool  tx_enabled  = false;
    float tx_gain_db  = 0.f;   ///< TX output gain

    // ---- Multi-stream audio configuration (up to MAX_STREAMS) ----
    std::array<StreamConfig, MAX_STREAMS> stream_configs{};
    std::array<float,        MAX_STREAMS> stream_rms_db{};   // RX level per stream

    // ---- Hierarchical modulation (shadows AudioEngineConfig::hier) ----
    // Owned by AppState so GUI panels (TX, LinkBudget) can read it
    // directly without coupling to AudioEngine. MainWindow syncs this
    // with the engine's config on apply.
    HierarchicalConfig hier;

    // ---- Link-budget panel inputs (persisted across restarts) ----
    LinkBudgetInputs link_budget;

    // ---- Thread safety ----
    mutable std::mutex mtx;

    // (GUI window visibility is managed by Qt QDialog/QAction — not stored here)

    AppState() {
        initFactoryPresets(presets);
        spectrum.initFreqs(static_cast<float>(ofdm.sample_rate));
        alarm_log.reserve(ALARM_LOG_SIZE);
        // Apply preset 1 (Standard) as default
        applyPreset(1);

        // Seed multi-stream configs: stream 0 enabled by default, others off.
        for (size_t i = 0; i < MAX_STREAMS; ++i) {
            auto& sc = stream_configs[i];
            sc.enabled     = (i == 0);
            sc.sample_rate = 48000;
            sc.channels    = 1;
            sc.bitrate_bps = 24000;
            sc.weight      = 1.0f;
            sc.frame_ms    = 20;
            sc.app         = OpusApplication::Audio;
            std::snprintf(sc.name, sizeof(sc.name), "Stream %zu", i);
            stream_rms_db[i] = -60.f;
        }
    }

    // ---- Preset operations ----

    void applyPreset(int slot) {
        if (slot < 0 || slot >= static_cast<int>(NUM_PRESETS)) return;
        if (!presets[slot].valid) return;
        ofdm               = presets[slot].ofdm;
        frame              = presets[slot].frame;
        modem              = presets[slot].modem;
        active_preset_slot = slot;
        preset_modified    = false;
        spectrum.initFreqs(static_cast<float>(ofdm.sample_rate));
    }

    void saveToPreset(int slot) {
        if (slot < 0 || slot >= static_cast<int>(NUM_PRESETS)) return;
        presets[slot].ofdm  = ofdm;
        presets[slot].frame = frame;
        presets[slot].modem = modem;
        presets[slot].valid = true;
        active_preset_slot  = slot;
        preset_modified     = false;
    }

    void copyPreset(int src, int dst) {
        if (src < 0 || src >= static_cast<int>(NUM_PRESETS)) return;
        if (dst < 0 || dst >= static_cast<int>(NUM_PRESETS)) return;
        presets[dst] = presets[src];
    }

    // ---- Alarm operations ----

    void updateAlarms() {
        AlarmStatus prev = alarm_status;

        alarm_status.snr_low  = stats.snr_db < alarm_thresh.snr_low_db
                                 && stats.sync_state >= SyncState::Locked;
        alarm_status.ber_high = stats.ber_estimate > alarm_thresh.ber_high
                                 && stats.frames_rx > 10;
        alarm_status.evm_high = stats.evm_percent > alarm_thresh.evm_high_pct;
        alarm_status.sync_lost = alarm_thresh.alarm_sync_loss
                                  && stats.sync_state == SyncState::Lost;
        alarm_status.level_low  = rx_meter.rms_db < alarm_thresh.level_low_db;
        // level_high now compares against the configured "Level high (dBFS)"
        // threshold (previously it ignored level_high_db entirely and was
        // driven by the audio-clip flag, so the spinbox was dead). The
        // audio-clip alarm is its own status, gated by its checkbox.
        alarm_status.level_high = rx_meter.rms_db > alarm_thresh.level_high_db;
        alarm_status.audio_clipped = alarm_thresh.alarm_audio_clip
                                      && rx_meter.clipping;
        // AGC pumping: gain ripple over the rolling window exceeds the
        // configured threshold. Indicates the AGC is chasing peaks
        // (release too short or input dynamics too fast) which makes
        // decoded audio sound "breathing" or unstable.
        alarm_status.agc_pumping = alarm_thresh.alarm_agc_pump
                                    && stats.agc_ripple_db >
                                        alarm_thresh.agc_ripple_db
                                    && stats.sync_state >= SyncState::Acquiring;

        // Log transitions
        auto logEvent = [&](bool cur, bool prev_val, AlarmEvent::Type t, float val) {
            if (cur != prev_val) {
                if (alarm_log.size() >= ALARM_LOG_SIZE)
                    alarm_log.erase(alarm_log.begin());
                alarm_log.push_back({t, val,
                    AlarmEvent::Clock::now(), cur});
                ++alarm_log_revision;
            }
        };
        logEvent(alarm_status.snr_low,      prev.snr_low,
                 AlarmEvent::Type::SNR_LOW,  stats.snr_db);
        logEvent(alarm_status.ber_high,     prev.ber_high,
                 AlarmEvent::Type::BER_HIGH, stats.ber_estimate);
        logEvent(alarm_status.evm_high,     prev.evm_high,
                 AlarmEvent::Type::EVM_HIGH, stats.evm_percent);
        logEvent(alarm_status.sync_lost,    prev.sync_lost,
                 AlarmEvent::Type::SYNC_LOST, 0.f);
        logEvent(alarm_status.audio_clipped,prev.audio_clipped,
                 AlarmEvent::Type::AUDIO_CLIP, rx_meter.rms_db);

        // Auto re-arm: Mute/Acknowledge silences the CURRENT alarm episode;
        // once every condition has cleared, new alarms must alert again.
        // (The mute used to be permanent — nothing ever cleared it, so one
        // Acknowledge disabled critical-alarm blinking for the whole
        // session, despite the tooltip promising re-arm.)
        if (alarm_status.muted && !alarm_status.anyActive())
            alarm_status.muted = false;
    }

    void acknowledgeAlarms() {
        alarm_status.muted = true;
    }
    void clearAlarmMute() {
        alarm_status.muted = false;
    }
    void clearAlarmLog() {
        alarm_log.clear();
    }

    // ---- Derived parameters ----

    ComputedParams computedParams() const {
        // Mirror modem.signal_bw into OFDMParams.target_bw_hz so the
        // auto-guard logic constrains the active subcarrier region to
        // fit inside the configured signal bandwidth. This makes the
        // displayed "OFDM-occupied BW" track the user's BW slider —
        // and matches what the engine actually does when configuring
        // OFDMModulator (see audio_engine.cpp syncConfig).
        OFDMParams o = ofdm;
        o.target_bw_hz = modem.signal_bw;
        return ComputedParams::compute(o, frame, modem.enable_rs_outer);
    }

    // ---- DSP thread interface (mutex-protected) ----

    void setStats(const ModemStats& s) {
        std::lock_guard<std::mutex> lock(mtx);
        stats = s;
    }

    ModemStats getStats() const {
        std::lock_guard<std::mutex> lock(mtx);
        return stats;
    }

    void pushConstellationSamples(const ComplexBuf& syms) {
        std::lock_guard<std::mutex> lock(mtx);
        constellation.push(syms);
    }
};

} // namespace dsca
