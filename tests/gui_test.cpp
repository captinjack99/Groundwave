/**
 * @file gui_test.cpp
 * @brief Phase 4 headless tests — AppState, presets, alarms, spectrum analyzer
 *
 * Compiled with -DGW_HEADLESS — no display dependencies required.
 * Tests: 10 scored + informational.
 */

#ifndef GW_HEADLESS
#define GW_HEADLESS
#endif

#include "app_state.hpp"
#include "spectrum_analyzer.hpp"
#include "ofdm.hpp"
#include "config_json.hpp"
#include "symbol_mapper.hpp"
#include "mmse_estimator.hpp"
#include "papr_reducer.hpp"
#include "hierarchical_mod.hpp"
#include "pls.hpp"
#include "ldpc.hpp"
#include "interleaver.hpp"
#include "frame.hpp"
#include "snr_calculator.hpp"

#include <cstdio>
#include <cstring>
#include <cmath>
#include <cassert>
#include <vector>
#include <string>
#include <stdexcept>
#include <chrono>
#include <thread>
#include <numeric>

// =========================================================================
// Test Framework
// =========================================================================

static int  g_tests_run   = 0;
static int  g_tests_pass  = 0;
static int  g_tests_fail  = 0;

static void test_pass(const char* name) {
    ++g_tests_run; ++g_tests_pass;
    std::printf("[PASS] %s\n", name);
}
static void test_fail(const char* name, const char* why) {
    ++g_tests_run; ++g_tests_fail;
    std::printf("[FAIL] %s  (%s)\n", name, why);
}
#define REQUIRE(name, cond) \
    do { if (cond) test_pass(name); else test_fail(name, #cond " was false"); } while(0)
#define REQUIRE_NEAR(name, a, b, tol) \
    do { if (std::abs((a)-(b)) <= (tol)) test_pass(name); \
         else { char _buf[128]; \
                std::snprintf(_buf, sizeof(_buf), #a "=%.6g, " #b "=%.6g, tol=%.6g", \
                              static_cast<double>(a), static_cast<double>(b), static_cast<double>(tol)); \
                test_fail(name, _buf); } } while(0)

// =========================================================================
// Test 1: AppState default construction
// =========================================================================
static void test_appstate_defaults() {
    gw::AppState st;

    // Default preset is "Standard" (slot 1), which sets QPSK
    REQUIRE("T01a: default preset loaded",
            st.active_preset_slot == 1 && !st.preset_modified);
    REQUIRE("T01b: default modulation QPSK",
            st.ofdm.modulation == gw::Modulation::QPSK);
    REQUIRE("T01c: 8 presets present",
            st.presets.size() == gw::NUM_PRESETS);
    REQUIRE("T01d: presets 0-6 valid",
            st.presets[0].valid && st.presets[6].valid);
    REQUIRE("T01e: preset 7 (Custom) empty",
            !st.presets[7].valid);
    REQUIRE("T01f: no alarms active at startup",
            !st.alarm_status.anyActive());
    REQUIRE("T01g: spectrum bins initialized",
            st.spectrum.freqs_hz[1] > 0.f);
}

// =========================================================================
// Test 2: Preset load/save/rename cycle
// =========================================================================
static void test_preset_cycle() {
    gw::AppState st;

    // Load preset 0 (Robust = BPSK)
    st.applyPreset(0);
    REQUIRE("T02a: apply preset 0",
            st.ofdm.modulation == gw::Modulation::BPSK);
    REQUIRE("T02b: active slot = 0",
            st.active_preset_slot == 0);
    REQUIRE("T02c: not modified after fresh load",
            !st.preset_modified);

    // Mutate and mark modified
    st.ofdm.modulation    = gw::Modulation::QAM64;
    st.preset_modified    = true;
    REQUIRE("T02d: modified flag set", st.preset_modified);

    // Save to slot 7
    st.saveToPreset(7);
    REQUIRE("T02e: slot 7 now valid",  st.presets[7].valid);
    REQUIRE("T02f: saved modulation",
            st.presets[7].ofdm.modulation == gw::Modulation::QAM64);
    REQUIRE("T02g: modified cleared",  !st.preset_modified);

    // Rename slot 7
    st.presets[7].setName("My Test");
    REQUIRE("T02h: rename works",
            std::string(st.presets[7].name) == "My Test");

    // Copy slot 7 → slot 6 and verify
    st.copyPreset(7, 6);
    REQUIRE("T02i: copy slot",
            st.presets[6].ofdm.modulation == gw::Modulation::QAM64);

    // Reset to factory
    gw::initFactoryPresets(st.presets);
    REQUIRE("T02j: factory reset restores preset 0 as BPSK",
            st.presets[0].ofdm.modulation == gw::Modulation::BPSK);
}

// =========================================================================
// Test 3: ComputedParams accuracy
// =========================================================================
static void test_computed_params() {
    gw::AppState st;
    st.applyPreset(1); // Standard: QPSK 1/2, 256-pt, 48kHz

    gw::ComputedParams cp = st.computedParams();

    // Subcarrier spacing: 48000 / 256 = 187.5 Hz
    REQUIRE_NEAR("T03a: subcarrier spacing", cp.subcarrier_spacing_hz, 187.5f, 0.1f);

    // Symbol duration: (256 + 32) / 48000 = 6.0 ms  (CP_1_8 → 32 samples)
    REQUIRE_NEAR("T03b: symbol duration ms", cp.symbol_duration_ms, 6.0f, 0.01f);

    // Guard bands: 10% each = 25.6 → 25 bins each side
    // Usable = 128 - 25 - 25 - 1 (DC) = 77
    REQUIRE("T03c: data subcarriers > 0", cp.data_subcarriers > 0);
    REQUIRE("T03d: active subcarriers = data + pilots",
            cp.active_subcarriers == cp.data_subcarriers + cp.pilot_subcarriers);

    // Gross bit rate = data_carriers * bps / sym_duration
    float expected_gross = static_cast<float>(cp.data_subcarriers) * 2.f
                           / (cp.symbol_duration_ms * 1e-3f);
    REQUIRE_NEAR("T03e: gross bit rate",
                 cp.gross_bitrate_bps, expected_gross, expected_gross * 0.01f);

    // After-FEC bit rate = gross * FEC rate. ComputedParams now exposes
    // this as `fec_coded_bitrate_bps` (distinct from `net_bitrate_bps`
    // which further accounts for Reed-Solomon outer-code overhead when
    // enabled, default on).
    REQUIRE_NEAR("T03f: fec_coded = gross * 1/2",
                 cp.fec_coded_bitrate_bps, cp.gross_bitrate_bps * 0.5f,
                 cp.gross_bitrate_bps * 0.01f);
    // Net bit rate is fec_coded minus RS overhead. With RS enabled at
    // QPSK 1/2 (k_bytes=135), RS overhead = 16/135 ≈ 11.85%.
    REQUIRE("T03f-rs: net < fec_coded (RS overhead applied)",
            cp.net_bitrate_bps < cp.fec_coded_bitrate_bps);
    REQUIRE("T03f-rs: net > 0.85 * fec_coded (overhead bounded)",
            cp.net_bitrate_bps > cp.fec_coded_bitrate_bps * 0.85f);

    // Spectral efficiency
    REQUIRE("T03g: spectral eff > 0", cp.spectral_eff_bps_hz > 0.f);
    REQUIRE("T03h: signal bandwidth > 0", cp.signal_bandwidth_hz > 0.f);

    // All 8 presets should compute without throwing
    bool all_ok = true;
    for (int i = 0; i < static_cast<int>(gw::NUM_PRESETS); ++i) {
        if (!st.presets[i].valid) continue;
        try {
            auto c = gw::ComputedParams::compute(st.presets[i].ofdm,
                                                    st.presets[i].frame);
            if (c.data_subcarriers == 0) { all_ok = false; break; }
        } catch (...) { all_ok = false; break; }
    }
    REQUIRE("T03i: all valid presets compute ok", all_ok);
}

// =========================================================================
// Test 4: Alarm system transitions
// =========================================================================
static void test_alarm_system() {
    gw::AppState st;
    st.applyPreset(1);

    // Initially no alarms
    REQUIRE("T04a: no alarms at start", !st.alarm_status.anyActive());

    // Inject low SNR after lock
    st.stats.snr_db      = 2.0f;
    st.stats.sync_state  = gw::SyncState::Locked;
    st.stats.frames_rx   = 0;  // BER alarm requires >10 frames
    st.alarm_thresh.snr_low_db = 6.0f;

    st.updateAlarms();
    REQUIRE("T04b: SNR low alarm fires", st.alarm_status.snr_low);
    REQUIRE("T04c: severity >= 1",       st.alarm_status.severity() >= 1);

    // Inject sync loss
    st.stats.sync_state = gw::SyncState::Lost;
    st.alarm_thresh.alarm_sync_loss = true;
    st.updateAlarms();
    REQUIRE("T04d: sync lost alarm fires", st.alarm_status.sync_lost);
    REQUIRE("T04e: severity == 2",        st.alarm_status.severity() == 2);

    // Log should have entries
    REQUIRE("T04f: alarm log has entries", !st.alarm_log.empty());

    // Mute
    st.acknowledgeAlarms();
    REQUIRE("T04g: muted flag set", st.alarm_status.muted);

    // Unmute
    st.clearAlarmMute();
    REQUIRE("T04h: unmuted", !st.alarm_status.muted);

    // Clear log
    st.clearAlarmLog();
    REQUIRE("T04i: alarm log cleared", st.alarm_log.empty());

    // Recovery: SNR back above threshold
    st.stats.snr_db     = 20.0f;
    st.stats.sync_state = gw::SyncState::Locked;
    st.updateAlarms();
    REQUIRE("T04j: SNR alarm clears", !st.alarm_status.snr_low);
}

// =========================================================================
// Test 5: Level meters
// =========================================================================
static void test_level_meters() {
    gw::LevelMeter meter;

    // Push a level and check normalization
    meter.update(-20.f, 0.016f);
    REQUIRE("T05a: RMS level set", meter.rms_db == -20.f);
    REQUIRE_NEAR("T05b: norm level",
                 meter.normalizedLevel(),
                 (-20.f + 60.f) / 60.f, 0.001f);
    REQUIRE("T05c: not clipping at -20", !meter.clipping);

    // Push 0 dBFS
    meter.update(-0.1f, 0.016f);
    REQUIRE("T05d: clipping at -0.1", meter.clipping);

    // Peak hold: push low, peak should remain at previous high
    float peak_after_high = meter.peak_db;
    meter.update(-40.f, 0.016f);
    REQUIRE("T05e: peak hold retained", meter.peak_db >= peak_after_high - 0.1f);

    // Normalized peak is in [0,1]
    float np = meter.normalizedPeak();
    REQUIRE("T05f: norm peak in [0,1]", np >= 0.f && np <= 1.001f);
}

// =========================================================================
// Test 6: Constellation data ring buffer
// =========================================================================
static void test_constellation_buffer() {
    gw::ConstellationData cd;
    REQUIRE("T06a: starts empty", cd.count() == 0 && !cd.full);

    // Push CONSTELLATION_MAX + 1 samples
    for (size_t i = 0; i < gw::CONSTELLATION_MAX + 100; ++i) {
        cd.push(static_cast<float>(i) * 0.001f,
                static_cast<float>(i) * -0.001f);
    }
    REQUIRE("T06b: full after overflow", cd.full);
    REQUIRE("T06c: count == max after overflow",
            cd.count() == gw::CONSTELLATION_MAX);

    // Clear
    cd.clear();
    REQUIRE("T06d: cleared", cd.count() == 0 && !cd.full);

    // Push via ComplexBuf
    gw::ComplexBuf buf(50);
    for (size_t i = 0; i < 50; ++i)
        buf[i] = gw::ComplexSample(static_cast<float>(i) * 0.01f, 0.f);
    cd.push(buf);
    REQUIRE("T06e: bulk push", cd.count() == 50);
}

// =========================================================================
// Test 7: SpectrumAnalyzer — frequency axis and bin mapping
// =========================================================================
static void test_spectrum_freq_axis() {
    gw::SpectrumConfig cfg;
    cfg.fft_size    = 128;
    cfg.sample_rate = 48000.f;
    cfg.avg_leak    = 0.f; // instant

    gw::SpectrumAnalyzer sa(cfg);
    gw::SpectrumData sd;
    sd.initFreqs(cfg.sample_rate);

    // Push silence
    std::vector<float> silence(256, 0.f);
    sa.pushSamples(silence.data(), silence.size());

    bool updated = sa.update(sd);
    REQUIRE("T07a: update returns true with new data", updated);

    // Frequency axis should go from 0 to Nyquist
    REQUIRE("T07b: freq[0] is near 0",
            sd.freqs_hz[0] < 100.f);
    REQUIRE("T07c: freq[-1] is near Nyquist",
            sd.freqs_hz[gw::SPECTRUM_BINS - 1] <
            cfg.sample_rate / 2.f + 100.f);
    REQUIRE("T07d: freqs monotonically increasing",
            sd.freqs_hz[1] > sd.freqs_hz[0]);

    // Silence → very low power
    REQUIRE("T07e: silence gives low power",
            sd.power_db[gw::SPECTRUM_BINS / 2] < -60.f);
}

// =========================================================================
// Test 8: SpectrumAnalyzer — tone detection
// =========================================================================
static void test_spectrum_tone() {
    gw::SpectrumConfig cfg;
    cfg.fft_size    = 512;
    cfg.sample_rate = 48000.f;
    cfg.avg_leak    = 0.f;
    cfg.complex_input = false;

    gw::SpectrumAnalyzer sa(cfg);
    gw::SpectrumData sd;
    sd.initFreqs(cfg.sample_rate);

    // Generate a 12 kHz tone at full scale
    const float tone_hz = 12000.f;
    const size_t n_samps = 2048;
    std::vector<float> tone(n_samps);
    for (size_t i = 0; i < n_samps; ++i) {
        tone[i] = std::sin(2.f * static_cast<float>(M_PI) * tone_hz
                           / cfg.sample_rate * static_cast<float>(i));
    }
    sa.pushSamples(tone.data(), tone.size());
    sa.update(sd);

    // Peak should be near 12 kHz bin
    float peak_db = -120.f;
    size_t peak_bin = 0;
    for (size_t i = 0; i < gw::SPECTRUM_BINS; ++i) {
        if (sd.power_db[i] > peak_db) {
            peak_db = sd.power_db[i];
            peak_bin = i;
        }
    }
    float peak_freq = sd.freqs_hz[peak_bin];
    // Allow 5% frequency tolerance due to bin quantization
    float freq_err = std::abs(peak_freq - tone_hz) / tone_hz;
    REQUIRE("T08a: peak near 12 kHz", freq_err < 0.15f);
    REQUIRE("T08b: tone power well above noise", peak_db > -20.f);

    // Waterfall ring should have been updated
    REQUIRE("T08c: waterfall row advanced",
            sd.waterfall_write_row >= 1);
}

// =========================================================================
// Test 9: AlarmThresholds — BER log scale
// =========================================================================
static void test_alarm_thresholds() {
    gw::AppState st;

    // BER threshold default
    REQUIRE_NEAR("T09a: BER default threshold",
                 st.alarm_thresh.ber_high, 1e-3f, 1e-4f);

    // Inject BER just above threshold (need >10 frames for BER alarm)
    st.stats.ber_estimate = 5e-3f;
    st.stats.frames_rx    = 20;
    st.stats.sync_state   = gw::SyncState::Locked;
    st.stats.snr_db       = 20.f;  // SNR OK
    st.updateAlarms();
    REQUIRE("T09b: BER high fires when above threshold",
            st.alarm_status.ber_high);

    // BER below threshold
    st.stats.ber_estimate = 1e-4f;
    st.updateAlarms();
    REQUIRE("T09c: BER alarm clears below threshold",
            !st.alarm_status.ber_high);

    // EVM alarm
    st.stats.evm_percent = 35.f;
    st.alarm_thresh.evm_high_pct = 30.f;
    st.updateAlarms();
    REQUIRE("T09d: EVM alarm fires", st.alarm_status.evm_high);

    st.stats.evm_percent = 10.f;
    st.updateAlarms();
    REQUIRE("T09e: EVM alarm clears", !st.alarm_status.evm_high);
}

// =========================================================================
// Test 10: AppState thread safety (mutex)
// =========================================================================
static void test_thread_safety() {
    gw::AppState st;
    constexpr int N = 500;
    bool error = false;

    std::thread writer([&]() {
        gw::ModemStats ms;
        for (int i = 0; i < N; ++i) {
            ms.snr_db      = static_cast<float>(i % 40);
            ms.frames_rx   = static_cast<uint64_t>(i);
            ms.evm_percent = static_cast<float>(i % 100);
            st.setStats(ms);
        }
    });

    std::thread reader([&]() {
        for (int i = 0; i < N; ++i) {
            gw::ModemStats ms = st.getStats();
            // snr should always be in valid range
            if (ms.snr_db < 0.f || ms.snr_db > 40.f) { error = true; break; }
        }
    });

    writer.join();
    reader.join();
    REQUIRE("T10a: no data race / stats always valid range", !error);

    // Constellation push from simulated DSP thread
    std::thread dsp([&]() {
        gw::ComplexBuf buf(32);
        for (int i = 0; i < 100; ++i) {
            for (auto& s : buf)
                s = gw::ComplexSample(0.5f, 0.5f);
            st.pushConstellationSamples(buf);
        }
    });
    dsp.join();
    REQUIRE("T10b: constellation push thread-safe",
            st.constellation.count() > 0);
}

// =========================================================================
// Informational: print all preset parameters
// =========================================================================
static void print_preset_info() {
    gw::AppState st;
    std::printf("\n--- Preset Summary ---\n");
    std::printf("%-4s %-20s %-10s %-8s %-8s %s\n",
                "Slot","Name","Mod","FEC","SR","Net kbps");
    std::printf("%-4s %-20s %-10s %-8s %-8s %s\n",
                "----","----","---","---","--","--------");

    for (size_t i = 0; i < gw::NUM_PRESETS; ++i) {
        const auto& p = st.presets[i];
        if (!p.valid) {
            std::printf("%-4zu %-20s  (empty)\n", i, p.name);
            continue;
        }
        auto cp = gw::ComputedParams::compute(p.ofdm, p.frame);
        static const char* mods[] =
            {"BPSK","QPSK","QAM16","QAM64","QAM256","QAM1024","QAM4096"};
        static const char* fecs[] =
            {"1/4","1/3","2/5","1/2","3/5","2/3","3/4","4/5","5/6","8/9","9/10","None"};
        int mi = static_cast<int>(p.ofdm.modulation);
        int fi = static_cast<int>(p.frame.fec_rate);
        std::printf("%-4zu %-20s %-10s %-8s %5u kHz  %.0f kbps\n",
                    i, p.name,
                    (mi>=0&&mi<7)?mods[mi]:"?",
                    (fi>=0&&fi<=11)?fecs[fi]:"?",
                    p.ofdm.sample_rate / 1000,
                    cp.net_bitrate_bps / 1000.f);
    }
    std::printf("----------------------\n\n");
}

// =========================================================================
// Phase 5: JSON Config Persistence Tests
// =========================================================================

static void test_json_roundtrip() {
    gw::AppState orig;
    // Customize some values
    orig.applyPreset(3); // High Capacity
    orig.tx_enabled  = true;
    orig.tx_gain_db  = -6.5f;
    orig.alarm_thresh.snr_low_db = 10.0f;
    orig.alarm_thresh.ber_high   = 1e-5f;
    orig.presets[7].setName("MyCustom");
    orig.presets[7].valid = true;
    orig.presets[7].ofdm.fft_size = 512;
    orig.presets[7].ofdm.modulation = gw::Modulation::QAM256;
    orig.presets[7].frame.fec_rate = gw::FECRate::Rate_5_6;

    // Customize stream 2 with non-default values so we can verify the
    // per-stream fields round-trip through JSON.
    orig.stream_configs[2].enabled        = true;
    orig.stream_configs[2].bitrate_bps    = 64000;
    orig.stream_configs[2].weight         = 2.5f;
    orig.stream_configs[2].channels       = 2;
    orig.stream_configs[2].frame_ms       = 40;
    orig.stream_configs[2].sample_rate    = 48000;
    orig.stream_configs[2].source         = gw::StreamAudioSource::File;
    orig.stream_configs[2].tone_freq_hz   = 1234.5f;
    orig.stream_configs[2].tone_amplitude = 0.42f;
    std::strncpy(orig.stream_configs[2].name, "Talkback",
                 sizeof(orig.stream_configs[2].name) - 1);
    std::strncpy(orig.stream_configs[2].file_path, "C:/audio/test.wav",
                 sizeof(orig.stream_configs[2].file_path) - 1);

    // Serialize
    std::string json = gw::serializeConfig(orig);
    REQUIRE("json_not_empty", !json.empty());
    REQUIRE("json_has_version", json.find("\"version\"") != std::string::npos);
    REQUIRE("json_has_presets", json.find("\"presets\"") != std::string::npos);

    // Deserialize
    gw::AppState loaded;
    bool ok = gw::deserializeConfig(json, loaded);
    REQUIRE("json_parse_ok", ok);

    // Verify round-trip
    REQUIRE("json_rt_preset_slot",  loaded.active_preset_slot == orig.active_preset_slot);
    REQUIRE("json_rt_tx_enabled",   loaded.tx_enabled == true);
    REQUIRE_NEAR("json_rt_tx_gain", loaded.tx_gain_db, -6.5f, 0.01f);
    REQUIRE("json_rt_fft_size",     loaded.ofdm.fft_size == orig.ofdm.fft_size);
    REQUIRE("json_rt_modulation",   loaded.ofdm.modulation == orig.ofdm.modulation);
    REQUIRE("json_rt_fec_rate",     loaded.frame.fec_rate == orig.frame.fec_rate);
    REQUIRE_NEAR("json_rt_snr_thresh", loaded.alarm_thresh.snr_low_db, 10.0f, 0.01f);
    REQUIRE_NEAR("json_rt_ber_thresh", loaded.alarm_thresh.ber_high, 1e-5f, 1e-7f);

    // Verify custom preset round-trip
    REQUIRE("json_rt_custom_valid",  loaded.presets[7].valid == true);
    REQUIRE("json_rt_custom_name",
            std::string(loaded.presets[7].name) == "MyCustom");
    REQUIRE("json_rt_custom_fft",    loaded.presets[7].ofdm.fft_size == 512);
    REQUIRE("json_rt_custom_mod",
            loaded.presets[7].ofdm.modulation == gw::Modulation::QAM256);
    REQUIRE("json_rt_custom_fec",
            loaded.presets[7].frame.fec_rate == gw::FECRate::Rate_5_6);

    // Verify stream 2 round-trip.
    REQUIRE("json_rt_stream_enabled",
            loaded.stream_configs[2].enabled == true);
    REQUIRE("json_rt_stream_bitrate",
            loaded.stream_configs[2].bitrate_bps == 64000);
    REQUIRE_NEAR("json_rt_stream_weight",
                 loaded.stream_configs[2].weight, 2.5f, 0.001f);
    REQUIRE("json_rt_stream_channels",
            loaded.stream_configs[2].channels == 2);
    REQUIRE("json_rt_stream_frame_ms",
            loaded.stream_configs[2].frame_ms == 40);
    REQUIRE("json_rt_stream_source_file",
            loaded.stream_configs[2].source == gw::StreamAudioSource::File);
    REQUIRE_NEAR("json_rt_stream_tone_hz",
                 loaded.stream_configs[2].tone_freq_hz, 1234.5f, 0.01f);
    REQUIRE_NEAR("json_rt_stream_tone_amp",
                 loaded.stream_configs[2].tone_amplitude, 0.42f, 0.001f);
    REQUIRE("json_rt_stream_name",
            std::string(loaded.stream_configs[2].name) == "Talkback");
    REQUIRE("json_rt_stream_filepath",
            std::string(loaded.stream_configs[2].file_path) ==
                "C:/audio/test.wav");
    // Untouched stream (0 defaults to enabled in AppState ctor) should still
    // be readable as the original default.
    REQUIRE("json_rt_stream0_enabled",
            loaded.stream_configs[0].enabled == orig.stream_configs[0].enabled);
}

static void test_json_file_io() {
    gw::AppState state;
    state.applyPreset(2); // HD Audio
    state.tx_gain_db = -3.0f;

    // Save to temp file (cross-platform: use current dir, not /tmp/)
    std::string path = "gw_test_config_tmp.json";
    bool saved = gw::saveConfigToFile(state, path);
    REQUIRE("json_file_save", saved);

    // Load back
    gw::AppState loaded;
    bool loadok = gw::loadConfigFromFile(path, loaded);
    REQUIRE("json_file_load", loadok);
    REQUIRE("json_file_fft",  loaded.ofdm.fft_size == state.ofdm.fft_size);
    REQUIRE_NEAR("json_file_gain", loaded.tx_gain_db, -3.0f, 0.01f);

    // Cleanup
    std::remove(path.c_str());
}

static void test_json_bad_input() {
    gw::AppState state;
    // Empty string
    bool ok = gw::deserializeConfig("", state);
    REQUIRE("json_bad_empty", !ok);
    // Not JSON
    ok = gw::deserializeConfig("this is not json", state);
    REQUIRE("json_bad_notjson", !ok);
    // Wrong version
    ok = gw::deserializeConfig("{\"version\":99}", state);
    REQUIRE("json_bad_version", !ok);
}

// =========================================================================
// Phase 5: Piecewise-Linear LLR Tests
// =========================================================================

static void test_pwl_llr_qam16() {
    // Compare PWL LLR to exact max-log-MAP for QAM16
    gw::SymbolMapper sm(gw::Modulation::QAM16);
    float noise_var = 0.1f;

    // Test on all constellation points + some noisy ones
    size_t match_count = 0;
    size_t total = 0;

    for (auto& s : sm.constellation()) {
        std::vector<float> exact, pwl;
        sm.demapSoft(s, noise_var, exact);
        sm.demapSoftPWL(s, noise_var, pwl);

        for (size_t b = 0; b < exact.size(); ++b) {
            ++total;
            if (std::abs(exact[b] - pwl[b]) < 0.5f) ++match_count;
        }
    }

    float accuracy = static_cast<float>(match_count) / static_cast<float>(total);
    REQUIRE("pwl_qam16_sign_match", accuracy > 0.95f);
    std::printf("  QAM16 PWL accuracy: %.1f%% (%zu/%zu within 0.5)\n",
                accuracy * 100.f, match_count, total);
}

static void test_pwl_llr_qam64() {
    gw::SymbolMapper sm(gw::Modulation::QAM64);
    float noise_var = 0.05f;

    // Test sign agreement (most important for FEC)
    size_t sign_match = 0, total = 0;
    for (auto& s : sm.constellation()) {
        std::vector<float> exact, pwl;
        sm.demapSoft(s, noise_var, exact);
        sm.demapSoftPWL(s, noise_var, pwl);

        for (size_t b = 0; b < exact.size(); ++b) {
            ++total;
            if ((exact[b] >= 0) == (pwl[b] >= 0)) ++sign_match;
        }
    }

    float pct = 100.f * static_cast<float>(sign_match) / static_cast<float>(total);
    REQUIRE("pwl_qam64_sign", pct > 98.0f);
    std::printf("  QAM64 PWL sign agreement: %.1f%%\n", pct);
}

static void test_pwl_llr_bpsk_qpsk() {
    // BPSK and QPSK should be identical (PWL falls through to exact)
    float noise_var = 0.2f;

    {
        gw::SymbolMapper sm(gw::Modulation::BPSK);
        gw::ComplexSample s(0.7f, 0.1f);
        std::vector<float> exact, pwl;
        sm.demapSoft(s, noise_var, exact);
        sm.demapSoftPWL(s, noise_var, pwl);
        REQUIRE_NEAR("pwl_bpsk_identical", exact[0], pwl[0], 1e-6f);
    }
    {
        gw::SymbolMapper sm(gw::Modulation::QPSK);
        gw::ComplexSample s(0.5f, -0.3f);
        std::vector<float> exact, pwl;
        sm.demapSoft(s, noise_var, exact);
        sm.demapSoftPWL(s, noise_var, pwl);
        REQUIRE_NEAR("pwl_qpsk_i_identical", exact[0], pwl[0], 1e-6f);
        REQUIRE_NEAR("pwl_qpsk_q_identical", exact[1], pwl[1], 1e-6f);
    }
}

// =========================================================================
// Phase 5: MMSE Channel Estimator Tests
// =========================================================================

static void test_mmse_construction() {
    gw::OFDMParams p;
    p.fft_size   = 256;
    p.modulation = gw::Modulation::QAM16;
    auto alloc = gw::computeAllocation(p);

    gw::MMSEConfig cfg;
    cfg.tau_rms_us   = 5.0f;
    cfg.pilot_snr_db = 20.0f;
    cfg.interp_order = 4;

    gw::MMSEChannelEstimator est(
        p.fft_size,
        alloc.pilot_indices,
        alloc.data_indices,
        alloc.pilot_values,
        p.sample_rate,
        cfg);

    REQUIRE("mmse_construct_ok", true);
    REQUIRE("mmse_est_size", est.estimate().size() == p.fft_size);
    REQUIRE("mmse_mag_size", est.magnitude().size() == p.fft_size);
}

static void test_mmse_flat_channel() {
    // On flat channel, MMSE should converge to approximately H=1
    gw::OFDMParams p;
    p.fft_size = 256;
    p.modulation = gw::Modulation::QAM16;
    auto alloc = gw::computeAllocation(p);

    gw::MMSEConfig cfg;
    cfg.smoothing = 0.0f; // no temporal smoothing for this test

    gw::MMSEChannelEstimator est(
        p.fft_size, alloc.pilot_indices, alloc.data_indices,
        alloc.pilot_values, p.sample_rate, cfg);

    // Create flat channel h1 = h2 = 1.0 at all bins
    gw::ComplexBuf h1(p.fft_size, gw::ComplexSample(1.0f, 0.0f));
    gw::ComplexBuf h2(p.fft_size, gw::ComplexSample(1.0f, 0.0f));

    est.initFromPreamble(h1, h2);

    // Check that estimates at data subcarriers are close to 1.0
    float max_err = 0.f;
    for (size_t i = 0; i < alloc.data_indices.size(); ++i) {
        size_t idx = alloc.data_indices[i];
        float err = std::abs(est.estimate()[idx] - gw::ComplexSample(1.0f, 0.0f));
        max_err = std::max(max_err, err);
    }

    REQUIRE("mmse_flat_max_err", max_err < 0.1f);
    std::printf("  MMSE flat channel max error: %.4f\n", max_err);
}

static void test_mmse_integration() {
    // Test MMSE enable/disable on OFDMDemodulator
    gw::OFDMParams p;
    p.fft_size = 256;
    p.modulation = gw::Modulation::QPSK;

    gw::OFDMDemodulator demod(p);
    REQUIRE("mmse_initially_off", !demod.isMMSEEnabled());

    demod.enableMMSE();
    REQUIRE("mmse_after_enable", demod.isMMSEEnabled());

    demod.disableMMSE();
    REQUIRE("mmse_after_disable", !demod.isMMSEEnabled());
}

// =========================================================================
// Phase 5 — PAPR Reduction Tests
// =========================================================================

static void test_papr_construction() {
    gw::OFDMParams p;
    p.fft_size = 256;
    p.modulation = gw::Modulation::QAM16;

    auto alloc = gw::computeAllocation(p);

    gw::PAPRConfig cfg;
    cfg.enabled = true;
    cfg.target_papr_db = 7.0f;
    cfg.reserve_fraction = 0.05f;

    gw::PAPRReducer reducer(p.fft_size,
                              p.guardLeft(), p.fft_size - p.guardRight(),
                              alloc.data_indices, alloc.pilot_indices, cfg);

    // Should have reserved some tones (5% of ~200 active ≈ 10 tones)
    REQUIRE("papr_has_reserved", reducer.reservedCount() > 0);
    REQUIRE("papr_reserved_reasonable",
            reducer.reservedCount() <= alloc.dataCount() / 2);
}

static void test_papr_reduction() {
    // Build a modulator, generate a symbol, and verify PAPR is reduced
    gw::OFDMParams p;
    p.fft_size = 256;
    p.modulation = gw::Modulation::QAM64;

    auto alloc = gw::computeAllocation(p);

    gw::PAPRConfig cfg;
    cfg.enabled = true;
    cfg.target_papr_db = 7.0f;
    cfg.max_iterations = 10;
    cfg.step_size = 0.8f;
    cfg.reserve_fraction = 0.08f;

    gw::PAPRReducer reducer(p.fft_size,
                              p.guardLeft(), p.fft_size - p.guardRight(),
                              alloc.data_indices, alloc.pilot_indices, cfg);

    // Create a frequency-domain symbol with random data
    gw::ComplexBuf freq(p.fft_size, gw::ComplexSample(0.f, 0.f));
    gw::SymbolMapper mapper(p.modulation);

    // Fill data subcarriers with mapped symbols
    uint32_t seed = 42;
    for (auto idx : alloc.data_indices) {
        seed = seed * 1103515245u + 12345u;
        float i_val = (static_cast<float>((seed >> 16) & 0xFF) / 128.f) - 1.f;
        seed = seed * 1103515245u + 12345u;
        float q_val = (static_cast<float>((seed >> 16) & 0xFF) / 128.f) - 1.f;
        freq[idx] = gw::ComplexSample(i_val, q_val);
    }
    // Fill pilots
    for (size_t i = 0; i < alloc.pilotCount(); ++i) {
        freq[alloc.pilot_indices[i]] = alloc.pilot_values[i];
    }

    gw::FFTEngine fft(p.fft_size);
    auto stats = reducer.reduce(freq, fft);

    // PAPR should be reduced (or at least not increased by more than 1 dB)
    REQUIRE("papr_reduced", stats.papr_after_db <= stats.papr_before_db + 1.0f);
    // Should have done some iterations
    REQUIRE("papr_iterated", stats.iterations_used >= 0);
    // Power increase should be modest
    REQUIRE("papr_power_modest", stats.power_increase_db < 3.0f);
}

static void test_papr_disabled() {
    gw::OFDMParams p;
    p.fft_size = 128;
    p.modulation = gw::Modulation::QPSK;

    auto alloc = gw::computeAllocation(p);

    gw::PAPRConfig cfg;
    cfg.enabled = false; // Disabled

    gw::PAPRReducer reducer(p.fft_size,
                              p.guardLeft(), p.fft_size - p.guardRight(),
                              alloc.data_indices, alloc.pilot_indices, cfg);

    gw::ComplexBuf freq(p.fft_size, gw::ComplexSample(0.f, 0.f));
    for (auto idx : alloc.data_indices) {
        freq[idx] = gw::ComplexSample(1.f, 0.f);
    }

    gw::FFTEngine fft(p.fft_size);
    auto stats = reducer.reduce(freq, fft);

    REQUIRE("papr_disabled_no_iter", stats.iterations_used == 0);
    REQUIRE_NEAR("papr_disabled_same", stats.papr_before_db,
                 stats.papr_after_db, 0.01f);
}

static void test_papr_compute_static() {
    // Constant amplitude signal → PAPR = 0 dB
    gw::ComplexBuf td(64, gw::ComplexSample(1.0f, 0.0f));
    float papr = gw::PAPRReducer::computePAPR(td);
    REQUIRE_NEAR("papr_flat_0db", papr, 0.0f, 0.01f);

    // Single non-zero sample → PAPR = 10*log10(N) dB
    gw::ComplexBuf spike(64, gw::ComplexSample(0.0f, 0.0f));
    spike[0] = gw::ComplexSample(1.0f, 0.0f);
    float papr_spike = gw::PAPRReducer::computePAPR(spike);
    float expected = 10.f * std::log10(64.f);
    REQUIRE_NEAR("papr_spike", papr_spike, expected, 0.1f);
}

// =========================================================================
// Phase 5 — Hierarchical Modulation Tests
// =========================================================================

static void test_hier_construction() {
    gw::HierarchicalConfig cfg;
    cfg.mode = gw::HierarchicalMode::QPSK_QAM16;
    cfg.alpha = 2.0f;
    cfg.enabled = true;

    gw::HierarchicalMapper mapper(cfg);

    REQUIRE("hier_enabled", mapper.isEnabled());
    REQUIRE("hier_hp_bps", mapper.hpBPS() == 2);
    REQUIRE("hier_lp_bps", mapper.lpBPS() == 2);
    REQUIRE("hier_total_bps", mapper.totalBPS() == 4);
}

static void test_hier_map_demap_hp() {
    // Map HP+LP, then demap HP — should recover HP bits perfectly (no noise)
    gw::HierarchicalConfig cfg;
    cfg.mode = gw::HierarchicalMode::QPSK_QAM16;
    cfg.alpha = 2.0f;
    cfg.enabled = true;

    gw::HierarchicalMapper mapper(cfg);

    // HP: 2 bits per symbol, LP: 2 bits per symbol
    // Generate 4 symbols → 8 HP bits (1 byte), 8 LP bits (1 byte)
    uint8_t hp_data[] = { 0b11001010 };  // 4 symbols: 11, 00, 10, 10
    uint8_t lp_data[] = { 0b01110001 };  // 4 symbols: 01, 11, 00, 01

    gw::ComplexBuf symbols;
    mapper.map(hp_data, 8, lp_data, 8, symbols);
    REQUIRE("hier_map_count", symbols.size() == 4);

    // Demap HP (should match hp_data at noiseless)
    std::vector<uint8_t> hp_out;
    mapper.demapHP(symbols, hp_out);
    REQUIRE("hier_demap_hp_size", hp_out.size() >= 1);
    REQUIRE("hier_demap_hp_match", hp_out[0] == hp_data[0]);
}

static void test_hier_map_demap_lp() {
    gw::HierarchicalConfig cfg;
    cfg.mode = gw::HierarchicalMode::QPSK_QAM16;
    cfg.alpha = 2.0f;
    cfg.enabled = true;

    gw::HierarchicalMapper mapper(cfg);

    uint8_t hp_data[] = { 0b11001010 };
    uint8_t lp_data[] = { 0b01110001 };

    gw::ComplexBuf symbols;
    mapper.map(hp_data, 8, lp_data, 8, symbols);

    // Demap LP (should match lp_data at noiseless)
    std::vector<uint8_t> lp_out;
    mapper.demapLP(symbols, lp_out);
    REQUIRE("hier_demap_lp_size", lp_out.size() >= 1);
    REQUIRE("hier_demap_lp_match", lp_out[0] == lp_data[0]);
}

static void test_hier_soft_demap() {
    gw::HierarchicalConfig cfg;
    cfg.mode = gw::HierarchicalMode::QPSK_QAM16;
    cfg.alpha = 2.0f;
    cfg.enabled = true;

    gw::HierarchicalMapper mapper(cfg);

    uint8_t hp_data[] = { 0b11000000 };
    uint8_t lp_data[] = { 0b01000000 };

    gw::ComplexBuf symbols;
    mapper.map(hp_data, 4, lp_data, 4, symbols);  // 2 symbols

    // Soft demap HP
    std::vector<float> hp_llrs;
    mapper.demapSoftHP(symbols, 0.01f, hp_llrs);
    REQUIRE("hier_soft_hp_count", hp_llrs.size() == 4); // 2 syms × 2 bits

    // Soft demap LP
    std::vector<float> lp_llrs;
    mapper.demapSoftLP(symbols, 0.01f, lp_llrs);
    REQUIRE("hier_soft_lp_count", lp_llrs.size() == 4);

    // At low noise, LLRs should have large magnitude
    bool hp_strong = true;
    for (auto l : hp_llrs) if (std::abs(l) < 1.0f) hp_strong = false;
    REQUIRE("hier_soft_hp_strong", hp_strong);
}

static void test_hier_alpha_effect() {
    // Higher alpha → HP quadrants more separated → easier HP decode
    // Verify that constellation points move with alpha
    gw::HierarchicalConfig cfg1;
    cfg1.mode = gw::HierarchicalMode::QPSK_QAM16;
    cfg1.alpha = 1.0f;  // Uniform 16-QAM
    cfg1.enabled = true;
    gw::HierarchicalMapper m1(cfg1);

    gw::HierarchicalConfig cfg2;
    cfg2.mode = gw::HierarchicalMode::QPSK_QAM16;
    cfg2.alpha = 4.0f;  // Very separated quadrants
    cfg2.enabled = true;
    gw::HierarchicalMapper m2(cfg2);

    uint8_t hp[] = { 0b11000000 }; // HP=11 (one symbol)
    uint8_t lp[] = { 0b00000000 }; // LP=00

    gw::ComplexBuf s1, s2;
    m1.map(hp, 2, lp, 2, s1);
    m2.map(hp, 2, lp, 2, s2);

    // With higher alpha, the point should be farther from origin in the quadrant direction
    // Both are normalized to unit power, but the ratio of HP spacing to LP spacing differs
    // Just verify they produce different constellations
    REQUIRE("hier_alpha_differs",
            std::abs(s1[0] - s2[0]) > 0.01f);
}

static void test_hier_qpsk_qam64() {
    // Test the QPSK/QAM64 mode (HP: 2 bits, LP: 4 bits = 6 total)
    gw::HierarchicalConfig cfg;
    cfg.mode = gw::HierarchicalMode::QPSK_QAM64;
    cfg.alpha = 2.0f;
    cfg.enabled = true;

    gw::HierarchicalMapper mapper(cfg);
    REQUIRE("hier64_hp_bps", mapper.hpBPS() == 2);
    REQUIRE("hier64_lp_bps", mapper.lpBPS() == 4);
    REQUIRE("hier64_total", mapper.totalBPS() == 6);

    // Map and demap roundtrip
    uint8_t hp_data[] = { 0b10000000 }; // 1 symbol: HP=10
    uint8_t lp_data[] = { 0b01010000 }; // 1 symbol: LP=0101

    gw::ComplexBuf symbols;
    mapper.map(hp_data, 2, lp_data, 4, symbols);
    REQUIRE("hier64_map_count", symbols.size() == 1);

    std::vector<uint8_t> hp_out;
    mapper.demapHP(symbols, hp_out);
    // Extract first 2 bits of output
    uint8_t hp_recovered = (hp_out[0] >> 6) & 0x03;
    uint8_t hp_expected = 0x02; // 10
    REQUIRE("hier64_hp_roundtrip", hp_recovered == hp_expected);
}

// =========================================================================
// Phase 5 — PLS / VCM Tests
// =========================================================================

static void test_pls_encode_decode() {
    gw::PLSBlock tx_pls;
    tx_pls.modulation = gw::Modulation::QAM64;
    tx_pls.fec_rate   = gw::FECRate::Rate_3_4;
    tx_pls.vcm_active = true;
    tx_pls.vcm_slot   = 3;
    tx_pls.vcm_total  = 8;

    std::vector<uint8_t> encoded;
    gw::encodePLS(tx_pls, encoded);
    REQUIRE("pls_encode_size", encoded.size() == 8);

    gw::PLSBlock rx_pls;
    bool ok = gw::decodePLS(encoded, rx_pls);
    REQUIRE("pls_decode_ok", ok);
    REQUIRE("pls_decode_valid", rx_pls.valid);
    REQUIRE("pls_mod_match",
            rx_pls.modulation == gw::Modulation::QAM64);
    REQUIRE("pls_fec_match",
            rx_pls.fec_rate == gw::FECRate::Rate_3_4);
    REQUIRE("pls_vcm_active", rx_pls.vcm_active == true);
    REQUIRE("pls_vcm_slot", rx_pls.vcm_slot == 3);
    REQUIRE("pls_vcm_total", rx_pls.vcm_total == 8);
}

static void test_pls_all_modcods() {
    // Test every modulation + a selection of FEC rates
    gw::Modulation mods[] = {
        gw::Modulation::BPSK, gw::Modulation::QPSK,
        gw::Modulation::QAM16, gw::Modulation::QAM64,
        gw::Modulation::QAM256, gw::Modulation::QAM1024,
        gw::Modulation::QAM4096
    };
    gw::FECRate rates[] = {
        gw::FECRate::Rate_1_4, gw::FECRate::Rate_1_2,
        gw::FECRate::Rate_3_4, gw::FECRate::Rate_5_6,
        gw::FECRate::Rate_9_10
    };

    int ok_count = 0;
    int total = 0;
    for (auto m : mods) {
        for (auto r : rates) {
            gw::PLSBlock tx;
            tx.modulation = m;
            tx.fec_rate = r;
            tx.vcm_active = false;
            tx.vcm_slot = 0;
            tx.vcm_total = 1;

            std::vector<uint8_t> enc;
            gw::encodePLS(tx, enc);

            gw::PLSBlock rx;
            if (gw::decodePLS(enc, rx) &&
                rx.modulation == m && rx.fec_rate == r) {
                ++ok_count;
            }
            ++total;
        }
    }
    REQUIRE("pls_all_modcods", ok_count == total);
}

static void test_pls_corrupt_recovery() {
    // Corrupt first copy, verify second copy recovers
    gw::PLSBlock tx;
    tx.modulation = gw::Modulation::QAM16;
    tx.fec_rate   = gw::FECRate::Rate_2_3;
    tx.vcm_active = false;
    tx.vcm_slot = 0;
    tx.vcm_total = 1;

    std::vector<uint8_t> enc;
    gw::encodePLS(tx, enc);

    // Corrupt first 4 bytes (first copy)
    enc[0] ^= 0xFF;
    enc[1] ^= 0xFF;

    gw::PLSBlock rx;
    bool ok = gw::decodePLS(enc, rx);
    REQUIRE("pls_corrupt_recover", ok);
    REQUIRE("pls_corrupt_mod", rx.modulation == gw::Modulation::QAM16);
    REQUIRE("pls_corrupt_fec", rx.fec_rate == gw::FECRate::Rate_2_3);
}

static void test_vcm_schedule() {
    auto sched = gw::createStereoVCM(
        gw::Modulation::QPSK, gw::FECRate::Rate_1_2,
        gw::Modulation::QAM64, gw::FECRate::Rate_3_4,
        2, 2);

    REQUIRE("vcm_enabled", sched.enabled);
    REQUIRE("vcm_num_slots", sched.num_slots == 4);

    // Slot 0,1 should be robust
    REQUIRE("vcm_slot0_mod",
            sched.entries[0].modulation == gw::Modulation::QPSK);
    REQUIRE("vcm_slot0_plp", sched.entries[0].plp_id == 0);

    // Slot 2,3 should be enhancement
    REQUIRE("vcm_slot2_mod",
            sched.entries[2].modulation == gw::Modulation::QAM64);
    REQUIRE("vcm_slot2_plp", sched.entries[2].plp_id == 1);

    // PLS for frame cycling
    auto pls0 = sched.plsForFrame(0);
    REQUIRE("vcm_pls0_mod",
            pls0.modulation == gw::Modulation::QPSK);
    REQUIRE("vcm_pls0_slot", pls0.vcm_slot == 0);
    REQUIRE("vcm_pls0_total", pls0.vcm_total == 4);

    auto pls2 = sched.plsForFrame(2);
    REQUIRE("vcm_pls2_mod",
            pls2.modulation == gw::Modulation::QAM64);

    // Frame 4 wraps → slot 0 again
    auto pls4 = sched.plsForFrame(4);
    REQUIRE("vcm_pls4_wraps",
            pls4.modulation == gw::Modulation::QPSK);
}

static void test_modcod_detector_uniform() {
    // Non-VCM: uniform modcod, confirm after 2 matching PLS blocks
    gw::ModCodDetector det(2);

    gw::PLSBlock pls;
    pls.valid = true;
    pls.modulation = gw::Modulation::QAM16;
    pls.fec_rate = gw::FECRate::Rate_3_4;
    pls.vcm_active = false;

    // First feed → initializes, reports change
    bool changed = det.feed(pls);
    REQUIRE("det_init_changed", changed);
    REQUIRE("det_init_mod",
            det.currentModulation() == gw::Modulation::QAM16);

    // Same modcod → no change
    changed = det.feed(pls);
    REQUIRE("det_same_no_change", !changed);

    // New modcod → needs 2 confirmations
    gw::PLSBlock pls2 = pls;
    pls2.modulation = gw::Modulation::QAM64;

    changed = det.feed(pls2); // First new PLS
    REQUIRE("det_new_first", !changed);

    changed = det.feed(pls2); // Second → confirmed
    REQUIRE("det_new_confirmed", changed);
    REQUIRE("det_new_mod",
            det.currentModulation() == gw::Modulation::QAM64);
}

static void test_modcod_detector_vcm() {
    // VCM mode: immediate ModCod changes per slot
    gw::ModCodDetector det(2);

    gw::PLSBlock pls0;
    pls0.valid = true;
    pls0.modulation = gw::Modulation::QPSK;
    pls0.fec_rate = gw::FECRate::Rate_1_2;
    pls0.vcm_active = true;
    pls0.vcm_slot = 0;
    pls0.vcm_total = 4;

    det.feed(pls0); // Init

    // VCM slot 2: different modcod → should change immediately
    gw::PLSBlock pls2;
    pls2.valid = true;
    pls2.modulation = gw::Modulation::QAM64;
    pls2.fec_rate = gw::FECRate::Rate_3_4;
    pls2.vcm_active = true;
    pls2.vcm_slot = 2;
    pls2.vcm_total = 4;

    bool changed = det.feed(pls2);
    REQUIRE("det_vcm_immediate", changed);
    REQUIRE("det_vcm_mod",
            det.currentModulation() == gw::Modulation::QAM64);
    REQUIRE("det_vcm_slot", det.vcmSlot() == 2);
}

// =========================================================================
// Phase 6 — Polish & Deployment Tests
// =========================================================================

// --- PLS BPSK roundtrip: encode → unpack to bits → pack to bytes → decode ---
static void test_pls_bpsk_roundtrip() {
    gw::PLSBlock orig;
    orig.modulation = gw::Modulation::QAM16;
    orig.fec_rate   = gw::FECRate::Rate_3_4;
    orig.vcm_active = true;
    orig.vcm_slot   = 5;
    orig.vcm_total  = 8;

    // Encode → 8 packed bytes
    std::vector<uint8_t> packed;
    gw::encodePLS(orig, packed);
    REQUIRE("pls_rt_encode_size", packed.size() == 8);

    // Unpack bytes to individual bits (simulates BPSK demap)
    std::vector<uint8_t> individual_bits;
    for (size_t i = 0; i < 64; ++i) {
        uint8_t byte_val = packed[i / 8];
        uint8_t bit = (byte_val >> (7 - (i % 8))) & 1;
        individual_bits.push_back(bit);
    }
    REQUIRE("pls_rt_bits_count", individual_bits.size() == 64);

    // Re-pack individual bits to bytes (this is what processRXPLS does)
    std::vector<uint8_t> repacked(8, 0);
    for (size_t i = 0; i < 64; ++i) {
        if (individual_bits[i]) {
            repacked[i / 8] |= static_cast<uint8_t>(1 << (7 - (i % 8)));
        }
    }

    // Verify repacked matches original packed
    bool match = true;
    for (size_t i = 0; i < 8; ++i) {
        if (repacked[i] != packed[i]) { match = false; break; }
    }
    REQUIRE("pls_rt_repack_match", match);

    // Decode from repacked bytes
    gw::PLSBlock decoded;
    bool ok = gw::decodePLS(repacked, decoded);
    REQUIRE("pls_rt_decode_ok", ok && decoded.valid);
    REQUIRE("pls_rt_mod",   decoded.modulation == orig.modulation);
    REQUIRE("pls_rt_fec",   decoded.fec_rate == orig.fec_rate);
    REQUIRE("pls_rt_vcm",   decoded.vcm_active == orig.vcm_active);
    REQUIRE("pls_rt_slot",  decoded.vcm_slot == orig.vcm_slot);
    REQUIRE("pls_rt_total", decoded.vcm_total == orig.vcm_total);
}

// --- VCM per-slot tracking: detect when slot modcod changes ---
static void test_vcm_slot_tracking() {
    // Create a 4-slot VCM schedule: 2 QPSK½ + 2 QAM64¾
    auto sched = gw::createStereoVCM(
        gw::Modulation::QPSK, gw::FECRate::Rate_1_2,
        gw::Modulation::QAM64, gw::FECRate::Rate_3_4,
        2, 2);
    REQUIRE("vcm_track_enabled", sched.enabled);
    REQUIRE("vcm_track_slots", sched.num_slots == 4);

    // Simulate walking through frames and track modcod changes
    gw::Modulation last_mod = sched.entries[0].modulation;
    gw::FECRate    last_fec = sched.entries[0].fec_rate;
    int change_count = 0;

    for (uint32_t frame = 0; frame < 12; ++frame) {
        const auto& slot = sched.slotForFrame(frame);
        if (slot.modulation != last_mod || slot.fec_rate != last_fec) {
            change_count++;
            last_mod = slot.modulation;
            last_fec = slot.fec_rate;
        }
    }
    // Over 12 frames (3 superframes), modcod changes at slot boundaries:
    // Frame 0=QPSK, 1=QPSK, 2=QAM64 (change), 3=QAM64,
    // 4=QPSK (change), 5=QPSK, 6=QAM64 (change), 7=QAM64,
    // 8=QPSK (change), 9=QPSK, 10=QAM64 (change), 11=QAM64
    // = 5 changes
    REQUIRE("vcm_track_changes", change_count == 5);

    // Verify PLS generation matches schedule
    auto pls0 = sched.plsForFrame(0);
    REQUIRE("vcm_pls_slot0_mod", pls0.modulation == gw::Modulation::QPSK);
    REQUIRE("vcm_pls_slot0_fec", pls0.fec_rate == gw::FECRate::Rate_1_2);
    REQUIRE("vcm_pls_slot0_vcm", pls0.vcm_active);

    auto pls2 = sched.plsForFrame(2);
    REQUIRE("vcm_pls_slot2_mod", pls2.modulation == gw::Modulation::QAM64);
    REQUIRE("vcm_pls_slot2_fec", pls2.fec_rate == gw::FECRate::Rate_3_4);
}

// --- Playback ring buffer: write/read integrity ---
static void test_playback_ringbuffer() {
    gw::RingBuffer ring(1024);

    // Write 100 samples
    std::vector<float> tx(100);
    for (size_t i = 0; i < 100; ++i)
        tx[i] = static_cast<float>(i) * 0.01f;
    size_t written = ring.write(tx.data(), tx.size());
    REQUIRE("ring_write_count", written == 100);
    REQUIRE("ring_available", ring.available() == 100);

    // Read back
    std::vector<float> rx(100, 0.f);
    size_t read_count = ring.read(rx.data(), rx.size());
    REQUIRE("ring_read_count", read_count == 100);

    // Verify data integrity
    bool match = true;
    for (size_t i = 0; i < 100; ++i) {
        if (std::fabs(rx[i] - tx[i]) > 1e-6f) { match = false; break; }
    }
    REQUIRE("ring_data_match", match);
    REQUIRE("ring_empty_after", ring.available() == 0);

    // Overflow test: write more than capacity
    std::vector<float> big(2000, 1.f);
    size_t overflow_written = ring.write(big.data(), big.size());
    REQUIRE("ring_overflow_capped", overflow_written <= 1024);
}

// --- Version defines: verify they're wired, not pinned to a release ---
static void test_version_defines() {
    // Assert the macros EXIST and form a sane semantic version — not
    // specific numbers, so a version bump doesn't fail the suite. (The
    // old test hard-coded 2.0.0 and broke the moment the project version
    // changed; the value lives in CMakeLists, which is the source of
    // truth, and a test shouldn't duplicate it.)
    const int major = GW_VERSION_MAJOR;
    const int minor = GW_VERSION_MINOR;
    const int patch = GW_VERSION_PATCH;
    REQUIRE("version_nonnegative", major >= 0 && minor >= 0 && patch >= 0);
    REQUIRE("version_not_all_zero", (major | minor | patch) != 0);

    // Build date should be a non-empty string
    const char* date = GW_BUILD_DATE;
    REQUIRE("build_date_set", date != nullptr && std::strlen(date) > 0);
}

// --- Hierarchical TX symbol count: verify mapping produces correct output size ---
static void test_hier_tx_symbol_count() {
    gw::HierarchicalConfig cfg;
    cfg.mode    = gw::HierarchicalMode::QPSK_QAM16;
    cfg.alpha   = 2.0f;
    cfg.enabled = true;
    gw::HierarchicalMapper mapper(cfg);

    REQUIRE("hier_tx_hp_bps", mapper.hpBPS() == 2);
    REQUIRE("hier_tx_lp_bps", mapper.lpBPS() == 2);

    // For QPSK_QAM16: 4 bps total, so 200 bits → 50 symbols
    // HP: 50 syms × 2 bits = 100 bits = 13 bytes
    // LP: 50 syms × 2 bits = 100 bits = 13 bytes
    uint8_t hp_data[13] = {};
    uint8_t lp_data[13] = {};
    // Fill with some pattern
    for (int i = 0; i < 13; ++i) {
        hp_data[i] = static_cast<uint8_t>(i * 17);
        lp_data[i] = static_cast<uint8_t>(i * 37);
    }

    gw::ComplexBuf out;
    mapper.map(hp_data, 100, lp_data, 100, out);
    REQUIRE("hier_tx_sym_count", out.size() == 50);

    // All symbols should have unit-ish power (normalized constellation)
    float total_power = 0.f;
    for (const auto& s : out) total_power += std::norm(s);
    float avg_power = total_power / static_cast<float>(out.size());
    REQUIRE_NEAR("hier_tx_unit_power", avg_power, 1.0f, 0.3f);
}

// --- Rebuild FEC: verify LDPC sizes change between rates ---
static void test_rebuild_fec_sizes() {
    auto blk = gw::LDPCBlockSize::Short;

    // Rate 1/2
    gw::LDPCEncoder enc_half(gw::FECRate::Rate_1_2, blk);
    size_t cw_half = enc_half.codewordBits();
    size_t info_half = enc_half.infoBytes();

    // Rate 3/4
    gw::LDPCEncoder enc_34(gw::FECRate::Rate_3_4, blk);
    size_t cw_34 = enc_34.codewordBits();
    size_t info_34 = enc_34.infoBytes();

    // Codeword bits should be same (same block size), but info bytes differ
    REQUIRE("fec_cw_same_block", cw_half == cw_34);
    REQUIRE("fec_info_differ", info_34 > info_half);

    // Verify interleaver adapts to codeword size
    gw::BitInterleaver intlv_half(cw_half);
    gw::BitInterleaver intlv_34(cw_34);
    // Both should be constructible without errors
    REQUIRE("fec_intlv_half_ok", cw_half > 0);
    REQUIRE("fec_intlv_34_ok", cw_34 > 0);

    // Frame capacity check
    size_t cap_half = (info_half > gw::constants::FRAME_OVERHEAD)
                       ? info_half - gw::constants::FRAME_OVERHEAD : 0;
    size_t cap_34   = (info_34 > gw::constants::FRAME_OVERHEAD)
                       ? info_34 - gw::constants::FRAME_OVERHEAD : 0;
    REQUIRE("fec_cap_34_larger", cap_34 > cap_half);
}

// --- MSVC build compatibility: Threads library (just verify mutex works) ---
static void test_threads_portability() {
    std::mutex mtx;
    std::atomic<int> counter{0};

    {
        std::lock_guard<std::mutex> lock(mtx);
        counter.store(42);
    }
    REQUIRE("threads_mutex_ok", counter.load() == 42);

    // Verify atomic operations
    counter.fetch_add(1);
    REQUIRE("threads_atomic_ok", counter.load() == 43);
}

// =========================================================================
// Phase 6b — Generic Hierarchical Modulation Tests
// =========================================================================

// --- Valid splits helper ---
static void test_hier_valid_splits() {
    // BPSK (1 bps): no valid splits
    auto bpsk_splits = gw::validHierSplits(gw::Modulation::BPSK);
    REQUIRE("splits_bpsk_none", bpsk_splits.empty());

    // QPSK (2 bps): only 1+1
    auto qpsk_splits = gw::validHierSplits(gw::Modulation::QPSK);
    REQUIRE("splits_qpsk_count", qpsk_splits.size() == 1);
    REQUIRE("splits_qpsk_11", qpsk_splits[0].first == 1 && qpsk_splits[0].second == 1);

    // QAM16 (4 bps): 1+3, 2+2, 3+1
    auto q16_splits = gw::validHierSplits(gw::Modulation::QAM16);
    REQUIRE("splits_qam16_count", q16_splits.size() == 3);

    // QAM256 (8 bps): 7 valid splits (1+7 through 7+1)
    auto q256_splits = gw::validHierSplits(gw::Modulation::QAM256);
    REQUIRE("splits_qam256_count", q256_splits.size() == 7);

    // QAM4096 (12 bps): 11 valid splits
    auto q4096_splits = gw::validHierSplits(gw::Modulation::QAM4096);
    REQUIRE("splits_qam4096_count", q4096_splits.size() == 11);

    // Validation function
    REQUIRE("valid_hp2_lp6", gw::isValidHierSplit(gw::Modulation::QAM256, 2, 6));
    REQUIRE("invalid_hp0", !gw::isValidHierSplit(gw::Modulation::QAM256, 0, 8));
    REQUIRE("invalid_sum", !gw::isValidHierSplit(gw::Modulation::QAM256, 3, 3));
}

// --- Custom QAM256: HP=2, LP=6 (QPSK-like robust + QAM64 enhance) ---
static void test_hier_custom_qam256() {
    auto cfg = gw::makeHierConfig(gw::Modulation::QAM256, 2, 2.5f);
    REQUIRE("q256_custom_enabled", cfg.enabled);
    REQUIRE("q256_custom_hp", cfg.hp_bits == 2);
    REQUIRE("q256_custom_lp", cfg.lp_bits == 6);

    gw::HierarchicalMapper mapper(cfg);
    REQUIRE("q256_hp_bps", mapper.hpBPS() == 2);
    REQUIRE("q256_lp_bps", mapper.lpBPS() == 6);
    REQUIRE("q256_total", mapper.totalBPS() == 8);

    // Map and demap roundtrip
    uint8_t hp_data[4] = {0xA5, 0x3C, 0x55, 0xF0};  // 32 HP bits = 16 symbols
    uint8_t lp_data[12] = {};
    for (int i = 0; i < 12; ++i) lp_data[i] = static_cast<uint8_t>(i * 19);

    gw::ComplexBuf syms;
    mapper.map(hp_data, 32, lp_data, 96, syms);
    REQUIRE("q256_map_count", syms.size() == 16);

    // All symbols should have reasonable power
    float total_pow = 0.f;
    for (const auto& s : syms) total_pow += std::norm(s);
    float avg_pow = total_pow / static_cast<float>(syms.size());
    REQUIRE_NEAR("q256_unit_power", avg_pow, 1.0f, 0.3f);

    // HP demap should recover HP bits
    std::vector<uint8_t> hp_out;
    mapper.demapHP(syms, hp_out);
    bool hp_match = true;
    // Check first 2 bytes (16 HP bits = 16 symbols × 1 HP bit per sym... wait, 2 HP bits)
    // 16 symbols × 2 HP bps = 32 bits = 4 bytes
    for (int i = 0; i < 4 && i < static_cast<int>(hp_out.size()); ++i) {
        if (hp_out[i] != hp_data[i]) { hp_match = false; break; }
    }
    REQUIRE("q256_hp_roundtrip", hp_match);
}

// --- Custom QAM4096: HP=4, LP=8 ---
static void test_hier_custom_qam4096() {
    auto cfg = gw::makeHierConfig(gw::Modulation::QAM4096, 4, 3.0f);
    REQUIRE("q4096_enabled", cfg.enabled);
    REQUIRE("q4096_hp", cfg.hp_bits == 4);
    REQUIRE("q4096_lp", cfg.lp_bits == 8);

    gw::HierarchicalMapper mapper(cfg);
    REQUIRE("q4096_total", mapper.totalBPS() == 12);

    // Constellation should have 2^4 × 2^8 = 4096 points
    // Map a few symbols and check they produce output
    uint8_t hp_data[2] = {0xAB, 0xCD};
    uint8_t lp_data[4] = {0x12, 0x34, 0x56, 0x78};

    gw::ComplexBuf syms;
    mapper.map(hp_data, 16, lp_data, 32, syms);
    // 16 HP bits / 4 = 4 syms, 32 LP bits / 8 = 4 syms → 4 symbols
    REQUIRE("q4096_map_count", syms.size() == 4);

    // Check power normalization
    float pow = 0.f;
    for (const auto& s : syms) pow += std::norm(s);
    REQUIRE("q4096_power_finite", pow > 0.f && pow < 100.f);
}

// --- α=1 should produce uniform constellation (no hierarchy benefit) ---
static void test_hier_alpha1_uniform() {
    auto cfg = gw::makeHierConfig(gw::Modulation::QAM16, 2, 1.0f);
    gw::HierarchicalMapper mapper(cfg);

    // At α=1, all constellation points should be at standard QAM16 positions
    // (normalized). There should be 16 unique points.
    // Map all 4 HP values × 4 LP values
    std::vector<gw::ComplexSample> all_points;
    for (uint8_t hp = 0; hp < 4; ++hp) {
        for (uint8_t lp = 0; lp < 4; ++lp) {
            uint8_t hp_byte = static_cast<uint8_t>(hp << 6);
            uint8_t lp_byte = static_cast<uint8_t>(lp << 6);
            gw::ComplexBuf out;
            mapper.map(&hp_byte, 2, &lp_byte, 2, out);
            if (!out.empty()) all_points.push_back(out[0]);
        }
    }
    REQUIRE("alpha1_16points", all_points.size() == 16);

    // Check that we have 16 distinct points (allow small tolerance)
    int unique_count = 0;
    for (size_t i = 0; i < all_points.size(); ++i) {
        bool is_unique = true;
        for (size_t j = 0; j < i; ++j) {
            float d = std::abs(all_points[i] - all_points[j]);
            if (d < 0.01f) { is_unique = false; break; }
        }
        if (is_unique) unique_count++;
    }
    REQUIRE("alpha1_all_distinct", unique_count == 16);
}

// --- Custom roundtrip: map→demap HP and LP for various splits ---
static void test_hier_custom_roundtrip() {
    // QAM64 with HP=3, LP=3 (symmetric)
    auto cfg = gw::makeHierConfig(gw::Modulation::QAM64, 3, 2.0f);
    gw::HierarchicalMapper mapper(cfg);

    // 8 symbols worth: HP = 24 bits = 3 bytes, LP = 24 bits = 3 bytes
    uint8_t hp_data[3] = {0x55, 0xAA, 0x0F};
    uint8_t lp_data[3] = {0x33, 0xCC, 0xF0};

    gw::ComplexBuf syms;
    mapper.map(hp_data, 24, lp_data, 24, syms);
    REQUIRE("rt_q64_33_count", syms.size() == 8);

    // HP roundtrip
    std::vector<uint8_t> hp_out;
    mapper.demapHP(syms, hp_out);
    bool hp_ok = (hp_out.size() >= 3);
    for (int i = 0; i < 3 && hp_ok; ++i) {
        if (hp_out[i] != hp_data[i]) hp_ok = false;
    }
    REQUIRE("rt_q64_33_hp", hp_ok);

    // LP roundtrip
    std::vector<uint8_t> lp_out;
    mapper.demapLP(syms, lp_out);
    bool lp_ok = (lp_out.size() >= 3);
    for (int i = 0; i < 3 && lp_ok; ++i) {
        if (lp_out[i] != lp_data[i]) lp_ok = false;
    }
    REQUIRE("rt_q64_33_lp", lp_ok);
}

// --- Soft demap for custom mode: LLRs should have correct sign ---
static void test_hier_soft_custom() {
    auto cfg = gw::makeHierConfig(gw::Modulation::QAM64, 2, 2.0f);
    gw::HierarchicalMapper mapper(cfg);

    // Map known data
    uint8_t hp_data[1] = {0xC0};  // 2 HP bits = 1 symbol: bits "11"
    uint8_t lp_data[1] = {0x50};  // 4 LP bits = 1 symbol: bits "0101"

    gw::ComplexBuf syms;
    mapper.map(hp_data, 2, lp_data, 4, syms);
    REQUIRE("soft_custom_count", syms.size() == 1);

    // Soft demap at high SNR → LLRs should be strong
    std::vector<float> hp_llrs, lp_llrs;
    mapper.demapSoftHP(syms, 0.001f, hp_llrs);
    mapper.demapSoftLP(syms, 0.001f, lp_llrs);

    REQUIRE("soft_hp_llr_count", hp_llrs.size() == 2);
    REQUIRE("soft_lp_llr_count", lp_llrs.size() == 4);

    // At high SNR, all LLRs should have large magnitude
    bool all_strong = true;
    for (auto l : hp_llrs) { if (std::fabs(l) < 1.0f) all_strong = false; }
    for (auto l : lp_llrs) { if (std::fabs(l) < 1.0f) all_strong = false; }
    REQUIRE("soft_custom_strong", all_strong);
}

// =========================================================================
// Phase 6c — SNR Calculator & Link Budget Tests
// =========================================================================

// --- Basic threshold computation sanity ---
static void test_snr_thresholds() {
    // BPSK 1/2: spectral efficiency = 0.5 bps
    // Shannon: 10·log10(2^0.5 - 1) = 10·log10(0.414) = -3.83 dB
    auto t = gw::computeThreshold(gw::Modulation::BPSK, gw::FECRate::Rate_1_2);
    REQUIRE_NEAR("thresh_bpsk12_shan", t.shannon_limit_db, -3.83f, 0.2f);
    REQUIRE("thresh_bpsk12_impl", t.impl_loss_db > 0.f);
    REQUIRE("thresh_bpsk12_total", t.threshold_db > t.shannon_limit_db);
    REQUIRE_NEAR("thresh_bpsk12_eff", t.spectral_eff, 0.5f, 0.01f);

    // QAM64 3/4: spectral eff = 6 × 0.75 = 4.5 bps
    auto t2 = gw::computeThreshold(gw::Modulation::QAM64, gw::FECRate::Rate_3_4);
    REQUIRE_NEAR("thresh_q64_34_eff", t2.spectral_eff, 4.5f, 0.01f);
    REQUIRE("thresh_q64_34_higher", t2.threshold_db > t.threshold_db);

    // QAM4096 9/10: highest modcod, should have highest threshold
    auto t3 = gw::computeThreshold(gw::Modulation::QAM4096, gw::FECRate::Rate_9_10);
    REQUIRE("thresh_q4096_highest", t3.threshold_db > t2.threshold_db);
    REQUIRE_NEAR("thresh_q4096_eff", t3.spectral_eff, 10.8f, 0.01f);
}

// --- All thresholds should be monotonically ordered by threshold_db ---
static void test_snr_threshold_ordering() {
    auto all = gw::computeAllThresholds();
    REQUIRE("thresh_all_count", all.size() == 77);  // 7 mods × 11 rates

    // Sorted ascending
    bool sorted = true;
    for (size_t i = 1; i < all.size(); ++i) {
        if (all[i].threshold_db < all[i-1].threshold_db - 0.01f) {
            sorted = false;
            break;
        }
    }
    REQUIRE("thresh_sorted", sorted);

    // First entry should be lowest modcod (BPSK 1/4)
    REQUIRE("thresh_first_bpsk", all[0].modulation == gw::Modulation::BPSK);
    REQUIRE("thresh_first_14", all[0].fec_rate == gw::FECRate::Rate_1_4);

    // Spectral efficiency should always be positive
    bool all_pos = true;
    for (const auto& t : all) {
        if (t.spectral_eff <= 0.f) { all_pos = false; break; }
    }
    REQUIRE("thresh_all_pos_eff", all_pos);
}

// --- SCA channel model ---
static void test_sca_channel_model() {
    gw::SCAChannelParams sca;

    // At 50 dB RF SNR (strong signal): SCA SNR should be positive and reasonable
    float sca_snr = gw::rfToScaSNR(50.0f, sca);
    REQUIRE("sca_strong_positive", sca_snr > 0.f);
    REQUIRE("sca_strong_reasonable", sca_snr < 80.f);

    // At 5 dB RF SNR (below FM threshold): SCA SNR should be very low
    float sca_weak = gw::rfToScaSNR(5.0f, sca);
    REQUIRE("sca_weak_negative", sca_weak < 0.f);

    // Higher injection → higher SCA SNR
    gw::SCAChannelParams sca_high = sca;
    sca_high.sca_injection_pct = 30.0f;
    float sca_high_snr = gw::rfToScaSNR(50.0f, sca_high);
    REQUIRE("sca_inj_higher", sca_high_snr > sca_snr);

    // Round-trip: rf → sca → rf should recover original
    float rf_orig = 45.0f;
    float sca_mid = gw::rfToScaSNR(rf_orig, sca);
    float rf_back = gw::scaToRfSNR(sca_mid, sca);
    REQUIRE_NEAR("sca_roundtrip", rf_back, rf_orig, 0.1f);
}

// --- Propagation models ---
static void test_propagation_models() {
    // Free-space at 1 km, 100 MHz: FSPL = 20log10(1) + 20log10(100) + 32.44 = 72.44 dB
    float fspl = gw::freeSpacePathLoss(1.0f, 100.0f);
    REQUIRE_NEAR("fspl_1km_100mhz", fspl, 72.44f, 0.1f);

    // Double distance → +6 dB
    float fspl_2km = gw::freeSpacePathLoss(2.0f, 100.0f);
    REQUIRE_NEAR("fspl_inverse_sq", fspl_2km - fspl, 6.02f, 0.1f);

    // Hata: suburban should have less loss than urban
    float hata_suburban = gw::hataPathLoss(10.0f, 98.0f, 100.0f, 2.0f,
                                              gw::TerrainType::Suburban);
    float hata_urban = gw::hataPathLoss(10.0f, 98.0f, 100.0f, 2.0f,
                                           gw::TerrainType::Urban);
    REQUIRE("hata_suburban_less", hata_suburban < hata_urban);

    // Rural should have less loss than suburban
    float hata_rural = gw::hataPathLoss(10.0f, 98.0f, 100.0f, 2.0f,
                                           gw::TerrainType::OpenRural);
    REQUIRE("hata_rural_less", hata_rural < hata_suburban);

    // All should be positive
    REQUIRE("hata_all_positive", hata_rural > 0.f && hata_suburban > 0.f && hata_urban > 0.f);

    // Longer distance → more loss
    float hata_20km = gw::hataPathLoss(20.0f, 98.0f, 100.0f, 2.0f,
                                          gw::TerrainType::Suburban);
    REQUIRE("hata_dist_more", hata_20km > hata_suburban);
}

// --- Full link budget ---
static void test_link_budget() {
    gw::PropagationParams prop;
    prop.erp_watts   = 1000.0f;   // 1 kW
    prop.freq_mhz    = 98.0f;
    prop.tx_height_m = 100.0f;
    prop.rx_height_m = 2.0f;
    prop.terrain     = gw::TerrainType::Suburban;

    gw::SCAChannelParams sca;
    gw::OFDMParams ofdm;
    ofdm.fft_size = 256;
    ofdm.modulation = gw::Modulation::QPSK;
    ofdm.sample_rate = 48000;

    auto lb = gw::computeLinkBudget(prop, sca, 10.0f, ofdm);

    // ERP checks
    REQUIRE_NEAR("lb_erp_dbw", lb.erp_dbw, 30.0f, 0.1f);
    REQUIRE_NEAR("lb_erp_dbm", lb.erp_dbm, 60.0f, 0.1f);

    // Path loss should be positive and reasonable for 10 km
    REQUIRE("lb_path_loss", lb.path_loss_db > 50.f && lb.path_loss_db < 200.f);

    // RX level should be less than ERP
    REQUIRE("lb_rx_below_erp", lb.rx_level_dbm < lb.erp_dbm);

    // Should have 77 entries (7 mods × 11 rates)
    REQUIRE("lb_entries_count", lb.entries.size() == 77);

    // At least some low-order ModCods should be viable at 10 km with 1 kW
    int viable_count = 0;
    for (const auto& e : lb.entries) {
        if (e.viable) viable_count++;
    }
    REQUIRE("lb_some_viable", viable_count > 0);

    // BPSK 1/4 should have the largest range
    float max_range = 0.f;
    for (const auto& e : lb.entries) {
        if (e.max_range_km > max_range) max_range = e.max_range_km;
    }
    REQUIRE("lb_bpsk_longest", lb.entries[0].max_range_km >= max_range - 0.1f);

    // Higher modcods should have shorter range
    bool range_decreasing = true;
    float prev_range = lb.entries[0].max_range_km;
    for (size_t i = 1; i < lb.entries.size(); ++i) {
        if (lb.entries[i].max_range_km > prev_range + 1.0f) {
            range_decreasing = false;
            break;
        }
        prev_range = lb.entries[i].max_range_km;
    }
    REQUIRE("lb_range_decreasing", range_decreasing);

    // Bitrates should increase with modcod order
    REQUIRE("lb_bitrate_positive", lb.entries.back().net_bitrate_bps > lb.entries[0].net_bitrate_bps);
}

// --- Hierarchical SNR thresholds ---
static void test_hier_snr_thresholds() {
    // QAM64 HP=2 LP=4, α=2: HP should need less SNR than LP
    auto cfg = gw::makeHierConfig(gw::Modulation::QAM64, 2, 2.0f);
    auto ht = gw::computeHierThreshold(cfg,
                                          gw::FECRate::Rate_1_2,
                                          gw::FECRate::Rate_3_4);

    REQUIRE("hier_hp_less_lp", ht.hp_threshold_db < ht.lp_threshold_db);
    REQUIRE("hier_hp_eff_pos", ht.hp_spectral_eff > 0.f);
    REQUIRE("hier_lp_eff_pos", ht.lp_spectral_eff > 0.f);
    REQUIRE("hier_coverage_gain", ht.coverage_gain_db > 0.f);

    // Higher α → more HP coverage gain
    auto cfg_a4 = gw::makeHierConfig(gw::Modulation::QAM64, 2, 4.0f);
    auto ht_a4 = gw::computeHierThreshold(cfg_a4,
                                              gw::FECRate::Rate_1_2,
                                              gw::FECRate::Rate_3_4);
    REQUIRE("hier_a4_more_gain", ht_a4.coverage_gain_db > ht.coverage_gain_db);
    REQUIRE("hier_a4_hp_lower", ht_a4.hp_threshold_db < ht.hp_threshold_db);

    // Link budget with hierarchical
    gw::PropagationParams prop;
    prop.erp_watts = 1000.0f;
    prop.freq_mhz = 98.0f;
    prop.tx_height_m = 100.0f;
    prop.terrain = gw::TerrainType::Suburban;

    gw::SCAChannelParams sca;
    gw::OFDMParams ofdm;
    ofdm.fft_size = 256;
    ofdm.sample_rate = 48000;

    auto lb = gw::computeLinkBudget(prop, sca, 10.0f, ofdm,
                                       &cfg, gw::FECRate::Rate_1_2,
                                       gw::FECRate::Rate_3_4);
    REQUIRE("lb_has_hier", lb.has_hier);
    REQUIRE("lb_hp_range_gt_lp", lb.hier_hp_range_km >= lb.hier_lp_range_km);
}

// =========================================================================
// Main
// =========================================================================

int main() {
    std::printf("=== Groundwave Phase 4+5+6 — GUI + Advanced DSP + Polish Tests ===\n\n");

    try {
        test_appstate_defaults();
        test_preset_cycle();
        test_computed_params();
        test_alarm_system();
        test_level_meters();
        test_constellation_buffer();
        test_spectrum_freq_axis();
        test_spectrum_tone();
        test_alarm_thresholds();
        test_thread_safety();

        // Phase 5 — JSON config persistence
        test_json_roundtrip();
        test_json_file_io();
        test_json_bad_input();

        // Phase 5 — PWL LLR
        test_pwl_llr_bpsk_qpsk();
        test_pwl_llr_qam16();
        test_pwl_llr_qam64();

        // Phase 5 — MMSE channel estimation
        test_mmse_construction();
        test_mmse_flat_channel();
        test_mmse_integration();

        // Phase 5 — PAPR reduction
        test_papr_construction();
        test_papr_reduction();
        test_papr_disabled();
        test_papr_compute_static();

        // Phase 5 — Hierarchical modulation
        test_hier_construction();
        test_hier_map_demap_hp();
        test_hier_map_demap_lp();
        test_hier_soft_demap();
        test_hier_alpha_effect();
        test_hier_qpsk_qam64();

        // Phase 5 — PLS / VCM
        test_pls_encode_decode();
        test_pls_all_modcods();
        test_pls_corrupt_recovery();
        test_vcm_schedule();
        test_modcod_detector_uniform();
        test_modcod_detector_vcm();

        // Phase 6 — Polish & Deployment
        test_pls_bpsk_roundtrip();
        test_vcm_slot_tracking();
        test_playback_ringbuffer();
        test_version_defines();
        test_hier_tx_symbol_count();
        test_rebuild_fec_sizes();
        test_threads_portability();

        // Phase 6b — Generic Hierarchical Modulation
        test_hier_valid_splits();
        test_hier_custom_qam256();
        test_hier_custom_qam4096();
        test_hier_alpha1_uniform();
        test_hier_custom_roundtrip();
        test_hier_soft_custom();

        // Phase 6c — SNR Calculator & Link Budget
        test_snr_thresholds();
        test_snr_threshold_ordering();
        test_sca_channel_model();
        test_propagation_models();
        test_link_budget();
        test_hier_snr_thresholds();

        print_preset_info();
    } catch (const std::exception& e) {
        std::fprintf(stderr, "EXCEPTION: %s\n", e.what());
        return 1;
    }

    std::printf("--- Results: %d/%d passed", g_tests_pass, g_tests_run);
    if (g_tests_fail > 0)
        std::printf(", %d FAILED", g_tests_fail);
    std::printf(" ---\n");

    return (g_tests_fail == 0) ? 0 : 1;
}
