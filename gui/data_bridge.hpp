/**
 * @file data_bridge.hpp
 * @brief Qt-side polling bridge — connects AppState to GUI signals at ~30fps
 *
 * Runs a QTimer in the GUI thread, polls AppState (mutex-protected),
 * and emits typed Qt signals that widgets connect to directly.
 * No shared state exposed to widgets — they receive copies of values.
 */
#pragma once

#include "../include/app_state.hpp"
#include "../include/spectrum_analyzer.hpp"

#include <QObject>
#include <QTimer>
#include <array>
#include <memory>
#include <mutex>
#include <vector>

namespace dsca {

class DataBridge : public QObject {
    Q_OBJECT

public:
    explicit DataBridge(AppState& state, QObject* parent = nullptr);

    void start(int interval_ms = 33);   // ~30 fps
    void stop();

    // Direct AppState access for GUI writes (call from GUI thread only)
    AppState& state() { return state_; }
    const AppState& state() const { return state_; }

    // Feed audio samples into the spectrum analyzer (call from audio thread)
    void pushSpectrumSamples(const float* samples, size_t count);
    void pushSpectrumComplex(const ComplexSample* samples, size_t count);

    // Feed raw audio for the time-domain scope (audio thread → GUI)
    void pushScopeSamples(const float* samples, size_t count);

    // RF/DSP diagnostic feeds (audio thread → GUI thread).
    void pushChannelResponse(const std::vector<float>& mag_db);
    void pushEyeSamples(const std::vector<float>& samples, int sps);
    void pushPLS(int modulation, int fec_rate, int vcm_slot, int vcm_total,
                 bool crc_ok, int confirmation_count);

signals:
    // Emitted every poll tick (copies — safe for GUI thread)
    void statsUpdated(dsca::ModemStats stats);
    void alarmsUpdated(dsca::AlarmStatus status);
    void spectrumReady();        // widgets read directly from state().spectrum
    void constellationReady();   // widgets read directly from state().constellation
    void metersUpdated(float tx_rms_db, float rx_rms_db,
                       float tx_peak_db, float rx_peak_db,
                       bool tx_clip, bool rx_clip);
    void agcUpdated(float gain_db);
    void presetsChanged();
    void scopeSamples(const std::vector<float>& samples);
    void streamLevels(std::array<float, 8> rms_db);
    void channelResponseReady(std::vector<float> mag_db);
    void eyeSamplesReady(std::vector<float> samples, int sps);
    void plsUpdated(int modulation, int fec_rate, int vcm_slot, int vcm_total,
                    bool crc_ok, int confirmation_count);

private slots:
    void onTimer();

private:
    AppState&        state_;
    SpectrumAnalyzer analyzer_;
    QTimer*          timer_;
    int              tick_ = 0;

    // Scope sample staging (audio thread fills, GUI tick drains)
    std::vector<float> scope_pending_;
    std::mutex         scope_mtx_;

    // Eye + channel-response staging. The engine pushes these from the audio
    // thread; previously they emitted immediately (cross-thread QueuedConnection
    // deep-copy), so a TX-active engine flooded the GUI event queue faster than
    // it could repaint to a real window — the event loop fell unboundedly behind
    // and the UI froze (the "GUI hangs on TX" bug). Stage newest-only here and
    // drain once per 30 Hz onTimer, exactly like scope, so the cross-thread
    // copy rate is capped and stale frames are dropped instead of queuing.
    std::vector<float> eye_pending_;
    int                eye_pending_sps_ = 0;
    bool               eye_has_pending_ = false;
    std::vector<float> channel_pending_;
    bool               channel_has_pending_ = false;
    std::mutex         diag_mtx_;
};

} // namespace dsca
