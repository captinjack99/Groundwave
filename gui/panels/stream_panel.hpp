/**
 * @file stream_panel.hpp
 * @brief Per-stream configuration panel for the multi-stream coordinator.
 *
 * Shows 8 rows (one per possible stream) with per-row controls:
 *   - enable checkbox
 *   - bitrate spinner (kbps)
 *   - weight spinner (relative bandwidth share)
 *   - channels (mono/stereo)
 *   - level meter
 *
 * Edits are pushed into AppState::stream_configs[] under lock; the
 * AudioEngine reads them on its next config sync.
 */
#pragma once

#include "../../include/app_state.hpp"
#include "../../include/multi_stream.hpp"
#include <QWidget>
#include <QGridLayout>
#include <QCheckBox>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <QLabel>
#include <QProgressBar>
#include <QPushButton>
#include <QToolButton>
#include <array>

namespace dsca {

class StreamPanel : public QWidget {
    Q_OBJECT
public:
    explicit StreamPanel(AppState& state, QWidget* parent = nullptr);

public slots:
    /** Refresh from AppState (called when presets load or external changes). */
    void refreshFromState();

    /** Update RX level meters per stream (called by DataBridge). */
    void onStreamLevels(const std::array<float, MAX_STREAMS>& rms_db);

signals:
    /** Emitted when any stream config changes. AudioEngine consumes this
     *  to rebuild encoders and reallocate budget. */
    void streamConfigChanged();

public:
    /** Connect to AudioEngine so the recording buttons can call into it. */
    void setEngine(class AudioEngine* engine) { engine_ = engine; }

private slots:
    void onEnableToggled(int id, bool on);
    void onBitrateChanged(int id, int kbps);
    void onWeightChanged(int id, double w);
    void onChannelsChanged(int id, int idx);
    void onSourceChanged(int id, int idx);
    void onToneFreqChanged(int id, int hz);
    void onPickFile(int id);
    void onToggleRecord(int id);

private:
    AppState& state_;

    struct Row {
        QCheckBox*      enable;
        QSpinBox*       bitrate;     // kbps
        QDoubleSpinBox* weight;
        QComboBox*      channels;    // mono / stereo
        QComboBox*      mode;        // Audio / VoIP / LowDelay
        QComboBox*      source;      // test tone / silence / mic / file
        QSpinBox*       tone_hz;     // active when source == TestTone
        QToolButton*    pick_file;   // ... button (active when source==File)
        QToolButton*    pick_input;  // ◉ button (active when source==Mic; picks capture device)
        QToolButton*    record_btn;  // toggle WAV record of decoded RX audio
        QLabel*         name;
        QProgressBar*   level;       // -60..0 dBFS
    };
    std::array<Row, MAX_STREAMS> rows_;

    /** Open a small dialog to pick the capture device for a stream's
     *  Microphone source. Populated from the system's enumerated
     *  capture devices via the engine's HWAudioDevice. */
    void onPickInputDevice(int id);
    bool                         updating_ = false;
    class AudioEngine*           engine_   = nullptr;
};

} // namespace dsca
