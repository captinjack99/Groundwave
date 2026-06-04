/**
 * @file tuning_panel.hpp
 * @brief AFC + AGC + squelch + RX gain controls in one compact panel.
 *
 * RF/DSP engineer's "knobs":
 *   - AFC enable/disable + tracked offset (Hz / ppm) readout
 *   - AGC target RMS / attack / release / max gain
 *   - RX gain (output of OS mic-boost-style gain stage, when on HW audio)
 *   - Squelch enable + open/close thresholds
 */
#pragma once

#include "../../include/app_state.hpp"
#include <QWidget>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QSlider>
#include <QLabel>

namespace dsca {

class TuningPanel : public QWidget {
    Q_OBJECT
public:
    explicit TuningPanel(AppState& state, QWidget* parent = nullptr);

public slots:
    void onStatsUpdated(dsca::ModemStats stats);
    void refreshFromState();

signals:
    void configChanged();

private:
    AppState& state_;

    QCheckBox*      afc_enable_;
    QLabel*         afc_offset_label_;       // "CFO: −24 Hz"
    QLabel*         ppm_label_;               // "Clock: +4.2 ppm"

    QDoubleSpinBox* agc_target_rms_;          // 0..1
    QDoubleSpinBox* agc_attack_ms_;
    QDoubleSpinBox* agc_release_ms_;
    QDoubleSpinBox* agc_max_gain_db_;
    QLabel*         agc_ripple_label_;        // "Ripple: 0.8 dB"

    QSlider*        rx_gain_slider_;          // -20..+40 dB
    QLabel*         rx_gain_label_;

    QCheckBox*      squelch_enable_;
    QDoubleSpinBox* squelch_open_;
    QDoubleSpinBox* squelch_close_;

    bool updating_ = false;
};

} // namespace dsca
