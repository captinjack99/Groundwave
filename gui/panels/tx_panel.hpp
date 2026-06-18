/**
 * @file tx_panel.hpp
 * @brief TX Control panel — ModCod, FEC, gain, enable
 */
#pragma once
#include "../../include/app_state.hpp"
#include "../style.hpp"

#include <QWidget>
#include <QComboBox>
#include <QSlider>
#include <QSpinBox>
#include <QPushButton>
#include <QLabel>
#include <QCheckBox>

namespace gw {

class TxPanel : public QWidget {
    Q_OBJECT
public:
    explicit TxPanel(AppState& state, QWidget* parent = nullptr);
    void refreshFromState();

    /** Replace the sample-rate combo entries with `rates`. Falls back to
     *  the standard hardcoded list when `rates` is empty. */
    void setAvailableSampleRates(const std::vector<uint32_t>& rates);

signals:
    /** Emitted when a config change requires the engine to rebuild DSP
     *  (ModCod, FEC, FFT, sample rate — anything that changes the
     *  encoder/decoder topology). MainWindow connects this to
     *  engine.onConfigChanged() which tears down and reinitializes
     *  the whole DSP chain. */
    void configChanged();

    /** Emitted for "light" state changes that DON'T require a DSP rebuild
     *  (tx_enabled flip, tx_gain change). The engine already reads these
     *  fresh from AppState every tick, so we just refresh the dependent
     *  UI panels (InfoPanel, LinkBudgetPanel) and skip the expensive
     *  teardown/init cycle that would briefly blank the engine. */
    void uiOnlyChange();

    /** Emitted when the user clicks the inline "Hier" button on the
     *  Modulation row. MainWindow connects this to openHierarchicalDialog().
     *  Putting the toggle in the modulation field (rather than a separate
     *  submenu) makes the relationship between Modulation and Hier
     *  visually obvious — hier mod IS a constellation choice. */
    void hierarchicalRequested();

private slots:
    void onTxToggle();
    void onModChanged(int idx);
    void onFecChanged(int idx);
    void onCpChanged(int idx);
    void onFftSizeChanged(int idx);
    void onSampleRateChanged(int idx);
    void onGainChanged(int val);
    void onFreqChanged(int hz);          // QSpinBox/QSlider valueChanged(int)
    void onBandwidthChanged(int hz);
    void onCenterInBandToggled(bool on);
    void onAutoBandwidth();              ///< Snap BW to OFDM occupied bandwidth
    void onChannelPresetSelected(int idx);

private:
    void buildUi();
    QLabel* makeSectionLabel(const QString& text);
    QLabel* makeValueLabel(const QString& text);

    /** Refresh slider/spinbox ranges from the current sample rate so the
     *  Nyquist constraint (BW ≤ SR/2, Fc ± BW/2 within [0, SR/2]) is
     *  enforced on every config change. */
    void refreshRanges();
    /** Update the "Band: 69.0 – 96.0 kHz" readout from current Fc and BW. */
    void refreshBandLabel();

    AppState& state_;

    QPushButton* tx_btn_;
    QComboBox*   mod_combo_;
    QPushButton* hier_btn_       = nullptr;  // inline "Hier" toggle
    QLabel*      hier_status_    = nullptr;  // shows current hier mode summary
    QComboBox*   fec_combo_;
    QComboBox*   cp_combo_;
    QComboBox*   fft_combo_;
    QComboBox*   sr_combo_;
    QSlider*     gain_slider_;
    QLabel*      gain_label_;

    // Center frequency (Hz): slider (coarse) + spinbox (precise) — both
    // bounded by the current Nyquist frequency.
    QSlider*     freq_slider_;
    QSpinBox*    freq_spin_;
    QLabel*      freq_label_;

    // Signal bandwidth (Hz): same dual control, capped at SR/2.
    QSlider*     bw_slider_;
    QSpinBox*    bw_spin_;

    // Live-computed band edge readout, e.g. "69.0 – 96.0 kHz"
    QLabel*      band_label_;

    // When on, dragging BW keeps Fc such that the upper edge stays inside
    // Nyquist and the lower edge stays ≥ 0.
    QCheckBox*   center_in_band_;

    // SCA / channel slot picker — pre-defined frequency plans optimized for
    // FM-broadcast subcarriers (67 kHz, 92 kHz) plus user-named slots.
    QComboBox*   channel_preset_;

    QLabel*      preset_label_;

    bool         updating_ = false;  // re-entrancy guard during refresh
};

} // namespace gw
