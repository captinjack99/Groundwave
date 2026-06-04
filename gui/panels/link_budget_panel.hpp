/**
 * @file link_budget_panel.hpp
 * @brief Real-time link-budget readout: required SNR, est. range, margin.
 *
 * The user enters TX power (W or dBm), antenna gains, frequency, terrain
 * type. The panel computes:
 *
 *   - Required Es/N0 for the current ModCod (Shannon + LDPC impl loss)
 *   - Free-space + Hata path loss for selected terrain
 *   - Maximum range (binary-search Hata until path loss eats the link margin)
 *   - Net bitrate at this ModCod
 *
 * Updates whenever any input or any AppState field that affects ModCod
 * (modulation, FEC rate) changes.
 */
#pragma once

#include "../../include/app_state.hpp"
#include "../../include/snr_calculator.hpp"
#include <QWidget>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QComboBox>
#include <QLabel>
#include <QGridLayout>

namespace dsca {

class LinkBudgetPanel : public QWidget {
    Q_OBJECT
public:
    explicit LinkBudgetPanel(AppState& state, QWidget* parent = nullptr);

public slots:
    /** Recompute and refresh readouts. Call when ModCod or any input changes. */
    void recompute();

private:
    AppState& state_;

    // Inputs
    QDoubleSpinBox* tx_power_w_;       // watts (also shown in dBm)
    QDoubleSpinBox* tx_gain_db_;       // dBi
    QDoubleSpinBox* rx_gain_db_;       // dBi
    QDoubleSpinBox* freq_mhz_;         // MHz
    QDoubleSpinBox* tx_height_m_;      // tower / antenna height
    QDoubleSpinBox* rx_height_m_;
    QComboBox*      terrain_;          // FreeSpace / OpenRural / Suburban / Urban / DenseUrban
    QDoubleSpinBox* nf_db_;            // RX noise figure
    QDoubleSpinBox* margin_db_;        // required link margin

    // Outputs
    QLabel* out_modcod_;
    QLabel* out_threshold_;            // Es/N0
    QLabel* out_threshold_eb_;         // Eb/N0 (= Es/N0 - 10·log10(spectral_eff))
    QLabel* out_coding_gain_;          // dB below uncoded BPSK 1/2 Shannon
    QLabel* out_required_pr_;
    QLabel* out_path_loss_;
    QLabel* out_max_range_;
    QLabel* out_net_bitrate_;
    QLabel* out_spectral_eff_;
    QLabel* out_margin_at_1km_;

    // Hierarchical-mode readouts. Visible only when hier is on. The HP
    // layer is decodable at lower SNR than LP; the threshold gap is
    // ~6 dB at α=2, scaling linearly with α.
    QLabel* hier_section_title_  = nullptr;
    QLabel* out_hp_threshold_    = nullptr;
    QLabel* out_lp_threshold_    = nullptr;
    QLabel* out_hp_lp_gap_       = nullptr;
};

} // namespace dsca
