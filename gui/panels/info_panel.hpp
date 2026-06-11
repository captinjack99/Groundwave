/**
 * @file info_panel.hpp
 * @brief Computed parameters panel — bit rates, subcarrier counts, SNR margin
 */
#pragma once
#include "../../include/app_state.hpp"
#include "../style.hpp"
#include <QWidget>
#include <QLabel>

namespace gw {

class InfoPanel : public QWidget {
    Q_OBJECT
public:
    explicit InfoPanel(AppState& state, QWidget* parent = nullptr);

public slots:
    void refresh();   // call after config changes or stats update

private:
    void buildUi();
    void setRow(QLabel* lbl, const QString& val);
    QLabel* addRow(QLayout* layout, const QString& field);

    AppState& state_;

    QLabel* sub_spacing_;
    QLabel* sym_duration_;
    QLabel* active_sc_;
    QLabel* data_sc_;
    QLabel* pilot_sc_;
    QLabel* gross_br_;
    QLabel* fec_br_;     ///< after FEC overhead
    QLabel* net_br_;     ///< after FEC + RS overhead
    QLabel* spec_eff_;
    QLabel* sig_bw_;
    QLabel* configured_bw_;   ///< modem.signal_bw (LPF cutoff)
    QLabel* overheads_;       ///< CP / pilot / FEC / RS breakdown

    // Hierarchical-modulation layer status. The whole block is hidden when
    // the engine isn't running in hierarchical mode (hier_active == false
    // in ModemStats); when active, it shows HP and LP frame counters and
    // a colored badge summarizing graceful-degradation state.
    QLabel* hier_section_title_;
    QLabel* hier_badge_;       // colored status: stereo / mono / lost
    QLabel* hp_status_;
    QLabel* lp_status_;
    QLabel* hp_frames_;
    QLabel* lp_frames_;
};

} // namespace gw
