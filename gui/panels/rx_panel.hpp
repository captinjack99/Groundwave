/**
 * @file rx_panel.hpp
 */
#pragma once
#include "../../include/app_state.hpp"
#include "../style.hpp"
#include <QWidget>
#include <QLabel>

namespace dsca {

class RxPanel : public QWidget {
    Q_OBJECT
public:
    explicit RxPanel(AppState& state, QWidget* parent = nullptr);

public slots:
    void onStatsUpdated(dsca::ModemStats stats);
    void onAgcUpdated(float gain_db);

private:
    void buildUi();
    void applyStatusColor(QLabel* lbl, float value, float good, float warn);
    QLabel* makeBigValue(const QString& text);
    QLabel* makeFieldRow(QWidget* parent, QLayout* layout,
                         const QString& label, QLabel** value_out);

    AppState& state_;

    // Sync
    QLabel* sync_indicator_;
    QLabel* sync_label_;

    // Key measurements — large display
    QLabel* snr_value_;
    QLabel* evm_value_;
    QLabel* mer_value_;
    QLabel* freq_offset_value_;

    // Frames
    QLabel* frames_tx_;
    QLabel* frames_rx_;
    QLabel* frames_ok_;
    QLabel* frames_err_;
    QLabel* ber_value_;

    // AGC
    QLabel* agc_value_;
};

} // namespace dsca
