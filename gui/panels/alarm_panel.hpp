/**
 * @file alarm_panel.hpp
 */
#pragma once
#include "../../include/app_state.hpp"
#include "../style.hpp"
#include <QWidget>
#include <QLabel>
#include <QListWidget>
#include <QTimer>

namespace dsca {

class AlarmPanel : public QWidget {
    Q_OBJECT
public:
    explicit AlarmPanel(AppState& state, QWidget* parent = nullptr);

public slots:
    void onAlarmsUpdated(dsca::AlarmStatus status);

private slots:
    void onBlink();
    void onAcknowledge();
    void onClearLog();

private:
    void buildUi();
    void setIndicator(QLabel* lbl, bool active, int severity);
    void refreshLog();

    AppState& state_;

    // Alarm indicators
    QLabel* ind_sync_;
    QLabel* ind_clip_;
    QLabel* ind_level_high_;
    QLabel* ind_snr_;
    QLabel* ind_evm_;
    QLabel* ind_ber_;
    QLabel* ind_level_low_;

    QListWidget* log_list_;
    QTimer* blink_timer_;
    bool blink_state_ = false;
    AlarmStatus last_status_;
};

} // namespace dsca
