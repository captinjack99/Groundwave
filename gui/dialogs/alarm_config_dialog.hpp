/**
 * @file alarm_config_dialog.hpp
 */
#pragma once
#include "../../include/app_state.hpp"
#include "../style.hpp"
#include <QDialog>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QLabel>

namespace dsca {

class AlarmConfigDialog : public QDialog {
    Q_OBJECT
public:
    explicit AlarmConfigDialog(AppState& state, QWidget* parent = nullptr);

private slots:
    void onApply();
    void onReset();

private:
    void buildUi();
    void loadFromState();
    QDoubleSpinBox* addThreshold(QLayout* layout,
                                  const QString& label,
                                  double min, double max,
                                  double step, const QString& suffix);

    AppState& state_;

    QDoubleSpinBox* snr_low_;
    QDoubleSpinBox* evm_high_;
    QDoubleSpinBox* ber_exp_;     // BER as 1e^x, x is the value
    QDoubleSpinBox* level_low_;
    QDoubleSpinBox* level_high_;
    QCheckBox*      cb_sync_loss_;
    QCheckBox*      cb_audio_clip_;
    QLabel*         ber_preview_;
};

} // namespace dsca
