/**
 * @file alarm_config_dialog.cpp
 */
#include "alarm_config_dialog.hpp"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QPushButton>
#include <QFrame>
#include <cmath>

namespace dsca {

AlarmConfigDialog::AlarmConfigDialog(AppState& state, QWidget* parent)
    : QDialog(parent), state_(state)
{
    setWindowTitle("Alarm Configuration");
    setFixedSize(380, 380);
    buildUi();
    loadFromState();
}

QDoubleSpinBox* AlarmConfigDialog::addThreshold(QLayout* layout,
    const QString& label, double min, double max,
    double step, const QString& suffix)
{
    auto* row  = new QHBoxLayout;
    row->setSpacing(8);
    auto* lbl  = new QLabel(label);
    lbl->setMinimumWidth(140);
    auto* spin = new QDoubleSpinBox;
    spin->setRange(min, max);
    spin->setSingleStep(step);
    spin->setSuffix(suffix);
    spin->setMinimumWidth(110);
    spin->setAlignment(Qt::AlignRight);
    row->addWidget(lbl);
    row->addWidget(spin);
    row->addStretch();
    static_cast<QVBoxLayout*>(layout)->addLayout(row);
    return spin;
}

void AlarmConfigDialog::buildUi() {
    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(16, 16, 16, 16);
    root->setSpacing(8);

    auto sec = [&](const QString& t) {
        auto* l = new QLabel(t.toUpper());
        l->setObjectName("sectionTitle");
        root->addWidget(l);
    };
    auto sep = [&]() {
        auto* f = new QFrame;
        f->setFrameShape(QFrame::HLine);
        f->setStyleSheet("background:#1C1C28;max-height:1px;border:none;");
        root->addSpacing(4); root->addWidget(f); root->addSpacing(2);
    };

    sec("Thresholds");
    snr_low_    = addThreshold(root, "SNR low alarm (dB)",    -10, 40,  0.5, " dB");
    evm_high_   = addThreshold(root, "EVM high alarm (%)",      1, 50,  1.0, " %");
    level_low_  = addThreshold(root, "Level low (dBFS)",      -80, -5,  1.0, " dBFS");
    level_high_ = addThreshold(root, "Level high (dBFS)",     -20,  0,  0.5, " dBFS");

    // BER — log scale via exponent
    {
        auto* row = new QHBoxLayout;
        row->setSpacing(8);
        auto* lbl = new QLabel("BER high alarm (1e^x)");
        lbl->setMinimumWidth(140);
        ber_exp_ = new QDoubleSpinBox;
        ber_exp_->setRange(-9, -1);
        ber_exp_->setSingleStep(0.5);
        ber_exp_->setDecimals(1);
        ber_exp_->setMinimumWidth(80);
        ber_exp_->setAlignment(Qt::AlignRight);
        ber_preview_ = new QLabel("= 1.00e-3");
        ber_preview_->setObjectName("valueLabel");
        row->addWidget(lbl);
        row->addWidget(ber_exp_);
        row->addWidget(ber_preview_);
        root->addLayout(row);
        connect(ber_exp_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                [this](double v) {
            ber_preview_->setText(QString("= %1").arg(
                std::pow(10.0, v), 0, 'e', 2));
        });
    }

    sep();
    sec("Enable / Disable");
    cb_sync_loss_  = new QCheckBox("Alarm on sync loss");
    cb_audio_clip_ = new QCheckBox("Alarm on audio clip");
    root->addWidget(cb_sync_loss_);
    root->addWidget(cb_audio_clip_);

    root->addStretch();
    sep();

    auto* btn_row = new QHBoxLayout;
    btn_row->setSpacing(8);
    auto* apply_btn  = new QPushButton("Apply");
    apply_btn->setObjectName("accentBtn");
    apply_btn->setMinimumHeight(28);
    auto* reset_btn  = new QPushButton("Reset Defaults");
    reset_btn->setMinimumHeight(28);
    auto* close_btn  = new QPushButton("Close");
    close_btn->setMinimumHeight(28);
    btn_row->addWidget(apply_btn);
    btn_row->addWidget(reset_btn);
    btn_row->addStretch();
    btn_row->addWidget(close_btn);
    root->addLayout(btn_row);

    connect(apply_btn, &QPushButton::clicked, this, &AlarmConfigDialog::onApply);
    connect(reset_btn, &QPushButton::clicked, this, &AlarmConfigDialog::onReset);
    // "Close" commits the current edits before dismissing — a user who edits a
    // threshold and clicks Close expects it to take effect (previously accept()
    // discarded everything not explicitly Applied).
    connect(close_btn, &QPushButton::clicked, this, [this]{ onApply(); accept(); });
}

void AlarmConfigDialog::loadFromState() {
    AlarmThresholds t;
    {
        std::lock_guard<std::mutex> lock(state_.mtx);
        t = state_.alarm_thresh;
    }
    snr_low_->setValue(static_cast<double>(t.snr_low_db));
    evm_high_->setValue(static_cast<double>(t.evm_high_pct));
    level_low_->setValue(static_cast<double>(t.level_low_db));
    level_high_->setValue(static_cast<double>(t.level_high_db));

    double exp = std::log10(std::max(static_cast<double>(t.ber_high), 1e-9));
    ber_exp_->setValue(exp);
    ber_preview_->setText(QString("= %1").arg(t.ber_high, 0, 'e', 2));

    cb_sync_loss_->setChecked(t.alarm_sync_loss);
    cb_audio_clip_->setChecked(t.alarm_audio_clip);
}

void AlarmConfigDialog::onApply() {
    // Snapshot widget values first, then write AppState under the lock — the
    // thresholds are read on the bridge/measurement path under state_.mtx.
    float snr   = static_cast<float>(snr_low_->value());
    float evm   = static_cast<float>(evm_high_->value());
    float lo    = static_cast<float>(level_low_->value());
    float hi    = static_cast<float>(level_high_->value());
    float ber   = static_cast<float>(std::pow(10.0, ber_exp_->value()));
    bool  sync  = cb_sync_loss_->isChecked();
    bool  clip  = cb_audio_clip_->isChecked();
    std::lock_guard<std::mutex> lock(state_.mtx);
    AlarmThresholds& t = state_.alarm_thresh;
    t.snr_low_db = snr; t.evm_high_pct = evm;
    t.level_low_db = lo; t.level_high_db = hi; t.ber_high = ber;
    t.alarm_sync_loss = sync; t.alarm_audio_clip = clip;
}

void AlarmConfigDialog::onReset() {
    {
        // Preserve the AGC-pumping fields, which this dialog has no widgets
        // for — a blanket AlarmThresholds{} would silently re-enable
        // alarm_agc_pump and reset agc_ripple_db behind the user's back. Reset
        // only what's actually editable here. (#55)
        std::lock_guard<std::mutex> lock(state_.mtx);
        AlarmThresholds def{};
        def.agc_ripple_db  = state_.alarm_thresh.agc_ripple_db;
        def.alarm_agc_pump = state_.alarm_thresh.alarm_agc_pump;
        state_.alarm_thresh = def;
    }
    loadFromState();
}

} // namespace dsca
