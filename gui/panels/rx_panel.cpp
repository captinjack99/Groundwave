/**
 * @file rx_panel.cpp
 */
#include "rx_panel.hpp"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QFrame>
#include <QStyle>
#include <cmath>

namespace gw {

static const char* syncStateName(SyncState s) {
    switch (s) {
        case SyncState::Searching: return "Searching";
        case SyncState::Acquiring: return "Acquiring";
        case SyncState::Locked:    return "Locked";
        case SyncState::Tracking:  return "Tracking";
        case SyncState::Lost:      return "SYNC LOST";
        default: return "—";
    }
}

RxPanel::RxPanel(AppState& state, QWidget* parent)
    : QWidget(parent), state_(state)
{
    buildUi();
}

QLabel* RxPanel::makeBigValue(const QString& text) {
    auto* l = new QLabel(text);
    l->setObjectName("bigValue");
    l->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    return l;
}

void RxPanel::buildUi() {
    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(style::dim::PANEL_PADDING,
                              style::dim::PANEL_PADDING,
                              style::dim::PANEL_PADDING,
                              style::dim::PANEL_PADDING);
    root->setSpacing(style::dim::ITEM_SPACING);

    auto addSep = [&]() {
        auto* line = new QFrame;
        line->setFrameShape(QFrame::HLine);
        line->setStyleSheet("background: #1C1C28; max-height: 1px; border: none;");
        root->addSpacing(4);
        root->addWidget(line);
        root->addSpacing(2);
    };

    auto addSection = [&](const QString& title) {
        auto* l = new QLabel(title.toUpper());
        l->setObjectName("sectionTitle");
        root->addWidget(l);
    };

    auto addMetric = [&](const QString& label, QLabel** out, bool big = false) {
        auto* row = new QHBoxLayout;
        row->setSpacing(8);
        auto* lbl = new QLabel(label);
        lbl->setMinimumWidth(80);
        row->addWidget(lbl, 0);
        *out = big ? makeBigValue("—") : new QLabel("—");
        if (!big) {
            (*out)->setObjectName("valueLabel");
            (*out)->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        }
        row->addWidget(*out, 1);
        root->addLayout(row);
    };

    // ---- Sync ----
    addSection("Synchronisation");
    {
        auto* row = new QHBoxLayout;
        sync_indicator_ = new QLabel("●");
        sync_indicator_->setFixedWidth(16);
        sync_indicator_->setAlignment(Qt::AlignCenter);
        sync_label_ = new QLabel("Searching");
        sync_label_->setObjectName("valueLabel");
        row->addWidget(sync_indicator_);
        row->addWidget(sync_label_, 1);
        root->addLayout(row);
    }

    // ---- Signal Quality ----
    addSep();
    addSection("Signal Quality");
    addMetric("SNR",  &snr_value_,  true);
    addMetric("EVM",  &evm_value_,  false);
    addMetric("MER",  &mer_value_,  false);
    // Same name and same quantity as the Tuning panel's CFO readout —
    // these two used to show DIFFERENT values ("Δf" = fractional-only
    // here, "CFO" = integer+fractional there) under different names.
    addMetric("CFO",  &freq_offset_value_, false);
    freq_offset_value_->setToolTip(
        "Total carrier frequency offset (integer + fractional) as "
        "corrected by the receiver. Matches the Tuning panel's CFO.");

    // ---- Frame Counters ----
    addSep();
    addSection("Frames");
    addMetric("TX",   &frames_tx_);
    addMetric("RX",   &frames_rx_);
    addMetric("OK",   &frames_ok_);
    addMetric("ERR",  &frames_err_);
    // This is the post-FEC FRAME error rate (frames_bad / frames_rx),
    // not a bit error rate — labeling it "BER" misled by orders of
    // magnitude.
    addMetric("FER",  &ber_value_);
    ber_value_->setToolTip(
        "Post-FEC frame error rate: bad frames / received frames, "
        "cumulative since start. Not a bit error rate.");

    // ---- AGC ----
    addSep();
    addSection("AGC");
    addMetric("Gain", &agc_value_);

    root->addStretch();
}

void RxPanel::applyStatusColor(QLabel* lbl, float value, float good, float warn_thresh) {
    if (value >= good)       lbl->setObjectName("statusGood");
    else if (value >= warn_thresh) lbl->setObjectName("statusWarn");
    else                     lbl->setObjectName("statusError");
    lbl->style()->unpolish(lbl);
    lbl->style()->polish(lbl);
}

void RxPanel::onStatsUpdated(gw::ModemStats stats) {
    // Sync
    bool locked = (stats.sync_state == SyncState::Locked ||
                   stats.sync_state == SyncState::Tracking);
    bool acquiring = (stats.sync_state == SyncState::Acquiring);

    sync_label_->setText(syncStateName(stats.sync_state));
    sync_indicator_->setText("●");
    if (locked)      sync_indicator_->setStyleSheet("color: #30D158;");
    else if (acquiring) sync_indicator_->setStyleSheet("color: #FF9F0A;");
    else             sync_indicator_->setStyleSheet("color: #FF453A;");

    // SNR (big readout)
    snr_value_->setText(QString("%1 dB").arg(stats.snr_db, 0, 'f', 1));
    applyStatusColor(snr_value_, stats.snr_db,
                     state_.alarm_thresh.snr_low_db + 6.f,
                     state_.alarm_thresh.snr_low_db);

    // EVM
    evm_value_->setText(QString("%1 %").arg(stats.evm_percent, 0, 'f', 2));
    applyStatusColor(evm_value_,
                     state_.alarm_thresh.evm_high_pct - stats.evm_percent,
                     10.f, 0.f);

    // MER
    float mer = (stats.evm_percent > 0.01f)
        ? -20.f * std::log10(stats.evm_percent / 100.f)
        : 60.f;
    mer_value_->setText(QString("%1 dB").arg(mer, 0, 'f', 2));
    mer_value_->setObjectName(mer > 25.f ? "statusGood"
                             : mer > 12.f ? "statusWarn" : "statusError");
    mer_value_->style()->unpolish(mer_value_);
    mer_value_->style()->polish(mer_value_);

    // Carrier frequency offset — total (integer + fractional), matching
    // the Tuning panel's CFO readout.
    freq_offset_value_->setText(
        QString("%1 Hz").arg(stats.cfo_total_hz, 0, 'f', 1));

    // Counters
    frames_tx_->setText(QString::number(static_cast<long long>(stats.frames_tx)));
    frames_rx_->setText(QString::number(static_cast<long long>(stats.frames_rx)));
    frames_ok_->setText(QString::number(static_cast<long long>(stats.frames_ok)));

    frames_err_->setText(QString::number(static_cast<long long>(stats.frames_bad)));
    frames_err_->setObjectName(stats.frames_bad == 0 ? "statusGood" : "statusError");
    frames_err_->style()->unpolish(frames_err_);
    frames_err_->style()->polish(frames_err_);

    // FER (post-FEC frame error rate)
    if (stats.ber_estimate < 1e-9f)
        ber_value_->setText("< 1e-9");
    else
        ber_value_->setText(QString("%1").arg(stats.ber_estimate, 0, 'e', 2));
}

void RxPanel::onAgcUpdated(float gain_db) {
    agc_value_->setText(QString("%1 dB").arg(gain_db, 0, 'f', 1));
}

} // namespace gw
