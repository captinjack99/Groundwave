/**
 * @file alarm_panel.cpp
 */
#include "alarm_panel.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QPushButton>
#include <QFrame>
#include <QScrollBar>
#include <chrono>

namespace dsca {

AlarmPanel::AlarmPanel(AppState& state, QWidget* parent)
    : QWidget(parent), state_(state)
{
    buildUi();
    blink_timer_ = new QTimer(this);
    blink_timer_->setInterval(400);
    connect(blink_timer_, &QTimer::timeout, this, &AlarmPanel::onBlink);
}

void AlarmPanel::buildUi() {
    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(style::dim::PANEL_PADDING,
                              style::dim::PANEL_PADDING,
                              style::dim::PANEL_PADDING,
                              style::dim::PANEL_PADDING);
    root->setSpacing(style::dim::ITEM_SPACING);

    // Section label helper
    auto sec = [&](const QString& t) {
        auto* l = new QLabel(t.toUpper());
        l->setObjectName("sectionTitle");
        root->addWidget(l);
    };

    auto addSep = [&]() {
        auto* f = new QFrame;
        f->setFrameShape(QFrame::HLine);
        f->setStyleSheet("background:#1C1C28;max-height:1px;border:none;");
        root->addSpacing(4); root->addWidget(f); root->addSpacing(2);
    };

    // Alarm indicator builder — dot + label on one row
    auto makeIndicator = [&](const QString& label,
                              const QString& tip = QString()) -> QLabel* {
        auto* row = new QHBoxLayout;
        row->setSpacing(8);
        auto* dot = new QLabel("●");
        dot->setFixedWidth(16);
        dot->setAlignment(Qt::AlignCenter);
        dot->setStyleSheet("color: #30D158;");   // green = OK initially
        auto* lbl = new QLabel(label);
        lbl->setObjectName("valueLabel");
        if (!tip.isEmpty()) {
            dot->setToolTip(tip);
            lbl->setToolTip(tip);
        }
        row->addWidget(dot);
        row->addWidget(lbl, 1);
        root->addLayout(row);
        return dot;  // caller stores the dot for later updates
    };

    sec("Critical");
    ind_sync_       = makeIndicator("Sync Lost",
        "Receiver lost frame synchronization. No frames are being decoded. "
        "Causes: signal dropped below threshold, frequency drift exceeded "
        "AFC pull-in range, or RX clock skew became unrecoverable.");
    ind_clip_       = makeIndicator("Audio Clip",
        "RX audio peaks reached or exceeded full-scale (0 dBFS). "
        "Persistent clipping causes distortion and AGC pumping. Reduce "
        "RX gain or check the input-stage analog headroom.");
    ind_level_high_ = makeIndicator("Level High",
        "RX input level above the configured high-threshold (default "
        "-3 dBFS). Indicates a hot signal source — verify analog "
        "front-end isn't overdriving the soundcard.");

    addSep();
    sec("Warning");
    ind_snr_        = makeIndicator("SNR Low",
        "Measured post-equalization SNR has dropped below the configured "
        "alarm threshold. Indicates marginal channel conditions; frame "
        "decode may still succeed but margin is shrinking.");
    ind_evm_        = makeIndicator("EVM High",
        "Error Vector Magnitude exceeded threshold. EVM measures the "
        "RMS distance of received symbols from ideal constellation "
        "points; high EVM at locked sync usually means residual phase "
        "noise or I/Q imbalance.");
    ind_ber_        = makeIndicator("BER High",
        "Post-FEC frame error rate exceeded threshold (default 1e-3). "
        "Frames are still arriving but failing CRC validation more "
        "often than expected.");
    ind_level_low_  = makeIndicator("Level Low",
        "RX input level fell below the configured low-threshold "
        "(default -40 dBFS). Signal may be too weak to decode reliably "
        "even with AGC at max gain.");

    // Buttons row
    addSep();
    {
        auto* row = new QHBoxLayout;
        row->setSpacing(6);
        auto* mute_btn = new QPushButton("Mute");
        mute_btn->setObjectName("accentBtn");
        mute_btn->setMinimumHeight(28);
        mute_btn->setToolTip(
            "Acknowledge current alarms. Blinking stops; the underlying "
            "conditions are still tracked and re-armed when they clear "
            "and re-trigger.");
        auto* clear_btn = new QPushButton("Clear Log");
        clear_btn->setMinimumHeight(28);
        clear_btn->setToolTip(
            "Empty the event log below. Does not affect the alarm "
            "thresholds or current condition states.");
        row->addWidget(mute_btn);
        row->addWidget(clear_btn);
        root->addLayout(row);
        connect(mute_btn,  &QPushButton::clicked, this, &AlarmPanel::onAcknowledge);
        connect(clear_btn, &QPushButton::clicked, this, &AlarmPanel::onClearLog);
    }

    // Event log
    addSep();
    sec("Event Log");
    log_list_ = new QListWidget;
    log_list_->setAlternatingRowColors(false);
    log_list_->setStyleSheet(
        "QListWidget { background:#0C0C10; border:1px solid #1C1C24;"
        "  border-radius:6px; font-size:11px; }"
        "QListWidget::item { padding:3px 8px; border:none; color:#8E8E93; }"
        "QListWidget::item:selected { background:#1C1C28; }"
    );
    log_list_->setMinimumHeight(80);
    root->addWidget(log_list_, 1);
}

// =========================================================================
// Slot: alarms updated
// =========================================================================

void AlarmPanel::onAlarmsUpdated(dsca::AlarmStatus status) {
    last_status_ = status;

    bool any_critical = status.sync_lost || status.audio_clipped || status.level_high;
    bool any_warning  = status.snr_low   || status.evm_high      ||
                        status.ber_high  || status.level_low;

    if ((any_critical || any_warning) && !status.muted)
        blink_timer_->start();
    else
        blink_timer_->stop();

    // Set indicators immediately (blink_state_ only affects active+critical)
    setIndicator(ind_sync_,       status.sync_lost,      2);
    setIndicator(ind_clip_,       status.audio_clipped,  2);
    setIndicator(ind_level_high_, status.level_high,     2);
    setIndicator(ind_snr_,        status.snr_low,        1);
    setIndicator(ind_evm_,        status.evm_high,       1);
    setIndicator(ind_ber_,        status.ber_high,       1);
    setIndicator(ind_level_low_,  status.level_low,      1);

    refreshLog();
}

void AlarmPanel::setIndicator(QLabel* dot, bool active, int severity) {
    if (!active) {
        dot->setStyleSheet("color: #30D158;");   // green
        return;
    }
    if (severity >= 2) {
        // Critical: red, blinks
        QString col = (blink_state_ && !last_status_.muted) ? "#FF453A" : "#7A201A";
        dot->setStyleSheet(QString("color: %1;").arg(col));
    } else {
        // Warning: amber, steady
        dot->setStyleSheet("color: #FF9F0A;");
    }
}

void AlarmPanel::onBlink() {
    blink_state_ = !blink_state_;
    setIndicator(ind_sync_,       last_status_.sync_lost,      2);
    setIndicator(ind_clip_,       last_status_.audio_clipped,  2);
    setIndicator(ind_level_high_, last_status_.level_high,     2);
}

void AlarmPanel::refreshLog() {
    // Snapshot the log under the lock (the engine/bridge appends to it via
    // updateAlarms()), then build the list from the local copy. Honors the
    // AppState "hold the mutex" contract instead of reading shared state
    // unlocked.
    std::vector<AlarmEvent> log_copy;
    {
        std::lock_guard<std::mutex> lock(state_.mtx);
        // Only rebuild if the log size changed (avoid flicker)
        if (log_list_->count() == static_cast<int>(state_.alarm_log.size())) return;
        log_copy = state_.alarm_log;
    }

    log_list_->clear();
    for (int i = static_cast<int>(log_copy.size()) - 1; i >= 0; --i) {
        const auto& ev = log_copy[static_cast<size_t>(i)];
        auto secs = std::chrono::duration_cast<std::chrono::seconds>(
            AlarmEvent::Clock::now() - ev.timestamp).count();

        QString text = QString("[%1s]  %2  %3")
            .arg(secs)
            .arg(ev.active ? "▲" : "▼")
            .arg(ev.typeName());

        auto* item = new QListWidgetItem(text, log_list_);
        item->setForeground(ev.active
            ? QColor(style::C_ERROR)
            : QColor(style::C_OK));
    }
    // Auto-scroll to top (newest)
    log_list_->scrollToTop();
}

void AlarmPanel::onAcknowledge() {
    {
        std::lock_guard<std::mutex> lock(state_.mtx);
        state_.acknowledgeAlarms();   // does not lock internally
    }
    blink_timer_->stop();
}

void AlarmPanel::onClearLog() {
    {
        std::lock_guard<std::mutex> lock(state_.mtx);
        state_.clearAlarmLog();       // does not lock internally
    }
    log_list_->clear();
}

} // namespace dsca
