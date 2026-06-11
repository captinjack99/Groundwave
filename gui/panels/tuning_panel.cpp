/**
 * @file tuning_panel.cpp
 */
#include "tuning_panel.hpp"
#include "../style.hpp"
#include <QVBoxLayout>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QFrame>

namespace gw {

TuningPanel::TuningPanel(AppState& state, QWidget* parent)
    : QWidget(parent), state_(state)
{
    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(style::dim::PANEL_PADDING,
                              style::dim::PANEL_PADDING,
                              style::dim::PANEL_PADDING,
                              style::dim::PANEL_PADDING);
    root->setSpacing(style::dim::ITEM_SPACING);

    auto title = [](const QString& s){
        auto* l = new QLabel(s.toUpper());
        l->setObjectName("sectionTitle");
        return l;
    };
    auto sep = []() {
        auto* f = new QFrame;
        f->setFrameShape(QFrame::HLine);
        f->setStyleSheet("background:#1C1C24; max-height:1px; border:none;");
        return f;
    };

    // ---- AFC ----
    root->addWidget(title("AFC"));
    {
        auto* row = new QHBoxLayout;
        afc_enable_ = new QCheckBox("Enable phase-tracker / SRO loop");
        afc_enable_->setToolTip(
            "Automatic Frequency Correction. When ON, a 2nd-order phase-"
            "tracker PLL runs on the equalized pilots each frame and a "
            "sample-rate-offset estimator logs the slope across pilot "
            "tones. Together they correct residual carrier and clock drift "
            "after preamble sync.");
        row->addWidget(afc_enable_);
        row->addStretch();
        root->addLayout(row);
        afc_offset_label_ = new QLabel("CFO: — Hz");
        ppm_label_        = new QLabel("Clock: — ppm");
        afc_offset_label_->setStyleSheet("color:#C7C7CC;");
        ppm_label_->setStyleSheet("color:#C7C7CC;");
        root->addWidget(afc_offset_label_);
        root->addWidget(ppm_label_);
    }
    root->addWidget(sep());

    // ---- AGC ----
    root->addWidget(title("AGC"));
    {
        auto* g = new QGridLayout;
        g->setHorizontalSpacing(6);
        g->setVerticalSpacing(3);
        int r = 0;
        auto add = [&](const QString& l, QWidget* w) {
            g->addWidget(new QLabel(l), r, 0);
            g->addWidget(w,             r, 1);
            ++r;
        };
        agc_target_rms_  = new QDoubleSpinBox;
        agc_target_rms_->setRange(0.05, 0.95);
        agc_target_rms_->setSingleStep(0.05);
        agc_target_rms_->setValue(0.25);
        agc_target_rms_->setToolTip(
            "AGC target RMS level (0..1, fraction of full-scale). The AGC "
            "scales the input so its measured RMS sits at this value. 0.25 "
            "= -12 dBFS, a safe headroom for OFDM peaks.");
        add("Target RMS",   agc_target_rms_);
        agc_attack_ms_   = new QDoubleSpinBox;
        agc_attack_ms_->setRange(0.5, 200.0);
        agc_attack_ms_->setValue(5.0);
        agc_attack_ms_->setSuffix(" ms");
        agc_attack_ms_->setToolTip(
            "AGC attack time. How fast the gain reacts to a sudden level "
            "increase. Shorter = faster response to bursts but more risk "
            "of pumping on momentary peaks. 5 ms is a good default for "
            "OFDM (slower than the 7+ dB PAPR transients).");
        add("Attack",       agc_attack_ms_);
        agc_release_ms_  = new QDoubleSpinBox;
        agc_release_ms_->setRange(5.0, 5000.0);
        agc_release_ms_->setValue(50.0);
        agc_release_ms_->setSuffix(" ms");
        agc_release_ms_->setToolTip(
            "AGC release time. How fast the gain recovers after a peak. "
            "Longer release = smoother but slower to track a fading "
            "signal. 50 ms suits typical channel-fade rates.");
        add("Release",      agc_release_ms_);
        agc_max_gain_db_ = new QDoubleSpinBox;
        agc_max_gain_db_->setRange(0.0, 80.0);
        agc_max_gain_db_->setValue(60.0);
        agc_max_gain_db_->setSuffix(" dB");
        agc_max_gain_db_->setToolTip(
            "AGC maximum gain ceiling. Prevents the loop from chasing "
            "noise-floor levels when no signal is present (squelch-off "
            "case). 60 dB is generous; lower for very low-noise inputs.");
        add("Max Gain",     agc_max_gain_db_);
        root->addLayout(g);

        agc_ripple_label_ = new QLabel("Ripple: — dB");
        agc_ripple_label_->setStyleSheet("color:#C7C7CC;");
        root->addWidget(agc_ripple_label_);
    }
    root->addWidget(sep());

    // ---- RX gain ----
    root->addWidget(title("RX Gain"));
    {
        auto* row = new QHBoxLayout;
        rx_gain_slider_ = new QSlider(Qt::Horizontal);
        rx_gain_slider_->setRange(-200, 400);   // ×0.1 → -20 .. +40 dB
        rx_gain_slider_->setValue(0);
        rx_gain_slider_->setToolTip(
            "Pre-AGC digital gain applied to the captured passband. Use "
            "this when the soundcard's analog mic-boost is too quiet — "
            "the AGC tracks downstream, so a clean digital boost here is "
            "preferable to running the AGC pegged.");
        rx_gain_label_ = new QLabel("0.0 dB");
        rx_gain_label_->setFixedWidth(64);
        rx_gain_label_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        row->addWidget(rx_gain_slider_, 1);
        row->addWidget(rx_gain_label_);
        root->addLayout(row);
    }
    root->addWidget(sep());

    // ---- Squelch ----
    root->addWidget(title("Squelch"));
    {
        squelch_enable_ = new QCheckBox("Enable RX squelch");
        squelch_enable_->setToolTip(
            "Energy-detector squelch with hysteresis. When ON, the RX "
            "path is muted unless input energy exceeds 'Open'; once open, "
            "it stays open until energy falls below 'Close'. Prevents the "
            "demod from chasing the noise floor between transmissions.");
        root->addWidget(squelch_enable_);
        auto* g = new QGridLayout;
        squelch_open_  = new QDoubleSpinBox;
        squelch_open_->setRange(-90.0, 0.0);
        squelch_open_->setValue(-50.0);
        squelch_open_->setSuffix(" dBFS");
        squelch_open_->setToolTip(
            "Squelch open threshold (dBFS). RX unmutes when input energy "
            "rises above this level. Set ~5 dB above the noise floor seen "
            "with no signal present.");
        squelch_close_ = new QDoubleSpinBox;
        squelch_close_->setRange(-90.0, 0.0);
        squelch_close_->setValue(-55.0);
        squelch_close_->setSuffix(" dBFS");
        squelch_close_->setToolTip(
            "Squelch close threshold (dBFS). RX re-mutes when input drops "
            "below this. Keep at least 3 dB below Open to prevent rapid "
            "open/close cycling on marginal signals.");
        g->addWidget(new QLabel("Open"),  0, 0);
        g->addWidget(squelch_open_,       0, 1);
        g->addWidget(new QLabel("Close"), 1, 0);
        g->addWidget(squelch_close_,      1, 1);
        root->addLayout(g);
    }
    root->addStretch();

    // Wire all changes to a single trigger that emits configChanged()
    auto trigger = [this]() {
        if (updating_) return;
        std::lock_guard<std::mutex> lock(state_.mtx);
        // AFC drives OFDMDemodulator's phase-tracker + SRO estimator,
        // independent from I/Q balance correction.
        state_.modem.enable_afc        = afc_enable_->isChecked();
        state_.modem.agc.target_rms    = static_cast<float>(agc_target_rms_->value());
        state_.modem.agc.attack_ms     = static_cast<float>(agc_attack_ms_->value());
        state_.modem.agc.release_ms    = static_cast<float>(agc_release_ms_->value());
        state_.modem.agc.max_gain      = static_cast<float>(agc_max_gain_db_->value());
        state_.modem.enable_rx_squelch = squelch_enable_->isChecked();
        state_.modem.squelch.open_threshold_db  = static_cast<float>(squelch_open_->value());
        state_.modem.squelch.close_threshold_db = static_cast<float>(squelch_close_->value());
        state_.modem.rx_gain_db        = static_cast<float>(rx_gain_slider_->value()) * 0.1f;
    };
    // The updating_ guard must cover the EMIT too, not just the state
    // write: refreshFromState() drives every widget setter, and each
    // previously fired configChanged() (a full engine DSP rebuild) eight
    // times per refresh — while holding state_.mtx.
    auto changed = [this, trigger]() {
        if (updating_) return;
        trigger();
        emit configChanged();
    };
    connect(afc_enable_,     &QCheckBox::toggled, this, [changed](bool){ changed(); });
    connect(squelch_enable_, &QCheckBox::toggled, this, [changed](bool){ changed(); });
    connect(agc_target_rms_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [changed](double){ changed(); });
    connect(agc_attack_ms_,  QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [changed](double){ changed(); });
    connect(agc_release_ms_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [changed](double){ changed(); });
    connect(agc_max_gain_db_,QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [changed](double){ changed(); });
    connect(squelch_open_,   QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [changed](double){ changed(); });
    connect(squelch_close_,  QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [changed](double){ changed(); });
    // RX gain is a pure scalar the modem applies inline; the engine re-reads
    // state_.modem.rx_gain_db every tick (AudioEngine::processTick →
    // modem_->setRxGainDb), so update AppState but do NOT emit configChanged —
    // a full DSP teardown/reinit per slider step made dragging stutter and
    // briefly glitched decode. The change is live within ~10 ms.
    connect(rx_gain_slider_, &QSlider::valueChanged, this,
            [this, trigger](int v){
                rx_gain_label_->setText(QString("%1 dB").arg(v * 0.1f, 0, 'f', 1));
                trigger();
            });

    refreshFromState();
}

void TuningPanel::onStatsUpdated(gw::ModemStats stats) {
    afc_offset_label_->setText(
        QString("CFO: %1 Hz   (int %2 bins)")
            .arg(stats.cfo_total_hz, 0, 'f', 1)
            .arg(stats.integer_cfo_bins));
    ppm_label_->setText(
        QString("Clock: %1 ppm").arg(stats.clock_ppm, 0, 'f', 2));
    QColor rcol = (stats.agc_ripple_db <= 2.f)  ? QColor("#30D158")
                : (stats.agc_ripple_db <= 6.f) ? QColor("#FF9F0A")
                                                : QColor("#FF453A");
    agc_ripple_label_->setText(
        QString("Ripple: %1 dB").arg(stats.agc_ripple_db, 0, 'f', 1));
    agc_ripple_label_->setStyleSheet(QString("color:%1;").arg(rcol.name()));
}

void TuningPanel::refreshFromState() {
    updating_ = true;
    std::lock_guard<std::mutex> lock(state_.mtx);
    afc_enable_->setChecked(state_.modem.enable_afc);
    agc_target_rms_->setValue(state_.modem.agc.target_rms);
    agc_attack_ms_->setValue(state_.modem.agc.attack_ms);
    agc_release_ms_->setValue(state_.modem.agc.release_ms);
    agc_max_gain_db_->setValue(state_.modem.agc.max_gain);
    squelch_enable_->setChecked(state_.modem.enable_rx_squelch);
    squelch_open_->setValue(state_.modem.squelch.open_threshold_db);
    squelch_close_->setValue(state_.modem.squelch.close_threshold_db);
    rx_gain_slider_->setValue(static_cast<int>(state_.modem.rx_gain_db * 10.f));
    rx_gain_label_->setText(QString("%1 dB")
        .arg(state_.modem.rx_gain_db, 0, 'f', 1));
    updating_ = false;
}

} // namespace gw
