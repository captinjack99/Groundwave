/**
 * @file tx_panel.cpp
 */
#include "tx_panel.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QFormLayout>
#include <QGroupBox>
#include <QLabel>
#include <QSizePolicy>
#include <QFrame>

namespace gw {

namespace {
// Format a sample rate as "48 kHz" / "44.1 kHz". Integer r/1000 mislabeled
// non-1000-multiple HW-probe rates (44100 → "44 kHz"). (#19-low)
QString srLabel(uint32_t r) {
    if (r % 1000u == 0u) return QString("%1 kHz").arg(r / 1000u);
    return QString("%1 kHz").arg(r / 1000.0, 0, 'f', 1);
}
} // anonymous

// =========================================================================
// Helper builders
// =========================================================================

QLabel* TxPanel::makeSectionLabel(const QString& text) {
    auto* l = new QLabel(text.toUpper());
    l->setObjectName("sectionTitle");
    return l;
}

QLabel* TxPanel::makeValueLabel(const QString& text) {
    auto* l = new QLabel(text);
    l->setObjectName("valueLabel");
    return l;
}

// =========================================================================
// Construction
// =========================================================================

TxPanel::TxPanel(AppState& state, QWidget* parent)
    : QWidget(parent), state_(state)
{
    buildUi();
    refreshFromState();
}

void TxPanel::buildUi() {
    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(style::dim::PANEL_PADDING,
                              style::dim::PANEL_PADDING,
                              style::dim::PANEL_PADDING,
                              style::dim::PANEL_PADDING);
    root->setSpacing(style::dim::ITEM_SPACING);

    // --- TX Enable ---
    tx_btn_ = new QPushButton("TX  OFF");
    tx_btn_->setObjectName("txBtn");
    tx_btn_->setProperty("active", false);
    tx_btn_->setMinimumHeight(36);
    tx_btn_->setFont([]{ QFont f; f.setPixelSize(12); f.setLetterSpacing(QFont::AbsoluteSpacing, 1.0); return f; }());
    tx_btn_->setToolTip(
        "Master transmit enable (F5). When OFF, no frames are generated — "
        "in loopback mode the receiver idles until TX is keyed again. "
        "Use Ctrl+1…Ctrl+8 to load presets.");
    root->addWidget(tx_btn_);
    connect(tx_btn_, &QPushButton::clicked, this, &TxPanel::onTxToggle);

    auto addSep = [&]() {
        auto* line = new QFrame;
        line->setFrameShape(QFrame::HLine);
        line->setStyleSheet("background: #1C1C28; max-height: 1px; border: none;");
        root->addSpacing(4);
        root->addWidget(line);
        root->addSpacing(2);
    };

    // Form-style row: fixed-width label on the left, flex control on right.
    // Label width tightened from 56 → 44 so the dock at minimum width
    // (300 px less ~24 px padding) still leaves the control comfortable
    // room. Spacing reduced from 8 → 6 px for the same reason.
    auto addRow = [&](const QString& label, QWidget* ctrl) {
        auto* row = new QHBoxLayout;
        row->setSpacing(style::dim::ITEM_SPACING);
        auto* lbl = new QLabel(label);
        lbl->setFixedWidth(44);
        row->addWidget(lbl);
        row->addWidget(ctrl, 1);
        root->addLayout(row);
    };

    // --- Modulation ---
    addSep();
    root->addWidget(makeSectionLabel("Modulation & FEC"));

    mod_combo_ = new QComboBox;
    for (const char* s : {"BPSK","QPSK","QAM-16","QAM-64","QAM-256","QAM-1024","QAM-4096"})
        mod_combo_->addItem(s);
    mod_combo_->setToolTip(
        "Constellation per subcarrier. Higher order = more bits/symbol but "
        "needs higher SNR. Rough thresholds (uncoded): BPSK ~3 dB, QPSK ~6 dB, "
        "16-QAM ~12 dB, 64-QAM ~18 dB, 256-QAM ~24 dB, 1024-QAM ~30 dB.\n\n"
        "When Hierarchical mod is enabled (Hier button below), this combo "
        "is disabled — the hierarchical configuration owns the "
        "constellation choice instead.");

    // Modulation row: combo + inline Hier button.
    {
        auto* row = new QHBoxLayout;
        row->setSpacing(style::dim::ITEM_SPACING);
        auto* lbl = new QLabel("Mod");
        lbl->setFixedWidth(44);
        row->addWidget(lbl);
        row->addWidget(mod_combo_, 1);
        hier_btn_ = new QPushButton("Hier…");
        hier_btn_->setCheckable(true);
        hier_btn_->setFixedWidth(60);
        hier_btn_->setToolTip(
            "Toggle hierarchical modulation. Opens the configuration dialog "
            "on first click; subsequent clicks turn it on/off without "
            "reopening the dialog. When ON, the Modulation combo above is "
            "disabled and the constellation is determined by the "
            "hierarchical configuration (HP/LP layer split + α).");
        row->addWidget(hier_btn_);
        root->addLayout(row);
        connect(hier_btn_, &QPushButton::clicked, this, [this](bool checked) {
            // First click (was OFF, going ON): open the config dialog so
            // the user picks a mode. The dialog applies; we re-read state
            // and reflect it. Subsequent clicks (was ON, going OFF):
            // just disable in place by re-emitting the request — caller
            // disables hier.enabled.
            (void)checked;
            emit hierarchicalRequested();
        });
    }
    // Hier status sub-line below the Mod row. Indented to align under
    // the Mod combo (44 px label width + 6 px spacing = 50 px).
    hier_status_ = makeValueLabel("Hier: off");
    hier_status_->setStyleSheet(
        "QLabel#valueLabel { color: #8E8E93; font-size: 11px; "
        "font-family: 'SF Mono', Menlo, 'DejaVu Sans Mono', monospace; "
        "padding-left: 50px; }");
    root->addWidget(hier_status_);

    connect(mod_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &TxPanel::onModChanged);

    fec_combo_ = new QComboBox;
    // No "None" entry: the engine has no uncoded transport path
    // (buildLDPCMatrix/encoder have no FECRate::None case — it fell through
    // to Rate_1_2), so offering it was a setting the engine could not
    // honor. Every listed rate maps 1:1 to a real LDPC code. (#16)
    for (const char* s : {"1/4","1/3","2/5","1/2","3/5","2/3","3/4","4/5","5/6","8/9","9/10"})
        fec_combo_->addItem(s);
    fec_combo_->setToolTip(
        "LDPC code rate (info / total). Lower rate = stronger error "
        "correction at the cost of throughput. 1/2 is a balanced default; "
        "1/4 for very weak links; 9/10 for near-Shannon-limit clean links.");
    addRow("FEC", fec_combo_);
    connect(fec_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &TxPanel::onFecChanged);

    // --- OFDM ---
    addSep();
    root->addWidget(makeSectionLabel("OFDM Parameters"));

    cp_combo_ = new QComboBox;
    for (const char* s : {"1/4","1/8","1/16","1/32"}) cp_combo_->addItem(s);
    cp_combo_->setToolTip(
        "Cyclic-prefix fraction of FFT size. Longer CP tolerates more "
        "multipath delay spread but wastes airtime. 1/8 is the default; "
        "1/4 for heavy multipath / SCA mountain paths; 1/32 for clean "
        "wired or LOS links.");
    addRow("CP", cp_combo_);
    connect(cp_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &TxPanel::onCpChanged);

    fft_combo_ = new QComboBox;
    for (const char* s : {"64","128","256","512","1024","2048","4096","8192","16384"})
        fft_combo_->addItem(s);
    fft_combo_->setToolTip(
        "OFDM FFT size = number of subcarriers. Larger FFT = finer "
        "subcarrier spacing (better frequency selectivity, more multipath "
        "robustness) but longer symbol time and higher DSP cost. 256 is a "
        "good default at 48 kHz; scale up with sample rate.");
    addRow("FFT", fft_combo_);
    connect(fft_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &TxPanel::onFftSizeChanged);

    sr_combo_ = new QComboBox;
    // Each entry stores the rate (Hz) as item data so we can swap the
    // contents at runtime via setAvailableSampleRates() without re-mapping
    // index → rate by hand. Default list is the typical soundcard rates;
    // a HW-device probe will replace this with what the device actually
    // supports when a device is selected.
    for (uint32_t r : {48000u, 96000u, 192000u, 384000u, 768000u}) {
        sr_combo_->addItem(srLabel(r),
                            QVariant::fromValue(r));
    }
    sr_combo_->setToolTip(
        "Audio sample rate (Hz). Sets the available passband: Fc + BW/2 "
        "must fit under SR/2 (Nyquist). 48 kHz is standard; 96/192 kHz "
        "give wider passband for super-wide SCA channels. The list is "
        "populated from the selected HW device's reported native rates "
        "when available.");
    addRow("Rate", sr_combo_);
    connect(sr_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &TxPanel::onSampleRateChanged);

    // --- Output ---
    addSep();
    root->addWidget(makeSectionLabel("Output"));

    {
        auto* row = new QHBoxLayout;
        row->setSpacing(style::dim::ITEM_SPACING);
        auto* lbl = new QLabel("Gain");
        lbl->setFixedWidth(44);
        gain_slider_ = new QSlider(Qt::Horizontal);
        gain_slider_->setRange(-200, 60);   // x0.1 → -20.0 to +6.0 dB
        gain_slider_->setValue(0);
        gain_slider_->setToolTip(
            "TX output gain in dB. Applied to the upconverted passband "
            "before the soundcard. Above 0 dB may saturate the output; "
            "use the TX meter to verify peaks stay below clip.");
        gain_label_ = makeValueLabel("0.0 dB");
        gain_label_->setFixedWidth(48);
        gain_label_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        row->addWidget(lbl);
        row->addWidget(gain_slider_, 1);
        row->addWidget(gain_label_);
        root->addLayout(row);
        connect(gain_slider_, &QSlider::valueChanged, this, &TxPanel::onGainChanged);
    }

    // --- RF tuning: center frequency + signal bandwidth ---
    addSep();
    root->addWidget(makeSectionLabel("RF Tuning"));

    // Channel preset row — SCA frequency plans + custom slots.
    // These match common FM-broadcast subsidiary channel allocations:
    //   67 kHz SCA: bandwidth 7.5–10 kHz, sits above L–R DSB (53 kHz) and RDS (57 kHz)
    //   92 kHz SCA: wider channel up to ~11 kHz, common for second SCA
    // For SDR / RF use cases, also include a few HF-style narrowband slots.
    {
        auto* row = new QHBoxLayout;
        row->setSpacing(style::dim::ITEM_SPACING);
        auto* lbl = new QLabel("Channel");
        lbl->setFixedWidth(44);
        channel_preset_ = new QComboBox;
        channel_preset_->setToolTip(
            "Common channel allocations. 67/92 kHz SCA: FM-broadcast "
            "subsidiary channels above L–R DSB (53 kHz) and RDS (57 kHz). "
            "Super-Wide 35 kHz: a wide 35 kHz channel centered for "
            "max-throughput SCA work. HF narrowband: 12 kHz channels "
            "typical of long-distance HF data.");
        channel_preset_->addItem("Custom",                 QVariant());          // 0
        channel_preset_->addItem("SCA 67 kHz (narrow)",   QVariant());          // 1
        channel_preset_->addItem("SCA 67 kHz (wide)",     QVariant());          // 2
        channel_preset_->addItem("SCA 92 kHz (narrow)",   QVariant());          // 3
        channel_preset_->addItem("SCA 92 kHz (wide)",     QVariant());          // 4
        channel_preset_->addItem("SCA Super-Wide 35 kHz", QVariant());          // 5
        channel_preset_->addItem("L+R Mono (0–15 kHz)",   QVariant());          // 6
        channel_preset_->addItem("HF Narrowband 12 kHz", QVariant());           // 7
        channel_preset_->addItem("Wide 24 kHz @ 12k",    QVariant());           // 8
        row->addWidget(lbl);
        row->addWidget(channel_preset_, 1);
        root->addLayout(row);
        connect(channel_preset_,
                QOverload<int>::of(&QComboBox::currentIndexChanged),
                this, &TxPanel::onChannelPresetSelected);
    }

    // Fc row: slider (coarse drag) + spinbox (1-Hz precision typing).
    // The duplicate kHz label (freq_label_) used to live here too but
    // was visually redundant with the spinbox — the spinbox shows raw
    // Hz which is the editable, unambiguous form. freq_label_ is still
    // updated in code paths for any external consumer / future use,
    // just not displayed in this row.
    {
        auto* row = new QHBoxLayout;
        row->setSpacing(style::dim::ITEM_SPACING);
        auto* lbl = new QLabel("Fc");
        lbl->setFixedWidth(44);
        freq_slider_ = new QSlider(Qt::Horizontal);
        freq_slider_->setRange(0, 24000);
        freq_slider_->setSingleStep(100);
        freq_slider_->setPageStep(1000);
        freq_slider_->setValue(12000);
        const QString fc_tip =
            "Center frequency of the OFDM signal (Hz). The complex baseband "
            "is upconverted to Fc on TX and downconverted on RX. Fc + BW/2 "
            "must stay under SR/2 (Nyquist) or 'Auto-clamp' will snap it.";
        freq_slider_->setToolTip(fc_tip);
        freq_spin_ = new QSpinBox;
        freq_spin_->setRange(0, 24000);
        freq_spin_->setSingleStep(100);
        freq_spin_->setSuffix(" Hz");
        freq_spin_->setValue(12000);
        freq_spin_->setMinimumWidth(88);
        freq_spin_->setMaximumWidth(110);
        freq_spin_->setAccelerated(true);
        freq_spin_->setToolTip(fc_tip);
        // freq_label_ is no longer displayed; kept alive as a member
        // because external slots reference it.
        freq_label_ = makeValueLabel("12.0 kHz");
        freq_label_->setVisible(false);
        row->addWidget(lbl);
        row->addWidget(freq_slider_, 1);
        row->addWidget(freq_spin_);
        root->addLayout(row);
        connect(freq_slider_, &QSlider::valueChanged,
                this, &TxPanel::onFreqChanged);
        connect(freq_spin_, QOverload<int>::of(&QSpinBox::valueChanged),
                this, &TxPanel::onFreqChanged);
    }

    // Bandwidth row: same pattern, capped at SR/2 (Nyquist).
    // The "Auto" button snaps BW to the OFDM occupied bandwidth
    // (subcarrier_spacing × active_subcarriers) so the analog filter exactly
    // matches the digital signal — this is the optimal setting in nearly all
    // cases, so we expose it as a one-click action instead of doing it
    // implicitly (the user may want narrower, e.g. for spectrum-mask testing).
    {
        auto* row = new QHBoxLayout;
        row->setSpacing(style::dim::ITEM_SPACING);
        auto* lbl = new QLabel("BW");
        lbl->setFixedWidth(44);
        bw_slider_ = new QSlider(Qt::Horizontal);
        bw_slider_->setRange(500, 24000);
        bw_slider_->setSingleStep(100);
        bw_slider_->setPageStep(1000);
        bw_slider_->setValue(20000);
        const QString bw_tip =
            "Signal bandwidth (Hz). Sets the LPF cutoff that brick-walls "
            "the RX baseband and shapes the TX upconvert. Smaller BW gives "
            "better adjacent-channel rejection but cuts off OFDM "
            "subcarriers beyond ±BW/2. Click 'Auto' to match the OFDM "
            "occupied bandwidth exactly.";
        bw_slider_->setToolTip(bw_tip);
        bw_spin_ = new QSpinBox;
        bw_spin_->setRange(500, 24000);
        bw_spin_->setSingleStep(100);
        bw_spin_->setSuffix(" Hz");
        bw_spin_->setValue(20000);
        bw_spin_->setMinimumWidth(88);
        bw_spin_->setMaximumWidth(110);
        bw_spin_->setAccelerated(true);
        bw_spin_->setToolTip(bw_tip);
        auto* auto_btn = new QPushButton("Auto");
        auto_btn->setToolTip("Snap BW to OFDM occupied bandwidth\n"
                              "(subcarrier_spacing × active_subcarriers)");
        auto_btn->setFixedWidth(44);
        row->addWidget(lbl);
        row->addWidget(bw_slider_, 1);
        row->addWidget(bw_spin_);
        row->addWidget(auto_btn);
        root->addLayout(row);
        connect(bw_slider_, &QSlider::valueChanged,
                this, &TxPanel::onBandwidthChanged);
        connect(bw_spin_, QOverload<int>::of(&QSpinBox::valueChanged),
                this, &TxPanel::onBandwidthChanged);
        connect(auto_btn, &QPushButton::clicked,
                this, &TxPanel::onAutoBandwidth);
    }

    // Live band-edge readout + auto-clamp toggle
    {
        auto* row = new QHBoxLayout;
        row->setSpacing(8);
        band_label_ = makeValueLabel("Band: — kHz");
        band_label_->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
        center_in_band_ = new QCheckBox("Auto-clamp to Nyquist");
        center_in_band_->setChecked(true);
        center_in_band_->setToolTip(
            "When ON, Fc is automatically clamped so the band edges "
            "(Fc ± BW/2) stay inside 0…SR/2. Turn off if you intentionally "
            "want to place the signal partly outside Nyquist for testing.");
        row->addWidget(band_label_, 1);
        row->addWidget(center_in_band_);
        root->addLayout(row);
        connect(center_in_band_, &QCheckBox::toggled,
                this, &TxPanel::onCenterInBandToggled);
    }

    // --- Preset indicator ---
    addSep();
    preset_label_ = new QLabel("Preset: Standard");
    preset_label_->setObjectName("valueLabel");
    preset_label_->setAlignment(Qt::AlignCenter);
    root->addWidget(preset_label_);

    root->addStretch();
}

// =========================================================================
// Refresh
// =========================================================================

void TxPanel::refreshRanges() {
    // Pull current sample rate under the lock to compute Nyquist.
    uint32_t sr;
    {
        std::lock_guard<std::mutex> lock(state_.mtx);
        sr = state_.ofdm.sample_rate ? state_.ofdm.sample_rate : 48000u;
    }
    int nyquist = static_cast<int>(sr / 2u);

    // Fc: 0..Nyquist (typically Fc + BW/2 ≤ Nyquist; the spinbox/slider
    // edit will be clamped further when BW is known).
    freq_slider_->blockSignals(true);
    freq_spin_->blockSignals(true);
    freq_slider_->setRange(0, nyquist);
    freq_spin_->setRange(0, nyquist);
    freq_slider_->blockSignals(false);
    freq_spin_->blockSignals(false);

    // BW: minimum 200 Hz so the LPF design stays sane; max = Nyquist
    bw_slider_->blockSignals(true);
    bw_spin_->blockSignals(true);
    bw_slider_->setRange(200, nyquist);
    bw_spin_->setRange(200, nyquist);
    bw_slider_->blockSignals(false);
    bw_spin_->blockSignals(false);
}

void TxPanel::refreshBandLabel() {
    int fc = freq_spin_->value();
    int bw = bw_spin_->value();
    int lo = fc - bw / 2;
    int hi = fc + bw / 2;
    int nyq;
    {
        std::lock_guard<std::mutex> lock(state_.mtx);
        nyq = static_cast<int>((state_.ofdm.sample_rate
                                 ? state_.ofdm.sample_rate : 48000u) / 2);
    }
    bool out_of_band = (lo < 0) || (hi > nyq);
    QString text = QString("Band: %1 – %2 kHz   (Nyquist %3 kHz)")
        .arg(lo / 1000.f, 0, 'f', 2)
        .arg(hi / 1000.f, 0, 'f', 2)
        .arg(nyq / 1000.f, 0, 'f', 1);
    if (out_of_band) {
        text += "  ⚠";
        band_label_->setStyleSheet("color:#FF453A;");
    } else {
        band_label_->setStyleSheet("");
    }
    band_label_->setText(text);
}

void TxPanel::refreshFromState() {
    updating_ = true;

    // Snapshot engine-shared config under the lock. The engine's AMC loop
    // writes ofdm.modulation / frame.fec_rate from the engine thread, so
    // reading them unlocked is a data race (torn enum reads). Snapshot all
    // fields the body needs, then drive the widgets from locals.
    // refreshRanges()/refreshBandLabel() lock state_.mtx internally, so we
    // must release the lock before calling them (non-recursive mutex).
    Modulation         s_mod;
    FECRate            s_fec;
    CyclicPrefix       s_cp;
    uint16_t           s_fft;
    uint32_t           s_sr;
    float              s_gain, s_fc, s_bw;
    bool               s_tx_on;
    HierarchicalConfig s_hier;
    int                s_preset_slot;
    bool               s_preset_mod;
    std::string        s_preset_name;
    {
        std::lock_guard<std::mutex> lock(state_.mtx);
        s_mod         = state_.ofdm.modulation;
        s_fec         = state_.frame.fec_rate;
        s_cp          = state_.ofdm.cyclic_prefix;
        s_fft         = state_.ofdm.fft_size;
        s_sr          = state_.ofdm.sample_rate;
        s_gain        = state_.tx_gain_db;
        s_fc          = state_.modem.center_freq;
        s_bw          = state_.modem.signal_bw;
        s_tx_on       = state_.tx_enabled;
        s_hier        = state_.hier;
        s_preset_slot = state_.active_preset_slot;
        s_preset_mod  = state_.preset_modified;
        if (s_preset_slot >= 0)
            s_preset_name =
                state_.presets[static_cast<size_t>(s_preset_slot)].name;
    }

    // Block signals during population
    const bool blocked = true;
    mod_combo_->blockSignals(blocked);
    fec_combo_->blockSignals(blocked);
    cp_combo_->blockSignals(blocked);
    fft_combo_->blockSignals(blocked);
    sr_combo_->blockSignals(blocked);
    gain_slider_->blockSignals(blocked);
    freq_slider_->blockSignals(blocked);
    freq_spin_->blockSignals(blocked);
    bw_slider_->blockSignals(blocked);
    bw_spin_->blockSignals(blocked);

    mod_combo_->setCurrentIndex(static_cast<int>(s_mod));
    fec_combo_->setCurrentIndex(static_cast<int>(s_fec) > 10
                                ? 10 : static_cast<int>(s_fec));
    cp_combo_->setCurrentIndex(static_cast<int>(s_cp));

    static const int fft_vals[] = {64,128,256,512,1024,2048,4096,8192,16384};
    int fft_idx = 2;
    for (int i = 0; i < 9; ++i)
        if (s_fft == fft_vals[i]) { fft_idx = i; break; }
    fft_combo_->setCurrentIndex(fft_idx);

    // Find matching rate by item data. If the active rate isn't in the list
    // (e.g. a preset/loaded config set a rate the current HW-device probe
    // dropped from the combo), APPEND it rather than silently snapping to
    // index 0 — otherwise the combo would display a different rate than the
    // engine is actually running, desynced with no indication.
    int sr_idx = -1;
    for (int i = 0; i < sr_combo_->count(); ++i) {
        if (sr_combo_->itemData(i).toUInt() == s_sr) { sr_idx = i; break; }
    }
    if (sr_idx < 0 && s_sr > 0) {
        sr_combo_->addItem(QString("%1 Hz").arg(s_sr), s_sr);
        sr_idx = sr_combo_->count() - 1;
    }
    sr_combo_->setCurrentIndex(sr_idx < 0 ? 0 : sr_idx);

    gain_slider_->setValue(static_cast<int>(s_gain * 10.f));

    // Update Fc/BW ranges based on the current sample rate, THEN set values.
    refreshRanges();
    int fc = static_cast<int>(s_fc);
    int bw = static_cast<int>(s_bw);
    if (bw <= 0) {
        // signal_bw == 0 means "auto" — pick a sensible default that
        // sits within Nyquist (40% of sample rate, capped).
        uint32_t sr = s_sr ? s_sr : 48000u;
        bw = static_cast<int>(sr) * 4 / 10;
    }
    freq_slider_->setValue(fc);
    freq_spin_->setValue(fc);
    bw_slider_->setValue(bw);
    bw_spin_->setValue(bw);

    mod_combo_->blockSignals(false);
    fec_combo_->blockSignals(false);
    cp_combo_->blockSignals(false);
    fft_combo_->blockSignals(false);
    sr_combo_->blockSignals(false);
    gain_slider_->blockSignals(false);
    freq_slider_->blockSignals(false);
    freq_spin_->blockSignals(false);
    bw_slider_->blockSignals(false);
    bw_spin_->blockSignals(false);

    // TX button state
    bool on = s_tx_on;
    tx_btn_->setText(on ? "TX   ON" : "TX  OFF");
    tx_btn_->setProperty("active", on);
    tx_btn_->style()->unpolish(tx_btn_);
    tx_btn_->style()->polish(tx_btn_);

    // Hierarchical-modulation reflection. When ON, disable the plain
    // Modulation combo (hier owns the constellation) and show the
    // resolved HP/LP layer summary below the combo.
    const auto& h = s_hier;
    if (hier_btn_) {
        hier_btn_->blockSignals(true);
        hier_btn_->setChecked(h.enabled);
        hier_btn_->setProperty("active", h.enabled);
        hier_btn_->style()->unpolish(hier_btn_);
        hier_btn_->style()->polish(hier_btn_);
        hier_btn_->blockSignals(false);
    }
    if (hier_status_) {
        if (h.enabled) {
            uint8_t hp = h.effectiveHP();
            uint8_t lp = h.effectiveLP();
            hier_status_->setText(QString("Hier: %1   ·   HP=%2 / LP=%3 bps   ·   α=%4")
                                      .arg(QString::fromUtf8(hierarchicalModeName(h.mode)))
                                      .arg(hp).arg(lp)
                                      .arg(h.alpha, 0, 'f', 2));
            hier_status_->setStyleSheet(
                "QLabel#valueLabel { color: #0099FF; font-size: 11px; "
                "font-family: 'SF Mono', Menlo, 'DejaVu Sans Mono', monospace; "
                "padding-left: 64px; }");
        } else {
            hier_status_->setText("Hier: off  (uniform modulation)");
            hier_status_->setStyleSheet(
                "QLabel#valueLabel { color: #48484E; font-size: 11px; "
                "font-family: 'SF Mono', Menlo, 'DejaVu Sans Mono', monospace; "
                "padding-left: 64px; }");
        }
    }
    mod_combo_->setEnabled(!h.enabled);
    if (h.enabled) {
        mod_combo_->setStyleSheet(
            "QComboBox { color: #48484E; }");
    } else {
        mod_combo_->setStyleSheet(QString());  // revert to global style
    }

    gain_label_->setText(QString("%1 dB").arg(s_gain, 0, 'f', 1));
    freq_label_->setText(QString("%1 kHz").arg(fc / 1000.f, 0, 'f', 2));

    refreshBandLabel();

    if (s_preset_slot >= 0) {
        QString name = QString::fromStdString(s_preset_name);
        preset_label_->setText(s_preset_mod
            ? QString("Preset: %1 *").arg(name)
            : QString("Preset: %1").arg(name));
    } else {
        preset_label_->setText("Custom");
    }
    updating_ = false;
}

// =========================================================================
// Slots
// =========================================================================

// Critical: these slots must NOT hold state_.mtx while emitting configChanged()
// or calling refreshFromState(). Connected slots (InfoPanel::refresh, others)
// re-lock the same non-recursive std::mutex, which is undefined behavior and
// crashes the GUI on Windows. Pattern: scope the lock to state mutations only,
// then emit and refresh outside.

void TxPanel::onTxToggle() {
    {
        std::lock_guard<std::mutex> lock(state_.mtx);
        state_.tx_enabled = !state_.tx_enabled;
    }
    refreshFromState();
    // CRITICAL: TX enable is read fresh from AppState by every tick of
    // the engine — see audio_engine.cpp processTick(). We DO NOT need
    // to teardown/rebuild the DSP chain just because the user clicked
    // the TX button. Emitting `configChanged()` here would do exactly
    // that, briefly nulling out opus_enc_/ldpc_enc_/etc. and causing
    // processTX to skip on its early-null-guard — which manifested as
    // "click TX, nothing happens for a second or two." Use the lighter
    // uiOnlyChange signal instead.
    emit uiOnlyChange();
}

void TxPanel::onModChanged(int idx) {
    if (idx < 0 || idx > 6) return;     // bounds: BPSK..QAM4096
    {
        std::lock_guard<std::mutex> lock(state_.mtx);
        state_.ofdm.modulation = static_cast<Modulation>(idx);
        state_.preset_modified = true;
    }
    refreshFromState();
    emit configChanged();
}

void TxPanel::onFecChanged(int idx) {
    // The combo now has exactly 11 entries (0..10 = the real LDPC rates);
    // the bogus "None" entry was removed since the engine has no uncoded
    // path. idx maps 1:1 to FECRate 0..10. (#16)
    if (idx < 0 || idx > 10) return;
    FECRate r = static_cast<FECRate>(idx);
    {
        std::lock_guard<std::mutex> lock(state_.mtx);
        state_.frame.fec_rate  = r;
        state_.preset_modified = true;
    }
    refreshFromState();
    emit configChanged();
}

void TxPanel::onCpChanged(int idx) {
    if (idx < 0 || idx > 3) return;     // CP_1_4 .. CP_1_32
    {
        std::lock_guard<std::mutex> lock(state_.mtx);
        state_.ofdm.cyclic_prefix = static_cast<CyclicPrefix>(idx);
        state_.preset_modified = true;
    }
    refreshFromState();
    emit configChanged();
}

void TxPanel::onFftSizeChanged(int idx) {
    static const int fft_vals[] = {64,128,256,512,1024,2048,4096,8192,16384};
    if (idx < 0 || idx >= static_cast<int>(sizeof(fft_vals) / sizeof(fft_vals[0]))) return;
    {
        std::lock_guard<std::mutex> lock(state_.mtx);
        state_.ofdm.fft_size = static_cast<uint16_t>(fft_vals[idx]);
        state_.preset_modified = true;
    }
    // Refresh panel-internal derived state (preset "*" modified marker, etc.)
    // before notifying the engine — every other modcod slot does this, but
    // onFftSizeChanged omitted it, leaving the panel's own state stale.
    refreshFromState();
    emit configChanged();
}

void TxPanel::onSampleRateChanged(int idx) {
    if (idx < 0 || idx >= sr_combo_->count()) return;
    uint32_t new_sr = sr_combo_->itemData(idx).toUInt();
    if (new_sr == 0) return;
    {
        std::lock_guard<std::mutex> lock(state_.mtx);
        state_.ofdm.sample_rate  = new_sr;
        state_.modem.sample_rate = new_sr;
        // If the existing center freq + BW would now exceed the new Nyquist,
        // clamp before the engine sees the changed config and rejects it.
        float nyq = static_cast<float>(new_sr) * 0.5f;
        if (state_.modem.signal_bw <= 0.f ||
            state_.modem.signal_bw > nyq) {
            state_.modem.signal_bw = nyq * 0.8f;
        }
        float half_bw = state_.modem.signal_bw * 0.5f;
        if (state_.modem.center_freq < half_bw) {
            state_.modem.center_freq = half_bw;
        }
        if (state_.modem.center_freq + half_bw > nyq) {
            state_.modem.center_freq = nyq - half_bw;
        }
        state_.preset_modified = true;
    }
    refreshFromState();        // re-bound the sliders/spinboxes
    emit configChanged();
}

void TxPanel::onGainChanged(int val) {
    // No state lock needed — these are atomic float/scalar writes only the
    // GUI thread mutates, and reads from the audio thread are racy but benign
    // (gain change is rate-limited by the slider).
    float db = val * 0.1f;
    {
        std::lock_guard<std::mutex> lock(state_.mtx);
        state_.tx_gain_db = db;
    }
    gain_label_->setText(QString("%1 dB").arg(db, 0, 'f', 1));
}

void TxPanel::onFreqChanged(int hz) {
    if (updating_) return;

    // Mirror slider ↔ spinbox without recursion
    if (sender() == freq_slider_ && freq_spin_->value() != hz) {
        freq_spin_->blockSignals(true);
        freq_spin_->setValue(hz);
        freq_spin_->blockSignals(false);
    } else if (sender() == freq_spin_ && freq_slider_->value() != hz) {
        freq_slider_->blockSignals(true);
        freq_slider_->setValue(hz);
        freq_slider_->blockSignals(false);
    }

    // Apply Nyquist + BW clamp if "Auto-clamp" is on
    int bw = bw_spin_->value();
    int nyq;
    {
        std::lock_guard<std::mutex> lock(state_.mtx);
        nyq = static_cast<int>(
            (state_.ofdm.sample_rate ? state_.ofdm.sample_rate : 48000u) / 2);
    }
    if (center_in_band_->isChecked()) {
        int half_bw = bw / 2;
        if (hz < half_bw) hz = half_bw;
        if (hz + half_bw > nyq) hz = nyq - half_bw;
        if (hz != freq_spin_->value()) {
            freq_spin_->blockSignals(true);
            freq_slider_->blockSignals(true);
            freq_spin_->setValue(hz);
            freq_slider_->setValue(hz);
            freq_spin_->blockSignals(false);
            freq_slider_->blockSignals(false);
        }
    }

    {
        std::lock_guard<std::mutex> lock(state_.mtx);
        state_.modem.center_freq = static_cast<float>(hz);
        state_.preset_modified = true;
    }
    freq_label_->setText(QString("%1 kHz").arg(hz / 1000.f, 0, 'f', 2));
    refreshBandLabel();
    emit configChanged();
}

void TxPanel::onBandwidthChanged(int hz) {
    if (updating_) return;

    if (sender() == bw_slider_ && bw_spin_->value() != hz) {
        bw_spin_->blockSignals(true);
        bw_spin_->setValue(hz);
        bw_spin_->blockSignals(false);
    } else if (sender() == bw_spin_ && bw_slider_->value() != hz) {
        bw_slider_->blockSignals(true);
        bw_slider_->setValue(hz);
        bw_slider_->blockSignals(false);
    }

    int nyq;
    {
        std::lock_guard<std::mutex> lock(state_.mtx);
        nyq = static_cast<int>(
            (state_.ofdm.sample_rate ? state_.ofdm.sample_rate : 48000u) / 2);
    }

    // If auto-clamp is on, slide Fc to keep the band inside Nyquist when
    // BW grows. Otherwise show the warning and let the user fix it.
    int fc = freq_spin_->value();
    if (center_in_band_->isChecked()) {
        int half_bw = hz / 2;
        if (fc - half_bw < 0)    fc = half_bw;
        if (fc + half_bw > nyq)  fc = nyq - half_bw;
        if (fc != freq_spin_->value()) {
            freq_spin_->blockSignals(true);
            freq_slider_->blockSignals(true);
            freq_spin_->setValue(fc);
            freq_slider_->setValue(fc);
            freq_spin_->blockSignals(false);
            freq_slider_->blockSignals(false);
        }
    }

    {
        std::lock_guard<std::mutex> lock(state_.mtx);
        state_.modem.signal_bw   = static_cast<float>(hz);
        state_.modem.center_freq = static_cast<float>(fc);
        state_.preset_modified   = true;
    }
    freq_label_->setText(QString("%1 kHz").arg(fc / 1000.f, 0, 'f', 2));
    refreshBandLabel();
    emit configChanged();
}

void TxPanel::onCenterInBandToggled(bool /*on*/) {
    // Re-run the clamp logic immediately if auto-clamp was just enabled.
    onFreqChanged(freq_spin_->value());
}

void TxPanel::onAutoBandwidth() {
    // Snap BW to the OFDM occupied bandwidth:
    //   occupied_bw = subcarrier_spacing × active_subcarriers
    //   subcarrier_spacing = sample_rate / fft_size
    //   active_subcarriers = fft_size − guards (typically 80% of FFT)
    uint32_t sr;
    uint16_t fft;
    {
        std::lock_guard<std::mutex> lock(state_.mtx);
        sr  = state_.ofdm.sample_rate ? state_.ofdm.sample_rate : 48000u;
        fft = state_.ofdm.fft_size    ? state_.ofdm.fft_size    : uint16_t(256);
    }
    if (fft == 0) return;
    // Guard 10% on each side (matches OFDMParams::guardLeft/guardRight defaults).
    double sc_spacing = static_cast<double>(sr) / fft;
    int active        = static_cast<int>(fft) - 2 * (fft / 10);
    int occupied_hz   = static_cast<int>(std::ceil(sc_spacing * active));
    // Round up to nearest 100 Hz to match spinbox step
    occupied_hz = ((occupied_hz + 99) / 100) * 100;
    // Apply via the normal change path (handles Nyquist clamping)
    bw_spin_->setValue(occupied_hz);
}

void TxPanel::setAvailableSampleRates(const std::vector<uint32_t>& rates) {
    // Rebuild the SR combo with the supplied rates, preserving the currently
    // selected rate if it's in the new list. Fall back to defaults when the
    // input is empty (e.g., no HW audio context).
    sr_combo_->blockSignals(true);
    uint32_t prev = sr_combo_->itemData(sr_combo_->currentIndex()).toUInt();
    sr_combo_->clear();
    if (rates.empty()) {
        for (uint32_t r : {48000u, 96000u, 192000u, 384000u, 768000u}) {
            sr_combo_->addItem(srLabel(r),
                                QVariant::fromValue(r));
        }
    } else {
        for (uint32_t r : rates) {
            sr_combo_->addItem(srLabel(r),
                                QVariant::fromValue(r));
        }
    }
    // Try to restore the previous selection
    int restore = 0;
    for (int i = 0; i < sr_combo_->count(); ++i) {
        if (sr_combo_->itemData(i).toUInt() == prev) { restore = i; break; }
    }
    sr_combo_->setCurrentIndex(restore);
    sr_combo_->blockSignals(false);
}

void TxPanel::onChannelPresetSelected(int idx) {
    if (updating_) return;

    // Each preset sets {Fc, BW, sample_rate (if not already high enough)}.
    // After applying state, we call refreshFromState to re-bound the GUI
    // sliders, then emit configChanged so the engine re-inits.
    struct Preset {
        const char* name;
        uint32_t    sample_rate;
        int         fc_hz;
        int         bw_hz;
    };
    static const Preset table[] = {
        { "Custom",                  0,       0,     0 },         // 0
        { "SCA 67 kHz (narrow)",     192000,  67000, 7500 },      // 1
        { "SCA 67 kHz (wide)",       192000,  67000, 10000 },     // 2
        { "SCA 92 kHz (narrow)",     192000,  92000, 8000 },      // 3
        { "SCA 92 kHz (wide)",       192000,  92000, 11000 },     // 4
        // Super-wide spans the entire usable SCA region between RDS and the
        // 192 kHz Nyquist limit. Lower edge 61 kHz (1 kHz guard above RDS at
        // 60 kHz), upper edge 96 kHz (= Nyquist; the soundcard physically
        // can't represent anything beyond it). Fc=78.5k, BW=35k.
        { "SCA Super-Wide 35 kHz",   192000,  78500, 35000 },     // 5
        { "L+R Mono (0–15 kHz)",     48000,   7500,  15000 },     // 6
        { "HF Narrowband 12 kHz",    48000,   12000, 3000 },      // 7
        { "Wide 24 kHz @ 12k",       96000,   12000, 24000 },     // 8
    };
    if (idx <= 0 || idx >= static_cast<int>(sizeof(table)/sizeof(table[0]))) {
        return; // "Custom" or out-of-range — leave settings as-is
    }
    const Preset& p = table[idx];
    {
        std::lock_guard<std::mutex> lock(state_.mtx);
        state_.ofdm.sample_rate  = p.sample_rate;
        state_.modem.sample_rate = p.sample_rate;
        state_.modem.center_freq = static_cast<float>(p.fc_hz);
        state_.modem.signal_bw   = static_cast<float>(p.bw_hz);
        state_.preset_modified   = true;
    }
    refreshFromState();
    emit configChanged();
}

} // namespace gw
