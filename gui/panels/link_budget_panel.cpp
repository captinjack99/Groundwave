/**
 * @file link_budget_panel.cpp
 */
#include "link_budget_panel.hpp"
#include "../style.hpp"
#include <QVBoxLayout>
#include <QLabel>
#include <QFrame>
#include <QSignalBlocker>

namespace gw {

namespace {
inline float wattsToDbm(float w) {
    if (w <= 0.f) return -120.f;
    return 10.f * std::log10(w * 1000.f);
}
} // anonymous

LinkBudgetPanel::LinkBudgetPanel(AppState& state, QWidget* parent)
    : QWidget(parent), state_(state)
{
    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(style::dim::PANEL_PADDING,
                              style::dim::PANEL_PADDING,
                              style::dim::PANEL_PADDING,
                              style::dim::PANEL_PADDING);
    root->setSpacing(style::dim::ITEM_SPACING);

    auto* title = new QLabel("LINK BUDGET");
    title->setObjectName("sectionTitle");
    root->addWidget(title);

    auto* in_grid = new QGridLayout();
    in_grid->setHorizontalSpacing(6);
    in_grid->setVerticalSpacing(3);
    int row = 0;

    auto addRow = [&](const QString& label, QWidget* w) {
        auto* l = new QLabel(label);
        l->setMinimumWidth(80);
        in_grid->addWidget(l, row, 0);
        in_grid->addWidget(w, row, 1);
        ++row;
    };

    tx_power_w_ = new QDoubleSpinBox();
    tx_power_w_->setRange(0.001, 100000.0);
    tx_power_w_->setDecimals(3);
    tx_power_w_->setValue(1.0);
    tx_power_w_->setSuffix(" W");
    tx_power_w_->setToolTip(
        "Transmitter output power at the antenna port (before antenna "
        "gain). For an FM SCA insertion this is your exciter's effective "
        "TX power on the SCA channel, not the main carrier.");
    addRow("TX Power", tx_power_w_);

    tx_gain_db_ = new QDoubleSpinBox();
    tx_gain_db_->setRange(-30.0, 30.0);
    tx_gain_db_->setDecimals(1);
    tx_gain_db_->setValue(0.0);
    tx_gain_db_->setSuffix(" dBi");
    tx_gain_db_->setToolTip(
        "TX antenna gain in dBi (referenced to isotropic). 0 dBi is a "
        "good default for an omnidirectional whip; +6 to +10 dBi for a "
        "Yagi or panel directional antenna.");
    addRow("TX Ant Gain", tx_gain_db_);

    rx_gain_db_ = new QDoubleSpinBox();
    rx_gain_db_->setRange(-30.0, 30.0);
    rx_gain_db_->setDecimals(1);
    rx_gain_db_->setValue(0.0);
    rx_gain_db_->setSuffix(" dBi");
    rx_gain_db_->setToolTip(
        "RX antenna gain in dBi. Same scale as TX. For a fixed receiver "
        "(home base) use the directional antenna gain; for portable use "
        "0 dBi.");
    addRow("RX Ant Gain", rx_gain_db_);

    cable_loss_db_ = new QDoubleSpinBox();
    cable_loss_db_->setRange(0.0, 20.0);
    cable_loss_db_->setDecimals(1);
    cable_loss_db_->setValue(2.0);
    cable_loss_db_->setSuffix(" dB");
    cable_loss_db_->setToolTip(
        "Total feedline + connector loss, TX and RX combined. Typical "
        "1–3 dB for short low-loss coax runs; more for long or lossy "
        "cable at higher frequencies. (Was a hidden 2 dB constant.)");
    addRow("Cable Loss", cable_loss_db_);

    freq_mhz_ = new QDoubleSpinBox();
    freq_mhz_->setRange(30.0, 6000.0);
    freq_mhz_->setDecimals(1);
    freq_mhz_->setValue(98.0);
    freq_mhz_->setSuffix(" MHz");
    freq_mhz_->setToolTip(
        "Carrier frequency in MHz (the FM channel center, not the SCA "
        "offset). Used in the path-loss model: free-space and Hata both "
        "scale with frequency.");
    addRow("Frequency", freq_mhz_);

    tx_height_m_ = new QDoubleSpinBox();
    tx_height_m_->setRange(1.0, 500.0);
    tx_height_m_->setValue(30.0);
    tx_height_m_->setSuffix(" m");
    tx_height_m_->setToolTip(
        "TX antenna height above local terrain (Hata model). 30 m is "
        "typical for a broadcast tower; raise this for hilltop sites. "
        "Ignored when Terrain = Free Space.");
    addRow("TX Height", tx_height_m_);

    rx_height_m_ = new QDoubleSpinBox();
    rx_height_m_->setRange(1.0, 50.0);
    rx_height_m_->setValue(2.0);
    rx_height_m_->setSuffix(" m");
    rx_height_m_->setToolTip(
        "RX antenna height above local terrain. 2 m is typical for a "
        "car or rooftop dipole; 10 m for a fixed home antenna.");
    addRow("RX Height", rx_height_m_);

    terrain_ = new QComboBox();
    terrain_->addItem("Free Space");
    terrain_->addItem("Open / Rural");
    terrain_->addItem("Suburban");
    terrain_->addItem("Urban");
    terrain_->addItem("Dense Urban");
    terrain_->setCurrentIndex(2);
    terrain_->setToolTip(
        "Path-loss propagation model. Free Space = ideal line-of-sight "
        "(no obstructions). Open/Suburban/Urban/Dense use Hata-Okumura "
        "with category-specific correction factors — Hata is well-"
        "validated for 30 MHz to 1.5 GHz.");
    addRow("Terrain", terrain_);

    nf_db_ = new QDoubleSpinBox();
    nf_db_->setRange(0.5, 20.0);
    nf_db_->setValue(8.0);
    nf_db_->setSuffix(" dB");
    nf_db_->setToolTip(
        "RX noise figure in dB above thermal (kT). Typical: 3 dB for "
        "an LNA-front-end receiver, 6–10 dB for a consumer SDR or "
        "FM tuner, 15+ dB for an unfiltered direct-coupled rig.");
    addRow("RX NF", nf_db_);

    margin_db_ = new QDoubleSpinBox();
    margin_db_->setRange(0.0, 30.0);
    margin_db_->setValue(3.0);
    margin_db_->setSuffix(" dB");
    margin_db_->setToolTip(
        "Extra SNR cushion above the FEC threshold required for "
        "reliable operation. 3 dB is a common rule-of-thumb that "
        "absorbs Rayleigh fading + AGC/sync residual losses. Increase "
        "to 6–10 dB for mobile or fast-fading environments.");
    addRow("Link Margin", margin_db_);

    root->addLayout(in_grid);

    // Divider
    auto* div = new QFrame();
    div->setFrameShape(QFrame::HLine);
    div->setStyleSheet("color:#2C2C38;");
    root->addWidget(div);

    // Outputs
    auto* out_grid = new QGridLayout();
    out_grid->setHorizontalSpacing(6);
    out_grid->setVerticalSpacing(2);
    int orow = 0;
    auto addOut = [&](const QString& label, QLabel*& target) {
        auto* l = new QLabel(label);
        l->setStyleSheet("color:#8E8E93; font-size:11px;");
        target = new QLabel("—");
        target->setObjectName("valueLabel");
        target->setStyleSheet("color:#F2F2F7; font-size:12px;");
        out_grid->addWidget(l,      orow, 0);
        out_grid->addWidget(target, orow, 1);
        ++orow;
    };
    addOut("ModCod",       out_modcod_);
    addOut("Es/N0 req",    out_threshold_);
    addOut("Eb/N0 req",    out_threshold_eb_);
    addOut("Coding gain",  out_coding_gain_);
    addOut("Path Loss",    out_path_loss_);
    addOut("Max Range",    out_max_range_);
    addOut("Net Bitrate",  out_net_bitrate_);
    addOut("Spec. Eff.",   out_spectral_eff_);
    addOut("Margin @ 1km", out_margin_at_1km_);
    addOut("Min Pr",       out_required_pr_);
    root->addLayout(out_grid);

    // ---- Hierarchical-mode section (visible only when hier is on) ----
    hier_section_title_ = new QLabel("HIERARCHICAL LAYERS");
    hier_section_title_->setObjectName("sectionTitle");
    hier_section_title_->setStyleSheet(
        "QLabel { color:#48484E; font-size:10px; font-weight:600; "
        "letter-spacing:0.08em; padding-top:8px; }");
    root->addWidget(hier_section_title_);

    auto* hier_grid = new QGridLayout();
    hier_grid->setHorizontalSpacing(6);
    hier_grid->setVerticalSpacing(2);
    int hrow = 0;
    auto addHier = [&](const QString& label, QLabel*& target) {
        auto* l = new QLabel(label);
        l->setStyleSheet("color:#8E8E93; font-size:11px;");
        target = new QLabel("—");
        target->setObjectName("valueLabel");
        target->setStyleSheet("color:#F2F2F7; font-size:12px;");
        hier_grid->addWidget(l,      hrow, 0);
        hier_grid->addWidget(target, hrow, 1);
        ++hrow;
    };
    addHier("HP threshold", out_hp_threshold_);
    addHier("LP threshold", out_lp_threshold_);
    addHier("HP/LP gap",    out_hp_lp_gap_);
    root->addLayout(hier_grid);

    root->addStretch();

    // Seed the input widgets from persisted state BEFORE wiring the change
    // triggers, so seeding doesn't spuriously fire write-back/recompute.
    seedInputsFromState();

    // Re-compute on any input change. First mirror the current widget values
    // back into AppState so they persist (Save Config) and survive restart.
    auto trigger = [this]() {
        {
            std::lock_guard<std::mutex> lock(state_.mtx);
            state_.link_budget.tx_power_w   = static_cast<float>(tx_power_w_->value());
            state_.link_budget.tx_gain_db   = static_cast<float>(tx_gain_db_->value());
            state_.link_budget.rx_gain_db   = static_cast<float>(rx_gain_db_->value());
            state_.link_budget.cable_loss_db = static_cast<float>(cable_loss_db_->value());
            state_.link_budget.freq_mhz     = static_cast<float>(freq_mhz_->value());
            state_.link_budget.tx_height_m  = static_cast<float>(tx_height_m_->value());
            state_.link_budget.rx_height_m  = static_cast<float>(rx_height_m_->value());
            state_.link_budget.terrain_idx  = terrain_->currentIndex();
            state_.link_budget.nf_db        = static_cast<float>(nf_db_->value());
            state_.link_budget.margin_db    = static_cast<float>(margin_db_->value());
        }
        recompute();
    };
    connect(tx_power_w_,   QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, trigger);
    connect(tx_gain_db_,   QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, trigger);
    connect(rx_gain_db_,   QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, trigger);
    connect(cable_loss_db_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, trigger);
    connect(freq_mhz_,     QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, trigger);
    connect(tx_height_m_,  QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, trigger);
    connect(rx_height_m_,  QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, trigger);
    connect(terrain_,      QOverload<int>::of(&QComboBox::currentIndexChanged), this, trigger);
    connect(nf_db_,        QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, trigger);
    connect(margin_db_,    QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, trigger);

    recompute();
}

void LinkBudgetPanel::seedInputsFromState() {
    // Snapshot the persisted inputs under the lock, then drive the widgets.
    LinkBudgetInputs lb;
    {
        std::lock_guard<std::mutex> lock(state_.mtx);
        lb = state_.link_budget;
    }
    // Block signals so seeding doesn't fire the change trigger (which would
    // write the values straight back and recompute mid-construction).
    const QSignalBlocker b0(tx_power_w_);
    const QSignalBlocker b1(tx_gain_db_);
    const QSignalBlocker b2(rx_gain_db_);
    const QSignalBlocker b3(freq_mhz_);
    const QSignalBlocker b4(tx_height_m_);
    const QSignalBlocker b5(rx_height_m_);
    const QSignalBlocker b6(terrain_);
    const QSignalBlocker b7(nf_db_);
    const QSignalBlocker b8(margin_db_);
    const QSignalBlocker b9(cable_loss_db_);
    tx_power_w_->setValue(lb.tx_power_w);
    tx_gain_db_->setValue(lb.tx_gain_db);
    rx_gain_db_->setValue(lb.rx_gain_db);
    cable_loss_db_->setValue(lb.cable_loss_db);
    freq_mhz_->setValue(lb.freq_mhz);
    tx_height_m_->setValue(lb.tx_height_m);
    rx_height_m_->setValue(lb.rx_height_m);
    int ti = (lb.terrain_idx >= 0 && lb.terrain_idx < terrain_->count())
                 ? lb.terrain_idx : 2;
    terrain_->setCurrentIndex(ti);
    nf_db_->setValue(lb.nf_db);
    margin_db_->setValue(lb.margin_db);
}

void LinkBudgetPanel::refreshFromState() {
    seedInputsFromState();
    recompute();
}

void LinkBudgetPanel::recompute() {
    Modulation mod;
    FECRate    fec;
    HierarchicalConfig hier;
    {
        std::lock_guard<std::mutex> lock(state_.mtx);
        mod  = state_.ofdm.modulation;
        fec  = state_.frame.fec_rate;
        hier = state_.hier;
    }
    // Resolve effective HP/LP modulations when hierarchical mod is on.
    // HP layer carries `hp_bps` bits per subcarrier → equivalent uniform
    // modulation. Same for LP. These are used to compute SEPARATE link
    // budget thresholds for each layer below.
    bool hier_on = hier.enabled && hier.effectiveHP() > 0 && hier.effectiveLP() > 0;
    auto bpsToMod = [](uint8_t bps) -> Modulation {
        switch (bps) {
            case 1:  return Modulation::BPSK;
            case 2:  return Modulation::QPSK;
            case 4:  return Modulation::QAM16;
            case 6:  return Modulation::QAM64;
            case 8:  return Modulation::QAM256;
            case 10: return Modulation::QAM1024;
            case 12: return Modulation::QAM4096;
            default: return Modulation::QPSK;
        }
    };
    Modulation hp_mod = bpsToMod(hier.effectiveHP());
    Modulation lp_mod = bpsToMod(hier.effectiveLP());
    // For the rest of recompute, when hier is on, evaluate against the
    // HIGHER-threshold layer (LP) so the link budget reflects the
    // worst-case requirement for full stereo decode. The HP layer's
    // threshold appears in the dedicated readout below.
    if (hier_on) mod = lp_mod;

    // Pure-FEC Es/N0 threshold (Shannon + LDPC implementation loss for the
    // chosen modulation/code combo).
    auto t = computeThreshold(mod, fec);

    // OFDM system-level penalties that the pure-FEC threshold doesn't see.
    // These reflect real-modem losses: channel-estimation noise from pilot
    // averaging, residual carrier/sync error, CP energy overhead, and
    // phase-noise sensitivity that grows with QAM order.
    OFDMParams ofdm_snap;
    {
        std::lock_guard<std::mutex> lock(state_.mtx);
        ofdm_snap = state_.ofdm;
    }
    float pilot_loss   = 0.5f;   // pilot-based channel estimate noise
    float sync_loss    = 0.3f;   // residual timing / fractional CFO
    // CP energy is sent but carries no info; treat as a SE penalty.
    // CP fraction → effective SE drops by N/(N+CP) → log-domain ≈ this dB:
    float cp_frac = cpRatio(ofdm_snap.cyclic_prefix);  // 1/4 .. 1/32
    float cp_loss  = -10.f * std::log10(1.f / (1.f + cp_frac));
    // Phase-noise + I/Q-imbalance sensitivity ramps with constellation order.
    // Empirically: 0 dB for BPSK/QPSK, 0.5 for QAM16, 1.0 for QAM64,
    //              1.5 for QAM256, 2.0 for QAM1024, 2.5 for QAM4096.
    float qam_loss   = 0.f;
    switch (mod) {
        case Modulation::QAM16:   qam_loss = 0.5f; break;
        case Modulation::QAM64:   qam_loss = 1.0f; break;
        case Modulation::QAM256:  qam_loss = 1.5f; break;
        case Modulation::QAM1024: qam_loss = 2.0f; break;
        case Modulation::QAM4096: qam_loss = 2.5f; break;
        default: qam_loss = 0.f;
    }
    // Smaller FFTs → coarser sync and channel estimate granularity.
    // Below 256 add a small margin; above 1024 subtract.
    float fft_loss = 0.f;
    if (ofdm_snap.fft_size <= 64)       fft_loss =  0.7f;
    else if (ofdm_snap.fft_size <= 128) fft_loss =  0.4f;
    else if (ofdm_snap.fft_size <= 256) fft_loss =  0.2f;
    else if (ofdm_snap.fft_size >= 2048) fft_loss = -0.2f;

    float total_threshold_db = t.threshold_db
                             + pilot_loss + sync_loss + cp_loss
                             + qam_loss + fft_loss;

    // ModCod readout: when hier is on, show both layers stacked.
    if (hier_on) {
        out_modcod_->setText(QString("HP: %1 / %2     LP: %3 / %2")
            .arg(modulationName(hp_mod))
            .arg(fecRateName(fec))
            .arg(modulationName(lp_mod)));
    } else {
        out_modcod_->setText(QString("%1 / %2")
            .arg(modulationName(mod))
            .arg(fecRateName(fec)));
    }
    out_threshold_->setText(QString("%1 dB (FEC %2 + OFDM %3)")
        .arg(total_threshold_db, 0, 'f', 1)
        .arg(t.threshold_db,     0, 'f', 1)
        .arg(total_threshold_db - t.threshold_db, 0, 'f', 1));

    // Eb/N0 derived from the FULL system threshold, not just pure FEC.
    float se = std::max(t.spectral_eff, 1e-6f);
    float eb_n0 = total_threshold_db - 10.f * std::log10(se);
    out_threshold_eb_->setText(QString("%1 dB").arg(eb_n0, 0, 'f', 1));

    // Coding gain: same modulation, rate 9/10 baseline → current rate.
    auto baseline = computeThreshold(mod, FECRate::Rate_9_10);
    float gain = baseline.threshold_db - t.threshold_db;
    out_coding_gain_->setText(QString("%1 dB vs 9/10")
                                .arg(gain, 0, 'f', 1));
    out_spectral_eff_->setText(QString("%1 b/Hz").arg(t.spectral_eff, 0, 'f', 2));

    // Override `t.threshold_db` for the rest of the function so the link
    // budget (path loss, range, required Pr) uses the realistic total.
    t.threshold_db = total_threshold_db;

    // Required RX power: P_r = N0·B + threshold + margin
    // Use the user's actual signal bandwidth (not a fixed 4 kHz) so the
    // noise floor and required-Pr readouts reflect the real channel.
    float bw_hz;
    {
        std::lock_guard<std::mutex> lock(state_.mtx);
        bw_hz = state_.modem.signal_bw > 0.f
                  ? state_.modem.signal_bw
                  : static_cast<float>(state_.ofdm.sample_rate) * 0.4f;
    }
    if (bw_hz < 1.f) bw_hz = 1.f;
    const float k_dbm = -174.f;        // kT thermal noise floor in dBm/Hz
    float noise_dbm = k_dbm
                    + 10.f * std::log10(bw_hz)
                    + static_cast<float>(nf_db_->value());
    float required_pr_dbm = noise_dbm
                          + t.threshold_db
                          + static_cast<float>(margin_db_->value());
    out_required_pr_->setText(QString("%1 dBm")
                                .arg(required_pr_dbm, 0, 'f', 1));

    // Path loss budget = TX_dBm + TX_gain - cable_loss + RX_gain - required_Pr
    float tx_dbm = wattsToDbm(static_cast<float>(tx_power_w_->value()));
    float cable_loss = static_cast<float>(cable_loss_db_->value());
    float pl_budget = tx_dbm + tx_gain_db_->value() - cable_loss
                    + rx_gain_db_->value() - required_pr_dbm;

    // Map terrain combo → enum
    TerrainType ter = TerrainType::Suburban;
    switch (terrain_->currentIndex()) {
        case 0: ter = TerrainType::FreeSpace;  break;
        case 1: ter = TerrainType::OpenRural;  break;
        case 2: ter = TerrainType::Suburban;   break;
        case 3: ter = TerrainType::Urban;      break;
        case 4: ter = TerrainType::DenseUrban; break;
    }

    float freq    = static_cast<float>(freq_mhz_->value());
    float hb      = static_cast<float>(tx_height_m_->value());
    float hm      = static_cast<float>(rx_height_m_->value());

    auto pathLoss = [&](float d_km) -> float {
        if (ter == TerrainType::FreeSpace) return freeSpacePathLoss(d_km, freq);
        return hataPathLoss(d_km, freq, hb, hm, ter);
    };

    // Margin at 1 km
    float pl_1km = pathLoss(1.f);
    out_path_loss_->setText(QString("%1 dB @ 1 km").arg(pl_1km, 0, 'f', 1));
    float margin_1km = pl_budget - pl_1km;
    out_margin_at_1km_->setText(QString("%1 dB").arg(margin_1km, 0, 'f', 1));

    // Binary search for max range that exhausts the budget
    float lo = 0.05f, hi = 500.f;
    for (int i = 0; i < 40; ++i) {
        float mid = 0.5f * (lo + hi);
        if (pathLoss(mid) <= pl_budget) lo = mid;
        else                            hi = mid;
    }
    out_max_range_->setText(QString("%1 km").arg(lo, 0, 'f', 1));

    // Net bitrate: use the SAME calculation the InfoPanel does so the
    // two readouts track each other exactly. ComputedParams accounts
    // for OFDM CP overhead, pilot subcarriers, FEC code rate, and
    // (optionally) Reed-Solomon outer-code parity.
    ComputedParams cp;
    {
        std::lock_guard<std::mutex> lock(state_.mtx);
        cp = state_.computedParams();
    }
    float bitrate = cp.net_bitrate_bps;
    QString bps_str = (bitrate >= 1.0e6f)
        ? QString("%1 Mbps").arg(bitrate / 1.0e6f, 0, 'f', 2)
        : QString("%1 kbps").arg(bitrate / 1.0e3f, 0, 'f', 1);
    out_net_bitrate_->setText(bps_str);

    // ---- Hierarchical-layer thresholds ----
    if (hier_on) {
        hier_section_title_->setVisible(true);
        out_hp_threshold_->setVisible(true);
        out_lp_threshold_->setVisible(true);
        out_hp_lp_gap_->setVisible(true);

        // HP layer threshold: equivalent uniform modulation at the same
        // FEC rate, plus the same OFDM overhead penalties. The HP layer
        // is robust BECAUSE its constellation is sparser; the α factor
        // shifts thresholds further apart.
        auto hp_t = computeThreshold(hp_mod, fec);
        auto lp_t = computeThreshold(lp_mod, fec);
        float hp_thr = hp_t.threshold_db + pilot_loss + sync_loss + cp_loss + fft_loss;
        float lp_thr = lp_t.threshold_db + pilot_loss + sync_loss + cp_loss + fft_loss;
        // Alpha shifts the gap: gap ≈ 20·log10(α + 1) − 6 (empirical
        // approximation around α=2). Apply as additional LP penalty
        // referenced to α=2 → 0 dB.
        float alpha_gap_db = 20.f * std::log10(hier.alpha + 1.f)
                           - 20.f * std::log10(3.f);   // baseline at α=2
        lp_thr += std::max(0.f, alpha_gap_db);
        hp_thr -= std::max(0.f, alpha_gap_db) * 0.25f;  // small HP benefit

        out_hp_threshold_->setText(QString("%1 dB Es/N0").arg(hp_thr, 0, 'f', 1));
        out_lp_threshold_->setText(QString("%1 dB Es/N0").arg(lp_thr, 0, 'f', 1));
        out_hp_lp_gap_->setText(QString("%1 dB (α=%2)")
                                    .arg(lp_thr - hp_thr, 0, 'f', 1)
                                    .arg(hier.alpha, 0, 'f', 2));
    } else {
        hier_section_title_->setVisible(false);
        out_hp_threshold_->setVisible(false);
        out_lp_threshold_->setVisible(false);
        out_hp_lp_gap_->setVisible(false);
    }
}

} // namespace gw
