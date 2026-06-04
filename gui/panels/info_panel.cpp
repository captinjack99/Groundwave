/**
 * @file info_panel.cpp
 */
#include "info_panel.hpp"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFrame>
#include <cmath>

namespace dsca {

static QString fmtBitrate(float bps) {
    if (bps >= 1e6f) return QString("%1 Mbps").arg(bps / 1e6f, 0, 'f', 2);
    if (bps >= 1e3f) return QString("%1 kbps").arg(bps / 1e3f, 0, 'f', 1);
    return QString("%1 bps").arg(bps, 0, 'f', 0);
}

InfoPanel::InfoPanel(AppState& state, QWidget* parent)
    : QWidget(parent), state_(state)
{
    buildUi();
    refresh();
}

QLabel* InfoPanel::addRow(QLayout* layout, const QString& field) {
    auto* row  = new QHBoxLayout;
    row->setSpacing(8);
    auto* lbl  = new QLabel(field);
    lbl->setMinimumWidth(120);
    auto* val  = new QLabel("—");
    val->setObjectName("valueLabel");
    val->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    row->addWidget(lbl);
    row->addWidget(val, 1);
    static_cast<QVBoxLayout*>(layout)->addLayout(row);
    return val;
}

void InfoPanel::buildUi() {
    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(style::dim::PANEL_PADDING,
                              style::dim::PANEL_PADDING,
                              style::dim::PANEL_PADDING,
                              style::dim::PANEL_PADDING);
    root->setSpacing(style::dim::ITEM_SPACING);

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

    sec("OFDM");
    sub_spacing_  = addRow(root, "Subcarrier spacing");
    sym_duration_ = addRow(root, "Symbol duration");
    active_sc_    = addRow(root, "Active subcarriers");
    data_sc_      = addRow(root, "Data subcarriers");
    pilot_sc_     = addRow(root, "Pilot subcarriers");

    sep();
    sec("Throughput");
    gross_br_      = addRow(root, "Gross  (raw OFDM)");
    fec_br_        = addRow(root, "After FEC");
    net_br_        = addRow(root, "Net  (after RS)");
    spec_eff_      = addRow(root, "Spectral eff.");
    sig_bw_        = addRow(root, "OFDM-occupied BW");
    configured_bw_ = addRow(root, "Configured BW (LPF)");
    overheads_     = addRow(root, "Overheads");
    overheads_->setStyleSheet("color:#8E8E93; font-size:10px;");

    // Link Budget section removed — its hardcoded SNR table didn't track
    // FEC rate / CP / FFT changes and shadowed the proper LinkBudgetPanel
    // below it. The dedicated LinkBudgetPanel now owns this readout.

    // ---- Hierarchical modulation layer status ----
    sep();
    hier_section_title_ = new QLabel("HIERARCHICAL LAYERS");
    hier_section_title_->setObjectName("sectionTitle");
    root->addWidget(hier_section_title_);

    hier_badge_ = new QLabel("—");
    hier_badge_->setAlignment(Qt::AlignCenter);
    hier_badge_->setStyleSheet(
        "QLabel { padding:6px; border-radius:4px; background:#2C2C36; "
        "color:#8E8E93; font-weight:bold; }");
    hier_badge_->setToolTip(
        "Graceful-degradation status. STEREO = HP and LP both decoded; "
        "MONO FALLBACK = HP locked but LP failed (stream 1 dropped); "
        "LOSS = HP also failed.");
    root->addWidget(hier_badge_);

    hp_status_   = addRow(root, "HP (Mid) layer");
    lp_status_   = addRow(root, "LP (Side) layer");
    hp_frames_   = addRow(root, "HP frames OK/bad");
    lp_frames_   = addRow(root, "LP frames OK/bad");

    root->addStretch();
}

void InfoPanel::refresh() {
    ComputedParams cp;
    float current_snr;
    ModemStats st;
    float configured_bw_hz = 0.f;
    {
        std::lock_guard<std::mutex> lock(state_.mtx);
        cp          = state_.computedParams();
        current_snr = state_.stats.snr_db;
        st          = state_.stats;
        configured_bw_hz = state_.modem.signal_bw;
    }

    sub_spacing_->setText(QString("%1 Hz")
        .arg(cp.subcarrier_spacing_hz, 0, 'f', 1));
    sym_duration_->setText(QString("%1 ms")
        .arg(cp.symbol_duration_ms, 0, 'f', 2));
    active_sc_->setText(QString::number(cp.active_subcarriers));
    data_sc_->setText(QString::number(cp.data_subcarriers));
    pilot_sc_->setText(QString::number(cp.pilot_subcarriers));

    gross_br_->setText(fmtBitrate(cp.gross_bitrate_bps));
    fec_br_->setText(fmtBitrate(cp.fec_coded_bitrate_bps));
    net_br_->setText(fmtBitrate(cp.net_bitrate_bps));
    spec_eff_->setText(QString("%1 bps/Hz")
        .arg(cp.spectral_eff_bps_hz, 0, 'f', 2));
    sig_bw_->setText(QString("%1 kHz")
        .arg(cp.signal_bandwidth_hz / 1000.f, 0, 'f', 1));
    if (configured_bw_hz > 0.f) {
        configured_bw_->setText(QString("%1 kHz")
            .arg(configured_bw_hz / 1000.f, 0, 'f', 1));
    } else {
        configured_bw_->setText("auto");
    }
    overheads_->setText(QString("CP %1%  ·  Pilot %2%  ·  FEC %3%  ·  RS %4%")
        .arg(cp.cp_overhead_pct,    0, 'f', 1)
        .arg(cp.pilot_overhead_pct, 0, 'f', 1)
        .arg(cp.fec_overhead_pct,   0, 'f', 1)
        .arg(cp.rs_overhead_pct,    0, 'f', 1));
    (void)current_snr;

    // ---- Hierarchical-layer status ----
    if (!st.hier_active) {
        // Hide / dim the whole block when not running hierarchical mode.
        hier_badge_->setText("disabled");
        hier_badge_->setStyleSheet(
            "QLabel { padding:6px; border-radius:4px; background:#2C2C36; "
            "color:#8E8E93; font-weight:bold; }");
        hp_status_->setText("—");
        lp_status_->setText("—");
        hp_frames_->setText("—");
        lp_frames_->setText("—");
    } else {
        // Graceful-degradation badge: stereo / mono / loss.
        if (st.hp_locked && st.lp_locked) {
            hier_badge_->setText("STEREO  (HP + LP)");
            hier_badge_->setStyleSheet(
                "QLabel { padding:6px; border-radius:4px; "
                "background:#0A4F1E; color:#30D158; font-weight:bold; }");
        } else if (st.hp_locked) {
            hier_badge_->setText("MONO FALLBACK  (HP only)");
            hier_badge_->setStyleSheet(
                "QLabel { padding:6px; border-radius:4px; "
                "background:#4F3D0A; color:#FF9F0A; font-weight:bold; }");
        } else {
            hier_badge_->setText("LOSS  (HP failed)");
            hier_badge_->setStyleSheet(
                "QLabel { padding:6px; border-radius:4px; "
                "background:#4F0A0A; color:#FF453A; font-weight:bold; }");
        }
        hp_status_->setText(QString("%1   ·   |LLR| = %2")
            .arg(st.hp_locked ? "LOCKED" : "FAIL")
            .arg(st.hp_avg_llr_mag, 0, 'f', 2));
        lp_status_->setText(QString("%1   ·   |LLR| = %2")
            .arg(st.lp_locked ? "LOCKED" : "FAIL")
            .arg(st.lp_avg_llr_mag, 0, 'f', 2));
        hp_frames_->setText(QString("%1 / %2")
            .arg(st.hp_frames_ok).arg(st.hp_frames_bad));
        lp_frames_->setText(QString("%1 / %2")
            .arg(st.lp_frames_ok).arg(st.lp_frames_bad));
    }
}

} // namespace dsca
