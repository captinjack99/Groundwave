/**
 * @file hierarchical_dialog.cpp
 */
#include "hierarchical_dialog.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QPushButton>
#include <QGroupBox>
#include <QSignalBlocker>
#include <QScrollArea>
#include <algorithm>

namespace gw {

HierarchicalDialog::HierarchicalDialog(const AudioEngineConfig& cfg,
                                        QWidget* parent)
    : QDialog(parent), cfg_(cfg)
{
    setWindowTitle("Hierarchical Modulation");
    // Small enough for a 768-px-class laptop at 125 % DPI; the content
    // scrolls, the Apply/Cancel row stays pinned (the old 720-px minimum
    // pushed the buttons off-screen on a modal dialog).
    setMinimumSize(560, 460);
    resize(640, 680);
    buildUi();

    // Initial population from cfg_. Block signals so the populating setters
    // don't fire the slots (which re-derive cfg_.hier from half-seeded
    // controls — e.g. landing on Custom before the custom base/HP are set,
    // corrupting the loaded split).
    {
        const QSignalBlocker b1(enable_cb_), b2(mode_combo_), b3(alpha_spin_),
                             b4(custom_base_combo_), b5(custom_hp_spin_);
        enable_cb_->setChecked(cfg_.hier.enabled);
        int mode_idx = static_cast<int>(cfg_.hier.mode);
        // The combo packs the enum value as itemData; locate by data.
        for (int i = 0; i < mode_combo_->count(); ++i) {
            if (mode_combo_->itemData(i).toInt() == mode_idx) {
                mode_combo_->setCurrentIndex(i);
                break;
            }
        }
        alpha_spin_->setValue(cfg_.hier.alpha);

        // Seed the Custom controls from cfg_ so opening a saved Custom config
        // shows the right base modulation + HP split (previously they kept
        // their build defaults, so a Custom config silently reverted).
        int total = static_cast<int>(cfg_.hier.hp_bits) + cfg_.hier.lp_bits;
        if (total > 0) {
            for (int i = 0; i < custom_base_combo_->count(); ++i) {
                auto m = static_cast<Modulation>(custom_base_combo_->itemData(i).toInt());
                if (static_cast<int>(bitsPerSymbol(m)) == total) {
                    custom_base_combo_->setCurrentIndex(i);
                    break;
                }
            }
            custom_hp_spin_->setRange(1, std::max(1, total - 1));
            if (cfg_.hier.hp_bits > 0)
                custom_hp_spin_->setValue(cfg_.hier.hp_bits);
        }
    }
    bool custom = (cfg_.hier.mode == HierarchicalMode::Custom);
    custom_base_combo_->setEnabled(custom);
    custom_hp_spin_->setEnabled(custom);
    refreshLabels();
}

void HierarchicalDialog::buildUi() {
    // Content lives on a scrollable page; only the button row is pinned.
    auto* outer = new QVBoxLayout(this);
    outer->setContentsMargins(0, 0, 0, 0);
    auto* page = new QWidget;
    auto* root = new QVBoxLayout(page);

    enable_cb_ = new QCheckBox("Enable hierarchical modulation");
    enable_cb_->setToolTip(
        "Master toggle. When OFF, the TX uses a uniform constellation "
        "and the entire payload rides a single protection layer (no "
        "graceful degradation). When ON, the constellation splits into "
        "HP and LP layers per the Mode setting below.");
    root->addWidget(enable_cb_);
    connect(enable_cb_, &QCheckBox::toggled,
            this, &HierarchicalDialog::onEnabledToggled);

    // ---- Mode selector ----
    auto* mode_group = new QGroupBox("Mode");
    auto* mode_layout = new QGridLayout(mode_group);
    mode_layout->addWidget(new QLabel("Constellation:"), 0, 0);
    mode_combo_ = new QComboBox;
    mode_combo_->setToolTip(
        "HP/LP layer pairing. Named presets use specific QAM "
        "constellations; Custom lets you set any base modulation and "
        "HP-bit count. Higher-order base = more total throughput; "
        "smaller HP-bit count = more robust HP layer.");
    auto add_mode = [&](HierarchicalMode m) {
        mode_combo_->addItem(QString::fromUtf8(hierarchicalModeName(m)),
                              static_cast<int>(m));
    };
    add_mode(HierarchicalMode::None);
    add_mode(HierarchicalMode::QPSK_QAM16);
    add_mode(HierarchicalMode::QPSK_QAM64);
    add_mode(HierarchicalMode::QAM16_QAM64);
    add_mode(HierarchicalMode::QPSK_QAM256);
    add_mode(HierarchicalMode::QAM16_QAM256);
    add_mode(HierarchicalMode::QAM64_QAM256);
    add_mode(HierarchicalMode::QPSK_QAM1024);
    add_mode(HierarchicalMode::QAM16_QAM1024);
    add_mode(HierarchicalMode::QAM64_QAM1024);
    add_mode(HierarchicalMode::QAM256_QAM1024);
    add_mode(HierarchicalMode::QPSK_QAM4096);
    add_mode(HierarchicalMode::QAM16_QAM4096);
    add_mode(HierarchicalMode::QAM64_QAM4096);
    add_mode(HierarchicalMode::QAM256_QAM4096);
    add_mode(HierarchicalMode::QAM1024_QAM4096);
    add_mode(HierarchicalMode::Custom);
    mode_layout->addWidget(mode_combo_, 0, 1);
    connect(mode_combo_,
            static_cast<void(QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
            this, &HierarchicalDialog::onModeChanged);

    // ---- Custom-mode controls (visible only when Mode = Custom) ----
    mode_layout->addWidget(new QLabel("Custom base:"), 1, 0);
    custom_base_combo_ = new QComboBox;
    custom_base_combo_->setToolTip(
        "Base modulation for Custom mode. The HP+LP bit count must "
        "equal this modulation's bits-per-symbol.");
    custom_base_combo_->addItem("16-QAM (4 bps)",   static_cast<int>(Modulation::QAM16));
    custom_base_combo_->addItem("64-QAM (6 bps)",   static_cast<int>(Modulation::QAM64));
    custom_base_combo_->addItem("256-QAM (8 bps)",  static_cast<int>(Modulation::QAM256));
    custom_base_combo_->addItem("1024-QAM (10 bps)", static_cast<int>(Modulation::QAM1024));
    custom_base_combo_->addItem("4096-QAM (12 bps)", static_cast<int>(Modulation::QAM4096));
    mode_layout->addWidget(custom_base_combo_, 1, 1);
    connect(custom_base_combo_,
            static_cast<void(QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
            this, &HierarchicalDialog::onCustomBaseChanged);

    mode_layout->addWidget(new QLabel("Custom HP bits:"), 2, 0);
    custom_hp_spin_ = new QSpinBox;
    custom_hp_spin_->setRange(1, 11);
    custom_hp_spin_->setValue(cfg_.hier.hp_bits > 0 ? cfg_.hier.hp_bits : 2);
    custom_hp_spin_->setToolTip(
        "Number of constellation bits assigned to the HP (robust) layer. "
        "LP gets the remaining bits. HP bits = 1 gives strongest "
        "graceful-degradation (large HP/LP threshold gap); HP bits = "
        "total-1 gives minimal hierarchy (LP is just one bit).");
    mode_layout->addWidget(custom_hp_spin_, 2, 1);
    connect(custom_hp_spin_,
            static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged),
            this, &HierarchicalDialog::onCustomHpChanged);

    root->addWidget(mode_group);

    // ---- Alpha (constellation HP/LP energy ratio) ----
    auto* alpha_group = new QGroupBox("Alpha (HP / LP distance ratio)");
    auto* alpha_layout = new QHBoxLayout(alpha_group);
    alpha_spin_ = new QDoubleSpinBox;
    alpha_spin_->setRange(1.0, 4.0);
    alpha_spin_->setSingleStep(0.1);
    alpha_spin_->setDecimals(2);
    alpha_spin_->setValue(cfg_.hier.alpha);
    alpha_spin_->setToolTip(
        "Ratio of inter-quadrant distance to intra-quadrant distance.\n"
        "α=1.0: uniform QAM (no hierarchy benefit).\n"
        "α=2.0: DVB-T default; ~6 dB HP/LP threshold gap.\n"
        "α=4.0: maximum HP protection; ~12 dB gap but LP needs much "
        "higher SNR to decode.");
    alpha_layout->addWidget(alpha_spin_);
    alpha_layout->addStretch();
    connect(alpha_spin_,
            static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
            this, &HierarchicalDialog::onAlphaChanged);
    root->addWidget(alpha_group);

    // ---- Visual signal-flow diagram (lives inside the dialog so the
    //      user can SEE M/S splitting, HP/LP layering, and how α
    //      reshapes the constellation as they tweak it) ----
    flow_widget_ = new HierFlowWidget;
    flow_widget_->setMinimumHeight(220);
    root->addWidget(flow_widget_);

    // ---- Info label (resolved HP/LP/total) ----
    info_label_ = new QLabel;
    info_label_->setStyleSheet("color:#8E8E93; padding:6px;");
    info_label_->setWordWrap(true);
    root->addWidget(info_label_);

    // ---- Assemble: scrollable page + pinned button row ----
    auto* scroll = new QScrollArea;
    scroll->setWidget(page);
    scroll->setWidgetResizable(true);
    scroll->setFrameShape(QFrame::NoFrame);
    outer->addWidget(scroll, 1);

    auto* button_row = new QHBoxLayout;
    button_row->setContentsMargins(12, 8, 12, 12);
    auto* apply_btn = new QPushButton("Apply");
    auto* cancel_btn = new QPushButton("Cancel");
    button_row->addStretch();
    button_row->addWidget(cancel_btn);
    button_row->addWidget(apply_btn);
    outer->addLayout(button_row);
    connect(apply_btn,  &QPushButton::clicked,
            this, &HierarchicalDialog::onApply);
    connect(cancel_btn, &QPushButton::clicked, this, &QDialog::reject);
}

void HierarchicalDialog::onEnabledToggled(bool on) {
    cfg_.hier.enabled = on;
    refreshLabels();
}

void HierarchicalDialog::onModeChanged(int idx) {
    if (idx < 0) return;
    auto mode = static_cast<HierarchicalMode>(mode_combo_->itemData(idx).toInt());
    cfg_.hier.mode = mode;
    bool custom = (mode == HierarchicalMode::Custom);
    custom_base_combo_->setEnabled(custom);
    custom_hp_spin_->setEnabled(custom);
    refreshLabels();
}

void HierarchicalDialog::onAlphaChanged(double v) {
    cfg_.hier.alpha = static_cast<float>(v);
    refreshLabels();
}

void HierarchicalDialog::onCustomBaseChanged(int /*idx*/) {
    auto mod = static_cast<Modulation>(custom_base_combo_->currentData().toInt());
    uint8_t total = bitsPerSymbol(mod);
    // Clamp HP spinner range and current value to [1, total-1]
    custom_hp_spin_->setRange(1, std::max<int>(1, total - 1));
    if (custom_hp_spin_->value() >= total) {
        custom_hp_spin_->setValue(std::max<int>(1, total - 1));
    }
    cfg_.hier.hp_bits = static_cast<uint8_t>(custom_hp_spin_->value());
    cfg_.hier.lp_bits = static_cast<uint8_t>(total - cfg_.hier.hp_bits);
    refreshLabels();
}

void HierarchicalDialog::onCustomHpChanged(int v) {
    auto mod = static_cast<Modulation>(custom_base_combo_->currentData().toInt());
    int total = static_cast<int>(bitsPerSymbol(mod));
    // Keep the invariant hp_bits + lp_bits == total. Clamping hp (rather than
    // forcing lp to 1 while hp keeps an out-of-range value) means the mapper
    // never sees hp+lp != base bps.
    int hp = std::clamp(v, 1, std::max(1, total - 1));
    cfg_.hier.hp_bits = static_cast<uint8_t>(hp);
    cfg_.hier.lp_bits = static_cast<uint8_t>(total - hp);
    refreshLabels();
}

void HierarchicalDialog::onApply() {
    accept();
}

void HierarchicalDialog::refreshLabels() {
    uint8_t hp = cfg_.hier.effectiveHP();
    uint8_t lp = cfg_.hier.effectiveLP();
    uint8_t total = hp + lp;
    bool symmetric = (hp == lp) && hp > 0;
    QString info;
    if (!cfg_.hier.enabled) {
        info = "Disabled — uniform modulation, single protection layer.";
    } else {
        info = QString("HP = %1 bps (%2)   ·   LP = %3 bps (%4)   ·   "
                       "Total = %5 bps (%6)   ·   α = %7")
                   .arg(hp).arg(QString::fromUtf8(hierLayerName(hp)))
                   .arg(lp).arg(QString::fromUtf8(hierLayerName(lp)))
                   .arg(total).arg(QString::fromUtf8(hierLayerName(total)))
                   .arg(cfg_.hier.alpha, 0, 'f', 2);
        if (symmetric) {
            info += "\nSymmetric split → M/S parallel chain (stream 0 ⇒ Mid/HP, "
                    "stream 1 ⇒ Side/LP). Graceful stereo→mono degradation "
                    "with LDPC-LLR-confidence crossfade enabled.";
        } else {
            info += "\nAsymmetric split → single-codeword bit-split mode. "
                    "HP and LP share one codeword; LP doesn't get its own "
                    "predictive recovery on failure.";
        }
    }
    info_label_->setText(info);
    if (flow_widget_) flow_widget_->setConfig(cfg_.hier);
}

} // namespace gw
