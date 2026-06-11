/**
 * @file preset_dialog.cpp
 */
#include "preset_dialog.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QPushButton>
#include <QLabel>
#include <QFrame>
#include <cmath>

namespace gw {

static const char* modName(Modulation m) {
    switch (m) {
        case Modulation::BPSK:    return "BPSK";
        case Modulation::QPSK:    return "QPSK";
        case Modulation::QAM16:   return "QAM-16";
        case Modulation::QAM64:   return "QAM-64";
        case Modulation::QAM256:  return "QAM-256";
        case Modulation::QAM1024: return "QAM-1024";
        case Modulation::QAM4096: return "QAM-4096";
        default: return "?";
    }
}
static const char* fecName(FECRate r) {
    switch (r) {
        case FECRate::Rate_1_4: return "1/4"; case FECRate::Rate_1_3: return "1/3";
        case FECRate::Rate_2_5: return "2/5"; case FECRate::Rate_1_2: return "1/2";
        case FECRate::Rate_3_5: return "3/5"; case FECRate::Rate_2_3: return "2/3";
        case FECRate::Rate_3_4: return "3/4"; case FECRate::Rate_4_5: return "4/5";
        case FECRate::Rate_5_6: return "5/6"; case FECRate::Rate_8_9: return "8/9";
        case FECRate::Rate_9_10: return "9/10"; case FECRate::None: return "None";
        default: return "?";
    }
}

PresetDialog::PresetDialog(AppState& state, QWidget* parent)
    : QDialog(parent), state_(state)
{
    setWindowTitle("Preset Manager");
    setMinimumSize(560, 400);
    buildUi();
    refreshTable();
}

void PresetDialog::buildUi() {
    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(16, 16, 16, 16);
    root->setSpacing(10);

    auto* hint = new QLabel("Click to select  ·  Double-click to load  ·  Use controls below to save or rename");
    hint->setObjectName("sectionTitle");
    root->addWidget(hint);

    // Table
    table_ = new QTableWidget(static_cast<int>(NUM_PRESETS), 5, this);
    table_->setHorizontalHeaderLabels({"#", "Name", "Mod", "FEC", "Net rate"});
    table_->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
    table_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Fixed);
    table_->setColumnWidth(0, 28);
    table_->setColumnWidth(2, 72);
    table_->setColumnWidth(3, 52);
    table_->setColumnWidth(4, 90);
    table_->setSelectionBehavior(QAbstractItemView::SelectRows);
    table_->setSelectionMode(QAbstractItemView::SingleSelection);
    table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    table_->verticalHeader()->hide();
    table_->setShowGrid(false);
    table_->setAlternatingRowColors(false);
    table_->setStyleSheet(
        "QTableWidget { border:1px solid #1C1C24; border-radius:8px;"
        "  background:#101014; gridline-color:transparent; }"
        "QTableWidget::item { padding:6px 8px; color:#8E8E93; border:none; }"
        "QTableWidget::item:selected { background:#003366; color:#F2F2F7; }"
        "QHeaderView::section { background:#14141A; color:#48484E;"
        "  font-size:10px; font-weight:600; letter-spacing:0.06em;"
        "  border:none; border-bottom:1px solid #1C1C24; padding:5px 8px; }"
    );
    root->addWidget(table_);

    connect(table_, &QTableWidget::cellDoubleClicked,
            this,   &PresetDialog::onRowDoubleClicked);
    connect(table_->selectionModel(), &QItemSelectionModel::selectionChanged,
            this, &PresetDialog::onSelectionChanged);

    // Divider
    auto* sep = new QFrame;
    sep->setFrameShape(QFrame::HLine);
    sep->setStyleSheet("background:#1C1C28;max-height:1px;border:none;");
    root->addWidget(sep);

    // Controls row
    auto* ctrl = new QHBoxLayout;
    ctrl->setSpacing(8);

    auto* slot_lbl = new QLabel("Slot:");
    slot_spin_ = new QSpinBox;
    slot_spin_->setRange(0, static_cast<int>(NUM_PRESETS) - 1);
    slot_spin_->setValue(7);
    slot_spin_->setFixedWidth(52);

    auto* name_lbl = new QLabel("Name:");
    name_edit_ = new QLineEdit;
    name_edit_->setPlaceholderText("Preset name…");
    name_edit_->setMaxLength(63);

    auto* save_btn   = new QPushButton("Save Here");
    auto* rename_btn = new QPushButton("Rename");
    auto* load_btn   = new QPushButton("Load Selected");
    load_btn->setObjectName("accentBtn");

    for (auto* b : {save_btn, rename_btn, load_btn})
        b->setMinimumHeight(28);

    ctrl->addWidget(slot_lbl);
    ctrl->addWidget(slot_spin_);
    ctrl->addWidget(name_lbl);
    ctrl->addWidget(name_edit_, 1);
    ctrl->addWidget(save_btn);
    ctrl->addWidget(rename_btn);
    ctrl->addStretch();
    ctrl->addWidget(load_btn);
    root->addLayout(ctrl);

    // Close
    auto* close_btn = new QPushButton("Close");
    close_btn->setFixedWidth(100);
    auto* btn_row = new QHBoxLayout;
    btn_row->addStretch();
    btn_row->addWidget(close_btn);
    root->addLayout(btn_row);

    connect(save_btn,   &QPushButton::clicked, this, &PresetDialog::onSave);
    connect(rename_btn, &QPushButton::clicked, this, &PresetDialog::onRename);
    connect(load_btn,   &QPushButton::clicked, this, &PresetDialog::onLoad);
    connect(close_btn,  &QPushButton::clicked, this, &QDialog::accept);
}

void PresetDialog::refreshTable() {
    table_->blockSignals(true);
    for (int i = 0; i < static_cast<int>(NUM_PRESETS); ++i) {
        const auto& p = state_.presets[static_cast<size_t>(i)];
        bool active = (state_.active_preset_slot == i);

        auto mkItem = [&](const QString& text) {
            auto* item = new QTableWidgetItem(text);
            if (active) item->setForeground(QColor(style::C_SIGNAL));
            else if (!p.valid) item->setForeground(QColor(style::TEXT_TERTIARY));
            return item;
        };

        table_->setItem(i, 0, mkItem(QString::number(i)));
        table_->setItem(i, 1, mkItem(p.valid ? p.name : "(empty)"));

        if (p.valid) {
            table_->setItem(i, 2, mkItem(modName(p.ofdm.modulation)));
            table_->setItem(i, 3, mkItem(fecName(p.frame.fec_rate)));
            // Match InfoPanel's net rate: mirror the preset's signal_bw into
            // target_bw_hz and honour its RS-outer flag (else the column
            // diverges for non-default BW / RS-off presets). (#56)
            OFDMParams o = p.ofdm;
            o.target_bw_hz = p.modem.signal_bw;
            auto cp = ComputedParams::compute(o, p.frame, p.modem.enable_rs_outer);
            float bps = cp.net_bitrate_bps;
            QString br = bps >= 1e6f
                ? QString("%1 Mbps").arg(bps/1e6f, 0,'f',2)
                : QString("%1 kbps").arg(bps/1e3f, 0,'f',1);
            table_->setItem(i, 4, mkItem(br));
        } else {
            for (int c = 2; c <= 4; ++c)
                table_->setItem(i, c, mkItem("—"));
        }
        table_->setRowHeight(i, 32);
    }
    table_->blockSignals(false);
}

void PresetDialog::onRowDoubleClicked(int row, int /*col*/) {
    if (!state_.presets[static_cast<size_t>(row)].valid) return;
    {
        // applyPreset mutates ofdm/frame/modem, which the engine thread
        // reads under state_.mtx every tick — hold the lock for the write.
        std::lock_guard<std::mutex> lock(state_.mtx);
        state_.applyPreset(row);
    }
    refreshTable();
    emit presetLoaded();   // emitted OUTSIDE the lock (handler re-locks)
}

void PresetDialog::onSelectionChanged() {
    auto rows = table_->selectionModel()->selectedRows();
    if (rows.isEmpty()) return;
    int row = rows.first().row();
    slot_spin_->setValue(row);
    if (state_.presets[static_cast<size_t>(row)].valid)
        name_edit_->setText(state_.presets[static_cast<size_t>(row)].name);
}

void PresetDialog::onLoad() {
    auto rows = table_->selectionModel()->selectedRows();
    if (rows.isEmpty()) return;
    int row = rows.first().row();
    if (!state_.presets[static_cast<size_t>(row)].valid) return;
    {
        std::lock_guard<std::mutex> lock(state_.mtx);
        state_.applyPreset(row);
    }
    refreshTable();
    emit presetLoaded();   // emitted OUTSIDE the lock (handler re-locks)
}

void PresetDialog::onSave() {
    int slot = slot_spin_->value();
    if (slot < 0 || slot >= static_cast<int>(NUM_PRESETS)) return;
    QString name = name_edit_->text().trimmed();
    {
        // saveToPreset mutates presets[]/active_preset_slot/preset_modified —
        // all read by the engine thread under state_.mtx. Match onLoad's
        // locking discipline (this path previously wrote them unlocked).
        std::lock_guard<std::mutex> lock(state_.mtx);
        state_.saveToPreset(slot);
        if (!name.isEmpty())
            state_.presets[static_cast<size_t>(slot)].setName(name.toUtf8().constData());
    }
    refreshTable();
}

void PresetDialog::onRename() {
    int slot = slot_spin_->value();
    if (slot < 0 || slot >= static_cast<int>(NUM_PRESETS)) return;
    QString name = name_edit_->text().trimmed();
    {
        std::lock_guard<std::mutex> lock(state_.mtx);
        if (name.isEmpty() || !state_.presets[static_cast<size_t>(slot)].valid) return;
        state_.presets[static_cast<size_t>(slot)].setName(name.toUtf8().constData());
    }
    refreshTable();
}

} // namespace gw
