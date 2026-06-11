/**
 * @file device_dialog.cpp
 * @brief Hardware audio device selection dialog implementation
 */
#include "device_dialog.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QFormLayout>

namespace gw {

DeviceDialog::DeviceDialog(QWidget* parent)
    : QDialog(parent)
{
    setWindowTitle("Audio Device Selection");
    setFixedSize(420, 320);
    buildUi();
    enumerateDevices();
}

void DeviceDialog::buildUi() {
    auto* main_layout = new QVBoxLayout(this);
    main_layout->setContentsMargins(20, 20, 20, 20);
    main_layout->setSpacing(12);

    // Enable checkbox
    hw_enable_check_ = new QCheckBox("Enable hardware audio I/O");
    hw_enable_check_->setStyleSheet(
        "QCheckBox { color: #F2F2F7; font-size: 13px; }"
        "QCheckBox::indicator { width: 16px; height: 16px; }");
    main_layout->addWidget(hw_enable_check_);

    // Device group
    auto* dev_group = new QGroupBox("Devices");
    dev_group->setStyleSheet(
        "QGroupBox { color: #8E8E93; font-size: 12px; "
        "border: 1px solid #2C2C34; border-radius: 6px; "
        "margin-top: 8px; padding-top: 16px; }"
        "QGroupBox::title { subcontrol-origin: margin; left: 12px; }");
    auto* form = new QFormLayout(dev_group);
    form->setContentsMargins(12, 12, 12, 12);
    form->setSpacing(8);

    auto combo_style = QString(
        "QComboBox { background: #1C1C24; color: #F2F2F7; border: 1px solid #2C2C34; "
        "border-radius: 4px; padding: 4px 8px; font-size: 12px; }"
        "QComboBox:disabled { color: #48484E; }"
        "QComboBox::drop-down { border: none; }"
        "QComboBox QAbstractItemView { background: #1C1C24; color: #F2F2F7; "
        "selection-background-color: #0A84FF; }");

    auto label_style = QString("color: #AEAEB2; font-size: 12px;");

    pb_combo_ = new QComboBox;
    pb_combo_->setStyleSheet(combo_style);
    pb_combo_->setMinimumWidth(260);
    auto* pb_label = new QLabel("Playback:");
    pb_label->setStyleSheet(label_style);
    form->addRow(pb_label, pb_combo_);

    cap_combo_ = new QComboBox;
    cap_combo_->setStyleSheet(combo_style);
    cap_combo_->setMinimumWidth(260);
    auto* cap_label = new QLabel("Capture:");
    cap_label->setStyleSheet(label_style);
    form->addRow(cap_label, cap_combo_);

    main_layout->addWidget(dev_group);

    // Status
    status_label_ = new QLabel;
    status_label_->setStyleSheet("color: #8E8E93; font-size: 11px;");
    status_label_->setWordWrap(true);
    main_layout->addWidget(status_label_);

    main_layout->addStretch();

    // Buttons
    auto* btn_layout = new QHBoxLayout;
    btn_layout->addStretch();

    cancel_btn_ = new QPushButton("Cancel");
    cancel_btn_->setFixedWidth(80);
    cancel_btn_->setStyleSheet(
        "QPushButton { background: #2C2C34; color: #F2F2F7; "
        "border-radius: 4px; padding: 6px; font-size: 12px; }"
        "QPushButton:hover { background: #3A3A44; }");
    connect(cancel_btn_, &QPushButton::clicked, this, &QDialog::reject);

    ok_btn_ = new QPushButton("Apply");
    ok_btn_->setFixedWidth(80);
    ok_btn_->setStyleSheet(
        "QPushButton { background: #0A84FF; color: #FFFFFF; "
        "border-radius: 4px; padding: 6px; font-size: 12px; font-weight: 500; }"
        "QPushButton:hover { background: #409CFF; }");
    connect(ok_btn_, &QPushButton::clicked, this, &DeviceDialog::onAccept);

    btn_layout->addWidget(cancel_btn_);
    btn_layout->addWidget(ok_btn_);
    main_layout->addLayout(btn_layout);

    // Wire enable checkbox to combos
    connect(hw_enable_check_, &QCheckBox::toggled, [this](bool on) {
        pb_combo_->setEnabled(on);
        cap_combo_->setEnabled(on);
    });
    pb_combo_->setEnabled(false);
    cap_combo_->setEnabled(false);
}

void DeviceDialog::enumerateDevices() {
    pb_combo_->clear();
    cap_combo_->clear();
    pb_combo_->addItem("System Default", -1);
    cap_combo_->addItem("System Default", -1);

    // Try to enumerate hardware devices
    HWAudioDevice hw;
    if (hw.init()) {
        pb_devices_  = hw.playbackDevices();
        cap_devices_ = hw.captureDevices();

        for (const auto& d : pb_devices_) {
            QString name = QString::fromStdString(d.name);
            if (d.is_default) name += " (default)";
            pb_combo_->addItem(name, static_cast<int>(d.id));
        }
        for (const auto& d : cap_devices_) {
            QString name = QString::fromStdString(d.name);
            if (d.is_default) name += " (default)";
            cap_combo_->addItem(name, static_cast<int>(d.id));
        }

        status_label_->setText(
            QString("Found %1 playback and %2 capture device(s).")
                .arg(pb_devices_.size())
                .arg(cap_devices_.size()));
    } else {
        status_label_->setText(
            "Hardware audio not available. Build with -DGW_ENABLE_AUDIO=ON "
            "to enable real soundcard I/O.");
        hw_enable_check_->setEnabled(false);
    }
}

void DeviceDialog::setCurrentConfig(int playback_device, int capture_device,
                                    bool hw_enabled) {
    init_pb_      = playback_device;
    init_cap_     = capture_device;
    init_enabled_ = hw_enabled;

    hw_enable_check_->setChecked(hw_enabled);
    pb_combo_->setEnabled(hw_enabled);
    cap_combo_->setEnabled(hw_enabled);

    // Select the combo entries whose item-data matches the current device
    // ids (combos are keyed by device id, -1 = System Default).
    auto selectByData = [](QComboBox* c, int id) {
        for (int i = 0; i < c->count(); ++i)
            if (c->itemData(i).toInt() == id) { c->setCurrentIndex(i); return; }
        c->setCurrentIndex(0);  // fall back to System Default
    };
    selectByData(pb_combo_, playback_device);
    selectByData(cap_combo_, capture_device);
}

void DeviceDialog::onAccept() {
    hw_enabled_ = hw_enable_check_->isChecked();
    pb_device_  = pb_combo_->currentData().toInt();
    cap_device_ = cap_combo_->currentData().toInt();
    // Only emit (which triggers a DSP rebuild) when something actually
    // changed. Previously the dialog always opened "disabled" and Apply
    // unconditionally emitted (-1,-1,false), silently turning OFF whatever
    // HW audio the user already had running. (#29)
    if (hw_enabled_ != init_enabled_ ||
        pb_device_  != init_pb_      ||
        cap_device_ != init_cap_) {
        emit deviceConfigChanged(pb_device_, cap_device_, hw_enabled_);
    }
    accept();
}

} // namespace gw
