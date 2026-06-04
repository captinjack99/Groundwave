/**
 * @file device_dialog.hpp
 * @brief Hardware audio device selection dialog
 *
 * Allows the user to choose playback and capture audio devices
 * from the enumerated hardware list (via HWAudioDevice).
 */
#pragma once

#include "../../include/app_state.hpp"
#include "../../include/hw_audio.hpp"
#include "../style.hpp"

#include <QDialog>
#include <QComboBox>
#include <QLabel>
#include <QPushButton>
#include <QCheckBox>

namespace dsca {

class DeviceDialog : public QDialog {
    Q_OBJECT
public:
    explicit DeviceDialog(QWidget* parent = nullptr);

    /** Seed the dialog from the engine's current audio config so it opens
     *  reflecting the real state (instead of always "disabled / System
     *  Default", which made Apply silently turn HW audio OFF). Call after
     *  construction (the device combos are already populated). */
    void setCurrentConfig(int playback_device, int capture_device, bool hw_enabled);

    int  selectedPlaybackDevice() const { return pb_device_; }
    int  selectedCaptureDevice()  const { return cap_device_; }
    bool hwAudioEnabled()         const { return hw_enabled_; }

signals:
    void deviceConfigChanged(int playback_device, int capture_device, bool hw_enabled);

private slots:
    void onAccept();

private:
    void buildUi();
    void enumerateDevices();

    QCheckBox*   hw_enable_check_  = nullptr;
    QComboBox*   pb_combo_         = nullptr;
    QComboBox*   cap_combo_        = nullptr;
    QLabel*      status_label_     = nullptr;
    QPushButton* ok_btn_           = nullptr;
    QPushButton* cancel_btn_       = nullptr;

    int  pb_device_   = -1;
    int  cap_device_  = -1;
    bool hw_enabled_  = false;

    // Initial (seeded) state, for change detection in onAccept.
    int  init_pb_      = -1;
    int  init_cap_     = -1;
    bool init_enabled_ = false;

    std::vector<AudioDeviceInfo> pb_devices_;
    std::vector<AudioDeviceInfo> cap_devices_;
};

} // namespace dsca
