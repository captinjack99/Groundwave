/**
 * @file main_window.hpp
 * @brief DSCA-NG main window — QMainWindow + QDockWidget layout
 *
 * Layout:
 *   Central:    SpectrumWidget (full width, top 60%) + ConstellationWidget (bottom-right)
 *   Left dock:  TxPanel (top) + AlarmPanel (bottom)
 *   Right dock: RxPanel (top) + InfoPanel (bottom)
 *   Bottom dock: MeterWidget
 */
#pragma once
#include "../include/app_state.hpp"
#include "data_bridge.hpp"
#include "audio_engine.hpp"
#include "style.hpp"

#include <QMainWindow>
#include <QDockWidget>
#include <QLabel>

namespace dsca {

// Forward declarations
class SpectrumWidget;
class ConstellationWidget;
class MeterWidget;
class TxPanel;
class RxPanel;
class AlarmPanel;
class InfoPanel;
class StreamPanel;
class ScopeWidget;
class LinkBudgetPanel;
class TuningPanel;
class ChannelResponseWidget;
class EyeDiagramWidget;
class PLSStatusWidget;

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(AppState& state, QWidget* parent = nullptr);

    DataBridge& bridge() { return bridge_; }

protected:
    void closeEvent(QCloseEvent* e) override;

private:
    void buildLayout();
    void buildMenuBar();
    void buildStatusBar();
    void buildKeyboardShortcuts();
    void exportSpectrumPNG();
    void showAboutDialog();
    void showDeviceDialog();

public:
    /** Open the hierarchical-modulation configuration dialog. Public
     *  because the TX panel invokes it from its inline "Hier" button.
     *  After accept, writes the new config to both `engine_` (live
     *  effect) and `state_.hier` (so panels can read it). */
    void openHierarchicalDialog();

private:
    QDockWidget* makeDock(const QString& title, QWidget* widget,
                           Qt::DockWidgetArea area, int min_w = 200);

    // State & bridge
    AppState&    state_;
    DataBridge   bridge_;
    AudioEngine  engine_;

    // Widgets
    SpectrumWidget*       spectrum_widget_      = nullptr;
    ConstellationWidget*  constellation_widget_ = nullptr;
    MeterWidget*          meter_widget_         = nullptr;
    TxPanel*              tx_panel_             = nullptr;
    RxPanel*              rx_panel_             = nullptr;
    AlarmPanel*           alarm_panel_          = nullptr;
    InfoPanel*            info_panel_           = nullptr;
    StreamPanel*          stream_panel_         = nullptr;
    ScopeWidget*          scope_widget_         = nullptr;
    LinkBudgetPanel*      link_budget_panel_    = nullptr;
    TuningPanel*          tuning_panel_         = nullptr;
    ChannelResponseWidget* channel_widget_      = nullptr;
    EyeDiagramWidget*     eye_widget_           = nullptr;
    PLSStatusWidget*      pls_widget_           = nullptr;

    // Status bar labels
    QLabel* status_sync_   = nullptr;
    QLabel* status_snr_    = nullptr;
    QLabel* status_preset_ = nullptr;
    QLabel* status_tick_   = nullptr;   // engine tick latency max/avg
    QLabel* status_engine_ = nullptr;   // engine running/stopped + frame counter
    QLabel* status_audio_  = nullptr;   // audio_monitor health
};

} // namespace dsca
