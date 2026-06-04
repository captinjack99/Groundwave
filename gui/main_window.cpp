/**
 * @file main_window.cpp
 */
#include "main_window.hpp"

#include "widgets/spectrum_widget.hpp"
#include "widgets/constellation_widget.hpp"
#include "widgets/meter_widget.hpp"
#include "panels/tx_panel.hpp"
#include "panels/rx_panel.hpp"
#include "panels/alarm_panel.hpp"
#include "panels/info_panel.hpp"
#include "panels/stream_panel.hpp"
#include "panels/link_budget_panel.hpp"
#include "panels/tuning_panel.hpp"
#include "widgets/scope_widget.hpp"
#include "widgets/channel_response_widget.hpp"
#include "widgets/eye_diagram_widget.hpp"
#include "widgets/pls_status_widget.hpp"
#include "dialogs/preset_dialog.hpp"
#include "dialogs/alarm_config_dialog.hpp"
#include "dialogs/device_dialog.hpp"
#include "dialogs/hierarchical_dialog.hpp"
#include "dialogs/help_dialog.hpp"
#include "../include/config_json.hpp"

#include <QMenuBar>
#include <QStatusBar>
#include <QSplitter>
#include <QScrollArea>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QWidget>
#include <QCloseEvent>
#include <QSettings>
#include <QApplication>
#include <QFileDialog>
#include <QShortcut>
#include <QKeySequence>
#include <QPixmap>
#include <QPushButton>

namespace dsca {

MainWindow::MainWindow(AppState& state, QWidget* parent)
    : QMainWindow(parent)
    , state_(state)
    , bridge_(state, this)
    , engine_(state, bridge_)
{
    setWindowTitle("DSCA-NG  v2.0  —  Digital Radio Modem");
    // Default and minimum sizes tuned so all docks have room to breathe
    // and rows don't collapse below the controls' natural widths. On
    // smaller screens (laptops at 1366×768) the user can still resize
    // smaller — minimum is set so the layout stays legible but starts
    // forcing scroll bars on heavy panels.
    setMinimumSize(1200, 780);
    resize(1680, 980);

    buildLayout();
    buildMenuBar();
    buildStatusBar();
    buildKeyboardShortcuts();

    // ---- Wire DataBridge signals to widgets ----
    connect(&bridge_, &DataBridge::spectrumReady,
            spectrum_widget_, &SpectrumWidget::onSpectrumReady);
    connect(&bridge_, &DataBridge::constellationReady,
            constellation_widget_, &ConstellationWidget::onConstellationReady);
    connect(&bridge_, &DataBridge::metersUpdated,
            meter_widget_, &MeterWidget::onMetersUpdated);
    connect(&bridge_, &DataBridge::statsUpdated,
            rx_panel_, &RxPanel::onStatsUpdated);
    connect(&bridge_, &DataBridge::agcUpdated,
            rx_panel_, &RxPanel::onAgcUpdated);
    connect(&bridge_, &DataBridge::alarmsUpdated,
            alarm_panel_, &AlarmPanel::onAlarmsUpdated);
    connect(&bridge_, &DataBridge::statsUpdated,
            [this](dsca::ModemStats s) {
        // Update status bar — full FSM state name (Searching/Acquiring/Locked/
        // Tracking/Lost) instead of just locked/unlocked, so the user can see
        // intermediate transitions during acquisition or lock loss.
        status_snr_->setText(QString("SNR  %1 dB").arg(s.snr_db, 0, 'f', 1));
        const char* name = dsca::syncStateName(s.sync_state);
        bool good = (s.sync_state == SyncState::Locked ||
                     s.sync_state == SyncState::Tracking);
        bool bad  = (s.sync_state == SyncState::Lost);
        status_sync_->setText(QString(good ? "● %1" : (bad ? "✗ %1" : "○ %1")).arg(name));
        status_sync_->setStyleSheet(
            good ? "color:#30D158; font-size:11px;"
                 : bad  ? "color:#FF453A; font-size:11px;"
                        : "color:#FF9F0A; font-size:11px;");

        // Engine tick latency. Color thresholds:
        //   < 10 ms  green  (engine is comfortable)
        //   < 50 ms  amber  (bursty — UI may feel choppy)
        //   ≥ 50 ms  red    (engine is over budget — hang likely)
        QColor tcol = (s.tick_max_ms < 10.f)
                        ? QColor("#48484E")
                        : (s.tick_max_ms < 50.f)
                            ? QColor("#FF9F0A")
                            : QColor("#FF453A");
        status_tick_->setText(
            QString("eng max %1ms / avg %2ms")
                .arg(s.tick_max_ms, 0, 'f', 1)
                .arg(s.tick_avg_ms, 0, 'f', 1));
        status_tick_->setStyleSheet(QString("color:%1;font-size:11px;"
            "font-family:'SF Mono',Menlo,'DejaVu Sans Mono',monospace;")
            .arg(tcol.name()));

        // Engine state + frame counters. The user reported "TX button
        // does nothing" — visibly tracking frames_tx/frames_rx makes
        // that diagnosable. Color goes green once frames are flowing,
        // amber if engine is up but idle, red if engine is offline.
        bool engine_alive = engine_.isRunning();
        QColor ecol;
        QString ename;
        if (!engine_alive) {
            ecol  = QColor("#FF453A");
            ename = "OFFLINE";
        } else if (s.frames_tx > 0 || s.frames_rx > 0) {
            ecol  = QColor("#30D158");
            ename = "RUNNING";
        } else {
            ecol  = QColor("#FF9F0A");
            ename = "IDLE";
        }
        status_engine_->setText(
            QString("ENGINE %1  ·  TX %2  RX %3")
                .arg(ename).arg(s.frames_tx).arg(s.frames_rx));
        status_engine_->setStyleSheet(QString("color:%1;font-size:11px;"
            "font-family:'SF Mono',Menlo,'DejaVu Sans Mono',monospace;")
            .arg(ecol.name()));
    });
    connect(&bridge_, &DataBridge::presetsChanged,
            [this]() {
        // Snapshot the slot + name under the lock — active_preset_slot and
        // presets[] are mutated under state_.mtx by applyPreset/saveToPreset.
        QString name;
        {
            std::lock_guard<std::mutex> lock(state_.mtx);
            int slot = state_.active_preset_slot;
            if (slot >= 0 && slot < static_cast<int>(NUM_PRESETS))
                name = QString::fromUtf8(state_.presets[static_cast<size_t>(slot)].name);
        }
        if (!name.isEmpty()) status_preset_->setText(name);
    });

    // Refresh info panel when config changes
    connect(tx_panel_, &TxPanel::configChanged,
            info_panel_, &InfoPanel::refresh);

    // Real-time link-budget recompute on any TX config change
    connect(tx_panel_, &TxPanel::configChanged,
            link_budget_panel_, &LinkBudgetPanel::recompute);

    // ---- Wire TxPanel config changes to AudioEngine ----
    connect(tx_panel_, &TxPanel::configChanged,
            &engine_, &AudioEngine::onConfigChanged);

    // uiOnlyChange: emitted by the TX button (and other state changes
    // that don't need a DSP rebuild). Refresh dependent UI but DO NOT
    // call engine.onConfigChanged() — that would teardown/rebuild the
    // DSP chain and briefly null out the encoders, causing processTX
    // to skip with the early-null guard. The engine reads tx_enabled
    // fresh from AppState every tick so the change takes effect on the
    // next tick automatically (~10 ms latency).
    connect(tx_panel_, &TxPanel::uiOnlyChange,
            info_panel_, &InfoPanel::refresh);
    connect(tx_panel_, &TxPanel::uiOnlyChange,
            link_budget_panel_, &LinkBudgetPanel::recompute);
    connect(tx_panel_, &TxPanel::uiOnlyChange, this, [this]() {
        bool tx_on;
        QString mod_name;
        QString fec_name;
        {
            std::lock_guard<std::mutex> lock(state_.mtx);
            tx_on    = state_.tx_enabled;
            mod_name = QString::fromUtf8(modulationName(state_.ofdm.modulation));
            fec_name = QString::fromUtf8(fecRateName(state_.frame.fec_rate));
        }
        QString msg = tx_on
            ? QString("TX ON  ·  %1 / %2  ·  transmitting")
                  .arg(mod_name).arg(fec_name)
            : QString("TX OFF  ·  %1 / %2 stays loaded")
                  .arg(mod_name).arg(fec_name);
        statusBar()->showMessage(msg, 3000);
    });

    // Inline Hier button on TX panel opens the same dialog the Engine
    // menu does — keeps the configuration single-sourced.
    connect(tx_panel_, &TxPanel::hierarchicalRequested,
            this, &MainWindow::openHierarchicalDialog);

    // ---- Wire AudioEngine status ----
    connect(&engine_, &AudioEngine::engineError, this, [this](const QString& msg) {
        statusBar()->showMessage("Engine error: " + msg, 5000);
    });

    // ---- Wire stream panel + scope ----
    connect(stream_panel_, &StreamPanel::streamConfigChanged,
            &engine_, &AudioEngine::onConfigChanged);
    connect(&bridge_, &DataBridge::streamLevels,
            stream_panel_, &StreamPanel::onStreamLevels);
    connect(&bridge_, &DataBridge::scopeSamples,
            scope_widget_, [this](const std::vector<float>& s) {
                scope_widget_->pushSamples(s.data(), s.size());
            });

    // ---- Diagnostics widgets ----
    connect(&bridge_, &DataBridge::channelResponseReady,
            channel_widget_,
            [this](const std::vector<float>& mag_db) {
                channel_widget_->pushEstimate(mag_db);
            });
    connect(&bridge_, &DataBridge::eyeSamplesReady,
            eye_widget_,
            [this](const std::vector<float>& s, int sps) {
                eye_widget_->pushSamples(s.data(), s.size(), sps);
            });
    connect(&bridge_, &DataBridge::plsUpdated,
            pls_widget_,
            [this](int mod, int fec, int slot, int total, bool ok, int conf) {
                pls_widget_->update(static_cast<Modulation>(mod),
                                    static_cast<FECRate>(fec),
                                    slot, total, ok, conf);
            });

    // ---- Tuning panel ----
    connect(tuning_panel_, &TuningPanel::configChanged,
            &engine_, &AudioEngine::onConfigChanged);
    connect(&bridge_, &DataBridge::statsUpdated,
            tuning_panel_, &TuningPanel::onStatsUpdated);

    // ---- Squelch + PAPR readout on the meter widget ----
    connect(&bridge_, &DataBridge::statsUpdated,
            this, [this](dsca::ModemStats s) {
                meter_widget_->setTxPapr(s.papr_tx_db);
                std::lock_guard<std::mutex> lock(state_.mtx);
                meter_widget_->setSquelchThreshold(
                    state_.modem.squelch.open_threshold_db);
            });

    // Restore layout
    QSettings settings("DSCA-NG", "MainWindow");
    if (settings.contains("geometry"))
        restoreGeometry(settings.value("geometry").toByteArray());
    if (settings.contains("windowState"))
        restoreState(settings.value("windowState").toByteArray());

    bridge_.start(33);   // ~30 fps

    // Start audio engine
    engine_.startup();
}

// =========================================================================
// Layout
// =========================================================================

void MainWindow::buildLayout() {
    setDockOptions(QMainWindow::AnimatedDocks |
                   QMainWindow::AllowNestedDocks |
                   QMainWindow::AllowTabbedDocks);

    // ---- Central widget: spectrum (top) + constellation (bottom) ----
    spectrum_widget_     = new SpectrumWidget(state_);
    constellation_widget_ = new ConstellationWidget(state_);
    meter_widget_        = new MeterWidget;

    // Minimum sizes ensure widgets don't get squeezed below readability when
    // docks are dragged in.
    spectrum_widget_->setMinimumHeight(160);
    constellation_widget_->setMinimumSize(280, 280);  // square-ish
    meter_widget_->setMinimumWidth(140);

    // Vertical splitter: spectrum on top, const+meter on bottom.
    // Roughly 1:1 vertical (was 3:2) so the constellation has real estate.
    auto* central_split = new QSplitter(Qt::Vertical);
    central_split->setHandleWidth(1);
    central_split->setStyleSheet(
        "QSplitter::handle { background:#1C1C24; }"
        "QSplitter::handle:hover { background:#0099FF60; }");
    central_split->addWidget(spectrum_widget_);

    // Bottom row: constellation (large) | meter (narrow)
    auto* bottom_w = new QWidget;
    auto* bottom_h = new QHBoxLayout(bottom_w);
    bottom_h->setContentsMargins(0, 0, 0, 0);
    bottom_h->setSpacing(0);
    auto* bottom_split = new QSplitter(Qt::Horizontal);
    bottom_split->setHandleWidth(1);
    bottom_split->setStyleSheet(
        "QSplitter::handle { background:#1C1C24; }"
        "QSplitter::handle:hover { background:#0099FF60; }");
    bottom_split->addWidget(constellation_widget_);
    bottom_split->addWidget(meter_widget_);
    bottom_split->setStretchFactor(0, 5);  // was 3 — give constellation more
    bottom_split->setStretchFactor(1, 1);
    bottom_h->addWidget(bottom_split);
    central_split->addWidget(bottom_w);
    // Spectrum and bottom row roughly equal — was 3:2 (constellation cramped)
    central_split->setStretchFactor(0, 1);
    central_split->setStretchFactor(1, 1);

    setCentralWidget(central_split);

    // ---- Panels ----
    tx_panel_    = new TxPanel(state_);
    rx_panel_    = new RxPanel(state_);
    alarm_panel_ = new AlarmPanel(state_);
    info_panel_  = new InfoPanel(state_);
    stream_panel_ = new StreamPanel(state_);
    stream_panel_->setEngine(&engine_);
    scope_widget_ = new ScopeWidget();
    link_budget_panel_ = new LinkBudgetPanel(state_);
    tuning_panel_      = new TuningPanel(state_);
    channel_widget_    = new ChannelResponseWidget(state_);
    eye_widget_        = new EyeDiagramWidget();
    pls_widget_        = new PLSStatusWidget();

    // Helper that wraps a panel in a vertical scroll area so docks always
    // honor the available height — content scrolls when window is too short
    // or when other docks consume vertical space. Horizontal scrolling is
    // disabled so panels reflow cleanly when the dock is resized.
    auto makeScrollable = [](QWidget* content) -> QScrollArea* {
        auto* scroll = new QScrollArea();
        scroll->setWidget(content);
        scroll->setWidgetResizable(true);
        scroll->setFrameShape(QFrame::NoFrame);
        scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        scroll->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
        return scroll;
    };

    // Variant that allows horizontal scroll. Used for panels with a wide
    // grid (Stream Panel's 10 columns) where compressing horizontally
    // would cause unreadable text / overlapping controls.
    auto makeScrollableHV = [](QWidget* content) -> QScrollArea* {
        auto* scroll = new QScrollArea();
        scroll->setWidget(content);
        scroll->setWidgetResizable(true);
        scroll->setFrameShape(QFrame::NoFrame);
        scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
        scroll->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
        return scroll;
    };

    // Left dock: TX + Alarms — each individually scrollable, separated by
    // a draggable splitter so the user can resize the relative heights.
    auto* left_split = new QSplitter(Qt::Vertical);
    left_split->setHandleWidth(1);
    left_split->setChildrenCollapsible(false);
    left_split->addWidget(makeScrollable(tx_panel_));
    left_split->addWidget(makeScrollable(alarm_panel_));
    left_split->setStretchFactor(0, 3);
    left_split->setStretchFactor(1, 2);

    auto* left_dock = makeDock("Controls", left_split, Qt::LeftDockWidgetArea, 300);
    addDockWidget(Qt::LeftDockWidgetArea, left_dock);

    // Right dock: RX + Info + Link Budget, each in its own scroll area.
    auto* right_split = new QSplitter(Qt::Vertical);
    right_split->setHandleWidth(1);
    right_split->setChildrenCollapsible(false);
    right_split->addWidget(makeScrollable(rx_panel_));
    right_split->addWidget(makeScrollable(info_panel_));
    right_split->addWidget(makeScrollable(link_budget_panel_));
    right_split->setStretchFactor(0, 2);
    right_split->setStretchFactor(1, 2);
    right_split->setStretchFactor(2, 4);

    auto* right_dock = makeDock("Status", right_split, Qt::RightDockWidgetArea, 340);
    addDockWidget(Qt::RightDockWidgetArea, right_dock);

    // Bottom dock: multi-stream + scope side-by-side, both wrapped in a
    // horizontal splitter (resizable) and each scrollable.
    auto* stream_bar_split = new QSplitter(Qt::Horizontal);
    stream_bar_split->setHandleWidth(1);
    stream_bar_split->setChildrenCollapsible(false);
    // Stream panel has a wide 10-column grid; allow horizontal scroll
    // so columns keep their natural width rather than overlapping.
    stream_bar_split->addWidget(makeScrollableHV(stream_panel_));
    stream_bar_split->addWidget(scope_widget_);   // scope draws its own area
    stream_bar_split->setStretchFactor(0, 3);
    stream_bar_split->setStretchFactor(1, 2);

    auto* stream_bar_dock = makeDock("Streams + Scope", stream_bar_split,
                                      Qt::BottomDockWidgetArea, 360);
    stream_bar_dock->setMinimumHeight(200);  // 8 stream rows + header need room
    addDockWidget(Qt::BottomDockWidgetArea, stream_bar_dock);

    // Diagnostics dock: channel response, eye diagram, PLS status side-by-side.
    auto* diag_split = new QSplitter(Qt::Horizontal);
    diag_split->setHandleWidth(1);
    diag_split->setChildrenCollapsible(false);
    diag_split->addWidget(channel_widget_);
    diag_split->addWidget(eye_widget_);
    diag_split->addWidget(pls_widget_);
    diag_split->setStretchFactor(0, 3);
    diag_split->setStretchFactor(1, 3);
    diag_split->setStretchFactor(2, 2);
    auto* diag_dock = makeDock("Diagnostics", diag_split,
                                Qt::BottomDockWidgetArea, 360);
    diag_dock->setMinimumHeight(140);
    addDockWidget(Qt::BottomDockWidgetArea, diag_dock);

    // Tuning panel: AFC/AGC/RX gain/squelch controls — left dock.
    auto* tuning_dock = makeDock("Tuning", makeScrollable(tuning_panel_),
                                  Qt::LeftDockWidgetArea, 300);
    addDockWidget(Qt::LeftDockWidgetArea, tuning_dock);
    tabifyDockWidget(left_dock, tuning_dock);
    // Default to Controls being active over Tuning when the tab group
    // first appears — otherwise users land on AGC settings rather than
    // the primary TX controls.
    left_dock->raise();
}

QDockWidget* MainWindow::makeDock(const QString& title, QWidget* widget,
                                   Qt::DockWidgetArea /*area*/, int min_w) {
    auto* dock = new QDockWidget(title, this);
    dock->setWidget(widget);
    dock->setFeatures(QDockWidget::DockWidgetMovable |
                      QDockWidget::DockWidgetFloatable);
    dock->setMinimumWidth(min_w);
    dock->setObjectName(title);   // needed for restoreState()
    return dock;
}

// =========================================================================
// Menu Bar
// =========================================================================

void MainWindow::buildMenuBar() {
    // File
    auto* file_menu = menuBar()->addMenu("File");
    file_menu->addAction("New Config", [this]() {
        {
            std::lock_guard<std::mutex> lock(state_.mtx);
            state_.applyPreset(1);     // reset to Standard
        }
        // Outside the lock: refresh panels (info panel re-locks) and
        // notify the engine so the running DSP picks up the new config
        // (FFT/modcod/FEC). Without onConfigChanged the engine kept the
        // old topology and lost lock.
        tx_panel_->refreshFromState();
        info_panel_->refresh();
        engine_.onConfigChanged();
    });
    file_menu->addSeparator();
    file_menu->addAction("Save Config…", [this]() {
        QString path = QFileDialog::getSaveFileName(
            this, "Save Configuration", "dsca_config.json",
            "JSON Files (*.json)");
        if (path.isEmpty()) return;
        bool ok;
        {
            // Serialize under the lock — the engine thread writes stats /
            // spectrum / stream_rms_db into AppState every tick, so an
            // unsynchronized read can serialize torn values (UB).
            std::lock_guard<std::mutex> lock(state_.mtx);
            ok = dsca::saveConfigToFile(state_, path.toStdString());
        }
        statusBar()->showMessage(ok ? "Config saved: " + path : "Save failed!", 3000);
    });
    file_menu->addAction("Load Config…", [this]() {
        QString path = QFileDialog::getOpenFileName(
            this, "Load Configuration", "",
            "JSON Files (*.json)");
        if (path.isEmpty()) return;
        bool ok;
        {
            // Mutate AppState under the lock; the engine thread reads
            // ofdm/frame/modem every tick. deserializeConfig does NOT
            // re-lock, so holding state_.mtx here is safe (no recursion),
            // and the values are range-validated inside the deserializer.
            std::lock_guard<std::mutex> lock(state_.mtx);
            ok = dsca::loadConfigFromFile(path.toStdString(), state_);
            if (ok) {
                // Clear stale live visualization/measurements so the display
                // doesn't show the previous config's constellation/levels/sync
                // until new frames flow under the loaded config.
                state_.constellation.clear();
                state_.stats = ModemStats{};
            }
        }
        // Refresh panels OUTSIDE the lock (they re-lock state_.mtx).
        if (ok) {
            tx_panel_->refreshFromState();
            info_panel_->refresh();
            engine_.onConfigChanged();
            statusBar()->showMessage("Config loaded: " + path, 3000);
        } else {
            statusBar()->showMessage("Load failed!", 3000);
        }
    });
    file_menu->addSeparator();
    auto* iq_rec_action = file_menu->addAction("Start IQ recording…");
    auto* iq_stop_action = file_menu->addAction("Stop IQ recording");
    iq_stop_action->setEnabled(false);
    connect(iq_rec_action, &QAction::triggered, this,
            [this, iq_rec_action, iq_stop_action]() {
        QString path = QFileDialog::getSaveFileName(this,
            "Record IQ to WAV", QString(),
            "IQ WAV (*.wav);;All files (*)");
        if (path.isEmpty()) return;
        if (engine_.startIQRecording(path.toStdString())) {
            iq_rec_action->setEnabled(false);
            iq_stop_action->setEnabled(true);
            statusBar()->showMessage("IQ recording: " + path, 3000);
        } else {
            statusBar()->showMessage("Failed to open IQ file for write", 3000);
        }
    });
    connect(iq_stop_action, &QAction::triggered, this,
            [this, iq_rec_action, iq_stop_action]() {
        engine_.stopIQRecording();
        iq_rec_action->setEnabled(true);
        iq_stop_action->setEnabled(false);
        statusBar()->showMessage("IQ recording stopped", 3000);
    });
    file_menu->addAction("Replay IQ from WAV…", [this]() {
        QString path = QFileDialog::getOpenFileName(this,
            "Replay IQ", QString(), "IQ WAV (*.wav);;All files (*)");
        if (path.isEmpty()) return;
        if (engine_.startIQPlayback(path.toStdString())) {
            statusBar()->showMessage("Replaying IQ: " + path, 5000);
        } else {
            statusBar()->showMessage("Failed to open IQ file (need 2-ch float WAV)", 5000);
        }
    });
    file_menu->addAction("Stop IQ replay", [this]() {
        engine_.stopIQPlayback();
        statusBar()->showMessage("IQ replay stopped", 3000);
    });
    file_menu->addSeparator();
    file_menu->addAction("Exit", qApp, &QApplication::quit);

    // Presets
    auto* preset_menu = menuBar()->addMenu("Presets");
    preset_menu->addAction("Manage Presets…", [this]() {
        PresetDialog dlg(state_, this);
        connect(&dlg, &PresetDialog::presetLoaded, [this]() {
            tx_panel_->refreshFromState();
            info_panel_->refresh();
            // The dialog applied a new preset to AppState; the running
            // engine must rebuild DSP for the new FFT/modcod/FEC.
            engine_.onConfigChanged();
            if (state_.active_preset_slot >= 0)
                status_preset_->setText(
                    state_.presets[static_cast<size_t>(
                        state_.active_preset_slot)].name);
        });
        dlg.exec();
    });
    preset_menu->addSeparator();
    for (size_t i = 0; i < NUM_PRESETS; ++i) {
        if (!state_.presets[i].valid) continue;
        QString name = QString("%1: %2").arg(i).arg(state_.presets[i].name);
        preset_menu->addAction(name, [this, i]() {
            {
                std::lock_guard<std::mutex> lock(state_.mtx);
                if (!state_.presets[i].valid) return;
                state_.applyPreset(static_cast<int>(i));
            }
            tx_panel_->refreshFromState();
            info_panel_->refresh();
            engine_.onConfigChanged();
        });
    }

    // Alarms
    auto* alarm_menu = menuBar()->addMenu("Alarms");
    alarm_menu->addAction("Configure…", [this]() {
        AlarmConfigDialog dlg(state_, this);
        dlg.exec();
    });
    alarm_menu->addAction("Acknowledge All", [this]() {
        std::lock_guard<std::mutex> lock(state_.mtx);
        state_.acknowledgeAlarms();   // does not lock internally
    });
    alarm_menu->addAction("Clear Log", [this]() {
        std::lock_guard<std::mutex> lock(state_.mtx);
        state_.clearAlarmLog();       // does not lock internally
    });

    // View
    auto* view_menu = menuBar()->addMenu("View");
    view_menu->addAction("Reset Layout", [this]() {
        QSettings settings("DSCA-NG", "MainWindow");
        settings.remove("geometry");
        settings.remove("windowState");
        resize(1440, 860);
    });
    view_menu->addSeparator();
    auto* mask_action = view_menu->addAction("Show FCC §73.319 mask");
    mask_action->setCheckable(true);
    connect(mask_action, &QAction::toggled, [this](bool on) {
        spectrum_widget_->setShowMask(on);
    });
    auto* clear_cursors = view_menu->addAction("Clear spectrum cursors");
    connect(clear_cursors, &QAction::triggered, [this]() {
        spectrum_widget_->clearCursors();
    });
    auto* evm_action = view_menu->addAction("Constellation EVM-detail mode");
    evm_action->setCheckable(true);
    connect(evm_action, &QAction::toggled, [this](bool on) {
        constellation_widget_->setEvmDetailMode(on);
    });
    view_menu->addSeparator();
    auto* theme_action = view_menu->addAction("Light Theme");
    theme_action->setCheckable(true);
    connect(theme_action, &QAction::toggled, [this](bool light) {
        QApplication* app = qobject_cast<QApplication*>(QCoreApplication::instance());
        if (!app) return;
        if (light) {
            app->setStyleSheet(style::buildStyleSheet() + style::buildLightStyleSheet());
        } else {
            app->setStyleSheet(style::buildStyleSheet());
        }
    });
    view_menu->addSeparator();
    view_menu->addAction("Export Spectrum PNG…",
                          QKeySequence("Ctrl+E"),
                          this,
                          &MainWindow::exportSpectrumPNG);

    // Engine
    auto* engine_menu = menuBar()->addMenu("Engine");

    auto* start_action = engine_menu->addAction("Start Engine", [this]() {
        engine_.startup();
    });
    auto* stop_action = engine_menu->addAction("Stop Engine", [this]() {
        engine_.shutdown();
    });
    engine_menu->addSeparator();

    auto* orbgrand_action = engine_menu->addAction("Use ORBGRAND Decoder");
    orbgrand_action->setCheckable(true);
    orbgrand_action->setChecked(true);
    orbgrand_action->setToolTip(
        "Ordered Reliability Bits GRAND — near-ML soft decoding for short "
        "high-rate LDPC codes (Duffy/Médard 2022–2024). Default ON. "
        "Trade-off: better BER near threshold at higher CPU cost than the "
        "layered min-sum BP decoder. Switch off if tick latency becomes "
        "an issue on heavy modcods.");
    connect(orbgrand_action, &QAction::toggled, [this](bool on) {
        auto cfg = engine_.engineConfig();
        cfg.use_orbgrand = on;
        engine_.setEngineConfig(cfg);
        statusBar()->showMessage(
            on ? "Decoder: ORBGRAND (near-ML)"
               : "Decoder: Min-Sum BP", 3000);
    });

    auto* awgn_action = engine_menu->addAction("Loopback AWGN (20 dB)");
    awgn_action->setCheckable(true);
    awgn_action->setChecked(false);
    connect(awgn_action, &QAction::toggled, [this](bool on) {
        auto cfg = engine_.engineConfig();
        cfg.loopback_snr = on ? 20.f : -1.f;
        engine_.setEngineConfig(cfg);
        statusBar()->showMessage(
            on ? "Loopback: AWGN @ 20 dB"
               : "Loopback: perfect channel", 3000);
    });

    engine_menu->addSeparator();

    auto* mmse_action = engine_menu->addAction("MMSE Channel Estimation");
    mmse_action->setCheckable(true);
    mmse_action->setChecked(false);
    connect(mmse_action, &QAction::toggled, [this](bool on) {
        auto cfg = engine_.engineConfig();
        cfg.use_mmse = on;
        engine_.setEngineConfig(cfg);
        engine_.onConfigChanged();  // needs DSP re-init
        statusBar()->showMessage(
            on ? "Channel Est: MMSE + Wiener"
               : "Channel Est: LS + Linear", 3000);
    });

    auto* pwl_action = engine_menu->addAction("PWL LLR Demapper");
    pwl_action->setCheckable(true);
    pwl_action->setChecked(true);
    connect(pwl_action, &QAction::toggled, [this](bool on) {
        auto cfg = engine_.engineConfig();
        cfg.use_pwl_llr = on;
        engine_.setEngineConfig(cfg);
        statusBar()->showMessage(
            on ? "Soft Demap: Piecewise-Linear"
               : "Soft Demap: Max-Log MAP", 3000);
    });

    auto* dd_action = engine_menu->addAction("Decision-Directed Channel Est.");
    dd_action->setCheckable(true);
    dd_action->setChecked(false);
    dd_action->setToolTip(
        "After the pilot-based equalizer, hard-decide each data symbol "
        "and treat it as an additional reference to refine the channel "
        "estimate between pilots. Buys ~0.2–0.4 dB at marginal SNR for "
        "the cost of one extra equalize + soft-demap pass per OFDM "
        "symbol.");
    connect(dd_action, &QAction::toggled, [this](bool on) {
        auto cfg = engine_.engineConfig();
        cfg.use_dd_chest = on;
        engine_.setEngineConfig(cfg);
        statusBar()->showMessage(
            on ? "Channel Est: Pilot + Decision-Directed refinement"
               : "Channel Est: Pilot only", 3000);
    });

    auto* bicm_action = engine_menu->addAction("BICM-ID (Iterative Demap)");
    bicm_action->setCheckable(true);
    bicm_action->setChecked(false);
    bicm_action->setToolTip(
        "Iterative bit-interleaved coded modulation: alternate symbol "
        "soft-demapping and LDPC decoding, exchanging extrinsic LLRs. "
        "Each outer iteration adds CPU. 3 iterations typically yields "
        "0.5–1.5 dB on QAM16+ at low-to-mid SNR. Replaces ORBGRAND / "
        "BP for the codeword when active.");
    connect(bicm_action, &QAction::toggled, [this](bool on) {
        auto cfg = engine_.engineConfig();
        cfg.use_bicm_id = on;
        engine_.setEngineConfig(cfg);
        statusBar()->showMessage(
            on ? "Decoder: BICM-ID (3 iterations)"
               : "Decoder: per-codeword (ORBGRAND or BP)", 3000);
    });

    engine_menu->addSeparator();

    auto* papr_action = engine_menu->addAction("PAPR Reduction (TR)");
    papr_action->setCheckable(true);
    papr_action->setChecked(false);
    connect(papr_action, &QAction::toggled, [this](bool on) {
        auto cfg = engine_.engineConfig();
        cfg.papr.enabled = on;
        cfg.papr.target_papr_db = 7.0f;
        cfg.papr.max_iterations = 8;
        cfg.papr.reserve_fraction = 0.05f;
        engine_.setEngineConfig(cfg);
        engine_.onConfigChanged();
        statusBar()->showMessage(
            on ? "PAPR: Tone Reservation (7 dB target)"
               : "PAPR: Disabled", 3000);
    });

    auto* rs_action = engine_menu->addAction("Reed-Solomon Outer Code");
    rs_action->setCheckable(true);
    rs_action->setChecked(state_.modem.enable_rs_outer);
    rs_action->setToolTip(
        "Outer Reed-Solomon (16 parity bytes per LDPC block) for residual "
        "byte-error cleanup in the LDPC waterfall region. Reduces net "
        "payload bitrate by 6–24 % depending on FEC rate (smaller blocks "
        "= bigger overhead). When OFF, the net bitrate equals the FEC-"
        "coded rate and the engine relies entirely on LDPC for error "
        "correction.");
    connect(rs_action, &QAction::toggled, [this](bool on) {
        {
            std::lock_guard<std::mutex> lock(state_.mtx);
            state_.modem.enable_rs_outer = on;
        }
        engine_.onConfigChanged();
        if (info_panel_)        info_panel_->refresh();
        if (link_budget_panel_) link_budget_panel_->recompute();
        statusBar()->showMessage(
            on ? "Reed-Solomon outer code: ON  (16-byte parity per LDPC block)"
               : "Reed-Solomon outer code: OFF  (LDPC only)", 4000);
    });

    auto* hier_action = engine_menu->addAction("Hierarchical Modulation…");
    hier_action->setToolTip(
        "Open the hierarchical modulation configuration dialog. Choose "
        "between 16 preset HP/LP layer pairings (QPSK/16-QAM through "
        "1024-QAM/4096-QAM), or define a custom split for any base "
        "modulation. Set α to control the HP/LP threshold gap.");
    connect(hier_action, &QAction::triggered,
            this, &MainWindow::openHierarchicalDialog);

    auto* vcm_action = engine_menu->addAction("VCM Stereo Schedule");
    vcm_action->setCheckable(true);
    vcm_action->setChecked(false);
    connect(vcm_action, &QAction::toggled, [this](bool on) {
        auto cfg = engine_.engineConfig();
        if (on) {
            cfg.vcm = dsca::createStereoVCM(
                dsca::Modulation::QPSK, dsca::FECRate::Rate_1_2,
                dsca::Modulation::QAM64, dsca::FECRate::Rate_3_4,
                2, 2);
        } else {
            cfg.vcm.enabled = false;
        }
        engine_.setEngineConfig(cfg);
        engine_.onConfigChanged();
        statusBar()->showMessage(
            on ? "VCM: QPSK½ + QAM64¾ stereo"
               : "VCM: Disabled (uniform ModCod)", 3000);
    });

    (void)start_action;
    (void)stop_action;

    engine_menu->addSeparator();
    engine_menu->addAction("Audio Devices…", this,
                            &MainWindow::showDeviceDialog);

    // Help
    auto* help_menu = menuBar()->addMenu("Help");
    auto* user_guide = help_menu->addAction("User Guide");
    user_guide->setShortcut(QKeySequence::HelpContents);  // F1
    connect(user_guide, &QAction::triggered, [this]() {
        auto* h = new HelpDialog(this);
        h->setAttribute(Qt::WA_DeleteOnClose);
        h->showTab(0);
        h->show();
    });
    help_menu->addAction("RF / DSP Concepts", [this]() {
        auto* h = new HelpDialog(this);
        h->setAttribute(Qt::WA_DeleteOnClose);
        h->showTab(1);
        h->show();
    });
    help_menu->addAction("Keyboard Shortcuts", [this]() {
        auto* h = new HelpDialog(this);
        h->setAttribute(Qt::WA_DeleteOnClose);
        h->showTab(2);
        h->show();
    });
    help_menu->addAction("Troubleshooting", [this]() {
        auto* h = new HelpDialog(this);
        h->setAttribute(Qt::WA_DeleteOnClose);
        h->showTab(3);
        h->show();
    });
    help_menu->addSeparator();
    help_menu->addAction("About DSCA-NG", this, &MainWindow::showAboutDialog);
}

// =========================================================================
// Status Bar
// =========================================================================

void MainWindow::buildStatusBar() {
    status_sync_ = new QLabel("○ UNLOCKED");
    status_sync_->setStyleSheet("color:#FF9F0A;font-size:11px;");
    status_sync_->setMinimumWidth(90);
    status_sync_->setToolTip(
        "Receive synchronization state: Searching, Acquiring, Locked, "
        "Tracking, or Lost. Green = locked; amber = transitioning; "
        "red = lost.");

    status_snr_ = new QLabel("SNR  0.0 dB");
    status_snr_->setStyleSheet("color:#8E8E93;font-size:11px;"
                               "font-family:'SF Mono',Menlo,'DejaVu Sans Mono',monospace;");
    status_snr_->setMinimumWidth(90);
    status_snr_->setToolTip("Post-equalization signal-to-noise ratio "
                             "estimate from pilot residuals.");

    // Engine state — TX/RX frame counters so the user can SEE the engine
    // working. "TX 0  RX 0" while nothing's transmitting; counters tick
    // up at ~30 fps once TX is enabled. Green when frames are flowing,
    // amber if engine is running but no frames, red if engine is stopped.
    status_engine_ = new QLabel("ENGINE — starting…");
    status_engine_->setStyleSheet("color:#FF9F0A;font-size:11px;"
                                   "font-family:'SF Mono',Menlo,'DejaVu Sans Mono',monospace;");
    status_engine_->setMinimumWidth(150);
    status_engine_->setToolTip(
        "Engine state and frame counters. Frames TX = encoded and "
        "transmitted; Frames RX = decoded with valid CRC. Both should "
        "tick up rapidly when TX is enabled in loopback mode.");

    // Audio monitor health: green dot when the always-on playback device
    // is running, red dot when it failed to start (user gets no audio).
    status_audio_ = new QLabel("● AUDIO");
    status_audio_->setStyleSheet("color:#30D158;font-size:11px;");
    status_audio_->setToolTip(
        "Always-on audio monitor status. Green = playing decoded audio "
        "to the default device. Red = audio_monitor failed to start "
        "(check Settings → Audio Devices).");

    status_preset_ = new QLabel(
        state_.active_preset_slot >= 0
            ? state_.presets[static_cast<size_t>(state_.active_preset_slot)].name
            : "Custom");
    status_preset_->setStyleSheet("color:#48484E;font-size:11px;");

    // Engine health: max/avg tick latency over recent window. Watch this
    // when the GUI feels sluggish — if max climbs above ~50 ms, the engine
    // thread is bursty and we're contending for something.
    status_tick_ = new QLabel("eng max 0.0 ms");
    status_tick_->setStyleSheet("color:#48484E;font-size:11px;"
                                "font-family:'SF Mono',Menlo,'DejaVu Sans Mono',monospace;");

    statusBar()->addWidget(status_sync_);
    statusBar()->addWidget(new QLabel("  "));
    statusBar()->addWidget(status_snr_);
    statusBar()->addWidget(new QLabel("  "));
    statusBar()->addWidget(status_engine_);
    statusBar()->addPermanentWidget(status_audio_);
    statusBar()->addPermanentWidget(new QLabel("  "));
    statusBar()->addPermanentWidget(status_tick_);
    statusBar()->addPermanentWidget(new QLabel("  "));
    statusBar()->addPermanentWidget(status_preset_);
    statusBar()->addPermanentWidget(new QLabel("  "));
}

// =========================================================================
// Close
// =========================================================================

void MainWindow::closeEvent(QCloseEvent* e) {
    engine_.shutdown();
    bridge_.stop();
    QSettings settings("DSCA-NG", "MainWindow");
    settings.setValue("geometry",    saveGeometry());
    settings.setValue("windowState", saveState());
    QMainWindow::closeEvent(e);
}

// =========================================================================
// Keyboard Shortcuts
// =========================================================================

void MainWindow::buildKeyboardShortcuts() {
    // F5 — TX toggle
    auto* tx_toggle = new QShortcut(QKeySequence(Qt::Key_F5), this);
    connect(tx_toggle, &QShortcut::activated, [this]() {
        bool now;
        {
            std::lock_guard<std::mutex> lock(state_.mtx);
            state_.tx_enabled = !state_.tx_enabled;
            now = state_.tx_enabled;   // read under the lock, use the local
        }
        tx_panel_->refreshFromState();
        statusBar()->showMessage(now ? "TX ON" : "TX OFF", 2000);
    });

    // F1..F8 — Preset recall. Preset apply mutates ofdm/frame/modem under
    // the state mutex, then the GUI thread refreshes panels OUTSIDE the lock
    // (info panel takes the lock itself, so we must not hold it during refresh).
    for (int i = 0; i < static_cast<int>(NUM_PRESETS); ++i) {
        // Skip F1 (bound to Help) and F5 (bound to TX-toggle) — binding a
        // preset to the same key produced an ambiguous QShortcut activation
        // (Qt fires activatedAmbiguously and neither action runs reliably).
        // Presets 0 and 4 remain reachable via the Presets menu. (#58)
        if (i == 0 || i == 4) continue;
        auto* sc = new QShortcut(QKeySequence(Qt::Key_F1 + i), this);
        connect(sc, &QShortcut::activated, [this, i]() {
            QString preset_name;
            int active_slot = -1;
            {
                std::lock_guard<std::mutex> lock(state_.mtx);
                if (!state_.presets[static_cast<size_t>(i)].valid) return;
                state_.applyPreset(i);
                preset_name  = QString::fromUtf8(
                    state_.presets[static_cast<size_t>(i)].name);
                active_slot = state_.active_preset_slot;
            }
            // Outside the lock: refresh widgets and notify engine.
            tx_panel_->refreshFromState();
            info_panel_->refresh();
            engine_.onConfigChanged();
            statusBar()->showMessage(
                QString("Preset %1: %2").arg(i).arg(preset_name), 2000);
            if (active_slot >= 0) status_preset_->setText(preset_name);
        });
    }

    // F9 — Start engine
    auto* start_sc = new QShortcut(QKeySequence(Qt::Key_F9), this);
    connect(start_sc, &QShortcut::activated, [this]() {
        engine_.startup();
        statusBar()->showMessage("Engine started", 2000);
    });

    // F10 — Stop engine
    auto* stop_sc = new QShortcut(QKeySequence(Qt::Key_F10), this);
    connect(stop_sc, &QShortcut::activated, [this]() {
        engine_.shutdown();
        statusBar()->showMessage("Engine stopped", 2000);
    });

    // Ctrl+S — Save config
    auto* save_sc = new QShortcut(QKeySequence("Ctrl+S"), this);
    connect(save_sc, &QShortcut::activated, [this]() {
        QString path = QFileDialog::getSaveFileName(
            this, "Save Configuration", "dsca_config.json",
            "JSON Files (*.json)");
        if (!path.isEmpty()) {
            if (dsca::saveConfigToFile(state_, path.toStdString()))
                statusBar()->showMessage("Config saved: " + path, 3000);
            else
                statusBar()->showMessage("Save failed!", 3000);
        }
    });

    // Ctrl+O — Load config
    auto* load_sc = new QShortcut(QKeySequence("Ctrl+O"), this);
    connect(load_sc, &QShortcut::activated, [this]() {
        QString path = QFileDialog::getOpenFileName(
            this, "Load Configuration", "",
            "JSON Files (*.json)");
        if (!path.isEmpty()) {
            if (dsca::loadConfigFromFile(path.toStdString(), state_)) {
                tx_panel_->refreshFromState();
                info_panel_->refresh();
                engine_.onConfigChanged();
                statusBar()->showMessage("Config loaded: " + path, 3000);
            } else {
                statusBar()->showMessage("Load failed!", 3000);
            }
        }
    });
}

// =========================================================================
// Export Spectrum/Waterfall as PNG
// =========================================================================

void MainWindow::exportSpectrumPNG() {
    if (!spectrum_widget_) return;

    QString path = QFileDialog::getSaveFileName(
        this, "Export Spectrum", "spectrum.png",
        "PNG Images (*.png);;All Files (*)");
    if (path.isEmpty()) return;

    // Render the spectrum widget to a high-DPI pixmap
    qreal dpr = devicePixelRatioF();
    QPixmap pixmap(static_cast<int>(spectrum_widget_->width() * dpr),
                   static_cast<int>(spectrum_widget_->height() * dpr));
    pixmap.setDevicePixelRatio(dpr);
    pixmap.fill(Qt::black);
    spectrum_widget_->render(&pixmap);

    if (pixmap.save(path, "PNG")) {
        statusBar()->showMessage("Exported: " + path, 3000);
    } else {
        statusBar()->showMessage("Export failed!", 3000);
    }
}

// =========================================================================
// Enhanced About Dialog
// =========================================================================

void MainWindow::openHierarchicalDialog() {
    // Seed the dialog from the engine's current hier config (the engine
    // is authoritative for what's actually running).
    dsca::HierarchicalDialog dlg(engine_.engineConfig(), this);
    if (dlg.exec() == QDialog::Accepted) {
        auto cfg = dlg.engineConfig();
        // Push to engine (triggers initDSP rebuild on next sync).
        engine_.setEngineConfig(cfg);
        // Mirror into AppState so panels (TX, LinkBudget) see the new
        // config. Done under the state mutex.
        {
            std::lock_guard<std::mutex> lock(state_.mtx);
            state_.hier = cfg.hier;
        }
        engine_.onConfigChanged();
        if (tx_panel_)         tx_panel_->refreshFromState();
        if (link_budget_panel_) link_budget_panel_->recompute();
        if (info_panel_)        info_panel_->refresh();
        QString summary;
        if (!cfg.hier.enabled) {
            summary = "Hierarchical: Disabled";
        } else {
            summary = QString("Hierarchical: %1 (α=%2)")
                          .arg(QString::fromUtf8(
                                   dsca::hierarchicalModeName(cfg.hier.mode)))
                          .arg(cfg.hier.alpha, 0, 'f', 2);
        }
        statusBar()->showMessage(summary, 4000);
    }
}

void MainWindow::showAboutDialog() {
    auto* dlg = new QDialog(this);
    dlg->setWindowTitle("About DSCA-NG");
    dlg->setMinimumSize(480, 420);
    dlg->setAttribute(Qt::WA_DeleteOnClose);

    auto* l = new QVBoxLayout(dlg);
    l->setContentsMargins(28, 28, 28, 28);
    l->setSpacing(8);

    // Title
    auto* title = new QLabel(QString("DSCA-NG  v%1.%2.%3")
        .arg(DSCA_VERSION_MAJOR).arg(DSCA_VERSION_MINOR).arg(DSCA_VERSION_PATCH));
    title->setStyleSheet(
        "font-size: 22px; font-weight: 200; color: #F2F2F7; "
        "letter-spacing: 1px;");
    l->addWidget(title);

    // Subtitle
    auto* sub = new QLabel("Digital SCA Next Generation\nOFDM Digital Radio Modem");
    sub->setStyleSheet("color: #AEAEB2; font-size: 13px; line-height: 1.4;");
    l->addWidget(sub);

    l->addSpacing(12);

    // Build info
    auto* info = new QLabel(QString(
        "Build:  %1\n"
        "Framework:  Qt %2 · C++17\n"
        "Tests:  269/269 passing (6 suites)\n\n"
        "Features:\n"
        "  BPSK → 4096-QAM  ·  LDPC FEC 1/4 → 9/10\n"
        "  ORBGRAND near-ML  ·  MMSE + Wiener\n"
        "  PWL LLR  ·  PAPR tone reservation\n"
        "  Hierarchical modulation  ·  VCM superframe\n"
        "  PLS auto-detect  ·  Opus audio codec\n"
        "  SNR/link budget  ·  FM SCA channel model")
        .arg(DSCA_BUILD_DATE)
        .arg(QT_VERSION_STR));
    info->setStyleSheet(
        "color: #8E8E93; font-size: 11px; "
        "font-family: 'SF Mono', Menlo, 'DejaVu Sans Mono', monospace; "
        "line-height: 1.5;");
    l->addWidget(info);

    l->addStretch();

    // Shortcuts reference
    auto* shortcuts = new QLabel(
        "Shortcuts:  F1-F8 Presets · F5 TX · F9/F10 Engine · Ctrl+S/O Config · Ctrl+E Export");
    shortcuts->setStyleSheet("color: #48484E; font-size: 10px;");
    shortcuts->setWordWrap(true);
    l->addWidget(shortcuts);

    l->addSpacing(8);

    // Close button
    auto* close_btn = new QPushButton("Close");
    close_btn->setFixedWidth(80);
    close_btn->setStyleSheet(
        "QPushButton { background: #2C2C34; color: #F2F2F7; "
        "border-radius: 4px; padding: 6px; font-size: 12px; }"
        "QPushButton:hover { background: #3A3A44; }");
    connect(close_btn, &QPushButton::clicked, dlg, &QDialog::close);
    l->addWidget(close_btn, 0, Qt::AlignRight);

    dlg->show();
}

// =========================================================================
// Hardware Audio Device Selection
// =========================================================================

void MainWindow::showDeviceDialog() {
    DeviceDialog dlg(this);
    // Seed the dialog with the engine's current audio config so it opens
    // reflecting reality and "Apply" with no change doesn't disable HW. (#29)
    {
        auto cfg = engine_.engineConfig();
        dlg.setCurrentConfig(cfg.playback_device, cfg.capture_device,
                             cfg.use_hw_audio);
    }
    connect(&dlg, &DeviceDialog::deviceConfigChanged,
            [this](int pb, int cap, bool hw_on) {
        auto cfg = engine_.engineConfig();
        cfg.use_hw_audio      = hw_on;
        cfg.playback_device   = pb;
        cfg.capture_device    = cap;
        engine_.setEngineConfig(cfg);
        engine_.onConfigChanged();

        // Refresh the TX panel's sample-rate combo with the rates this
        // specific device pair actually supports. If HW audio is disabled
        // (loopback), revert to the standard hardcoded list.
        if (hw_on) {
            tx_panel_->setAvailableSampleRates(engine_.supportedSampleRates());
        } else {
            tx_panel_->setAvailableSampleRates({});
        }

        statusBar()->showMessage(
            hw_on ? QString("Hardware audio: PB=%1  CAP=%2").arg(pb).arg(cap)
                  : "Hardware audio: disabled (loopback)", 3000);
    });
    dlg.exec();
}

} // namespace dsca
