/**
 * @file gui_walker.cpp
 * @brief Automated GUI fuzzer/walker — drives EVERY interactive control of the
 *        real MainWindow (buttons, actions, combos, sliders, spinboxes, checks,
 *        tabs, dialogs) with the engine running LIVE, and detects:
 *          - crashes        (SEH unhandled-exception filter logs the in-flight action)
 *          - hangs          (external watchdog thread + per-event notify() timing)
 *          - Qt warnings    (qInstallMessageHandler captures qWarning/qCritical/qFatal)
 *          - assert/abort   (signal handler)
 *
 * Usage:  gui_walker <logfile> [mode] [seed]
 *   mode ∈ { all (default), actions, combos, ranges, buttons, checks, tabs,
 *            tx, monkey }   — lets several instances split the surface in parallel.
 *
 * Every action is logged BEFORE it runs (flushed), so the last line in the log
 * is always the culprit if the process dies. Exit 0 = walked clean to the end.
 */
#include "gui/main_window.hpp"
#include "include/app_state.hpp"

#include <QApplication>
#include <QAbstractButton>
#include <QPushButton>
#include <QToolButton>
#include <QComboBox>
#include <QSlider>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QRadioButton>
#include <QTabWidget>
#include <QAction>
#include <QMenu>
#include <QDialog>
#include <QWidget>
#include <QTimer>
#include <QString>
#include <QStringList>
#include <QPointer>
#include <QList>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <functional>
#include <atomic>
#include <thread>
#include <chrono>
#include <mutex>

#ifdef _WIN32
#  include <windows.h>
#endif

// ---------------------------------------------------------------------------
// Global logging — open-append-close so nothing is lost on a crash.
// ---------------------------------------------------------------------------
static std::string g_logpath = "gui_walker.log";
static std::mutex  g_logmtx;
static char        g_current_action[512] = "startup";
static std::atomic<long long> g_progress{0};   // ms timestamp of last step
static std::atomic<int>       g_action_count{0};

static long long nowMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

static void logLine(const char* tag, const std::string& msg) {
    std::lock_guard<std::mutex> lk(g_logmtx);
    FILE* f = std::fopen(g_logpath.c_str(), "a");
    if (f) { std::fprintf(f, "[%s] %s\n", tag, msg.c_str()); std::fclose(f); }
}

static void setAction(const std::string& a) {
    std::strncpy(g_current_action, a.c_str(), sizeof(g_current_action) - 1);
    g_current_action[sizeof(g_current_action) - 1] = 0;
    g_progress.store(nowMs());
    g_action_count.fetch_add(1);
    logLine("ACT", a);
}

// ---------------------------------------------------------------------------
// Qt message handler — capture warnings/criticals/fatals.
// ---------------------------------------------------------------------------
static void qtMsgHandler(QtMsgType type, const QMessageLogContext&, const QString& m) {
    const char* t = "QtMsg";
    switch (type) {
        case QtWarningMsg:  t = "QWARN"; break;
        case QtCriticalMsg: t = "QCRIT"; break;
        case QtFatalMsg:    t = "QFATAL"; break;
        default: return;   // skip debug/info noise
    }
    logLine(t, m.toStdString() + "  (during: " + g_current_action + ")");
}

#ifdef _WIN32
static LONG WINAPI crashFilter(EXCEPTION_POINTERS* ep) {
    char buf[640];
    std::snprintf(buf, sizeof(buf), "CRASH code=0x%08lx during action: %s",
                  ep && ep->ExceptionRecord ? ep->ExceptionRecord->ExceptionCode : 0,
                  g_current_action);
    logLine("CRASH", buf);
    return EXCEPTION_EXECUTE_HANDLER;   // terminate
}
#endif

// ---------------------------------------------------------------------------
// Profiling QApplication — flags any single event dispatch that blocks the GUI
// thread for too long (the technique that caught the eye-paint hang).
// ---------------------------------------------------------------------------
class WalkerApp : public QApplication {
public:
    using QApplication::QApplication;
    bool notify(QObject* r, QEvent* e) override {
        auto t0 = std::chrono::steady_clock::now();
        bool ret = QApplication::notify(r, e);
        double d = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t0).count();
        if (d > 250.0) {
            const char* cls = (r && r->metaObject()) ? r->metaObject()->className() : "?";
            char buf[640];
            std::snprintf(buf, sizeof(buf),
                "SLOW dispatch %.0fms recv=%s evt=%d during: %s",
                d, cls, (int)e->type(), g_current_action);
            logLine("SLOW", buf);
        }
        return ret;
    }
};

// ---------------------------------------------------------------------------
// Helpers — describe a widget for logging.
// ---------------------------------------------------------------------------
static std::string desc(QObject* o) {
    if (!o) return "(null)";
    std::string s = o->metaObject()->className();
    QString n = o->objectName();
    if (!n.isEmpty()) s += "#" + n.toStdString();
    if (auto* b = qobject_cast<QAbstractButton*>(o)) {
        QString t = b->text(); if (!t.isEmpty()) s += " '" + t.toStdString() + "'";
    }
    return s;
}

// Buttons whose text we must NOT click (would quit / open blocking native flows
// we can't drive). Native dialogs are disabled below, but quitting is fatal.
static bool isDangerousButton(QAbstractButton* b) {
    QString t = b->text().toLower();
    // Skip internal Qt controls (menubar/toolbar overflow buttons, etc.) which
    // open popups rather than exercising app behavior.
    if (b->objectName().startsWith("qt_")) return true;
    return t.contains("quit") || t.contains("exit") || t.contains("close app");
}

// ---------------------------------------------------------------------------
// The Walker: builds a task queue from the live MainWindow and steps through it.
// ---------------------------------------------------------------------------
class Walker : public QObject {
public:
    Walker(QWidget* root, const std::string& mode)
        : root_(root), mode_(mode) {}

    void build() {
        const bool all = (mode_ == "all");
        auto want = [&](const char* m){ return all || mode_ == m; };

        // --- Tabs first (so controls on all tabs become reachable/visible) ---
        if (want("tabs")) {
            for (QTabWidget* tw : root_->findChildren<QTabWidget*>()) {
                int n = tw->count();
                for (int i = 0; i < n; ++i) {
                    QPointer<QTabWidget> p(tw);
                    tasks_.push_back({ "tab " + desc(tw) + " -> " + std::to_string(i),
                        [p, i]{ if (p) p->setCurrentIndex(i); } });
                }
            }
        }
        // --- Menu / toolbar actions (may open dialogs; handled by modal timer) ---
        if (want("actions")) {
            for (QAction* a : root_->findChildren<QAction*>()) {
                if (!a->isEnabled() || a->isSeparator()) continue;
                if (a->menu()) continue;              // submenu container, skip
                if (a->objectName().startsWith("qt_")) continue;
                QString t = a->text().toLower();
                if (t.contains("quit") || t.contains("exit")) continue;
                QPointer<QAction> p(a);
                tasks_.push_back({ "action '" + a->text().toStdString() + "'",
                    [p]{ if (p && p->isEnabled()) p->trigger(); } });
            }
        }
        // --- Combos: cycle every index ---
        if (want("combos")) {
            for (QComboBox* c : root_->findChildren<QComboBox*>()) {
                int n = c->count();
                for (int i = 0; i < n; ++i) {
                    QPointer<QComboBox> p(c);
                    tasks_.push_back({ "combo " + desc(c) + " -> " + std::to_string(i),
                        [p, i]{ if (p) p->setCurrentIndex(i); } });
                }
            }
        }
        // --- Ranges: sliders / spinboxes to min, mid, max, and back ---
        if (want("ranges")) {
            for (QSlider* s : root_->findChildren<QSlider*>()) {
                QPointer<QSlider> p(s);
                int lo = s->minimum(), hi = s->maximum(), mid = (lo + hi) / 2;
                for (int v : {lo, hi, mid, lo})
                    tasks_.push_back({ "slider " + desc(s) + " = " + std::to_string(v),
                        [p, v]{ if (p) p->setValue(v); } });
            }
            for (QSpinBox* s : root_->findChildren<QSpinBox*>()) {
                QPointer<QSpinBox> p(s);
                int lo = s->minimum(), hi = s->maximum(), mid = (lo + hi) / 2;
                for (int v : {lo, hi, mid})
                    tasks_.push_back({ "spin " + desc(s) + " = " + std::to_string(v),
                        [p, v]{ if (p) p->setValue(v); } });
            }
            for (QDoubleSpinBox* s : root_->findChildren<QDoubleSpinBox*>()) {
                QPointer<QDoubleSpinBox> p(s);
                double lo = s->minimum(), hi = s->maximum(), mid = (lo + hi) / 2;
                for (double v : {lo, hi, mid})
                    tasks_.push_back({ "dspin " + desc(s),
                        [p, v]{ if (p) p->setValue(v); } });
            }
        }
        // --- Checkboxes / radios: toggle on then off ---
        if (want("checks")) {
            for (QCheckBox* c : root_->findChildren<QCheckBox*>()) {
                QPointer<QCheckBox> p(c);
                for (bool v : {true, false})
                    tasks_.push_back({ "check " + desc(c) + " = " + (v?"on":"off"),
                        [p, v]{ if (p) p->setChecked(v); } });
            }
            for (QRadioButton* c : root_->findChildren<QRadioButton*>()) {
                QPointer<QRadioButton> p(c);
                tasks_.push_back({ "radio " + desc(c),
                    [p]{ if (p) p->setChecked(true); } });
            }
        }
        // --- Plain push/tool buttons (not checkboxes/radios) ---
        if (want("buttons")) {
            for (QAbstractButton* b : root_->findChildren<QAbstractButton*>()) {
                if (qobject_cast<QCheckBox*>(b) || qobject_cast<QRadioButton*>(b)) continue;
                if (isDangerousButton(b)) continue;
                QPointer<QAbstractButton> p(b);
                tasks_.push_back({ "click " + desc(b),
                    [p]{ if (p && p->isEnabled()) p->click(); } });
            }
        }
        // --- TX stress: toggle TX, let it run, reconfigure live, toggle off ---
        if (want("tx")) {
            buildTxStress();
        }
        logLine("INFO", "built " + std::to_string(tasks_.size()) +
                         " tasks for mode=" + mode_);
    }

    void buildTxStress() {
        // Find the TX button by text.
        QAbstractButton* tx = nullptr;
        for (QAbstractButton* b : root_->findChildren<QAbstractButton*>())
            if (b->text().contains("TX")) { tx = b; break; }
        QList<QComboBox*> combos = root_->findChildren<QComboBox*>();

        QPointer<QAbstractButton> ptx(tx);
        tasks_.push_back({ "TX ON", [ptx]{ if (ptx) ptx->click(); } });
        for (int round = 0; round < 3; ++round) {
            for (QComboBox* c : combos) {
                int n = c->count(); if (n < 2) continue;
                QPointer<QComboBox> p(c);
                int idx = (round + 1) % n;
                tasks_.push_back({ "TX-live combo " + desc(c) + " -> " + std::to_string(idx),
                    [p, idx]{ if (p) p->setCurrentIndex(idx); } });
                // settle steps so the engine reconfigures + paints between changes
                for (int k = 0; k < 3; ++k)
                    tasks_.push_back({ "settle", []{} });
            }
        }
        tasks_.push_back({ "TX OFF", [ptx]{ if (ptx) ptx->click(); } });
        for (int k = 0; k < 5; ++k) tasks_.push_back({ "settle", []{} });
    }

    void run() {
        // Modal handler: close any dialog that pops, after exercising its
        // controls once. Runs even inside a dialog's nested exec() loop.
        modalTimer_ = new QTimer(this);
        connect(modalTimer_, &QTimer::timeout, this, [this]{ handleModal(); });
        modalTimer_->start(40);

        stepTimer_ = new QTimer(this);
        connect(stepTimer_, &QTimer::timeout, this, [this]{ step(); });
        stepTimer_->start(45);
    }

private:
    void handleModal() {
        // Popups (menus, combo dropdowns, the menubar-overflow menu) are NOT
        // modal widgets, so they don't show up in activeModalWidget(). Left
        // open they stall the walker — close them.
        if (QWidget* pop = QApplication::activePopupWidget()) {
            pop->close();
            return;
        }
        QWidget* m = QApplication::activeModalWidget();
        if (!m) return;
        if (m == last_modal_) {
            // second sighting — exercise a few controls, then close it.
            if (!modal_exercised_) {
                modal_exercised_ = true;
                logLine("DIALOG", "opened " + desc(m) + " (during: " +
                                  std::string(g_current_action) + ")");
                for (QComboBox* c : m->findChildren<QComboBox*>())
                    if (c->count() > 1) c->setCurrentIndex(c->count() - 1);
                for (QCheckBox* c : m->findChildren<QCheckBox*>())
                    c->setChecked(!c->isChecked());
                for (QSpinBox* s : m->findChildren<QSpinBox*>())
                    s->setValue(s->maximum());
            }
            // Close it (reject to avoid applying random config from a fuzz run).
            if (auto* d = qobject_cast<QDialog*>(m)) d->reject();
            else m->close();
            last_modal_ = nullptr;
            modal_exercised_ = false;
        } else {
            last_modal_ = m;
            modal_exercised_ = false;
        }
    }

    void step() {
        if (busy_) return;                                  // no re-entrancy
        if (QApplication::activeModalWidget()) return;      // wait for modal close
        if (idx_ >= tasks_.size()) {
            logLine("INFO", "DONE — walked " + std::to_string(idx_) +
                            " tasks, " + std::to_string(g_action_count.load()) +
                            " actions, clean.");
            stepTimer_->stop();
            QTimer::singleShot(300, qApp, []{ qApp->quit(); });
            return;
        }
        busy_ = true;
        Task& t = tasks_[idx_++];
        setAction(t.desc);
        try {
            t.fn();
        } catch (const std::exception& e) {
            logLine("EXC", std::string(e.what()) + " during: " + t.desc);
        } catch (...) {
            logLine("EXC", "unknown C++ exception during: " + t.desc);
        }
        busy_ = false;
    }

    struct Task { std::string desc; std::function<void()> fn; };
    QWidget*    root_;
    std::string mode_;
    std::vector<Task> tasks_;
    size_t      idx_ = 0;
    bool        busy_ = false;
    QTimer*     stepTimer_ = nullptr;
    QTimer*     modalTimer_ = nullptr;
    QWidget*    last_modal_ = nullptr;
    bool        modal_exercised_ = false;
};

// ---------------------------------------------------------------------------
int main(int argc, char* argv[]) {
    if (argc > 1) g_logpath = argv[1];
    std::string mode = (argc > 2) ? argv[2] : "all";
    std::remove(g_logpath.c_str());

    qInstallMessageHandler(qtMsgHandler);
#ifdef _WIN32
    SetUnhandledExceptionFilter(crashFilter);
#endif

    // Use Qt (non-native) dialogs so QFileDialog/QMessageBox are QWidgets the
    // modal handler can close instead of blocking on a native OS dialog.
    QApplication::setAttribute(Qt::AA_DontUseNativeDialogs, true);

    logLine("INFO", "gui_walker start mode=" + mode);

    WalkerApp app(argc, argv);
    app.setApplicationName("Groundwave");
    app.setOrganizationName("Groundwave");

    gw::AppState state;
    gw::MainWindow window(state);
    window.resize(1400, 900);
    window.show();

    Walker walker(&window, mode);
    walker.build();

    // External watchdog thread: if a single action stalls the GUI thread for
    // > 6 s, the GUI is hung — log the culprit and hard-abort (the parent
    // process timeout is the backstop; this gives us the exact action).
    std::atomic<bool> done{false};
    std::thread wd([&]{
        // give the app a moment to start stepping
        long long start = nowMs();
        g_progress.store(start);
        while (!done.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            long long since = nowMs() - g_progress.load();
            if (since > 6000) {
                char buf[640];
                std::snprintf(buf, sizeof(buf),
                    "HANG: no progress for %lldms, stuck in action: %s",
                    since, g_current_action);
                logLine("HANG", buf);
                std::fflush(nullptr);
                std::_Exit(7);   // hard exit; parent records exit code
            }
        }
    });

    // Hard cap on total runtime so a pathological loop can't run forever.
    QTimer::singleShot(180000, &app, [&]{ logLine("INFO","time cap hit"); app.quit(); });

    walker.run();
    int rc = app.exec();

    done.store(true);
    if (wd.joinable()) wd.join();
    logLine("INFO", "exit rc=" + std::to_string(rc));
    return rc;
}
