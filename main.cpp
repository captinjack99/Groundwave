/**
 * @file main.cpp
 * @brief DSCA-NG application entry point
 */
#include "gui/main_window.hpp"
#include "gui/style.hpp"
#include "include/app_state.hpp"

#include <QApplication>
#include <QSurfaceFormat>
#include <QFont>
#include <QFontDatabase>

int main(int argc, char* argv[]) {
    // HiDPI is always enabled in Qt 6 — no setAttribute needed
#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
    QApplication::setAttribute(Qt::AA_EnableHighDpiScaling, true);
    QApplication::setAttribute(Qt::AA_UseHighDpiPixmaps,    true);
#endif

    QApplication app(argc, argv);
    app.setApplicationName("DSCA-NG");
    app.setOrganizationName("DSCA-NG");
    app.setApplicationVersion("2.0.0");

    // Apply global stylesheet
    app.setStyleSheet(dsca::style::buildStyleSheet());

    // System font — prefer SF Pro on macOS, Segoe UI on Windows, system sans elsewhere
    QFont default_font = app.font();
    default_font.setFamily("-apple-system, SF Pro Text, Segoe UI, Helvetica Neue, DejaVu Sans");
    default_font.setPixelSize(12);
    default_font.setHintingPreference(QFont::PreferFullHinting);
    app.setFont(default_font);

    // Application-wide state (owns all DSP data, no Qt deps)
    dsca::AppState state;

    // Main window
    dsca::MainWindow window(state);
    window.show();

    return app.exec();
}
