/**
 * @file style.hpp
 * @brief Groundwave design system — dark broadcast instrument aesthetic
 *
 * Color palette inspired by Apple's dark mode HIG applied to
 * broadcast test & measurement equipment (Tektronix / R&S palette).
 *
 * Three background layers, semantic signal colors, SF Pro typography
 * with Menlo monospace fallback for numeric readouts.
 */
#pragma once

#include <QColor>
#include <QString>

namespace gw::style {

// =========================================================================
// Color Tokens
// =========================================================================

// --- Backgrounds (3 layers + raised) ---
inline const QColor BG_BASE     { 10,  10,  13};   // #0A0A0D  deepest
inline const QColor BG_PANEL    { 16,  16,  20};   // #101014  panel chrome
inline const QColor BG_SURFACE  { 22,  22,  28};   // #16161C  widget surface
inline const QColor BG_RAISED   { 30,  30,  38};   // #1E1E26  button / input

// --- Borders ---
inline const QColor BORDER_FAINT { 36,  36,  46, 160};  // barely visible
inline const QColor BORDER_DIM   { 44,  44,  56, 200};  // panel edges
inline const QColor BORDER_NORM  { 58,  58,  74, 220};  // active / hover

// --- Text ---
inline const QColor TEXT_PRIMARY   {242, 242, 247};  // #F2F2F7  white-ish
inline const QColor TEXT_SECONDARY {142, 142, 147};  // #8E8E93  muted
inline const QColor TEXT_TERTIARY  { 72,  72,  74};  // #484848  disabled / labels

// --- Semantic Signal Colors (Apple HIG) ---
inline const QColor C_SIGNAL  {  0, 180, 255};  // electric blue  — RF / data
inline const QColor C_OK      { 48, 209,  88};  // Apple green    — locked / pass
inline const QColor C_WARNING {255, 159,  10};  // Apple amber    — caution
inline const QColor C_ERROR   {255,  69,  58};  // Apple red      — alarm / fail
inline const QColor C_PILOT   {255, 149,   0};  // Apple orange   — pilot subcarrier

// --- Spectrum Plot Colors ---
inline const QColor SPEC_TRACE    {  0, 200, 255};          // trace line
inline const QColor SPEC_FILL_TOP {  0, 180, 255,  55};     // gradient top
inline const QColor SPEC_FILL_BOT {  0,  80, 140,   5};     // gradient bottom
inline const QColor SPEC_GRID     { 40,  40,  52, 100};     // grid lines
inline const QColor SPEC_THRESH   {255, 159,  10,  80};     // alarm threshold line
inline const QColor WFALL_COLD    {  8,  12,  28};          // waterfall cold
inline const QColor WFALL_HOT     {255, 200,  50};          // waterfall hot

// --- Meter Colors ---
inline const QColor METER_GREEN  { 48, 209,  88};
inline const QColor METER_AMBER  {255, 159,  10};
inline const QColor METER_RED    {255,  69,  58};
inline const QColor METER_BG     { 14,  14,  18};
inline const QColor METER_PEAK   {255, 255, 120};

// =========================================================================
// Semantic helpers
// =========================================================================

inline QColor statusColor(float snr_db, float threshold) {
    if (snr_db >= threshold + 6.f) return C_OK;
    if (snr_db >= threshold)       return C_WARNING;
    return C_ERROR;
}

enum class Theme { Dark, Light };

/** Light-theme stylesheet (overrides on top of the dark base). */
inline QString buildLightStyleSheet() {
    return R"(
/* ===== Light theme overrides ===== */
QWidget {
    background-color: #F5F5F7;
    color: #1C1C1E;
    selection-background-color: #007AFF40;
    selection-color: #1C1C1E;
}
QMainWindow             { background: #F5F5F7; }
QMainWindow::separator  { background: #D8D8DC; }
QMenuBar                { background: #FFFFFF; border-bottom: 1px solid #D8D8DC; }
QMenuBar::item:selected { background: #E5E5EA; }
QMenu                   { background: #FFFFFF; border: 1px solid #D8D8DC; }
QMenu::item:selected    { background: #007AFF20; }
QStatusBar              { background: #FFFFFF; border-top: 1px solid #D8D8DC; }
QDockWidget             { background: #F5F5F7; titlebar-close-icon: none; }
QDockWidget::title      { background: #ECECEE; padding: 4px 8px; }
QFrame                  { background: #F5F5F7; }
QGroupBox               { background: #FFFFFF; border: 1px solid #D8D8DC; border-radius: 4px; margin-top: 1ex; }
QGroupBox::title        { color: #1C1C1E; padding: 0 6px; }
QPushButton             { background: #FFFFFF; color: #1C1C1E; border: 1px solid #D8D8DC; border-radius: 4px; padding: 4px 10px; }
QPushButton:hover       { background: #ECECEE; }
QPushButton:pressed     { background: #D8D8DC; }
QPushButton[active="true"] { background: #007AFF; color: #FFFFFF; }
QLabel                  { background: transparent; color: #1C1C1E; }
QLabel#sectionLabel     { color: #6E6E73; font-size: 11px; padding: 4px 0; }
QLabel#valueLabel       { color: #1C1C1E; }
QComboBox, QSpinBox, QDoubleSpinBox {
    background: #FFFFFF; color: #1C1C1E;
    border: 1px solid #D8D8DC; border-radius: 4px; padding: 2px 6px;
}
QComboBox QAbstractItemView { background: #FFFFFF; color: #1C1C1E; }
QSlider::groove:horizontal { background: #D8D8DC; height: 4px; }
QSlider::handle:horizontal { background: #007AFF; width: 14px; margin: -5px 0; border-radius: 7px; }
QSlider::sub-page:horizontal { background: #007AFF; }
QCheckBox::indicator { background: #FFFFFF; border: 1px solid #D8D8DC; }
QCheckBox::indicator:checked { background: #007AFF; }
QProgressBar { background: #ECECEE; color: #1C1C1E; border: 1px solid #D8D8DC; border-radius: 3px; text-align: center; }
QProgressBar::chunk { background: #007AFF; }
QTableWidget { background: #FFFFFF; alternate-background-color: #F5F5F7; gridline-color: #D8D8DC; }
QHeaderView::section { background: #ECECEE; color: #1C1C1E; padding: 4px; border: none; border-right: 1px solid #D8D8DC; }
QToolTip { background: #FFFFFF; color: #1C1C1E; border: 1px solid #D8D8DC; }
)";
}

// =========================================================================
// Global Stylesheet
// =========================================================================

inline QString buildStyleSheet() {
    return R"(

/* ===== Base ===== */
QWidget {
    background-color: #0A0A0D;
    color: #F2F2F7;
    font-family: -apple-system, "SF Pro Text", "Segoe UI", "Helvetica Neue",
                 "DejaVu Sans", Arial, sans-serif;
    font-size: 12px;
    selection-background-color: #0099FF40;
    selection-color: #F2F2F7;
}

/* ===== Main Window ===== */
QMainWindow {
    background: #0A0A0D;
}
QMainWindow::separator {
    background: #1C1C24;
    width: 1px;
    height: 1px;
}
QMainWindow::separator:hover {
    background: #0099FF60;
}

/* ===== Menu Bar ===== */
QMenuBar {
    background: #0A0A0D;
    border-bottom: 1px solid #1C1C24;
    padding: 2px 4px;
    spacing: 2px;
}
QMenuBar::item {
    background: transparent;
    padding: 4px 10px;
    border-radius: 5px;
    color: #8E8E93;
    font-size: 12px;
}
QMenuBar::item:selected, QMenuBar::item:pressed {
    background: #1C1C28;
    color: #F2F2F7;
}
QMenu {
    background: #18181F;
    border: 1px solid #2C2C38;
    border-radius: 8px;
    padding: 4px;
}
QMenu::item {
    padding: 6px 28px 6px 16px;
    border-radius: 5px;
    font-size: 12px;
}
QMenu::item:selected {
    background: #0099FF22;
    color: #F2F2F7;
}
QMenu::separator {
    height: 1px;
    background: #1C1C28;
    margin: 3px 8px;
}

/* ===== Dock Widgets ===== */
QDockWidget {
    titlebar-close-icon: none;
    titlebar-normal-icon: none;
    font-size: 10px;
    font-weight: 600;
    color: #8E8E93;
    letter-spacing: 0.10em;
    text-transform: uppercase;
}
QDockWidget::title {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                                 stop:0 #101014, stop:1 #0C0C10);
    padding: 8px 14px 7px;
    border-bottom: 1px solid #1C1C24;
    text-align: left;
}
QDockWidget::close-button, QDockWidget::float-button {
    background: transparent;
    border: none;
    padding: 2px;
    border-radius: 4px;
    icon-size: 12px;
}
QDockWidget::close-button:hover, QDockWidget::float-button:hover {
    background: #2C2C38;
}

/* ===== Scroll Bars ===== */
QScrollBar:vertical {
    background: transparent;
    width: 6px;
    margin: 0;
}
QScrollBar::handle:vertical {
    background: #2C2C3A;
    border-radius: 3px;
    min-height: 24px;
}
QScrollBar::handle:vertical:hover { background: #3C3C4E; }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0px; }
QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical { background: none; }
QScrollBar:horizontal {
    background: transparent;
    height: 6px;
}
QScrollBar::handle:horizontal {
    background: #2C2C3A;
    border-radius: 3px;
    min-width: 24px;
}

/* ===== Buttons ===== */
QPushButton {
    background: #1C1C26;
    border: 1px solid #2C2C3A;
    border-radius: 6px;
    padding: 6px 14px;
    color: #E2E2E7;
    font-size: 12px;
    min-height: 26px;
}
QPushButton:hover {
    background: #26262F;
    border-color: #44445A;
    color: #F2F2F7;
}
QPushButton:pressed {
    background: #14141C;
    border-color: #222230;
}
QPushButton:focus {
    border-color: #0099FFB0;
    outline: none;
}
QPushButton:checked {
    background: #0A2A4A;
    border-color: #0099FF;
    color: #66BBFF;
}
QPushButton:disabled {
    color: #48484A;
    border-color: #1C1C24;
    background: #14141A;
}
QPushButton#txBtn[active="true"] {
    background: #0C2C16;
    border: 1px solid #30D158;
    color: #30D158;
}
QPushButton#txBtn[active="false"] {
    background: #1C1C26;
    border: 1px solid #2C2C3A;
    color: #8E8E93;
}
QPushButton#accentBtn {
    background: #003880;
    border: 1px solid #0055C0;
    color: #F2F2F7;
}
QPushButton#accentBtn:hover {
    background: #004499;
    border-color: #0066D0;
}
QPushButton#dangerBtn {
    background: #300000;
    border: 1px solid #FF453A;
    color: #FF453A;
}

/* ===== Combo Box ===== */
QComboBox {
    background: #1C1C26;
    border: 1px solid #2C2C3A;
    border-radius: 6px;
    padding: 4px 8px 4px 10px;
    min-height: 28px;
    color: #F2F2F7;
}
QComboBox:hover {
    border-color: #3A3A4C;
    background: #22222C;
}
QComboBox:focus {
    border-color: #0099FFB0;
    outline: none;
}
QComboBox:disabled {
    color: #48484A;
    background: #14141A;
    border-color: #1C1C24;
}
QComboBox::drop-down {
    border: none;
    width: 22px;
    subcontrol-position: right center;
}
QComboBox::down-arrow {
    width: 10px;
    height: 6px;
    image: none;
    border-left: 4px solid transparent;
    border-right: 4px solid transparent;
    border-top: 5px solid #8E8E93;
}
QComboBox QAbstractItemView {
    background: #18181F;
    border: 1px solid #2C2C38;
    border-radius: 6px;
    selection-background-color: #0099FF28;
    outline: none;
    padding: 3px;
}
QComboBox QAbstractItemView::item {
    padding: 5px 10px;
    border-radius: 4px;
}

/* ===== Sliders ===== */
QSlider::groove:horizontal {
    height: 4px;
    background: #1C1C26;
    border: 1px solid #22222C;
    border-radius: 2px;
}
QSlider::handle:horizontal {
    width: 14px;
    height: 14px;
    border-radius: 7px;
    background: #E8E8EF;
    border: 1px solid #3A3A4C;
    margin: -6px 0;
}
QSlider::handle:horizontal:hover {
    background: #FFFFFF;
    border-color: #0099FF;
}
QSlider::handle:horizontal:pressed {
    background: #66BBFF;
    border-color: #0099FF;
}
QSlider::handle:horizontal:disabled {
    background: #48484A;
    border-color: #2C2C36;
}
QSlider::sub-page:horizontal {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                                 stop:0 #0066B0, stop:1 #0099FF);
    border-radius: 2px;
}
QSlider::sub-page:horizontal:disabled {
    background: #2C2C36;
}
QSlider::groove:vertical {
    width: 3px;
    background: #2C2C3A;
    border-radius: 2px;
}
QSlider::handle:vertical {
    width: 14px;
    height: 14px;
    border-radius: 7px;
    background: #E8E8EF;
    border: 1px solid #3A3A4C;
    margin: 0 -6px;
}
QSlider::sub-page:vertical {
    background: #0099FF;
    border-radius: 2px;
}

/* ===== Line Edit / Input ===== */
QLineEdit, QSpinBox, QDoubleSpinBox {
    background: #14141A;
    border: 1px solid #26262E;
    border-radius: 6px;
    padding: 4px 8px;
    color: #F2F2F7;
    min-height: 26px;
    font-family: "SF Mono", "Menlo", "DejaVu Sans Mono", monospace;
    font-size: 12px;
}
QLineEdit:hover, QSpinBox:hover, QDoubleSpinBox:hover {
    border-color: #3A3A4C;
}
QLineEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus {
    border-color: #0099FFB0;
    background: #1A1A22;
}
QLineEdit:disabled, QSpinBox:disabled, QDoubleSpinBox:disabled {
    color: #48484A;
    background: #0E0E14;
    border-color: #1C1C24;
}
QSpinBox::up-button, QSpinBox::down-button,
QDoubleSpinBox::up-button, QDoubleSpinBox::down-button {
    background: #22222C;
    border: none;
    border-radius: 3px;
    width: 18px;
}
QSpinBox::up-button:hover, QDoubleSpinBox::up-button:hover,
QSpinBox::down-button:hover, QDoubleSpinBox::down-button:hover {
    background: #2E2E3A;
}
QSpinBox::up-button:pressed, QDoubleSpinBox::up-button:pressed,
QSpinBox::down-button:pressed, QDoubleSpinBox::down-button:pressed {
    background: #0099FF60;
}
/* Render up/down arrows as CSS-border triangles in light gray.
   Without these explicit declarations Qt6 uses theme-dependent images
   that come out black-on-black against the dark background. */
QSpinBox::up-arrow, QDoubleSpinBox::up-arrow {
    image: none;
    border-left: 4px solid transparent;
    border-right: 4px solid transparent;
    border-bottom: 5px solid #C7C7CC;
    width: 0; height: 0;
}
QSpinBox::up-arrow:disabled, QDoubleSpinBox::up-arrow:disabled {
    border-bottom: 5px solid #48484A;
}
QSpinBox::down-arrow, QDoubleSpinBox::down-arrow {
    image: none;
    border-left: 4px solid transparent;
    border-right: 4px solid transparent;
    border-top: 5px solid #C7C7CC;
    width: 0; height: 0;
}
QSpinBox::down-arrow:disabled, QDoubleSpinBox::down-arrow:disabled {
    border-top: 5px solid #48484A;
}

/* ===== Labels ===== */
QLabel {
    background: transparent;
    color: #8E8E93;
    font-size: 11px;
}
QLabel#valueLabel {
    color: #F2F2F7;
    font-size: 13px;
    font-family: "SF Mono", "Menlo", "DejaVu Sans Mono", monospace;
}
QLabel#bigValue {
    color: #F2F2F7;
    font-size: 22px;
    font-family: "SF Mono", "Menlo", "DejaVu Sans Mono", monospace;
    font-weight: 300;
}
QLabel#sectionTitle {
    color: #6E6E78;
    font-size: 10px;
    font-weight: 600;
    letter-spacing: 0.10em;
    padding: 6px 0 3px 0;
}
/* Backward-compat alias: many panels use "sectionLabel" rather than the
   newer "sectionTitle" object name. Keep both styles in sync so old
   callsites don't visually regress. */
QLabel#sectionLabel {
    color: #6E6E78;
    font-size: 10px;
    font-weight: 600;
    letter-spacing: 0.10em;
    padding: 6px 0 3px 0;
}
QLabel#statusGood  { color: #30D158; }
QLabel#statusWarn  { color: #FF9F0A; }
QLabel#statusError { color: #FF453A; }
QLabel#statusMuted { color: #8E8E93; }

/* ===== Group Box ===== */
QGroupBox {
    border: 1px solid #1E1E28;
    border-radius: 8px;
    margin-top: 16px;
    padding: 12px 12px 10px;
    font-size: 10px;
    font-weight: 600;
    color: #6E6E78;
    letter-spacing: 0.10em;
    text-transform: uppercase;
}
QGroupBox::title {
    subcontrol-origin: margin;
    subcontrol-position: top left;
    left: 12px;
    padding: 0 8px;
    background: #0A0A0D;
}
QGroupBox:disabled {
    color: #2C2C36;
    border-color: #14141A;
}

/* ===== Check Box ===== */
QCheckBox {
    color: #8E8E93;
    spacing: 7px;
    font-size: 12px;
}
QCheckBox::indicator {
    width: 16px;
    height: 16px;
    border-radius: 4px;
    border: 1px solid #3A3A4C;
    background: #1C1C26;
}
QCheckBox::indicator:checked {
    background: #0099FF;
    border-color: #0099FF;
}
QCheckBox::indicator:hover {
    border-color: #5A5A6C;
}

/* ===== Table / List ===== */
QTableWidget, QListWidget, QTreeWidget {
    background: #101014;
    border: 1px solid #1C1C24;
    border-radius: 8px;
    gridline-color: #1C1C24;
    font-size: 12px;
    outline: none;
}
QTableWidget::item, QListWidget::item, QTreeWidget::item {
    padding: 5px 8px;
    border: none;
}
QTableWidget::item:selected, QListWidget::item:selected, QTreeWidget::item:selected {
    background: #0099FF22;
    color: #F2F2F7;
}
QTableWidget::item:hover, QListWidget::item:hover, QTreeWidget::item:hover {
    background: #1C1C28;
}
QHeaderView::section {
    background: #14141A;
    border: none;
    border-right: 1px solid #1C1C24;
    border-bottom: 1px solid #1C1C24;
    padding: 5px 10px;
    color: #48484E;
    font-size: 10px;
    font-weight: 600;
    letter-spacing: 0.06em;
}

/* ===== Status Bar ===== */
QStatusBar {
    background: #08080B;
    border-top: 1px solid #1C1C24;
    color: #8E8E93;
    font-size: 11px;
    padding: 2px 4px;
}
QStatusBar::item { border: none; }
QStatusBar QLabel {
    padding: 1px 8px;
    color: #8E8E93;
}

/* ===== Splitter ===== */
QSplitter::handle {
    background: #1C1C24;
}
QSplitter::handle:hover {
    background: #0099FF60;
}
QSplitter::handle:horizontal { width: 1px; }
QSplitter::handle:vertical   { height: 1px; }

/* ===== Tooltip ===== */
QToolTip {
    background: #18181F;
    border: 1px solid #2C2C38;
    border-radius: 6px;
    color: #F2F2F7;
    font-size: 11px;
    padding: 6px 10px;
    opacity: 240;
}

/* ===== Dialog ===== */
QDialog {
    background: #0E0E14;
    border: 1px solid #1C1C28;
}

/* ===== Tab Bar ===== */
QTabWidget::pane {
    border: 1px solid #1C1C24;
    border-radius: 0 0 8px 8px;
    background: #0E0E14;
    top: -1px;
}
QTabWidget::tab-bar {
    alignment: left;
}
QTabBar::tab {
    background: transparent;
    color: #8E8E93;
    padding: 8px 18px;
    font-size: 11px;
    font-weight: 500;
    border-bottom: 2px solid transparent;
    margin-right: 1px;
}
QTabBar::tab:selected {
    color: #F2F2F7;
    border-bottom: 2px solid #0099FF;
}
QTabBar::tab:hover:!selected {
    color: #C8C8CF;
    border-bottom: 2px solid #2C2C3A;
}

/* ===== Dock Tab Bar (when docks are tabified together) ===== */
QTabBar::tab[role="dock"] {
    background: #0C0C10;
    padding: 6px 14px;
}

)";
}

// =========================================================================
// Dimension constants (8px grid)
// =========================================================================

namespace dim {
    constexpr int PANEL_PADDING   = 12;
    constexpr int SECTION_SPACING = 16;
    constexpr int ITEM_SPACING    =  6;
    constexpr int DOCK_TITLE_H    = 30;
    constexpr int CONTROL_H       = 28;  // standard control height
    constexpr int LABEL_H         = 16;
    constexpr int SEPARATOR_H     =  1;
}

} // namespace gw::style
