/**
 * @file spectrum_widget.hpp
 * @brief Spectrum analyser + waterfall display — the hero widget
 *
 * Top section: live FFT power trace with gradient fill, grid, axis labels,
 *              alarm threshold line, peak hold.
 * Bottom section: scrolling waterfall heatmap (cyan→yellow colormap).
 *
 * All rendering via QPainter with anti-aliasing — no external plot library.
 * The spectrum trace is drawn as a QPainterPath for smooth curves.
 * The waterfall is maintained as a QImage that scrolls by one row per update.
 */
#pragma once

#include "../include/app_state.hpp"
#include "style.hpp"

#include <QWidget>
#include <QImage>
#include <QPainterPath>
#include <QTimer>

namespace dsca {

class SpectrumWidget : public QWidget {
    Q_OBJECT

public:
    explicit SpectrumWidget(AppState& state, QWidget* parent = nullptr);

    // Called by DataBridge signal
    void onSpectrumReady();

    // Display range
    void setDbRange(float min_db, float max_db);
    void setAutoRange(bool on) { auto_range_ = on; }
    bool autoRange() const { return auto_range_; }

    QSize sizeHint() const override { return {600, 320}; }
    QSize minimumSizeHint() const override { return {240, 160}; }

    void setShowMask(bool on) { show_mask_ = on; update(); }
    bool showMask() const { return show_mask_; }
    void clearCursors() { c1_x_ = c2_x_ = -1; update(); }

signals:
    void dbRangeChanged(float min_db, float max_db);

protected:
    void paintEvent(QPaintEvent*) override;
    void resizeEvent(QResizeEvent*) override;
    void wheelEvent(QWheelEvent*) override;
    void mousePressEvent(QMouseEvent*) override;

private:
    void paintSpectrum(QPainter& p, const QRect& r);
    void paintWaterfall(QPainter& p, const QRect& r);
    void paintAxes(QPainter& p, const QRect& spec_r);
    void paintMask(QPainter& p, const QRect& spec_r);
    void paintCursors(QPainter& p, const QRect& spec_r);
    void updateWaterfall();
    QColor dbToWaterfallColor(float db) const;
    float  dbToY(float db, const QRect& r) const;
    /** Convert pixel x within spec_r to frequency in Hz. */
    float  xToHz(int x, const QRect& spec_r) const;

    AppState&  state_;
    QImage     waterfall_img_;
    // Reused across paints (cleared, not reallocated) to avoid churning a
    // ~SPECTRUM_BINS-point path allocation every frame. (#48)
    QPainterPath trace_path_;
    QPainterPath fill_path_;

    float  min_db_     = -80.f;
    float  max_db_     =  10.f;
    bool   auto_range_ = true;

    // Cursors: -1 means inactive. Click sets cursor 1; shift-click sets 2.
    int    c1_x_ = -1;
    int    c2_x_ = -1;

    // FCC SCA emission mask overlay toggle (View menu).
    bool   show_mask_ = false;

    // Split ratio: top fraction is spectrum, rest is waterfall
    static constexpr float SPEC_FRAC = 0.62f;
};

} // namespace dsca
