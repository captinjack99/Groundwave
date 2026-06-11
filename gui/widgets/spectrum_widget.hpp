/**
 * @file spectrum_widget.hpp
 * @brief Spectrum analyser + waterfall display — the hero widget
 *
 * Top section: live FFT power trace (eased at the display rate) with
 *              gradient fill, grid, axis labels, alarm threshold line,
 *              optional peak-hold and averaged traces.
 * Bottom section: scrolling waterfall heatmap.
 *
 * RENDERING. The widget is a QOpenGLWidget. The waterfall is a textured
 * quad: raw dB values live in a ring texture (one row uploaded per
 * spectrum frame) and the colormap is applied in the fragment shader via
 * a 256x1 LUT texture — so dB-range changes and colormap switches
 * re-color the ENTIRE history instantly, scrolling is a uniform update
 * (no CPU memmove / per-pixel colormap), and a 60 fps animation timer
 * adds sub-row scroll glide plus trace easing. Everything else (trace,
 * axes, band overlay, mask, cursors, hover) is QPainter drawn over the
 * GL surface, which Qt composites on the same FBO. If shader/texture
 * init fails (broken driver, headless RDP), gl_ok_ stays false and the
 * waterfall falls back to the legacy CPU QImage path — same widget,
 * same API.
 *
 * INTERACTION. Drag the band overlay to retune (Fc center line, band
 * edges); hover for a frequency/level readout; left-click cursors
 * (Shift for the delta cursor); right-drag box-zooms the frequency
 * axis; Ctrl+wheel zooms frequency about the cursor (plain wheel zooms
 * the dB range); middle- or Ctrl-drag pans; double-click resets to the
 * full span and re-enables auto range.
 */
#pragma once

#include "../include/app_state.hpp"
#include "style.hpp"

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QImage>
#include <QPainterPath>
#include <QTimer>
#include <QElapsedTimer>

#include <array>
#include <vector>

class QOpenGLShaderProgram;

namespace gw {

class SpectrumWidget : public QOpenGLWidget, protected QOpenGLFunctions {
    Q_OBJECT

public:
    explicit SpectrumWidget(AppState& state, QWidget* parent = nullptr);
    ~SpectrumWidget() override;

    // Called by DataBridge signal
    void onSpectrumReady();

    // Display range
    void setDbRange(float min_db, float max_db);
    void setAutoRange(bool on) { auto_range_ = on; update(); }
    bool autoRange() const { return auto_range_; }
    float minDb() const { return min_db_; }
    float maxDb() const { return max_db_; }

    // Trace processing + waterfall colormap (View menu, persisted)
    enum class Colormap { Classic = 0, Turbo = 1, Viridis = 2 };
    void setColormap(int id);
    int  colormap() const { return static_cast<int>(colormap_); }
    void setPeakHold(bool on);
    bool peakHold() const { return peak_hold_; }
    void setAveraging(bool on);
    bool averaging() const { return averaging_; }

    QSize sizeHint() const override { return {600, 320}; }
    QSize minimumSizeHint() const override { return {240, 160}; }

    void setShowMask(bool on) { show_mask_ = on; update(); }
    bool showMask() const { return show_mask_; }
    void clearCursors() { c1_x_ = c2_x_ = -1; update(); }

signals:
    void dbRangeChanged(float min_db, float max_db);
    /** Auto-range was re-enabled from the widget (double-click). */
    void autoRangeChanged(bool on);
    /** Fc / BW were edited by dragging the band overlay. Fired on mouse
     *  RELEASE (the visual markers track live, but the engine rebuild is
     *  deferred to the drop so a long drag doesn't queue dozens of DSP
     *  re-inits). */
    void tuneChanged();

protected:
    void initializeGL() override;
    void paintGL() override;
    void wheelEvent(QWheelEvent*) override;
    void mousePressEvent(QMouseEvent*) override;
    void mouseMoveEvent(QMouseEvent*) override;
    void mouseReleaseEvent(QMouseEvent*) override;
    void mouseDoubleClickEvent(QMouseEvent*) override;
    void leaveEvent(QEvent*) override;

private:
    void paintSpectrum(QPainter& p, const QRect& r);
    void paintWaterfallCPU(QPainter& p, const QRect& r);
    void drawWaterfallGL(const QRect& r);
    void paintAxes(QPainter& p, const QRect& spec_r);
    void paintMask(QPainter& p, const QRect& spec_r);
    void paintCursors(QPainter& p, const QRect& spec_r);
    void paintHover(QPainter& p, const QRect& spec_r);
    void paintZoomBand(QPainter& p, const QRect& spec_r);
    void updateWaterfallCPU();
    void animTick();
    void ensureAnimRunning();
    void uploadPendingRows();
    void rebuildLut();
    /** Colormap color for normalized t in [0,1] under the SELECTED map —
     *  single source of truth for the GL LUT and the CPU fallback. */
    QColor colormapColor(float t) const;
    QColor dbToWaterfallColor(float db) const;
    float  dbToY(float db, const QRect& r) const;
    /** Pixel x ↔ frequency, honoring the current freq zoom window. */
    float  xToHz(int x, const QRect& spec_r) const;
    float  hzToX(float hz, const QRect& spec_r) const;
    /** Clamp + apply a new frequency window (normalized 0..1 of Nyquist). */
    void   setFreqWindow(float lo, float hi);
    /** The spectrum plot rect (same geometry paintGL uses) — single
     *  source of truth for paint AND mouse hit-testing. */
    QRect  plotRect() const;
    QRect  waterfallRect() const;
    /** Which draggable band element (if any) is within grab range of x. */
    enum class DragTarget { None, Fc, EdgeLo, EdgeHi };
    DragTarget hitTestBand(int x, const QRect& spec_r) const;

    AppState&  state_;

    // ---- CPU fallback waterfall ----
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

    // Trace processing
    bool   peak_hold_ = false;
    bool   averaging_ = false;
    std::array<float, SPECTRUM_BINS> peak_db_{};
    std::array<float, SPECTRUM_BINS> avg_db_{};
    std::array<float, SPECTRUM_BINS> disp_db_{};   // eased displayed trace
    bool   avg_primed_ = false;
    bool   disp_primed_ = false;
    Colormap colormap_ = Colormap::Classic;

    // Direct manipulation state
    DragTarget drag_ = DragTarget::None;
    int    hover_x_  = -1;
    int    hover_y_  = -1;

    // Frequency zoom window, normalized to [0,1] of Nyquist.
    float  win_lo_ = 0.f;
    float  win_hi_ = 1.f;
    // Right-drag box zoom (x pixels; -1 = inactive) and Ctrl/middle pan.
    int    zoom_x0_ = -1;
    int    zoom_x1_ = -1;
    bool   panning_ = false;
    int    pan_last_x_ = 0;

    // ---- GL waterfall state ----
    bool   gl_ok_ = false;
    QOpenGLShaderProgram* wf_prog_ = nullptr;
    unsigned int db_tex_  = 0;
    unsigned int lut_tex_ = 0;
    bool   lut_dirty_ = true;
    // Deeper history than the CPU fallback: the texture ring is cheap.
    static constexpr int WF_TEX_ROWS = 512;
    int    wf_head_ = 0;                 // last-written ring row
    std::vector<uint8_t> pending_rows_;  // staged RGBA rows for upload
    // dB is packed into 8 bits over a fixed absolute span so range
    // changes are pure shader math (no re-upload).
    static constexpr float PACK_DB_MIN = -160.f;
    static constexpr float PACK_DB_SPAN = 200.f;

    // ---- Animation (60 fps while data flows, idle otherwise) ----
    QTimer        anim_timer_;
    QElapsedTimer clock_;
    qint64        last_tick_ms_ = 0;
    qint64        last_data_ms_ = -10000;
    float         scroll_frac_ = 0.f;    // sub-row waterfall glide

    // Split ratio: top fraction is spectrum, rest is waterfall
    static constexpr float SPEC_FRAC = 0.62f;
};

} // namespace gw
