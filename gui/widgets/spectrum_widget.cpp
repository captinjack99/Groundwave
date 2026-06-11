/**
 * @file spectrum_widget.cpp
 */
#include "spectrum_widget.hpp"

#include <QOpenGLShaderProgram>
#include <QPainter>
#include <QPainterPath>
#include <QLinearGradient>
#include <QWheelEvent>
#include <QMouseEvent>
#include <QFontMetrics>
#include <cmath>
#include <cstring>
#include <algorithm>
#include <iterator>

namespace gw {

namespace {

// Waterfall quad: position in NDC, uv in [0,1]^2 (u: freq, v: 0=newest row).
const char* kWfVertSrc =
    "attribute vec2 a_pos;\n"
    "attribute vec2 a_uv;\n"
    "varying vec2 v_uv;\n"
    "void main() {\n"
    "    v_uv = a_uv;\n"
    "    gl_Position = vec4(a_pos, 0.0, 1.0);\n"
    "}\n";

// dB values are packed into the R channel over a fixed absolute span; the
// display window (u_min/u_max, in packed units) and the colormap LUT are
// applied here — so range and palette changes re-color ALL history without
// touching the texture. u_head scrolls the ring; u_vspan selects how many
// ring rows are visible (1 row per device pixel).
const char* kWfFragSrc =
    "varying vec2 v_uv;\n"
    "uniform sampler2D u_db;\n"
    "uniform sampler2D u_lut;\n"
    "uniform float u_head;\n"
    "uniform float u_vspan;\n"
    "uniform float u_min;\n"
    "uniform float u_max;\n"
    "uniform float u_lo;\n"
    "uniform float u_hi;\n"
    "void main() {\n"
    "    float u = mix(u_lo, u_hi, v_uv.x);\n"
    "    float v = fract(u_head - v_uv.y * u_vspan);\n"
    "    float db = texture2D(u_db, vec2(u, v)).r;\n"
    "    float t = clamp((db - u_min) / max(u_max - u_min, 1e-6), 0.0, 1.0);\n"
    "    gl_FragColor = texture2D(u_lut, vec2(t, 0.5));\n"
    "}\n";

inline float packDb(float db, float pack_min, float pack_span) {
    return std::clamp((db - pack_min) / pack_span, 0.f, 1.f);
}

} // anonymous

SpectrumWidget::SpectrumWidget(AppState& state, QWidget* parent)
    : QOpenGLWidget(parent), state_(state)
{
    setMinimumSize(240, 160);
    // Hover readout + drag affordances need move events without buttons.
    setMouseTracking(true);
    setToolTip(
        "Drag the band lines to retune (center = move, edges = bandwidth). "
        "Left-click: cursor (Shift: delta cursor). Right-drag: zoom the "
        "frequency axis. Ctrl+wheel: zoom about the pointer. Wheel: dB "
        "range. Middle/Ctrl-drag: pan. Double-click: full span + auto "
        "range. Right-click the scope for its own options.");
    peak_db_.fill(-160.f);
    avg_db_.fill(-160.f);
    disp_db_.fill(-80.f);

    // CPU-fallback waterfall image (used only when GL init fails).
    waterfall_img_ = QImage(
        static_cast<int>(SPECTRUM_BINS),
        static_cast<int>(WATERFALL_ROWS),
        QImage::Format_RGB32);
    waterfall_img_.fill(QColor(style::WFALL_COLD).rgb());

    clock_.start();
    anim_timer_.setInterval(16);   // ~60 fps while data flows
    connect(&anim_timer_, &QTimer::timeout, this, [this]() { animTick(); });
}

SpectrumWidget::~SpectrumWidget() {
    // The widget may be destroyed without ever having been shown (no GL
    // context yet) — only touch GL with a live context.
    if (context()) {
        makeCurrent();
        if (db_tex_)  glDeleteTextures(1, &db_tex_);
        if (lut_tex_) glDeleteTextures(1, &lut_tex_);
        delete wf_prog_;
        wf_prog_ = nullptr;
        doneCurrent();
    } else {
        delete wf_prog_;
        wf_prog_ = nullptr;
    }
}

// =========================================================================
// GL setup
// =========================================================================

void SpectrumWidget::initializeGL() {
    initializeOpenGLFunctions();

    wf_prog_ = new QOpenGLShaderProgram;
    bool ok = wf_prog_->addShaderFromSourceCode(QOpenGLShader::Vertex,
                                                kWfVertSrc)
           && wf_prog_->addShaderFromSourceCode(QOpenGLShader::Fragment,
                                                kWfFragSrc)
           && wf_prog_->link();

    if (ok) {
        glGenTextures(1, &db_tex_);
        glBindTexture(GL_TEXTURE_2D, db_tex_);
        // RGBA8 with dB in R: universally supported (incl. GLES2 paths),
        // unlike single-channel float formats.
        std::vector<uint8_t> zero(SPECTRUM_BINS * WF_TEX_ROWS * 4, 0);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA,
                     static_cast<GLsizei>(SPECTRUM_BINS), WF_TEX_ROWS, 0,
                     GL_RGBA, GL_UNSIGNED_BYTE, zero.data());
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        // T wraps: the ring address fract() lands anywhere in [0,1).
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

        glGenTextures(1, &lut_tex_);
        glBindTexture(GL_TEXTURE_2D, lut_tex_);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        lut_dirty_ = true;

        glBindTexture(GL_TEXTURE_2D, 0);
        gl_ok_ = (glGetError() == GL_NO_ERROR);
    }
    if (!ok || !gl_ok_) {
        // Fall back to the CPU QImage waterfall; everything else is
        // QPainter anyway. (Broken GL drivers, remote desktops.)
        gl_ok_ = false;
        delete wf_prog_;
        wf_prog_ = nullptr;
    }
}

void SpectrumWidget::rebuildLut() {
    if (!gl_ok_) return;
    std::array<uint8_t, 256 * 4> lut{};
    for (int i = 0; i < 256; ++i) {
        const QColor c = colormapColor(static_cast<float>(i) / 255.f);
        lut[static_cast<size_t>(i) * 4 + 0] = static_cast<uint8_t>(c.red());
        lut[static_cast<size_t>(i) * 4 + 1] = static_cast<uint8_t>(c.green());
        lut[static_cast<size_t>(i) * 4 + 2] = static_cast<uint8_t>(c.blue());
        lut[static_cast<size_t>(i) * 4 + 3] = 255;
    }
    glBindTexture(GL_TEXTURE_2D, lut_tex_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 256, 1, 0,
                 GL_RGBA, GL_UNSIGNED_BYTE, lut.data());
    glBindTexture(GL_TEXTURE_2D, 0);
    lut_dirty_ = false;
}

void SpectrumWidget::uploadPendingRows() {
    if (!gl_ok_ || pending_rows_.empty()) return;
    const size_t row_bytes = SPECTRUM_BINS * 4;
    const size_t n_rows = pending_rows_.size() / row_bytes;
    glBindTexture(GL_TEXTURE_2D, db_tex_);
    for (size_t k = 0; k < n_rows; ++k) {
        wf_head_ = (wf_head_ + 1) % WF_TEX_ROWS;
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, wf_head_,
                        static_cast<GLsizei>(SPECTRUM_BINS), 1,
                        GL_RGBA, GL_UNSIGNED_BYTE,
                        pending_rows_.data() + k * row_bytes);
    }
    glBindTexture(GL_TEXTURE_2D, 0);
    pending_rows_.clear();
}

// =========================================================================
// Data + animation
// =========================================================================

void SpectrumWidget::onSpectrumReady() {
    const SpectrumData& sd = state_.spectrum;
    if (auto_range_) {
        float new_max = sd.peak_db + 6.f;
        float new_min = sd.noise_floor - 12.f;
        if (new_max - new_min < 30.f) new_min = new_max - 60.f;
        // Smooth range change
        max_db_ = max_db_ * 0.92f + new_max * 0.08f;
        min_db_ = min_db_ * 0.92f + new_min * 0.08f;
    }
    // EMA average runs on every frame regardless of the display toggle so
    // switching it on shows history immediately.
    constexpr float kAvgAlpha = 0.35f;
    if (!avg_primed_) {
        for (size_t i = 0; i < SPECTRUM_BINS; ++i) avg_db_[i] = sd.power_db[i];
        avg_primed_ = true;
    } else {
        for (size_t i = 0; i < SPECTRUM_BINS; ++i) {
            avg_db_[i] += kAvgAlpha * (sd.power_db[i] - avg_db_[i]);
        }
    }
    for (size_t i = 0; i < SPECTRUM_BINS; ++i) {
        peak_db_[i] = std::max(peak_db_[i], sd.power_db[i]);
    }

    if (gl_ok_) {
        // Stage one packed row; uploaded in paintGL with the context live.
        const size_t row_bytes = SPECTRUM_BINS * 4;
        if (pending_rows_.size() / row_bytes >=
            static_cast<size_t>(WF_TEX_ROWS)) {
            pending_rows_.erase(pending_rows_.begin(),
                                pending_rows_.begin() +
                                    static_cast<ptrdiff_t>(row_bytes));
        }
        const size_t base = pending_rows_.size();
        pending_rows_.resize(base + row_bytes);
        uint8_t* row = pending_rows_.data() + base;
        for (size_t i = 0; i < SPECTRUM_BINS; ++i) {
            const uint8_t v = static_cast<uint8_t>(
                packDb(sd.power_db[i], PACK_DB_MIN, PACK_DB_SPAN) * 255.f);
            row[i * 4 + 0] = v;
            row[i * 4 + 1] = 0;
            row[i * 4 + 2] = 0;
            row[i * 4 + 3] = 255;
        }
        scroll_frac_ -= 1.f;                       // glide in the new row
        scroll_frac_ = std::max(scroll_frac_, -3.f);
    } else {
        updateWaterfallCPU();
    }

    last_data_ms_ = clock_.elapsed();
    ensureAnimRunning();
}

void SpectrumWidget::ensureAnimRunning() {
    if (!anim_timer_.isActive()) {
        last_tick_ms_ = clock_.elapsed();
        anim_timer_.start();
    }
}

void SpectrumWidget::animTick() {
    const qint64 now = clock_.elapsed();
    float dt = static_cast<float>(now - last_tick_ms_) / 1000.f;
    last_tick_ms_ = now;
    dt = std::clamp(dt, 0.f, 0.1f);

    // Ease the displayed trace toward the (raw or averaged) target — the
    // motion that makes 30 Hz data read as a live instrument at 60 fps.
    const SpectrumData& sd = state_.spectrum;
    const float* target = averaging_ ? avg_db_.data() : sd.power_db.data();
    if (!disp_primed_) {
        for (size_t i = 0; i < SPECTRUM_BINS; ++i) disp_db_[i] = target[i];
        disp_primed_ = true;
    } else {
        const float k = 1.f - std::exp(-dt / 0.05f);   // tau = 50 ms
        for (size_t i = 0; i < SPECTRUM_BINS; ++i) {
            disp_db_[i] += k * (target[i] - disp_db_[i]);
        }
    }

    // Peak-hold decay, time-based.
    if (peak_hold_) {
        const float decay = 7.5f * dt;                 // dB per second
        for (size_t i = 0; i < SPECTRUM_BINS; ++i) {
            peak_db_[i] = std::max(peak_db_[i] - decay, -160.f);
        }
    }

    // Waterfall sub-row glide back toward alignment (~one row per 33 ms,
    // matching the data cadence).
    if (scroll_frac_ < 0.f) {
        scroll_frac_ = std::min(0.f, scroll_frac_ + dt / 0.033f);
    }

    // Idle: no data for a while → stop burning frames; setters still
    // repaint via update().
    if (now - last_data_ms_ > 1500) {
        anim_timer_.stop();
    }
    update();
}

// =========================================================================
// Display options
// =========================================================================

void SpectrumWidget::setDbRange(float min_db, float max_db) {
    // Enforce a minimum non-zero span. dbToY / the waterfall colormap
    // divide by (max_db_ - min_db_); a degenerate min==max range would
    // produce NaN/inf through std::clamp and paint garbage. (#50)
    if (max_db <= min_db) max_db = min_db + 1.f;
    min_db_ = min_db;
    max_db_ = max_db;
    update();
}

void SpectrumWidget::setColormap(int id) {
    colormap_ = static_cast<Colormap>(std::clamp(id, 0, 2));
    if (gl_ok_) {
        // The LUT recolors the ENTIRE history retroactively — no reset.
        lut_dirty_ = true;
    } else {
        // CPU rows were baked under the old palette; restart the scroll
        // so the display never mixes two palettes.
        waterfall_img_.fill(QColor(style::WFALL_COLD).rgb());
    }
    update();
}

void SpectrumWidget::setPeakHold(bool on) {
    peak_hold_ = on;
    if (on) peak_db_.fill(-160.f);   // start fresh, not with stale peaks
    update();
}

void SpectrumWidget::setAveraging(bool on) {
    averaging_ = on;
    avg_primed_ = false;             // re-prime from the next frame
    update();
}

void SpectrumWidget::setFreqWindow(float lo, float hi) {
    constexpr float MIN_SPAN = 0.01f;   // 1 % of Nyquist
    lo = std::clamp(lo, 0.f, 1.f - MIN_SPAN);
    hi = std::clamp(hi, lo + MIN_SPAN, 1.f);
    win_lo_ = lo;
    win_hi_ = hi;
    update();
}

// =========================================================================
// Geometry + coordinate mapping
// =========================================================================

QRect SpectrumWidget::plotRect() const {
    const QRect full  = rect();
    const int   spec_h = static_cast<int>(full.height() * SPEC_FRAC);
    const int margin_left   = 42;
    const int margin_bottom = 18;
    const int margin_right  =  8;
    return QRect(full.left() + margin_left,
                 full.top() + 8,
                 full.width() - margin_left - margin_right,
                 spec_h - 8 - margin_bottom);
}

QRect SpectrumWidget::waterfallRect() const {
    const QRect full  = rect();
    const int   spec_h = static_cast<int>(full.height() * SPEC_FRAC);
    const int margin_left  = 42;
    const int margin_right =  8;
    return QRect(full.left() + margin_left, spec_h,
                 full.width() - margin_left - margin_right,
                 full.height() - spec_h);
}

float SpectrumWidget::xToHz(int x, const QRect& spec_r) const {
    if (spec_r.width() <= 0) return 0.f;
    float t = static_cast<float>(x - spec_r.left()) / spec_r.width();
    t = std::clamp(t, 0.f, 1.f);
    float nyq = 0.5f * static_cast<float>(state_.spectrum.sample_rate);
    return (win_lo_ + t * (win_hi_ - win_lo_)) * nyq;
}

float SpectrumWidget::hzToX(float hz, const QRect& spec_r) const {
    float nyq = 0.5f * static_cast<float>(state_.spectrum.sample_rate);
    if (nyq <= 0.f) return static_cast<float>(spec_r.left());
    float norm = hz / nyq;
    float t = (norm - win_lo_) / std::max(win_hi_ - win_lo_, 1e-6f);
    t = std::clamp(t, -0.5f, 1.5f);   // allow slight off-plot for clipping
    return spec_r.left() + t * static_cast<float>(spec_r.width());
}

float SpectrumWidget::dbToY(float db, const QRect& r) const {
    float norm = (db - min_db_) / (max_db_ - min_db_);
    norm = std::clamp(norm, 0.f, 1.f);
    return r.bottom() - norm * r.height();
}

// =========================================================================
// Paint
// =========================================================================

void SpectrumWidget::paintGL() {
    if (gl_ok_) {
        if (lut_dirty_) rebuildLut();
        uploadPendingRows();
    }

    // Clear the whole FBO to the instrument base color, then draw the
    // waterfall quad with raw GL; everything else is QPainter on top.
    glClearColor(style::BG_BASE.redF(), style::BG_BASE.greenF(),
                 style::BG_BASE.blueF(), 1.f);
    glClear(GL_COLOR_BUFFER_BIT);

    const QRect wfall_r = waterfallRect();
    if (gl_ok_ && wfall_r.height() > 2) drawWaterfallGL(wfall_r);

    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    p.setRenderHint(QPainter::SmoothPixmapTransform);

    const QRect plot_r = plotRect();

    paintSpectrum(p, plot_r);
    paintAxes(p, plot_r);
    if (show_mask_) paintMask(p, plot_r);
    paintCursors(p, plot_r);
    paintHover(p, plot_r);
    paintZoomBand(p, plot_r);

    if (!gl_ok_) {
        paintWaterfallCPU(p, wfall_r);
    } else {
        // Subtle top separator over the GL quad.
        p.setPen(QPen(style::BORDER_DIM, 0.5));
        p.drawLine(wfall_r.topLeft(), wfall_r.topRight());
    }
}

void SpectrumWidget::drawWaterfallGL(const QRect& r) {
    // Widget-space rect → NDC. Both axes scale identically by the device
    // pixel ratio, so logical fractions are exact.
    const float W = static_cast<float>(width());
    const float H = static_cast<float>(height());
    if (W <= 0.f || H <= 0.f) return;
    const float x0 = 2.f * static_cast<float>(r.left())      / W - 1.f;
    const float x1 = 2.f * static_cast<float>(r.right() + 1) / W - 1.f;
    const float y0 = 1.f - 2.f * static_cast<float>(r.top())        / H;
    const float y1 = 1.f - 2.f * static_cast<float>(r.bottom() + 1) / H;

    const GLfloat pos[] = { x0, y0,  x1, y0,  x0, y1,  x1, y1 };
    const GLfloat uv[]  = { 0.f, 0.f,  1.f, 0.f,  0.f, 1.f,  1.f, 1.f };

    // One ring row per device pixel of waterfall height.
    const float dpr = static_cast<float>(devicePixelRatioF());
    const float vis_rows = std::clamp(
        static_cast<float>(r.height()) * dpr, 32.f,
        static_cast<float>(WF_TEX_ROWS));

    wf_prog_->bind();
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, db_tex_);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, lut_tex_);
    wf_prog_->setUniformValue("u_db", 0);
    wf_prog_->setUniformValue("u_lut", 1);
    wf_prog_->setUniformValue(
        "u_head",
        (static_cast<float>(wf_head_) + 0.5f + scroll_frac_) /
            static_cast<float>(WF_TEX_ROWS));
    wf_prog_->setUniformValue(
        "u_vspan", vis_rows / static_cast<float>(WF_TEX_ROWS));
    wf_prog_->setUniformValue(
        "u_min", packDb(min_db_, PACK_DB_MIN, PACK_DB_SPAN));
    wf_prog_->setUniformValue(
        "u_max", packDb(max_db_, PACK_DB_MIN, PACK_DB_SPAN));
    wf_prog_->setUniformValue("u_lo", win_lo_);
    wf_prog_->setUniformValue("u_hi", win_hi_);

    const int loc_pos = wf_prog_->attributeLocation("a_pos");
    const int loc_uv  = wf_prog_->attributeLocation("a_uv");
    wf_prog_->enableAttributeArray(loc_pos);
    wf_prog_->enableAttributeArray(loc_uv);
    wf_prog_->setAttributeArray(loc_pos, pos, 2);
    wf_prog_->setAttributeArray(loc_uv,  uv,  2);

    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

    wf_prog_->disableAttributeArray(loc_pos);
    wf_prog_->disableAttributeArray(loc_uv);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, 0);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, 0);
    wf_prog_->release();
}

void SpectrumWidget::paintSpectrum(QPainter& p, const QRect& r) {
    const SpectrumData& sd = state_.spectrum;
    const int W = r.width();
    const int N = static_cast<int>(SPECTRUM_BINS);

    if (W < 2 || N < 2) return;

    // --- Grid lines ---
    p.setPen(QPen(style::SPEC_GRID, 0.5));
    float db_step = 10.f;
    float grid_start = std::ceil(min_db_ / db_step) * db_step;
    QFont label_font;
    label_font.setFamily("SF Mono, Menlo, DejaVu Sans Mono, monospace");
    label_font.setPixelSize(10);
    p.setFont(label_font);

    for (float db = grid_start; db <= max_db_ + 0.1f; db += db_step) {
        float y = dbToY(db, r);
        if (y < r.top() || y > r.bottom()) continue;
        p.setPen(QPen(style::SPEC_GRID, 0.5));
        p.drawLine(QPointF(r.left(), y), QPointF(r.right(), y));
    }

    // Vertical: every ~10% of the visible span
    p.setPen(QPen(style::SPEC_GRID, 0.5));
    for (int tick = 1; tick < 10; ++tick) {
        float x = r.left() + tick * 0.1f * r.width();
        p.drawLine(QPointF(x, r.top()), QPointF(x, r.bottom()));
    }

    // --- Alarm SNR threshold line ---
    float snr_thresh_db = state_.alarm_thresh.snr_low_db + sd.noise_floor;
    {
        float ty = dbToY(snr_thresh_db, r);
        if (ty >= r.top() && ty <= r.bottom()) {
            QPen thresh_pen(style::C_WARNING, 0.75, Qt::DashLine);
            thresh_pen.setDashPattern({4, 4});
            p.setPen(thresh_pen);
            p.drawLine(QPointF(r.left(), ty), QPointF(r.right(), ty));
        }
    }

    // --- Band overlay: Fc center + Fc ± BW/2 edges ---
    // Same mapping hitTestBand uses, so the painted lines and the mouse
    // grab zones can never disagree.
    {
        float fc, bw;
        {
            std::lock_guard<std::mutex> lock(state_.mtx);
            fc = state_.modem.center_freq;
            bw = state_.modem.signal_bw;
        }
        if (state_.spectrum.sample_rate > 0.f) {
            float x_lo  = hzToX(fc - bw * 0.5f, r);
            float x_hi  = hzToX(fc + bw * 0.5f, r);
            float x_ctr = hzToX(fc, r);

            QPen edge_pen(style::C_SIGNAL, 0.9, Qt::DashLine);
            edge_pen.setDashPattern({3, 4});
            p.setPen(edge_pen);
            p.drawLine(QPointF(x_lo, r.top()),
                       QPointF(x_lo, r.bottom()));
            p.drawLine(QPointF(x_hi, r.top()),
                       QPointF(x_hi, r.bottom()));

            QPen ctr_pen(style::C_SIGNAL, 1.2, Qt::DashLine);
            ctr_pen.setDashPattern({6, 4});
            p.setPen(ctr_pen);
            p.drawLine(QPointF(x_ctr, r.top()),
                       QPointF(x_ctr, r.bottom()));

            // Band-edge labels
            p.setPen(QColor(120, 200, 255, 200));
            QFont small = p.font();
            small.setPixelSize(9);
            p.setFont(small);
            p.drawText(QRectF(x_lo - 30, r.top() + 2, 60, 12),
                       Qt::AlignCenter,
                       QString("%1k").arg((fc - bw * 0.5f) / 1000.f, 0, 'f', 1));
            p.drawText(QRectF(x_hi - 30, r.top() + 2, 60, 12),
                       Qt::AlignCenter,
                       QString("%1k").arg((fc + bw * 0.5f) / 1000.f, 0, 'f', 1));
            p.setFont(label_font);
        }
    }

    // Visible bin range under the current freq window (with one bin of
    // overhang so the trace runs off-plot instead of stopping short).
    const float fN = static_cast<float>(N - 1);
    int i0 = static_cast<int>(std::floor(win_lo_ * fN)) - 1;
    int i1 = static_cast<int>(std::ceil(win_hi_ * fN)) + 1;
    i0 = std::clamp(i0, 0, N - 1);
    i1 = std::clamp(i1, 0, N - 1);
    auto binX = [&](int i) {
        const float norm = static_cast<float>(i) / fN;
        const float t = (norm - win_lo_) / std::max(win_hi_ - win_lo_, 1e-6f);
        return r.left() + t * static_cast<float>(r.width());
    };

    // --- Peak-hold trace (decaying max), drawn UNDER the live trace ---
    if (peak_hold_) {
        QPainterPath peak_path;
        bool pfirst = true;
        for (int i = i0; i <= i1; ++i) {
            float x = binX(i);
            float y = dbToY(peak_db_[static_cast<size_t>(i)], r);
            if (pfirst) { peak_path.moveTo(x, y); pfirst = false; }
            else         peak_path.lineTo(x, y);
        }
        QColor pk = style::C_PILOT; pk.setAlpha(150);
        p.setPen(QPen(pk, 0.9));
        p.setClipRect(r);
        p.drawPath(peak_path);
        p.setClipping(false);
    }

    // --- Live trace: the eased display buffer (reused member paths) ---
    trace_path_.clear();
    bool first = true;
    for (int i = i0; i <= i1; ++i) {
        float x = binX(i);
        float y = dbToY(disp_db_[static_cast<size_t>(i)], r);
        if (first) { trace_path_.moveTo(x, y); first = false; }
        else        trace_path_.lineTo(x, y);
    }

    {
        fill_path_.clear();
        fill_path_.addPath(trace_path_);
        fill_path_.lineTo(binX(i1), r.bottom());
        fill_path_.lineTo(binX(i0), r.bottom());
        fill_path_.closeSubpath();

        QLinearGradient grad(0, r.top(), 0, r.bottom());
        grad.setColorAt(0.0, style::SPEC_FILL_TOP);
        grad.setColorAt(1.0, style::SPEC_FILL_BOT);
        p.setClipRect(r);
        p.fillPath(fill_path_, grad);
    }

    p.setPen(QPen(style::SPEC_TRACE, 1.2));
    p.drawPath(trace_path_);
    p.setClipping(false);

    // --- Border ---
    p.setPen(QPen(style::BORDER_DIM, 0.5));
    p.drawRect(r);
}

void SpectrumWidget::paintAxes(QPainter& p, const QRect& r) {
    const SpectrumData& sd = state_.spectrum;

    QFont font;
    font.setFamily("SF Mono, Menlo, DejaVu Sans Mono, monospace");
    font.setPixelSize(10);
    p.setFont(font);
    p.setPen(style::TEXT_TERTIARY);

    // dB axis (left)
    float db_step = 10.f;
    float grid_start = std::ceil(min_db_ / db_step) * db_step;
    for (float db = grid_start; db <= max_db_ + 0.1f; db += db_step) {
        float y = dbToY(db, r);
        if (y < r.top() - 1 || y > r.bottom() + 1) continue;
        QString label = QString("%1").arg(static_cast<int>(db));
        p.drawText(QRectF(0, y - 7, r.left() - 4, 14),
                   Qt::AlignRight | Qt::AlignVCenter, label);
    }

    // Frequency axis (bottom): 1-2-5 ticks over the VISIBLE span, so the
    // labels stay sensible at any sample rate and any zoom.
    float sr = sd.sample_rate > 0.f ? sd.sample_rate : 48000.f;
    float nyquist = sr / 2.f;
    const float f_lo = win_lo_ * nyquist;
    const float f_hi = win_hi_ * nyquist;
    const float span = std::max(f_hi - f_lo, 1.f);
    float raw_step = span / 8.f;
    float mag  = std::pow(10.f, std::floor(std::log10(raw_step)));
    float frac = raw_step / mag;
    float nice = (frac < 1.5f) ? 1.f : (frac < 3.5f) ? 2.f : (frac < 7.5f) ? 5.f : 10.f;
    float step = nice * mag;
    for (float freq = std::ceil(f_lo / step) * step;
         freq <= f_hi + step * 0.01f; freq += step) {
        float x = hzToX(freq, r);
        if (x < r.left() - 1 || x > r.right() + 1) continue;
        QString label;
        if (freq == 0.f) {
            label = "0";
        } else if (step >= 1000.f) {
            int decimals = (std::fmod(step, 1000.f) == 0.f) ? 0 : 1;
            label = QString("%1k").arg(freq / 1000.f, 0, 'f', decimals);
        } else {
            label = QString("%1").arg(freq, 0, 'f', 0);
        }
        p.drawText(QRectF(x - 28, r.bottom() + 3, 56, 14),
                   Qt::AlignHCenter | Qt::AlignTop, label);
    }

    // Zoom hint when windowed in.
    if (win_lo_ > 0.f || win_hi_ < 1.f) {
        p.setPen(style::TEXT_SECONDARY);
        p.drawText(QRectF(r.left(), r.top() - 1, r.width() - 4, 12),
                   Qt::AlignRight | Qt::AlignTop,
                   QString("zoom %1–%2 kHz (double-click: full span)")
                       .arg(f_lo / 1000.f, 0, 'f', 1)
                       .arg(f_hi / 1000.f, 0, 'f', 1));
    }
}

// =========================================================================
// Waterfall — colormaps + CPU fallback
// =========================================================================

QColor SpectrumWidget::colormapColor(float t) const {
    t = std::clamp(t, 0.f, 1.f);

    struct Stop { float t; uint8_t r, g, b; };
    // Classic house map: deep navy → teal → cyan → yellow → white
    static constexpr Stop classic[] = {
        {0.00f,   5,   8,  28},
        {0.20f,   6,  40,  80},
        {0.40f,   0, 100, 160},
        {0.58f,   0, 190, 220},
        {0.75f,  80, 220, 120},
        {0.88f, 240, 200,  30},
        {1.00f, 255, 255, 200},
    };
    // Compact approximation of Google's Turbo (perceptually even, high
    // contrast — the SDR community favorite).
    static constexpr Stop turbo[] = {
        {0.00f,  48,  18,  59},
        {0.15f,  65,  69, 171},
        {0.30f,  53, 118, 233},
        {0.45f,  26, 175, 208},
        {0.60f,  64, 219, 120},
        {0.75f, 175, 240,  57},
        {0.875f, 249, 186,  56},
        {1.00f, 165,   0,  38},
    };
    // Compact viridis (colorblind-safe, matplotlib default).
    static constexpr Stop viridis[] = {
        {0.00f,  68,   1,  84},
        {0.20f,  62,  74, 137},
        {0.40f,  49, 104, 142},
        {0.60f,  33, 145, 140},
        {0.80f,  94, 201,  98},
        {1.00f, 253, 231,  37},
    };

    const Stop* stops; int N;
    switch (colormap_) {
        case Colormap::Turbo:   stops = turbo;   N = static_cast<int>(std::size(turbo));   break;
        case Colormap::Viridis: stops = viridis; N = static_cast<int>(std::size(viridis)); break;
        default:                stops = classic; N = static_cast<int>(std::size(classic)); break;
    }

    int lo = 0;
    for (int i = 1; i < N - 1; ++i) {
        if (t >= stops[i].t) lo = i; else break;
    }
    int hi = std::min(lo + 1, N - 1);
    float range = stops[hi].t - stops[lo].t;
    float s = (range > 0.f) ? (t - stops[lo].t) / range : 0.f;
    s = std::clamp(s, 0.f, 1.f);

    auto lerp = [](uint8_t a, uint8_t b, float s) -> uint8_t {
        return static_cast<uint8_t>(a + s * (b - a));
    };
    return QColor(
        lerp(stops[lo].r, stops[hi].r, s),
        lerp(stops[lo].g, stops[hi].g, s),
        lerp(stops[lo].b, stops[hi].b, s));
}

QColor SpectrumWidget::dbToWaterfallColor(float db) const {
    float t = (db - min_db_) / (max_db_ - min_db_);
    return colormapColor(t);
}

void SpectrumWidget::updateWaterfallCPU() {
    const SpectrumData& sd = state_.spectrum;

    if (waterfall_img_.width() != static_cast<int>(SPECTRUM_BINS) ||
        waterfall_img_.height() != static_cast<int>(WATERFALL_ROWS)) {
        waterfall_img_ = QImage(
            static_cast<int>(SPECTRUM_BINS),
            static_cast<int>(WATERFALL_ROWS),
            QImage::Format_RGB32);
        waterfall_img_.fill(QColor(style::WFALL_COLD).rgb());
    }

    if (WATERFALL_ROWS > 1) {
        memmove(waterfall_img_.bits() + waterfall_img_.bytesPerLine(),
                waterfall_img_.bits(),
                waterfall_img_.bytesPerLine() * (WATERFALL_ROWS - 1));
    }

    QRgb* top_row = reinterpret_cast<QRgb*>(waterfall_img_.bits());
    for (size_t i = 0; i < SPECTRUM_BINS; ++i) {
        top_row[i] = dbToWaterfallColor(sd.power_db[i]).rgb();
    }
}

void SpectrumWidget::paintWaterfallCPU(QPainter& p, const QRect& r) {
    p.fillRect(r, style::BG_BASE);
    // Honor the frequency zoom window by drawing the matching sub-rect.
    const float img_w = static_cast<float>(waterfall_img_.width());
    QRectF src(win_lo_ * img_w, 0.f,
               std::max(win_hi_ - win_lo_, 1e-6f) * img_w,
               static_cast<float>(waterfall_img_.height()));
    p.drawImage(QRectF(r), waterfall_img_, src);

    p.setPen(QPen(style::BORDER_DIM, 0.5));
    p.drawLine(r.topLeft(), r.topRight());
}

// =========================================================================
// Mouse interaction
// =========================================================================

SpectrumWidget::DragTarget SpectrumWidget::hitTestBand(
        int x, const QRect& spec_r) const {
    float fc, bw;
    {
        std::lock_guard<std::mutex> lock(state_.mtx);
        fc = state_.modem.center_freq;
        bw = state_.modem.signal_bw;
    }
    if (bw <= 0.f) return DragTarget::None;
    constexpr int GRAB_PX = 6;
    const float x_ctr = hzToX(fc, spec_r);
    const float x_lo  = hzToX(fc - bw * 0.5f, spec_r);
    const float x_hi  = hzToX(fc + bw * 0.5f, spec_r);
    // Edges win over center when they're close together (narrow band).
    if (std::abs(x - x_lo)  <= GRAB_PX) return DragTarget::EdgeLo;
    if (std::abs(x - x_hi)  <= GRAB_PX) return DragTarget::EdgeHi;
    if (std::abs(x - x_ctr) <= GRAB_PX) return DragTarget::Fc;
    return DragTarget::None;
}

void SpectrumWidget::mousePressEvent(QMouseEvent* e) {
    const QRect spec_r = plotRect();
    if (!spec_r.contains(e->pos())) {
        QOpenGLWidget::mousePressEvent(e);
        return;
    }

    // Right-drag: frequency box zoom.
    if (e->button() == Qt::RightButton) {
        zoom_x0_ = zoom_x1_ = e->pos().x();
        update();
        return;
    }
    // Middle (or Ctrl+left) drag: pan the frequency window.
    if (e->button() == Qt::MiddleButton ||
        (e->button() == Qt::LeftButton &&
         (e->modifiers() & Qt::ControlModifier))) {
        panning_ = true;
        pan_last_x_ = e->pos().x();
        setCursor(Qt::ClosedHandCursor);
        return;
    }

    // Band manipulation has priority over measurement cursors.
    if (e->button() == Qt::LeftButton &&
        !(e->modifiers() & Qt::ShiftModifier)) {
        drag_ = hitTestBand(e->pos().x(), spec_r);
        if (drag_ != DragTarget::None) {
            setCursor(Qt::SizeHorCursor);
            return;
        }
    }
    // Measurement cursors: left button only (a right-click used to place
    // one too, which collided with every context-style gesture).
    if (e->button() == Qt::LeftButton) {
        if (e->modifiers() & Qt::ShiftModifier) {
            c2_x_ = e->pos().x();
        } else {
            c1_x_ = e->pos().x();
            c2_x_ = -1;
        }
        update();
    }
}

void SpectrumWidget::mouseMoveEvent(QMouseEvent* e) {
    const QRect spec_r = plotRect();

    if (zoom_x0_ >= 0) {
        zoom_x1_ = e->pos().x();
        update();
        return;
    }
    if (panning_) {
        const int dx = e->pos().x() - pan_last_x_;
        pan_last_x_ = e->pos().x();
        if (spec_r.width() > 0) {
            const float span = win_hi_ - win_lo_;
            const float shift = -static_cast<float>(dx) /
                                static_cast<float>(spec_r.width()) * span;
            float lo = win_lo_ + shift;
            float hi = win_hi_ + shift;
            if (lo < 0.f) { hi -= lo; lo = 0.f; }
            if (hi > 1.f) { lo -= (hi - 1.f); hi = 1.f; }
            setFreqWindow(lo, hi);
        }
        return;
    }

    if (drag_ != DragTarget::None) {
        const float f = xToHz(e->pos().x(), spec_r);
        const float nyq = 0.5f * static_cast<float>(state_.spectrum.sample_rate);
        if (nyq > 0.f) {
            std::lock_guard<std::mutex> lock(state_.mtx);
            float fc = state_.modem.center_freq;
            float bw = state_.modem.signal_bw;
            constexpr float MARGIN = 200.f;   // keep edges off DC/Nyquist
            constexpr float MIN_BW = 1000.f;
            if (drag_ == DragTarget::Fc) {
                float half = bw * 0.5f;
                state_.modem.center_freq =
                    std::clamp(f, half + MARGIN, nyq - half - MARGIN);
            } else {
                // Either edge resizes symmetrically around the fixed Fc.
                float half = std::abs(f - fc);
                half = std::clamp(half, MIN_BW * 0.5f,
                                  std::min(fc, nyq - fc) - MARGIN);
                state_.modem.signal_bw = half * 2.f;
            }
        }
        update();
        return;
    }

    // Hover: cursor affordance + measurement readout.
    if (spec_r.contains(e->pos())) {
        hover_x_ = e->pos().x();
        hover_y_ = e->pos().y();
        setCursor(hitTestBand(hover_x_, spec_r) != DragTarget::None
                      ? Qt::SizeHorCursor : Qt::CrossCursor);
    } else {
        hover_x_ = hover_y_ = -1;
        unsetCursor();
    }
    update();
}

void SpectrumWidget::mouseReleaseEvent(QMouseEvent* e) {
    const QRect spec_r = plotRect();

    if (e->button() == Qt::RightButton && zoom_x0_ >= 0) {
        const int x0 = std::min(zoom_x0_, zoom_x1_);
        const int x1 = std::max(zoom_x0_, zoom_x1_);
        zoom_x0_ = zoom_x1_ = -1;
        if (x1 - x0 > 8) {
            const float nyq =
                0.5f * static_cast<float>(state_.spectrum.sample_rate);
            if (nyq > 0.f) {
                const float f0 = xToHz(x0, spec_r);
                const float f1 = xToHz(x1, spec_r);
                setFreqWindow(f0 / nyq, f1 / nyq);
            }
        }
        update();
        return;
    }
    if (panning_ &&
        (e->button() == Qt::MiddleButton || e->button() == Qt::LeftButton)) {
        panning_ = false;
        unsetCursor();
        return;
    }
    if (drag_ != DragTarget::None && e->button() == Qt::LeftButton) {
        drag_ = DragTarget::None;
        unsetCursor();
        emit tuneChanged();   // one engine rebuild, on the drop
        return;
    }
    QOpenGLWidget::mouseReleaseEvent(e);
}

void SpectrumWidget::mouseDoubleClickEvent(QMouseEvent* e) {
    const QRect spec_r = plotRect();
    if (spec_r.contains(e->pos()) || waterfallRect().contains(e->pos())) {
        setFreqWindow(0.f, 1.f);
        auto_range_ = true;
        emit autoRangeChanged(true);
        update();
        return;
    }
    QOpenGLWidget::mouseDoubleClickEvent(e);
}

void SpectrumWidget::leaveEvent(QEvent* e) {
    hover_x_ = hover_y_ = -1;
    unsetCursor();
    update();
    QOpenGLWidget::leaveEvent(e);
}

void SpectrumWidget::wheelEvent(QWheelEvent* e) {
    const float steps = e->angleDelta().y() / 120.f;

    if (e->modifiers() & Qt::ControlModifier) {
        // Frequency zoom about the pointer.
        const QRect spec_r = plotRect();
        const float span = win_hi_ - win_lo_;
        const float factor = std::pow(1.0f / 1.15f, steps);
        float new_span = std::clamp(span * factor, 0.01f, 1.f);
        // Keep the frequency under the cursor fixed.
        float t = 0.5f;
        if (spec_r.width() > 0) {
            t = std::clamp(
                static_cast<float>(e->position().x() - spec_r.left()) /
                    static_cast<float>(spec_r.width()),
                0.f, 1.f);
        }
        const float f_at = win_lo_ + t * span;
        float lo = f_at - t * new_span;
        float hi = lo + new_span;
        if (lo < 0.f) { hi -= lo; lo = 0.f; }
        if (hi > 1.f) { lo -= (hi - 1.f); hi = 1.f; }
        setFreqWindow(lo, hi);
        return;
    }

    // Plain wheel: zoom the dB range (and drop out of auto range).
    float range = max_db_ - min_db_;
    range = std::clamp(range - steps * 5.f, 10.f, 160.f);
    float center = (max_db_ + min_db_) * 0.5f;
    min_db_ = center - range * 0.5f;
    max_db_ = center + range * 0.5f;
    auto_range_ = false;
    emit dbRangeChanged(min_db_, max_db_);
    update();
}

// =========================================================================
// Overlays: cursors, hover readout, zoom band, FCC mask
// =========================================================================

void SpectrumWidget::paintCursors(QPainter& p, const QRect& spec_r) {
    if (c1_x_ < 0) return;
    auto drawCursor = [&](int x, QColor c, const QString& label) {
        if (x < spec_r.left() || x > spec_r.right()) return;
        QPen pen(c, 1.0, Qt::SolidLine);
        p.setPen(pen);
        p.drawLine(QPointF(x, spec_r.top()), QPointF(x, spec_r.bottom()));
        float hz = xToHz(x, spec_r);
        QString text = QString("%1  %2 kHz")
            .arg(label).arg(hz / 1000.f, 0, 'f', 2);
        QFont small;
        small.setPixelSize(10);
        p.setFont(small);
        QFontMetrics fm(small);
        int tw = fm.horizontalAdvance(text) + 8;
        int th = fm.height() + 4;
        QRectF box(x + 4, spec_r.top() + 4, tw, th);
        if (box.right() > spec_r.right()) box.moveLeft(x - tw - 4);
        p.fillRect(box, QColor(20, 22, 28, 200));
        p.setPen(c);
        p.drawRect(box);
        p.drawText(box, Qt::AlignCenter, text);
    };
    drawCursor(c1_x_, style::C_OK,      "M1");
    if (c2_x_ >= 0) {
        drawCursor(c2_x_, style::C_PILOT, "M2");
        float f1 = xToHz(c1_x_, spec_r);
        float f2 = xToHz(c2_x_, spec_r);
        QString d = QString("Δf = %1 kHz").arg((f2 - f1) / 1000.f, 0, 'f', 2);
        QFont mid;
        mid.setPixelSize(11);
        mid.setBold(true);
        p.setFont(mid);
        QFontMetrics fm(mid);
        int tw = fm.horizontalAdvance(d) + 12;
        QRectF box((spec_r.width() - tw) * 0.5, spec_r.top() + 4, tw,
                    fm.height() + 6);
        p.fillRect(box, QColor(20, 22, 28, 220));
        p.setPen(style::C_SIGNAL);
        p.drawRect(box);
        p.setPen(style::TEXT_PRIMARY);
        p.drawText(box, Qt::AlignCenter, d);
    }
}

void SpectrumWidget::paintHover(QPainter& p, const QRect& spec_r) {
    if (hover_x_ < 0 || drag_ != DragTarget::None || zoom_x0_ >= 0 ||
        panning_) {
        return;
    }
    if (hover_x_ < spec_r.left() || hover_x_ > spec_r.right()) return;

    const float nyq = 0.5f * static_cast<float>(state_.spectrum.sample_rate);
    const float hz = xToHz(hover_x_, spec_r);
    const int N = static_cast<int>(SPECTRUM_BINS);
    int bin = 0;
    if (nyq > 0.f) {
        bin = std::clamp(
            static_cast<int>(hz / nyq * static_cast<float>(N - 1) + 0.5f),
            0, N - 1);
    }
    const float db = state_.spectrum.power_db[static_cast<size_t>(bin)];

    QPen pen(style::TEXT_SECONDARY, 0.6, Qt::DotLine);
    p.setPen(pen);
    p.drawLine(QPointF(hover_x_, spec_r.top()),
               QPointF(hover_x_, spec_r.bottom()));

    QString text = QString("%1 kHz   %2 dB")
        .arg(hz / 1000.f, 0, 'f', 2)
        .arg(static_cast<double>(db), 0, 'f', 1);
    QFont small;
    small.setPixelSize(10);
    p.setFont(small);
    QFontMetrics fm(small);
    int tw = fm.horizontalAdvance(text) + 10;
    int th = fm.height() + 4;
    QRectF box(hover_x_ + 6, spec_r.bottom() - th - 4, tw, th);
    if (box.right() > spec_r.right()) box.moveLeft(hover_x_ - tw - 6);
    p.fillRect(box, QColor(20, 22, 28, 215));
    p.setPen(style::BORDER_NORM);
    p.drawRect(box);
    p.setPen(style::TEXT_PRIMARY);
    p.drawText(box, Qt::AlignCenter, text);
}

void SpectrumWidget::paintZoomBand(QPainter& p, const QRect& spec_r) {
    if (zoom_x0_ < 0) return;
    const int x0 = std::min(zoom_x0_, zoom_x1_);
    const int x1 = std::max(zoom_x0_, zoom_x1_);
    QRectF band(x0, spec_r.top(), x1 - x0, spec_r.height());
    QColor fill = style::C_SIGNAL;
    fill.setAlpha(36);
    p.fillRect(band, fill);
    QPen pen(style::C_SIGNAL, 0.8, Qt::DashLine);
    p.setPen(pen);
    p.drawRect(band);
}

// =========================================================================
// FCC §73.319 SCA emission mask overlay (composite-baseband):
//   Above 99 kHz from carrier center the modulation product must be
//   ≤ -25 dBc; for frequencies inside the SCA channel allocation the
//   inner shoulders are ~-25 dBc at the channel edge with a -35 dBc
//   floor 600 Hz beyond. Drawn as a stair envelope referenced to the
//   user's Fc/BW so the mask moves with the tuner.
// =========================================================================
void SpectrumWidget::paintMask(QPainter& p, const QRect& spec_r) {
    float fc, bw;
    {
        std::lock_guard<std::mutex> lock(state_.mtx);
        fc = state_.modem.center_freq;
        bw = state_.modem.signal_bw;
    }
    if (bw <= 0.f || state_.spectrum.sample_rate <= 0.f) return;

    float nyq = 0.5f * state_.spectrum.sample_rate;

    // Reference 0 dBc = peak in-band power; map to dB scale used by widget.
    // We anchor the mask at max_db_ (top) for visual clarity.
    float ref_db = max_db_ - 5.f;

    auto dbToYl = [&](float db) { return this->dbToY(db, spec_r); };

    QPen mask_pen(style::C_WARNING, 1.2, Qt::DashLine);
    mask_pen.setDashPattern({4, 3});
    p.setPen(mask_pen);
    p.setClipRect(spec_r);

    auto stair = [&](float f_lo, float f_hi, float db) {
        float x_lo = hzToX(f_lo, spec_r), x_hi = hzToX(f_hi, spec_r);
        float y    = dbToYl(ref_db + db);
        p.drawLine(QPointF(x_lo, y), QPointF(x_hi, y));
    };
    stair(0.f,                fc - bw * 0.5f - 600.f, -35.f);
    stair(fc - bw * 0.5f - 600.f, fc - bw * 0.5f,      -25.f);
    stair(fc - bw * 0.5f,     fc + bw * 0.5f,           0.f);
    stair(fc + bw * 0.5f,     fc + bw * 0.5f + 600.f, -25.f);
    stair(fc + bw * 0.5f + 600.f, nyq,                -35.f);

    auto vline = [&](float f, float db_top, float db_bot) {
        float x = hzToX(f, spec_r);
        p.drawLine(QPointF(x, dbToYl(ref_db + db_top)),
                   QPointF(x, dbToYl(ref_db + db_bot)));
    };
    vline(fc - bw * 0.5f - 600.f, -35.f, -25.f);
    vline(fc - bw * 0.5f,         -25.f,   0.f);
    vline(fc + bw * 0.5f,           0.f, -25.f);
    vline(fc + bw * 0.5f + 600.f, -25.f, -35.f);

    p.setClipping(false);
    p.setPen(style::C_WARNING);
    QFont small;
    small.setPixelSize(9);
    p.setFont(small);
    p.drawText(spec_r.adjusted(0, 0, -4, 0),
               Qt::AlignTop | Qt::AlignRight, "FCC §73.319 mask");
}

} // namespace gw
