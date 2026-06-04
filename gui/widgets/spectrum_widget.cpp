/**
 * @file spectrum_widget.cpp
 */
#include "spectrum_widget.hpp"

#include <QPainter>
#include <QPainterPath>
#include <QLinearGradient>
#include <QResizeEvent>
#include <QWheelEvent>
#include <QMouseEvent>
#include <QFontMetrics>
#include <cmath>
#include <algorithm>

namespace dsca {

SpectrumWidget::SpectrumWidget(AppState& state, QWidget* parent)
    : QWidget(parent), state_(state)
{
    setMinimumSize(240, 160);
    setAttribute(Qt::WA_OpaquePaintEvent);
    // Init waterfall image with current size
    waterfall_img_ = QImage(
        static_cast<int>(SPECTRUM_BINS),
        static_cast<int>(WATERFALL_ROWS),
        QImage::Format_RGB32);
    waterfall_img_.fill(QColor(style::WFALL_COLD).rgb());
}

void SpectrumWidget::onSpectrumReady() {
    if (auto_range_) {
        const SpectrumData& sd = state_.spectrum;
        float new_max = sd.peak_db + 6.f;
        float new_min = sd.noise_floor - 12.f;
        if (new_max - new_min < 30.f) new_min = new_max - 60.f;
        // Smooth range change
        max_db_ = max_db_ * 0.92f + new_max * 0.08f;
        min_db_ = min_db_ * 0.92f + new_min * 0.08f;
    }
    updateWaterfall();
    update();
}

void SpectrumWidget::setDbRange(float min_db, float max_db) {
    // Enforce a minimum non-zero span. dbToY / the waterfall colormap
    // divide by (max_db_ - min_db_); a degenerate min==max range would
    // produce NaN/inf through std::clamp and paint garbage. (#50)
    if (max_db <= min_db) max_db = min_db + 1.f;
    min_db_ = min_db;
    max_db_ = max_db;
    update();
}

void SpectrumWidget::resizeEvent(QResizeEvent* e) {
    QWidget::resizeEvent(e);
    update();
}

void SpectrumWidget::wheelEvent(QWheelEvent* e) {
    // Scroll to zoom dB range
    float delta = e->angleDelta().y() / 120.f;
    float range = max_db_ - min_db_;
    range = std::clamp(range - delta * 5.f, 10.f, 160.f);
    float center = (max_db_ + min_db_) * 0.5f;
    min_db_ = center - range * 0.5f;
    max_db_ = center + range * 0.5f;
    auto_range_ = false;
    emit dbRangeChanged(min_db_, max_db_);
    update();
}

// =========================================================================
// Paint
// =========================================================================

void SpectrumWidget::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    p.setRenderHint(QPainter::SmoothPixmapTransform);

    const QRect full  = rect();
    const int   spec_h = static_cast<int>(full.height() * SPEC_FRAC);
    const QRect spec_r(full.left(), full.top(), full.width(), spec_h);
    const QRect wfall_r(full.left(), spec_h, full.width(), full.height() - spec_h);

    // Margins for axis labels
    const int margin_left  = 42;
    const int margin_bottom = 18;
    const int margin_right  =  8;
    const QRect plot_r(
        spec_r.left() + margin_left,
        spec_r.top() + 8,
        spec_r.width() - margin_left - margin_right,
        spec_r.height() - 8 - margin_bottom);

    paintSpectrum(p, plot_r);
    paintAxes(p, plot_r);
    if (show_mask_) paintMask(p, plot_r);
    paintCursors(p, plot_r);
    paintWaterfall(p, wfall_r);
}

float SpectrumWidget::dbToY(float db, const QRect& r) const {
    float norm = (db - min_db_) / (max_db_ - min_db_);
    norm = std::clamp(norm, 0.f, 1.f);
    return r.bottom() - norm * r.height();
}

void SpectrumWidget::paintSpectrum(QPainter& p, const QRect& r) {
    // Background
    p.fillRect(rect().adjusted(0, 0, 0, -static_cast<int>(rect().height() * (1.f - SPEC_FRAC))),
               style::BG_BASE);

    const SpectrumData& sd = state_.spectrum;
    const int W = r.width();
    const int N = static_cast<int>(SPECTRUM_BINS);

    if (W < 2 || N < 2) return;

    // --- Grid lines ---
    p.setPen(QPen(style::SPEC_GRID, 0.5));
    // Horizontal: every 10 dB
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

    // Vertical: every ~10% of bandwidth
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
    // Vertical dashed lines marking the configured signal band so the user
    // can visually verify where the modem is tuned vs the actual measured
    // spectrum. Frequency axis covers [0, Nyquist = sample_rate/2].
    {
        float fc, bw;
        uint32_t sr;
        {
            std::lock_guard<std::mutex> lock(state_.mtx);
            fc = state_.modem.center_freq;
            bw = state_.modem.signal_bw;
            sr = state_.ofdm.sample_rate;
        }
        if (sr > 0) {
            float nyq = static_cast<float>(sr) * 0.5f;
            auto xFor = [&](float freq_hz) {
                float t = std::clamp(freq_hz / nyq, 0.f, 1.f);
                return r.left() + t * static_cast<float>(r.width());
            };
            float x_lo  = xFor(fc - bw * 0.5f);
            float x_hi  = xFor(fc + bw * 0.5f);
            float x_ctr = xFor(fc);

            // Edges in subtle electric blue
            QPen edge_pen(style::C_SIGNAL, 0.9, Qt::DashLine);
            edge_pen.setDashPattern({3, 4});
            p.setPen(edge_pen);
            p.drawLine(QPointF(x_lo, r.top()),
                       QPointF(x_lo, r.bottom()));
            p.drawLine(QPointF(x_hi, r.top()),
                       QPointF(x_hi, r.bottom()));

            // Center line slightly stronger
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
        }
    }

    // --- Build spectrum trace path (reused member: clear() keeps capacity,
    //     so no per-frame ~N-point allocation) ---
    trace_path_.clear();
    bool first = true;
    for (int i = 0; i < N; ++i) {
        float x = r.left() + (static_cast<float>(i) / (N - 1)) * r.width();
        float y = dbToY(sd.power_db[static_cast<size_t>(i)], r);
        if (first) { trace_path_.moveTo(x, y); first = false; }
        else        trace_path_.lineTo(x, y);
    }

    // --- Gradient fill under trace (reused member; addPath copies the
    //     already-built points into the retained buffer — no fresh copy) ---
    {
        fill_path_.clear();
        fill_path_.addPath(trace_path_);
        fill_path_.lineTo(r.right(), r.bottom());
        fill_path_.lineTo(r.left(),  r.bottom());
        fill_path_.closeSubpath();

        QLinearGradient grad(0, r.top(), 0, r.bottom());
        grad.setColorAt(0.0, style::SPEC_FILL_TOP);
        grad.setColorAt(1.0, style::SPEC_FILL_BOT);
        p.fillPath(fill_path_, grad);
    }

    // --- Trace line ---
    p.setRenderHint(QPainter::Antialiasing, true);
    p.setPen(QPen(style::SPEC_TRACE, 1.2));
    p.drawPath(trace_path_);

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

    // Frequency axis (bottom)
    float sr = sd.sample_rate > 0.f ? sd.sample_rate : 48000.f;
    float nyquist = sr / 2.f;
    int freq_steps[] = {0, 4000, 8000, 12000, 16000, 20000, 24000};
    for (int freq : freq_steps) {
        if (freq > nyquist + 1) break;
        float x = r.left() + (static_cast<float>(freq) / nyquist) * r.width();
        QString label;
        if (freq == 0)           label = "0";
        else if (freq % 1000 == 0) label = QString("%1k").arg(freq / 1000);
        else                     label = QString("%1").arg(freq);
        p.drawText(QRectF(x - 24, r.bottom() + 3, 48, 14),
                   Qt::AlignHCenter | Qt::AlignTop, label);
    }
}

// =========================================================================
// Waterfall
// =========================================================================

QColor SpectrumWidget::dbToWaterfallColor(float db) const {
    // Map [min_db_, max_db_] → [0,1] → colormap
    float t = (db - min_db_) / (max_db_ - min_db_);
    t = std::clamp(t, 0.f, 1.f);

    // Colormap: deep navy → teal → cyan → yellow → white
    struct Stop { float t; uint8_t r, g, b; };
    static constexpr Stop stops[] = {
        {0.00f,   5,   8,  28},   // deep navy
        {0.20f,   6,  40,  80},   // dark blue
        {0.40f,   0, 100, 160},   // blue
        {0.58f,   0, 190, 220},   // cyan
        {0.75f,  80, 220, 120},   // cyan-green
        {0.88f, 240, 200,  30},   // yellow
        {1.00f, 255, 255, 200},   // near-white
    };
    constexpr int N = sizeof(stops) / sizeof(stops[0]);

    // Find bracket
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

void SpectrumWidget::updateWaterfall() {
    const SpectrumData& sd = state_.spectrum;

    // Scroll existing image down by 1 pixel
    if (waterfall_img_.width() != static_cast<int>(SPECTRUM_BINS) ||
        waterfall_img_.height() != static_cast<int>(WATERFALL_ROWS)) {
        waterfall_img_ = QImage(
            static_cast<int>(SPECTRUM_BINS),
            static_cast<int>(WATERFALL_ROWS),
            QImage::Format_RGB32);
        waterfall_img_.fill(QColor(style::WFALL_COLD).rgb());
    }

    // Shift all rows down by 1
    if (WATERFALL_ROWS > 1) {
        memmove(waterfall_img_.bits() + waterfall_img_.bytesPerLine(),
                waterfall_img_.bits(),
                waterfall_img_.bytesPerLine() * (WATERFALL_ROWS - 1));
    }

    // Write newest row at top
    QRgb* top_row = reinterpret_cast<QRgb*>(waterfall_img_.bits());
    for (size_t i = 0; i < SPECTRUM_BINS; ++i) {
        top_row[i] = dbToWaterfallColor(sd.power_db[i]).rgb();
    }
}

void SpectrumWidget::paintWaterfall(QPainter& p, const QRect& r) {
    // Fill background
    p.fillRect(r, style::BG_BASE);

    const int margin_left  = 42;
    const int margin_right =  8;
    QRect dst(r.left() + margin_left, r.top(),
              r.width() - margin_left - margin_right, r.height());

    p.drawImage(dst, waterfall_img_);

    // Subtle top separator line
    p.setPen(QPen(style::BORDER_DIM, 0.5));
    p.drawLine(r.topLeft(), r.topRight());
}

// =========================================================================
// Mouse cursors: click sets primary, shift-click sets secondary delta cursor
// =========================================================================

float SpectrumWidget::xToHz(int x, const QRect& spec_r) const {
    if (spec_r.width() <= 0) return 0.f;
    float t = static_cast<float>(x - spec_r.left()) / spec_r.width();
    t = std::clamp(t, 0.f, 1.f);
    float nyq = 0.5f * static_cast<float>(state_.spectrum.sample_rate);
    return t * nyq;
}

void SpectrumWidget::mousePressEvent(QMouseEvent* e) {
    // Plot rect (recomputed; matches paintEvent layout)
    int spec_h = static_cast<int>(rect().height() * SPEC_FRAC);
    QRect spec_r(0, 0, rect().width(), spec_h);
    if (!spec_r.contains(e->pos())) {
        QWidget::mousePressEvent(e);
        return;
    }
    if (e->modifiers() & Qt::ShiftModifier) {
        c2_x_ = e->pos().x();
    } else {
        c1_x_ = e->pos().x();
        c2_x_ = -1;
    }
    update();
}

void SpectrumWidget::paintCursors(QPainter& p, const QRect& spec_r) {
    if (c1_x_ < 0) return;
    auto drawCursor = [&](int x, QColor c, const QString& label) {
        if (x < spec_r.left() || x > spec_r.right()) return;
        QPen pen(c, 1.0, Qt::SolidLine);
        p.setPen(pen);
        p.drawLine(QPointF(x, spec_r.top()), QPointF(x, spec_r.bottom()));
        // Label box
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
        // Delta box at top center
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
    auto hzToX = [&](float h) {
        float t = std::clamp(h / nyq, 0.f, 1.f);
        return spec_r.left() + t * spec_r.width();
    };

    // Reference 0 dBc = peak in-band power; map to dB scale used by widget.
    // We anchor the mask at max_db_ (top) for visual clarity.
    float ref_db = max_db_ - 5.f;

    // Mask shoulder corners (relative to Fc):
    //   |f - fc| ≤ bw/2   → 0 dBc        (in-band)
    //   bw/2 < |f - fc| ≤ bw/2 + 0.6 kHz → -25 dBc
    //   beyond                            → -35 dBc
    auto dbToY = [&](float db) { return this->dbToY(db, spec_r); };

    QPen mask_pen(style::C_WARNING, 1.2, Qt::DashLine);
    mask_pen.setDashPattern({4, 3});
    p.setPen(mask_pen);

    auto stair = [&](float f_lo, float f_hi, float db) {
        float x_lo = hzToX(f_lo), x_hi = hzToX(f_hi);
        float y    = dbToY(ref_db + db);
        p.drawLine(QPointF(x_lo, y), QPointF(x_hi, y));
    };
    stair(0.f,                fc - bw * 0.5f - 600.f, -35.f);
    stair(fc - bw * 0.5f - 600.f, fc - bw * 0.5f,      -25.f);
    stair(fc - bw * 0.5f,     fc + bw * 0.5f,           0.f);
    stair(fc + bw * 0.5f,     fc + bw * 0.5f + 600.f, -25.f);
    stair(fc + bw * 0.5f + 600.f, nyq,                -35.f);

    // Vertical drops between segments (cosmetic)
    auto vline = [&](float f, float db_top, float db_bot) {
        float x = hzToX(f);
        p.drawLine(QPointF(x, dbToY(ref_db + db_top)),
                   QPointF(x, dbToY(ref_db + db_bot)));
    };
    vline(fc - bw * 0.5f - 600.f, -35.f, -25.f);
    vline(fc - bw * 0.5f,         -25.f,   0.f);
    vline(fc + bw * 0.5f,           0.f, -25.f);
    vline(fc + bw * 0.5f + 600.f, -25.f, -35.f);

    // Label
    p.setPen(style::C_WARNING);
    QFont small;
    small.setPixelSize(9);
    p.setFont(small);
    p.drawText(spec_r.adjusted(0, 0, -4, 0),
               Qt::AlignTop | Qt::AlignRight, "FCC §73.319 mask");
}

} // namespace dsca
