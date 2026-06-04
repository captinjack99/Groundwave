/**
 * @file eye_diagram_widget.cpp
 */
#include "eye_diagram_widget.hpp"
#include <QPainter>
#include <algorithm>
#include <cmath>
#include <vector>

namespace dsca {

EyeDiagramWidget::EyeDiagramWidget(QWidget* parent)
    : QWidget(parent)
{
    setMinimumHeight(80);
}

void EyeDiagramWidget::pushSamples(const float* samples, size_t n,
                                    int samples_per_symbol) {
    if (n == 0 || samples_per_symbol < 2) return;
    {
        std::lock_guard<std::mutex> lk(mtx_);
        samples_per_symbol_ = samples_per_symbol;
        // Slice the input into EYE_SYMBOLS-symbol-wide segments, each becomes
        // one trace overlay. Drop oldest when we exceed MAX_TRACES.
        const size_t trace_len = static_cast<size_t>(samples_per_symbol_)
                               * static_cast<size_t>(EYE_SYMBOLS);
        for (size_t off = 0; off + trace_len <= n; off += trace_len / 2) {
            std::vector<float> trace(samples + off, samples + off + trace_len);
            traces_.push_back(std::move(trace));
            while (traces_.size() > MAX_TRACES) traces_.pop_front();
        }
    }
    update();
}

void EyeDiagramWidget::paintEvent(QPaintEvent*) {
    QPainter p(this);
    // NB: antialiasing is deliberately NOT enabled for the dense trace overlay.
    // Stroking dozens of long, semi-transparent, near-vertical polylines with
    // the AA rasterizer costs ~100x more per segment on the native window
    // backing store — at MAX_TRACES that grew each eye paint to several SECONDS,
    // which backed up the GUI event queue and froze the UI whenever TX was on.
    // Aliased strokes are visually fine for an overlaid eye and keep the paint
    // bounded at a few ms. (Crosshair/labels below don't need AA either.)
    p.fillRect(rect(), style::BG_BASE);

    const QRectF r = rect().adjusted(8, 8, -8, -16);
    p.fillRect(r, QColor(10, 10, 14));

    // Center crosshair (decision instant + decision threshold)
    p.setPen(QPen(QColor(60, 60, 75), 0.5, Qt::DashLine));
    float cy = r.top() + r.height() * 0.5f;
    float cx = r.left() + r.width() * 0.5f;
    p.drawLine(QPointF(r.left(), cy), QPointF(r.right(), cy));
    p.drawLine(QPointF(cx, r.top()),  QPointF(cx, r.bottom()));

    std::vector<std::vector<float>> snap;
    int sps;
    {
        std::lock_guard<std::mutex> lk(mtx_);
        snap.assign(traces_.begin(), traces_.end());
        sps = samples_per_symbol_;
    }
    if (snap.empty() || sps < 2) {
        p.setPen(QColor(142, 142, 147));
        QFont f;
        f.setPixelSize(11);
        p.setFont(f);
        p.drawText(r, Qt::AlignCenter, "Eye diagram – waiting for samples");
        return;
    }

    // Normalize amplitude across all traces. Ignore non-finite samples so a
    // single NaN/Inf in the RX stream can't poison the normalization (a NaN
    // amax makes every plotted y NaN, and drawing a polyline through NaN
    // coordinates sends the rasterizer scanning an unbounded range — hundreds
    // of ms per paint, which froze the GUI).
    float amax = 0.f;
    for (auto& t : snap)
        for (float v : t)
            if (std::isfinite(v)) amax = std::max(amax, std::fabs(v));
    if (!(amax > 1e-9f)) amax = 1.f;

    // Use additive alpha to make density visible (more overlap = brighter).
    // Draw each trace with drawPolyline rather than QPainterPath::drawPath:
    // path stroking carries large per-element overhead that, at MAX_TRACES
    // long traces, made each eye paint hundreds of ms (the GUI-freeze cause);
    // drawPolyline on a flat QPointF buffer is the optimized polyline path and
    // is ~10-50x faster. Long traces are decimated to <= MAX_TRACE_PTS points
    // so the segment count stays bounded regardless of FFT size.
    // Width-0 = COSMETIC pen (always 1 device pixel). A non-zero geometric
    // width (the previous 0.9) makes Qt stroke every segment into a filled
    // polygon — at MAX_TRACES long traces that was ~240 ms/paint and the
    // primary cause of the TX GUI freeze. Cosmetic is the fast line path.
    p.setPen(QPen(QColor(64, 209, 88, 70), 0));
    std::vector<QPointF> pts;
    pts.reserve(static_cast<size_t>(MAX_TRACE_PTS) + 1);
    for (auto& t : snap) {
        if (t.size() < 2) continue;
        const size_t n = t.size();
        const size_t step = (n > static_cast<size_t>(MAX_TRACE_PTS))
                            ? n / static_cast<size_t>(MAX_TRACE_PTS) : 1;
        pts.clear();
        for (size_t i = 0; i < n; i += step) {
            float x = r.left() + (r.width() * static_cast<float>(i))
                      / static_cast<float>(n - 1);
            float s = std::isfinite(t[i]) ? t[i] : 0.f;
            float v = std::clamp(s / amax, -1.f, 1.f);
            float y = r.top() + (1.f - (v * 0.5f + 0.5f)) * r.height();
            pts.emplace_back(x, y);
        }
        if (pts.size() >= 2)
            p.drawPolyline(pts.data(), static_cast<int>(pts.size()));
    }

    p.setPen(QColor(142, 142, 147));
    QFont small;
    small.setPixelSize(10);
    p.setFont(small);
    p.drawText(rect().adjusted(8, 0, -8, -2),
               Qt::AlignLeft | Qt::AlignBottom,
               QString("Eye diagram (%1 traces, %2 sps)")
                   .arg(static_cast<int>(snap.size())).arg(sps));
}

} // namespace dsca
