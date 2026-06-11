/**
 * @file channel_response_widget.cpp
 */
#include "channel_response_widget.hpp"
#include <QPainter>
#include <algorithm>
#include <cmath>
#include <vector>

namespace gw {

ChannelResponseWidget::ChannelResponseWidget(AppState& state, QWidget* parent)
    : QWidget(parent), state_(state)
{
    setMinimumHeight(80);
}

void ChannelResponseWidget::pushEstimate(const std::vector<float>& mag_db) {
    {
        std::lock_guard<std::mutex> lk(mtx_);
        mag_db_ = mag_db;
    }
    update();
}

void ChannelResponseWidget::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    p.fillRect(rect(), style::BG_BASE);

    const QRectF r = rect().adjusted(36, 6, -6, -16);
    p.fillRect(r, QColor(10, 10, 14));

    // Vertical scale: ±20 dB around 0
    constexpr float DB_TOP =  20.f;
    constexpr float DB_BOT = -40.f;
    auto dbToY = [&](float db) {
        float t = (DB_TOP - db) / (DB_TOP - DB_BOT);
        t = std::clamp(t, 0.f, 1.f);
        return r.top() + t * r.height();
    };

    // Grid: dB lines every 10 dB
    p.setPen(QPen(style::SPEC_GRID, 0.5));
    QFont small;
    small.setPixelSize(9);
    p.setFont(small);
    for (int db = -40; db <= 20; db += 10) {
        float y = dbToY(static_cast<float>(db));
        p.drawLine(QPointF(r.left(), y), QPointF(r.right(), y));
        p.setPen(QColor(120, 120, 130));
        p.drawText(QRectF(0, y - 7, 32, 14),
                   Qt::AlignRight | Qt::AlignVCenter,
                   QString::number(db));
        p.setPen(QPen(style::SPEC_GRID, 0.5));
    }

    // Trace
    std::vector<float> snap;
    {
        std::lock_guard<std::mutex> lk(mtx_);
        snap = mag_db_;
    }
    if (snap.size() >= 2) {
        // Cosmetic (width-0) pen + drawPolyline + decimation. snap is one point
        // per subcarrier (~fft_size points: thousands at FFT>=2048); a geometric
        // pen (the previous width 1.4) strokes every segment into a filled
        // polygon, which at large FFT was hundreds of ms to seconds per paint —
        // the same GUI-freeze class as the eye-diagram bug. Cap the drawn points
        // and use the fast cosmetic line path so cost is bounded vs FFT size.
        constexpr int MAX_PTS = 512;
        const size_t n = snap.size();
        const size_t step = (n > static_cast<size_t>(MAX_PTS))
                          ? n / static_cast<size_t>(MAX_PTS) : 1;
        std::vector<QPointF> pts;
        pts.reserve(static_cast<size_t>(MAX_PTS) + 1);
        for (size_t i = 0; i < n; i += step) {
            float x = r.left() + (r.width() * static_cast<float>(i))
                      / static_cast<float>(n - 1);
            float y = dbToY(std::isfinite(snap[i]) ? snap[i] : DB_BOT);
            pts.emplace_back(x, y);
        }
        p.setPen(QPen(style::C_SIGNAL, 0));
        if (pts.size() >= 2)
            p.drawPolyline(pts.data(), static_cast<int>(pts.size()));
    } else {
        // Empty state: no estimate until the receiver acquires a preamble.
        p.setPen(QColor(142, 142, 147));
        QFont f;
        f.setPixelSize(11);
        p.setFont(f);
        p.drawText(r, Qt::AlignCenter,
                   "Channel response — waiting for acquisition");
    }

    // Title
    p.setPen(QColor(142, 142, 147));
    QFont title;
    title.setPixelSize(10);
    p.setFont(title);
    p.drawText(rect().adjusted(6, -1, -6, -2),
               Qt::AlignLeft | Qt::AlignBottom,
               "|H(f)|  channel response  (subcarrier index →)");
}

} // namespace gw
