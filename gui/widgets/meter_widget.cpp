/**
 * @file meter_widget.cpp
 */
#include "meter_widget.hpp"
#include <QPainter>
#include <QPainterPath>
#include <QLinearGradient>
#include <cmath>
#include <algorithm>

namespace gw {

MeterWidget::MeterWidget(QWidget* parent) : QWidget(parent) {
    setAttribute(Qt::WA_OpaquePaintEvent);
    setMinimumSize(60, 120);
}

void MeterWidget::onMetersUpdated(float tx_rms_db, float rx_rms_db,
                                   float tx_peak_db, float rx_peak_db,
                                   bool tx_clip, bool rx_clip) {
    tx_rms_ = tx_rms_db;  tx_peak_ = tx_peak_db;  tx_clip_ = tx_clip;
    rx_rms_ = rx_rms_db;  rx_peak_ = rx_peak_db;  rx_clip_ = rx_clip;
    update();
}

void MeterWidget::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    p.fillRect(rect(), style::BG_BASE);

    const float W = rect().width();
    const float H = rect().height();
    const float gap    = 6.f;
    const float bottom = 22.f;   // space for dB label
    const float top    = 8.f;
    const float bar_w  = (W - gap * 3.f) * 0.5f;
    const float bar_h  = H - bottom - top;

    QRectF tx_r(gap,           top, bar_w, bar_h);
    QRectF rx_r(gap * 2 + bar_w, top, bar_w, bar_h);

    paintBar(p, tx_r, norm(tx_rms_), norm(tx_peak_), tx_clip_, "TX");
    paintBar(p, rx_r, norm(rx_rms_), norm(rx_peak_), rx_clip_, "RX");

    // Squelch threshold marker on RX bar — small triangle pointing right
    {
        float sn = norm(squelch_db_);
        float sy = rx_r.bottom() - sn * bar_h;
        QPen sq_pen(style::C_WARNING, 1.0, Qt::DashLine);
        sq_pen.setDashPattern({3, 3});
        p.setPen(sq_pen);
        p.drawLine(QPointF(rx_r.left() - 2,  sy),
                   QPointF(rx_r.right() + 2, sy));
        // Small SQ label
        QFont sf;
        sf.setPixelSize(8);
        p.setFont(sf);
        p.setPen(style::C_WARNING);
        p.drawText(QRectF(rx_r.right() + 2, sy - 6, 18, 12),
                   Qt::AlignLeft | Qt::AlignVCenter, "SQ");
    }
    // PAPR readout under TX bar
    {
        QFont sf;
        sf.setPixelSize(9);
        p.setFont(sf);
        p.setPen(style::TEXT_TERTIARY);
        p.drawText(QRectF(tx_r.left(), tx_r.bottom() + 14,
                          tx_r.width(), 12),
                   Qt::AlignCenter,
                   QString("PAPR %1 dB").arg(tx_papr_db_, 0, 'f', 1));
    }

    // dB scale ticks (right of both bars)
    QFont f;
    f.setFamily("SF Mono, Menlo, DejaVu Sans Mono, monospace");
    f.setPixelSize(9);
    p.setFont(f);
    p.setPen(style::TEXT_TERTIARY);
    for (int db : {0, -6, -12, -20, -40, -60}) {
        float yn = norm(static_cast<float>(db));
        float y  = tx_r.bottom() - yn * bar_h;
        QString lbl = (db == 0) ? "0" : QString::number(db);
        p.drawText(QRectF(W - 22, y - 6, 20, 12),
                   Qt::AlignRight | Qt::AlignVCenter, lbl);
    }
}

void MeterWidget::paintBar(QPainter& p, const QRectF& r,
                             float rms_norm, float peak_norm,
                             bool clipping, const QString& label) {
    const float W = r.width();
    const float H = r.height();

    // Background track
    p.fillRect(r, style::METER_BG);
    p.setPen(QPen(style::BORDER_DIM, 0.5));
    p.drawRect(r);

    if (rms_norm <= 0.f) return;

    // Zone boundaries (normalized)
    constexpr float GREEN_TOP = 0.70f;  // 0–70% = green
    constexpr float AMBER_TOP = 0.90f;  // 70–90% = amber
    // above 90% = red

    float fill_h  = rms_norm * H;
    float fill_y  = r.bottom() - fill_h;

    // Draw zones bottom-to-top
    auto fillZone = [&](float zone_lo, float zone_hi, const QColor& col) {
        float y0 = r.bottom() - std::min(rms_norm, zone_hi)  * H;
        float y1 = r.bottom() - zone_lo * H;
        if (rms_norm < zone_lo) return;
        float actual_h = y1 - std::max(y0, fill_y);
        if (actual_h <= 0.f) return;
        QRectF zone(r.left() + 1, std::max(y0, fill_y),
                    W - 2, actual_h);
        p.fillRect(zone, col);
    };

    fillZone(0.f,        GREEN_TOP, style::METER_GREEN);
    fillZone(GREEN_TOP,  AMBER_TOP, style::METER_AMBER);
    fillZone(AMBER_TOP,  1.0f,      style::METER_RED);

    // Peak hold line
    if (peak_norm > 0.01f) {
        float py = r.bottom() - peak_norm * H;
        QColor peak_col = clipping
            ? style::C_ERROR
            : style::METER_PEAK;
        p.setPen(QPen(peak_col, 1.5));
        p.drawLine(QPointF(r.left() + 1, py),
                   QPointF(r.right() - 1, py));
    }

    // Clip indicator at very top
    if (clipping) {
        p.fillRect(QRectF(r.left() + 1, r.top() + 1, W - 2, 4), style::C_ERROR);
    }

    // Label below bar
    QFont f;
    f.setFamily("SF Mono, Menlo, DejaVu Sans Mono, monospace");
    f.setPixelSize(9);
    f.setLetterSpacing(QFont::AbsoluteSpacing, 0.5);
    p.setFont(f);
    p.setPen(style::TEXT_TERTIARY);
    p.drawText(QRectF(r.left(), r.bottom() + 4, W, 14),
               Qt::AlignHCenter | Qt::AlignTop, label);
}

} // namespace gw
