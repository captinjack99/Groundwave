/**
 * @file hier_flow_widget.cpp
 * @brief Live signal-flow diagram + constellation inset for hierarchical M/S.
 */
#include "hier_flow_widget.hpp"
#include "../style.hpp"

#include <QPainter>
#include <QPaintEvent>
#include <QFont>
#include <QFontMetrics>
#include <QPainterPath>
#include <cmath>

namespace dsca {

HierFlowWidget::HierFlowWidget(QWidget* parent) : QWidget(parent) {
    setMinimumSize(420, 220);
    setAutoFillBackground(false);
}

void HierFlowWidget::setConfig(const HierarchicalConfig& cfg) {
    cfg_ = cfg;
    update();
}

void HierFlowWidget::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);

    // Background
    p.fillRect(rect(), QColor(14, 14, 20));

    int w = width();
    int h = height();
    int inset_size = std::min(160, std::min(w, h) - 40);

    // Top half (signal flow) and bottom-right (constellation inset)
    drawSignalFlow(p, 8, 8, w - inset_size - 16, h - 16);
    drawConstellationInset(p, w - inset_size - 8,
                              h - inset_size - 8,
                              inset_size, inset_size);
}

void HierFlowWidget::drawSignalFlow(QPainter& p, int x, int y,
                                     int w, int h) const {
    (void)h;
    const bool enabled = cfg_.enabled &&
                         cfg_.effectiveHP() > 0 &&
                         cfg_.effectiveLP() > 0;
    const QColor dim   (72, 72, 80);
    const QColor norm  (200, 200, 210);
    const QColor accent(0, 153, 255);
    const QColor mid_c (48, 209, 88);   // green for HP/Mid
    const QColor side_c(255, 159, 10);  // amber for LP/Side

    QFont label_font(p.font());
    label_font.setPixelSize(10);
    label_font.setLetterSpacing(QFont::PercentageSpacing, 105);
    QFont mono_font("SF Mono");
    mono_font.setPixelSize(10);

    auto box = [&](int bx, int by, int bw, int bh,
                    const QString& text, const QColor& color, bool active) {
        QColor fill = active ? QColor(color.red(), color.green(), color.blue(), 30)
                              : QColor(28, 28, 36);
        QColor edge = active ? color : dim;
        p.setPen(QPen(edge, 1));
        p.setBrush(fill);
        p.drawRoundedRect(bx, by, bw, bh, 4, 4);
        p.setFont(label_font);
        p.setPen(active ? norm : dim);
        p.drawText(QRect(bx, by, bw, bh), Qt::AlignCenter, text);
    };

    auto arrow = [&](int x1, int y1, int x2, int y2, bool active) {
        p.setPen(QPen(active ? accent : dim, 1));
        p.drawLine(x1, y1, x2, y2);
        // arrowhead
        QPolygonF tri;
        if (x2 > x1) {
            tri << QPointF(x2, y2) << QPointF(x2 - 5, y2 - 3) << QPointF(x2 - 5, y2 + 3);
        } else if (y2 > y1) {
            tri << QPointF(x2, y2) << QPointF(x2 - 3, y2 - 5) << QPointF(x2 + 3, y2 - 5);
        }
        p.setBrush(active ? accent : dim);
        p.drawPolygon(tri);
    };

    // Layout: 4 columns × 2 rows
    //   Col 1: Stream 0/1 inputs
    //   Col 2: M/S matrix
    //   Col 3: Per-layer Opus + LDPC
    //   Col 4: Hierarchical mapper → QAM symbol
    int col_w  = (w - 24) / 4;
    int row_h  = 36;
    int gap_y  = 24;
    int y_top  = y + 28;
    int y_bot  = y_top + row_h + gap_y;

    int col_x[4];
    for (int i = 0; i < 4; ++i) col_x[i] = x + 8 + i * (col_w + 4);

    // Title
    QFont tfont(p.font());
    tfont.setPixelSize(11);
    tfont.setWeight(QFont::DemiBold);
    p.setFont(tfont);
    p.setPen(QColor(142, 142, 147));
    p.drawText(QRect(x, y + 4, w, 18), Qt::AlignLeft,
               enabled ? "M/S OVER HIERARCHICAL MODULATION"
                       : "M/S OVER HIERARCHICAL MODULATION  ·  (disabled)");

    // ---- Column 1: Stream inputs ----
    box(col_x[0], y_top, col_w, row_h, "Stream 0\n(Left)",
        enabled ? norm : dim, enabled);
    box(col_x[0], y_bot, col_w, row_h, "Stream 1\n(Right)",
        enabled ? norm : dim, enabled);

    // ---- Column 2: M/S matrix ----
    int matrix_x = col_x[1];
    int matrix_y = y_top + (row_h * 2 + gap_y) / 2 - row_h / 2 - 4;
    int matrix_h = row_h * 2 + gap_y;
    QColor matrix_edge = enabled ? accent : dim;
    p.setPen(QPen(matrix_edge, 1, Qt::DashLine));
    p.setBrush(QColor(accent.red(), accent.green(), accent.blue(), 12));
    p.drawRoundedRect(matrix_x, matrix_y - 8, col_w, matrix_h + 16, 6, 6);
    p.setFont(mono_font);
    p.setPen(enabled ? norm : dim);
    p.drawText(QRect(matrix_x, matrix_y, col_w, row_h),
               Qt::AlignCenter, "M = (L+R)/2");
    p.drawText(QRect(matrix_x, matrix_y + row_h + gap_y, col_w, row_h),
               Qt::AlignCenter, "S = (L-R)/2");
    p.setFont(label_font);
    p.setPen(QColor(110, 110, 120));
    p.drawText(QRect(matrix_x, matrix_y - 22, col_w, 16),
               Qt::AlignCenter, "M/S MATRIX");

    // ---- Column 3: Opus → LDPC per layer ----
    auto draw_layer = [&](int by, const QString& title, const QColor& c) {
        // Stacked sub-boxes: Opus → LDPC
        int sub_h = (row_h - 2) / 2;
        QString opus_label = QString("Opus  %1").arg(title);
        QString ldpc_label = QString("LDPC  %1").arg(title);
        box(col_x[2], by,         col_w, sub_h, opus_label, c, enabled);
        box(col_x[2], by + sub_h + 2, col_w, sub_h, ldpc_label, c, enabled);
    };
    draw_layer(y_top, "Mid",  mid_c);
    draw_layer(y_bot, "Side", side_c);

    // ---- Column 4: HP/LP bits → constellation symbol ----
    uint8_t hp_bps = cfg_.effectiveHP();
    uint8_t lp_bps = cfg_.effectiveLP();
    uint8_t total = hp_bps + lp_bps;
    QString hp_label = enabled
        ? QString("HP %1 bps\n(robust)").arg(hp_bps)
        : QString("HP layer");
    QString lp_label = enabled
        ? QString("LP %1 bps\n(graceful)").arg(lp_bps)
        : QString("LP layer");
    box(col_x[3], y_top, col_w, row_h, hp_label,  mid_c,  enabled);
    box(col_x[3], y_bot, col_w, row_h, lp_label,  side_c, enabled);

    // Total bits summary below column 4
    if (enabled) {
        p.setFont(mono_font);
        p.setPen(accent);
        p.drawText(QRect(col_x[3], y_bot + row_h + 4, col_w, 16),
                   Qt::AlignCenter,
                   QString("→ %1+%2 = %3 bps QAM   α=%4")
                       .arg(hp_bps).arg(lp_bps).arg(total)
                       .arg(cfg_.alpha, 0, 'f', 2));
    }

    // ---- Arrows ----
    // Stream 0 → M, Stream 1 → M (both contribute to Mid)
    int mid_arrow_y = matrix_y + row_h / 2;
    int side_arrow_y = matrix_y + row_h + gap_y + row_h / 2;
    arrow(col_x[0] + col_w, y_top + row_h / 2, matrix_x, mid_arrow_y, enabled);
    arrow(col_x[0] + col_w, y_bot + row_h / 2, matrix_x, side_arrow_y, enabled);
    // Stream 0 also feeds Side (diagonal); Stream 1 feeds Mid (already shown)
    // For clarity skip the cross-arrows — the labels make the math clear.

    // Matrix → Opus (Mid), Matrix → Opus (Side)
    arrow(matrix_x + col_w, mid_arrow_y,  col_x[2], y_top + row_h / 2, enabled);
    arrow(matrix_x + col_w, side_arrow_y, col_x[2], y_bot + row_h / 2, enabled);
    // LDPC → HP/LP boxes
    arrow(col_x[2] + col_w, y_top + row_h / 2, col_x[3], y_top + row_h / 2, enabled);
    arrow(col_x[2] + col_w, y_bot + row_h / 2, col_x[3], y_bot + row_h / 2, enabled);
}

void HierFlowWidget::drawConstellationInset(QPainter& p, int x, int y,
                                             int w, int h) const {
    const bool enabled = cfg_.enabled &&
                         cfg_.effectiveHP() > 0 &&
                         cfg_.effectiveLP() > 0;

    // Container
    p.setPen(QPen(QColor(40, 40, 52), 1));
    p.setBrush(QColor(10, 10, 14));
    p.drawRoundedRect(x, y, w, h, 6, 6);

    QFont lbl(p.font());
    lbl.setPixelSize(9);
    lbl.setLetterSpacing(QFont::PercentageSpacing, 105);
    p.setFont(lbl);
    p.setPen(QColor(110, 110, 120));
    p.drawText(QRect(x, y + 4, w, 14), Qt::AlignCenter, "CONSTELLATION");

    if (!enabled) {
        p.setPen(QColor(72, 72, 80));
        p.drawText(QRect(x, y, w, h), Qt::AlignCenter, "uniform");
        return;
    }

    // Draw the hierarchical constellation: HP defines quadrant centers
    // (split by α), LP populates each quadrant with sub-points.
    int padding = 22;
    int cx = x + w / 2;
    int cy = y + h / 2;
    int half_extent = std::min(w, h) / 2 - padding;

    // HP layer: cross-hatch at α-scaled center.
    // For α > 1, HP quadrants are pulled in toward the origin.
    // Render HP cluster center positions.
    uint8_t hp_bps = cfg_.effectiveHP();
    uint8_t lp_bps = cfg_.effectiveLP();
    uint8_t hp_per_axis = hp_bps / 2;
    uint8_t lp_per_axis = lp_bps / 2;
    int hp_grid = 1 << hp_per_axis;  // points per axis from HP
    int lp_grid = 1 << lp_per_axis;
    float alpha = cfg_.alpha;

    // HP boundary cross (origin = quadrant divider for HP).
    p.setPen(QPen(QColor(48, 209, 88, 120), 1, Qt::DashLine));
    p.drawLine(cx - half_extent, cy, cx + half_extent, cy);
    p.drawLine(cx, cy - half_extent, cx, cy + half_extent);

    // Constellation points. Map each (hp_i, hp_q, lp_i, lp_q) tuple to a
    // position. HP spacing is α larger than LP spacing.
    float lp_step = static_cast<float>(half_extent) /
                     (alpha * hp_grid + lp_grid);
    float hp_step = lp_step * alpha;

    for (int hi = 0; hi < hp_grid; ++hi) {
        for (int hq = 0; hq < hp_grid; ++hq) {
            // HP cluster center
            float hp_x = (static_cast<float>(hi) - (hp_grid - 1) * 0.5f)
                         * (hp_step * lp_grid * 2);
            float hp_y = (static_cast<float>(hq) - (hp_grid - 1) * 0.5f)
                         * (hp_step * lp_grid * 2);
            for (int li = 0; li < lp_grid; ++li) {
                for (int lq = 0; lq < lp_grid; ++lq) {
                    float lp_x = (static_cast<float>(li) - (lp_grid - 1) * 0.5f)
                                  * (lp_step * 2);
                    float lp_y = (static_cast<float>(lq) - (lp_grid - 1) * 0.5f)
                                  * (lp_step * 2);
                    float px = cx + hp_x + lp_x;
                    float py = cy - (hp_y + lp_y);
                    p.setPen(QColor(0, 153, 255));
                    p.setBrush(QColor(0, 153, 255, 200));
                    p.drawEllipse(QPointF(px, py), 1.8, 1.8);
                }
            }
        }
    }

    // Legend
    p.setFont(lbl);
    p.setPen(QColor(48, 209, 88));
    p.drawText(QRect(x + 6, y + h - 28, w - 12, 12),
               Qt::AlignLeft, "── HP boundary (Mid)");
    p.setPen(QColor(0, 153, 255));
    p.drawText(QRect(x + 6, y + h - 16, w - 12, 12),
               Qt::AlignLeft, "● LP point (Side)");
}

} // namespace dsca
