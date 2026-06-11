/**
 * @file hier_flow_widget.hpp
 * @brief Visual signal-flow diagram for hierarchical M/S encoding.
 *
 * Renders an educational illustration showing how stereo audio flows
 * through the hierarchical-modulation chain:
 *
 *   Stream 0 (L) ─┐
 *                  ├─► M = (L+R)/2 ─► Opus ─► LDPC ─► HP bits ─┐
 *   Stream 1 (R) ─┤                                              ├─► QAM symbol
 *                  └─► S = (L-R)/2 ─► Opus ─► LDPC ─► LP bits ─┘   (HP+LP bps)
 *
 * Updates live to reflect the current HP/LP bit split and α.
 * Renders a small inset constellation showing HP cluster boundaries
 * (the inner cross at α-scaled spacing) and LP cluster positions.
 */
#pragma once

#include "../../include/hierarchical_mod.hpp"
#include <QWidget>

namespace gw {

class HierFlowWidget : public QWidget {
    Q_OBJECT
public:
    explicit HierFlowWidget(QWidget* parent = nullptr);

    /** Update with the current hierarchical configuration. Triggers a
     *  repaint. Pass `enabled=false` to render the disabled state. */
    void setConfig(const HierarchicalConfig& cfg);

protected:
    void paintEvent(QPaintEvent* e) override;
    QSize sizeHint() const override { return QSize(540, 280); }
    QSize minimumSizeHint() const override { return QSize(420, 220); }

private:
    HierarchicalConfig cfg_{};

    /** Draw a constellation inset with HP quadrant boundaries +
     *  LP cluster positions in the bottom-right corner. */
    void drawConstellationInset(QPainter& p, int x, int y, int w, int h) const;

    /** Draw the flow arrows + boxes for the M/S split through
     *  HP/LP layers. Top-half of the widget. */
    void drawSignalFlow(QPainter& p, int x, int y, int w, int h) const;
};

} // namespace gw
