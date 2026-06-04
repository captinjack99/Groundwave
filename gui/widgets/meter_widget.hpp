/**
 * @file meter_widget.hpp
 * @brief Broadcast VU meter — Logic Pro style, custom painted
 *
 * Vertical bar with three color zones, peak hold line, numeric readout.
 * Two bars side-by-side for TX and RX.
 */
#pragma once
#include "../style.hpp"
#include <QWidget>

namespace dsca {

class MeterWidget : public QWidget {
    Q_OBJECT
public:
    explicit MeterWidget(QWidget* parent = nullptr);

    // Slot for DataBridge::metersUpdated
    void onMetersUpdated(float tx_rms_db, float rx_rms_db,
                         float tx_peak_db, float rx_peak_db,
                         bool tx_clip, bool rx_clip);

    /** Squelch threshold dBFS to draw on the RX bar (no line if NaN). */
    void setSquelchThreshold(float db) { squelch_db_ = db; update(); }

    /** TX PAPR (dB) shown as a small numeric readout below the TX bar. */
    void setTxPapr(float db) { tx_papr_db_ = db; update(); }

    QSize sizeHint() const override { return {100, 200}; }
    QSize minimumSizeHint() const override { return {60, 120}; }

protected:
    void paintEvent(QPaintEvent*) override;

private:
    void paintBar(QPainter& p, const QRectF& bar_r,
                  float rms_norm, float peak_norm,
                  bool clipping, const QString& label);

    // normalize -60..0 dBFS → 0..1
    static float norm(float db) {
        return std::max(0.f, std::min(1.f, (db + 60.f) / 60.f));
    }

    float tx_rms_  = 0.f, tx_peak_  = 0.f; bool tx_clip_  = false;
    float rx_rms_  = 0.f, rx_peak_  = 0.f; bool rx_clip_  = false;
    float squelch_db_ = -55.f;   ///< RX squelch threshold (dBFS)
    float tx_papr_db_ = 0.f;     ///< TX PAPR readout
};

} // namespace dsca
