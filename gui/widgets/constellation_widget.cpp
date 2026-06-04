/**
 * @file constellation_widget.cpp
 */
#include "constellation_widget.hpp"
#include "../../include/snr_calculator.hpp"   // modulationName / fecRateName
#include "../../include/sync_fsm.hpp"          // syncStateName
#include "../../include/hierarchical_mod.hpp"  // hierLayerName
#include <QPainter>
#include <QPainterPath>
#include <cmath>
#include <algorithm>
#include <limits>

namespace dsca {

namespace {
const char* syncStr(SyncState s) {
    switch (s) {
        case SyncState::Searching: return "SEARCH";
        case SyncState::Acquiring: return "ACQ";
        case SyncState::Locked:    return "LOCK";
        case SyncState::Tracking:  return "TRACK";
        case SyncState::Lost:      return "LOST";
    }
    return "?";
}
QColor syncCol(SyncState s) {
    switch (s) {
        case SyncState::Locked:
        case SyncState::Tracking: return style::C_OK;
        case SyncState::Lost:     return style::C_ERROR;
        default:                  return style::C_WARNING;
    }
}
} // anonymous

ConstellationWidget::ConstellationWidget(AppState& state, QWidget* parent)
    : QWidget(parent), state_(state)
{
    setAttribute(Qt::WA_OpaquePaintEvent);
    setMinimumSize(140, 140);
    // Prime the snapshot so the very first paint (which Qt can trigger on
    // show/resize before any constellationReady signal) is safe.
    snapshotState();
}

void ConstellationWidget::snapshotState() {
    std::lock_guard<std::mutex> lock(state_.mtx);
    cd_snap_    = state_.constellation;   // 2× CONSTELLATION_MAX floats (~16 KB)
    stats_snap_ = state_.stats;
    mod_snap_   = state_.ofdm.modulation;
    fec_snap_   = state_.frame.fec_rate;
    hier_snap_  = state_.hier;
}

void ConstellationWidget::onConstellationReady() {
    // Runs on the GUI thread (queued/auto-connected from DataBridge). Take
    // a coherent copy of engine-written state under the lock, then repaint
    // from the local snapshot — paintEvent itself never locks or touches
    // shared AppState.
    snapshotState();
    update();
}

const SymbolMapper& ConstellationWidget::mapper() {
    if (!mapper_ || mapper_mod_ != mod_snap_) {
        mapper_     = std::make_unique<SymbolMapper>(mod_snap_);
        mapper_mod_ = mod_snap_;
    }
    return *mapper_;
}

QPointF ConstellationWidget::toPlot(float i, float q,
                                     const QRectF& r, float extent) const {
    float nx = (i / extent) * 0.5f + 0.5f;
    float ny = 0.5f - (q / extent) * 0.5f;
    return {r.left() + nx * r.width(),
            r.top()  + ny * r.height()};
}

void ConstellationWidget::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);

    const QRectF full = QRectF(rect());
    p.fillRect(rect(), style::BG_BASE);

    // When the widget is wider than tall (the typical layout case), put
    // the constellation as a square on the left and use the right side
    // for live RX readouts instead of leaving it blank.
    const float pad = 8.f;
    bool side_panel = (full.width() > full.height() * 1.4f) &&
                       (full.width() - full.height() > 140.f);

    QRectF plot;
    if (side_panel) {
        float side = std::min(full.height() - 2.f * pad,
                              full.width() * 0.65f);
        side = std::max(side, 80.f);
        plot = QRectF(full.left() + pad,
                      full.top()  + (full.height() - side) * 0.5f,
                      side, side);
    } else {
        float side = std::min(full.width(), full.height()) - 2.f * pad;
        side = std::max(side, 60.f);
        plot = QRectF(full.left() + (full.width()  - side) * 0.5f,
                      full.top()  + (full.height() - side) * 0.5f,
                      side, side);
    }

    p.fillRect(plot, QColor(10, 10, 14));

    // Extent from symbol mapper normalization (single cached mapper).
    const SymbolMapper& m = mapper();
    float norm = m.normalization();
    float extent = (norm > 0.f) ? 1.2f / norm : 1.5f;

    drawDecisionGrid(p, plot, extent, m);
    drawReferencePoints(p, plot, extent, m);
    drawReceivedPoints(p, plot, extent, m);

    p.setPen(QPen(style::BORDER_DIM, 0.5));
    p.drawRect(plot);

    p.setPen(QPen(QColor(50, 50, 65, 180), 0.5));
    float cx = plot.left() + plot.width()  * 0.5f;
    float cy = plot.top()  + plot.height() * 0.5f;
    p.drawLine(QPointF(plot.left(), cy), QPointF(plot.right(), cy));
    p.drawLine(QPointF(cx, plot.top()), QPointF(cx, plot.bottom()));

    // When hierarchical mod is active, overlay the HP quadrant boundaries
    // as bright green dashed lines so the user sees which lines are the
    // robust (HP) decision boundaries vs the within-quadrant LP grid.
    const auto& h = hier_snap_;
    if (h.enabled && h.effectiveHP() > 0 && h.effectiveLP() > 0) {
        // HP boundary is the axis cross at scale = α (separator between
        // HP quadrants). For α = 1 this is just the existing crosshairs;
        // for α > 1 the HP groups are pulled inward.
        QPen hp_pen(QColor(48, 209, 88, 200), 1.2, Qt::DashLine);
        p.setPen(hp_pen);
        p.drawLine(QPointF(plot.left(), cy),
                   QPointF(plot.right(), cy));
        p.drawLine(QPointF(cx, plot.top()),
                   QPointF(cx, plot.bottom()));

        // Mode label in the corner so the user knows hier is on.
        QFont f = p.font();
        f.setPixelSize(10);
        f.setBold(true);
        p.setFont(f);
        p.setPen(QColor(48, 209, 88));
        QString label = QString("HIER  HP=%1 / LP=%2  α=%3")
                            .arg(QString::fromUtf8(hierLayerName(h.effectiveHP())))
                            .arg(QString::fromUtf8(hierLayerName(h.effectiveLP())))
                            .arg(h.alpha, 0, 'f', 2);
        p.drawText(QRectF(plot.left() + 6.f, plot.top() + 4.f,
                          plot.width() - 12.f, 14.f),
                   Qt::AlignLeft | Qt::AlignTop, label);
    }

    if (side_panel) drawSidePanel(p, plot);
}

void ConstellationWidget::drawSidePanel(QPainter& p, const QRectF& plot) {
    // Right of the plot: live SNR / EVM / BER / sync / mod / fec / counts.
    QRectF info(plot.right() + 12.f, plot.top(),
                rect().right() - plot.right() - 16.f,
                plot.height());
    if (info.width() < 100.f) return;

    p.setPen(Qt::NoPen);

    QFont label_font = p.font();
    label_font.setPixelSize(11);
    QFont value_font = p.font();
    value_font.setPixelSize(18);
    value_font.setBold(true);
    QFont small_font = p.font();
    small_font.setPixelSize(10);

    const auto& stats = stats_snap_;
    auto mod   = mod_snap_;
    auto fec   = fec_snap_;

    float y = info.top() + 4.f;
    auto drawRow = [&](const QString& label, const QString& value, QColor vcol) {
        p.setFont(label_font);
        p.setPen(QColor(142, 142, 147));
        p.drawText(QRectF(info.left(), y, info.width(), 14),
                   Qt::AlignLeft | Qt::AlignVCenter, label);
        y += 14.f;
        p.setFont(value_font);
        p.setPen(vcol);
        p.drawText(QRectF(info.left(), y, info.width(), 22),
                   Qt::AlignLeft | Qt::AlignVCenter, value);
        y += 26.f;
    };

    // Sync state (color-coded)
    drawRow("SYNC", QString::fromLatin1(syncStr(stats.sync_state)),
            syncCol(stats.sync_state));

    // SNR with traffic-light coloring
    QColor snr_col = (stats.snr_db >= 18.f) ? style::C_OK
                    : (stats.snr_db >= 8.f)  ? style::C_WARNING
                                              : style::C_ERROR;
    drawRow("SNR", QString::number(stats.snr_db, 'f', 1) + " dB", snr_col);

    // EVM
    QColor evm_col = (stats.evm_percent < 5.f) ? style::C_OK
                    : (stats.evm_percent < 15.f) ? style::C_WARNING
                                                  : style::C_ERROR;
    drawRow("EVM", QString::number(stats.evm_percent, 'f', 1) + " %", evm_col);

    // BER
    QString ber_str = (stats.ber_estimate <= 0.f)
        ? "0"
        : (stats.ber_estimate >= 1.0f
            ? "—"
            : QString::number(stats.ber_estimate, 'g', 2));
    QColor ber_col = (stats.ber_estimate < 1e-4f) ? style::C_OK
                    : (stats.ber_estimate < 1e-2f) ? style::C_WARNING
                                                    : style::C_ERROR;
    drawRow("BER", ber_str, ber_col);

    // ModCod
    QString modcod = QString("%1 / %2")
        .arg(modulationName(mod)).arg(fecRateName(fec));
    drawRow("MODCOD", modcod, style::TEXT_PRIMARY);

    // Frame counters
    p.setFont(small_font);
    p.setPen(QColor(142, 142, 147));
    QString counts = QString("RX %1   OK %2   BAD %3")
        .arg(stats.frames_rx).arg(stats.frames_ok).arg(stats.frames_bad);
    p.drawText(QRectF(info.left(), y, info.width(), 14),
               Qt::AlignLeft | Qt::AlignVCenter, counts);
}

void ConstellationWidget::drawDecisionGrid(QPainter& p,
                                            const QRectF& plot, float extent,
                                            const SymbolMapper& mapper) {
    Modulation mod = mod_snap_;
    if (mod == Modulation::BPSK) return;

    size_t order = static_cast<size_t>(1) << bitsPerSymbol(mod);
    size_t grid  = static_cast<size_t>(std::sqrt(static_cast<double>(order)));
    int half     = static_cast<int>(grid) / 2;
    float step   = 2.f * mapper.normalization();

    p.setPen(QPen(QColor(38, 38, 52, 160), 0.5));
    for (int k = -(half - 1); k <= (half - 1); ++k) {
        float pos = static_cast<float>(k) * step;
        QPointF v0 = toPlot(pos, -extent, plot, extent);
        QPointF v1 = toPlot(pos,  extent, plot, extent);
        p.drawLine(v0, v1);
        QPointF h0 = toPlot(-extent, pos, plot, extent);
        QPointF h1 = toPlot( extent, pos, plot, extent);
        p.drawLine(h0, h1);
    }
}

void ConstellationWidget::drawReferencePoints(QPainter& p,
                                               const QRectF& plot, float extent,
                                               const SymbolMapper& mapper) {
    const auto& ref = mapper.constellation();

    p.setPen(QPen(QColor(80, 80, 100, 160), 0.5));
    p.setBrush(Qt::NoBrush);
    for (const auto& s : ref) {
        QPointF pt = toPlot(s.real(), s.imag(), plot, extent);
        p.drawEllipse(pt, 3.5, 3.5);
    }
}

void ConstellationWidget::drawReceivedPoints(QPainter& p,
                                              const QRectF& plot, float extent,
                                              const SymbolMapper& ref_mapper) {
    const ConstellationData& cd = cd_snap_;
    size_t n = cd.count();
    if (n == 0) return;

    // Color by EVM: good = signal blue, bad = warm orange
    float evm = stats_snap_.evm_percent;
    QColor dot_color;
    if (evm < 5.f)       dot_color = style::C_SIGNAL;
    else if (evm < 15.f) dot_color = style::C_WARNING;
    else                 dot_color = style::C_ERROR;
    dot_color.setAlpha(140);

    p.setPen(Qt::NoPen);
    p.setBrush(dot_color);

    // Fade older points — draw newest CONSTELLATION_MAX/4 brighter
    size_t bright_start = (n > CONSTELLATION_MAX / 4)
                          ? n - CONSTELLATION_MAX / 4 : 0;

    // EVM detail mode: classify each received point by its distance to the
    // nearest reference constellation point. Inliers (< 15% spacing) → faint
    // gray; outliers ≥ 15% spacing → red and slightly bigger. Quickly shows
    // *which* symbols are misbehaving. Reuses the cached mapper (no rebuild).
    const auto& ref_pts = ref_mapper.constellation();
    float ref_norm = ref_mapper.normalization();
    float spacing  = (ref_norm > 0.f) ? 2.f * ref_norm : 1.f;
    float evm_thresh = 0.15f * spacing;
    float evm_thresh_sq = evm_thresh * evm_thresh;   // compare in squared space (#48)

    // EVM-detail does a nearest-reference search per received point: O(n*|ref|).
    // For high-order constellations (|ref| = 256/1024/4096) that is up to ~2M
    // float-distance ops PER paint at 10 Hz, which makes the GUI sluggish.
    // Gate the per-point classification to <=QAM64 (|ref|<=64); larger orders
    // fall through to the plain (cheap) scatter draw.
    const bool evm_detail_active =
        evm_detail_ && !ref_pts.empty() && ref_pts.size() <= 64;

    for (size_t i = 0; i < n; ++i) {
        float ix = cd.i_vals[i];
        float qv = cd.q_vals[i];
        QPointF pt = toPlot(ix, qv, plot, extent);
        if (!plot.contains(pt)) continue;

        if (evm_detail_active) {
            // Squared distance to nearest reference point — the result is only
            // thresholded (not displayed), so the per-point sqrt of hypot is
            // wasted; compare against the squared threshold instead. (#48)
            float best = std::numeric_limits<float>::max();
            for (const auto& s : ref_pts) {
                float dr = ix - s.real();
                float di = qv - s.imag();
                float d2 = dr * dr + di * di;
                if (d2 < best) best = d2;
            }
            bool outlier = (best > evm_thresh_sq);
            QColor c = outlier ? style::C_ERROR : QColor(120, 120, 130, 130);
            c.setAlpha(outlier ? 220 : 110);
            p.setBrush(c);
            p.drawEllipse(pt, outlier ? 2.2 : 1.4, outlier ? 2.2 : 1.4);
            continue;
        }

        if (i >= bright_start) {
            QColor c = dot_color;
            c.setAlpha(200);
            p.setBrush(c);
            p.drawEllipse(pt, 1.8, 1.8);
        } else {
            QColor c = dot_color;
            c.setAlpha(70);
            p.setBrush(c);
            p.drawEllipse(pt, 1.4, 1.4);
        }
    }
}

} // namespace dsca
