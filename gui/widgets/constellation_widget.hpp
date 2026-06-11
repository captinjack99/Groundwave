/**
 * @file constellation_widget.hpp
 */
#pragma once
#include "../../include/app_state.hpp"
#include "../../include/symbol_mapper.hpp"
#include "../../include/hierarchical_mod.hpp"
#include "../style.hpp"
#include <QWidget>
#include <memory>

namespace gw {

class ConstellationWidget : public QWidget {
    Q_OBJECT
public:
    explicit ConstellationWidget(AppState& state, QWidget* parent = nullptr);
    void onConstellationReady();
    QSize sizeHint() const override { return {260, 260}; }
    QSize minimumSizeHint() const override { return {140, 140}; }

    /** Toggle EVM-detail mode: highlights points whose error vector exceeds
     *  the configured threshold (default 15 % of the symbol distance). */
    void setEvmDetailMode(bool on) { evm_detail_ = on; update(); }
    bool evmDetailMode() const { return evm_detail_; }

protected:
    void paintEvent(QPaintEvent*) override;
    void mousePressEvent(QMouseEvent*) override;

private:
    void drawDecisionGrid(QPainter& p, const QRectF& plot, float extent,
                          const SymbolMapper& m);
    void drawReferencePoints(QPainter& p, const QRectF& plot, float extent,
                             const SymbolMapper& m);
    void drawReceivedPoints(QPainter& p, const QRectF& plot, float extent,
                            const SymbolMapper& m);
    void drawSidePanel(QPainter& p, const QRectF& plot);

    QPointF toPlot(float i, float q, const QRectF& r, float extent) const;

    /** Copy engine-written AppState fields into the local snapshot under
     *  state_.mtx. Called on the GUI thread (from onConstellationReady and
     *  the ctor) so paintEvent never touches shared AppState concurrently
     *  with the engine writer thread. */
    void snapshotState();

    /** Cached symbol mapper, rebuilt only when the modulation changes —
     *  paintEvent used to construct 3-4 mappers per repaint (full
     *  constellation + bit-pattern + PWL tables, up to 4096 points). */
    const SymbolMapper& mapper();

    AppState& state_;
    bool      evm_detail_ = false;

    // ---- Snapshot of engine-written state (GUI-thread-confined) ----
    // Taken under state_.mtx in snapshotState(); read lock-free by
    // paintEvent and its helpers. This is the fix for the critical
    // cross-thread data race where paintEvent read state_.constellation /
    // state_.stats while the engine mutated them under the lock.
    ConstellationData  cd_snap_;
    ModemStats         stats_snap_;
    Modulation         mod_snap_  = Modulation::QPSK;
    FECRate            fec_snap_  = FECRate::Rate_1_2;
    HierarchicalConfig hier_snap_;

    std::unique_ptr<SymbolMapper> mapper_;
    Modulation                    mapper_mod_ = Modulation::QPSK;
};

} // namespace gw
