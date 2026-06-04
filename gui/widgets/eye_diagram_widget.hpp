/**
 * @file eye_diagram_widget.hpp
 * @brief Time-domain eye diagram of the equalized signal.
 *
 * Overlays multiple sub-symbol traces (re-aligned to symbol boundaries) so
 * the user can spot ISI, timing-recovery jitter, and clipping. Standard
 * RF/DSP diagnostic tool — a "wide-open" eye = good, closed = problems.
 */
#pragma once

#include "../style.hpp"
#include <QWidget>
#include <vector>
#include <deque>
#include <mutex>

namespace dsca {

class EyeDiagramWidget : public QWidget {
    Q_OBJECT
public:
    explicit EyeDiagramWidget(QWidget* parent = nullptr);

    /** Push real-valued time-domain samples. */
    void pushSamples(const float* samples, size_t n, int samples_per_symbol);

    QSize sizeHint() const override { return {280, 120}; }
    QSize minimumSizeHint() const override { return {180, 80}; }

protected:
    void paintEvent(QPaintEvent*) override;

private:
    static constexpr size_t MAX_TRACES    = 24;  ///< overlaid sub-symbol traces
    static constexpr int    EYE_SYMBOLS   = 2;   ///< two symbols per eye span
    static constexpr int    MAX_TRACE_PTS = 128; ///< decimate long traces in paint

    std::deque<std::vector<float>> traces_;
    int  samples_per_symbol_ = 8;
    mutable std::mutex mtx_;
};

} // namespace dsca
