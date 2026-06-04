/**
 * @file scope_widget.hpp
 * @brief Time-domain audio scope (oscilloscope-style waveform).
 *
 * Displays a rolling buffer of recent audio samples. The display has a
 * triggered mode (rising-edge zero crossing) for stable display of
 * periodic signals, and a free-run mode for arbitrary content.
 */
#pragma once

#include <QWidget>
#include <vector>
#include <mutex>

namespace dsca {

class ScopeWidget : public QWidget {
    Q_OBJECT
public:
    explicit ScopeWidget(QWidget* parent = nullptr);

    /** Push new audio samples into the scope's display buffer. */
    void pushSamples(const float* samples, size_t n);

    /** Push complex samples (uses Re part). */
    void pushComplex(const float* re, const float* im, size_t n);

    void setTriggered(bool on)   { triggered_ = on; update(); }
    void setTimeBase(int samples) { time_base_ = samples; update(); }

protected:
    void paintEvent(QPaintEvent*) override;

private:
    static constexpr size_t BUF_LEN = 4096;
    std::vector<float> buf_;
    size_t             write_pos_ = 0;
    int                time_base_ = 1024;   // visible width in samples
    bool               triggered_ = true;
    mutable std::mutex mtx_;
};

} // namespace dsca
