/**
 * @file scope_widget.cpp
 */
#include "scope_widget.hpp"
#include <QPainter>
#include <QPaintEvent>
#include <algorithm>

namespace dsca {

ScopeWidget::ScopeWidget(QWidget* parent) : QWidget(parent), buf_(BUF_LEN, 0.f) {
    setMinimumHeight(120);
    setMinimumWidth(240);
}

void ScopeWidget::pushSamples(const float* samples, size_t n) {
    std::lock_guard<std::mutex> lk(mtx_);
    for (size_t i = 0; i < n; ++i) {
        buf_[write_pos_] = samples[i];
        write_pos_ = (write_pos_ + 1) % BUF_LEN;
    }
    update();
}

void ScopeWidget::pushComplex(const float* re, const float* /*im*/, size_t n) {
    pushSamples(re, n);
}

void ScopeWidget::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);

    // Background
    p.fillRect(rect(), QColor(20, 22, 28));

    // Grid
    p.setPen(QPen(QColor(40, 44, 52), 1, Qt::DotLine));
    for (int gx = 0; gx <= 8; ++gx) {
        int x = rect().left() + (rect().width() * gx) / 8;
        p.drawLine(x, rect().top(), x, rect().bottom());
    }
    for (int gy = 0; gy <= 4; ++gy) {
        int y = rect().top() + (rect().height() * gy) / 4;
        p.drawLine(rect().left(), y, rect().right(), y);
    }

    // Snapshot buffer under lock
    std::vector<float> snap;
    size_t wp;
    {
        std::lock_guard<std::mutex> lk(mtx_);
        snap = buf_;
        wp   = write_pos_;
    }

    // Determine starting index — last `time_base_` samples ending at wp
    size_t base = (wp + BUF_LEN - static_cast<size_t>(time_base_)) % BUF_LEN;

    // Trigger search: first rising edge with hysteresis. A bare zero-crossing
    // (snap[i] <= 0 && snap[i+1] > 0) chatters on noise near zero, jittering
    // the trace frame-to-frame. Require the signal to first fall below -H, then
    // trigger when it rises above +H — a stable Schmitt-trigger edge. (#48)
    if (triggered_) {
        constexpr float H = 0.05f;   // hysteresis band (signal clamped to ±1.5)
        bool armed = false;
        for (size_t k = 0; k + 1 < static_cast<size_t>(time_base_); ++k) {
            size_t i  = (base + k) % BUF_LEN;
            float v = snap[i];
            if (v < -H) {
                armed = true;
            } else if (armed && v > H) {
                base = i;
                break;
            }
        }
    }

    // Plot
    p.setPen(QPen(QColor(64, 209, 88), 1.2));
    int W = rect().width();
    int H = rect().height();
    int H2 = H / 2;

    QPointF prev(0, H2);
    bool first = true;
    for (int x = 0; x < W; ++x) {
        size_t k = static_cast<size_t>(static_cast<int64_t>(x) *
                                       static_cast<int64_t>(time_base_) / W);
        size_t i = (base + k) % BUF_LEN;
        float v = std::clamp(snap[i], -1.5f, 1.5f);
        float y = static_cast<float>(H2) - v * static_cast<float>(H2 - 4);
        QPointF cur(x, y);
        if (!first) p.drawLine(prev, cur);
        prev = cur;
        first = false;
    }

    // Center line label
    p.setPen(QColor(120, 130, 140));
    p.drawText(rect().adjusted(4, 4, -4, -4), Qt::AlignTop | Qt::AlignLeft,
               QString("Scope %1× %2").arg(time_base_)
                   .arg(triggered_ ? "TRIG" : "FREE"));
}

} // namespace dsca
