/**
 * @file channel_response_widget.hpp
 * @brief Live |H(k)| / arg(H(k)) display from the OFDM channel estimate.
 *
 * Shows the magnitude (and optionally phase) of the equalized channel
 * response across the active subcarriers. Notches and tilts here reveal
 * multipath fading, IQ imbalance, or analog filter rolloff.
 */
#pragma once

#include "../../include/app_state.hpp"
#include "../style.hpp"
#include <QWidget>
#include <vector>
#include <mutex>

namespace gw {

class ChannelResponseWidget : public QWidget {
    Q_OBJECT
public:
    explicit ChannelResponseWidget(AppState& state, QWidget* parent = nullptr);

    /** Push a new channel-estimate snapshot (one entry per subcarrier).
     *  Engine calls this from the RX thread via signal-slot to GUI thread. */
    void pushEstimate(const std::vector<float>& mag_db);

    QSize sizeHint() const override { return {320, 100}; }
    QSize minimumSizeHint() const override { return {200, 80}; }

protected:
    void paintEvent(QPaintEvent*) override;

private:
    AppState& state_;
    std::vector<float> mag_db_;     ///< current snapshot
    mutable std::mutex mtx_;
};

} // namespace gw
