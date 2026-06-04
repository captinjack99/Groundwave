/**
 * @file data_bridge.cpp
 */
#include "data_bridge.hpp"
#include <QMutexLocker>

namespace dsca {

DataBridge::DataBridge(AppState& state, QObject* parent)
    : QObject(parent)
    , state_(state)
    , analyzer_(SpectrumConfig{
          SPECTRUM_BINS * 2,
          static_cast<float>(state.ofdm.sample_rate),
          0.80f,
          0.f,
          false
      })
    , timer_(new QTimer(this))
{
    connect(timer_, &QTimer::timeout, this, &DataBridge::onTimer);
}

void DataBridge::start(int interval_ms) {
    timer_->start(interval_ms);
}

void DataBridge::stop() {
    timer_->stop();
}

void DataBridge::pushSpectrumSamples(const float* samples, size_t count) {
    analyzer_.pushSamples(samples, count);
}

void DataBridge::pushSpectrumComplex(const ComplexSample* samples, size_t count) {
    analyzer_.pushComplex(samples, count);
}

void DataBridge::pushScopeSamples(const float* samples, size_t count) {
    std::lock_guard<std::mutex> lk(scope_mtx_);
    if (scope_pending_.size() > 8192) scope_pending_.clear();  // bound stash
    scope_pending_.insert(scope_pending_.end(), samples, samples + count);
}

void DataBridge::pushChannelResponse(const std::vector<float>& mag_db) {
    // Stage newest-only; the 30 Hz onTimer drains and emits on the GUI thread.
    // Coalescing here (rather than emitting inline from the audio thread) keeps
    // the cross-thread copy + repaint rate bounded so an active TX engine can't
    // flood the GUI event loop into a freeze.
    std::lock_guard<std::mutex> lk(diag_mtx_);
    channel_pending_ = mag_db;
    channel_has_pending_ = true;
}

void DataBridge::pushEyeSamples(const std::vector<float>& samples, int sps) {
    // Stage newest-only (drop any undrained prior frame); drained at 30 Hz.
    std::lock_guard<std::mutex> lk(diag_mtx_);
    eye_pending_ = samples;
    eye_pending_sps_ = sps;
    eye_has_pending_ = true;
}

void DataBridge::pushPLS(int modulation, int fec_rate,
                          int vcm_slot, int vcm_total,
                          bool crc_ok, int confirmation_count) {
    emit plsUpdated(modulation, fec_rate, vcm_slot, vcm_total,
                    crc_ok, confirmation_count);
}

void DataBridge::onTimer() {
    ++tick_;

    // --- Spectrum ---
    // analyzer_.update() every tick keeps the FFT/sample ring drained, but the
    // waterfall repaint (a full QImage scroll + scaled drawImage) is the single
    // heaviest GUI-thread paint, so present it at 15 Hz. The cheap stats/meter
    // emits below stay at the full 30 Hz so text readouts remain snappy. This
    // (with the eye/channel coalescing) keeps the cumulative real-window paint
    // load under budget so a TX-active engine doesn't starve the event loop.
    const bool spec_ready = analyzer_.update(state_.spectrum);
    if (spec_ready && (tick_ % 2 == 0)) {
        emit spectrumReady();
    }

    // Take a coherent snapshot of all state under the lock, then emit
    // signals OUTSIDE the lock so slot handlers can't deadlock by
    // re-entering AppState. (Audit found the prior ordering reacquired
    // mtx via getStats() between unlock and emit, which left a small
    // window where stats could differ from what alarms saw.)
    ModemStats   stats_snap;
    AlarmStatus  alarms_snap;
    float        tx_rms{}, rx_rms{}, tx_peak{}, rx_peak{};
    bool         tx_clip{}, rx_clip{};
    float        agc_snap{};
    {
        std::lock_guard<std::mutex> lock(state_.mtx);
        // NOTE: do NOT write tx_meter/rx_meter here. The engine populates
        // them with the real TX/RX audio RMS under this same lock
        // (audio_engine.cpp processTX/processRX). The previous code
        // overwrote those true levels with SNR/AGC-derived proxies
        // (snr_db-30 / agc_gain_db-20) at a different cadence, producing a
        // two-writer conflict that corrupted both the level displays and
        // the level/clip alarms read by updateAlarms(). Just read them.
        state_.updateAlarms();

        stats_snap   = state_.stats;
        alarms_snap  = state_.alarm_status;
        tx_rms = state_.tx_meter.rms_db;
        rx_rms = state_.rx_meter.rms_db;
        tx_peak = state_.tx_meter.peak_db;
        rx_peak = state_.rx_meter.peak_db;
        tx_clip = state_.tx_meter.clipping;
        rx_clip = state_.rx_meter.clipping;
        agc_snap = state_.agc_gain_db;
    }

    emit statsUpdated(stats_snap);
    emit alarmsUpdated(alarms_snap);
    emit metersUpdated(tx_rms, rx_rms, tx_peak, rx_peak, tx_clip, rx_clip);
    emit agcUpdated(agc_snap);

    // Drain pending scope samples (time-domain trace) at 15 Hz — the trace
    // repaint is moderately heavy and 15 fps is visually smooth.
    if (tick_ % 2 == 0) {
        std::vector<float> scope_drain;
        {
            std::lock_guard<std::mutex> lk(scope_mtx_);
            scope_drain.swap(scope_pending_);
        }
        if (!scope_drain.empty()) emit scopeSamples(scope_drain);
    }

    // Drain newest-only eye + channel-response frames staged by the engine.
    // Eye repaints at 15 Hz (heavy trace overlay); channel response is a
    // low-rate per-preamble event so it drains whenever present. Coalescing +
    // throttling here caps the cross-thread copy + repaint rate so a TX-active
    // engine can't outrun the GUI event loop (fixes the TX hang).
    std::vector<float> eye_drain; int eye_sps = 0; bool eye_go = false;
    std::vector<float> chan_drain;                  bool chan_go = false;
    {
        std::lock_guard<std::mutex> lk(diag_mtx_);
        if (eye_has_pending_ && (tick_ % 2 == 0)) {
            eye_drain.swap(eye_pending_); eye_sps = eye_pending_sps_;
            eye_has_pending_ = false; eye_go = true;
        }
        if (channel_has_pending_) {
            chan_drain.swap(channel_pending_);
            channel_has_pending_ = false; chan_go = true;
        }
    }
    if (eye_go)  emit eyeSamplesReady(eye_drain, eye_sps);
    if (chan_go) emit channelResponseReady(chan_drain);

    // Stream RMS levels (snapshot under main lock above)
    std::array<float, 8> levels{};
    {
        std::lock_guard<std::mutex> lock(state_.mtx);
        for (size_t i = 0; i < 8 && i < MAX_STREAMS; ++i)
            levels[i] = state_.stream_rms_db[i];
    }
    emit streamLevels(levels);

    // --- Constellation (every 3rd tick ~10fps — slower is fine for scatter) ---
    if (tick_ % 3 == 0) {
        emit constellationReady();
    }
}

} // namespace dsca
