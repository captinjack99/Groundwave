/**
 * @file pls_status_widget.hpp
 * @brief Compact display of the most recently decoded PLS block + ModCod
 *        detector confidence.
 *
 * PLS = Physical Layer Signaling — the 32-bit known block that tells the
 * receiver the current modulation/FEC pair, VCM slot index, and parity.
 * Showing the raw bytes plus the ModCodDetector's running agreement count
 * is invaluable when debugging "in lock but BER high" scenarios.
 */
#pragma once

#include "../../include/types.hpp"
#include <QWidget>
#include <QString>
#include <mutex>

namespace dsca {

class PLSStatusWidget : public QWidget {
    Q_OBJECT
public:
    explicit PLSStatusWidget(QWidget* parent = nullptr);

    /** Update with last decoded PLS contents. */
    void update(Modulation mod, FECRate fec, int vcm_slot, int vcm_total,
                bool crc_ok, int confirmation_count);

    QSize sizeHint() const override { return {200, 90}; }
    QSize minimumSizeHint() const override { return {160, 80}; }

protected:
    void paintEvent(QPaintEvent*) override;

private:
    Modulation mod_   = Modulation::QPSK;
    FECRate    fec_   = FECRate::Rate_1_2;
    int        slot_  = 0;
    int        total_ = 1;
    bool       ok_    = false;
    int        conf_  = 0;
    mutable std::mutex mtx_;
};

} // namespace dsca
