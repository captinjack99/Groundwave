/**
 * @file pls_status_widget.cpp
 */
#include "pls_status_widget.hpp"
#include "../style.hpp"
#include "../../include/snr_calculator.hpp"
#include <QPainter>

namespace gw {

PLSStatusWidget::PLSStatusWidget(QWidget* parent) : QWidget(parent) {
    setMinimumSize(160, 80);
}

void PLSStatusWidget::update(Modulation mod, FECRate fec,
                              int vcm_slot, int vcm_total,
                              bool crc_ok, int confirmation_count) {
    {
        std::lock_guard<std::mutex> lk(mtx_);
        mod_  = mod;
        fec_  = fec;
        slot_ = vcm_slot;
        total_ = vcm_total;
        ok_   = crc_ok;
        conf_ = confirmation_count;
    }
    QWidget::update();
}

void PLSStatusWidget::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    p.fillRect(rect(), style::BG_BASE);

    Modulation mod;
    FECRate    fec;
    int slot, total, conf;
    bool ok;
    {
        std::lock_guard<std::mutex> lk(mtx_);
        mod   = mod_;
        fec   = fec_;
        slot  = slot_;
        total = total_;
        ok    = ok_;
        conf  = conf_;
    }

    QFont label;
    label.setPixelSize(10);
    QFont value;
    value.setPixelSize(13);
    value.setBold(true);

    p.setFont(label);
    p.setPen(QColor(142, 142, 147));
    p.drawText(rect().adjusted(8, 6, -8, -6),
               Qt::AlignLeft | Qt::AlignTop, "PLS / MODCOD DETECT");

    int y = 26;
    auto row = [&](const QString& k, const QString& v, QColor vcol) {
        p.setFont(label);
        p.setPen(QColor(142, 142, 147));
        p.drawText(QRectF(8, y, 60, 14),
                   Qt::AlignLeft | Qt::AlignVCenter, k);
        p.setFont(value);
        p.setPen(vcol);
        p.drawText(QRectF(70, y - 1, rect().width() - 78, 16),
                   Qt::AlignLeft | Qt::AlignVCenter, v);
        y += 16;
    };

    row("ModCod", QString("%1 / %2")
            .arg(modulationName(mod)).arg(fecRateName(fec)),
        style::TEXT_PRIMARY);
    row("VCM",    QString("slot %1 / %2").arg(slot).arg(total),
        style::TEXT_PRIMARY);
    row("CRC",    ok ? "OK" : "FAIL",
        ok ? style::C_OK : style::C_ERROR);
    row("Conf.",  QString::number(conf),
        conf >= 2 ? style::C_OK : style::C_WARNING);
}

} // namespace gw
