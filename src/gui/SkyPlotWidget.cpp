#include "SkyPlotWidget.h"
#include "GuiUtil.h"

#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QToolTip>
#include <cmath>

namespace gui {

SkyPlotWidget::SkyPlotWidget(QWidget* parent) : QWidget(parent)
{
    setMouseTracking(true);
    setMinimumSize(240, 240);
}

void SkyPlotWidget::setData(const QVector<Sat>& sats, const QVector<Track>& tracks)
{
    sats_ = sats;
    tracks_ = tracks;
    update();
}

QPointF SkyPlotWidget::project(double az, double el) const
{
    const double r = radius_ * (90.0 - el) / 90.0;
    const double a = az * 3.14159265358979323846 / 180.0;
    return QPointF(center_.x() + r * std::sin(a), center_.y() - r * std::cos(a));
}

int SkyPlotWidget::satAt(const QPoint& p) const
{
    for (int i = int(sats_.size()) - 1; i >= 0; --i) {
        const QPointF q = project(sats_[i].az, sats_[i].el);
        if (std::hypot(q.x() - p.x(), q.y() - p.y()) < 12) return i;
    }
    return -1;
}

void SkyPlotWidget::paintEvent(QPaintEvent*)
{
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    p.fillRect(rect(), palette().color(QPalette::Window));
    const QFontMetrics fm(font());
    const int top = title_.isEmpty() ? 0 : fm.height() + 4;
    center_ = QPointF(width() / 2.0, top + (height() - top) / 2.0);
    radius_ = std::min(width(), height() - top) / 2.0 - fm.height() - 8;
    if (radius_ < 20) return;
    const QColor text = palette().color(QPalette::Text);
    QColor grid = palette().color(QPalette::Mid);

    p.setPen(QPen(grid, 1));
    p.setBrush(palette().color(QPalette::Base));
    p.drawEllipse(center_, radius_, radius_);
    if (mask_ > 0) {
        QColor m(200, 80, 80, 40);
        p.setBrush(m);
        p.setPen(Qt::NoPen);
        QPainterPath path;
        path.addEllipse(center_, radius_, radius_);
        const double rm = radius_ * (90.0 - mask_) / 90.0;
        path.addEllipse(center_, rm, rm);
        p.drawPath(path);
    }
    p.setBrush(Qt::NoBrush);
    for (int el = 0; el < 90; el += 15) {
        const double r = radius_ * (90.0 - el) / 90.0;
        p.setPen(QPen(grid, 1, el == 0 ? Qt::SolidLine : Qt::DotLine));
        p.drawEllipse(center_, r, r);
        p.setPen(text);
        p.drawText(QPointF(center_.x() + 3, center_.y() - r + fm.ascent()), QStringLiteral("%1°").arg(el));
    }
    for (int az = 0; az < 360; az += 30) {
        p.setPen(QPen(grid, 1, Qt::DotLine));
        p.drawLine(center_, project(az, 0));
        const QPointF q = project(az, -8);
        QString lab = QString::number(az);
        if (az == 0) lab = QStringLiteral("N");
        else if (az == 90) lab = QStringLiteral("E");
        else if (az == 180) lab = QStringLiteral("S");
        else if (az == 270) lab = QStringLiteral("W");
        p.setPen(text);
        const int w = fm.horizontalAdvance(lab);
        p.drawText(QPointF(q.x() - w / 2.0, q.y() + fm.ascent() / 2.0), lab);
    }
    for (const Track& t : tracks_) {
        if (t.azel.size() < 2) continue;
        QPolygonF poly;
        for (const QPointF& ae : t.azel) poly << project(ae.x(), ae.y());
        p.setPen(QPen(t.color, 2));
        p.drawPolyline(poly);
    }
    int gps = 0, galileo = 0;
    for (const Sat& s : sats_) {
        if (s.el < -5) continue;
        const QPointF q = project(s.az, s.el);
        const bool gal = gnss::isGalileo(s.prn);
        if (s.filled) ++(gal ? galileo : gps);
        p.setPen(QPen(s.color.darker(150), 1.5));
        p.setBrush(s.filled ? s.color : palette().color(QPalette::Base));
        if (gal) p.drawRoundedRect(QRectF(q.x() - 12, q.y() - 10, 24, 20), 3, 3);
        else p.drawEllipse(q, 10, 10);
        p.setPen(s.filled ? Qt::white : text);
        QFont f = font();
        f.setBold(true);
        f.setPointSizeF(font().pointSizeF() * 0.85);
        p.setFont(f);
        p.drawText(QRectF(q.x() - 12, q.y() - 10, 24, 20), Qt::AlignCenter, satLabel(s.prn));
        p.setFont(font());
    }
    if (!sats_.isEmpty()) {
        const int lh = fm.height() + 4;
        const QPointF a(12, height() - 2 * lh), b(12, height() - lh);
        p.setPen(QPen(text, 1));
        p.setBrush(Qt::NoBrush);
        p.drawEllipse(a, 5, 5);
        p.drawRoundedRect(QRectF(b.x() - 6, b.y() - 5, 12, 10), 2, 2);
        p.drawText(QPointF(24, a.y() + fm.ascent() / 2.0 - 1), tr("GPS: %1").arg(gps));
        p.drawText(QPointF(24, b.y() + fm.ascent() / 2.0 - 1), tr("Galileo: %1").arg(galileo));
    }
    if (!title_.isEmpty()) {
        QFont f = font();
        f.setBold(true);
        p.setFont(f);
        p.setPen(text);
        p.drawText(QRect(0, 2, width(), fm.height() + 2), Qt::AlignCenter, title_);
    }
}

void SkyPlotWidget::mouseMoveEvent(QMouseEvent* e)
{
    const int i = satAt(e->pos());
    if (i >= 0) {
        const Sat& s = sats_[i];
        QToolTip::showText(e->globalPosition().toPoint(),
                           QStringLiteral("%1\naz %2°  el %3°\n%4").arg(satFullName(s.prn)).arg(s.az, 0, 'f', 1)
                               .arg(s.el, 0, 'f', 1).arg(s.tip), this);
    } else {
        QToolTip::hideText();
    }
}

void SkyPlotWidget::mousePressEvent(QMouseEvent* e)
{
    const int i = satAt(e->pos());
    if (i >= 0) emit satelliteClicked(sats_[i].prn);
}

} // namespace gui
