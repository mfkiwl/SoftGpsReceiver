#pragma once

#include <QColor>
#include <QPointF>
#include <QString>
#include <QVector>
#include <QWidget>

namespace gui {

// Polar azimuth/elevation plot of satellite positions: circles for GPS, squares for Galileo.
class SkyPlotWidget : public QWidget
{
    Q_OBJECT
    Q_PROPERTY(QString title READ title WRITE setTitle)
public:
    struct Sat
    {
        int prn = 0;
        double az = 0, el = 0;     // degrees
        QColor color;
        bool filled = true;        // tracked/used (filled) or predicted (hollow)
        QString tip;
    };
    struct Track
    {
        int prn = 0;
        QVector<QPointF> azel;     // degrees
        QColor color;
    };

    explicit SkyPlotWidget(QWidget* parent = nullptr);
    void setData(const QVector<Sat>& sats, const QVector<Track>& tracks);
    const QVector<Sat>& satellites() const { return sats_; }
    void setElevationMask(double deg) { mask_ = deg; update(); }
    QString title() const { return title_; }
    void setTitle(const QString& t) { title_ = t; update(); }
    QSize sizeHint() const override { return QSize(380, 380); }

signals:
    void satelliteClicked(int prn);

protected:
    void paintEvent(QPaintEvent*) override;
    void mouseMoveEvent(QMouseEvent* e) override;
    void mousePressEvent(QMouseEvent* e) override;

private:
    QPointF project(double azDeg, double elDeg) const;
    int satAt(const QPoint& p) const;

    QVector<Sat> sats_;
    QVector<Track> tracks_;
    double mask_ = 0;
    QString title_;
    QPointF center_;
    double radius_ = 100;
};

} // namespace gui
