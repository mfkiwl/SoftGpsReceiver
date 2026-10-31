#pragma once

#include <QColor>
#include <QImage>
#include <QPointF>
#include <QString>
#include <QVector>
#include <QWidget>

#include <functional>

namespace gui {

// Lightweight, fast 2-D plotting widget.
//  - line, point, step, bar and stick series; min/max decimation so that
//    millions of points draw quickly
//  - heat-map layer with colour bar (max-pooled when zoomed out)
//  - mouse: wheel = zoom (Ctrl: x only, Shift: y only), left drag = box zoom,
//    right drag = pan, right click = menu, double click = reset view
//  - hover read-out of the cursor position and nearest data point
class PlotWidget : public QWidget
{
    Q_OBJECT
    // Editable in Qt Designer as dynamic properties of the promoted widget
    Q_PROPERTY(QString title READ title WRITE setTitle)
    Q_PROPERTY(QString xLabel READ xLabel WRITE setXLabel)
    Q_PROPERTY(QString yLabel READ yLabel WRITE setYLabel)
    Q_PROPERTY(bool legendVisible READ legendVisible WRITE setLegendVisible)
    Q_PROPERTY(bool aspectLocked READ aspectLocked WRITE setAspectLocked)
public:
    enum Style { Lines, Points, LinesPoints, Steps, Bars, Sticks };

    struct Series
    {
        QString name;
        QVector<double> x, y;
        QColor color;
        Style style = Lines;
        double width = 1.2;
        double pointSize = 3.0;
        double barWidth = 0;     // data units, 0 = automatic
        bool legend = true;
        bool sorted = true;      // x increasing (enables decimation / fast lookup)
    };

    explicit PlotWidget(QWidget* parent = nullptr);

    void clear();
    bool isEmpty() const;      // nothing to draw
    int addSeries(const QString& name, const QVector<double>& x, const QVector<double>& y,
                  Style style = Lines, const QColor& color = QColor(), double width = 1.2);
    Series& series(int i) { return series_[i]; }
    int seriesCount() const { return int(series_.size()); }

    void addVLine(double x, const QColor& c, const QString& label = QString(), Qt::PenStyle s = Qt::DashLine);
    void addHLine(double y, const QColor& c, const QString& label = QString(), Qt::PenStyle s = Qt::DashLine);
    void addCircle(double cx, double cy, double r, const QColor& c, const QString& label = QString());
    void addText(double x, double y, const QString& text, const QColor& c = QColor());
    void addSpan(double x0, double x1, const QColor& c);

    void setHeatmap(const QVector<float>& z, int nx, int ny, double x0, double x1, double y0, double y1,
                    const QString& zLabel = QString(), bool logScale = false);

    QString title() const { return title_; }
    QString xLabel() const { return xLabel_; }
    QString yLabel() const { return yLabel_; }
    bool legendVisible() const { return legend_; }
    bool aspectLocked() const { return aspect_; }
    void setTitle(const QString& t) { title_ = t; update(); }
    void setXLabel(const QString& t) { xLabel_ = t; update(); }
    void setYLabel(const QString& t) { yLabel_ = t; update(); }
    void setLabels(const QString& title, const QString& x, const QString& y) { title_ = title; xLabel_ = x; yLabel_ = y; update(); }
    void setAspectLocked(bool on) { aspect_ = on; autoScale(); }
    void setLegendVisible(bool on) { legend_ = on; update(); }
    void setFixedXRange(double a, double b) { fixX_ = true; fx0_ = a; fx1_ = b; }
    void setFixedYRange(double a, double b) { fixY_ = true; fy0_ = a; fy1_ = b; }
    void clearFixedRanges() { fixX_ = fixY_ = false; }
    void setYIncludesZero(bool on) { yZero_ = on; }
    // Text of the x ticks and hover read-out, for an axis whose numbers are not what they mean
    void setXTickText(std::function<QString(double)> text) { xTickText_ = std::move(text); update(); }
    void setView(double x0, double x1, double y0, double y1);
    void autoScale();
    // While on, a view the user zoomed or panned to survives the plot being refilled (live updates).
    static void setKeepUserView(bool on);
    double viewX0() const { return vx0_; }
    double viewX1() const { return vx1_; }

    QSize sizeHint() const override { return QSize(420, 260); }
    QSize minimumSizeHint() const override { return QSize(200, 140); }

public slots:
    void setXView(double x0, double x1);   // used to link time axes (does not re-emit)
    void savePng();
    void copyImage();
    void exportCsv();

signals:
    void clicked(double x, double y);
    void xViewChanged(double x0, double x1);

protected:
    void paintEvent(QPaintEvent*) override;
    void mousePressEvent(QMouseEvent* e) override;
    void mouseMoveEvent(QMouseEvent* e) override;
    void mouseReleaseEvent(QMouseEvent* e) override;
    void mouseDoubleClickEvent(QMouseEvent* e) override;
    void wheelEvent(QWheelEvent* e) override;
    void leaveEvent(QEvent* e) override;

private:
    struct Line { bool vertical; double v; QColor c; QString label; Qt::PenStyle style; };
    struct Circle { double cx, cy, r; QColor c; QString label; };
    struct Text { double x, y; QString text; QColor c; };
    struct Span { double x0, x1; QColor c; };

    QRect plotRect() const;
    QPointF toPx(double x, double y) const;
    QPointF toData(const QPointF& p) const;
    void dataBounds(double& x0, double& x1, double& y0, double& y1) const;
    void applyAspect();
    void drawSeries(QPainter& p, const Series& s, const QRect& r) const;
    void drawHeatmap(QPainter& p, const QRect& r) const;
    void drawAxes(QPainter& p, const QRect& r) const;
    void drawLegend(QPainter& p, const QRect& r) const;
    void showMenu(const QPoint& global);
    QString hoverText(const QPointF& pos) const;
    static QVector<double> ticks(double a, double b, int approx, double* step);
    static QString tickLabel(double v, double step);
    QColor colormap(double t) const;

    std::vector<Series> series_;
    std::vector<Line> lines_;
    std::vector<Circle> circles_;
    std::vector<Text> texts_;
    std::vector<Span> spans_;

    // heat map
    QVector<float> hz_;
    int hnx_ = 0, hny_ = 0;
    double hx0_ = 0, hx1_ = 0, hy0_ = 0, hy1_ = 0;
    float hmin_ = 0, hmax_ = 1;
    bool hlog_ = false;
    QString hLabel_;

    QString title_, xLabel_, yLabel_;
    std::function<QString(double)> xTickText_;
    double vx0_ = 0, vx1_ = 1, vy0_ = 0, vy1_ = 1;
    bool fixX_ = false, fixY_ = false, yZero_ = false;
    double fx0_ = 0, fx1_ = 1, fy0_ = 0, fy1_ = 1;
    bool aspect_ = false, legend_ = true, grid_ = true;
    bool hasView_ = false;
    bool userView_ = false;

    // interaction
    enum Drag { None, Zoom, Pan } drag_ = None;
    QPoint pressPos_, lastPos_;
    bool moved_ = false;
    QPoint hover_;
    bool hovering_ = false;
};

} // namespace gui
