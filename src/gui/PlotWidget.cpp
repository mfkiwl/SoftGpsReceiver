#include "PlotWidget.h"

#include <QApplication>
#include <QClipboard>
#include <QFile>
#include <QFileDialog>
#include <QMenu>
#include <QMouseEvent>
#include <QPainter>
#include <QTextStream>
#include <QWheelEvent>

#include <algorithm>
#include <climits>
#include <cmath>
#include <limits>

namespace gui {

namespace {

QColor defaultColor(int i)
{
    static const QColor c[] = {QColor(31, 119, 180), QColor(255, 127, 14), QColor(44, 160, 44), QColor(214, 39, 40),
                               QColor(148, 103, 189), QColor(140, 86, 75), QColor(227, 119, 194), QColor(127, 127, 127),
                               QColor(188, 189, 34), QColor(23, 190, 207)};
    return c[i % 10];
}

inline double clampPx(double v) { return std::clamp(v, -1.0e6, 1.0e6); }

QString num(double v) { return QString::number(v, 'g', 9); }

bool g_keepUserView = false;

} // namespace

void PlotWidget::setKeepUserView(bool on)
{
    g_keepUserView = on;
}

PlotWidget::PlotWidget(QWidget* parent) : QWidget(parent)
{
    setMouseTracking(true);
    setAttribute(Qt::WA_OpaquePaintEvent);
    setFocusPolicy(Qt::ClickFocus);
}

void PlotWidget::clear()
{
    series_.clear();
    lines_.clear();
    circles_.clear();
    texts_.clear();
    spans_.clear();
    hz_ = QVector<float>();    // a clear() keeps the memory
    hnx_ = hny_ = 0;
    if (g_keepUserView && userView_) {
        update();
        return;
    }
    userView_ = false;
    hasView_ = false;
    update();
}

bool PlotWidget::isEmpty() const
{
    return hz_.isEmpty() && lines_.empty() && circles_.empty() && texts_.empty() && spans_.empty()
           && std::all_of(series_.begin(), series_.end(), [](const Series& s) { return s.x.isEmpty(); });
}

int PlotWidget::addSeries(const QString& name, const QVector<double>& x, const QVector<double>& y, Style style,
                          const QColor& color, double width)
{
    Series s;
    s.name = name;
    const int n = std::min(x.size(), y.size());
    s.x = x.mid(0, n);
    s.y = y.mid(0, n);
    s.style = style;
    s.color = color.isValid() ? color : defaultColor(int(series_.size()));
    s.width = width;
    s.sorted = std::is_sorted(s.x.begin(), s.x.end());
    series_.push_back(s);
    autoScale();
    return int(series_.size()) - 1;
}

void PlotWidget::addVLine(double x, const QColor& c, const QString& label, Qt::PenStyle s)
{
    lines_.push_back({true, x, c, label, s});
    update();
}

void PlotWidget::addHLine(double y, const QColor& c, const QString& label, Qt::PenStyle s)
{
    lines_.push_back({false, y, c, label, s});
    update();
}

void PlotWidget::addCircle(double cx, double cy, double r, const QColor& c, const QString& label)
{
    circles_.push_back({cx, cy, r, c, label});
    autoScale();
}

void PlotWidget::addText(double x, double y, const QString& text, const QColor& c)
{
    texts_.push_back({x, y, text, c.isValid() ? c : palette().color(QPalette::Text)});
    update();
}

void PlotWidget::addSpan(double x0, double x1, const QColor& c)
{
    spans_.push_back({x0, x1, c});
    update();
}

void PlotWidget::setHeatmap(const QVector<float>& z, int nx, int ny, double x0, double x1, double y0, double y1,
                            const QString& zLabel, bool logScale)
{
    hz_ = z;
    hnx_ = nx;
    hny_ = ny;
    hx0_ = x0; hx1_ = x1; hy0_ = y0; hy1_ = y1;
    hLabel_ = zLabel;
    hlog_ = logScale;
    hmin_ = std::numeric_limits<float>::max();
    hmax_ = -std::numeric_limits<float>::max();
    for (float v : hz_)
        if (std::isfinite(v)) { hmin_ = std::min(hmin_, v); hmax_ = std::max(hmax_, v); }
    if (hmax_ <= hmin_) hmax_ = hmin_ + 1;
    autoScale();
}

void PlotWidget::dataBounds(double& x0, double& x1, double& y0, double& y1) const
{
    x0 = y0 = std::numeric_limits<double>::max();
    x1 = y1 = -std::numeric_limits<double>::max();
    for (const Series& s : series_) {
        for (int i = 0; i < s.x.size(); ++i) {
            const double x = s.x[i], y = s.y[i];
            if (!std::isfinite(x) || !std::isfinite(y)) continue;
            x0 = std::min(x0, x); x1 = std::max(x1, x);
            y0 = std::min(y0, y); y1 = std::max(y1, y);
        }
        if (s.style == Bars || s.style == Sticks) { y0 = std::min(y0, 0.0); y1 = std::max(y1, 0.0); }
        if (s.style == Bars && !s.x.isEmpty()) { x0 -= 0.6; x1 += 0.6; }
    }
    for (const Circle& c : circles_) {
        x0 = std::min(x0, c.cx - c.r); x1 = std::max(x1, c.cx + c.r);
        y0 = std::min(y0, c.cy - c.r); y1 = std::max(y1, c.cy + c.r);
    }
    if (!hz_.isEmpty()) {
        x0 = std::min(x0, hx0_); x1 = std::max(x1, hx1_);
        y0 = std::min(y0, hy0_); y1 = std::max(y1, hy1_);
    }
    if (x0 > x1) { x0 = 0; x1 = 1; }
    if (y0 > y1) { y0 = 0; y1 = 1; }
}

void PlotWidget::autoScale()
{
    if (g_keepUserView && userView_) {
        update();
        return;
    }
    double x0, x1, y0, y1;
    dataBounds(x0, x1, y0, y1);
    const bool heatOnly = !hz_.isEmpty() && series_.empty();
    if (fixX_) { x0 = fx0_; x1 = fx1_; }
    else if (!heatOnly) {
        const double p = (x1 - x0) * 0.02;
        x0 -= p; x1 += p;
    }
    if (yZero_) { y0 = std::min(y0, 0.0); y1 = std::max(y1, 0.0); }
    if (fixY_) { y0 = fy0_; y1 = fy1_; }
    else if (!heatOnly) {
        const double p = (y1 - y0) * 0.06;
        y0 -= p; y1 += p;
    }
    if (x1 - x0 < 1e-12 * std::max(1.0, std::fabs(x0))) { x0 -= 0.5; x1 += 0.5; }
    if (y1 - y0 < 1e-12 * std::max(1.0, std::fabs(y0))) {
        const double d = std::max(0.5, std::fabs(y0) * 0.01);
        y0 -= d; y1 += d;
    }
    vx0_ = x0; vx1_ = x1; vy0_ = y0; vy1_ = y1;
    hasView_ = true;
    applyAspect();
    update();
}

void PlotWidget::setView(double x0, double x1, double y0, double y1)
{
    if (x1 > x0) { vx0_ = x0; vx1_ = x1; }
    if (y1 > y0) { vy0_ = y0; vy1_ = y1; }
    applyAspect();
    update();
}

void PlotWidget::setXView(double x0, double x1)
{
    if (!(x1 > x0)) return;
    userView_ = true;
    vx0_ = x0;
    vx1_ = x1;
    update();
}

void PlotWidget::applyAspect()
{
    if (!aspect_) return;
    const QRect r = plotRect();
    if (r.width() <= 0 || r.height() <= 0) return;
    const double ux = (vx1_ - vx0_) / r.width();
    const double uy = (vy1_ - vy0_) / r.height();
    const double u = std::max(ux, uy);
    const double cx = 0.5 * (vx0_ + vx1_), cy = 0.5 * (vy0_ + vy1_);
    vx0_ = cx - u * r.width() / 2; vx1_ = cx + u * r.width() / 2;
    vy0_ = cy - u * r.height() / 2; vy1_ = cy + u * r.height() / 2;
}

QVector<double> PlotWidget::ticks(double a, double b, int approx, double* step)
{
    QVector<double> t;
    *step = 1;
    if (!(b > a) || !std::isfinite(a) || !std::isfinite(b)) return t;
    const double raw = (b - a) / std::max(1, approx);
    const double mag = std::pow(10.0, std::floor(std::log10(raw)));
    const double nrm = raw / mag;
    const double st = (nrm < 1.5 ? 1 : nrm < 3 ? 2 : nrm < 7 ? 5 : 10) * mag;
    *step = st;
    const double first = std::ceil(a / st) * st;
    for (int i = 0; i < 200; ++i) {
        double v = first + i * st;
        if (v > b + st * 1e-9) break;
        if (std::fabs(v) < st * 1e-9) v = 0;
        t.append(v);
    }
    return t;
}

QString PlotWidget::tickLabel(double v, double step)
{
    if (std::fabs(v) >= 1e9 || (std::fabs(v) > 0 && std::fabs(v) < 1e-5 && step < 1e-5))
        return QString::number(v, 'g', 6);
    const int dec = std::max(0, int(std::ceil(-std::log10(step) - 1e-9)));
    if (dec > 9) return QString::number(v, 'g', 6);
    return QString::number(v, 'f', dec);
}

QRect PlotWidget::plotRect() const
{
    const QFontMetrics fm(font());
    double st;
    const QVector<double> yt = ticks(vy0_, vy1_, std::max(3, height() / 45), &st);
    int lw = fm.horizontalAdvance(QStringLiteral("0.0"));
    for (double v : yt) lw = std::max(lw, fm.horizontalAdvance(tickLabel(v, st)));
    const int left = lw + 10 + (yLabel_.isEmpty() ? 0 : fm.height() + 4);
    const int top = title_.isEmpty() ? 8 : fm.height() + 10;
    const int bottom = fm.height() + 8 + (xLabel_.isEmpty() ? 0 : fm.height() + 2);
    const int right = hz_.isEmpty() ? 14 : 96;
    return QRect(left, top, std::max(10, width() - left - right), std::max(10, height() - top - bottom));
}

QPointF PlotWidget::toPx(double x, double y) const
{
    const QRect r = plotRect();
    return QPointF(clampPx(r.left() + (x - vx0_) / (vx1_ - vx0_) * r.width()),
                   clampPx(r.bottom() - (y - vy0_) / (vy1_ - vy0_) * r.height()));
}

QPointF PlotWidget::toData(const QPointF& p) const
{
    const QRect r = plotRect();
    return QPointF(vx0_ + (p.x() - r.left()) / r.width() * (vx1_ - vx0_),
                   vy0_ + (r.bottom() - p.y()) / r.height() * (vy1_ - vy0_));
}

QColor PlotWidget::colormap(double t) const
{
    static const int anchors[6][3] = {{48, 18, 59}, {50, 101, 212}, {30, 186, 171}, {140, 222, 60}, {250, 186, 40}, {200, 30, 20}};
    t = std::clamp(t, 0.0, 1.0) * 5.0;
    const int i = std::min(4, int(t));
    const double f = t - i;
    return QColor(int(anchors[i][0] + f * (anchors[i + 1][0] - anchors[i][0])),
                  int(anchors[i][1] + f * (anchors[i + 1][1] - anchors[i][1])),
                  int(anchors[i][2] + f * (anchors[i + 1][2] - anchors[i][2])));
}

void PlotWidget::drawHeatmap(QPainter& p, const QRect& r) const
{
    if (hz_.isEmpty() || hnx_ <= 0 || hny_ <= 0) return;
    const double dx = (hx1_ - hx0_) / hnx_, dy = (hy1_ - hy0_) / hny_;
    const int c0 = std::clamp(int(std::floor((vx0_ - hx0_) / dx)), 0, hnx_);
    const int c1 = std::clamp(int(std::ceil((vx1_ - hx0_) / dx)), 0, hnx_);
    const int r0 = std::clamp(int(std::floor((vy0_ - hy0_) / dy)), 0, hny_);
    const int r1 = std::clamp(int(std::ceil((vy1_ - hy0_) / dy)), 0, hny_);
    if (c1 <= c0 || r1 <= r0) return;
    const QPointF tl = toPx(hx0_ + c0 * dx, hy0_ + r1 * dy);
    const QPointF br = toPx(hx0_ + c1 * dx, hy0_ + r0 * dy);
    const QRectF target(tl, br);
    const int outW = std::max(1, std::min(c1 - c0, int(target.width()) + 1));
    const int outH = std::max(1, std::min(r1 - r0, int(target.height()) + 1));
    QImage img(outW, outH, QImage::Format_RGB32);
    const double lmin = hlog_ ? std::log10(std::max(1e-12f, hmin_)) : hmin_;
    const double lmax = hlog_ ? std::log10(std::max(1e-12f, hmax_)) : hmax_;
    for (int oy = 0; oy < outH; ++oy) {
        const int rb0 = r0 + int((long long)(outH - 1 - oy) * (r1 - r0) / outH);
        const int rb1 = std::max(rb0 + 1, r0 + int((long long)(outH - oy) * (r1 - r0) / outH));
        QRgb* line = reinterpret_cast<QRgb*>(img.scanLine(oy));
        for (int ox = 0; ox < outW; ++ox) {
            const int cb0 = c0 + int((long long)ox * (c1 - c0) / outW);
            const int cb1 = std::max(cb0 + 1, c0 + int((long long)(ox + 1) * (c1 - c0) / outW));
            float m = -std::numeric_limits<float>::max();
            for (int rr = rb0; rr < rb1; ++rr)
                for (int cc = cb0; cc < cb1; ++cc) m = std::max(m, hz_[rr * hnx_ + cc]);
            double v = hlog_ ? std::log10(std::max(1e-12f, m)) : m;
            line[ox] = colormap((v - lmin) / (lmax - lmin)).rgb();
        }
    }
    p.save();
    p.setClipRect(r);
    p.drawImage(target, img);
    p.restore();

    // colour bar
    const QRect cb(r.right() + 14, r.top(), 14, r.height());
    for (int y = 0; y < cb.height(); ++y) {
        p.setPen(colormap(1.0 - double(y) / std::max(1, cb.height() - 1)));
        p.drawLine(cb.left(), cb.top() + y, cb.right(), cb.top() + y);
    }
    p.setPen(palette().color(QPalette::Text));
    p.drawRect(cb);
    const QFontMetrics fm(font());
    p.drawText(QPointF(cb.right() + 3, cb.top() + fm.ascent()), QString::number(hmax_, 'g', 3));
    p.drawText(QPointF(cb.right() + 3, cb.bottom()), QString::number(hmin_, 'g', 3));
    if (!hLabel_.isEmpty()) {
        p.save();
        p.translate(cb.right() + 3 + fm.height(), cb.center().y());
        p.rotate(90);
        p.drawText(QRectF(-cb.height() / 2.0, -fm.height(), cb.height(), fm.height()), Qt::AlignCenter, hLabel_);
        p.restore();
    }
}

void PlotWidget::drawSeries(QPainter& p, const Series& s, const QRect& r) const
{
    const int n = int(s.x.size());
    if (n == 0) return;
    const double sx = r.width() / (vx1_ - vx0_), sy = r.height() / (vy1_ - vy0_);
    auto X = [&](double x) { return clampPx(r.left() + (x - vx0_) * sx); };
    auto Y = [&](double y) { return clampPx(r.bottom() - (y - vy0_) * sy); };
    int i0 = 0, i1 = n;
    if (s.sorted && n > 2) {
        i0 = std::max(0, int(std::lower_bound(s.x.begin(), s.x.end(), vx0_) - s.x.begin()) - 1);
        i1 = std::min(n, int(std::upper_bound(s.x.begin(), s.x.end(), vx1_) - s.x.begin()) + 1);
    }
    if (i1 <= i0) return;
    QPen pen(s.color, s.width);
    pen.setJoinStyle(Qt::RoundJoin);
    p.setPen(pen);
    p.setBrush(Qt::NoBrush);

    switch (s.style) {
    case Lines:
    case LinesPoints:
    case Steps: {
        const int count = i1 - i0;
        QPolygonF poly;
        if (s.style != Steps && s.sorted && count > 3 * r.width()) {
            poly.reserve(4 * r.width() + 8);
            int col = INT_MIN;
            double mn = 0, mx = 0, first = 0, last = 0, cx = 0;
            auto flush = [&] {
                if (col == INT_MIN) return;
                poly << QPointF(cx, Y(first));
                if (mn != first || mx != first) { poly << QPointF(cx, Y(mn)) << QPointF(cx, Y(mx)); }
                poly << QPointF(cx, Y(last));
            };
            for (int i = i0; i < i1; ++i) {
                const double y = s.y[i];
                if (!std::isfinite(y)) {
                    flush();
                    if (poly.size() > 1) p.drawPolyline(poly);
                    poly.clear();
                    col = INT_MIN;
                    continue;
                }
                const double px = X(s.x[i]);
                const int c = int(std::floor(px));
                if (c != col) {
                    flush();
                    col = c; cx = c + 0.5;
                    mn = mx = first = last = y;
                } else {
                    mn = std::min(mn, y); mx = std::max(mx, y); last = y;
                }
            }
            flush();
            if (poly.size() > 1) p.drawPolyline(poly);
        } else {
            poly.reserve(count * (s.style == Steps ? 2 : 1));
            for (int i = i0; i < i1; ++i) {
                if (!std::isfinite(s.y[i]) || !std::isfinite(s.x[i])) {
                    if (poly.size() > 1) p.drawPolyline(poly);
                    poly.clear();
                    continue;
                }
                const QPointF pt(X(s.x[i]), Y(s.y[i]));
                if (s.style == Steps && !poly.isEmpty()) poly << QPointF(pt.x(), poly.last().y());
                poly << pt;
            }
            if (s.style == Steps && !poly.isEmpty() && i1 == n && n > 1)
                poly << QPointF(X(s.x[n - 1] + (s.x[n - 1] - s.x[n - 2])), poly.last().y());
            if (poly.size() > 1) p.drawPolyline(poly);
            if (s.style == LinesPoints && count < 5000) {
                p.setBrush(s.color);
                for (int i = i0; i < i1; ++i)
                    if (std::isfinite(s.y[i])) p.drawEllipse(QPointF(X(s.x[i]), Y(s.y[i])), s.pointSize, s.pointSize);
            }
        }
        break;
    }
    case Points: {
        QPen pp(s.color, s.pointSize, Qt::SolidLine, Qt::RoundCap);
        p.setPen(pp);
        QPolygonF pts;
        pts.reserve(i1 - i0);
        const QRectF lim = QRectF(r).adjusted(-5, -5, 5, 5);
        for (int i = i0; i < i1; ++i) {
            if (!std::isfinite(s.x[i]) || !std::isfinite(s.y[i])) continue;
            const QPointF pt(X(s.x[i]), Y(s.y[i]));
            if (lim.contains(pt)) pts << pt;
        }
        p.drawPoints(pts);
        break;
    }
    case Bars: {
        double bw = s.barWidth;
        if (bw <= 0) {
            double md = std::numeric_limits<double>::max();
            for (int i = 1; i < n; ++i) md = std::min(md, std::fabs(s.x[i] - s.x[i - 1]));
            bw = (md < std::numeric_limits<double>::max() && md > 0) ? md * 0.8 : 0.8;
        }
        p.setPen(QPen(s.color.darker(140), 1));
        p.setBrush(s.color);
        for (int i = i0; i < i1; ++i) {
            if (!std::isfinite(s.y[i])) continue;
            const QRectF b(QPointF(X(s.x[i] - bw / 2), Y(s.y[i])), QPointF(X(s.x[i] + bw / 2), Y(0)));
            p.drawRect(b.normalized());
        }
        break;
    }
    case Sticks: {
        for (int i = i0; i < i1; ++i) {
            if (!std::isfinite(s.y[i])) continue;
            p.drawLine(QPointF(X(s.x[i]), Y(0)), QPointF(X(s.x[i]), Y(s.y[i])));
        }
        if (i1 - i0 < 600) {
            p.setBrush(s.color);
            for (int i = i0; i < i1; ++i)
                if (std::isfinite(s.y[i])) p.drawEllipse(QPointF(X(s.x[i]), Y(s.y[i])), 2.0, 2.0);
        }
        break;
    }
    }
}

void PlotWidget::drawAxes(QPainter& p, const QRect& r) const
{
    const QFontMetrics fm(font());
    const QColor text = palette().color(QPalette::Text);
    QColor gridC = palette().color(QPalette::Mid);
    gridC.setAlpha(110);
    double xs, ys;
    const QVector<double> xt = ticks(vx0_, vx1_, std::max(3, r.width() / 95), &xs);
    const QVector<double> yt = ticks(vy0_, vy1_, std::max(3, r.height() / 45), &ys);
    for (double x : xt) {
        const double px = toPx(x, 0).x();
        if (grid_) { p.setPen(QPen(gridC, 1, Qt::DotLine)); p.drawLine(QPointF(px, r.top()), QPointF(px, r.bottom())); }
        p.setPen(text);
        p.drawLine(QPointF(px, r.bottom()), QPointF(px, r.bottom() + 4));
        const QString lab = xTickText_ ? xTickText_(x) : tickLabel(x, xs);
        const int w = fm.horizontalAdvance(lab);
        p.drawText(QPointF(px - w / 2.0, r.bottom() + 5 + fm.ascent()), lab);
    }
    for (double y : yt) {
        const double py = toPx(0, y).y();
        if (grid_) { p.setPen(QPen(gridC, 1, Qt::DotLine)); p.drawLine(QPointF(r.left(), py), QPointF(r.right(), py)); }
        p.setPen(text);
        p.drawLine(QPointF(r.left() - 4, py), QPointF(r.left(), py));
        const QString lab = tickLabel(y, ys);
        const int w = fm.horizontalAdvance(lab);
        p.drawText(QPointF(r.left() - 6 - w, py + fm.ascent() / 2.0 - 1), lab);
    }
    p.setPen(text);
    p.setBrush(Qt::NoBrush);
    p.drawRect(r);
    if (!xLabel_.isEmpty())
        p.drawText(QRect(r.left(), r.bottom() + 6 + fm.height(), r.width(), fm.height() + 2), Qt::AlignCenter, xLabel_);
    if (!yLabel_.isEmpty()) {
        p.save();
        p.translate(3 + fm.ascent(), r.center().y());
        p.rotate(-90);
        p.drawText(QRectF(-r.height() / 2.0, -fm.ascent(), r.height(), fm.height()), Qt::AlignCenter, yLabel_);
        p.restore();
    }
    if (!title_.isEmpty()) {
        QFont f = font();
        f.setBold(true);
        p.setFont(f);
        p.drawText(QRect(0, 2, width(), fm.height() + 4), Qt::AlignCenter, title_);
        p.setFont(font());
    }
}

void PlotWidget::drawLegend(QPainter& p, const QRect& r) const
{
    if (!legend_) return;
    QStringList names;
    std::vector<const Series*> items;
    for (const Series& s : series_)
        if (s.legend && !s.name.isEmpty()) { names << s.name; items.push_back(&s); }
    if (items.empty() || items.size() > 16) return;
    const QFontMetrics fm(font());
    int w = 0;
    for (const QString& n : names) w = std::max(w, fm.horizontalAdvance(n));
    const int lh = fm.height();
    const QRect box(r.right() - w - 34, r.top() + 4, w + 30, lh * int(items.size()) + 6);
    QColor bg = palette().color(QPalette::Base);
    bg.setAlpha(210);
    p.setPen(palette().color(QPalette::Mid));
    p.setBrush(bg);
    p.drawRect(box);
    for (size_t i = 0; i < items.size(); ++i) {
        const int y = box.top() + 3 + int(i) * lh + lh / 2;
        p.setPen(QPen(items[i]->color, 2.5));
        if (items[i]->style == Points || items[i]->style == Bars) {
            p.setBrush(items[i]->color);
            p.drawRect(box.left() + 7, y - 3, 8, 6);
        } else {
            p.drawLine(box.left() + 4, y, box.left() + 20, y);
        }
        p.setPen(palette().color(QPalette::Text));
        p.drawText(QPointF(box.left() + 25, y + fm.ascent() / 2.0 - 1), names[int(i)]);
    }
}

QString PlotWidget::hoverText(const QPointF& pos) const
{
    const QPointF d = toData(pos);
    QString t = QStringLiteral("x = %1   y = %2").arg(num(d.x()), num(d.y()));
    if (!hz_.isEmpty() && d.x() >= hx0_ && d.x() < hx1_ && d.y() >= hy0_ && d.y() < hy1_) {
        const int c = std::clamp(int((d.x() - hx0_) / (hx1_ - hx0_) * hnx_), 0, hnx_ - 1);
        const int rr = std::clamp(int((d.y() - hy0_) / (hy1_ - hy0_) * hny_), 0, hny_ - 1);
        t += QStringLiteral("   z = %1").arg(num(hz_[rr * hnx_ + c]));
    }
    double best = 81.0;
    const Series* bs = nullptr;
    int bi = -1;
    const QRect r = plotRect();
    const double sx = r.width() / (vx1_ - vx0_), sy = r.height() / (vy1_ - vy0_);
    auto px = [&](double x, double y) { return QPointF(r.left() + (x - vx0_) * sx, r.bottom() - (y - vy0_) * sy); };
    for (const Series& s : series_) {
        const int n = int(s.x.size());
        auto test = [&](int i) {
            if (!std::isfinite(s.x[i]) || !std::isfinite(s.y[i])) return;
            const QPointF q = px(s.x[i], s.y[i]);
            const double dd = (q.x() - pos.x()) * (q.x() - pos.x()) + (q.y() - pos.y()) * (q.y() - pos.y());
            if (dd < best) { best = dd; bs = &s; bi = i; }
        };
        if (s.sorted && n > 64) {
            const int c = int(std::lower_bound(s.x.begin(), s.x.end(), d.x()) - s.x.begin());
            const double pxPerIdx = n > 1 ? std::max(1e-9, std::fabs(px(s.x[n - 1], 0).x() - px(s.x[0], 0).x()) / n) : 1;
            const int span = std::min(2000, std::max(4, int(10.0 / pxPerIdx) + 2));
            for (int i = std::max(0, c - span); i < std::min(n, c + span); ++i) test(i);
        } else if (n <= 60000) {
            for (int i = 0; i < n; ++i) test(i);
        }
    }
    if (bs) t += QStringLiteral("\n%1:  x = %2   y = %3").arg(bs->name.isEmpty() ? QStringLiteral("point") : bs->name,
                                                            xTickText_ ? xTickText_(bs->x[bi]) : num(bs->x[bi]), num(bs->y[bi]));
    return t;
}

void PlotWidget::paintEvent(QPaintEvent*)
{
    QPainter p(this);
    p.fillRect(rect(), palette().color(QPalette::Window));
    const QRect r = plotRect();
    p.fillRect(r, palette().color(QPalette::Base));
    if (!hasView_) autoScale();

    drawHeatmap(p, r);
    p.setRenderHint(QPainter::Antialiasing, true);
    p.save();
    p.setClipRect(r.adjusted(1, 1, 0, 0));
    for (const Span& s : spans_) {
        const double a = toPx(s.x0, 0).x(), b = toPx(s.x1, 0).x();
        p.fillRect(QRectF(QPointF(a, r.top()), QPointF(b, r.bottom())), s.c);
    }
    for (const Series& s : series_) drawSeries(p, s, r);
    const QFontMetrics fm(font());
    for (const Line& l : lines_) {
        p.setPen(QPen(l.c, 1.2, l.style));
        if (l.vertical) {
            const double x = toPx(l.v, 0).x();
            p.drawLine(QPointF(x, r.top()), QPointF(x, r.bottom()));
            if (!l.label.isEmpty()) p.drawText(QPointF(x + 3, r.top() + fm.ascent() + 2), l.label);
        } else {
            const double y = toPx(0, l.v).y();
            p.drawLine(QPointF(r.left(), y), QPointF(r.right(), y));
            if (!l.label.isEmpty()) p.drawText(QPointF(r.left() + 4, y - 3), l.label);
        }
    }
    for (const Circle& c : circles_) {
        p.setPen(QPen(c.c, 1.5, Qt::DashLine));
        p.setBrush(Qt::NoBrush);
        const QPointF a = toPx(c.cx - c.r, c.cy + c.r), b = toPx(c.cx + c.r, c.cy - c.r);
        p.drawEllipse(QRectF(a, b));
        if (!c.label.isEmpty()) {
            p.setPen(c.c);
            const double w = fm.horizontalAdvance(c.label);
            p.drawText(QPointF(0.5 * (a.x() + b.x()) - w / 2, a.y() - 3), c.label);
        }
    }
    for (const Text& t : texts_) {
        p.setPen(t.c);
        p.drawText(toPx(t.x, t.y) + QPointF(3, -3), t.text);
    }
    p.restore();
    p.setRenderHint(QPainter::Antialiasing, false);
    drawAxes(p, r);
    drawLegend(p, r);

    if (drag_ == Zoom) {
        p.setPen(QPen(palette().color(QPalette::Highlight), 1, Qt::DashLine));
        QColor f = palette().color(QPalette::Highlight);
        f.setAlpha(40);
        p.setBrush(f);
        p.drawRect(QRect(pressPos_, lastPos_).normalized());
    }
    if (hovering_ && r.contains(hover_) && drag_ == None) {
        const QString t = hoverText(hover_);
        const QStringList lines = t.split('\n');
        int w = 0;
        for (const QString& l : lines) w = std::max(w, fm.horizontalAdvance(l));
        const QRect box(r.left() + 4, r.top() + 4, w + 10, fm.height() * int(lines.size()) + 6);
        QColor bg = palette().color(QPalette::ToolTipBase);
        bg.setAlpha(225);
        p.setPen(palette().color(QPalette::Mid));
        p.setBrush(bg);
        p.drawRect(box);
        p.setPen(palette().color(QPalette::ToolTipText));
        for (int i = 0; i < lines.size(); ++i)
            p.drawText(QPointF(box.left() + 5, box.top() + 3 + fm.ascent() + i * fm.height()), lines[i]);
        QColor cc = palette().color(QPalette::Text);
        cc.setAlpha(70);
        p.setPen(QPen(cc, 1, Qt::DotLine));
        p.drawLine(hover_.x(), r.top(), hover_.x(), r.bottom());
        p.drawLine(r.left(), hover_.y(), r.right(), hover_.y());
    }
}

void PlotWidget::mousePressEvent(QMouseEvent* e)
{
    pressPos_ = lastPos_ = e->pos();
    moved_ = false;
    if (e->button() == Qt::LeftButton) drag_ = Zoom;
    else if (e->button() == Qt::RightButton || e->button() == Qt::MiddleButton) drag_ = Pan;
}

void PlotWidget::mouseMoveEvent(QMouseEvent* e)
{
    hover_ = e->pos();
    hovering_ = true;
    if (drag_ == Pan) {
        const QRect r = plotRect();
        const QPoint d = e->pos() - lastPos_;
        if ((e->pos() - pressPos_).manhattanLength() > 3) moved_ = true;
        const double dx = -d.x() * (vx1_ - vx0_) / r.width();
        const double dy = d.y() * (vy1_ - vy0_) / r.height();
        vx0_ += dx; vx1_ += dx; vy0_ += dy; vy1_ += dy;
        lastPos_ = e->pos();
        userView_ = true;
        emit xViewChanged(vx0_, vx1_);
    } else if (drag_ == Zoom) {
        lastPos_ = e->pos();
    }
    update();
}

void PlotWidget::mouseReleaseEvent(QMouseEvent* e)
{
    if (drag_ == Zoom && e->button() == Qt::LeftButton) {
        const QRect sel = QRect(pressPos_, e->pos()).normalized();
        drag_ = None;
        if (sel.width() > 5) userView_ = true;
        if (sel.width() > 5 && sel.height() > 5) {
            const QPointF a = toData(sel.topLeft()), b = toData(sel.bottomRight());
            vx0_ = a.x(); vx1_ = b.x(); vy0_ = b.y(); vy1_ = a.y();
            applyAspect();
            emit xViewChanged(vx0_, vx1_);
        } else if (sel.width() > 5) {
            const QPointF a = toData(sel.topLeft()), b = toData(sel.bottomRight());
            vx0_ = a.x(); vx1_ = b.x();
            emit xViewChanged(vx0_, vx1_);
        } else {
            const QPointF d = toData(e->pos());
            emit clicked(d.x(), d.y());
        }
        update();
        return;
    }
    if (drag_ == Pan) {
        drag_ = None;
        if (!moved_ && e->button() == Qt::RightButton) showMenu(e->globalPosition().toPoint());
        update();
        return;
    }
    drag_ = None;
}

void PlotWidget::mouseDoubleClickEvent(QMouseEvent*)
{
    userView_ = false;
    autoScale();
    emit xViewChanged(vx0_, vx1_);
}

void PlotWidget::wheelEvent(QWheelEvent* e)
{
    const double steps = e->angleDelta().y() / 120.0;
    if (steps == 0) return;
    const double f = std::pow(0.8, steps);
    const QPointF c = toData(e->position());
    const bool zx = !(e->modifiers() & Qt::ShiftModifier);
    const bool zy = !(e->modifiers() & Qt::ControlModifier);
    if (zx) { vx0_ = c.x() + (vx0_ - c.x()) * f; vx1_ = c.x() + (vx1_ - c.x()) * f; }
    if (zy) { vy0_ = c.y() + (vy0_ - c.y()) * f; vy1_ = c.y() + (vy1_ - c.y()) * f; }
    userView_ = true;
    applyAspect();
    emit xViewChanged(vx0_, vx1_);
    update();
    e->accept();
}

void PlotWidget::leaveEvent(QEvent*)
{
    hovering_ = false;
    update();
}

void PlotWidget::showMenu(const QPoint& global)
{
    QMenu m(this);
    m.addAction(tr("Reset view (double-click)"), this, [this] { userView_ = false; autoScale(); emit xViewChanged(vx0_, vx1_); });
    QAction* lg = m.addAction(tr("Show legend"), this, [this](bool on) { legend_ = on; update(); });
    lg->setCheckable(true);
    lg->setChecked(legend_);
    QAction* gr = m.addAction(tr("Show grid"), this, [this](bool on) { grid_ = on; update(); });
    gr->setCheckable(true);
    gr->setChecked(grid_);
    m.addSeparator();
    m.addAction(tr("Copy image"), this, &PlotWidget::copyImage);
    m.addAction(tr("Save image as PNG..."), this, &PlotWidget::savePng);
    m.addAction(tr("Export data to CSV..."), this, &PlotWidget::exportCsv);
    m.addSeparator();
    QAction* help = m.addAction(tr("Wheel: zoom (Ctrl = x only, Shift = y only); left-drag: box zoom; right-drag: pan"));
    help->setEnabled(false);
    m.exec(global);
}

void PlotWidget::copyImage()
{
    QApplication::clipboard()->setPixmap(grab());
}

void PlotWidget::savePng()
{
    const QString fn = QFileDialog::getSaveFileName(this, tr("Save plot"), title_.isEmpty() ? QStringLiteral("plot.png") : title_ + ".png",
                                                    tr("PNG images (*.png)"));
    if (!fn.isEmpty()) grab().save(fn, "PNG");
}

void PlotWidget::exportCsv()
{
    const QString fn = QFileDialog::getSaveFileName(this, tr("Export plot data"), QString(), tr("CSV files (*.csv)"));
    if (fn.isEmpty()) return;
    QFile f(fn);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Text)) return;
    QTextStream ts(&f);
    ts << "series,x,y\n";
    for (const Series& s : series_)
        for (int i = 0; i < s.x.size(); ++i)
            ts << '"' << s.name << "\"," << num(s.x[i]) << ',' << num(s.y[i]) << '\n';
    if (!hz_.isEmpty()) {
        ts << "\nheatmap x,y,z\n";
        for (int r = 0; r < hny_; ++r)
            for (int c = 0; c < hnx_; ++c)
                ts << num(hx0_ + (c + 0.5) * (hx1_ - hx0_) / hnx_) << ',' << num(hy0_ + (r + 0.5) * (hy1_ - hy0_) / hny_)
                   << ',' << num(hz_[r * hnx_ + c]) << '\n';
    }
}

} // namespace gui
