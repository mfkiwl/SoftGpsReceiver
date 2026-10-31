#include "GuiUtil.h"
#include "DataTable.h"
#include "PlotWidget.h"
#include "SkyPlotWidget.h"
#include "core/Constants.h"

#include <QVBoxLayout>
#include <cmath>

namespace gui {

static int g_readableStage = 0;

static bool g_workerRunning = false;

int readableStage() { return g_readableStage; }
void setReadableStage(int s) { g_readableStage = s; }
bool workerRunning() { return g_workerRunning; }
void setWorkerRunning(bool on) { g_workerRunning = on; }

static bool g_liveMode = false;

bool liveMode() { return g_liveMode; }
void setLiveMode(bool on) { g_liveMode = on; }

bool PageBase::readable(int stage) const { return rx_ && g_readableStage >= stage; }

void PageBase::setReceiver(gnss::Receiver* rx)
{
    rx_ = rx;
    for (PlotWidget* p : findChildren<PlotWidget*>()) plotLabels_.push_back({p, p->title(), p->xLabel(), p->yLabel()});
    for (DataTable* t : findChildren<DataTable*>()) {
        QStringList headers;
        for (int c = 0; c < t->columnCount(); ++c)
            headers << (t->horizontalHeaderItem(c) ? t->horizontalHeaderItem(c)->text() : QString());
        tableColumns_.push_back({t, headers});
    }
    for (QLabel* l : findChildren<QLabel*>()) labelTexts_.push_back({l, l->text()});
    for (QGroupBox* g : findChildren<QGroupBox*>()) groupTitles_.push_back({g, g->title()});
}

void PageBase::clearContents()
{
    for (const PlotLabels& l : plotLabels_) {
        l.plot->clear();
        l.plot->setLabels(l.title, l.x, l.y);
    }
    for (const TableColumns& c : tableColumns_) {
        c.table->beginFill(0);
        c.table->setColumns(c.headers);
        c.table->endFill();
    }
    for (SkyPlotWidget* s : findChildren<SkyPlotWidget*>()) {
        s->setData({}, {});
        s->setElevationMask(0);
    }
    for (const LabelText& l : labelTexts_) l.label->setText(l.text);
    for (const GroupTitle& g : groupTitles_) g.group->setTitle(g.title);
    if (auto* spin = findChild<QSpinBox*>(QStringLiteral("epochSpin")))
        setEpochRange(spin, findChild<QSlider*>(QStringLiteral("epochSlider")), 0);
}

QColor seriesColor(int i)
{
    static const QColor c[] = {QColor(31, 119, 180), QColor(255, 127, 14), QColor(44, 160, 44), QColor(214, 39, 40),
                               QColor(148, 103, 189), QColor(140, 86, 75), QColor(227, 119, 194), QColor(127, 127, 127),
                               QColor(188, 189, 34), QColor(23, 190, 207), QColor(0, 0, 128), QColor(128, 128, 0)};
    const int n = int(sizeof(c) / sizeof(c[0]));
    return c[((i % n) + n) % n];
}

QColor prnColor(int prn)
{
    return QColor::fromHsv((prn * 47) % 360, 200, 200);
}

QString satLabel(int id)
{
    return gnss::isGalileo(id) ? gnss::satName(id) : QString::number(id);
}

QString satSystem(int id)
{
    return gnss::isGalileo(id) ? QStringLiteral("Galileo") : QStringLiteral("GPS");
}
QString satFullName(int id)
{
    return gnss::isGalileo(id) ? QStringLiteral("Galileo %1").arg(gnss::satName(id)) : QStringLiteral("GPS %1").arg(gnss::satName(id));
}
double satAxis(int id)
{
    return gnss::isGalileo(id) ? 35.0 + gnss::svidOf(id) : double(id);
}
QString satAxisText(double x)
{
    const int n = int(std::lround(x));
    if (std::fabs(x - n) > 1e-6) return QString();
    if (n >= 36 && n <= 71) return gnss::satName(gnss::kGalileoIdBase + n - 35);
    return n >= 1 && n <= 32 ? QString::number(n) : QString();
}

QColor goodColor() { return QColor(170, 230, 170); }
QColor badColor() { return QColor(245, 170, 170); }
QColor warnColor() { return QColor(250, 225, 150); }
QColor neutralColor() { return QColor(225, 225, 225); }

QVector<double> range(int n, double scale, double offset)
{
    QVector<double> r(n);
    for (int i = 0; i < n; ++i) r[i] = i * scale + offset;
    return r;
}

QString fmt(double v, int prec)
{
    if (!std::isfinite(v)) return QStringLiteral("-");
    return QString::number(v, 'f', prec);
}

QString fmtSci(double v, int prec)
{
    if (!std::isfinite(v)) return QStringLiteral("-");
    return QString::number(v, 'g', prec);
}

QString deg(double rad, int prec)
{
    return QString::number(rad * 180.0 / gnss::kPi, 'f', prec);
}

QGroupBox* group(const QString& title, QWidget* content)
{
    auto* g = new QGroupBox(title);
    auto* l = new QVBoxLayout(g);
    l->setContentsMargins(4, 4, 4, 4);
    l->addWidget(content);
    return g;
}

QLabel* valueLabel()
{
    auto* l = new QLabel(QStringLiteral("-"));
    l->setTextInteractionFlags(Qt::TextSelectableByMouse);
    return l;
}

void setEpochRange(QSpinBox* spin, QSlider* slider, int count)
{
    const bool atEnd = spin->value() >= spin->maximum();
    slider->setRange(0, count - 1);
    spin->setRange(0, count - 1);
    if (g_liveMode && atEnd) spin->setValue(count - 1);
}

QString channelLabel(const gnss::TrackingChannel& ch)
{
    return QStringLiteral("Ch %1 - %2").arg(ch.channel).arg(gnss::satName(ch.prn));
}

} // namespace gui
