#include "LivePage.h"
#include "ui_LivePage.h"
#include "DataTable.h"
#include "PlotWidget.h"
#include "SkyPlotWidget.h"
#include "core/Constants.h"
#include "core/Geodesy.h"

#include <QFileInfo>
#include <QLabel>
#include <cmath>
#include <map>

using namespace gnss;

namespace gui {

namespace {

constexpr int kMaxHistory = 3600;

QColor stateColor(const QString& state)
{
    if (state == QLatin1String("Ready")) return goodColor();
    if (state == QLatin1String("Pull-in")) return neutralColor();
    return warnColor();
}

} // namespace

LivePage::LivePage(QWidget* parent) : PageBase(parent), ui(new Ui::LivePage)
{
    ui->setupUi(this);
    ui->splitter->setStretchFactor(0, 1);
    ui->splitter->setStretchFactor(1, 3);
    ui->rightSplitter->setStretchFactor(0, 1);
    ui->rightSplitter->setStretchFactor(1, 2);
    ui->rightSplitter->setSizes({300, 600});
    for (QLabel* l : findChildren<QLabel*>())
        if (l->objectName().endsWith("Value")) {
            l->setWordWrap(true);
            l->setTextInteractionFlags(Qt::TextSelectableByMouse);
        }
    ui->channelsTable->setColumns({tr("Channel"), tr("Satellite"), tr("System"), tr("State"), tr("C/N0"), tr("Doppler (Hz)"), tr("Phase lock"),
                                   tr("Tracked (s)"), tr("Decoded"), tr("Orbit"), tr("Elevation"), tr("Azimuth"), tr("Used")});
    ui->cn0Plot->setFixedYRange(0, 60);
    ui->cn0Plot->setXTickText(satAxisText);
    connect(ui->startButton, &QPushButton::clicked, this, &LivePage::startRequested);
    connect(ui->stopButton, &QPushButton::clicked, this, &LivePage::stopRequested);
    connect(ui->settingsButton, &QPushButton::clicked, this, &LivePage::settingsRequested);
    connect(ui->freezeCheck, &QCheckBox::toggled, this, &LivePage::freezeToggled);
    connect(ui->channelsTable, &QTableWidget::cellDoubleClicked, this, [this](int r, int) {
        const QVariant k = ui->channelsTable->keyOfRow(r);
        if (k.isValid()) emit prnSelected(k.toInt());
    });
}

LivePage::~LivePage()
{
    delete ui;
}

bool LivePage::frozen() const
{
    return ui->freezeCheck->isChecked();
}

void LivePage::clearContents()
{
    PageBase::clearContents();
    history_.clear();
    channels_.clear();
    status_ = LiveStatus();
}

void LivePage::setRunning(bool running)
{
    ui->startButton->setEnabled(!running);
    ui->stopButton->setEnabled(running);
    ui->settingsButton->setEnabled(!running);
    if (!running) ui->freezeCheck->setChecked(false);
}

void LivePage::setMessage(const QString& text)
{
    ui->stateLabel->setText(text);
}

void LivePage::setStatus(const LiveStatus& st, const std::vector<LiveChannelStatus>& channels)
{
    status_ = st;
    channels_ = channels;
    for (const LiveChannelStatus& c : channels) {
        QVector<QPointF>& h = history_[c.prn];
        h.append(QPointF(st.streamSec, c.cn0));
        if (h.size() > kMaxHistory) h.remove(0, h.size() - kMaxHistory);
    }

    ui->sourceValue->setText(st.source);
    ui->streamValue->setText(tr("%1 s received, %2 MS/s").arg(st.streamSec, 0, 'f', 1).arg(st.measuredRate / 1e6, 0, 'f', 3));
    if (st.levelDbfs != 0.0) {
        QString lvl = tr("%1 dBFS RMS, %2 % clipped").arg(st.levelDbfs, 0, 'f', 1).arg(st.clipPercent, 0, 'f', 2);
        if (st.levelDbfs > -12.0 || st.clipPercent > 0.5) lvl += tr(" - gain too high");
        else if (st.levelDbfs < -65.0) lvl += tr(" - gain low");
        ui->levelValue->setText(lvl);
    } else {
        ui->levelValue->setText(tr("%1 counts RMS").arg(st.levelRms, 0, 'f', 1));
    }
    ui->lossValue->setText(tr("%1 / %2 ms").arg(st.overflows).arg(st.lostSamples / std::max(1.0, st.sampleRate) * 1000.0, 0, 'f', 0));
    ui->lagValue->setText(tr("%1 ms (%2 % of the buffer)").arg(st.lagMs, 0, 'f', 1).arg(st.bufferUsed * 100, 0, 'f', 1));
    ui->acqValue->setText(tr("%1 searches, last took %2 s").arg(st.acqCycles).arg(st.lastAcqSec, 0, 'f', 2));
    ui->recordValue->setText(st.recordFile.isEmpty() ? tr("off")
                                                     : tr("%1 (%2 MB)").arg(QFileInfo(st.recordFile).fileName())
                                                           .arg(st.recordedBytes / 1048576.0, 0, 'f', 0));
    int gps = 0, galileo = 0;
    for (const LiveChannelStatus& c : channels) ++(isGalileo(c.prn) ? galileo : gps);
    ui->stateLabel->setText(st.running ? tr("Running: %1 GPS and %2 Galileo channels, %3 with orbit and timing").arg(gps).arg(galileo).arg(st.usable)
                                       : tr("Stopped"));
    ui->ttffValue->setText(st.ttffSec >= 0 ? tr("%1 s").arg(st.ttffSec, 0, 'f', 1) : tr("waiting for a fix"));

    DataTable* t = ui->channelsTable;
    t->beginFill(int(channels.size()));
    for (int r = 0; r < int(channels.size()); ++r) {
        const LiveChannelStatus& c = channels[size_t(r)];
        t->setNum(r, 0, c.channel, 0);
        t->setKey(r, c.prn);
        t->setText(r, 1, satName(c.prn));
        t->setText(r, 2, satSystem(c.prn));
        t->setText(r, 3, c.state, stateColor(c.state));
        t->setNum(r, 4, c.cn0, 1, c.cn0 >= 35 ? goodColor() : warnColor());
        t->setNum(r, 5, c.doppler, 1);
        t->setNum(r, 6, c.pli, 2);
        t->setNum(r, 7, c.ageSec, 0);
        t->setNum(r, 8, c.subframes, 0);
        t->setText(r, 9, !c.ephemeris ? tr("-") : (c.cachedEphemeris ? tr("remembered") : tr("complete")), c.ephemeris ? goodColor() : QColor());
    }
    t->endFill();
}

void LivePage::updatePosition()
{
    const NavSolution* fix = nullptr;
    if (rx_ && readable(Receiver::StageSolved))
        for (auto it = rx_->pvt.sols.rbegin(); it != rx_->pvt.sols.rend() && !fix; ++it)
            if (it->valid) fix = &*it;

    for (int r = 0; r < ui->channelsTable->rowCount(); ++r) {
        if (!fix) break;
        const int prn = ui->channelsTable->keyOfRow(r).toInt();
        for (const auto& o : fix->obs) {
            if (o.prn != prn) continue;
            if (o.geomRange > 0) {
                ui->channelsTable->setNum(r, 10, o.el * 180 / kPi, 1);
                ui->channelsTable->setNum(r, 11, o.az * 180 / kPi, 1);
            }
            ui->channelsTable->setText(r, 12, o.used ? tr("yes") : o.status, o.used ? goodColor() : warnColor());
        }
    }
    ui->channelsTable->resizeColumnToContents(12);
    if (!fix) {
        ui->fixValue->setText(status_.running ? tr("No fix yet: needs four satellites with a whole orbit, up to about a minute")
                                              : QStringLiteral("-"));
        return;
    }
    const double age = (double(status_.samples) - fix->rxSample) / std::max(1.0, status_.sampleRate);
    ui->fixValue->setText(age < 3.0 ? tr("Current") : tr("Lost: last fix %1 s ago").arg(age, 0, 'f', 0));
    ui->latValue->setText(tr("%1°  (%2)").arg(fix->llh.lat * 180 / kPi, 0, 'f', 7).arg(geo::formatDms(fix->llh.lat, true)));
    ui->lonValue->setText(tr("%1°  (%2)").arg(fix->llh.lon * 180 / kPi, 0, 'f', 7).arg(geo::formatDms(fix->llh.lon, false)));
    ui->heightValue->setText(tr("%1 m").arg(fix->llh.h, 0, 'f', 1));
    if (fix->velValid)
        ui->velocityValue->setText(tr("%1 / %2 / %3 m/s").arg(fix->velEnu.x, 0, 'f', 2).arg(fix->velEnu.y, 0, 'f', 2).arg(fix->velEnu.z, 0, 'f', 2));
    ui->dopValue->setText(tr("%1 / %2 / %3 m").arg(fix->nUsed).arg(fix->pdop, 0, 'f', 2).arg(fix->rms, 0, 'f', 1));
    ui->timeValue->setText(tr("%1 s (%2)").arg(fix->gpsTime, 0, 'f', 1).arg(nav::towText(fix->gpsTime)));

    QVector<SkyPlotWidget::Sat> sats;
    for (const auto& o : fix->obs) {
        if (o.geomRange <= 0) continue;
        SkyPlotWidget::Sat s;
        s.prn = o.prn;
        s.az = o.az * 180 / kPi;
        s.el = o.el * 180 / kPi;
        s.color = o.used ? prnColor(o.prn) : QColor(150, 150, 150);
        s.filled = true;
        s.tip = tr("C/N0 %1 dB-Hz").arg(o.cn0, 0, 'f', 1);
        sats.push_back(s);
    }
    ui->skyPlot->setData(sats, {});

    QVector<double> e, n, t, u, used;
    for (const auto& s : rx_->pvt.sols) {
        if (!s.valid) continue;
        e << s.enu.x; n << s.enu.y;
        t << s.gpsTime; u << s.enu.z; used << s.nUsed;
    }
    ui->scatterPlot->clear();
    ui->scatterPlot->addSeries(tr("fixes"), e, n, PlotWidget::Points, QColor(31, 119, 180));
    ui->scatterPlot->series(0).sorted = false;
    ui->scatterPlot->series(0).pointSize = 4;
    ui->scatterPlot->addSeries(tr("newest"), {e.last()}, {n.last()}, PlotWidget::Points, QColor(214, 39, 40));
    ui->scatterPlot->series(1).pointSize = 9;
    ui->scatterPlot->addCircle(0, 0, rx_->pvt.cep50, QColor(214, 39, 40), tr("CEP50 %1 m").arg(rx_->pvt.cep50, 0, 'f', 1));
    ui->heightPlot->clear();
    ui->heightPlot->addSeries(tr("height"), t, u, PlotWidget::Lines, QColor(214, 39, 40), 1.2);
    ui->heightPlot->addSeries(tr("satellites used"), t, used, PlotWidget::Steps, QColor(31, 119, 180), 1.2);
}

void LivePage::refresh()
{
    QVector<double> px, py, gx, gy;
    for (const LiveChannelStatus& c : channels_) {
        const double x = satAxis(c.prn);
        if (c.state == QLatin1String("Ready")) { px << x; py << c.cn0; }
        else { gx << x; gy << c.cn0; }
    }
    PlotWidget* b = ui->cn0Plot;
    b->clear();
    b->addSeries(tr("ready"), px, py, PlotWidget::Bars, QColor(44, 160, 44));
    b->addSeries(tr("not ready yet"), gx, gy, PlotWidget::Bars, QColor(250, 190, 90));
    b->series(0).barWidth = b->series(1).barWidth = 0.7;
    b->autoScale();

    PlotWidget* h = ui->historyPlot;
    h->clear();
    for (auto it = history_.constBegin(); it != history_.constEnd(); ++it) {
        QVector<double> x, y;
        x.reserve(it.value().size());
        y.reserve(it.value().size());
        for (const QPointF& p : it.value()) { x << p.x(); y << p.y(); }
        h->addSeries(satName(it.key()), x, y, PlotWidget::Lines, prnColor(it.key()), 1.2);
    }
    updatePosition();
}

} // namespace gui
