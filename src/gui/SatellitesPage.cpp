#include "SatellitesPage.h"
#include "ui_SatellitesPage.h"
#include "DataTable.h"
#include "PlotWidget.h"
#include "SkyPlotWidget.h"
#include "core/Constants.h"
#include "core/Ephemeris.h"
#include "core/Geodesy.h"

#include <cmath>
#include <map>

using namespace gnss;

namespace gui {

SatellitesPage::SatellitesPage(QWidget* parent) : PageBase(parent), ui(new Ui::SatellitesPage)
{
    ui->setupUi(this);
    ui->splitter->setStretchFactor(0, 2);
    ui->splitter->setStretchFactor(1, 3);
    ui->groundPlot->setAspectLocked(true);
    ui->stateTable->setColumns({tr("PRN"), tr("Used"), tr("X (km)"), tr("Y (km)"), tr("Z (km)"), tr("|r| (km)"),
                                tr("Vx (m/s)"), tr("Vy (m/s)"), tr("Vz (m/s)"), tr("|v| (m/s)"), tr("Elevation (deg)"),
                                tr("Azimuth (deg)"), tr("Range (km)"), tr("Range rate (m/s)"), tr("SV clock (us)"),
                                tr("Relativistic (ns)"), tr("T_GD (ns)"), tr("SV clock drift (ns/s)"), tr("Transmit time (s)"),
                                tr("Travel time (ms)"), tr("Earth rotation (m)")});
    ui->keplerTable->setColumns({tr("Symbol"), tr("Computation"), tr("Value"), tr("Units")});
    connect(ui->epochSpin, &QSpinBox::valueChanged, this, &SatellitesPage::showEpoch);
    connect(ui->prnCombo, &QComboBox::currentIndexChanged, this, &SatellitesPage::showKepler);
    connect(ui->skyPlot, &SkyPlotWidget::satelliteClicked, this, &SatellitesPage::selectPrn);
    connect(ui->stateTable, &QTableWidget::cellClicked, this, [this](int r, int) {
        const QVariant k = ui->stateTable->keyOfRow(r);
        if (k.isValid()) selectPrn(k.toInt());
    });
}

SatellitesPage::~SatellitesPage()
{
    delete ui;
}

const NavSolution* SatellitesPage::currentSolution() const
{
    if (!readable(Receiver::StageSolved)) return nullptr;
    const int i = ui->epochSpin->value();
    if (i < 0 || i >= int(rx_->pvt.sols.size())) return nullptr;
    const NavSolution* s = &rx_->pvt.sols[size_t(i)];
    return s->valid ? s : nullptr;
}

void SatellitesPage::refresh()
{
    ui->skyPlot->setData({}, {});
    ui->stateTable->beginFill(0);
    ui->stateTable->endFill();
    ui->keplerTable->beginFill(0);
    ui->keplerTable->endFill();
    ui->elevationPlot->clear();
    ui->groundPlot->clear();
    ui->prnCombo->blockSignals(true);
    ui->prnCombo->clear();
    if (!readable(Receiver::StageSolved) || rx_->pvt.sols.empty()) {
        ui->prnCombo->blockSignals(false);
        ui->epochInfoLabel->setText(tr("(no position solution)"));
        return;
    }
    for (int i : rx_->pvt.usableChannels) ui->prnCombo->addItem(satName(rx_->nav[size_t(i)].prn), rx_->nav[size_t(i)].prn);
    ui->prnCombo->blockSignals(false);
    const int n = int(rx_->pvt.sols.size());
    setEpochRange(ui->epochSpin, ui->epochSlider, n);
    ui->skyPlot->setElevationMask(rx_->settings.elevationMask);
    fillTimePlots();
    showEpoch();
}

void SatellitesPage::selectPrn(int prn)
{
    const int i = ui->prnCombo->findData(prn);
    if (i >= 0) ui->prnCombo->setCurrentIndex(i);
}

void SatellitesPage::fillTimePlots()
{
    std::map<int, QVector<double>> t, el, lat, lon;
    for (const auto& s : rx_->pvt.sols) {
        if (!s.valid) continue;
        for (const auto& o : s.obs) {
            if (o.geomRange <= 0) continue;
            t[o.prn] << s.gpsTime;
            el[o.prn] << o.el * 180 / kPi;
            const Geodetic g = geo::ecefToGeodetic(o.satPos);
            lat[o.prn] << g.lat * 180 / kPi;
            lon[o.prn] << g.lon * 180 / kPi;
        }
    }
    int k = 0;
    for (const auto& kv : t) {
        const QColor c = prnColor(kv.first);
        ui->elevationPlot->addSeries(satName(kv.first), kv.second, el[kv.first], PlotWidget::Lines, c, 2.0);
        ui->groundPlot->addSeries(satName(kv.first), lon[kv.first], lat[kv.first], PlotWidget::LinesPoints, c, 2.0);
        auto& s = ui->groundPlot->series(ui->groundPlot->seriesCount() - 1);
        s.sorted = false;
        s.pointSize = 1.5;
        if (!lat[kv.first].isEmpty())
            ui->groundPlot->addText(lon[kv.first].last(), lat[kv.first].last(), satLabel(kv.first), c);
        ++k;
    }
    ui->elevationPlot->addHLine(rx_->settings.elevationMask, QColor(214, 39, 40), tr("mask"));
    const Geodetic& m = rx_->pvt.meanLlh;
    ui->groundPlot->addSeries(tr("receiver"), {m.lon * 180 / kPi}, {m.lat * 180 / kPi}, PlotWidget::Points, QColor(0, 0, 0));
    ui->groundPlot->series(ui->groundPlot->seriesCount() - 1).pointSize = 10;
    for (int la = -60; la <= 60; la += 30) ui->groundPlot->addHLine(la, QColor(180, 180, 180), QString(), Qt::DotLine);
    ui->groundPlot->addHLine(0, QColor(120, 120, 120), tr("equator"), Qt::SolidLine);
    ui->groundPlot->autoScale();
}

void SatellitesPage::showEpoch()
{
    ui->stateTable->beginFill(0);
    ui->stateTable->endFill();
    const NavSolution* sol = currentSolution();
    if (!sol) {
        ui->epochInfoLabel->setText(tr("(invalid epoch)"));
        return;
    }
    ui->epochInfoLabel->setText(tr("GPS TOW %1 s (%2), %3 satellites used")
                                    .arg(sol->gpsTime, 0, 'f', 3).arg(nav::towText(sol->gpsTime)).arg(sol->nUsed));

    // sky plot
    QVector<SkyPlotWidget::Sat> sats;
    QVector<SkyPlotWidget::Track> tracks;
    std::map<int, QVector<QPointF>> tr_;
    for (const auto& s : rx_->pvt.sols) {
        if (!s.valid) continue;
        for (const auto& o : s.obs)
            if (o.geomRange > 0) tr_[o.prn] << QPointF(o.az * 180 / kPi, o.el * 180 / kPi);
    }
    for (const auto& kv : tr_) tracks.push_back({kv.first, kv.second, prnColor(kv.first)});
    QVector<int> shown;
    for (const auto& o : sol->obs) {
        if (o.geomRange <= 0) continue;
        SkyPlotWidget::Sat s;
        s.prn = o.prn;
        s.az = o.az * 180 / kPi;
        s.el = o.el * 180 / kPi;
        s.color = o.used ? prnColor(o.prn) : QColor(150, 150, 150);
        s.filled = true;
        s.tip = tr("C/N0 %1 dB-Hz\n%2").arg(o.cn0, 0, 'f', 1).arg(o.used ? tr("used in fix") : o.status);
        sats.push_back(s);
        shown << o.prn;
    }
    // satellites predicted from the almanac but not tracked
    const GlobalNavData& g = rx_->gnav;
    for (int sv = 1; sv <= 32; ++sv) {
        if (!g.almanac[sv].valid || shown.contains(sv)) continue;
        const Vec3 p = orbit::almanacPosition(g.almanac[sv], sol->gpsTime);
        double az, el;
        geo::azimuthElevation(sol->pos, p, &az, &el);
        if (el < 0) continue;
        SkyPlotWidget::Sat s;
        s.prn = sv;
        s.az = az * 180 / kPi;
        s.el = el * 180 / kPi;
        s.color = QColor(120, 120, 120);
        s.filled = false;
        s.tip = tr("predicted from almanac (not tracked)");
        sats.push_back(s);
    }
    ui->skyPlot->setData(sats, tracks);

    DataTable* t = ui->stateTable;
    t->beginFill(int(sol->obs.size()));
    int r = 0;
    for (const auto& o : sol->obs) {
        if (o.geomRange <= 0) continue;
        t->setText(r, 0, satLabel(o.prn));
        t->setKey(r, o.prn);
        t->setText(r, 1, o.used ? tr("yes") : o.status, o.used ? goodColor() : warnColor());
        t->setNum(r, 2, o.satPosTx.x / 1000, 4);
        t->setNum(r, 3, o.satPosTx.y / 1000, 4);
        t->setNum(r, 4, o.satPosTx.z / 1000, 4);
        t->setNum(r, 5, o.satPosTx.norm() / 1000, 4);
        t->setNum(r, 6, o.satVelTx.x, 3);
        t->setNum(r, 7, o.satVelTx.y, 3);
        t->setNum(r, 8, o.satVelTx.z, 3);
        t->setNum(r, 9, o.satVelTx.norm(), 3);
        t->setNum(r, 10, o.el * 180 / kPi, 3);
        t->setNum(r, 11, o.az * 180 / kPi, 3);
        t->setNum(r, 12, o.geomRange / 1000, 4);
        t->setNum(r, 13, o.predRangeRate, 3);
        t->setNum(r, 14, o.satClock * 1e6, 4);
        t->setNum(r, 15, o.satClockRel * 1e9, 3);
        t->setNum(r, 16, o.tgd * 1e9, 3);
        t->setNum(r, 17, o.satClkDrift * 1e9, 5);
        t->setNum(r, 18, o.txTimeGps, 7);
        t->setNum(r, 19, o.geomRange / kSpeedOfLight * 1000, 4);
        t->setNum(r, 20, o.sagnac, 3);
        ++r;
    }
    t->setRowCount(r);
    t->endFill();
    showKepler();
}

void SatellitesPage::showKepler()
{
    DataTable* t = ui->keplerTable;
    t->beginFill(0);
    t->endFill();
    const NavSolution* sol = currentSolution();
    const int prn = ui->prnCombo->currentData().toInt();
    if (!sol || prn <= 0) return;
    const SatObservation* obs = nullptr;
    for (const auto& o : sol->obs)
        if (o.prn == prn) obs = &o;
    const NavChannel* nc = nullptr;
    for (const auto& n : rx_->nav)
        if (n.prn == prn) nc = &n;
    if (!obs || !nc || !nc->eph.complete()) return;
    std::vector<KeplerStep> steps;
    steps.push_back({QStringLiteral("t_sv"), tr("Satellite time of transmission (from code epoch timing)"), obs->txTime, "s"});
    steps.push_back({QStringLiteral("dt_sv"), tr("SV clock correction (iterated)"), obs->satClock, "s"});
    orbit::satState(nc->eph, obs->txTimeGps, &steps);
    steps.push_back({QStringLiteral("tau"), tr("Signal travel time (geometric range / c)"), obs->geomRange / kSpeedOfLight, "s"});
    steps.push_back({QStringLiteral("theta"), tr("Earth rotation during travel  OmegaE_dot * tau"),
                     kOmegaEDot * obs->geomRange / kSpeedOfLight, "rad"});
    steps.push_back({QStringLiteral("X_rot"), tr("X rotated into the ECEF frame at reception"), obs->satPos.x, "m"});
    steps.push_back({QStringLiteral("Y_rot"), tr("Y rotated into the ECEF frame at reception"), obs->satPos.y, "m"});
    steps.push_back({QStringLiteral("Z_rot"), tr("Z (unchanged)"), obs->satPos.z, "m"});
    t->beginFill(int(steps.size()));
    for (int i = 0; i < int(steps.size()); ++i) {
        const KeplerStep& s = steps[size_t(i)];
        t->setText(i, 0, s.symbol);
        t->setText(i, 1, s.description);
        t->setSci(i, 2, s.value, 15);
        t->setText(i, 3, s.units);
    }
    t->endFill();
    t->setSortingEnabled(false);
}

} // namespace gui
