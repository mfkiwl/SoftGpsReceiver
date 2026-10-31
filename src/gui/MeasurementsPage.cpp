#include "MeasurementsPage.h"
#include "ui_MeasurementsPage.h"
#include "DataTable.h"
#include "PlotWidget.h"
#include "core/Constants.h"

#include <cmath>
#include <map>

using namespace gnss;

namespace gui {

MeasurementsPage::MeasurementsPage(QWidget* parent) : PageBase(parent), ui(new Ui::MeasurementsPage)
{
    ui->setupUi(this);
    ui->splitter->setStretchFactor(0, 2);
    ui->splitter->setStretchFactor(1, 3);
    ui->obsTable->setColumns({tr("PRN"), tr("Channel"), tr("Status"), tr("C/N0"), tr("Elevation"), tr("Azimuth"),
                              tr("t_tx satellite (s)"), tr("Raw PR (m)"), tr("c*dt_sv (m)"), tr("of which relativistic (m)"),
                              tr("of which -T_GD (m)"), tr("Iono (m)"), tr("Tropo (m)"), tr("Earth rotation (m)"),
                              tr("Corrected PR (m)"), tr("Geometric range (m)"), tr("Residual (m)"), tr("Weight"),
                              tr("Carrier NCO (Hz)"), tr("Doppler meas. - common offset (Hz)"), tr("Doppler pred. (Hz)"), tr("PR rate (m/s)"),
                              tr("Range rate pred. (m/s)"), tr("Rate residual (m/s)"), tr("ADR (cycles)")});
    connect(ui->epochSpin, &QSpinBox::valueChanged, this, &MeasurementsPage::showEpoch);
}

MeasurementsPage::~MeasurementsPage()
{
    delete ui;
}

void MeasurementsPage::refresh()
{
    ui->obsTable->beginFill(0);
    ui->obsTable->endFill();
    for (PlotWidget* p : {ui->prPlot, ui->residualPlot, ui->cmcPlot, ui->dopplerPlot, ui->tropoPlot, ui->clockPlot}) p->clear();
    if (!readable(Receiver::StageSolved) || rx_->pvt.sols.empty()) {
        ui->epochInfoLabel->setText(tr("(no position solution)"));
        return;
    }
    const int n = int(rx_->pvt.sols.size());
    setEpochRange(ui->epochSpin, ui->epochSlider, n);
    fillPlots();
    showEpoch();
}

void MeasurementsPage::fillPlots()
{
    const PvtOutput& p = rx_->pvt;
    const double sign = p.dopplerSign;
    std::map<int, QVector<double>> t, pr, res, cmc, dm, dp, tdp, tro, clk;
    std::map<int, double> pr0, cmc0;
    // code-minus-carrier: rawPr - lambda * (-sign * adr); common mode removed per epoch
    std::vector<double> epochTimes;
    std::vector<std::map<int, double>> cmcEpoch;
    for (const auto& s : p.sols) {
        if (!s.valid) continue;
        std::map<int, double> ce;
        for (const auto& o : s.obs) {
            if (!o.available || o.geomRange <= 0) continue;
            t[o.prn] << s.gpsTime;
            if (!pr0.count(o.prn)) pr0[o.prn] = o.rawPr;
            pr[o.prn] << o.rawPr - pr0[o.prn];
            if (o.used) {
                res[o.prn] << o.residual;
            } else {
                res[o.prn] << NAN;
            }
            tro[o.prn] << o.tropo;
            clk[o.prn] << o.satClock * kSpeedOfLight / 1000.0;
            dm[o.prn] << sign * (o.doppler - p.commonFreqOffset);
            tdp[o.prn] << s.gpsTime;
            dp[o.prn] << (s.velValid ? o.predDoppler : NAN);
            ce[o.prn] = o.rawPr - kL1Wavelength * (-sign * o.adrCycles);
        }
        epochTimes.push_back(s.gpsTime);
        cmcEpoch.push_back(ce);
    }
    std::map<int, QVector<double>> ct;
    for (size_t e = 0; e < cmcEpoch.size(); ++e) {
        for (const auto& kv : cmcEpoch[e])
            if (!cmc0.count(kv.first)) cmc0[kv.first] = kv.second;
        double mean = 0;
        int cnt = 0;
        for (const auto& kv : cmcEpoch[e]) { mean += kv.second - cmc0[kv.first]; ++cnt; }
        mean = cnt ? mean / cnt : 0;
        for (const auto& kv : cmcEpoch[e]) {
            ct[kv.first] << epochTimes[e];
            cmc[kv.first] << kv.second - cmc0[kv.first] - mean;
        }
    }
    for (const auto& kv : t) {
        const int prn = kv.first;
        const QColor c = prnColor(prn);
        const QString name = satName(prn);
        ui->prPlot->addSeries(name, kv.second, pr[prn], PlotWidget::Lines, c, 1.5);
        ui->residualPlot->addSeries(name, kv.second, res[prn], PlotWidget::LinesPoints, c, 1.2);
        ui->residualPlot->series(ui->residualPlot->seriesCount() - 1).pointSize = 2;
        ui->cmcPlot->addSeries(name, ct[prn], cmc[prn], PlotWidget::Lines, c, 1.2);
        ui->dopplerPlot->addSeries(name, kv.second, dm[prn], PlotWidget::Lines, c, 1.5);
        ui->dopplerPlot->addSeries(QString(), tdp[prn], dp[prn], PlotWidget::Points, c, 1.0);
        ui->dopplerPlot->series(ui->dopplerPlot->seriesCount() - 1).legend = false;
        ui->tropoPlot->addSeries(name, kv.second, tro[prn], PlotWidget::Lines, c, 1.5);
        ui->clockPlot->addSeries(name, kv.second, clk[prn], PlotWidget::Lines, c, 1.5);
    }
}

void MeasurementsPage::showEpoch()
{
    DataTable* t = ui->obsTable;
    t->beginFill(0);
    t->endFill();
    if (!readable(Receiver::StageSolved)) return;
    const int i = ui->epochSpin->value();
    if (i < 0 || i >= int(rx_->pvt.sols.size())) return;
    const NavSolution& s = rx_->pvt.sols[size_t(i)];
    ui->epochInfoLabel->setText(s.valid ? tr("receiver sample %1, receiver clock %2 s, GPS TOW %3 s, clock bias %4 m (%5 us), %6 used, residual RMS %7 m")
                                              .arg(s.rxSample, 0, 'f', 1).arg(s.rxTime, 0, 'f', 6).arg(s.gpsTime, 0, 'f', 6)
                                              .arg(s.bias, 0, 'f', 2).arg(s.bias / kSpeedOfLight * 1e6, 0, 'f', 4)
                                              .arg(s.nUsed).arg(s.rms, 0, 'f', 2)
                                        : tr("invalid epoch: %1").arg(s.message));
    t->beginFill(int(s.obs.size()));
    const double c = kSpeedOfLight;
    for (int r = 0; r < int(s.obs.size()); ++r) {
        const SatObservation& o = s.obs[size_t(r)];
        t->setText(r, 0, satLabel(o.prn));
        t->setNum(r, 1, o.channel, 0);
        t->setText(r, 2, o.used ? tr("used") : o.status, o.used ? goodColor() : warnColor());
        t->setNum(r, 3, o.cn0, 1);
        if (o.geomRange > 0) {
            t->setNum(r, 4, o.el * 180 / kPi, 2);
            t->setNum(r, 5, o.az * 180 / kPi, 2);
        }
        t->setNum(r, 6, o.txTime, 9);
        t->setNum(r, 7, o.rawPr, 3);
        t->setNum(r, 8, c * o.satClock, 3);
        t->setNum(r, 9, c * o.satClockRel, 3);
        t->setNum(r, 10, -c * o.tgd, 3);
        t->setNum(r, 11, o.iono, 3);
        t->setNum(r, 12, o.tropo, 3);
        t->setNum(r, 13, o.sagnac, 3);
        t->setNum(r, 14, o.correctedPr, 3);
        t->setNum(r, 15, o.geomRange, 3);
        if (o.used) t->setNum(r, 16, o.residual, 3, std::fabs(o.residual) > 10 ? badColor() : QColor());
        t->setNum(r, 17, o.weight, 3);
        t->setNum(r, 18, o.carrFreq, 3);
        t->setNum(r, 19, rx_->pvt.dopplerSign * (o.doppler - rx_->pvt.commonFreqOffset), 3);
        if (s.velValid && o.used) {
            t->setNum(r, 20, o.predDoppler, 3);
            t->setNum(r, 21, o.prRate, 3);
            t->setNum(r, 22, o.predRangeRate, 3);
            t->setNum(r, 23, o.rateResidual, 4);
        }
        t->setNum(r, 24, o.adrCycles, 3);
    }
    t->endFill();
}

} // namespace gui
