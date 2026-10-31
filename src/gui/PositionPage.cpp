#include "PositionPage.h"
#include "ui_PositionPage.h"
#include "DataTable.h"
#include "PlotWidget.h"
#include "core/Constants.h"
#include "core/Geodesy.h"

#include <QDesktopServices>
#include <QFile>
#include <QFileDialog>
#include <QMessageBox>
#include <QTextStream>
#include <QUrl>
#include <cmath>

using namespace gnss;

namespace gui {

PositionPage::PositionPage(QWidget* parent) : PageBase(parent), ui(new Ui::PositionPage)
{
    ui->setupUi(this);
    ui->summarySplitter->setStretchFactor(0, 1);
    ui->summarySplitter->setStretchFactor(1, 4);
    ui->lsqSplitter->setStretchFactor(0, 1);
    ui->lsqSplitter->setStretchFactor(1, 1);
    for (QLabel* l : findChildren<QLabel*>())
        if (l->objectName().endsWith("Value")) {
            l->setWordWrap(true);
            l->setTextInteractionFlags(Qt::TextSelectableByMouse);
        }
    ui->solutionsTable->setColumns({tr("Epoch"), tr("Valid"), tr("GPS TOW (s)"), tr("Receiver clock (s)"), tr("Sats"),
                                    tr("Latitude (deg)"), tr("Longitude (deg)"), tr("Height (m)"), tr("X (m)"), tr("Y (m)"),
                                    tr("Z (m)"), tr("E (m)"), tr("N (m)"), tr("U (m)"), tr("Clock bias (m)"),
                                    tr("Clock drift (m/s)"), tr("GDOP"), tr("PDOP"), tr("HDOP"), tr("VDOP"), tr("TDOP"),
                                    tr("vE (m/s)"), tr("vN (m/s)"), tr("vU (m/s)"), tr("Residual RMS (m)"), tr("Iterations")});
    ui->iterTable->setColumns({tr("Iteration"), tr("Sats"), tr("X (m)"), tr("Y (m)"), tr("Z (m)"), tr("Bias (m)"),
                               tr("dX (m)"), tr("dY (m)"), tr("dZ (m)"), tr("dBias (m)"), tr("|dx| (m)"),
                               tr("Pre-fit residual RMS (m)"), tr("Latitude"), tr("Longitude"), tr("Height (m)")});
    ui->designTable->setColumns({tr("PRN"), tr("dr/dX"), tr("dr/dY"), tr("dr/dZ"), tr("dr/db"), tr("Residual (m)"),
                                 tr("Weight"), tr("Elevation"), tr("dr/d(Galileo bias)")});
    ui->cofactorTable->setColumns({QString(), tr("E"), tr("N"), tr("U"), tr("clock")});
    connect(ui->epochSpin, &QSpinBox::valueChanged, this, &PositionPage::showEpoch);
    connect(ui->exportCsvButton, &QPushButton::clicked, this, &PositionPage::exportCsv);
    connect(ui->exportKmlButton, &QPushButton::clicked, this, &PositionPage::exportKml);
    connect(ui->mapButton, &QPushButton::clicked, this, [this] {
        if (!readable(Receiver::StageSolved) || rx_->pvt.nValid == 0) return;
        const double lat = rx_->pvt.meanLlh.lat * 180.0 / kPi, lon = rx_->pvt.meanLlh.lon * 180.0 / kPi;
        QDesktopServices::openUrl(QUrl(QStringLiteral("https://www.openstreetmap.org/?mlat=%1&mlon=%2#map=17/%1/%2")
                                           .arg(lat, 0, 'f', 7).arg(lon, 0, 'f', 7)));
    });
    connect(ui->solutionsTable, &QTableWidget::cellDoubleClicked, this, [this](int r, int) {
        const QVariant k = ui->solutionsTable->keyOfRow(r);
        if (!k.isValid()) return;
        ui->epochSpin->setValue(k.toInt());
        ui->tabs->setCurrentIndex(2);
    });
}

PositionPage::~PositionPage()
{
    delete ui;
}

void PositionPage::refresh()
{
    for (PlotWidget* p : {ui->horizontalPlot, ui->enuPlot, ui->biasPlot, ui->driftPlot, ui->dopPlot, ui->satsPlot,
                          ui->velocityPlot, ui->convergencePlot})
        p->clear();
    for (DataTable* t : {ui->solutionsTable, ui->iterTable, ui->designTable, ui->cofactorTable}) {
        t->beginFill(0);
        t->endFill();
    }
    const bool ok = readable(Receiver::StageSolved) && rx_->pvt.nValid > 0;
    for (QPushButton* b : {ui->mapButton, ui->exportCsvButton, ui->exportKmlButton}) b->setEnabled(ok);
    fillSummary();
    if (!ok) return;
    fillPlots();
    fillSolutions();
    const int n = int(rx_->pvt.sols.size());
    setEpochRange(ui->epochSpin, ui->epochSlider, n);
    showEpoch();
}

void PositionPage::fillSummary()
{
    QList<QLabel*> vals = {ui->latValue, ui->lonValue, ui->heightValue, ui->ecefValue, ui->utmValue, ui->stdValue,
                           ui->cepValue, ui->dopValue, ui->velValue, ui->biasValue, ui->driftValue, ui->ifValue,
                           ui->signValue, ui->timeValue};
    if (!readable(Receiver::StageSolved) || rx_->pvt.nValid == 0) {
        const QString msg = readable(Receiver::StageDecoded) && !workerRunning() && !rx_->pvt.error.isEmpty()
                                ? rx_->pvt.error : QStringLiteral("-");
        for (QLabel* l : vals) l->setText(msg);
        return;
    }
    const PvtOutput& p = rx_->pvt;
    ui->latValue->setText(tr("%1° (%2)").arg(p.meanLlh.lat * 180 / kPi, 0, 'f', 8).arg(geo::formatDms(p.meanLlh.lat, true)));
    ui->lonValue->setText(tr("%1° (%2)").arg(p.meanLlh.lon * 180 / kPi, 0, 'f', 8).arg(geo::formatDms(p.meanLlh.lon, false)));
    ui->heightValue->setText(tr("%1 m").arg(p.meanLlh.h, 0, 'f', 2));
    ui->ecefValue->setText(tr("X %1\nY %2\nZ %3 m").arg(p.meanPos.x, 0, 'f', 2).arg(p.meanPos.y, 0, 'f', 2).arg(p.meanPos.z, 0, 'f', 2));
    ui->utmValue->setText(tr("%1%2  E %3  N %4").arg(p.utm.zone).arg(p.utm.band).arg(p.utm.easting, 0, 'f', 2).arg(p.utm.northing, 0, 'f', 2));
    ui->stdValue->setText(tr("%1 / %2 / %3 m").arg(p.stdE, 0, 'f', 2).arg(p.stdN, 0, 'f', 2).arg(p.stdU, 0, 'f', 2));
    ui->cepValue->setText(tr("%1 / %2 / %3 m").arg(p.cep50, 0, 'f', 2).arg(p.drms2, 0, 'f', 2).arg(p.sep50, 0, 'f', 2));
    ui->dopValue->setText(tr("%1 / %2 / %3").arg(p.meanSats, 0, 'f', 1).arg(p.meanPdop, 0, 'f', 2).arg(p.meanHdop, 0, 'f', 2));
    double ve = 0, vn = 0, vu = 0;
    int nv = 0;
    for (const auto& s : p.sols)
        if (s.velValid) { ve += s.velEnu.x; vn += s.velEnu.y; vu += s.velEnu.z; ++nv; }
    if (nv) ui->velValue->setText(tr("%1 / %2 / %3 m/s").arg(ve / nv, 0, 'f', 3).arg(vn / nv, 0, 'f', 3).arg(vu / nv, 0, 'f', 3));
    QString biasText = tr("%1 m/s = %2 ppb (sample clock frequency error)").arg(p.biasDrift, 0, 'f', 3)
                           .arg(p.biasDrift / kSpeedOfLight * 1e9, 0, 'f', 2);
    if (p.nGps > 0 && p.nGalileo > 0)
        biasText += tr("; Galileo-GPS bias %1 m (%2 GPS + %3 Galileo satellites)").arg(p.meanIsb, 0, 'f', 2).arg(p.nGps).arg(p.nGalileo);
    ui->biasValue->setText(biasText);
    ui->driftValue->setText(tr("%1 m/s").arg(p.meanDrift, 0, 'f', 3));
    ui->ifValue->setText(tr("%1 MHz (nominal %2 MHz, offset %3 Hz)").arg(p.effectiveIf / 1e6, 0, 'f', 6)
                             .arg(rx_->settings.ifFreq / 1e6, 0, 'f', 6).arg(p.commonFreqOffset, 0, 'f', 1));
    ui->signValue->setText(tr("%1 (%2; velocity RMS %3 / %4 m/s for normal / inverted)")
                               .arg(p.dopplerSign > 0 ? tr("normal") : tr("inverted")).arg(p.signAuto ? tr("auto") : tr("manual"))
                               .arg(p.signRms[0], 0, 'f', 3).arg(p.signRms[1], 0, 'f', 3));
    ui->timeValue->setText(tr("week %1 (mod 1024), TOW %2 s (%3)").arg(p.weekNumber).arg(p.firstGpsTime, 0, 'f', 3)
                               .arg(nav::towText(p.firstGpsTime)));
}

void PositionPage::fillPlots()
{
    const PvtOutput& p = rx_->pvt;
    QVector<double> t, e, n, u, b, d, gd, pd, hd, vd, td, ns, rms, ve, vn, vu, tv;
    for (const auto& s : p.sols) {
        if (!s.valid) continue;
        t << s.gpsTime;
        e << s.enu.x; n << s.enu.y; u << s.enu.z;
        b << s.bias;
        gd << s.gdop; pd << s.pdop; hd << s.hdop; vd << s.vdop; td << s.tdop;
        ns << s.nUsed;
        rms << s.rms;
        if (s.velValid) { tv << s.gpsTime; d << s.drift; ve << s.velEnu.x; vn << s.velEnu.y; vu << s.velEnu.z; }
    }
    PlotWidget* h = ui->horizontalPlot;
    h->addSeries(tr("fixes"), e, n, PlotWidget::Points, QColor(31, 119, 180));
    h->series(0).sorted = false;
    h->series(0).pointSize = 4;
    h->addCircle(0, 0, p.cep50, QColor(214, 39, 40), tr("CEP50 %1 m").arg(p.cep50, 0, 'f', 2));
    h->addCircle(0, 0, p.drms2, QColor(255, 127, 14), tr("2DRMS %1 m").arg(p.drms2, 0, 'f', 2));
    h->addHLine(0, QColor(127, 127, 127), QString(), Qt::DotLine);
    h->addVLine(0, QColor(127, 127, 127), QString(), Qt::DotLine);
    ui->enuPlot->addSeries(tr("East"), t, e, PlotWidget::LinesPoints, QColor(31, 119, 180), 1.2);
    ui->enuPlot->addSeries(tr("North"), t, n, PlotWidget::LinesPoints, QColor(44, 160, 44), 1.2);
    ui->enuPlot->addSeries(tr("Up"), t, u, PlotWidget::LinesPoints, QColor(214, 39, 40), 1.2);
    for (int i = 0; i < 3; ++i) ui->enuPlot->series(i).pointSize = 2;
    ui->biasPlot->addSeries(tr("bias"), t, b, PlotWidget::LinesPoints, QColor(148, 103, 189), 1.5);
    ui->biasPlot->series(0).pointSize = 2;
    if (!t.isEmpty()) {
        QVector<double> lt{t.first(), t.last()}, lb{b.first(), b.first() + p.biasDrift * (t.last() - t.first())};
        ui->biasPlot->addSeries(tr("fit: %1 m/s").arg(p.biasDrift, 0, 'f', 3), lt, lb, PlotWidget::Lines, QColor(255, 127, 14), 1.0);
    }
    ui->driftPlot->addSeries(tr("Doppler-derived drift"), tv, d, PlotWidget::LinesPoints, QColor(140, 86, 75), 1.5);
    ui->driftPlot->series(0).pointSize = 2;
    ui->dopPlot->addSeries(tr("GDOP"), t, gd, PlotWidget::Lines, QColor(0, 0, 0), 1.5);
    ui->dopPlot->addSeries(tr("PDOP"), t, pd, PlotWidget::Lines, QColor(31, 119, 180), 1.5);
    ui->dopPlot->addSeries(tr("HDOP"), t, hd, PlotWidget::Lines, QColor(44, 160, 44), 1.5);
    ui->dopPlot->addSeries(tr("VDOP"), t, vd, PlotWidget::Lines, QColor(214, 39, 40), 1.5);
    ui->dopPlot->addSeries(tr("TDOP"), t, td, PlotWidget::Lines, QColor(148, 103, 189), 1.5);
    ui->dopPlot->setYIncludesZero(true);
    ui->dopPlot->autoScale();
    ui->satsPlot->addSeries(tr("satellites used"), t, ns, PlotWidget::Steps, QColor(31, 119, 180), 1.5);
    ui->satsPlot->addSeries(tr("residual RMS (m)"), t, rms, PlotWidget::Lines, QColor(214, 39, 40), 1.2);
    ui->satsPlot->setYIncludesZero(true);
    ui->satsPlot->autoScale();
    ui->velocityPlot->addSeries(tr("vE"), tv, ve, PlotWidget::LinesPoints, QColor(31, 119, 180), 1.2);
    ui->velocityPlot->addSeries(tr("vN"), tv, vn, PlotWidget::LinesPoints, QColor(44, 160, 44), 1.2);
    ui->velocityPlot->addSeries(tr("vU"), tv, vu, PlotWidget::LinesPoints, QColor(214, 39, 40), 1.2);
    for (int i = 0; i < 3; ++i) ui->velocityPlot->series(i).pointSize = 2;
}

void PositionPage::fillSolutions()
{
    DataTable* t = ui->solutionsTable;
    const auto& sols = rx_->pvt.sols;
    t->beginFill(int(sols.size()));
    for (int r = 0; r < int(sols.size()); ++r) {
        const NavSolution& s = sols[size_t(r)];
        t->setNum(r, 0, s.index, 0);
        t->setKey(r, r);
        t->setText(r, 1, s.valid ? tr("yes") : s.message, s.valid ? goodColor() : badColor());
        t->setNum(r, 3, s.rxTime, 6);
        if (!s.valid) continue;
        t->setNum(r, 2, s.gpsTime, 6);
        t->setNum(r, 4, s.nUsed, 0);
        t->setNum(r, 5, s.llh.lat * 180 / kPi, 9);
        t->setNum(r, 6, s.llh.lon * 180 / kPi, 9);
        t->setNum(r, 7, s.llh.h, 3);
        t->setNum(r, 8, s.pos.x, 3);
        t->setNum(r, 9, s.pos.y, 3);
        t->setNum(r, 10, s.pos.z, 3);
        t->setNum(r, 11, s.enu.x, 3);
        t->setNum(r, 12, s.enu.y, 3);
        t->setNum(r, 13, s.enu.z, 3);
        t->setNum(r, 14, s.bias, 3);
        if (s.velValid) t->setNum(r, 15, s.drift, 4);
        t->setNum(r, 16, s.gdop, 3);
        t->setNum(r, 17, s.pdop, 3);
        t->setNum(r, 18, s.hdop, 3);
        t->setNum(r, 19, s.vdop, 3);
        t->setNum(r, 20, s.tdop, 3);
        if (s.velValid) {
            t->setNum(r, 21, s.velEnu.x, 4);
            t->setNum(r, 22, s.velEnu.y, 4);
            t->setNum(r, 23, s.velEnu.z, 4);
        }
        t->setNum(r, 24, s.rms, 3);
        t->setNum(r, 25, double(s.iterations.size()), 0);
    }
    t->endFill();
}

void PositionPage::showEpoch()
{
    for (DataTable* t : {ui->iterTable, ui->designTable, ui->cofactorTable}) {
        t->beginFill(0);
        t->endFill();
    }
    ui->convergencePlot->clear();
    if (!readable(Receiver::StageSolved)) return;
    const int i = ui->epochSpin->value();
    if (i < 0 || i >= int(rx_->pvt.sols.size())) return;
    const NavSolution& s = rx_->pvt.sols[size_t(i)];
    ui->epochInfoLabel->setText(s.valid ? tr("GPS TOW %1 s, %2 satellites, %3 iterations, GDOP %4")
                                              .arg(s.gpsTime, 0, 'f', 4).arg(s.nUsed).arg(s.iterations.size()).arg(s.gdop, 0, 'f', 3)
                                        : tr("invalid: %1").arg(s.message));

    DataTable* it = ui->iterTable;
    it->beginFill(int(s.iterations.size()));
    QVector<double> cx, cy;
    for (int r = 0; r < int(s.iterations.size()); ++r) {
        const LsqIteration& li = s.iterations[size_t(r)];
        it->setNum(r, 0, li.iter, 0);
        it->setNum(r, 1, li.nSats, 0);
        it->setNum(r, 2, li.pos.x, 4);
        it->setNum(r, 3, li.pos.y, 4);
        it->setNum(r, 4, li.pos.z, 4);
        it->setNum(r, 5, li.bias, 4);
        it->setSci(r, 6, li.dpos.x, 6);
        it->setSci(r, 7, li.dpos.y, 6);
        it->setSci(r, 8, li.dpos.z, 6);
        it->setSci(r, 9, li.dbias, 6);
        it->setSci(r, 10, li.stepNorm, 6);
        it->setSci(r, 11, li.rms, 6);
        if (li.pos.norm() > 6e6) {
            const Geodetic g = geo::ecefToGeodetic(li.pos);
            it->setNum(r, 12, g.lat * 180 / kPi, 8);
            it->setNum(r, 13, g.lon * 180 / kPi, 8);
            it->setNum(r, 14, g.h, 3);
        }
        cx << li.iter;
        cy << std::log10(std::max(1e-12, li.stepNorm));
    }
    it->endFill();
    it->setSortingEnabled(false);
    ui->convergencePlot->addSeries(tr("log10 |dx|"), cx, cy, PlotWidget::LinesPoints, QColor(31, 119, 180), 1.5);
    ui->convergencePlot->addHLine(-4, QColor(214, 39, 40), tr("convergence threshold 0.1 mm"));
    if (!s.valid) return;

    DataTable* dt = ui->designTable;
    int n = 0;
    for (const auto& o : s.obs) n += o.used ? 1 : 0;
    dt->beginFill(n);
    int r = 0;
    for (const auto& o : s.obs) {
        if (!o.used) continue;
        const Vec3 u = (s.pos - o.satPos) / (s.pos - o.satPos).norm();
        dt->setText(r, 0, satLabel(o.prn));
        dt->setNum(r, 1, u.x, 6);
        dt->setNum(r, 2, u.y, 6);
        dt->setNum(r, 3, u.z, 6);
        dt->setNum(r, 4, 1.0, 1);
        dt->setNum(r, 5, o.residual, 3);
        dt->setNum(r, 6, o.weight, 3);
        dt->setNum(r, 7, o.el * 180 / kPi, 2);
        dt->setNum(r, 8, isGalileo(o.prn) && rx_->pvt.nGps > 0 ? 1.0 : 0.0, 1);
        ++r;
    }
    dt->endFill();

    DataTable* ct = ui->cofactorTable;
    const char* names[4] = {"E", "N", "U", "clock"};
    ct->beginFill(4);
    for (int a = 0; a < 4; ++a) {
        ct->setText(a, 0, QLatin1String(names[a]));
        for (int b = 0; b < 4; ++b)
            ct->setNum(a, 1 + b, s.qEnu[a][b], 4, a == b ? QColor(220, 235, 255) : QColor());
    }
    ct->endFill();
    ct->setSortingEnabled(false);
}

void PositionPage::exportCsv()
{
    if (!readable(Receiver::StageSolved)) return;
    const QString fn = QFileDialog::getSaveFileName(this, tr("Export solutions"), QStringLiteral("solutions.csv"), tr("CSV files (*.csv)"));
    if (fn.isEmpty()) return;
    QFile f(fn);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QMessageBox::warning(this, tr("Export"), f.errorString());
        return;
    }
    f.write(ui->solutionsTable->toCsv(false).toUtf8());
}

void PositionPage::exportKml()
{
    if (!readable(Receiver::StageSolved)) return;
    const QString fn = QFileDialog::getSaveFileName(this, tr("Export KML"), QStringLiteral("track.kml"), tr("KML files (*.kml)"));
    if (fn.isEmpty()) return;
    QFile f(fn);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QMessageBox::warning(this, tr("Export"), f.errorString());
        return;
    }
    const PvtOutput& p = rx_->pvt;
    QTextStream ts(&f);
    ts << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<kml xmlns=\"http://www.opengis.net/kml/2.2\">\n<Document>\n"
       << "<name>Software GPS receiver solutions</name>\n"
       << "<Placemark><name>Mean position</name><Point><coordinates>"
       << QString::number(p.meanLlh.lon * 180 / kPi, 'f', 9) << ',' << QString::number(p.meanLlh.lat * 180 / kPi, 'f', 9)
       << ',' << QString::number(p.meanLlh.h, 'f', 2) << "</coordinates></Point></Placemark>\n";
    for (const auto& s : p.sols) {
        if (!s.valid) continue;
        ts << "<Placemark><name>" << s.index << "</name><description>TOW " << QString::number(s.gpsTime, 'f', 3)
           << " s, PDOP " << QString::number(s.pdop, 'f', 2) << "</description><Point><coordinates>"
           << QString::number(s.llh.lon * 180 / kPi, 'f', 9) << ',' << QString::number(s.llh.lat * 180 / kPi, 'f', 9) << ','
           << QString::number(s.llh.h, 'f', 2) << "</coordinates></Point></Placemark>\n";
    }
    ts << "</Document>\n</kml>\n";
}

} // namespace gui
