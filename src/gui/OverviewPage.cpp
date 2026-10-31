#include "OverviewPage.h"
#include "ui_OverviewPage.h"
#include "DataTable.h"
#include "core/Constants.h"
#include "core/Geodesy.h"

#include <QDesktopServices>
#include <QDir>
#include <QFile>
#include <QLabel>
#include <QUrl>
#include <cmath>

using namespace gnss;

namespace gui {

OverviewPage::OverviewPage(QWidget* parent) : PageBase(parent), ui(new Ui::OverviewPage)
{
    ui->setupUi(this);
    ui->splitter->setStretchFactor(0, 3);
    ui->splitter->setStretchFactor(1, 2);
    for (QLabel* l : findChildren<QLabel*>())
        if (l->objectName().endsWith("Value")) {
            l->setWordWrap(true);
            l->setTextInteractionFlags(Qt::TextSelectableByMouse);
        }
    ui->pipelineTable->setColumns({tr("Stage"), tr("Status"), tr("Time (s)"), tr("Result")});
    ui->satellitesTable->setColumns({tr("PRN"), tr("System"), tr("Channel"), tr("Acq metric"), tr("Acq C/N0"), tr("Doppler (Hz)"),
                                     tr("Code phase (chips)"), tr("Track C/N0"), tr("Phase lock %"), tr("Bit sync"),
                                     tr("Subframes OK"), tr("Words parity OK"), tr("Ephemeris"), tr("SV health"), tr("URA"),
                                     tr("Elevation"), tr("Azimuth"), tr("Used in fix")});
    connect(ui->satellitesTable, &QTableWidget::cellDoubleClicked, this, [this](int r, int) {
        const QVariant k = ui->satellitesTable->keyOfRow(r);
        if (k.isValid()) emit prnSelected(k.toInt());
    });
    connect(ui->mapButton, &QPushButton::clicked, this, [this] {
        if (!readable(Receiver::StageSolved) || rx_->pvt.nValid == 0) return;
        const double lat = rx_->pvt.meanLlh.lat * 180.0 / kPi, lon = rx_->pvt.meanLlh.lon * 180.0 / kPi;
        QDesktopServices::openUrl(QUrl(QStringLiteral("https://www.openstreetmap.org/?mlat=%1&mlon=%2#map=17/%1/%2")
                                           .arg(lat, 0, 'f', 7).arg(lon, 0, 'f', 7)));
    });
    QFile f(QStringLiteral(":/docs/algorithms.html"));
    if (f.open(QIODevice::ReadOnly)) ui->docBrowser->setHtml(QString::fromUtf8(f.readAll()));
}

OverviewPage::~OverviewPage()
{
    delete ui;
}

void OverviewPage::clearPosition(const QString& text)
{
    for (QLabel* l : {ui->latValue, ui->lonValue, ui->heightValue, ui->ecefValue, ui->utmValue, ui->fixesValue, ui->stdValue,
                      ui->cepValue, ui->dopValue, ui->timeValue, ui->dateValue, ui->clockValue, ui->ifCheckValue})
        l->setText(text);
}

void OverviewPage::refresh()
{
    if (!rx_) return;
    const ReceiverSettings& s = rx_->settings;
    const bool open = rx_->source.isOpen();
    const bool stream = !rx_->liveDescription.isEmpty();
    ui->fileValue->setText(stream ? rx_->liveDescription
                                  : open ? QDir::toNativeSeparators(s.fileName) : tr("(no file open - use File > Open)"));
    ui->sizeValue->setText(rx_->source.isLive() ? tr("%1 samples so far").arg(rx_->source.numSamples())
                           : open ? tr("%1 bytes (%2 MB), %3 samples").arg(rx_->source.fileSize())
                                        .arg(rx_->source.fileSize() / 1048576.0, 0, 'f', 1).arg(rx_->source.numSamples())
                                  : QStringLiteral("-"));
    if (!open) {
        for (QLabel* l : {ui->formatValue, ui->fsValue, ui->ifValue, ui->durationValue, ui->spmsValue, ui->spcValue, ui->ifPlacementValue})
            l->setText(QStringLiteral("-"));
    } else {
        ui->formatValue->setText(sampleFormatName(s.format));
        ui->fsValue->setText(tr("%1 MHz").arg(s.samplingFreq / 1e6, 0, 'f', 6));
        ui->ifValue->setText(tr("%1 MHz").arg(s.ifFreq / 1e6, 0, 'f', 6));
        ui->durationValue->setText(tr("%1 s").arg(rx_->fileDurationSec(), 0, 'f', 3));
        ui->spmsValue->setText(tr("%1 (exact %2)").arg(s.samplesPerCode()).arg(s.samplingFreq / 1000.0, 0, 'f', 3));
        ui->spcValue->setText(QString::number(s.samplingFreq / kCaChipRate, 'f', 4));
        const double ratio = s.ifFreq / s.samplingFreq;
        QString note = tr("IF/fs = %1").arg(ratio, 0, 'f', 5);
        if (std::fabs(ratio - 0.25) < 0.001) note += tr(" (close to fs/4 = %1 MHz)").arg(s.samplingFreq / 4e6, 0, 'f', 6);
        if (s.ifFreq > s.samplingFreq / 2 && !s.isComplex()) note += tr(" - band-pass sampled (IF above Nyquist)");
        ui->ifPlacementValue->setText(note);
    }

    // pipeline
    const int st = readableStage();
    struct Row { const char* name; int stage; };
    const Row rows[4] = {{"Acquisition", Receiver::StageAcquired}, {"Tracking", Receiver::StageTracked},
                         {"Bit sync & navigation message", Receiver::StageDecoded},
                         {"Position / velocity / time", Receiver::StageSolved}};
    DataTable* pt = ui->pipelineTable;
    pt->beginFill(4);
    for (int i = 0; i < 4; ++i) {
        const bool done = st >= rows[i].stage;
        pt->setText(i, 0, tr(rows[i].name));
        pt->setText(i, 1, done ? tr("Done") : tr("Pending"), done ? goodColor() : neutralColor());
        pt->setNum(i, 2, done ? rx_->stageSeconds(rows[i].stage) : NAN, 2);
        QString res;
        if (done && rows[i].stage == Receiver::StageAcquired) {
            int gps = 0, galileo = 0;
            for (const auto& r : rx_->acq.results)
                if (r.acquired) ++(isGalileo(r.prn) ? galileo : gps);
            res = tr("%1 GPS and %2 Galileo satellites acquired, of %3 searched").arg(gps).arg(galileo).arg(rx_->acq.results.size());
        } else if (done && rows[i].stage == Receiver::StageTracked) {
            res = tr("%1 channels tracked").arg(rx_->trk.size());
        } else if (done && rows[i].stage == Receiver::StageDecoded) {
            int eph = 0, sf = 0, words = 0;
            for (const auto& n : rx_->nav) { eph += n.eph.complete() ? 1 : 0; sf += int(n.subframes.size()); words += int(n.inavWords.size()); }
            res = words ? tr("%1 subframes and %2 I/NAV words decoded, %3 complete ephemerides").arg(sf).arg(words).arg(eph)
                        : tr("%1 subframes decoded, %2 complete ephemerides").arg(sf).arg(eph);
        } else if (done && rows[i].stage == Receiver::StageSolved) {
            res = rx_->pvt.nGalileo > 0 && rx_->pvt.nGps > 0
                      ? tr("%1 valid fixes from %2 GPS and %3 Galileo satellites").arg(rx_->pvt.nValid).arg(rx_->pvt.nGps).arg(rx_->pvt.nGalileo)
                      : tr("%1 valid fixes").arg(rx_->pvt.nValid);
        } else if (!done && rows[i].stage == Receiver::StageSolved && st == Receiver::StageDecoded && !workerRunning()
                   && !rx_->pvt.error.isEmpty()) {
            res = rx_->pvt.error;
        }
        pt->setText(i, 3, res);
    }
    pt->endFill();

    // satellites
    DataTable* t = ui->satellitesTable;
    std::vector<const AcqResult*> list;
    if (st >= Receiver::StageAcquired)
        for (const auto& r : rx_->acq.results)
            if (r.acquired) list.push_back(&r);
    const NavSolution* lastSol = nullptr;
    if (st >= Receiver::StageSolved)
        for (const auto& sol : rx_->pvt.sols)
            if (sol.valid) lastSol = &sol;
    t->beginFill(int(list.size()));
    for (int i = 0; i < int(list.size()); ++i) {
        const AcqResult& a = *list[size_t(i)];
        t->setText(i, 0, satLabel(a.prn));
        t->setKey(i, a.prn);
        t->setText(i, 1, satSystem(a.prn));
        t->setNum(i, 3, a.metric, 2);
        t->setNum(i, 4, a.cn0, 1);
        t->setNum(i, 5, a.doppler, 1);
        t->setNum(i, 6, a.codePhaseChips, 2);
        const int ch = st >= Receiver::StageTracked ? rx_->channelOfPrn(a.prn) : -1;
        t->setText(i, 2, ch >= 0 ? QString::number(ch) : (st >= Receiver::StageTracked ? tr("no free channel") : QString()));
        if (ch < 0) continue;
        const TrackingChannel& tc = rx_->trk[size_t(ch)];
        t->setNum(i, 7, tc.meanCn0, 1, tc.meanCn0 > 35 ? goodColor() : warnColor());
        t->setNum(i, 8, tc.lockFraction * 100, 1, tc.lockFraction > 0.9 ? goodColor() : badColor());
        if (st < Receiver::StageDecoded || ch >= int(rx_->nav.size())) continue;
        const NavChannel& n = rx_->nav[size_t(ch)];
        const bool gal = isGalileo(n.prn);
        t->setText(i, 9, n.bitSyncOk ? (gal ? tr("page sync ok") : tr("ok (%1 ms)").arg(n.bitOffset)) : tr("failed"), n.bitSyncOk ? goodColor() : badColor());
        int ok = 0;
        for (const auto& sf : n.subframes) ok += sf.allParityOk() ? 1 : 0;
        t->setText(i, 10, gal ? QStringLiteral("-") : QStringLiteral("%1 / %2").arg(ok).arg(n.subframes.size()));
        t->setText(i, 11, QStringLiteral("%1 / %2").arg(n.parityOkWords).arg(n.parityTotalWords),
                   n.parityOkWords == n.parityTotalWords && n.parityTotalWords > 0 ? goodColor() : warnColor());
        t->setText(i, 12, n.eph.complete() ? (n.eph.consistent() ? tr("complete") : tr("IODE mismatch")) : tr("incomplete"),
                   n.eph.complete() ? goodColor() : badColor());
        if (n.eph.sf1) {
            t->setText(i, 13, QString::number(n.eph.svHealth), n.eph.svHealth == 0 ? goodColor() : badColor(),
                       gal ? QString() : nav::health6Text(n.eph.svHealth));
            t->setText(i, 14, gal ? tr("SISA %1").arg(n.eph.uraIndex) : nav::uraText(n.eph.uraIndex));
        }
        if (lastSol)
            for (const auto& o : lastSol->obs) {
                if (o.prn != a.prn) continue;
                if (o.geomRange > 0) {
                    t->setNum(i, 15, o.el * 180.0 / kPi, 1);
                    t->setNum(i, 16, o.az * 180.0 / kPi, 1);
                }
                t->setText(i, 17, o.used ? tr("yes") : o.status, o.used ? goodColor() : warnColor());
            }
    }
    t->endFill();

    // position
    const bool solved = st >= Receiver::StageSolved && rx_->pvt.nValid > 0;
    ui->mapButton->setEnabled(solved);
    if (!solved) {
        clearPosition(!workerRunning() && st >= Receiver::StageDecoded && !rx_->pvt.error.isEmpty() ? rx_->pvt.error
                                                                                                   : QStringLiteral("-"));
        return;
    }
    const PvtOutput& p = rx_->pvt;
    ui->latValue->setText(tr("%1°  (%2)").arg(p.meanLlh.lat * 180 / kPi, 0, 'f', 8).arg(geo::formatDms(p.meanLlh.lat, true)));
    ui->lonValue->setText(tr("%1°  (%2)").arg(p.meanLlh.lon * 180 / kPi, 0, 'f', 8).arg(geo::formatDms(p.meanLlh.lon, false)));
    ui->heightValue->setText(tr("%1 m (WGS-84 ellipsoid)").arg(p.meanLlh.h, 0, 'f', 2));
    ui->ecefValue->setText(tr("%1, %2, %3 m").arg(p.meanPos.x, 0, 'f', 2).arg(p.meanPos.y, 0, 'f', 2).arg(p.meanPos.z, 0, 'f', 2));
    ui->utmValue->setText(tr("zone %1%2  E %3 m  N %4 m").arg(p.utm.zone).arg(p.utm.band)
                              .arg(p.utm.easting, 0, 'f', 2).arg(p.utm.northing, 0, 'f', 2));
    ui->fixesValue->setText(tr("%1 valid of %2 epochs (%3 ms period)").arg(p.nValid).arg(p.sols.size()).arg(s.navSolPeriodMs));
    ui->stdValue->setText(tr("%1 / %2 / %3 m").arg(p.stdE, 0, 'f', 2).arg(p.stdN, 0, 'f', 2).arg(p.stdU, 0, 'f', 2));
    ui->cepValue->setText(tr("%1 / %2 / %3 m").arg(p.cep50, 0, 'f', 2).arg(p.drms2, 0, 'f', 2).arg(p.sep50, 0, 'f', 2));
    ui->dopValue->setText(tr("%1 / %2 / %3").arg(p.meanSats, 0, 'f', 1).arg(p.meanPdop, 0, 'f', 2).arg(p.meanHdop, 0, 'f', 2));
    ui->timeValue->setText(tr("WN %1, first fix at TOW %2 s (%3)").arg(p.weekNumber).arg(p.firstGpsTime, 0, 'f', 3)
                               .arg(nav::towText(p.firstGpsTime)));
    QStringList dates;
    for (int k = 0; k < 3; ++k) {
        const QDateTime dt = geo::gpsToDateTime(p.weekNumber + 1024 * k, p.firstGpsTime);
        const int ls = geo::leapSeconds(dt);
        dates << tr("week %1: %2 UTC").arg(p.weekNumber + 1024 * k).arg(dt.addSecs(-ls).toString("yyyy-MM-dd hh:mm:ss"));
    }
    ui->dateValue->setText(dates.join("\n"));
    ui->clockValue->setText(tr("bias drift from pseudoranges %1 m/s (%2 ppb of the sample clock); Doppler clock drift %3 m/s")
                                .arg(p.biasDrift, 0, 'f', 3).arg(p.biasDrift / kSpeedOfLight * 1e9, 0, 'f', 2)
                                .arg(p.meanDrift, 0, 'f', 2));
    ui->ifCheckValue->setText(tr("common carrier offset %1 Hz -> effective IF %2 MHz; Doppler sign %3")
                                  .arg(p.commonFreqOffset, 0, 'f', 1).arg(p.effectiveIf / 1e6, 0, 'f', 6)
                                  .arg(p.dopplerSign > 0 ? tr("normal") : tr("inverted")));
}

} // namespace gui
