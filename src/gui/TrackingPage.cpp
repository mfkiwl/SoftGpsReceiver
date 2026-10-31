#include "TrackingPage.h"
#include "ui_TrackingPage.h"
#include "DataTable.h"
#include "PlotWidget.h"
#include "core/Constants.h"

#include <QApplication>
#include <cmath>

using namespace gnss;

namespace gui {

TrackingPage::TrackingPage(QWidget* parent) : PageBase(parent), ui(new Ui::TrackingPage)
{
    ui->setupUi(this);
    ui->pliPlot->setFixedYRange(-1.05, 1.1);
    ui->allPliPlot->setFixedYRange(-1.05, 1.1);
    ui->shapeSplitter->setStretchFactor(0, 4);
    ui->shapeSplitter->setStretchFactor(1, 1);
    timePlots_ = {ui->promptPlot, ui->eplPlot, ui->pllPlot, ui->dopplerPlot, ui->fllPlot, ui->dllPlot,
                  ui->codePlot, ui->cn0Plot, ui->pliPlot, ui->adrPlot, ui->driftPlot};
    linkPlots();

    ui->shapeTable->setColumns({tr("Offset (chips)"), tr("|R|"), tr("I"), tr("Q")});
    ui->summaryTable->setColumns({tr("Channel"), tr("PRN"), tr("Status"), tr("Start sample"), tr("ms tracked"),
                                  tr("Initial freq (Hz)"), tr("Initial Doppler (Hz)"), tr("Final Doppler (Hz)"),
                                  tr("Doppler rate (Hz/s)"), tr("Mean C/N0"), tr("Min C/N0"), tr("Phase lock %"),
                                  tr("RMS PLL error (deg)"), tr("RMS DLL error (chips)"), tr("Processing time (s)")});
    ui->loopTable->setColumns({tr("Loop"), tr("Parameter"), tr("Value"), tr("Comment")});

    connect(ui->computeButton, &QPushButton::clicked, this, &TrackingPage::computeShape);
    connect(ui->shapeMsSlider, &QSlider::sliderReleased, this, &TrackingPage::computeShape);
    connect(ui->shapeMsSpin, &QSpinBox::editingFinished, this, &TrackingPage::computeShape);
    connect(ui->shapeChannelCombo, &QComboBox::currentIndexChanged, this, [this](int i) {
        if (!readable(Receiver::StageTracked) || i < 0 || i >= int(rx_->trk.size())) return;
        const int n = rx_->trk[size_t(i)].numMs * rx_->trk[size_t(i)].periodMs;
        ui->shapeMsSlider->setRange(0, std::max(0, n - 1));
        ui->shapeMsSpin->setRange(0, std::max(0, n - 1));
    });
    connect(ui->summaryTable, &QTableWidget::cellDoubleClicked, this, [this](int r, int) {
        const QVariant k = ui->summaryTable->keyOfRow(r);
        if (!k.isValid()) return;
        ui->channelCombo->setCurrentIndex(k.toInt());
        ui->tabs->setCurrentIndex(0);
    });
    connect(ui->channelCombo, &QComboBox::currentIndexChanged, this, &TrackingPage::showChannel);
    connect(ui->skipSpin, &QSpinBox::valueChanged, this, &TrackingPage::showChannel);
}

TrackingPage::~TrackingPage()
{
    delete ui;
}

QVector<PlotWidget*> TrackingPage::channelPlots() const
{
    QVector<PlotWidget*> v{ui->iqPlot};
    v += timePlots_;
    return v;
}

void TrackingPage::linkPlots()
{
    for (PlotWidget* p : timePlots_) {
        connect(p, &PlotWidget::xViewChanged, this, [this, p](double a, double b) {
            if (!ui->linkCheck->isChecked() || syncing_) return;
            syncing_ = true;
            for (PlotWidget* q : timePlots_)
                if (q != p) q->setXView(a, b);
            syncing_ = false;
        });
    }
}

void TrackingPage::refresh()
{
    QComboBox* cc = ui->channelCombo;
    QComboBox* sc = ui->shapeChannelCombo;
    const int keep = cc->currentIndex();
    const QString keepName = cc->currentText();
    cc->blockSignals(true);
    sc->blockSignals(true);
    cc->clear();
    sc->clear();
    if (readable(Receiver::StageTracked))
        for (const auto& t : rx_->trk) {
            cc->addItem(channelLabel(t));
            sc->addItem(channelLabel(t));
        }
    cc->blockSignals(false);
    sc->blockSignals(false);
    if (cc->count() > 0) {
        const int same = cc->findText(keepName);          // channels come and go on a live stream
        cc->setCurrentIndex(same >= 0 ? same : std::clamp(keep, 0, cc->count() - 1));
        sc->setCurrentIndex(cc->currentIndex());
        const TrackingChannel& shapeCh = rx_->trk[size_t(sc->currentIndex())];
        const int n = shapeCh.numMs * shapeCh.periodMs;
        ui->shapeMsSlider->setRange(0, std::max(0, n - 1));
        ui->shapeMsSpin->setRange(0, std::max(0, n - 1));
        if (liveMode()) ui->shapeMsSpin->setValue(std::max(0, n - 500));   // recent samples are the ones still held
        else if (ui->shapeMsSpin->value() == 0) ui->shapeMsSpin->setValue(std::min(n - 1, 5000));
    }
    showChannel();
    fillAllChannels();
    fillSummary();
    ui->shapePlot->clear();
    ui->shapeTable->beginFill(0);
    ui->shapeTable->endFill();
    ui->shapeInfoLabel->clear();
    if (cc->count() > 0) computeShape();   // so the correlator tab is not empty after tracking
}

void TrackingPage::selectPrn(int prn)
{
    if (!readable(Receiver::StageTracked)) return;
    const int ch = rx_->channelOfPrn(prn);
    if (ch >= 0) {
        ui->channelCombo->setCurrentIndex(ch);
        ui->shapeChannelCombo->setCurrentIndex(ch);
    }
}

void TrackingPage::showChannel()
{
    for (PlotWidget* p : channelPlots()) p->clear();
    const int ci = ui->channelCombo->currentIndex();
    if (!readable(Receiver::StageTracked) || ci < 0 || ci >= int(rx_->trk.size())) return;
    const TrackingChannel& t = rx_->trk[size_t(ci)];
    if (!t.valid) return;
    const ReceiverSettings& s = rx_->settings;
    const int n = t.numMs;
    const QVector<double> x = range(n, t.periodMs * 1e-3);
    const QColor cI(31, 119, 180), cQ(214, 39, 40), cG(44, 160, 44), cP(148, 103, 189), cO(255, 127, 14);

    // constellation
    const int skip = std::min(n, ui->skipSpin->value());
    QVector<double> ix, qy;
    ix.reserve(n - skip);
    qy.reserve(n - skip);
    for (int k = skip; k < n; ++k) { ix << t.IP[size_t(k)]; qy << t.QP[size_t(k)]; }
    PlotWidget* iq = ui->iqPlot;
    iq->addSeries(tr("prompt"), ix, qy, PlotWidget::Points, cI);
    iq->series(0).pointSize = 2;
    iq->series(0).sorted = false;
    iq->setTitle(tr("Prompt (I, Q) scatter, %1 (%2 points)").arg(satName(t.prn)).arg(ix.size()));
    iq->autoScale();

    ui->promptPlot->addSeries(tr("I_P"), x, toQ(t.IP), PlotWidget::Lines, cI, 1.0);
    ui->promptPlot->addSeries(tr("Q_P"), x, toQ(t.QP), PlotWidget::Lines, cQ, 1.0);

    QVector<double> e(n), p(n), l(n);
    for (int k = 0; k < n; ++k) {
        e[k] = std::hypot(t.IE[size_t(k)], t.QE[size_t(k)]);
        p[k] = std::hypot(t.IP[size_t(k)], t.QP[size_t(k)]);
        l[k] = std::hypot(t.IL[size_t(k)], t.QL[size_t(k)]);
    }
    ui->eplPlot->addSeries(tr("|P|"), x, p, PlotWidget::Lines, cG, 1.0);
    ui->eplPlot->addSeries(tr("|E|"), x, e, PlotWidget::Lines, cI, 1.0);
    ui->eplPlot->addSeries(tr("|L|"), x, l, PlotWidget::Lines, cQ, 1.0);

    const double fllOff = s.fllAssistMs * 1e-3;
    ui->pllPlot->addSeries(tr("raw"), x, toQ(t.pllDisc), PlotWidget::Lines, cP, 1.0);
    ui->pllPlot->addVLine(fllOff, cO, tr("FLL assist off"));
    ui->dopplerPlot->addSeries(tr("carrier - IF"), x, toQ(t.carrFreq, 1.0, -s.ifFreq), PlotWidget::Lines, cI, 1.2);
    ui->dopplerPlot->addVLine(fllOff, cO, tr("FLL off"));
    ui->fllPlot->addSeries(tr("FLL error"), x, toQ(t.fllDisc), PlotWidget::Lines, cO, 1.0);
    ui->fllPlot->addVLine(fllOff, cP, tr("assist off"));
    ui->dllPlot->addSeries(tr("raw"), x, toQ(t.dllDisc), PlotWidget::Lines, cG, 1.0);
    ui->codePlot->addSeries(tr("code freq - 1.023 MHz"), x, toQ(t.codeFreq, 1.0, -kCaChipRate), PlotWidget::Lines, cI, 1.2);
    ui->codePlot->addSeries(tr("DLL filter output"), x, toQ(t.codeNco), PlotWidget::Lines, cO, 1.0);
    ui->cn0Plot->addSeries(tr("C/N0"), x, toQ(t.cn0), PlotWidget::Lines, cG, 1.2);
    ui->cn0Plot->addHLine(s.minCn0, cQ, tr("min for navigation"));
    ui->pliPlot->addSeries(tr("PLI"), x, toQ(t.pli), PlotWidget::Lines, cP, 1.0);
    ui->pliPlot->addSeries(tr("lock flag"), x, toQ(t.lock), PlotWidget::Steps, cO, 1.0);
    ui->pliPlot->addHLine(0.6, cQ, tr("lock threshold"));
    ui->pliPlot->autoScale();
    ui->adrPlot->addSeries(tr("ADR"), x, toQ(t.adr), PlotWidget::Lines, cI, 1.2);
    QVector<double> dr(n);
    const double spms = s.samplingFreq / 1000.0, spc = s.samplingFreq / kCaChipRate;
    for (int k = 0; k < n; ++k) dr[k] = (t.codeEpochSample[size_t(k)] - t.codeEpochSample[0] - k * t.periodMs * spms) / spc;
    ui->driftPlot->addSeries(tr("code epoch drift"), x, dr, PlotWidget::Lines, cG, 1.2);
}

void TrackingPage::fillAllChannels()
{
    for (PlotWidget* p : {ui->allCn0Plot, ui->allDopplerPlot, ui->allPliPlot, ui->allDllPlot}) p->clear();
    if (!readable(Receiver::StageTracked)) return;
    const double fif = rx_->settings.ifFreq;
    for (size_t i = 0; i < rx_->trk.size(); ++i) {
        const TrackingChannel& t = rx_->trk[i];
        if (!t.valid) continue;
        const QVector<double> x = range(t.numMs, t.periodMs * 1e-3);
        const QColor c = seriesColor(int(i));
        const QString name = satName(t.prn);
        ui->allCn0Plot->addSeries(name, x, toQ(t.cn0), PlotWidget::Lines, c, 1.2);
        ui->allDopplerPlot->addSeries(name, x, toQ(t.carrFreq, 1.0, -fif), PlotWidget::Lines, c, 1.2);
        ui->allPliPlot->addSeries(name, x, toQ(t.pli), PlotWidget::Lines, c, 1.0);
        QVector<double> bx, by;
        for (int k = 0; k + 100 <= t.numMs; k += 100) {
            double a = 0;
            for (int j = 0; j < 100; ++j) a += t.dllDisc[size_t(k + j)];
            bx << (k + 50) * t.periodMs * 1e-3;
            by << a / 100.0;
        }
        ui->allDllPlot->addSeries(name, bx, by, PlotWidget::Lines, c, 1.2);
    }
    ui->allPliPlot->autoScale();
}

void TrackingPage::fillSummary()
{
    DataTable* sum = ui->summaryTable;
    DataTable* loop = ui->loopTable;
    sum->beginFill(0);
    sum->endFill();
    loop->beginFill(0);
    loop->endFill();
    if (!readable(Receiver::StageTracked) || rx_->trk.empty()) return;
    const ReceiverSettings& s = rx_->settings;
    sum->beginFill(int(rx_->trk.size()));
    for (int i = 0; i < int(rx_->trk.size()); ++i) {
        const TrackingChannel& t = rx_->trk[size_t(i)];
        sum->setNum(i, 0, t.channel, 0);
        sum->setKey(i, i);
        sum->setText(i, 1, satLabel(t.prn));
        sum->setText(i, 2, t.status, t.lockFraction > 0.9 ? goodColor() : warnColor());
        sum->setNum(i, 3, double(t.startSample), 0);
        sum->setNum(i, 4, t.numMs * t.periodMs, 0);
        sum->setNum(i, 5, t.initCarrFreq, 2);
        sum->setNum(i, 6, t.initCarrFreq - s.ifFreq, 2);
        sum->setNum(i, 7, t.finalDoppler, 2);
        const int from = std::min(t.numMs - 1, std::max(0, s.bitSyncStartMs / t.periodMs));
        double rate = 0, mn = 1e9, ep = 0, ed = 0;
        int cnt = 0;
        if (t.numMs > from + 1000 / t.periodMs) rate = (t.carrFreq.back() - t.carrFreq[size_t(from)]) / ((t.numMs - 1 - from) * t.periodMs * 1e-3);
        for (int k = std::max(0, from); k < t.numMs; ++k) {
            mn = std::min(mn, double(t.cn0[size_t(k)]));
            ep += double(t.pllDisc[size_t(k)]) * t.pllDisc[size_t(k)];
            ed += double(t.dllDisc[size_t(k)]) * t.dllDisc[size_t(k)];
            ++cnt;
        }
        sum->setNum(i, 8, rate, 3);
        sum->setNum(i, 9, t.meanCn0, 1);
        sum->setNum(i, 10, cnt ? mn : NAN, 1);
        sum->setNum(i, 11, t.lockFraction * 100, 1, t.lockFraction > 0.9 ? goodColor() : badColor());
        sum->setNum(i, 12, cnt ? std::sqrt(ep / cnt) * 180 / kPi : NAN, 2);
        sum->setNum(i, 13, cnt ? std::sqrt(ed / cnt) : NAN, 4);
        sum->setNum(i, 14, t.elapsedSec, 2);
    }
    sum->endFill();

    const TrackingChannel& t0 = rx_->trk.front();
    loop->beginFill(0);
    auto row = [&](const QString& a, const QString& b, double v, const QString& c) {
        const int r = loop->rowCount();
        loop->setRowCount(r + 1);
        loop->setText(r, 0, a);
        loop->setText(r, 1, b);
        loop->setSci(r, 2, v, 6);
        loop->setText(r, 3, c);
    };
    row(tr("PLL"), tr("Noise bandwidth Bn (Hz)"), s.pllBandwidth, tr("2nd order Costas loop"));
    row(tr("PLL"), tr("Damping ratio zeta"), s.pllDamping, QString());
    row(tr("PLL"), tr("Natural frequency wn (rad/s)"), t0.pllWn, tr("wn = 8 zeta Bn / (4 zeta^2 + 1)"));
    row(tr("PLL"), tr("Proportional gain K1 = 2 zeta wn"), t0.pllK1, tr("rad/s per rad"));
    row(tr("PLL"), tr("Integral gain K2 = wn^2"), t0.pllK2, tr("rad/s^2 per rad"));
    row(tr("PLL"), tr("Bn * T"), s.pllBandwidth * 1e-3, tr("must be << 1 for the analogue design to hold"));
    row(tr("FLL"), tr("Noise bandwidth (Hz)"), s.fllBandwidth, tr("1st-order FLL assisting the PLL integrator"));
    row(tr("FLL"), tr("w0f = Bn / 0.25 (rad/s)"), t0.fllW0, QString());
    row(tr("FLL"), tr("Assist duration (ms)"), s.fllAssistMs, tr("cross/dot discriminator, insensitive to data bits"));
    row(tr("DLL"), tr("Noise bandwidth Bn (Hz)"), s.dllBandwidth, tr("2nd order, normalised early-minus-late envelope"));
    row(tr("DLL"), tr("Damping ratio zeta"), s.dllDamping, QString());
    row(tr("DLL"), tr("Natural frequency wn (rad/s)"), t0.dllWn, QString());
    row(tr("DLL"), tr("K1 = 2 zeta wn"), t0.dllK1, tr("chips/s per chip"));
    row(tr("DLL"), tr("K2 = wn^2"), t0.dllK2, QString());
    row(tr("DLL"), tr("Early/late offset (chips)"), s.dllSpacing, tr("total E-L spacing = %1 chip").arg(2 * s.dllSpacing));
    row(tr("DLL"), tr("Carrier aiding"), s.carrierAiding ? 1 : 0, s.carrierAiding ? tr("on") : tr("off"));
    row(tr("Common"), tr("Pre-detection integration T (ms)"), 1.0, tr("one C/A code period"));
    row(tr("Common"), tr("C/N0 estimator window (ms)"), s.cn0WindowMs, tr("M2M4 moments of |P|^2"));
    loop->endFill();
}

void TrackingPage::computeShape()
{
    const int ci = ui->shapeChannelCombo->currentIndex();
    if (!readable(Receiver::StageTracked) || ci < 0 || ci >= int(rx_->trk.size())) return;
    const TrackingChannel& t = rx_->trk[size_t(ci)];
    const int ms = std::clamp(ui->shapeMsSpin->value() / t.periodMs, 0, t.numMs - 1);   // tracking index
    const int avg = std::max(1, ui->avgSpin->value() / t.periodMs);
    if (!liveMode()) QApplication::setOverrideCursor(Qt::WaitCursor);
    const auto pts = Tracking::correlationShape(rx_->source, rx_->settings, t, ms, ui->spanSpin->value(), ui->tapsSpin->value(), avg);
    if (!liveMode()) QApplication::restoreOverrideCursor();
    PlotWidget* sp = ui->shapePlot;
    DataTable* st = ui->shapeTable;
    sp->clear();
    st->beginFill(int(pts.size()));
    QVector<double> x, m, ii, qq;
    for (int i = 0; i < int(pts.size()); ++i) {
        const auto& p = pts[size_t(i)];
        x << p.offset; m << p.magnitude; ii << p.inPhase; qq << p.quadrature;
        st->setNum(i, 0, p.offset, 3);
        st->setNum(i, 1, p.magnitude, 1);
        st->setNum(i, 2, p.inPhase, 1);
        st->setNum(i, 3, p.quadrature, 1);
    }
    st->endFill();
    sp->addSeries(tr("|R| (non-coherent mean)"), x, m, PlotWidget::LinesPoints, QColor(31, 119, 180), 1.5);
    sp->addSeries(tr("I (sign-normalised)"), x, ii, PlotWidget::Lines, QColor(44, 160, 44), 1.0);
    sp->addSeries(tr("Q"), x, qq, PlotWidget::Lines, QColor(214, 39, 40), 1.0);
    const int last = std::min(t.numMs, ms + avg);
    double e = 0, p = 0, l = 0;
    for (int k = ms; k < last; ++k) {
        e += std::hypot(t.IE[size_t(k)], t.QE[size_t(k)]);
        p += std::hypot(t.IP[size_t(k)], t.QP[size_t(k)]);
        l += std::hypot(t.IL[size_t(k)], t.QL[size_t(k)]);
    }
    const int cnt = std::max(1, last - ms);
    const double d = isGalileo(t.prn) ? rx_->settings.galDllSpacing : rx_->settings.dllSpacing;
    sp->addSeries(tr("tracking E / P / L"), {-d, 0.0, d}, {l / cnt, p / cnt, e / cnt}, PlotWidget::Points, QColor(255, 127, 14));
    sp->series(sp->seriesCount() - 1).pointSize = 9;
    sp->addVLine(0, QColor(127, 127, 127), tr("prompt"), Qt::DotLine);
    sp->setTitle(tr("%1 correlation function at t = %2 s (averaged over %3 ms)").arg(satName(t.prn))
                     .arg(ms * t.periodMs * 1e-3, 0, 'f', 3).arg(cnt * t.periodMs));
    ui->shapeInfoLabel->setText(tr("Carrier %1 Hz, code %2 chips/s, code phase at block start %3 chips, carrier phase %4 cycles, "
                                   "block of %5 samples starting at sample %6. The asymmetry |E| - |L| drives the DLL; "
                                   "the rounded peak reflects the front-end bandwidth.")
                                    .arg(t.carrFreq[size_t(ms)], 0, 'f', 3).arg(t.codeFreq[size_t(ms)], 0, 'f', 4)
                                    .arg(t.remCodePhase[size_t(ms)], 0, 'f', 5).arg(t.remCarrPhase[size_t(ms)], 0, 'f', 5)
                                    .arg(t.blockSize[size_t(ms)]).arg(t.blockStart[size_t(ms)]));
}

} // namespace gui
