#include "RawSignalPage.h"
#include "ui_RawSignalPage.h"
#include "DataTable.h"
#include "PlotWidget.h"
#include "core/Constants.h"
#include "core/SignalAnalysis.h"

#include <QApplication>
#include <cmath>

using namespace gnss;

namespace gui {

RawSignalPage::RawSignalPage(QWidget* parent) : PageBase(parent), ui(new Ui::RawSignalPage)
{
    ui->setupUi(this);
    ui->splitter->setStretchFactor(0, 4);
    ui->splitter->setStretchFactor(1, 1);
    ui->statsTable->setColumns({tr("Statistic"), tr("Value"), tr("Comment")});
    connect(ui->analyseButton, &QPushButton::clicked, this, &RawSignalPage::analyse);
    connect(ui->wholeFileButton, &QPushButton::clicked, this, &RawSignalPage::analyseSpectrogram);
}

RawSignalPage::~RawSignalPage()
{
    delete ui;
}

void RawSignalPage::refresh()
{
    if (!rx_ || !rx_->source.isOpen()) return;
    ui->startSpin->setMaximum(rx_->fileDurationSec() * 1000.0);
    if (lastFile_ != rx_->settings.fileName) { done_ = false; lastFile_ = rx_->settings.fileName; }
    if ((!done_ || liveMode()) && isVisible()) analyse();
}

void RawSignalPage::clearContents()
{
    PageBase::clearContents();
    done_ = false;
    lastFile_.clear();
}

void RawSignalPage::showEvent(QShowEvent* e)
{
    PageBase::showEvent(e);
    if (!done_ && rx_ && rx_->source.isOpen()) analyse();
}

void RawSignalPage::analyse()
{
    if (!rx_ || !rx_->source.isOpen()) return;
    const bool wait = !liveMode();
    if (wait) QApplication::setOverrideCursor(Qt::WaitCursor);
    done_ = true;
    const ReceiverSettings& s = rx_->settings;
    const double fs = s.samplingFreq;
    qint64 start = qint64(ui->startSpin->value() * fs / 1000.0);
    const qint64 count = qint64(ui->lengthSpin->value() * fs / 1000.0);
    if (liveMode()) start = std::max(rx_->source.oldestSample(), rx_->source.numSamples() - count);   // the newest samples
    PlotWidget* tp = ui->timePlot;
    PlotWidget* hp = ui->histPlot;
    PlotWidget* pp = ui->psdPlot;
    DataTable* st_ = ui->statsTable;

    // time domain
    const int nt = ui->samplesSpin->value();
    std::vector<float> bi(static_cast<size_t>(nt)), bq(static_cast<size_t>(nt));
    const int got = rx_->source.read(start, nt, bi.data(), bq.data());
    QVector<double> t(got), yi(got), yq(got);
    for (int i = 0; i < got; ++i) { t[i] = i / fs * 1e6; yi[i] = bi[size_t(i)]; yq[i] = bq[size_t(i)]; }
    tp->clear();
    tp->addSeries(rx_->source.isComplex() ? tr("I") : tr("samples"), t, yi, PlotWidget::LinesPoints, QColor(31, 119, 180), 1.0);
    tp->series(0).pointSize = 2;
    if (rx_->source.isComplex()) tp->addSeries(tr("Q"), t, yq, PlotWidget::LinesPoints, QColor(214, 39, 40), 1.0);

    // statistics & histogram
    const RawStats st = analysis::computeStats(rx_->source, start, count);
    QVector<double> hx, hy;
    for (const auto& kv : st.histogram) { hx << kv.first; hy << double(kv.second) / std::max<qint64>(1, st.count); }
    hp->clear();
    hp->addSeries(tr("measured"), hx, hy, PlotWidget::Bars, QColor(44, 160, 44));
    QVector<double> gx, gy;
    const double gaussStep = std::max(0.05, (st.maxV - st.minV) / 400.0);
    if (st.stddev > 0)
        for (double x = st.minV - 1; x <= st.maxV + 1; x += gaussStep) {
            gx << x;
            gy << std::exp(-0.5 * std::pow((x - st.mean) / st.stddev, 2)) / (st.stddev * std::sqrt(2 * kPi));
        }
    hp->addSeries(tr("Gaussian (same mean, sigma)"), gx, gy, PlotWidget::Lines, QColor(214, 39, 40), 1.5);

    st_->beginFill(0);
    auto row = [&](const QString& a, const QString& b, const QString& c = QString()) {
        const int r = st_->rowCount();
        st_->setRowCount(r + 1);
        st_->setText(r, 0, a);
        st_->setText(r, 1, b);
        st_->setText(r, 2, c);
    };
    row(tr("Samples analysed"), QString::number(st.count),
        tr("%1 ms from %2 ms").arg(ui->lengthSpin->value()).arg(ui->startSpin->value()));
    row(tr("Mean (DC offset)"), QString::number(st.mean, 'f', 5), tr("should be ~0"));
    row(tr("Standard deviation"), QString::number(st.stddev, 'f', 5), tr("AGC / quantiser loading"));
    row(tr("RMS"), QString::number(st.rms, 'f', 5));
    row(tr("Min / max"), QStringLiteral("%1 / %2").arg(st.minV).arg(st.maxV));
    row(tr("Distinct levels"), QString::number(st.distinctLevels), tr("%1 effective bits").arg(st.bitsUsed, 0, 'f', 2));
    row(tr("Samples at extreme levels"), QStringLiteral("%1 %").arg(st.clipFraction * 100, 0, 'f', 3), tr("clipping indicator"));
    row(tr("Max level / sigma"), QString::number(std::max(std::fabs(st.minV), std::fabs(st.maxV)) / std::max(1e-9, st.stddev), 'f', 2),
        tr("~2.5-3.5 typical for a multi-bit ADC"));
    row(tr("Skewness"), QString::number(st.skewness, 'f', 4), tr("0 for a symmetric distribution"));
    row(tr("Kurtosis"), QString::number(st.kurtosis, 'f', 4), tr("3 for Gaussian noise; deviations indicate interference/clipping"));
    if (st.hasQ) {
        row(tr("Q mean / sigma"), QStringLiteral("%1 / %2").arg(st.meanQ, 0, 'f', 4).arg(st.stddevQ, 0, 'f', 4));
        row(tr("I/Q correlation"), QString::number(st.iqCorrelation, 'f', 5), tr("quadrature balance"));
        row(tr("I/Q amplitude ratio"), QString::number(st.stddev / std::max(1e-9, st.stddevQ), 'f', 5));
    }
    row(tr("Sampling frequency"), QStringLiteral("%1 MHz").arg(fs / 1e6, 0, 'f', 6));
    row(tr("Intermediate frequency"), QStringLiteral("%1 MHz").arg(s.ifFreq / 1e6, 0, 'f', 6));
    row(tr("Signal at IF"), tr("GPS L1 C/A is ~20 dB below the noise floor"), tr("the PSD shows the front-end filter shape"));

    // PSD
    const int nfft = ui->fftCombo->currentText().toInt();
    const Psd p = analysis::welchPsd(rx_->source, fs, start, count, nfft);
    pp->clear();
    pp->addSeries(tr("PSD"), toQ(p.freq, 1e-6), toQ(p.powerDb), PlotWidget::Lines, QColor(31, 119, 180), 1.0);
    pp->addVLine(s.ifFreq / 1e6, QColor(214, 39, 40), tr("IF"));
    pp->addVLine((s.ifFreq - kCaChipRate) / 1e6, QColor(255, 127, 14), tr("IF-1.023"), Qt::DotLine);
    pp->addVLine((s.ifFreq + kCaChipRate) / 1e6, QColor(255, 127, 14), tr("IF+1.023"), Qt::DotLine);
    row(tr("PSD resolution"), QStringLiteral("%1 kHz").arg(p.resolution / 1e3, 0, 'f', 3), tr("%1 averaged segments").arg(p.segments));
    if (!p.powerDb.empty()) {
        std::vector<double> sm(p.powerDb.size());
        const int k = std::max(1, int(p.powerDb.size() / 200));
        for (size_t i = 0; i < sm.size(); ++i) {
            double a = 0;
            int c = 0;
            for (int j = -k; j <= k; ++j) {
                const int q = int(i) + j;
                if (q >= 0 && q < int(sm.size())) { a += p.powerDb[size_t(q)]; ++c; }
            }
            sm[i] = a / c;
        }
        const size_t im = size_t(std::max_element(sm.begin(), sm.end()) - sm.begin());
        size_t lo = im, hi = im;
        while (lo > 0 && sm[lo] > sm[im] - 3) --lo;
        while (hi + 1 < sm.size() && sm[hi] > sm[im] - 3) ++hi;
        row(tr("Spectrum peak"), QStringLiteral("%1 MHz").arg(p.freq[im] / 1e6, 0, 'f', 3));
        row(tr("3-dB bandwidth (front-end)"), QStringLiteral("%1 MHz").arg((p.freq[hi] - p.freq[lo]) / 1e6, 0, 'f', 3),
            QStringLiteral("%1 - %2 MHz").arg(p.freq[lo] / 1e6, 0, 'f', 3).arg(p.freq[hi] / 1e6, 0, 'f', 3));
    }
    st_->endFill();

    const Spectrogram sg = analysis::spectrogram(rx_->source, fs, start, count, std::min(nfft, 4096), ui->slicesSpin->value(), 4);
    ui->specPlot->clear();
    ui->specPlot->setHeatmap(QVector<float>(sg.db.begin(), sg.db.end()), sg.nf, sg.nt, sg.f0 / 1e6, sg.f1 / 1e6, sg.t0, sg.t1, tr("dB"));
    ui->specPlot->setTitle(tr("Spectrogram (%1 - %2 s)").arg(sg.t0, 0, 'f', 3).arg(sg.t1, 0, 'f', 3));
    if (wait) QApplication::restoreOverrideCursor();
}

void RawSignalPage::analyseSpectrogram()
{
    if (!rx_ || !rx_->source.isOpen()) return;
    QApplication::setOverrideCursor(Qt::WaitCursor);
    const double fs = rx_->settings.samplingFreq;
    const qint64 first = rx_->source.oldestSample();
    const Spectrogram sg = analysis::spectrogram(rx_->source, fs, first, rx_->source.numSamples() - first, 2048, ui->slicesSpin->value(), 8);
    ui->specPlot->clear();
    ui->specPlot->setHeatmap(QVector<float>(sg.db.begin(), sg.db.end()), sg.nf, sg.nt, sg.f0 / 1e6, sg.f1 / 1e6, sg.t0, sg.t1, tr("dB"));
    ui->specPlot->setTitle(tr("Spectrogram of the whole recording"));
    QApplication::restoreOverrideCursor();
}

} // namespace gui
