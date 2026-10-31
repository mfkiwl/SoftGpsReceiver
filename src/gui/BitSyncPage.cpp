#include "BitSyncPage.h"
#include "ui_BitSyncPage.h"
#include "DataTable.h"
#include "PlotWidget.h"
#include "core/Constants.h"

#include <QComboBox>
#include <cmath>

using namespace gnss;

namespace gui {

BitSyncPage::BitSyncPage(QWidget* parent) : PageBase(parent), ui(new Ui::BitSyncPage)
{
    ui->setupUi(this);
    ui->splitter->setStretchFactor(0, 3);
    ui->splitter->setStretchFactor(1, 2);
    ui->summaryTable->setColumns({tr("Property"), tr("Value")});
    QStringList cols{tr("First bit"), tr("ms")};
    for (int i = 1; i <= 30; ++i) cols << QString::number(i);
    ui->bitsTable->setColumns(cols);
    connect(ui->channelCombo, &QComboBox::currentIndexChanged, this, &BitSyncPage::showChannel);
}

BitSyncPage::~BitSyncPage()
{
    delete ui;
}
void BitSyncPage::refresh()
{
    QComboBox* channel_ = ui->channelCombo;
    const int keep = channel_->currentIndex();
    const QString keepName = channel_->currentText();
    channel_->blockSignals(true);
    channel_->clear();
    if (readable(Receiver::StageDecoded))
        for (const auto& n : rx_->nav) channel_->addItem(QStringLiteral("Ch %1 - %2").arg(n.channel).arg(satName(n.prn)));
    channel_->blockSignals(false);
    if (channel_->count() > 0) {
        const int same = channel_->findText(keepName);    // channels come and go on a live stream
        channel_->setCurrentIndex(same >= 0 ? same : std::clamp(keep, 0, channel_->count() - 1));
    }
    showChannel();
}

void BitSyncPage::selectPrn(int prn)
{
    QComboBox* channel_ = ui->channelCombo;
    if (!readable(Receiver::StageDecoded)) return;
    for (int i = 0; i < int(rx_->nav.size()); ++i)
        if (rx_->nav[size_t(i)].prn == prn) channel_->setCurrentIndex(i);
}

void BitSyncPage::showChannel()
{
    QComboBox* channel_ = ui->channelCombo;
    PlotWidget* hist_ = ui->histPlot;
    PlotWidget* ip_ = ui->promptPlot;
    PlotWidget* soft_ = ui->softPlot;
    PlotWidget* softHist_ = ui->softHistPlot;
    PlotWidget* pre_ = ui->preamblePlot;
    DataTable* summary_ = ui->summaryTable;
    DataTable* bits_ = ui->bitsTable;
    for (PlotWidget* p : {hist_, ip_, soft_, softHist_, pre_}) p->clear();
    summary_->beginFill(0);
    summary_->endFill();
    bits_->beginFill(0);
    bits_->endFill();
    const int ci = channel_->currentIndex();
    if (!readable(Receiver::StageDecoded) || ci < 0 || ci >= int(rx_->nav.size()) || ci >= int(rx_->trk.size())) return;
    const NavChannel& n = rx_->nav[size_t(ci)];
    const TrackingChannel& t = rx_->trk[size_t(ci)];

    // Galileo sends one symbol per code period, so its histogram counts sync patterns per offset within a page part
    const bool gal = isGalileo(n.prn);
    ui->infoLabel->setText(gal ? tr("Symbols arrive one per 4 ms code period, so there is no bit synchronisation; pages are found by the sync pattern 0101100000 and confirmed by the CRC")
                            : tr("Bit synchronisation by histogram of prompt sign changes (mod 20 ms); bits = sum of 20 prompt values; preamble 10001011 verified by TLM/HOW parity"));
    const int nHist = gal ? kInavPartSymbols : 20;
    std::vector<int> histCount(size_t(nHist), 0);
    if (gal) for (int j : n.preambleMatches) histCount[size_t(j % nHist)]++;
    else std::copy(n.transitionHist.begin(), n.transitionHist.end(), histCount.begin());
    QVector<double> hx, hy, bx, by;
    for (int i = 0; i < nHist; ++i) {
        if (i == n.bitOffset) { bx << i; by << histCount[size_t(i)]; }
        else { hx << i; hy << histCount[size_t(i)]; }
    }
    hist_->addSeries(tr("other offsets"), hx, hy, PlotWidget::Bars, QColor(160, 160, 160));
    hist_->addSeries(gal ? tr("page start") : tr("bit edge"), bx, by, PlotWidget::Bars, QColor(44, 160, 44));
    hist_->series(0).barWidth = hist_->series(1).barWidth = 0.75;
    hist_->autoScale();
    hist_->setLabels(gal ? tr("Sync pattern matches per offset within a 250-symbol page part") : tr("Histogram of I_P sign transitions vs ms mod 20"),
                     gal ? tr("Symbol offset") : tr("ms within the 20-ms bit"), gal ? tr("Matches") : tr("Transitions"));
    soft_->setLabels(gal ? tr("E1-B symbols") : tr("Integrated (soft) navigation bits"),
                     gal ? tr("Symbol index") : tr("Bit index"), gal ? tr("I_P (E1-B)") : tr("Sum of 20 I_P"));
    pre_->setLabels(gal ? tr("Correlation of the symbols with the sync pattern 0101100000 (+/-10 = match, red = page part)")
                        : tr("Correlation of the bit stream with the preamble (+/-8 = match, red = parity-verified subframe start)"),
                    gal ? tr("Symbol index") : tr("Bit index"), tr("Correlation"));

    // I_P over the first 1 s after bit sync start, with bit boundaries
    const int a = std::max(0, n.firstBitMs - 40);
    const int b = std::min(t.numMs, a + (gal ? 2 * kInavPartSymbols : 1000));
    QVector<double> x, y;
    if (gal) {
        for (int k = a; k < b; ++k) { x << k; y << t.IB[size_t(k)]; }
        ip_->addSeries(tr("I_P (E1-B)"), x, y, PlotWidget::Steps, QColor(31, 119, 180), 1.0);
        for (int k = n.firstBitMs + n.bitOffset; k < b; k += kInavPartSymbols) ip_->addVLine(k, QColor(214, 39, 40, 140), QString(), Qt::DotLine);
        ip_->setTitle(tr("E1-B symbols with page part boundaries (first symbol at index %1, page offset %2 symbols)").arg(n.firstBitMs).arg(n.bitOffset));
    } else {
        for (int k = a; k < b; ++k) { x << k; y << t.IP[size_t(k)]; }
        ip_->addSeries(tr("I_P"), x, y, PlotWidget::Steps, QColor(31, 119, 180), 1.0);
        for (int k = n.firstBitMs; k < b; k += kMsPerBit) ip_->addVLine(k, QColor(214, 39, 40, 140), QString(), Qt::DotLine);
        QVector<double> sx, sy;
        for (int j = 0; j < int(n.softBits.size()); ++j) {
            const int ms = n.firstBitMs + kMsPerBit * j;
            if (ms < a) continue;
            if (ms >= b) break;
            sx << ms;
            sy << n.softBits[size_t(j)] / kMsPerBit;
        }
        ip_->addSeries(tr("bit value / 20"), sx, sy, PlotWidget::Steps, QColor(255, 127, 14), 2.0);
        ip_->setTitle(tr("Prompt I with bit boundaries (first bit at ms %1, offset %2 ms)").arg(n.firstBitMs).arg(n.bitOffset));
    }

    soft_->addSeries(tr("soft bits"), range(int(n.softBits.size())), toQ(n.softBits), PlotWidget::Points, QColor(148, 103, 189));
    soft_->series(0).pointSize = 2.5;
    soft_->addHLine(0, QColor(127, 127, 127), QString(), Qt::SolidLine);

    if (!n.softBits.empty()) {
        float mx = 0;
        for (float v : n.softBits) mx = std::max(mx, std::fabs(v));
        const int nbins = 80;
        QVector<double> cx(nbins), cy(nbins, 0);
        for (int i = 0; i < nbins; ++i) cx[i] = -mx + (i + 0.5) * 2 * mx / nbins;
        for (float v : n.softBits) cy[std::clamp(int((v + mx) / (2 * mx) * nbins), 0, nbins - 1)] += 1;
        softHist_->addSeries(tr("count"), cx, cy, PlotWidget::Bars, QColor(23, 190, 207));
        softHist_->series(0).barWidth = 2 * mx / nbins;
        softHist_->autoScale();
    }

    QVector<double> px, py, vx, vy;
    for (int j = 0; j < int(n.preambleCorr.size()); ++j) { px << j; py << n.preambleCorr[size_t(j)]; }
    for (int j : n.preambleVerified) { vx << j; vy << n.preambleCorr[size_t(j)]; }
    pre_->addSeries(tr("correlation"), px, py, PlotWidget::Lines, QColor(31, 119, 180), 0.8);
    pre_->addSeries(gal ? tr("page parts") : tr("verified subframe starts"), vx, vy, PlotWidget::Points, QColor(214, 39, 40));
    pre_->series(1).pointSize = 7;
    const int full = gal ? 10 : 8;
    pre_->addHLine(full, QColor(44, 160, 44), QStringLiteral("+%1").arg(full), Qt::DotLine);
    pre_->addHLine(-full, QColor(44, 160, 44), QStringLiteral("-%1 (inverted)").arg(full), Qt::DotLine);

    summary_->beginFill(0);
    auto row = [&](const QString& k, const QString& v, const QColor& c = QColor()) {
        const int r = summary_->rowCount();
        summary_->setRowCount(r + 1);
        summary_->setText(r, 0, k);
        summary_->setText(r, 1, v, c);
    };
    int total = 0;
    for (int v : n.transitionHist) total += v;
    row(tr("Satellite"), satName(n.prn));
    if (gal) {
        row(tr("Symbol rate"), tr("250 symbols/s: one per 4 ms code period, so no bit synchronisation is needed"));
        row(tr("Page offset"), tr("%1 symbols").arg(n.bitOffset));
        row(tr("Confidence"), tr("%1 % of sync pattern matches").arg(n.bitSyncConfidence * 100, 0, 'f', 1),
            n.bitSyncOk ? goodColor() : badColor());
        row(tr("Symbols"), QString::number(n.softBits.size()));
        row(tr("Symbol SNR (mean^2/var)"), tr("%1 dB").arg(n.bitSnrDb, 0, 'f', 1));
        row(tr("Raw symbol error rate"), QString::number(n.estimatedBer, 'g', 3));
        row(tr("Sync pattern matches"), QString::number(n.preambleMatches.size()));
        row(tr("Page parts with a clean sync"), QString::number(n.preambleVerified.size()),
            n.preambleVerified.empty() ? badColor() : goodColor());
        row(tr("Words passing the CRC"), QStringLiteral("%1 / %2").arg(n.parityOkWords).arg(n.parityTotalWords));
    } else {
    row(tr("Histogram start"), tr("ms %1 (pull-in ignored)").arg(n.bitSyncStartMs));
    row(tr("Transitions counted"), QString::number(total));
    row(tr("Bit edge offset"), tr("%1 ms").arg(n.bitOffset));
    row(tr("Confidence"), tr("%1 % of transitions").arg(n.bitSyncConfidence * 100, 0, 'f', 1),
        n.bitSyncOk ? goodColor() : badColor());
    row(tr("First full bit"), tr("tracking ms %1").arg(n.firstBitMs));
    row(tr("Bits integrated"), QString::number(n.softBits.size()));
    row(tr("Bit SNR (mean^2/var)"), tr("%1 dB").arg(n.bitSnrDb, 0, 'f', 1));
    row(tr("Estimated bit error rate"), QString::number(n.estimatedBer, 'g', 3));
    row(tr("Preamble pattern matches"), QString::number(n.preambleMatches.size()), QColor());
    row(tr("Parity-verified preambles"), QString::number(n.preambleVerified.size()),
        n.preambleVerified.empty() ? badColor() : goodColor());
    row(tr("Subframes decoded"), QString::number(n.subframes.size()));
    }
    if (!n.subframes.empty())
        row(tr("Polarity"), n.subframes.front().inverted ? tr("inverted (PLL locked 180 deg)") : tr("normal"));
    row(tr("Status"), n.status);
    summary_->endFill();

    // bit stream grid
    const int nb = int(n.hardBits.size());
    const int phase = gal ? n.bitOffset % 30 : (n.subframes.empty() ? 0 : n.subframes.front().startBit % 30);
    const int rows = (nb - phase) / 30 + (phase > 0 ? 1 : 0);
    bits_->setUpdatesEnabled(false);
    bits_->beginFill(std::max(0, rows));
    std::vector<char> preStart(size_t(nb), 0);
    for (int j : n.preambleVerified) if (j < nb) preStart[size_t(j)] = 1;
    int r = 0;
    for (int start = phase > 0 ? phase - 30 : 0; start < nb && r < rows; start += 30, ++r) {
        bits_->setNum(r, 0, std::max(0, start), 0);
        bits_->setNum(r, 1, (n.firstBitMs + (gal ? 1 : kMsPerBit) * std::max(0, start)) * t.periodMs, 0);
        for (int i = 0; i < 30; ++i) {
            const int j = start + i;
            if (j < 0 || j >= nb) continue;
            QColor bg = n.hardBits[size_t(j)] ? QColor(200, 220, 255) : QColor(255, 255, 255);
            bool inPre = false;
            for (int k = 0; k < full && !inPre; ++k)
                if (j - k >= 0 && preStart[size_t(j - k)]) inPre = true;
            if (inPre) bg = QColor(255, 210, 140);
            bits_->setText(r, 2 + i, QString::number(n.hardBits[size_t(j)]), bg);
        }
    }
    bits_->endFill(false);
    for (int c = 2; c < 32; ++c) bits_->setColumnWidth(c, 24);
    bits_->resizeColumnToContents(0);
    bits_->resizeColumnToContents(1);
    bits_->setUpdatesEnabled(true);
}

} // namespace gui
