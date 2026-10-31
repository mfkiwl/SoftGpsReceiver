#include "NavMessagePage.h"
#include "ui_NavMessagePage.h"
#include "DataTable.h"
#include "core/Constants.h"
#include "core/GalileoNav.h"

#include <QComboBox>
#include <QFont>
#include <QHeaderView>
#include <QLabel>

using namespace gnss;

namespace gui {

namespace {

QString bin(uint32_t v, int n)
{
    return QStringLiteral("%1").arg(v, n, 2, QLatin1Char('0'));
}

QString contentOf(const Subframe& sf)
{
    switch (sf.id) {
    case 1: return QStringLiteral("Week, SV accuracy & health, clock correction");
    case 2: return QStringLiteral("Ephemeris (part 1)");
    case 3: return QStringLiteral("Ephemeris (part 2)");
    case 4:
    case 5: return QStringLiteral("Page %1: %2").arg(sf.page).arg(nav::pageDescription(sf.id, sf.svId));
    default: return QStringLiteral("Invalid subframe ID");
    }
}

QColor fieldColor(const NavFieldDef& f, int idx)
{
    switch (f.kind) {
    case FieldKind::Parity: return QColor(205, 205, 205);
    case FieldKind::Reserved: return f.key == "t" ? QColor(250, 240, 190) : QColor(235, 235, 235);
    default: break;
    }
    if (f.key == "preamble") return QColor(255, 190, 120);
    return QColor::fromHsv((idx * 53) % 360, 70, 250);
}

} // namespace

NavMessagePage::NavMessagePage(QWidget* parent) : PageBase(parent), ui(new Ui::NavMessagePage)
{
    ui->setupUi(this);
    ui->splitter->setStretchFactor(0, 2);
    ui->splitter->setStretchFactor(1, 3);
    ui->rightSplitter->setStretchFactor(0, 2);
    ui->rightSplitter->setStretchFactor(1, 2);
    ui->rightSplitter->setStretchFactor(2, 4);
    setSubframeColumns(false);
    QStringList bc;
    for (int i = 1; i <= 30; ++i) bc << QString::number(i);
    ui->bitmapTable->setColumns(bc);
    ui->bitmapTable->verticalHeader()->setVisible(true);
    ui->bitmapTable->setSelectionMode(QAbstractItemView::NoSelection);
    ui->fieldsTable->setColumns({tr("Field"), tr("Word"), tr("Bits"), tr("Length"), tr("Raw bits"), tr("Raw (unsigned)"),
                                 tr("Raw (signed)"), tr("Scale"), tr("Value"), tr("Units"), tr("Interpretation")});
    connect(ui->channelCombo, &QComboBox::currentIndexChanged, this, &NavMessagePage::showChannel);
    connect(ui->subframesTable, &QTableWidget::currentCellChanged, this, [this](int r, int, int, int) {
        const QVariant k = ui->subframesTable->keyOfRow(r);
        if (k.isValid()) showSubframe(k.toInt());
    });
}

NavMessagePage::~NavMessagePage()
{
    delete ui;
}
void NavMessagePage::refresh()
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

void NavMessagePage::selectPrn(int prn)
{
    QComboBox* channel_ = ui->channelCombo;
    if (!readable(Receiver::StageDecoded)) return;
    for (int i = 0; i < int(rx_->nav.size()); ++i)
        if (rx_->nav[size_t(i)].prn == prn) channel_->setCurrentIndex(i);
}

void NavMessagePage::setSubframeColumns(bool galileo)
{
    if (galileo) {
        ui->subframesGroup->setTitle(tr("I/NAV words (select one)"));
        ui->wordsGroup->setTitle(tr("Word: CRC-24Q over the even and odd page parts"));
        ui->bitmapGroup->setTitle(tr("Bit map of the 128-bit word (colour = field, hover for its name, dark = word failed the CRC)"));
        ui->subframesTable->setColumns({tr("#"), tr("Start ms"), tr("Word type"), tr("TOW at start (s)"), tr("Time of week"),
                                        tr("CRC"), tr("Polarity"), tr("Content")});
        ui->wordsTable->setColumns({tr("Type"), tr("Content"), tr("Word bits"), tr("Hex"), tr("Status")});
    } else {
        ui->subframesGroup->setTitle(tr("Subframes (select one)"));
        ui->wordsGroup->setTitle(tr("Words: parity check per IS-GPS-200 Table 20-XIV (d = D xor D30*)"));
        ui->bitmapGroup->setTitle(tr("Bit map of source data bits (colour = field, hover for its name, grey = parity, dark = word failed parity)"));
        ui->subframesTable->setColumns({tr("#"), tr("Start ms"), tr("Start bit"), tr("SF ID"), tr("Page"), tr("TOW count"),
                                        tr("TOW at start (s)"), tr("Time of week"), tr("Parity"), tr("Polarity"), tr("Alert"),
                                        tr("A-S"), tr("ISF"), tr("TOW consistent"), tr("Content")});
        ui->wordsTable->setColumns({tr("Word"), tr("Name"), tr("Received D1..D30"), tr("Source data d1..d24"), tr("Hex (d1-d24)"),
                                    tr("D29* D30*"), tr("Parity received"), tr("Parity computed"), tr("Status")});
    }
}

void NavMessagePage::showChannel()
{
    QComboBox* channel_ = ui->channelCombo;
    DataTable* subframes_ = ui->subframesTable;
    DataTable* words_ = ui->wordsTable;
    DataTable* bitmap_ = ui->bitmapTable;
    DataTable* fields_ = ui->fieldsTable;
    QLabel* header_ = ui->headerLabel;
    const int keepRow = subframes_->currentRow();
    subframes_->beginFill(0);
    subframes_->endFill();
    for (DataTable* t : {words_, bitmap_, fields_}) { t->beginFill(0); t->endFill(); }
    header_->clear();
    const int ci = channel_->currentIndex();
    if (!readable(Receiver::StageDecoded) || ci < 0 || ci >= int(rx_->nav.size())) return;
    const NavChannel& n = rx_->nav[size_t(ci)];
    setSubframeColumns(isGalileo(n.prn));
    if (isGalileo(n.prn)) {
        header_->setText(tr("   %1: %2 of %3 I/NAV words pass the CRC. %4")
                             .arg(satName(n.prn)).arg(n.parityOkWords).arg(n.parityTotalWords).arg(n.status));
        subframes_->beginFill(int(n.inavWords.size()));
        for (int i = 0; i < int(n.inavWords.size()); ++i) {
            const InavWord& w = n.inavWords[size_t(i)];
            const QColor bg = w.crcOk ? QColor() : badColor();
            subframes_->setNum(i, 0, i, 0, bg);
            subframes_->setKey(i, i);
            subframes_->setNum(i, 1, w.startMs * kE1CodeMs, 0);
            subframes_->setNum(i, 2, w.type, 0);
            subframes_->setText(i, 3, w.hasTow ? QString::number(w.tow, 'f', 0) : QStringLiteral("-"));
            subframes_->setText(i, 4, w.hasTow ? nav::towText(w.tow) : QStringLiteral("-"));
            subframes_->setText(i, 5, w.crcOk ? tr("OK") : tr("FAIL"), w.crcOk ? goodColor() : badColor());
            subframes_->setText(i, 6, w.inverted ? tr("inverted") : tr("normal"));
            subframes_->setText(i, 7, inav::wordDescription(w.type));
        }
        subframes_->endFill();
        if (!n.inavWords.empty()) {
            const int row = liveMode() ? std::clamp(keepRow, 0, int(n.inavWords.size()) - 1) : 0;
            subframes_->selectRow(row);
            showSubframe(row);
        }
        return;
    }
    header_->setText(tr("   PRN %1: %2 subframes, %3/%4 words pass parity. %5")
                         .arg(n.prn).arg(n.subframes.size()).arg(n.parityOkWords).arg(n.parityTotalWords).arg(n.status));
    subframes_->beginFill(int(n.subframes.size()));
    for (int i = 0; i < int(n.subframes.size()); ++i) {
        const Subframe& sf = n.subframes[size_t(i)];
        const QColor bg = sf.allParityOk() ? QColor() : badColor();
        subframes_->setNum(i, 0, i, 0, bg);
        subframes_->setKey(i, i);
        subframes_->setNum(i, 1, sf.startMs, 0);
        subframes_->setNum(i, 2, sf.startBit, 0);
        subframes_->setNum(i, 3, sf.id, 0);
        subframes_->setText(i, 4, sf.page ? QString::number(sf.page) : QStringLiteral("-"));
        subframes_->setNum(i, 5, sf.towCount, 0);
        subframes_->setNum(i, 6, sf.towStart, 0);
        subframes_->setText(i, 7, nav::towText(sf.towStart));
        subframes_->setText(i, 8, QStringLiteral("%1/10").arg(sf.parityOkCount), sf.allParityOk() ? goodColor() : badColor());
        subframes_->setText(i, 9, sf.inverted ? tr("inverted") : tr("normal"));
        subframes_->setText(i, 10, sf.alert ? tr("ALERT") : tr("-"), sf.alert ? badColor() : QColor());
        subframes_->setText(i, 11, sf.antiSpoof ? tr("on") : tr("off"));
        subframes_->setText(i, 12, sf.isf ? tr("1") : tr("0"));
        subframes_->setText(i, 13, sf.towConsistent ? tr("yes") : tr("NO"), sf.towConsistent ? QColor() : badColor());
        subframes_->setText(i, 14, contentOf(sf));
    }
    subframes_->endFill();
    if (!n.subframes.empty()) {
        const int row = liveMode() ? std::clamp(keepRow, 0, int(n.subframes.size()) - 1) : 0;
        subframes_->selectRow(row);
        showSubframe(row);
    }
}

void NavMessagePage::showSubframe(int index)
{
    QComboBox* channel_ = ui->channelCombo;
    DataTable* subframes_ = ui->subframesTable;
    DataTable* words_ = ui->wordsTable;
    DataTable* bitmap_ = ui->bitmapTable;
    DataTable* fields_ = ui->fieldsTable;
    QLabel* header_ = ui->headerLabel;
    (void)subframes_;
    (void)header_;
    const int ci = channel_->currentIndex();
    if (!readable(Receiver::StageDecoded) || ci < 0 || ci >= int(rx_->nav.size())) return;
    const NavChannel& n = rx_->nav[size_t(ci)];
    if (isGalileo(n.prn)) {
        showInavWord(n, index);
        return;
    }
    if (index < 0 || index >= int(n.subframes.size())) return;
    const Subframe& sf = n.subframes[size_t(index)];

    // words
    words_->beginFill(10);
    QFont mono(QStringLiteral("Consolas"));
    mono.setStyleHint(QFont::Monospace);
    for (int w = 0; w < 10; ++w) {
        const NavWord& nw = sf.words[w];
        const QColor bg = nw.parityOk ? QColor() : badColor();
        words_->setNum(w, 0, w + 1, 0, bg);
        words_->setText(w, 1, w == 0 ? tr("TLM") : (w == 1 ? tr("HOW") : tr("Word %1").arg(w + 1)), bg);
        words_->setText(w, 2, bin(nw.raw30, 30), bg);
        words_->setText(w, 3, bin(nw.data24, 24), bg);
        words_->setText(w, 4, QStringLiteral("0x%1").arg(nw.data24, 6, 16, QLatin1Char('0')).toUpper(), bg);
        words_->setText(w, 5, QStringLiteral("%1 %2").arg(nw.d29s).arg(nw.d30s), bg,
                        nw.d30s ? tr("D30* = 1: data bits d1..d24 are the complement of D1..D24") : tr("D30* = 0: data bits transmitted as-is"));
        words_->setText(w, 6, bin(nw.parityRx, 6), bg);
        words_->setText(w, 7, bin(nw.parityCalc, 6), bg);
        words_->setText(w, 8, nw.parityOk ? tr("OK") : tr("FAIL"), nw.parityOk ? goodColor() : badColor());
        for (int c = 2; c <= 7; ++c)
            if (QTableWidgetItem* it = words_->item(w, c)) it->setFont(mono);
    }
    words_->endFill();

    // fields
    const auto fields = nav::decodeFields(sf);
    std::vector<int> owner(300, -1);
    for (int i = 0; i < int(fields.size()); ++i) {
        const NavFieldDef& f = fields[size_t(i)].def;
        for (int b = 0; b < f.l1; ++b) owner[size_t(f.s1 - 1 + b)] = i;
        for (int b = 0; b < f.l2; ++b) owner[size_t(f.s2 - 1 + b)] = i;
    }
    // bit map
    bitmap_->setUpdatesEnabled(false);
    bitmap_->beginFill(10);
    QStringList vh;
    for (int w = 0; w < 10; ++w) vh << (w == 0 ? tr("1 TLM") : (w == 1 ? tr("2 HOW") : QString::number(w + 1)));
    bitmap_->setVerticalHeaderLabels(vh);
    for (int w = 0; w < 10; ++w)
        for (int b = 0; b < 30; ++b) {
            const int pos = 30 * w + b;
            const int o = owner[size_t(pos)];
            const QColor c = o >= 0 ? fieldColor(fields[size_t(o)].def, o) : QColor(255, 255, 255);
            const QString tip = o >= 0 ? QStringLiteral("Bit %1 (word %2, bit %3)\n%4").arg(pos + 1).arg(w + 1).arg(b + 1)
                                                .arg(fields[size_t(o)].def.name)
                                       : QStringLiteral("Bit %1").arg(pos + 1);
            bitmap_->setText(w, b, QString::number(sf.dataBits[size_t(pos)]), sf.words[w].parityOk ? c : c.darker(130), tip);
            bitmap_->item(w, b)->setTextAlignment(Qt::AlignCenter);
        }
    bitmap_->endFill(false);
    for (int c = 0; c < 30; ++c) bitmap_->setColumnWidth(c, 26);
    bitmap_->setUpdatesEnabled(true);

    fields_->beginFill(int(fields.size()));
    for (int i = 0; i < int(fields.size()); ++i) {
        const DecodedField& d = fields[size_t(i)];
        const NavFieldDef& f = d.def;
        const QColor c = fieldColor(f, i);
        fields_->setText(i, 0, f.name, c);
        const int w1 = (f.s1 - 1) / 30 + 1;
        const int w2 = f.l2 > 0 ? (f.s2 - 1) / 30 + 1 : w1;
        fields_->setText(i, 1, w1 == w2 ? QString::number(w1) : QStringLiteral("%1 + %2").arg(w1).arg(w2));
        fields_->setText(i, 2, f.l2 > 0 ? QStringLiteral("%1-%2, %3-%4").arg(f.s1).arg(f.s1 + f.l1 - 1).arg(f.s2).arg(f.s2 + f.l2 - 1)
                                        : QStringLiteral("%1-%2").arg(f.s1).arg(f.s1 + f.l1 - 1));
        fields_->setNum(i, 3, f.totalBits(), 0);
        fields_->setText(i, 4, d.bitString);
        if (QTableWidgetItem* it = fields_->item(i, 4)) it->setFont(mono);
        fields_->setNum(i, 5, double(d.raw), 0);
        fields_->setText(i, 6, f.isSigned ? QString::number(d.rawSigned) : QStringLiteral("-"));
        fields_->setText(i, 7, f.scaleText);
        fields_->setText(i, 8, d.valueText);
        fields_->setText(i, 9, f.units);
        fields_->setText(i, 10, d.interpretation, f.kind == FieldKind::Parity && d.interpretation.contains("FAIL") ? badColor() : QColor());
    }
    fields_->endFill();
}

// One I/NAV word: 128 bits laid out like a GPS subframe, 30 to a row
void NavMessagePage::showInavWord(const NavChannel& n, int index)
{
    if (index < 0 || index >= int(n.inavWords.size())) return;
    DataTable* words_ = ui->wordsTable;
    DataTable* bitmap_ = ui->bitmapTable;
    DataTable* fields_ = ui->fieldsTable;
    const InavWord& w = n.inavWords[size_t(index)];
    const QColor bg = w.crcOk ? QColor() : badColor();
    QFont mono(QStringLiteral("Consolas"));
    mono.setStyleHint(QFont::Monospace);

    QString bitText, hex;
    for (int b = 0; b < 128; ++b) bitText += QLatin1Char(w.bits[size_t(b)] ? '1' : '0');
    for (int b = 0; b < 128; b += 4) hex += QString::number(w.field(b, 4), 16).toUpper();
    words_->beginFill(1);
    words_->setNum(0, 0, w.type, 0, bg);
    words_->setText(0, 1, inav::wordDescription(w.type), bg);
    words_->setText(0, 2, bitText, bg);
    words_->setText(0, 3, QStringLiteral("0x") + hex, bg);
    words_->setText(0, 4, w.crcOk ? tr("CRC OK") : tr("CRC FAIL"), w.crcOk ? goodColor() : badColor());
    if (QTableWidgetItem* it = words_->item(0, 2)) it->setFont(mono);
    words_->endFill();

    const auto fields = inav::decodeFields(w);
    std::vector<int> owner(128, -1);
    for (int i = 0; i < int(fields.size()); ++i)
        for (int b = 0; b < fields[size_t(i)].def.l1; ++b) owner[size_t(fields[size_t(i)].def.s1 - 1 + b)] = i;

    const int rows = 5;
    bitmap_->setUpdatesEnabled(false);
    bitmap_->beginFill(rows);
    QStringList vh;
    for (int r = 0; r < rows; ++r) vh << QString::number(r * 30 + 1);
    bitmap_->setVerticalHeaderLabels(vh);
    for (int pos = 0; pos < 128; ++pos) {
        const int o = owner[size_t(pos)];
        const QColor c = o >= 0 ? fieldColor(fields[size_t(o)].def, o) : QColor(255, 255, 255);
        const QString tip = o >= 0 ? QStringLiteral("Bit %1\n%2").arg(pos + 1).arg(fields[size_t(o)].def.name)
                                   : QStringLiteral("Bit %1").arg(pos + 1);
        bitmap_->setText(pos / 30, pos % 30, QString::number(w.bits[size_t(pos)]), w.crcOk ? c : c.darker(130), tip);
        bitmap_->item(pos / 30, pos % 30)->setTextAlignment(Qt::AlignCenter);
    }
    bitmap_->endFill(false);
    for (int c = 0; c < 30; ++c) bitmap_->setColumnWidth(c, 26);
    bitmap_->setUpdatesEnabled(true);

    fields_->beginFill(int(fields.size()));
    for (int i = 0; i < int(fields.size()); ++i) {
        const DecodedField& d = fields[size_t(i)];
        const NavFieldDef& f = d.def;
        fields_->setText(i, 0, f.name, fieldColor(f, i));
        fields_->setText(i, 1, QString::number(w.type));
        fields_->setText(i, 2, QStringLiteral("%1-%2").arg(f.s1).arg(f.s1 + f.l1 - 1));
        fields_->setNum(i, 3, f.totalBits(), 0);
        fields_->setText(i, 4, d.bitString);
        if (QTableWidgetItem* it = fields_->item(i, 4)) it->setFont(mono);
        fields_->setText(i, 5, f.totalBits() <= 64 ? QString::number(d.raw) : QStringLiteral("-"));
        fields_->setText(i, 6, f.isSigned ? QString::number(d.rawSigned) : QStringLiteral("-"));
        fields_->setText(i, 7, f.scaleText);
        fields_->setText(i, 8, d.valueText);
        fields_->setText(i, 9, f.units);
        fields_->setText(i, 10, d.interpretation);
    }
    fields_->endFill();
}

} // namespace gui
