// Tests that drive the real widgets. With QT_QPA_PLATFORM=offscreen they need no display.
//
//   test_gui                                   how Galileo satellites are shown, from made-up results
//   test_gui <file.bin>                        also File > Close recording: after processing a recording and after
//                                              replaying it live, the window must match a freshly started one. The file
//                                              needs a fix within 800 MB, so about 16 MS/s of int8 samples at most
//   test_gui <file.bin> <uhd.dll> <iq.bin>     also the same after a stream, with tests/fake_uhd and a file for it
//
// Set TEST_GUI_SHOTS to a directory to save pictures of the Galileo pages.

#include "AcquisitionPage.h"
#include "DataTable.h"
#include "GuiUtil.h"
#include "LivePage.h"
#include "MainWindow.h"
#include "NavMessagePage.h"
#include "PlotWidget.h"
#include "SkyPlotWidget.h"
#include "core/Constants.h"

#include <QAction>
#include <QApplication>
#include <QComboBox>
#include <QDialog>
#include <QDir>
#include <QElapsedTimer>
#include <QFile>
#include <QFileInfo>
#include <QGroupBox>
#include <QLabel>
#include <QPushButton>
#include <QSet>
#include <QSettings>
#include <QSlider>
#include <QSpinBox>
#include <QTabWidget>
#include <QTemporaryDir>
#include <QThread>
#include <QTimer>

#include <cstdio>
#include <functional>

#ifdef Q_OS_WIN
#include <windows.h>
#include <psapi.h>
#endif

using namespace gnss;
using namespace gui;

namespace {

int failures = 0;

void check(bool ok, const QString& what)
{
    std::printf("%s  %s\n", ok ? "ok    " : "FAILED", qPrintable(what));
    std::fflush(stdout);
    failures += ok ? 0 : 1;
}

void note(const QString& text)
{
    std::printf("\n%s\n", qPrintable(text));
    std::fflush(stdout);
}

double privateMb()
{
#ifdef Q_OS_WIN
    PROCESS_MEMORY_COUNTERS_EX m{};
    GetProcessMemoryInfo(GetCurrentProcess(), reinterpret_cast<PROCESS_MEMORY_COUNTERS*>(&m), sizeof(m));
    return double(m.PrivateUsage) / 1048576.0;
#else
    return 0;
#endif
}

bool waitFor(const std::function<bool()>& done, int ms)
{
    QElapsedTimer t;
    t.start();
    while (!done() && t.elapsed() < ms) {
        QCoreApplication::processEvents(QEventLoop::AllEvents, 50);
        QThread::msleep(10);
    }
    return done();
}

QAction* action(QMainWindow& w, const char* name)
{
    return w.findChild<QAction*>(QLatin1String(name));
}

template <typename T>
T* widget(QWidget& w, const char* name)
{
    return w.findChild<T*>(QLatin1String(name));
}

// Page and name of a widget, to say where a difference is
QString where(const QWidget* w)
{
    for (const QWidget* p = w; p; p = p->parentWidget())
        if (qobject_cast<const PageBase*>(p)) return p->objectName() + QLatin1Char('/') + w->objectName();
    return w->objectName();
}

// The C/A codes are reference data, which closing a recording leaves alone
bool onCodePage(const QWidget* w)
{
    return where(w).startsWith(QLatin1String("caCodePage"));
}

// Everything that could show something of a recording. The spin boxes are left out: start times and lengths are the
// user's settings, and the epoch selectors are checked on their own.
QStringList fingerprint(QMainWindow& w)
{
    QStringList out{QStringLiteral("window title: %1").arg(w.windowTitle())};
    for (QAction* a : w.findChildren<QAction*>())
        if (!a->objectName().isEmpty()) out << QStringLiteral("action %1 enabled %2").arg(a->objectName()).arg(a->isEnabled());
    for (PlotWidget* p : w.findChildren<PlotWidget*>())
        if (!onCodePage(p))
            out << QStringLiteral("plot %1 empty %2 '%3' '%4' '%5'").arg(where(p)).arg(p->isEmpty()).arg(p->title(), p->xLabel(), p->yLabel());
    for (DataTable* t : w.findChildren<DataTable*>()) {
        if (onCodePage(t)) continue;
        QStringList headers;
        for (int c = 0; c < t->columnCount(); ++c) headers << (t->horizontalHeaderItem(c) ? t->horizontalHeaderItem(c)->text() : QString());
        out << QStringLiteral("table %1 rows %2 columns %3").arg(where(t)).arg(t->rowCount()).arg(headers.join(QLatin1Char(',')));
    }
    for (SkyPlotWidget* s : w.findChildren<SkyPlotWidget*>()) out << QStringLiteral("sky %1 sats %2").arg(where(s)).arg(s->satellites().size());
    for (QLabel* l : w.findChildren<QLabel*>())
        if (!l->objectName().isEmpty() && !onCodePage(l)) out << QStringLiteral("label %1 '%2'").arg(where(l), l->text());
    for (QGroupBox* g : w.findChildren<QGroupBox*>())
        if (!onCodePage(g)) out << QStringLiteral("group %1 '%2'").arg(where(g), g->title());
    for (QComboBox* c : w.findChildren<QComboBox*>())
        if (!onCodePage(c)) out << QStringLiteral("combo %1 items %2 '%3'").arg(where(c)).arg(c->count()).arg(c->currentText());
    for (QPushButton* b : w.findChildren<QPushButton*>())
        if (!onCodePage(b)) out << QStringLiteral("button %1 enabled %2").arg(where(b)).arg(b->isEnabled());
    return out;
}

void compareWithFresh(QMainWindow& w, const QStringList& fresh, const QString& what)
{
    // As sets: raising a tab reorders the widgets
    const QStringList now = fingerprint(w);
    const QSet<QString> a(now.begin(), now.end()), b(fresh.begin(), fresh.end());
    for (const QString& s : a - b) std::printf("         left over: %s\n", qPrintable(s));
    for (const QString& s : b - a) std::printf("         fresh:     %s\n", qPrintable(s));
    check(a == b, QStringLiteral("%1: %2 items match a fresh window, %3 differ").arg(what).arg((a & b).size()).arg((a - b).size()));
    for (const char* page : {"satellitesPage", "measurementsPage", "positionPage"}) {
        QWidget* p = w.findChild<QWidget*>(QLatin1String(page));
        auto* spin = p->findChild<QSpinBox*>(QStringLiteral("epochSpin"));
        auto* slider = p->findChild<QSlider*>(QStringLiteral("epochSlider"));
        check(spin->maximum() == 0 && spin->value() == 0 && slider->maximum() == 0, QStringLiteral("%1 epoch selector reset").arg(page));
    }
}

// Pictures of some pages, if asked for
void savePictures(QMainWindow& w, const QString& prefix)
{
    const QString shots = qEnvironmentVariable("TEST_GUI_SHOTS");
    if (shots.isEmpty()) return;
    QDir().mkpath(shots);
    auto* pages = widget<QTabWidget>(w, "pages");
    for (int i : {0, 1, 2, 5, 11}) {
        pages->setCurrentIndex(i);
        QCoreApplication::processEvents();
        w.grab().save(QDir(shots).filePath(QStringLiteral("%1_%2.png").arg(prefix).arg(pages->widget(i)->objectName())));
    }
    pages->setCurrentIndex(0);
    QCoreApplication::processEvents();
}

// Pages fill some things only once they are seen
void visitPages(QMainWindow& w)
{
    auto* pages = widget<QTabWidget>(w, "pages");
    for (int i = 0; i < pages->count(); ++i) {
        pages->setCurrentIndex(i);
        QCoreApplication::processEvents();
        if (auto* sub = pages->widget(i)->findChild<QTabWidget*>())
            for (int k = 0; k < sub->count(); ++k) {
                sub->setCurrentIndex(k);
                QCoreApplication::processEvents();
            }
    }
    pages->setCurrentIndex(0);
    QCoreApplication::processEvents();
}

int filledPlots(QMainWindow& w)
{
    int n = 0;
    for (PlotWidget* p : w.findChildren<PlotWidget*>()) n += !onCodePage(p) && !p->isEmpty() ? 1 : 0;
    return n;
}

void runAll(MainWindow& w)
{
    QAction* run = action(w, "actionRunAll");
    run->trigger();
    check(!action(w, "actionClose")->isEnabled(), QStringLiteral("Close recording is off while processing"));
    check(waitFor([run] { return run->isEnabled(); }, 300000), QStringLiteral("processing finished"));
    visitPages(w);
}

void testBatch(const QString& path, const QStringList& fresh)
{
    note(QStringLiteral("Closing a processed recording"));
    const double start = privateMb();
    MainWindow w;
    w.show();
    check(!action(w, "actionClose")->isEnabled(), QStringLiteral("Close recording is off with nothing open"));
    check(w.openFile(path), QStringLiteral("opened the recording"));
    check(action(w, "actionClose")->isEnabled(), QStringLiteral("Close recording is on with a file open"));
    runAll(w);
    const int filled = filledPlots(w);
    check(filled >= 30, QStringLiteral("%1 plots filled").arg(filled));
    const QString first = widget<DataTable>(w, "solutionsTable")->toCsv(false);
    check(first.count(QLatin1Char('\n')) > 10, QStringLiteral("%1 lines of solutions").arg(first.count(QLatin1Char('\n'))));
    const double full = privateMb();
#ifdef Q_OS_WIN
    check(!QFile::rename(path, path + QStringLiteral(".moved")), QStringLiteral("Windows holds the file while it is open"));
#endif

    action(w, "actionClose")->trigger();
    const double closed = privateMb();
    compareWithFresh(w, fresh, QStringLiteral("after closing"));
    check(closed < start + 0.25 * (full - start), QStringLiteral("private memory %1 MB at start, %2 MB processed, %3 MB closed")
              .arg(start, 0, 'f', 0).arg(full, 0, 'f', 0).arg(closed, 0, 'f', 0));
    check(QFile::rename(path, path + QStringLiteral(".moved")), QStringLiteral("the file is released: it can be moved"));
    QFile::rename(path + QStringLiteral(".moved"), path);
    check(QSettings().contains("lastFile") && QSettings().value("lastFile").toString().isEmpty(), QStringLiteral("no last file remembered"));
    savePictures(w, QStringLiteral("closed"));
    visitPages(w);
    compareWithFresh(w, fresh, QStringLiteral("after looking at every page"));

    // Nothing may carry over into the next recording
    check(w.openFile(path), QStringLiteral("opened it again"));
    runAll(w);
    check(widget<DataTable>(w, "solutionsTable")->toCsv(false) == first, QStringLiteral("processing it again gives identical solutions"));
    action(w, "actionClose")->trigger();
    compareWithFresh(w, fresh, QStringLiteral("after closing again"));

    // With the file closed, changing the sample format must not bring it back. It used to, or show an empty warning.
    bool warned = false;
    QTimer::singleShot(0, &w, [] {
        auto* dlg = qobject_cast<QDialog*>(QApplication::activeModalWidget());
        auto* format = dlg->findChild<QComboBox*>(QStringLiteral("formatCombo"));
        format->setCurrentIndex((format->currentIndex() + 1) % format->count());
        dlg->accept();
    });
    QTimer::singleShot(3000, &w, [&warned] {
        if (auto* box = qobject_cast<QDialog*>(QApplication::activeModalWidget())) {
            warned = true;
            box->reject();
        }
    });
    action(w, "actionSettings")->trigger();
    check(!warned, QStringLiteral("changing the format with nothing open shows no warning"));
    compareWithFresh(w, fresh, QStringLiteral("after changing settings"));
    QSettings qs;
    qs.remove(QStringLiteral("receiver"));
}

void testNextStart(const QString& path)
{
    note(QStringLiteral("Starting again after a close"));
    QDir::setCurrent(QFileInfo(path).absolutePath());    // the search for a recording would find this one
    {
        MainWindow w;
        w.openInitialFile(QString());
        check(!action(w, "actionClose")->isEnabled(), QStringLiteral("starts with nothing open"));
    }
    QSettings().remove(QStringLiteral("lastFile"));
    {
        MainWindow w;
        w.openInitialFile(QString());
        check(action(w, "actionClose")->isEnabled(), QStringLiteral("the very first start still finds a recording"));
    }
    QSettings().remove(QStringLiteral("lastFile"));
}

void stopLive(MainWindow& w)
{
    action(w, "actionStop")->trigger();
    check(waitFor([&w] { return action(w, "actionStartLive")->isEnabled(); }, 30000), QStringLiteral("live run stopped"));
}

void testReplay(const QString& path, const QStringList& fresh)
{
    note(QStringLiteral("Closing after a real-time replay"));
    QSettings().setValue("live/source", int(LiveSourceKind::Replay));
    MainWindow w;
    w.show();
    check(w.openFile(path), QStringLiteral("opened the recording"));
    action(w, "actionStartLive")->trigger();
    check(!action(w, "actionClose")->isEnabled(), QStringLiteral("Close recording is off while live"));
    QLabel* fix = w.findChild<QWidget*>(QStringLiteral("livePage"))->findChild<QLabel*>(QStringLiteral("fixValue"));
    check(waitFor([fix] { return fix->text() == QLatin1String("Current"); }, 90000), QStringLiteral("live fix reached"));
    stopLive(w);
    check(filledPlots(w) >= 4, QStringLiteral("%1 plots filled").arg(filledPlots(w)));
    check(action(w, "actionClose")->isEnabled(), QStringLiteral("Close recording is on after stopping"));
    action(w, "actionClose")->trigger();
    compareWithFresh(w, fresh, QStringLiteral("after closing a replay"));
    QSettings().remove(QStringLiteral("live"));
}

void testStream(const QString& dll, const QString& iq, const QStringList& fresh)
{
    note(QStringLiteral("Closing after a stream from the stand-in radio"));
    qputenv("FAKE_UHD_FILE", QFile::encodeName(iq));
    qputenv("FAKE_UHD_RATE", "4774000");
    QSettings qs;
    qs.setValue("live/source", int(LiveSourceKind::Usrp));
    qs.setValue("live/uhdLibrary", dll);
    qs.setValue("live/sampleRate", 4.774e6);
    qs.sync();
    const double start = privateMb();
    MainWindow w;
    w.show();
    action(w, "actionStartLive")->trigger();
    auto* stream = w.findChild<QWidget*>(QStringLiteral("livePage"))->findChild<QLabel*>(QStringLiteral("streamValue"));
    check(waitFor([stream] { return stream->text().startsWith(QLatin1String("8.")); }, 30000), QStringLiteral("stream running"));
    stopLive(w);
    const double full = privateMb();
    check(action(w, "actionClose")->isEnabled(), QStringLiteral("Close recording is on for a stopped stream"));
    action(w, "actionClose")->trigger();
    const double closed = privateMb();
    check(closed < start + 0.25 * (full - start), QStringLiteral("private memory %1 MB at start, %2 MB streamed, %3 MB closed")
              .arg(start, 0, 'f', 0).arg(full, 0, 'f', 0).arg(closed, 0, 'f', 0));
    compareWithFresh(w, fresh, QStringLiteral("after closing a stream"));
    qs.remove(QStringLiteral("live"));
}

SatObservation observation(int prn, double azDeg, double elDeg, double cn0)
{
    SatObservation o;
    o.prn = prn;
    o.available = o.used = true;
    o.geomRange = 2.2e7;
    o.az = azDeg * kPi / 180;
    o.el = elDeg * kPi / 180;
    o.cn0 = cn0;
    return o;
}

void testGalileoLabels()
{
    note(QStringLiteral("Telling Galileo from GPS"));
    check(satAxisText(5) == QLatin1String("5") && satAxisText(32) == QLatin1String("32"), QStringLiteral("chart axis: GPS keeps its PRN"));
    check(satAxisText(36) == QLatin1String("E01") && satAxisText(40) == QLatin1String("E05") && satAxisText(71) == QLatin1String("E36"),
          QStringLiteral("chart axis: Galileo is named E01 to E36"));
    check(satAxisText(35).isEmpty() && satAxisText(72).isEmpty() && satAxisText(2.5).isEmpty(), QStringLiteral("chart axis: nothing between satellites"));
    const int e24 = kGalileoIdBase + 24, e05 = kGalileoIdBase + 5;
    check(satFullName(24) == QLatin1String("GPS PRN 24") && satFullName(e24) == QLatin1String("Galileo E24"), QStringLiteral("tool tip names"));

    Receiver rx;
    setReadableStage(Receiver::StageSolved);
    NavSolution fix;
    fix.valid = true;
    fix.nUsed = 4;
    fix.obs = {observation(24, 40, 60, 47), observation(e24, 150, 45, 44), observation(3, 250, 30, 42), observation(e05, 320, 20, 38)};
    rx.pvt.sols.push_back(fix);

    LiveStatus st;
    st.running = true;
    st.sampleRate = 5e6;
    st.streamSec = 42;
    std::vector<LiveChannelStatus> channels;
    int n = 0;
    for (int prn : {3, 24, e05, e24}) {
        LiveChannelStatus c;
        c.channel = n++;
        c.prn = prn;
        c.state = QStringLiteral("Ready");
        c.cn0 = 44;
        channels.push_back(c);
    }
    LivePage live;
    live.setReceiver(&rx);
    live.resize(1400, 900);
    live.show();
    live.setStatus(st, channels);
    live.refresh();
    auto* table = live.findChild<DataTable*>(QStringLiteral("channelsTable"));
    for (int r = 0; r < table->rowCount(); ++r) {
        const int prn = table->keyOfRow(r).toInt();
        check(table->item(r, 2)->text() == satSystem(prn), QStringLiteral("Live table: %1 is %2").arg(table->item(r, 1)->text(), table->item(r, 2)->text()));
        check(table->item(r, 3)->text() == QLatin1String("Ready") && table->item(r, 12)->text() == QLatin1String("yes"), QStringLiteral("Live table: later columns line up"));
    }
    QVector<int> ids;
    for (const SkyPlotWidget::Sat& s : live.findChild<SkyPlotWidget*>(QStringLiteral("skyPlot"))->satellites()) ids << s.prn;
    check(ids.contains(24) && ids.contains(e24), QStringLiteral("Live sky plot keeps E24 and PRN 24 apart"));
    check(live.findChild<QLabel*>(QStringLiteral("stateLabel"))->text().contains(QLatin1String("2 GPS, 2 Galileo and 0 GLONASS")), QStringLiteral("Live state line counts each system"));

    rx.acq.samplesPerCode = 5000;
    for (int prn : {3, 24, e05, e24}) {
        AcqResult r;
        r.prn = prn;
        r.acquired = true;
        r.metric = 8;
        r.cn0 = 45;
        rx.acq.results.push_back(r);
    }
    AcquisitionPage acq;
    acq.setReceiver(&rx);
    acq.resize(1400, 900);
    acq.show();
    acq.refresh();
    auto* results = acq.findChild<DataTable*>(QStringLiteral("resultsTable"));
    bool systems = results->rowCount() == 4;
    for (int r = 0; r < results->rowCount(); ++r)
        systems = systems && results->item(r, 1)->text() == satSystem(results->keyOfRow(r).toInt())
                  && results->item(r, 2)->text() == QLatin1String("yes");
    check(systems, QStringLiteral("Acquisition table has the system, and later columns line up"));
    QCoreApplication::processEvents();

    // A Galileo channel rewrites this page's wording, which must not outlive it
    NavMessagePage message;
    message.setReceiver(&rx);
    setReadableStage(Receiver::StageNone);
    message.refresh();
    auto* group = message.findChild<QGroupBox*>(QStringLiteral("subframesGroup"));
    const QString before = group->title();
    NavChannel channel;
    channel.prn = e24;
    rx.nav.push_back(channel);
    setReadableStage(Receiver::StageDecoded);
    message.refresh();
    check(group->title() != before, QStringLiteral("Navigation page words a Galileo channel differently"));
    setReadableStage(Receiver::StageNone);
    message.clearContents();
    message.refresh();
    check(group->title() == before, QStringLiteral("clearing returns the page to its own wording"));
    setReadableStage(Receiver::StageSolved);

    const QString shots = qEnvironmentVariable("TEST_GUI_SHOTS");
    if (!shots.isEmpty()) {
        QDir().mkpath(shots);
        live.grab().save(QDir(shots).filePath(QStringLiteral("galileo_live.png")));
        acq.grab().save(QDir(shots).filePath(QStringLiteral("galileo_acquisition.png")));
    }
    setReadableStage(Receiver::StageNone);
}

} // namespace

int main(int argc, char** argv)
{
    QApplication app(argc, argv);
    QTemporaryDir temp;
    // Keeps the tests away from the settings of the installed program
    QSettings::setDefaultFormat(QSettings::IniFormat);
    QSettings::setPath(QSettings::IniFormat, QSettings::UserScope, temp.path());
    QCoreApplication::setOrganizationName(QStringLiteral("SoftGpsReceiverTest"));
    QCoreApplication::setApplicationName(QStringLiteral("test_gui"));

    testGalileoLabels();

    if (argc >= 2) {
        QStringList fresh;
        {
            MainWindow w;
            w.show();
            fresh = fingerprint(w);
        }
        // Closing is tested on a copy, as it moves the file about
        const QString copy = QDir(temp.path()).filePath(QFileInfo(QString::fromLocal8Bit(argv[1])).fileName());
        QFile in(QString::fromLocal8Bit(argv[1])), out(copy);
        if (!in.open(QIODevice::ReadOnly) || !out.open(QIODevice::WriteOnly)) {
            std::printf("cannot copy %s\n", argv[1]);
            return 2;
        }
        for (qint64 left = 800LL * 1024 * 1024; left > 0;) {
            const QByteArray block = in.read(std::min<qint64>(left, 64 * 1024 * 1024));
            if (block.isEmpty()) break;
            out.write(block);
            left -= block.size();
        }
        out.close();
        testBatch(copy, fresh);
        testNextStart(copy);
        testReplay(copy, fresh);
        if (argc >= 4) testStream(QString::fromLocal8Bit(argv[2]), QString::fromLocal8Bit(argv[3]), fresh);
    }

    std::printf("\n%d failure(s)\n", failures);
    return failures ? 1 : 0;
}
