#include "MainWindow.h"
#include "ui_MainWindow.h"

#include "AcquisitionPage.h"
#include "BitSyncPage.h"
#include "CaCodePage.h"
#include "EphemerisPage.h"
#include "GuiUtil.h"
#include "LivePage.h"
#include "MeasurementsPage.h"
#include "NavMessagePage.h"
#include "OverviewPage.h"
#include "PlotWidget.h"
#include "PositionPage.h"
#include "RawSignalPage.h"
#include "SatellitesPage.h"
#include "SettingsDialog.h"
#include "TrackingPage.h"

#include <QApplication>
#include <QCloseEvent>
#include <QCoreApplication>
#include <QDesktopServices>
#include <QDateTime>
#include <QDir>
#include <QElapsedTimer>
#include <QEventLoop>
#include <QFileDialog>
#include <QFileInfo>
#include <QLabel>
#include <QMessageBox>
#include <QProgressBar>
#include <QSettings>
#include <QStyle>
#include <QThread>
#include <QTimer>
#include <QUrl>

using namespace gnss;

namespace gui {

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    pages_ = {ui->overviewPage, ui->livePage, ui->rawSignalPage, ui->caCodePage, ui->acquisitionPage, ui->trackingPage,
              ui->bitSyncPage, ui->navMessagePage, ui->ephemerisPage, ui->satellitesPage, ui->measurementsPage,
              ui->positionPage};
    for (PageBase* p : pages_) p->setReceiver(&rx_);

    // icons from the platform style (no external resources required)
    QStyle* st = style();
    ui->actionOpen->setIcon(st->standardIcon(QStyle::SP_DialogOpenButton));
    ui->actionClose->setIcon(st->standardIcon(QStyle::SP_DialogCloseButton));
    ui->actionSettings->setIcon(st->standardIcon(QStyle::SP_FileDialogDetailedView));
    ui->actionRunAll->setIcon(st->standardIcon(QStyle::SP_MediaPlay));
    ui->actionAcquire->setIcon(st->standardIcon(QStyle::SP_MediaSeekForward));
    ui->actionTrack->setIcon(st->standardIcon(QStyle::SP_MediaSeekForward));
    ui->actionDecode->setIcon(st->standardIcon(QStyle::SP_MediaSeekForward));
    ui->actionSolve->setIcon(st->standardIcon(QStyle::SP_MediaSeekForward));
    ui->actionStartLive->setIcon(st->standardIcon(QStyle::SP_DriveNetIcon));
    ui->actionStop->setIcon(st->standardIcon(QStyle::SP_MediaStop));
    ui->actionAbout->setIcon(st->standardIcon(QStyle::SP_MessageBoxInformation));
    ui->menuView->addAction(ui->logDock->toggleViewAction());
    ui->menuView->addAction(ui->toolBar->toggleViewAction());

    stageLabel_ = new QLabel(tr("Ready"));
    progress_ = new QProgressBar;
    progress_->setRange(0, 100);
    progress_->setMaximumWidth(300);
    progress_->setVisible(false);
    ui->statusbar->addWidget(stageLabel_, 1);
    ui->statusbar->addPermanentWidget(progress_);

    connect(ui->actionOpen, &QAction::triggered, this, &MainWindow::onOpen);
    connect(ui->actionClose, &QAction::triggered, this, &MainWindow::closeRecording);
    connect(ui->actionSettings, &QAction::triggered, this, &MainWindow::onSettings);
    connect(ui->actionRunAll, &QAction::triggered, this, [this] { startRun(Receiver::StageSolved, true); });
    connect(ui->actionAcquire, &QAction::triggered, this, [this] { startRun(Receiver::StageAcquired, true); });
    connect(ui->actionTrack, &QAction::triggered, this, [this] { startRun(Receiver::StageTracked, false); });
    connect(ui->actionDecode, &QAction::triggered, this, [this] { startRun(Receiver::StageDecoded, false); });
    connect(ui->actionSolve, &QAction::triggered, this, [this] { startRun(Receiver::StageSolved, false); });
    connect(ui->actionStartLive, &QAction::triggered, this, &MainWindow::startLive);
    connect(ui->actionStop, &QAction::triggered, this, [this] {
        if (liveMode()) {
            stopLive();
            return;
        }
        rx_.cancel();
        onLog(tr("Stop requested"));
    });
    connect(ui->livePage, &LivePage::startRequested, this, &MainWindow::startLive);
    connect(ui->livePage, &LivePage::stopRequested, this, &MainWindow::stopLive);
    connect(ui->livePage, &LivePage::settingsRequested, this, &MainWindow::onSettings);
    connect(ui->livePage, &LivePage::prnSelected, this, &MainWindow::selectPrnEverywhere);
    connect(ui->pages, &QTabWidget::currentChanged, this, [this] {
        if (liveMode() && !ui->livePage->frozen()) refreshVisiblePage();
    });
    connect(&live_, &LiveEngine::logMessage, this, &MainWindow::onLog, Qt::QueuedConnection);
    connect(&live_, &LiveEngine::snapshotReady, this, &MainWindow::onLiveSnapshot, Qt::QueuedConnection);
    connect(&live_, &LiveEngine::finished, this, &MainWindow::onLiveFinished, Qt::QueuedConnection);
    connect(ui->actionExportCsv, &QAction::triggered, ui->positionPage, &PositionPage::exportCsv);
    connect(ui->actionExportKml, &QAction::triggered, ui->positionPage, &PositionPage::exportKml);
    connect(ui->actionAbout, &QAction::triggered, this, &MainWindow::onAbout);
    connect(ui->actionSetupGuide, &QAction::triggered, this, &MainWindow::onSetupGuide);
    connect(ui->actionAlgorithms, &QAction::triggered, this, [this] { ui->pages->setCurrentWidget(ui->overviewPage); });
    connect(ui->overviewPage, &OverviewPage::prnSelected, this, &MainWindow::selectPrnEverywhere);

    connect(&rx_, &Receiver::logMessage, this, &MainWindow::onLog, Qt::QueuedConnection);
    connect(&rx_, &Receiver::progressChanged, this, &MainWindow::onProgress, Qt::QueuedConnection);
    connect(&rx_, &Receiver::stageCompleted, this, &MainWindow::onStageCompleted, Qt::QueuedConnection);

    QSettings qs;
    rx_.settings.load(qs);
    restoreGeometry(qs.value("window/geometry").toByteArray());
    restoreState(qs.value("window/state").toByteArray());
    updateActions();
    refreshPages(-1);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::openInitialFile(const QString& commandLinePath)
{
    if (!commandLinePath.isEmpty() && openFile(commandLinePath)) return;
    QSettings qs;
    const QString last = qs.value("lastFile").toString();
    if (!last.isEmpty() && QFileInfo::exists(last) && openFile(last)) return;
    // Look for a recording next to (or a few levels above) the executable and in the working directory
    QStringList dirs{QDir::currentPath()};
    QDir d(QCoreApplication::applicationDirPath());
    for (int i = 0; i < 5; ++i) {
        dirs << d.absolutePath();
        if (!d.cdUp()) break;
    }
    if (qs.contains("lastFile") && last.isEmpty()) dirs.clear();    // closed by the user: stay empty
    for (const QString& dir : dirs) {
        const QFileInfoList files = QDir(dir).entryInfoList({"*fs*if*.bin", "*.bin"}, QDir::Files, QDir::Size | QDir::Reversed);
        for (const QFileInfo& f : files)
            if (f.size() > 10 * 1024 * 1024) {
                openFile(f.absoluteFilePath());
                return;
            }
    }
    onLog(tr("Use File > Open to load a recording. Sampling and IF frequencies are read from file names such as "
             "\"...-fs38_192-if9_55.bin\" (MHz)."));
}

bool MainWindow::openFile(const QString& path)
{
    if (worker_ || liveMode()) return false;
    QString err;
    if (!rx_.openFile(path, &err)) {
        QMessageBox::warning(this, tr("Open recording"), err);
        return false;
    }
    setReadableStage(Receiver::StageNone);
    QSettings().setValue("lastFile", path);
    setWindowTitle(tr("Software GPS Receiver - %1").arg(QFileInfo(path).fileName()));
    stageLabel_->setText(tr("Opened %1 - press F5 (Run all) to process").arg(QFileInfo(path).fileName()));
    refreshPages(-1);
    updateActions();
    return true;
}

void MainWindow::onOpen()
{
    QSettings qs;
    const QString dir = qs.value("lastDir", QFileInfo(rx_.settings.fileName).absolutePath()).toString();
    const QString fn = QFileDialog::getOpenFileName(this, tr("Open recorded IF samples"), dir,
                                                    tr("Sample files (*.bin *.dat *.raw *.bb *.int8 *.iq);;All files (*)"));
    if (fn.isEmpty()) return;
    qs.setValue("lastDir", QFileInfo(fn).absolutePath());
    openFile(fn);
}

void MainWindow::closeRecording()
{
    if (worker_ || liveMode()) return;
    const QString name = rx_.source.isLive() ? tr("the live stream") : QFileInfo(rx_.source.path()).fileName();
    live_.release();
    rx_.closeFile();
    pendingAcq_ = AcquisitionOutput();
    havePendingAcq_ = false;
    setReadableStage(Receiver::StageNone);
    for (PageBase* p : pages_) p->clearContents();
    QSettings().setValue("lastFile", QString());
    setWindowTitle(tr("Software GPS Receiver"));
    stageLabel_->setText(tr("Ready"));
    onLog(tr("Closed %1: results, plots and sample buffers cleared").arg(name));
    refreshPages(-1);
    updateActions();
}

void MainWindow::onSettings()
{
    if (worker_ || liveMode()) return;
    SettingsDialog dlg(rx_.settings, this);
    if (dlg.exec() != QDialog::Accepted) return;
    const ReceiverSettings ns = dlg.settings();
    const bool reopen = rx_.source.isOpen() && !rx_.source.isLive() && (ns.format != rx_.settings.format || ns.skipBytes != rx_.settings.skipBytes);
    rx_.settings = ns;
    QSettings qs;
    rx_.settings.save(qs);
    if (reopen) {
        QString err;
        if (!rx_.reopen(&err)) QMessageBox::warning(this, tr("Settings"), err);
    } else {
        rx_.resetResults();
    }
    setReadableStage(Receiver::StageNone);
    onLog(tr("Settings changed: %1 - results cleared").arg(rx_.settings.summary()));
    refreshPages(-1);
    updateActions();
}

void MainWindow::startRun(int target, bool fromScratch)
{
    if (worker_ || liveMode()) return;
    if (!rx_.source.isOpen()) {
        onOpen();
        if (!rx_.source.isOpen()) return;
    }
    // first stage that must be (re)computed
    int from = fromScratch ? int(Receiver::StageAcquired) : std::min(target, readableStage() + 1);
    from = std::max(from, int(Receiver::StageAcquired));
    rx_.invalidateFrom(from);
    setReadableStage(from - 1);
    refreshPages(-1);
    setWorkerRunning(true);
    worker_ = QThread::create([this, target] {
        switch (target) {
        case Receiver::StageAcquired: rx_.runAcquisition(); break;
        case Receiver::StageTracked: rx_.runTracking(); break;
        case Receiver::StageDecoded: rx_.runNavigation(); break;
        default: rx_.runPvt(); break;
        }
    });
    connect(worker_, &QThread::finished, this, &MainWindow::onRunFinished);
    progress_->setValue(0);
    progress_->setVisible(true);
    stageLabel_->setText(tr("Processing..."));
    updateActions();
    worker_->start();
}

void MainWindow::onRunFinished()
{
    worker_->deleteLater();
    worker_ = nullptr;
    setWorkerRunning(false);
    progress_->setVisible(false);
    if (rx_.stage() > readableStage()) setReadableStage(rx_.stage());
    stageLabel_->setText(rx_.isCancelled() ? tr("Stopped") : tr("Finished (stage %1 of 4)").arg(readableStage()));
    ui->overviewPage->refresh();
    refreshPages(Receiver::StageSolved);
    updateActions();
}

void MainWindow::onStageCompleted(int stage)
{
    if (stage > readableStage()) setReadableStage(stage);
    refreshPages(stage);
    ui->overviewPage->refresh();
}

void MainWindow::startLive()
{
    if (worker_ || liveMode()) return;
    live_.stop();                     // threads of a run that ended by itself
    if (rx_.settings.live.source == LiveSourceKind::Replay && (!rx_.source.isOpen() || rx_.source.isLive())) {
        onOpen();
        if (!rx_.source.isOpen() || rx_.source.isLive()) return;
    }
    rx_.resetResults();
    setReadableStage(Receiver::StageNone);
    ui->livePage->clearContents();
    QString err;
    if (!live_.start(rx_.settings, &rx_.source, &err)) {
        ui->livePage->setMessage(tr("Could not start"));
        onLog(err);
        if (rx_.settings.live.source == LiveSourceKind::Usrp)
            err += tr("\n\nTo try live mode without a radio, open a recording and set Live source to "
                      "\"Replay the open recording in real time\" in Settings > Live.");
        QMessageBox::warning(this, tr("Live"), err);
        updateActions();
        return;
    }
    setLiveMode(true);
    PlotWidget::setKeepUserView(true);
    rx_.liveDescription = rx_.settings.live.source == LiveSourceKind::Usrp ? tr("Live stream from %1").arg(rx_.settings.live.deviceArgs)
                                                                           : QString();
    ui->livePage->setRunning(true);
    ui->livePage->setMessage(tr("Starting..."));
    ui->pages->setCurrentWidget(ui->livePage);
    stageLabel_->setText(tr("Live"));
    refreshPages(-1);
    updateActions();
}

void MainWindow::stopLive()
{
    if (!liveMode()) return;
    live_.stop();                     // onLiveFinished follows with the final snapshot
}

void MainWindow::onLiveSnapshot()
{
    LiveSnapshot snap;
    if (!live_.takeSnapshot(snap)) return;
    const LiveStatus status = snap.status;
    ui->livePage->setStatus(status, snap.channels);
    if (ui->livePage->frozen() && status.running) {
        if (snap.hasAcq) {
            pendingAcq_ = std::move(snap.acq);
            havePendingAcq_ = true;
        }
        return;
    }
    if (havePendingAcq_ && !snap.hasAcq) {
        snap.acq = std::move(pendingAcq_);
        snap.hasAcq = true;
    }
    havePendingAcq_ = false;
    const int stage = snap.stage;
    rx_.applyLiveSnapshot(std::move(snap));
    setReadableStage(stage);
    refreshVisiblePage();
}

void MainWindow::refreshVisiblePage()
{
    QElapsedTimer t;
    t.start();
    if (auto* p = qobject_cast<PageBase*>(ui->pages->currentWidget())) p->refresh();
    stageLabel_->setText(tr("Live - display refresh %1 ms").arg(t.elapsed()));
}

void MainWindow::onLiveFinished(const QString& reason)
{
    liveReason_ = reason;
    live_.stop();
    onLiveSnapshot();                 // whatever arrived last
    leaveLive();
    ui->livePage->setMessage(tr("Stopped: %1").arg(reason));
    stageLabel_->setText(tr("Live stopped: %1").arg(reason));
}

void MainWindow::leaveLive()
{
    setLiveMode(false);
    PlotWidget::setKeepUserView(false);
    ui->livePage->setRunning(false);
    refreshVisiblePage();
    updateActions();
}

void MainWindow::refreshPages(int stage)
{
    QApplication::setOverrideCursor(Qt::WaitCursor);
    for (PageBase* p : pages_)
        if (stage < 0 || p->stage() == stage) p->refresh();
    QApplication::restoreOverrideCursor();
}

void MainWindow::updateActions()
{
    const bool live = liveMode();
    const bool busy = worker_ != nullptr || live;
    const bool open = rx_.source.isOpen() && !rx_.source.isLive();   // the batch stages need a whole recording
    ui->actionOpen->setEnabled(!busy);
    ui->actionClose->setEnabled(!busy && rx_.source.isOpen());
    ui->actionSettings->setEnabled(!busy);
    ui->actionStartLive->setEnabled(!busy);
    ui->actionRunAll->setEnabled(!busy && open);
    ui->actionAcquire->setEnabled(!busy && open);
    ui->actionTrack->setEnabled(!busy && open);
    ui->actionDecode->setEnabled(!busy && open);
    ui->actionSolve->setEnabled(!busy && open);
    ui->actionStop->setEnabled(busy);
    const bool solved = !busy && readableStage() >= Receiver::StageSolved;
    ui->actionExportCsv->setEnabled(solved);
    ui->actionExportKml->setEnabled(solved);
}

void MainWindow::onLog(const QString& text)
{
    ui->logView->appendPlainText(QStringLiteral("[%1] %2").arg(QDateTime::currentDateTime().toString("hh:mm:ss.zzz"), text));
}

void MainWindow::onProgress(const QString& stage, int percent)
{
    stageLabel_->setText(tr("%1...").arg(stage));
    progress_->setValue(percent);
}

void MainWindow::selectPrnEverywhere(int prn)
{
    ui->acquisitionPage->selectPrn(prn);
    ui->trackingPage->selectPrn(prn);
    ui->bitSyncPage->selectPrn(prn);
    ui->navMessagePage->selectPrn(prn);
    ui->satellitesPage->selectPrn(prn);
    ui->pages->setCurrentWidget(readableStage() >= Receiver::StageTracked ? static_cast<QWidget*>(ui->trackingPage)
                                                                           : static_cast<QWidget*>(ui->acquisitionPage));
}

void MainWindow::captureScreenshots(const QString& dir, int liveSeconds, bool usrp)
{
    if (!rx_.source.isOpen() && !usrp) return;
    QDir().mkpath(dir);
    if (liveSeconds > 0) {
        rx_.settings.live.source = usrp ? LiveSourceKind::Usrp : LiveSourceKind::Replay;
        startLive();
        QEventLoop loop;
        QTimer::singleShot(liveSeconds * 1000, &loop, &QEventLoop::quit);
        loop.exec();
    } else {
        // Same threaded path as "Run all", including the per-stage page refreshes
        startRun(Receiver::StageSolved, true);
        if (worker_ && !worker_->isFinished()) {
            QEventLoop loop;
            connect(worker_, &QThread::finished, &loop, &QEventLoop::quit);
            loop.exec();
        }
    }
    QCoreApplication::processEvents();
    auto safe = [](QString s) { return s.replace('&', "").replace('/', '-').replace(' ', '_'); };
    int shot = 0;
    for (int i = 0; i < ui->pages->count(); ++i) {
        ui->pages->setCurrentIndex(i);
        QCoreApplication::processEvents();
        QWidget* page = ui->pages->widget(i);
        QTabWidget* sub = page->findChild<QTabWidget*>();
        const int nSub = sub ? sub->count() : 1;
        for (int k = 0; k < nSub; ++k) {
            if (sub) sub->setCurrentIndex(k);
            QCoreApplication::processEvents();
            QString name = QStringLiteral("%1_%2").arg(++shot, 2, 10, QLatin1Char('0')).arg(safe(ui->pages->tabText(i)));
            if (sub) name += QLatin1Char('_') + safe(sub->tabText(k));
            if (liveSeconds > 0) {                 // let a fresh snapshot reach this page
                QEventLoop loop;
                QTimer::singleShot(1200, &loop, &QEventLoop::quit);
                loop.exec();
            }
            grab().save(QDir(dir).filePath(name + QStringLiteral(".png")));
        }
        if (sub) sub->setCurrentIndex(0);
    }
    ui->pages->setCurrentIndex(0);
    if (liveSeconds > 0) {
        stopLive();
        QCoreApplication::processEvents();
    }
}

void MainWindow::onSetupGuide()
{
    const QString chm = QDir(QCoreApplication::applicationDirPath()).filePath(QStringLiteral("SoftGpsReceiver.chm"));
    if (!QFileInfo::exists(chm) || !QDesktopServices::openUrl(QUrl::fromLocalFile(chm)))
        QMessageBox::warning(this, tr("Help"), tr("Could not open %1. Building it needs Microsoft HTML Help Workshop.").arg(chm));
}

void MainWindow::onAbout()
{
    QMessageBox::about(this, tr("About Software GPS Receiver"),
                       tr("<h3>Software GPS Receiver</h3>"
                          "<p>A GPS L1 C/A and Galileo E1 receiver for recorded IF samples or a live USRP, built to expose every "
                          "step of the signal processing: raw signal statistics, C/A code generation, FFT-based "
                          "acquisition, FLL-assisted PLL / DLL tracking, bit synchronisation, LNAV decoding with "
                          "parity, ephemeris/almanac/health, satellite orbits, pseudoranges and least-squares "
                          "position, velocity and time.</p>"
                          "<p>Reference: IS-GPS-200; Galileo OS SIS ICD Issue 2.1; K. Borre et al., <i>A Software-Defined GPS and Galileo Receiver</i>; "
                          "E. D. Kaplan, <i>Understanding GPS</i>.</p>"
                          "<p>Built with Qt %1.</p>").arg(QT_VERSION_STR));
}

void MainWindow::closeEvent(QCloseEvent* e)
{
    live_.stop();
    if (worker_) {
        rx_.cancel();
        worker_->wait();
    }
    QSettings qs;
    qs.setValue("window/geometry", saveGeometry());
    qs.setValue("window/state", saveState());
    rx_.settings.save(qs);
    e->accept();
}

} // namespace gui
