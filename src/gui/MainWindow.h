#pragma once

#include "core/LiveEngine.h"
#include "core/Receiver.h"

#include <QList>
#include <QMainWindow>

class QLabel;
class QProgressBar;
class QThread;

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

namespace gui {

class PageBase;

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override;

    bool openFile(const QString& path);
    void openInitialFile(const QString& commandLinePath);
    // Runs the whole pipeline synchronously and saves a PNG of every page and
    // sub-tab into 'dir' (command line option --screenshots <dir>).
    // With liveSeconds > 0 it replays the open recording for that long and captures the pages as they stand.
    void captureScreenshots(const QString& dir, int liveSeconds = 0, bool usrp = false);

protected:
    void closeEvent(QCloseEvent* e) override;

private slots:
    void onOpen();
    void closeRecording();
    void onSettings();
    void onAbout();
    void onSetupGuide();
    void onLog(const QString& text);
    void onProgress(const QString& stage, int percent);
    void onStageCompleted(int stage);
    void onRunFinished();
    void selectPrnEverywhere(int prn);
    void startLive();
    void stopLive();
    void onLiveSnapshot();
    void onLiveFinished(const QString& reason);

private:
    void startRun(int targetStage, bool fromScratch);
    void refreshPages(int stage);
    void updateActions();
    void refreshVisiblePage();
    void leaveLive();
    void openHelp(const QString& topic);

    Ui::MainWindow* ui;
    gnss::Receiver rx_;
    gnss::LiveEngine live_;
    gnss::AcquisitionOutput pendingAcq_;    // arrived while the pages were frozen
    bool havePendingAcq_ = false;
    QString liveReason_;
    QThread* worker_ = nullptr;
    QProgressBar* progress_ = nullptr;
    QLabel* stageLabel_ = nullptr;
    QList<PageBase*> pages_;
};

} // namespace gui
