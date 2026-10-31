#pragma once

#include "Acquisition.h"
#include "NavMessage.h"
#include "Pvt.h"
#include "Settings.h"
#include "SignalSource.h"
#include "Tracking.h"

#include <QObject>
#include <QString>
#include <algorithm>
#include <atomic>
#include <vector>

namespace gnss {

struct LiveSnapshot;

// Owns the signal source and all processing results, and runs the pipeline
// stages. Stages are intended to run on a worker thread; signals are emitted
// from that thread (connections to GUI objects become queued automatically).
class Receiver : public QObject
{
    Q_OBJECT
public:
    enum Stage { StageNone = 0, StageAcquired, StageTracked, StageDecoded, StageSolved };
    Q_ENUM(Stage)

    explicit Receiver(QObject* parent = nullptr);

    ReceiverSettings settings;
    SignalSource source;
    QString liveDescription;       // what is streaming, while results come from a radio

    AcquisitionOutput acq;
    std::vector<TrackingChannel> trk;
    std::vector<NavChannel> nav;
    GlobalNavData gnav;
    PvtOutput pvt;

    int stage() const { return stage_; }
    // Forces the given stage and all later ones to be recomputed by the next run.
    // Call only while no stage is running.
    void invalidateFrom(int stage) { stage_ = std::max(0, std::min(stage_, stage - 1)); }
    double stageSeconds(int s) const { return stageTime_[std::clamp(s, 0, 4)]; }

    bool openFile(const QString& path, QString* error);   // also applies the file-name settings
    bool reopen(QString* error);                          // after a format/skip change
    void closeFile();                                     // lets go of the samples and all results
    void resetResults();
    // Replaces the results with what the live engine has so far (GUI thread)
    void applyLiveSnapshot(LiveSnapshot&& snap);

    bool runAcquisition();
    bool runTracking();
    bool runNavigation();       // bit sync, decoding, ephemeris
    bool runPvt();
    bool runAll();

    void cancel() { cancel_ = true; }
    bool isCancelled() const { return cancel_.load(); }
    void clearCancel() { cancel_ = false; }

    double fileDurationSec() const;
    int channelOfPrn(int prn) const;

signals:
    void logMessage(const QString& text);
    void progressChanged(const QString& stage, int percent);
    void stageCompleted(int stage);

private:
    void log(const QString& t);
    std::atomic<bool> cancel_{false};
    int stage_ = StageNone;
    double stageTime_[5] = {0, 0, 0, 0, 0};
};

} // namespace gnss
