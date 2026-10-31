#pragma once

#include "Acquisition.h"
#include "LiveInput.h"
#include "NavMessage.h"
#include "Pvt.h"
#include "Settings.h"
#include "SignalSource.h"
#include "Tracking.h"

#include <QElapsedTimer>
#include <QFile>
#include <QMutex>
#include <QObject>
#include <QString>
#include <QThreadPool>
#include <QVector>
#include <atomic>
#include <map>
#include <memory>
#include <vector>

class QThread;

namespace gnss {

struct LiveChannelStatus
{
    int channel = 0;
    int prn = 0;
    QString state;
    double cn0 = 0;
    double doppler = 0;
    double pli = 0;
    double ageSec = 0;
    int subframes = 0;           // GPS subframes or Galileo words that passed their check
    bool ephemeris = false;
    bool cachedEphemeris = false;
};

struct LiveStatus
{
    bool running = false;
    QString source;
    QString message;
    double sampleRate = 0, centerFreq = 0, gainDb = 0;
    double streamSec = 0;        // seconds of signal received
    double measuredRate = 0;     // samples per second actually arriving
    qint64 samples = 0;
    qint64 overflows = 0, lostSamples = 0;
    double lagMs = 0;            // how far the slowest channel is behind the newest sample
    double bufferUsed = 0;       // the same as a fraction of the sample buffer (0..1)
    double levelRms = 0;         // sample RMS in ADC counts
    double levelDbfs = 0;        // only for a radio
    double clipPercent = 0;
    int acqCycles = 0;
    double lastAcqSec = 0;
    int channels = 0, usable = 0;
    bool haveFix = false;
    double ttffSec = -1;         // stream time of the first fix
    QString recordFile;
    qint64 recordedBytes = 0;
};

// A consistent view of the receiver, handed to the display. Nothing in it is shared with the running engine.
struct LiveSnapshot
{
    double samplingFreq = 0, ifFreq = 0;
    bool hasAcq = false;         // acquisition results are large: only present when they changed
    AcquisitionOutput acq;
    std::vector<TrackingChannel> trk;
    std::vector<NavChannel> nav;
    GlobalNavData gnav;
    PvtOutput pvt;
    std::vector<LiveChannelStatus> channels;
    LiveStatus status;
    int stage = 0;               // a Receiver::Stage
};

// The real-time receiver. Three kinds of thread cooperate:
//   input        receives samples (LiveInput)
//   acquisition  repeatedly searches for satellites without a channel
//   pipeline     advances every channel to the newest sample and, once per update period, decodes the rolling window
//                of tracking data, extends the position solution and publishes a snapshot
// Channels start from an acquisition result and end when they lose the signal or stop producing verified data, so the
// receiver follows the sky as satellites rise and set.
class LiveEngine : public QObject
{
    Q_OBJECT
public:
    explicit LiveEngine(QObject* parent = nullptr);
    ~LiveEngine() override;

    // 'source' is switched to the live stream (or to paced reading of the open file) and is read by the
    // engine until stop(). Any thread may read it at the same time.
    bool start(const ReceiverSettings& settings, SignalSource* source, QString* error);
    void stop();
    // Stops, and frees the sample buffer and everything kept from the last run, which otherwise stay until the next start()
    void release();
    bool isRunning() const { return running_.load(); }

    // Latest snapshot, if there is a new one
    bool takeSnapshot(LiveSnapshot& out);

signals:
    void snapshotReady();
    void logMessage(const QString& text);
    void finished(const QString& reason);

private:
    struct Channel;
    void clearState();
    void pipeline();
    void acquisitionLoop();
    void searchFor(bool galileo);
    void recorderLoop();
    void adoptAcquisitions();
    void advanceChannels(qint64 head);
    void updateChannels();
    void publish(qint64 head, bool last);
    void measureLevel(qint64 head, LiveStatus& st);
    void log(const QString& t) { emit logMessage(t); }

    ReceiverSettings s_;
    SignalSource* src_ = nullptr;
    std::unique_ptr<LiveInput> input_;
    std::shared_ptr<LiveRing> ring_;
    std::atomic<bool> stop_{false};
    std::atomic<bool> running_{false};
    std::atomic<bool> searchNow_{false};
    QThread* pipelineThread_ = nullptr;
    QThread* acqThread_ = nullptr;
    QThread* recordThread_ = nullptr;
    // Channels must keep up with the stream whatever else runs. An acquisition fills the global pool for
    // seconds at a time, so the real-time work has threads of its own.
    QThreadPool pool_;

    std::vector<std::unique_ptr<Channel>> channels_;   // pipeline thread only

    QMutex acqMutex_;                                  // guards the members up to acqDirty_
    QVector<int> trackedSats_;
    double commonDoppler_ = 0;
    bool haveCommonDoppler_ = false;
    std::vector<AcqResult> newlyAcquired_;
    std::map<int, AcqResult> acqBySat_;
    std::vector<double> gpsBins_;
    int acqSamplesPerCode_ = 0, acqNonCoherent_ = 0, acqFftSize_ = 0, acqFftInternal_ = 0;
    qint64 acqStartSample_ = 0;
    double acqElapsed_ = 0;
    int acqCycles_ = 0;
    bool acqDirty_ = false;

    GlobalNavData gnav_;
    PvtOutput pvt_;
    pvt::LiveSolver solver_;
    std::map<int, Ephemeris> ephCache_;
    double ttff_ = -1;
    qint64 lastHead_ = 0;
    qint64 lastTickNs_ = 0;
    QElapsedTimer rateClock_;
    double measuredRate_ = 0;

    QMutex snapMutex_;
    LiveSnapshot latest_;
    bool haveLatest_ = false;

    QFile recordFile_;
    std::atomic<qint64> recordedBytes_{0};
    std::atomic<qint64> recordDropped_{0};
};

} // namespace gnss
