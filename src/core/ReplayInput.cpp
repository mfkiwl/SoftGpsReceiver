#include "LiveInput.h"

#include <QElapsedTimer>
#include <QFileInfo>
#include <QThread>

namespace gnss {

ReplayInput::ReplayInput(SignalSource* source, const ReceiverSettings& s)
    : source_(source), fs_(s.samplingFreq), speed_(std::max(0.01, s.live.replaySpeed))
{
    info_.sampleRate = fs_;
    info_.centerFreq = 0;
    info_.description = QStringLiteral("Replay of %1 at %2x real time").arg(QFileInfo(source->path()).fileName()).arg(speed_);
}

ReplayInput::~ReplayInput()
{
    stop();
}

bool ReplayInput::start(QString* error)
{
    if (!source_->isOpen() || source_->isLive()) {
        if (error) *error = QStringLiteral("Open a recording first: replay plays the file that is open.");
        return false;
    }
    limit_ = 0;
    source_->setVisibleLimit(&limit_);
    thread_ = QThread::create([this] { run(); });
    thread_->start();
    return true;
}

void ReplayInput::stop()
{
    stop_ = true;
    if (thread_) {
        thread_->wait();
        delete thread_;
        thread_ = nullptr;
        source_->setVisibleLimit(nullptr);     // everything is available again for ordinary processing
    }
}

void ReplayInput::run()
{
    const qint64 total = source_->totalFileSamples();
    QElapsedTimer t;
    t.start();
    while (!stop_.load()) {
        const qint64 n = std::min(total, qint64(double(t.nsecsElapsed()) * 1e-9 * fs_ * speed_));
        limit_.store(n, std::memory_order_release);
        if (n >= total) break;
        QThread::msleep(2);
    }
    finished_ = true;
}

} // namespace gnss
