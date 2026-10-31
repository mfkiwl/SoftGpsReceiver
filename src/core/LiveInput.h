#pragma once

#include "LiveRing.h"
#include "Settings.h"
#include "SignalSource.h"

#include <QString>
#include <atomic>
#include <memory>

class QThread;

namespace gnss {

// Actual radio settings, which can differ from those requested
struct LiveInputInfo
{
    double sampleRate = 0;       // Hz
    double centerFreq = 0;       // Hz, tuned frequency
    double gainDb = 0;
    QString description;         // one line for the status display
    QString details;             // the device's own description, for the log
};

// Produces samples in real time, on a thread of its own.
class LiveInput
{
public:
    virtual ~LiveInput() = default;
    virtual bool start(QString* error) = 0;
    virtual void stop() = 0;

    bool finished() const { return finished_.load(); }      // no more samples will come
    QString failure() const { return failed_.load() ? failure_ : QString(); }
    qint64 overflows() const { return overflows_.load(); }  // times the radio had to discard samples
    qint64 lostSamples() const { return lost_.load(); }
    const LiveInputInfo& info() const { return info_; }

protected:
    void fail(const QString& why)
    {
        failure_ = why;
        failed_ = true;
        finished_ = true;
    }

    std::atomic<bool> finished_{false};
    std::atomic<bool> failed_{false};
    QString failure_;
    std::atomic<qint64> overflows_{0};
    std::atomic<qint64> lost_{0};
    LiveInputInfo info_;
};

// Plays the open recording at its real sample rate (or a multiple of it), so that everything downstream sees what
// it would see from a radio: lets the live mode be tried without any hardware.
class ReplayInput : public LiveInput
{
public:
    ReplayInput(SignalSource* source, const ReceiverSettings& s);
    ~ReplayInput() override;
    bool start(QString* error) override;
    void stop() override;

private:
    void run();

    SignalSource* source_;
    double fs_;
    double speed_;
    std::atomic<qint64> limit_{0};
    std::atomic<bool> stop_{false};
    QThread* thread_ = nullptr;
};

// Streams from a USRP through the UHD driver. UHD is loaded when streaming starts, through its C API: its C++ API
// cannot be used across compilers, the Windows installer is built with MSVC, and the program must also run on
// computers without UHD.
class UhdInput : public LiveInput
{
public:
    explicit UhdInput(const LiveSettings& s);
    ~UhdInput() override;

    bool start(QString* error) override;
    void stop() override;

    // Valid after open(): the ring has to be sized from the rate the device really gives.
    bool open(QString* error);
    void setRing(std::shared_ptr<LiveRing> ring) { ring_ = std::move(ring); }

private:
    struct Api;
    void run();
    QString lastError(int code) const;
    void closeDevice();

    LiveSettings cfg_;
    std::unique_ptr<Api> api_;
    void* usrp_ = nullptr;
    void* streamer_ = nullptr;
    void* metadata_ = nullptr;
    size_t maxSamps_ = 0;
    std::shared_ptr<LiveRing> ring_;
    std::atomic<bool> stop_{false};
    QThread* thread_ = nullptr;
};

} // namespace gnss
