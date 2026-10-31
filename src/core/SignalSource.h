#pragma once

#include "Settings.h"

#include <QFile>
#include <QString>

#include <atomic>
#include <memory>

namespace gnss {

class LiveRing;

// Access to IF samples: a memory-mapped recording, or the stream from a radio. All read
// functions are const and thread-safe so that every tracking channel can read in parallel.
class SignalSource
{
public:
    SignalSource() = default;
    ~SignalSource();
    SignalSource(const SignalSource&) = delete;
    SignalSource& operator=(const SignalSource&) = delete;

    bool open(const QString& path, SampleFormat fmt, qint64 skipBytes, QString* error);
    void close();

    // Serves samples from a live stream instead of a file.
    void attachRing(std::shared_ptr<LiveRing> ring);
    // Makes only the first *limit samples of the file visible, as if the recording were still arriving.
    void setVisibleLimit(const std::atomic<qint64>* limit) { limit_ = limit; }

    bool isOpen() const { return data_ != nullptr || ring_ != nullptr; }
    bool isLive() const { return ring_ != nullptr; }
    QString path() const { return path_; }
    qint64 fileSize() const { return fileSize_; }
    qint64 numSamples() const;
    qint64 oldestSample() const;     // first sample that can still be read (0 for a file)
    qint64 totalFileSamples() const { return numSamples_; }
    SampleFormat format() const { return fmt_; }
    bool isComplex() const { return fmt_ == SampleFormat::Int8IQ || fmt_ == SampleFormat::Int16IQ; }

    // Reads up to 'count' samples starting at sample index 'start'. The in-phase
    // (or real) component goes to outI; the quadrature component to outQ when
    // outQ is non-null (zero for real formats). Returns the number of samples read.
    int read(qint64 start, int count, float* outI, float* outQ = nullptr) const;

private:
    QFile file_;
    QString path_;
    const uchar* data_ = nullptr;
    qint64 fileSize_ = 0;
    qint64 skip_ = 0;
    qint64 numSamples_ = 0;
    int bytesPerSample_ = 1;
    int samplesPerByte_ = 1;
    SampleFormat fmt_ = SampleFormat::Int8Real;
    std::shared_ptr<LiveRing> ring_;
    const std::atomic<qint64>* limit_ = nullptr;
};

} // namespace gnss
