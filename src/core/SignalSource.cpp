#include "SignalSource.h"
#include "LiveRing.h"

#include <cstring>
#include <cstdint>
#include <algorithm>

namespace gnss {

SignalSource::~SignalSource()
{
    close();
}

bool SignalSource::open(const QString& path, SampleFormat fmt, qint64 skipBytes, QString* error)
{
    close();
    file_.setFileName(path);
    if (!file_.open(QIODevice::ReadOnly)) {
        if (error) *error = QStringLiteral("Cannot open %1: %2").arg(path, file_.errorString());
        return false;
    }
    fileSize_ = file_.size();
    data_ = file_.map(0, fileSize_);
    if (!data_) {
        if (error) *error = QStringLiteral("Cannot memory-map %1 (%2)").arg(path, file_.errorString());
        file_.close();
        return false;
    }
    path_ = path;
    fmt_ = fmt;
    samplesPerByte_ = 1;
    switch (fmt) {
    case SampleFormat::Int8Real:
    case SampleFormat::UInt8Real: bytesPerSample_ = 1; break;
    case SampleFormat::Int16Real: bytesPerSample_ = 2; break;
    case SampleFormat::Int8IQ:    bytesPerSample_ = 2; break;
    case SampleFormat::Int16IQ:   bytesPerSample_ = 4; break;
    case SampleFormat::Packed2Real: bytesPerSample_ = 1; samplesPerByte_ = 4; break;
    }
    skip_ = std::clamp<qint64>(skipBytes, 0, fileSize_);
    numSamples_ = (fileSize_ - skip_) / bytesPerSample_ * samplesPerByte_;
    return true;
}

void SignalSource::attachRing(std::shared_ptr<LiveRing> ring)
{
    close();
    ring_ = std::move(ring);
    fmt_ = SampleFormat::Int16IQ;
    bytesPerSample_ = 4;
    samplesPerByte_ = 1;
    skip_ = 0;
}

qint64 SignalSource::numSamples() const
{
    if (ring_) return ring_->head();
    if (limit_) return std::min(limit_->load(std::memory_order_acquire), numSamples_);
    return numSamples_;
}

qint64 SignalSource::oldestSample() const
{
    return ring_ ? ring_->oldest() : 0;
}

void SignalSource::close()
{
    ring_.reset();
    limit_ = nullptr;
    if (data_) {
        file_.unmap(const_cast<uchar*>(data_));
        data_ = nullptr;
    }
    if (file_.isOpen()) file_.close();
    numSamples_ = 0;
    fileSize_ = 0;
    path_.clear();
}

int SignalSource::read(qint64 start, int count, float* outI, float* outQ) const
{
    if (ring_) return ring_->read(start, count, outI, outQ);
    if (!data_ || start < 0 || start >= numSamples_ || count <= 0) return 0;
    const int n = int(std::min<qint64>(count, numSamples_ - start));
    const uchar* p = data_ + skip_ + start / samplesPerByte_ * bytesPerSample_;
    switch (fmt_) {
    case SampleFormat::Int8Real: {
        const int8_t* s = reinterpret_cast<const int8_t*>(p);
        for (int i = 0; i < n; ++i) outI[i] = float(s[i]);
        if (outQ) std::fill(outQ, outQ + n, 0.0f);
        break;
    }
    case SampleFormat::UInt8Real:
        for (int i = 0; i < n; ++i) outI[i] = float(p[i]) - 127.5f;
        if (outQ) std::fill(outQ, outQ + n, 0.0f);
        break;
    case SampleFormat::Int16Real:
        for (int i = 0; i < n; ++i) {
            int16_t v; std::memcpy(&v, p + 2 * i, 2);
            outI[i] = float(v);
        }
        if (outQ) std::fill(outQ, outQ + n, 0.0f);
        break;
    case SampleFormat::Int8IQ: {
        const int8_t* s = reinterpret_cast<const int8_t*>(p);
        for (int i = 0; i < n; ++i) {
            outI[i] = float(s[2 * i]);
            if (outQ) outQ[i] = float(s[2 * i + 1]);
        }
        break;
    }
    case SampleFormat::Int16IQ:
        for (int i = 0; i < n; ++i) {
            int16_t a, b;
            std::memcpy(&a, p + 4 * i, 2);
            std::memcpy(&b, p + 4 * i + 2, 2);
            outI[i] = float(a);
            if (outQ) outQ[i] = float(b);
        }
        break;
    case SampleFormat::Packed2Real: {
        static const float level[4] = {1.0f, 3.0f, -3.0f, -1.0f};
        const int first = int(start & 3);
        for (int i = 0; i < n; ++i) {
            const int k = first + i;
            outI[i] = level[(p[k >> 2] >> (2 * (k & 3))) & 3];
        }
        if (outQ) std::fill(outQ, outQ + n, 0.0f);
        break;
    }
    }
    return n;
}

} // namespace gnss
