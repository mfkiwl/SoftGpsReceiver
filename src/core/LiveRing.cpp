#include "LiveRing.h"

#include <algorithm>
#include <cstring>

namespace gnss {

LiveRing::LiveRing(qint64 capacitySamples)
    : capacity_(std::max<qint64>(capacitySamples, 1024)), margin_(capacity_ / 16), buf_(size_t(capacity_) * 2, 0)
{
}

void LiveRing::write(const int16_t* iq, qint64 count)
{
    const qint64 h = head_.load(std::memory_order_relaxed);
    qint64 done = 0;
    if (count > capacity_) {
        done = count - capacity_;      // only the newest samples fit
    }
    while (done < count) {
        const qint64 at = (h + done) % capacity_;
        const qint64 n = std::min(count - done, capacity_ - at);
        std::memcpy(buf_.data() + 2 * at, iq + 2 * done, size_t(n) * 4);
        done += n;
    }
    head_.store(h + count, std::memory_order_release);
}

void LiveRing::writeZeros(qint64 count)
{
    if (count <= 0) return;
    const qint64 h = head_.load(std::memory_order_relaxed);
    qint64 todo = std::min(count, capacity_);
    qint64 pos = h + count - todo;
    while (todo > 0) {
        const qint64 at = pos % capacity_;
        const qint64 n = std::min(todo, capacity_ - at);
        std::memset(buf_.data() + 2 * at, 0, size_t(n) * 4);
        pos += n;
        todo -= n;
    }
    head_.store(h + count, std::memory_order_release);
}

qint64 LiveRing::oldest() const
{
    return std::max<qint64>(0, head() - capacity_ + margin_);
}

int LiveRing::available(qint64& start, int count) const
{
    const qint64 h = head();
    const qint64 first = std::max<qint64>(0, h - capacity_ + margin_);
    if (start < first || start >= h || count <= 0) return 0;
    return int(std::min<qint64>(count, h - start));
}

int LiveRing::read(qint64 start, int count, float* outI, float* outQ) const
{
    const int n = available(start, count);
    for (int i = 0; i < n; ++i) {
        const qint64 at = (start + i) % capacity_;
        outI[i] = float(buf_[size_t(2 * at)]);
        if (outQ) outQ[i] = float(buf_[size_t(2 * at + 1)]);
    }
    return n;
}

int LiveRing::readRaw(qint64 start, int count, int16_t* outIq) const
{
    const int n = available(start, count);
    int done = 0;
    while (done < n) {
        const qint64 at = (start + done) % capacity_;
        const int part = int(std::min<qint64>(n - done, capacity_ - at));
        std::memcpy(outIq + 2 * done, buf_.data() + 2 * at, size_t(part) * 4);
        done += part;
    }
    return n;
}

} // namespace gnss
