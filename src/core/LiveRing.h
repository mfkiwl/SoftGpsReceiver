#pragma once

#include <QtGlobal>

#include <atomic>
#include <cstdint>
#include <vector>

namespace gnss {

// Ring buffer of complex 16-bit samples between the thread receiving from the radio and everything that reads them.
// Samples keep their absolute index since the start of the stream, so the code that works on recorded files can
// address them the same way. One writer; any number of readers. A reader that falls behind by more than the capacity
// gets fewer samples than it asked for rather than old data.
class LiveRing
{
public:
    explicit LiveRing(qint64 capacitySamples);

    void write(const int16_t* iq, qint64 count);
    // For samples lost on the way: keeps the indices of later samples aligned with time.
    void writeZeros(qint64 count);

    qint64 head() const { return head_.load(std::memory_order_acquire); }     // samples written so far
    qint64 oldest() const;                                                    // first index still readable
    qint64 capacity() const { return capacity_; }

    int read(qint64 start, int count, float* outI, float* outQ) const;
    int readRaw(qint64 start, int count, int16_t* outIq) const;

private:
    int available(qint64& start, int count) const;

    qint64 capacity_;
    qint64 margin_;      // kept out of reach of readers, so that they never touch what is being overwritten
    std::vector<int16_t> buf_;
    std::atomic<qint64> head_{0};
};

} // namespace gnss
