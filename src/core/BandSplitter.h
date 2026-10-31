#pragma once

#include <cstdint>
#include <vector>

namespace gnss {

// Takes one band out of a wideband complex stream: shifts it to zero frequency, low-pass filters and decimates.
class BandSplitter
{
public:
    // offsetHz is the band's centre relative to the stream's centre. cutoffHz is the one-sided filter edge.
    BandSplitter(double inputRate, int decimation, double offsetHz, double cutoffHz, int taps = 101);

    // iq holds n interleaved complex floats. Writes interleaved int16, multiplied by scale; returns the complex samples written.
    int process(const float* iq, int n, float scale, int16_t* out);

private:
    int decimation_;
    int phaseToNext_ = 0;
    double phase_ = 0, step_ = 0;       // cycles
    std::vector<float> taps_, histI_, histQ_;
};

} // namespace gnss
