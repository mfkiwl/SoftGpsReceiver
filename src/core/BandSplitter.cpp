#include "BandSplitter.h"
#include "Constants.h"

#include <algorithm>
#include <cmath>

namespace gnss {

BandSplitter::BandSplitter(double inputRate, int decimation, double offsetHz, double cutoffHz, int taps)
    : decimation_(decimation), step_(-offsetHz / inputRate), taps_(size_t(taps))
{
    const double fc = cutoffHz / inputRate, mid = (taps - 1) / 2.0;
    double sum = 0;
    for (int i = 0; i < taps; ++i) {                         // windowed sinc
        const double t = i - mid;
        const double sinc = t == 0 ? 2 * fc : std::sin(kTwoPi * fc * t) / (kPi * t);
        const double hamming = 0.54 - 0.46 * std::cos(kTwoPi * i / (taps - 1));
        taps_[size_t(i)] = float(sinc * hamming);
        sum += sinc * hamming;
    }
    for (float& t : taps_) t = float(t / sum);
    histI_.assign(size_t(taps), 0.0f);
    histQ_.assign(size_t(taps), 0.0f);
}

int BandSplitter::process(const float* iq, int n, float scale, int16_t* out)
{
    const int L = int(taps_.size());
    std::vector<float> I(size_t(L + n)), Q(size_t(L + n));
    std::copy(histI_.begin(), histI_.end(), I.begin());
    std::copy(histQ_.begin(), histQ_.end(), Q.begin());

    // A rotator: far cheaper than sin and cos per sample, with the phase recomputed once per block
    const double c0 = std::cos(kTwoPi * phase_), s0 = std::sin(kTwoPi * phase_);
    const double cd = std::cos(kTwoPi * step_), sd = std::sin(kTwoPi * step_);
    double c = c0, s = s0;
    for (int k = 0; k < n; ++k) {
        const float x = iq[2 * k], y = iq[2 * k + 1];
        I[size_t(L + k)] = float(x * c - y * s);
        Q[size_t(L + k)] = float(x * s + y * c);
        const double nc = c * cd - s * sd;
        s = c * sd + s * cd;
        c = nc;
    }
    phase_ = phase_ + step_ * n;
    phase_ -= std::floor(phase_);

    int written = 0;
    const float* h = taps_.data();
    int k = phaseToNext_;
    for (; k < n; k += decimation_) {
        const float* a = I.data() + k + 1;
        const float* b = Q.data() + k + 1;
        float sumI = 0, sumQ = 0;
        for (int t = 0; t < L; ++t) {
            sumI += h[t] * a[t];
            sumQ += h[t] * b[t];
        }
        out[2 * written] = int16_t(std::clamp(sumI * scale, -32767.0f, 32767.0f));
        out[2 * written + 1] = int16_t(std::clamp(sumQ * scale, -32767.0f, 32767.0f));
        ++written;
    }
    phaseToNext_ = k - n;
    std::copy(I.end() - L, I.end(), histI_.begin());
    std::copy(Q.end() - L, Q.end(), histQ_.begin());
    return written;
}

} // namespace gnss
