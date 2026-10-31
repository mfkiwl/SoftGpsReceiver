#pragma once

#include "Acquisition.h"
#include "SignalSource.h"
#include "Settings.h"

#include <QString>
#include <atomic>
#include <cstdint>
#include <vector>

namespace gnss {

// Everything recorded by one tracking channel, one entry per code period: about
// 1 ms for the GPS C/A code and 4 ms for Galileo E1. All arrays have numMs elements.
struct TrackingChannel
{
    int channel = 0;
    int prn = 0;
    bool valid = false;
    QString status;
    qint64 startSample = 0;
    double initCarrFreq = 0;
    int numMs = 0;
    int periodMs = 1;                     // nominal code period, so 1 entry = periodMs milliseconds
    int codeLength = 1023;                // chips

    std::vector<qint64> blockStart;       // first sample of the integration block
    std::vector<int> blockSize;           // samples integrated
    std::vector<double> codeEpochSample;  // (fractional) sample at which the prompt code epoch occurred
    std::vector<double> carrFreq;         // NCO carrier frequency used for the block (Hz)
    std::vector<double> codeFreq;         // NCO code frequency used for the block (chips/s)
    std::vector<double> remCarrPhase;     // carrier phase at block start (cycles)
    std::vector<double> remCodePhase;     // code phase at block start (chips)
    std::vector<float> IE, IP, IL, QE, QP, QL;   // on E1 these belong to the E1-C pilot
    std::vector<float> IB, QB;            // E1-B data prompt (Galileo only)
    std::vector<float> pllDisc;           // Costas discriminator atan(Q/I) (rad)
    std::vector<float> fllDisc;           // cross/dot FLL discriminator (Hz)
    std::vector<float> dllDisc;           // normalised early-minus-late envelope (chips)
    std::vector<float> carrNco;           // carrier loop filter output (Hz, relative to initial freq)
    std::vector<float> codeNco;           // code loop filter output (chips/s)
    std::vector<float> cn0;               // dB-Hz (moment method)
    std::vector<float> pli;               // phase lock indicator cos(2*dphi)
    std::vector<double> adr;              // accumulated Doppler (cycles, relative to IF)
    std::vector<uint8_t> lock;            // 1 when phase locked

    // loop parameters used (for display)
    double pllWn = 0, pllK1 = 0, pllK2 = 0, fllW0 = 0;
    double dllWn = 0, dllK1 = 0, dllK2 = 0;
    double meanCn0 = 0, lockFraction = 0, finalDoppler = 0;
    double elapsedSec = 0;
    int sidePeakJumps = 0;                // Galileo: times the code loop left a BOC side peak

    void reserve(int n);
    void trimFront(int n);                // drops the oldest n entries (live mode rolling window)
    void summarise(const ReceiverSettings& s);   // numMs, valid and the statistics below
};

struct CorrShapePoint
{
    double offset;     // chips, positive = early (advanced replica)
    double magnitude;  // |I + jQ| averaged non-coherently
    double inPhase;    // I, sign-normalised to the prompt
    double quadrature;
};

// The carrier and code loops of one channel. step() processes one code period, so the same loops serve a whole
// recording (Tracking::trackChannel) and a live stream that only has samples up to the newest one received.
class ChannelTracker
{
public:
    ChannelTracker(const ReceiverSettings& s, const AcqResult& acq, qint64 startSample, bool complexInput,
                   TrackingChannel& ch);

    // Appends one code period to the channel. Returns false, changing nothing, when the
    // samples it needs lie beyond availableSamples.
    bool step(const SignalSource& src, qint64 availableSamples);

    qint64 nextSample() const { return pos_; }

private:
    TrackingChannel& ch_;
    double fs_, ifFreq_;
    bool cpx_;
    int fllAssistMs_;
    bool gal_ = false;
    int periodMs_ = 1, codeLength_ = 1023;
    double tableScale_ = 1.0;
    std::vector<float> codeP_, dataP_;
    double zp_ = 0, wnp_ = 0, w0f_ = 0, zd_ = 0, wnd_ = 0;
    double d_ = 0, dopSign_ = 1, dllGain_ = 1;
    double carrBase_ = 0, carrFreq_ = 0, codeFreq_ = 0, aidRef_ = 0;
    bool aiding_ = false;
    double remCode_ = 0, remCarr_ = 0;
    double carrInteg_ = 0, codeInteg_ = 0;
    double adr_ = 0, sI2_ = 0, sQ2_ = 0;
    float prevI_ = 0, prevQ_ = 0, cn0_ = 0;
    int cn0Win_ = 20;
    int bumpEarly_ = 0, bumpLate_ = 0;
    std::vector<double> pwr_;
    qint64 pos_ = 0;
    long long epoch_ = 0;
    std::vector<float> bufI_, bufQ_;
};

class Tracking
{
public:
    static void trackChannel(const SignalSource& src, const ReceiverSettings& s, const AcqResult& acq,
                             qint64 startSample, TrackingChannel& ch,
                             const std::atomic<bool>* cancel, std::atomic<qint64>* msCounter);

    // Re-computes a bank of correlators around the prompt for a given ms using
    // the NCO state stored during tracking (the "correlation triangle").
    static std::vector<CorrShapePoint> correlationShape(const SignalSource& src, const ReceiverSettings& s,
                                                        const TrackingChannel& ch, int ms,
                                                        double span, int taps, int avgMs);
};

} // namespace gnss
