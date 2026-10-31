#pragma once

#include "SignalSource.h"
#include "Settings.h"

#include <QString>
#include <atomic>
#include <functional>
#include <vector>

namespace gnss {

struct AcqResult
{
    int prn = 0;
    bool acquired = false;
    double metric = 0;          // peak / second peak
    double peak = 0;
    double secondPeak = 0;
    double noiseMean = 0;
    double cn0 = 0;             // dB-Hz estimate from the correlation peak
    int codePhase = 0;          // sample (within the code period) at which the code starts
    qint64 startSample = 0;     // live mode: first sample of the data searched, to which codePhase refers
    double codePhaseChips = 0;
    double coarseFreq = 0;      // Hz (IF + Doppler bin)
    double fineFreq = 0;        // Hz after fine estimation
    double fineDelta = 0;       // fine - coarse (Hz)
    double doppler = 0;         // fineFreq - nominal IF
    double commonDoppler = 0;   // Galileo only: Doppler shared by all satellites, i.e. the IF and clock error
    int peakBin = 0;

    // Diagnostics for display
    int surfaceCols = 0;                 // decimated code-phase columns (max-pooled)
    std::vector<float> surface;          // [bin * surfaceCols + col]
    std::vector<float> codeSlice;        // full resolution at the peak Doppler bin
    std::vector<float> dopplerSlice;     // per Doppler bin at the peak code phase
    std::vector<double> binFreqs;        // Galileo only: its own Doppler bins (Hz); GPS shares the list in the output
    std::vector<double> fineAxis;        // Hz offset from the coarse frequency
    std::vector<float> fineSpectrum;     // spectrum of the squared (data-wiped) signal
};

struct AcquisitionOutput
{
    std::vector<AcqResult> results;      // one per searched PRN
    std::vector<double> binFreqs;        // Hz
    int samplesPerCode = 0;
    qint64 startSample = 0;
    int nonCoherent = 0;
    int fftSize = 0;
    int fftInternalSize = 0;
    double elapsedSec = 0;
    QString error;
};

class Acquisition
{
public:
    // Parallel code-phase search: for every Doppler bin the carrier-wiped signal
    // is circularly correlated with each PRN replica using FFTs. 'progress'
    // receives 0..100 and may be called from worker threads.
    static AcquisitionOutput run(const SignalSource& src, const ReceiverSettings& s,
                                 const std::atomic<bool>* cancel,
                                 const std::function<void(int)>& progress);
};

} // namespace gnss
