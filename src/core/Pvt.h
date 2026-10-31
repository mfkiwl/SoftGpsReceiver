#pragma once

#include "Ephemeris.h"
#include "Geodesy.h"
#include "MathUtil.h"
#include "NavMessage.h"
#include "Settings.h"
#include "Tracking.h"

#include <QString>
#include <QStringList>
#include <array>
#include <vector>

namespace gnss {

struct SatObservation
{
    int channel = -1;
    int prn = 0;
    bool available = false;      // measurement valid (timing, C/N0, health)
    bool used = false;           // used in the final position fix
    QString status;

    double txTime = 0;           // satellite time of transmission (s of week)
    double txTimeGps = 0;        // corrected for the satellite clock
    double rawPr = 0;            // c * (t_rx - t_tx)  (m)
    double satClock = 0;         // total satellite clock correction (s), incl. relativistic and -TGD
    double satClockRel = 0;      // relativistic part (s)
    double tgd = 0;
    double satClkDrift = 0;      // s/s
    double prClockCorrected = 0; // rawPr + c * satClock
    double iono = 0, tropo = 0;  // m
    double sagnac = 0;           // range change from earth rotation during signal flight (m)
    double correctedPr = 0;      // pseudorange after all corrections (m)
    double geomRange = 0;        // |sat - rx| (m)
    double residual = 0;         // post-fit residual (m)
    double weight = 1;

    Vec3 satPosTx, satVelTx;     // ECEF at transmission
    Vec3 satPos, satVel;         // rotated into the ECEF frame at reception
    double el = 0, az = 0;       // rad

    double carrFreq = 0;         // tracking NCO frequency (Hz)
    double doppler = 0;          // carrFreq - nominal IF (Hz)
    double prRate = 0;           // measured pseudorange rate (m/s)
    double predRangeRate = 0;    // geometric range rate from the solution (m/s)
    double predDoppler = 0;      // -predRangeRate / lambda (Hz)
    double rateResidual = 0;     // velocity solution residual (m/s)
    double cn0 = 0, pli = 0;
    double wavelength = kL1Wavelength;   // GLONASS: of its frequency channel
    double adrCycles = 0;
    int health = 0;
};

struct LsqIteration
{
    int iter = 0;
    Vec3 pos;
    double bias = 0;
    Vec3 dpos;
    double dbias = 0;
    double isb = 0;
    double stepNorm = 0;
    double rms = 0;
    int nSats = 0;
    std::vector<int> prns;
    std::vector<double> residuals;          // pre-fit residuals for this iteration
    std::vector<std::array<double, 5>> H;   // design matrix rows (column 5 is used with two constellations)
};

struct NavSolution
{
    int index = 0;
    double rxSample = 0;         // receiver sample index of the measurement epoch
    double rxTime = 0;           // receiver clock time (s of week)
    double gpsTime = 0;          // rxTime - bias/c
    bool valid = false;
    int nUsed = 0;
    Vec3 pos;
    double bias = 0;             // receiver clock bias (m)
    double isb = 0;              // Galileo minus GPS: system time offset plus receiver delays (m)
    Geodetic llh;
    Vec3 enu;                    // relative to the mean position
    double gdop = 0, pdop = 0, hdop = 0, vdop = 0, tdop = 0;
    double qEnu[4][4] = {};      // cofactor matrix in local (E,N,U,clock) frame
    double rms = 0;
    bool velValid = false;
    Vec3 vel, velEnu;
    double drift = 0;            // receiver clock drift (m/s)
    double velRms = 0;
    std::vector<SatObservation> obs;
    std::vector<LsqIteration> iterations;
    QString message;
};

struct PvtOutput
{
    std::vector<NavSolution> sols;
    std::vector<int> usableChannels;
    int nValid = 0;
    Vec3 meanPos;
    Geodetic meanLlh;
    Utm utm;
    double stdE = 0, stdN = 0, stdU = 0;
    double cep50 = 0, drms2 = 0, sep50 = 0;
    double meanPdop = 0, meanHdop = 0, meanSats = 0;
    double meanIsb = 0;          // mean Galileo minus GPS bias (m), with both constellations
    int nGps = 0, nGalileo = 0;  // usable satellites
    double biasDrift = 0;        // slope of the clock bias (m/s)
    double meanDrift = 0;        // mean Doppler-derived clock drift (m/s)
    double commonFreqOffset = 0; // Hz, as seen at IF
    double effectiveIf = 0;      // Hz
    int dopplerSign = 1;
    double signRms[2] = {0, 0};  // velocity residual RMS for sign +1 / -1
    bool signAuto = true;
    int weekNumber = -1;
    double firstGpsTime = 0;
    QString error;
    QStringList log;
};

namespace pvt {

// Mean phase lock indicator below which a live channel no longer counts as following a signal. The indicator
// is S/(S+2N) per integration: 0.24 at 28 dB-Hz over 1 ms, and within about +/-0.03 of zero for noise over 0.5 s.
constexpr double kLivePhaseLock = 0.12;

PvtOutput solve(const std::vector<TrackingChannel>& trk, const std::vector<NavChannel>& nav,
                const GlobalNavData& gnav, const ReceiverSettings& s);

// Solves a stream epoch by epoch: every call adds the epochs that became available since the last one to
// out.sols and refreshes the statistics, keeping the receiver clock continuous. Unlike solve() it needs no
// interval common to all channels, so a satellite joining or leaving does not disturb the fixes before it.
class LiveSolver
{
public:
    void update(const std::vector<TrackingChannel>& trk, const std::vector<NavChannel>& nav,
                const GlobalNavData& gnav, const ReceiverSettings& s, int maxFixes, PvtOutput& out);

private:
    bool haveEpoch_ = false;
    double nextR_ = 0;           // next measurement epoch (receiver sample)
    bool haveRef_ = false;
    double trxRef_ = 0, Rref_ = 0;
    bool havePrev_ = false;
    Vec3 xPrev_;
    double bPrev_ = 0, isbPrev_ = 0;
    bool signKnown_ = false;
    double sign_ = 1.0;
    int nextIndex_ = 0;
};

} // namespace pvt
} // namespace gnss
