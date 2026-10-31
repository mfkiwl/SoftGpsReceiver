#pragma once

#include "Constants.h"

#include <QString>
#include <QVector>

class QSettings;

namespace gnss {

// Packed2Real: 2-bit real samples, four per byte with the first sample in the low bits,
// two's complement adjusted (00 = +1, 01 = +3, 10 = -3, 11 = -1).
enum class SampleFormat { Int8Real = 0, UInt8Real, Int16Real, Int8IQ, Int16IQ, Packed2Real };
QString sampleFormatName(SampleFormat f);

// Sign relationship between the IF carrier offset and the true Doppler.
// "Inverted" occurs with high-side LO injection in the front end.
enum class DopplerSign { Auto = 0, Normal, Inverted };

// Which signals are searched, tracked and used in the position fix.
enum class Constellations { GpsGalileo = 0, GpsOnly, GalileoOnly, GlonassOnly };

// "PRN 5" for GPS, "E30" for Galileo (see the satellite ids in Constants.h).
QString satName(int id);

// Where live samples come from: a USRP through UHD, or the open recording played back at its real rate.
enum class LiveSourceKind { Usrp = 0, Replay };

struct LiveSettings
{
    LiveSourceKind source = LiveSourceKind::Usrp;

    // ---------------- USRP
    QString deviceArgs = QStringLiteral("addr=192.168.10.2");
    QString subdev;                     // empty = device default
    QString antenna;                    // empty = device default
    QString clockSource = QStringLiteral("internal");
    QString uhdLibrary;                 // path of uhd.dll; empty = search the usual places
    double centerFreq = 1575.42e6;      // Hz
    double sampleRate = 5e6;            // Hz; the N200 offers 100 MHz divided by an integer
    double gainDb = 30.0;
    double bandwidth = 0.0;             // Hz; 0 = leave the daughterboard filter alone
    double loOffset = 0.0;              // Hz, tunes the LO away from the band centre to keep its DC spike out

    // ---------------- Processing
    int ringSeconds = 10;               // sample buffer
    int windowSeconds = 60;             // tracking history kept per channel (at least one whole GPS frame)
    int historySeconds = 600;           // position fixes kept
    int updateMs = 1000;                // navigation solution and display update
    int acqIntervalSec = 10;            // search for satellites not being tracked
    double replaySpeed = 1.0;           // replay only

    // ---------------- Recording
    bool record = false;
    QString recordDir;
};

struct ReceiverSettings
{
    // ---------------- Input file
    QString fileName;
    SampleFormat format = SampleFormat::Int8Real;
    double samplingFreq = 38.192e6;     // Hz
    double ifFreq = 9.55e6;             // Hz
    qint64 skipBytes = 0;
    int msToProcess = 0;                // 0 = whole file
    Constellations constellations = Constellations::GpsGalileo;

    // ---------------- Acquisition
    QVector<int> acqPrns;               // GPS PRNs to search (empty = 1..32)
    QVector<int> acqGalSvids;           // Galileo SVIDs to search (empty = all)
    bool acqHasDopplerHint = false;     // the receiver clock offset is already known ...
    double acqDopplerHint = 0.0;        // ... as this Doppler common to all satellites (Hz)
    int galNonCoherent = 8;             // number of 4-ms E1 correlations summed non-coherently
    int acqStartMs = 0;                 // where in the file the acquisition block starts
    double acqSearchBand = 10000.0;     // +/- Hz around the IF
    double acqStep = 500.0;             // Doppler bin width (Hz)
    int acqNonCoherent = 4;             // number of 1-ms correlations summed non-coherently
    double acqThreshold = 2.5;          // peak / second-peak ratio
    int fineFreqMs = 40;                // data used for fine frequency estimation

    // ---------------- Tracking
    int numChannels = 12;               // per constellation
    double dllBandwidth = 2.0;          // Hz
    double dllDamping = 0.7;
    double dllSpacing = 0.5;            // early/late offset from prompt (chips)
    double galDllSpacing = 0.15;        // same for E1: the BOC(1,1) peak is much narrower
    double pllBandwidth = 20.0;         // Hz
    double pllDamping = 0.7;
    double fllBandwidth = 10.0;         // Hz (FLL assist during pull-in)
    int fllAssistMs = 500;              // FLL assist duration
    bool carrierAiding = false;         // carrier aiding of the DLL
    int cn0WindowMs = 100;              // M2M4 C/N0 estimator window

    // ---------------- Navigation
    int bitSyncStartMs = 1000;          // ignore pull-in transient before this
    int navSolPeriodMs = 500;
    double elevationMask = 5.0;         // degrees
    double minCn0 = 25.0;               // dB-Hz required to use a channel in a fix
    bool useIono = true;
    bool useTropo = true;
    bool useEarthRotation = true;
    bool elevationWeighting = false;
    DopplerSign dopplerSign = DopplerSign::Auto;

    LiveSettings live;

    ReceiverSettings();

    int samplesPerCode() const;                   // samples in one C/A code period (rounded)
    bool isComplex() const;
    int bytesPerSample() const;                   // per (complex) sample; 1 for packed samples
    QVector<int> prnList() const;
    double dopplerSignValue() const;              // +1 / -1 (Auto -> +1)
    bool useGps() const { return constellations == Constellations::GpsGalileo || constellations == Constellations::GpsOnly; }
    bool useGalileo() const { return constellations == Constellations::GpsGalileo || constellations == Constellations::GalileoOnly; }
    bool useGlonass() const { return constellations == Constellations::GlonassOnly; }
    // IF of a GLONASS frequency channel. In a GLONASS-only file the IF is that of channel 0; otherwise it is that of L1.
    double carrierIf(int id) const { return isGlonass(id) ? glonassIf(glonassChannel(id)) : ifFreq; }
    double glonassIf(int channel) const
    {
        return ifFreq + (useGlonass() ? 0.0 : kGlonassL1Base - kL1Freq) + channel * kGlonassL1Step;
    }

    // Extract the sampling and IF frequencies from a file name such as
    // "GPSdata-DiscreteComponents-fs38_192-if9_55.bin" (values in MHz).
    static bool parseFileName(const QString& path, double* fs, double* fif, SampleFormat* fmt = nullptr);

    QString summary() const;
    void save(QSettings& qs) const;
    void load(QSettings& qs);
};

} // namespace gnss
