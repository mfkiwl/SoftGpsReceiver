#pragma once

#include "Constants.h"
#include "Settings.h"
#include "Tracking.h"

#include <QString>
#include <QVector>
#include <algorithm>
#include <array>
#include <cstdint>
#include <map>
#include <vector>

namespace gnss {

// ---------------------------------------------------------------------------
// Field layout definitions (IS-GPS-200 Figures 20-1 and 20-2). Bit positions
// are 1-based within the 300-bit subframe, as in the ICD.
// ---------------------------------------------------------------------------
enum class FieldKind { Number, Semicircle, Code, Reserved, Parity, Text };

struct NavFieldDef
{
    QString key;
    QString name;
    int s1 = 0, l1 = 0;          // first segment
    int s2 = 0, l2 = 0;          // optional second segment (LSBs)
    bool isSigned = false;
    double scale = 1.0;
    QString scaleText;
    QString units;
    FieldKind kind = FieldKind::Number;
    int totalBits() const { return l1 + l2; }
};

struct DecodedField
{
    NavFieldDef def;
    quint64 raw = 0;             // unsigned raw bits
    qint64 rawSigned = 0;        // two's complement interpretation
    double value = 0;            // scaled value in ICD units
    QString bitString;
    QString valueText;
    QString interpretation;
};

// ---------------------------------------------------------------------------
struct NavWord
{
    uint32_t raw30 = 0;          // received bits D1..D30 (after preamble polarity correction)
    uint32_t data24 = 0;         // source data bits d1..d24 (after D30* inversion)
    uint8_t parityRx = 0;        // received D25..D30
    uint8_t parityCalc = 0;      // computed from d1..d24, D29*, D30*
    bool parityOk = false;
    uint8_t d29s = 0, d30s = 0;  // last two bits of the previous word
};

struct Subframe
{
    int startBit = 0;            // index into the channel's bit stream
    int startMs = 0;             // tracking ms index of the first bit of the preamble
    bool inverted = false;       // PLL 180-degree ambiguity detected via the preamble
    std::array<uint8_t, 300> rxBits{};     // D (received, polarity corrected)
    std::array<uint8_t, 300> dataBits{};   // d (source bits; parity bits copied as received)
    std::array<float, 300> softBits{};
    NavWord words[10];
    int parityOkCount = 0;
    bool allParityOk() const { return parityOkCount == 10; }

    int id = 0;                  // subframe ID 1..5
    int towCount = 0;            // HOW time-of-week count (6 s units, next subframe)
    double towStart = 0;         // time of week at the start of this subframe (s)
    bool alert = false, antiSpoof = false, isf = false;
    int tlmMessage = 0;
    int dataId = -1, svId = -1;  // subframes 4/5
    int page = 0;                // 1..25 for subframes 4/5
    bool towConsistent = true;

    quint64 bits(int start1, int len) const;   // from dataBits, 1-based
};

// One Galileo I/NAV word: the payload of an even and odd page part.
struct InavWord
{
    int startMs = 0;             // tracking index of the first sync symbol of the even part
    bool inverted = false;       // carrier 180-degree ambiguity, resolved by the sync pattern
    bool crcOk = false;
    int type = -1;
    bool hasTow = false;
    double tow = 0;              // GST time of week at startMs (s), when the word carries it
    std::array<uint8_t, 128> bits{};

    quint64 field(int start, int len) const;   // 0-based, as counted from the word type
};

// For Galileo the fields hold the I/NAV equivalents: sf1..sf3 are all set once words 1 to 5 are in, iode2, iode3 and
// iodc hold IODnav from words 1, 3 and 4, uraIndex the SISA index, tgd BGD(E1,E5b), svHealth 2*E1-BHS + E1-BDVS.
struct Ephemeris
{
    int prn = 0;
    bool sf1 = false, sf2 = false, sf3 = false;
    int sfIndex[4] = {-1, -1, -1, -1};
    // subframe 1
    int weekNumber = 0, l2Code = 0, uraIndex = 0, svHealth = 0, iodc = 0, l2pFlag = 0;
    double tgd = 0, toc = 0, af2 = 0, af1 = 0, af0 = 0;
    // subframe 2 (angles in radians)
    int iode2 = 0, fitFlag = 0, aodo = 0;
    double crs = 0, deltaN = 0, m0 = 0, cuc = 0, e = 0, cus = 0, sqrtA = 0, toe = 0;
    // subframe 3
    int iode3 = 0;
    double cic = 0, omega0 = 0, cis = 0, i0 = 0, crc = 0, omega = 0, omegaDot = 0, idot = 0;

    bool complete() const { return sf1 && sf2 && sf3; }
    bool consistent() const
    {
        return complete() && iode2 == iode3 && (isGalileo(prn) ? iodc == iode2 : (iodc & 0xFF) == iode2);
    }
};

struct AlmanacEntry
{
    bool valid = false;
    int svId = 0;
    int health = 0;              // 8-bit
    double e = 0, toa = 0, deltaI = 0, omegaDot = 0, sqrtA = 0, omega0 = 0, omega = 0, m0 = 0, af0 = 0, af1 = 0;
    int sourcePrn = 0;           // satellite that transmitted it
    int page = 0, subframe = 0;
};

struct IonoUtc
{
    bool valid = false;
    int sourcePrn = 0;
    double alpha[4] = {0, 0, 0, 0};
    double beta[4] = {0, 0, 0, 0};
    double A0 = 0, A1 = 0;
    int tot = 0, wnt = 0, dtls = 0, wnlsf = 0, dn = 0, dtlsf = 0;
};

struct PageRecord
{
    int prn = 0;
    int subframe = 0;
    int svId = 0;
    int page = 0;
    double tow = 0;
    bool parityOk = false;
};

struct NavChannel
{
    int channel = 0;
    int prn = 0;

    // bit synchronisation
    std::array<int, 20> transitionHist{};
    int bitSyncStartMs = 0;
    int bitOffset = 0;               // ms (mod 20) at which bits start
    double bitSyncConfidence = 0;
    bool bitSyncOk = false;
    int firstBitMs = 0;
    std::vector<float> softBits;     // 20-ms integrated prompt
    std::vector<uint8_t> hardBits;   // as received (unknown polarity)
    double bitSnrDb = 0;
    double estimatedBer = 0;

    // preamble detection
    std::vector<int> preambleCorr;   // +/-8 correlation of the bit stream with 10001011
    std::vector<int> preambleMatches;
    std::vector<int> preambleVerified;

    std::vector<Subframe> subframes;
    std::vector<InavWord> inavWords;     // Galileo only; the bit sync fields then describe page sync
    int inavTowErrors = 0;               // words whose TOW disagrees with the page count
    // Decoded data that passes its check proves the channel was really tracking at that moment, which the
    // lock indicator cannot do at low C/N0. These are the tracking indices of the words of fully parity-checked GPS
    // subframes, or of I/NAV words passing the CRC.
    std::vector<int> verifiedMs;
    int verifySpan = 10000;              // how far from one of them a measurement may lie, in tracking indices
    bool verifiedAt(double ms) const
    {
        const auto it = std::lower_bound(verifiedMs.begin(), verifiedMs.end(), int(ms) - verifySpan);
        return it != verifiedMs.end() && *it <= int(ms) + verifySpan;
    }
    Ephemeris eph;

    // timing: transmit time of the code epoch at tracking index refMs
    bool timingValid = false;
    int refMs = 0;
    double refTow = 0;
    double epochSec = 1e-3;              // duration of one tracking index
    double txTimeAtMs(double ms) const { return refTow + (ms - refMs) * epochSec; }

    int parityOkWords = 0, parityTotalWords = 0;   // for Galileo: I/NAV words passing the CRC
    QString status;
};

// Galileo system parameters from I/NAV words 5, 6 and 10
struct GalileoGlobal
{
    int sourcePrn = 0;
    bool ionoValid = false;
    double ai0 = 0, ai1 = 0, ai2 = 0;    // effective ionisation level coefficients (for NeQuick)
    bool utcValid = false;
    double A0 = 0, A1 = 0;
    int dtLs = 0, t0t = 0, wn0t = 0, wnLsf = 0, dn = 0, dtLsf = 0;
    bool ggtoValid = false;              // GPS to Galileo time offset
    double a0g = 0, a1g = 0;
    int t0g = 0, wn0g = 0;
};

struct GlobalNavData
{
    AlmanacEntry almanac[33];
    bool sf5p25 = false;
    int p25Toa = 0, p25Wna = 0;
    int health6[33];
    bool health6Valid[33];
    bool sf4p25 = false;
    int svConfig[33];
    bool svConfigValid[33];
    IonoUtc ionoUtc;
    QString specialMessage;
    int specialMessagePrn = 0;
    std::map<int, std::vector<int>> nmct;   // prn -> [AI, ERD1..ERD30]
    std::vector<PageRecord> pages;
    int weekNumber = -1;                     // 10-bit week from subframe 1
    GalileoGlobal galileo;

    GlobalNavData();
};

namespace nav {

std::vector<NavFieldDef> layoutFor(int subframeId, int svId);
std::vector<DecodedField> decodeFields(const Subframe& sf);
QString pageDescription(int subframeId, int svId);

// textual interpretations
QString uraText(int index);
double uraNominalMeters(int index);
QString signalHealthText(int code5);
QString navDataHealthText(int code3);
QString health6Text(int h);
QString health8Text(int h);
QString l2CodeText(int c);
QString svConfigText(int c4);
QString towText(double tow);

// parity per IS-GPS-200 Table 20-XIV; D points to 30 received bits
uint8_t computeParity(const uint8_t* D, int d29s, int d30s, uint8_t* dataOut24);

NavChannel decodeChannel(const TrackingChannel& trk, const ReceiverSettings& s);
void collectGlobal(const std::vector<NavChannel>& channels, GlobalNavData& g);
// Adds what 'fresh' holds to 'total', which keeps everything older. Live mode decodes a rolling window, and pages
// such as the ionospheric parameters come round only every 12.5 minutes.
void mergeGlobal(const GlobalNavData& fresh, GlobalNavData& total);

} // namespace nav
} // namespace gnss
