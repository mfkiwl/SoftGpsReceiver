#include "GlonassNav.h"
#include "Constants.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <vector>

namespace gnss {
namespace glo {

namespace {

constexpr int kMarkSymbols = 30;
constexpr int kStringMs = 2000, kSymbolMs = 10, kMarkMs = kMarkSymbols * kSymbolMs;
constexpr char kMark[] = "111110001101110101000010010110";

// Bytes hold b85 first, so bit 84 of the buffer is b1. One row per check bit.
constexpr uint8_t kMasks[8][11] = {
    {0x55, 0x55, 0x5A, 0xAA, 0xAA, 0xAA, 0xB5, 0x55, 0x6A, 0xD8, 0x08},
    {0x66, 0x66, 0x6C, 0xCC, 0xCC, 0xCC, 0xD9, 0x99, 0xB3, 0x68, 0x10},
    {0x87, 0x87, 0x8F, 0x0F, 0x0F, 0x0F, 0x1E, 0x1E, 0x3C, 0x70, 0x20},
    {0x07, 0xF8, 0x0F, 0xF0, 0x0F, 0xF0, 0x1F, 0xE0, 0x3F, 0x80, 0x40},
    {0xF8, 0x00, 0x0F, 0xFF, 0xF0, 0x00, 0x1F, 0xFF, 0xC0, 0x00, 0x80},
    {0x00, 0x00, 0x0F, 0xFF, 0xFF, 0xFF, 0xE0, 0x00, 0x00, 0x01, 0x00},
    {0xFF, 0xFF, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00},
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC, 0x00}};

int maskParity(const StringBits& b, int row)
{
    int p = 0;
    for (int i = 0; i < kStringBits; ++i)
        if (kMasks[row][i / 8] & (0x80 >> (i % 8))) p ^= b[size_t(kStringBits - i)];
    return p;
}

quint64 field(const StringBits& b, int hi, int len)
{
    quint64 v = 0;
    for (int i = 0; i < len; ++i) v = (v << 1) | b[size_t(hi - i)];
    return v;
}

double sfield(const StringBits& b, int hi, int len, double lsb)
{
    const double m = double(field(b, hi - 1, len - 1)) * lsb;
    return b[size_t(hi)] ? -m : m;
}

} // namespace

bool hammingOk(const StringBits& b)
{
    for (int r = 0; r < 8; ++r)
        if (maskParity(b, r)) return false;
    return true;
}

void setHamming(StringBits& b)
{
    for (int i = 1; i <= 8; ++i) b[size_t(i)] = 0;
    for (int r = 0; r < 8; ++r) b[size_t(r + 1)] = uint8_t(maskParity(b, r));
}

std::array<int8_t, kStringSymbols> modulate(const StringBits& b)
{
    std::array<int8_t, kStringSymbols> out{};
    int d = 0;
    for (int i = 0; i < kStringBits; ++i) {
        d ^= b[size_t(kStringBits - i)];                     // relative coding
        out[size_t(2 * i)] = int8_t(d ? -1 : 1);             // Manchester pair
        out[size_t(2 * i + 1)] = int8_t(d ? 1 : -1);
    }
    for (int i = 0; i < kMarkSymbols; ++i) out[size_t(2 * kStringBits + i)] = int8_t(kMark[i] == '1' ? -1 : 1);
    return out;
}

void putField(StringBits& b, int hi, int len, quint64 value)
{
    for (int i = 0; i < len; ++i) b[size_t(hi - i)] = uint8_t((value >> (len - 1 - i)) & 1);
}

void putSigned(StringBits& b, int hi, int len, double value, double lsb)
{
    b[size_t(hi)] = value < 0 ? 1 : 0;
    putField(b, hi - 1, len - 1, quint64(std::llround(std::fabs(value) / lsb)));
}

std::array<StringBits, 5> encodeFrame(const GlonassEphemeris& e, int frameStart)
{
    constexpr double km = 1000.0;
    std::array<StringBits, 5> s{};
    for (int i = 0; i < 5; ++i) putField(s[size_t(i)], 84, 4, quint64(i + 1));
    const int h = frameStart / 3600, m = frameStart / 60 % 60;
    putField(s[0], 76, 5, quint64(h));
    putField(s[0], 71, 6, quint64(m));
    putField(s[0], 65, 1, quint64(frameStart % 60 / 30));
    putSigned(s[0], 64, 24, e.vel[0], km * std::ldexp(1.0, -20));
    putSigned(s[0], 40, 5, e.acc[0], km * std::ldexp(1.0, -30));
    putSigned(s[0], 35, 27, e.pos[0], km * std::ldexp(1.0, -11));
    putField(s[1], 80, 3, quint64(e.health));
    putField(s[1], 76, 7, quint64(std::llround(e.tb / 900.0)));
    putSigned(s[1], 64, 24, e.vel[1], km * std::ldexp(1.0, -20));
    putSigned(s[1], 40, 5, e.acc[1], km * std::ldexp(1.0, -30));
    putSigned(s[1], 35, 27, e.pos[1], km * std::ldexp(1.0, -11));
    putSigned(s[2], 79, 11, e.gammaN, std::ldexp(1.0, -40));
    putSigned(s[2], 64, 24, e.vel[2], km * std::ldexp(1.0, -20));
    putSigned(s[2], 40, 5, e.acc[2], km * std::ldexp(1.0, -30));
    putSigned(s[2], 35, 27, e.pos[2], km * std::ldexp(1.0, -11));
    putSigned(s[3], 84 - 4, 22, e.tauN, std::ldexp(1.0, -30));
    putField(s[3], 15, 5, quint64(e.slot));
    putField(s[3], 26, 11, quint64(e.dayNumber));
    putField(s[4], 80, 11, quint64(e.dayNumber));
    putSigned(s[4], 69, 32, e.tauC, std::ldexp(1.0, -27));
    putField(s[4], 36, 5, quint64(e.fourYear));
    for (StringBits& b : s) setHamming(b);
    return s;
}

NavChannel decodeChannel(const TrackingChannel& trk, const ReceiverSettings&)
{
    NavChannel nc;
    nc.channel = trk.channel;
    nc.prn = trk.prn;
    nc.eph.prn = trk.prn;
    nc.epochSec = 1e-3;
    const int n = trk.numMs;
    if (!trk.valid || n < 2 * kStringMs + kMarkMs) {
        nc.status = QStringLiteral("Too little tracking data");
        return nc;
    }

    // ---- time mark: correlate at every ms, as nothing else gives the 10 ms symbol timing
    std::vector<double> sum(size_t(n) + 1, 0.0);
    for (int i = 0; i < n; ++i) sum[size_t(i + 1)] = sum[size_t(i)] + trk.IP[size_t(i)];
    const auto symbol = [&](int ms) { return sum[size_t(ms + kSymbolMs)] - sum[size_t(ms)]; };
    std::vector<double> score(kStringMs, 0.0);
    std::vector<double> corr(size_t(n), 0.0);
    double mean = 0;
    for (int i = 0; i < n; ++i) mean += std::fabs(trk.IP[size_t(i)]);
    mean = std::max(1e-9, mean / n);
    for (int m = 0; m + kMarkMs <= n; ++m) {
        double c = 0;
        for (int i = 0; i < kMarkSymbols; ++i) c += (kMark[i] == '1' ? -1 : 1) * symbol(m + i * kSymbolMs);
        corr[size_t(m)] = c;
        score[size_t(m % kStringMs)] += std::fabs(c);
    }
    const int phase = int(std::max_element(score.begin(), score.end()) - score.begin());
    nc.bitOffset = phase;
    nc.bitSyncOk = true;
    nc.bitSyncStartMs = phase;
    nc.firstBitMs = phase;

    // ---- strings
    std::map<int, std::array<const StringBits*, 5>> frames;
    std::vector<StringBits> strings;
    std::vector<int> starts;
    strings.reserve(size_t(n / kStringMs + 1));
    for (int m = phase; m + kMarkMs + 2 * kStringBits * kSymbolMs <= n - kSymbolMs; m += kStringMs) {
        const int start = m + kMarkMs;
        const bool inverted = corr[size_t(m)] < 0;
        int previous = 0;
        StringBits b{};
        for (int i = 0; i < kStringBits; ++i) {
            const double soft = symbol(start + 2 * i * kSymbolMs) - symbol(start + (2 * i + 1) * kSymbolMs);
            nc.softBits.push_back(float(soft));
            const int d = (soft < 0) != inverted ? 1 : 0;
            b[size_t(kStringBits - i)] = uint8_t(d ^ previous);
            previous = d;
        }
        ++nc.parityTotalWords;
        if (!hammingOk(b)) continue;
        ++nc.parityOkWords;
        nc.verifiedMs.push_back(start);
        strings.push_back(b);
        starts.push_back(start);
    }

    // ---- time and ephemeris: a frame starts at the tk of its string 1
    std::array<const StringBits*, 6> latest{};
    for (size_t i = 0; i < strings.size(); ++i) {
        const StringBits& b = strings[i];
        const int k = int(field(b, 84, 4));
        if (k < 1 || k > 15) continue;
        if (k == 1) {
            const int tk = int(field(b, 76, 5)) * 3600 + int(field(b, 71, 6)) * 60 + int(field(b, 65, 1)) * 30;
            if (!nc.timingValid) {
                nc.timingValid = true;
                nc.refMs = starts[i];
                nc.refTow = tk;
            }
        }
        if (k <= 5) latest[size_t(k)] = &b;
        if (k != 4 || nc.glo.valid || !latest[1] || !latest[2] || !latest[3]) continue;
        const StringBits& s1 = *latest[1];
        const StringBits& s2 = *latest[2];
        const StringBits& s3 = *latest[3];
        const StringBits& s4 = b;
        constexpr double km = 1000.0;
        GlonassEphemeris& e = nc.glo;
        for (int a = 0; a < 3; ++a) {
            const StringBits& s = a == 0 ? s1 : a == 1 ? s2 : s3;
            e.vel[a] = sfield(s, 64, 24, km * std::ldexp(1.0, -20));
            e.acc[a] = sfield(s, 40, 5, km * std::ldexp(1.0, -30));
            e.pos[a] = sfield(s, 35, 27, km * std::ldexp(1.0, -11));
        }
        e.health = int(field(s2, 80, 3));
        e.tb = double(field(s2, 76, 7)) * 900.0;
        e.gammaN = sfield(s3, 79, 11, std::ldexp(1.0, -40));
        e.tauN = sfield(s4, 80, 22, std::ldexp(1.0, -30));
        e.slot = int(field(s4, 15, 5));
        e.dayNumber = int(field(s4, 26, 11));
        e.channel = glonassChannel(trk.prn);
        e.valid = true;
        nc.eph.sf1 = nc.eph.sf2 = nc.eph.sf3 = true;
        nc.eph.svHealth = e.health >> 2;
    }
    if (latest[5]) {
        nc.glo.tauC = sfield(*latest[5], 69, 32, std::ldexp(1.0, -27));
        nc.glo.fourYear = int(field(*latest[5], 36, 5));
    }
    nc.status = nc.glo.valid ? QStringLiteral("Ephemeris complete")
                             : QStringLiteral("Ephemeris incomplete (%1 of %2 strings passed the Hamming check)")
                                   .arg(nc.parityOkWords).arg(nc.parityTotalWords);
    return nc;
}

} // namespace glo
} // namespace gnss
