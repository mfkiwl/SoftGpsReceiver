// Adds simulated Galileo E1 satellites to a GPS recording, so that the Galileo path can be tried on a file that has none.
//
//   add_galileo <in.bin> <out.bin> [--sats N] [--cn0 dB-Hz] [--isb metres]
//
// The receiver first solves the recording with GPS alone. Its position, and its clock as a function of sample index,
// then decide when each simulated signal arrives, so the satellites agree with the GPS ones already in the file:
//
//   - a Walker 24/3/1 constellation of made-up satellites, with broadcast ephemerides that describe them exactly
//   - E1-B and E1-C with sine-phased BOC(1,1) subcarriers at equal power, and the CS25 secondary code on the pilot
//   - I/NAV pages (word types 0 to 6, FEC, interleaving and CRC). There are no almanac words: spare words take their place
//   - the delays that the receiver removes again: Earth rotation, troposphere and, if the file has the data, ionosphere
//
// The recording's samples are multiplied by a whole number first. A signal is well below one count, so without that it
// would vanish when rounded back to integers. The GPS signals are unchanged apart from the scale.

#include "core/Constants.h"
#include "core/Ephemeris.h"
#include "core/GalileoCodes.h"
#include "core/GalileoNav.h"
#include "core/Geodesy.h"
#include "core/Receiver.h"
#include "core/SignalAnalysis.h"

#include <QCoreApplication>
#include <QFile>
#include <QFileInfo>
#include <QThread>
#include <QtConcurrent>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <vector>

using namespace gnss;

namespace {

constexpr double kHalfChipsPerSec = 2.0 * kCaChipRate;      // BOC(1,1): two subcarrier values per chip
constexpr int kHalfChipsPerCode = 2 * kE1CodeLength;
constexpr double kOrbitRadius = 29600.0e3;                  // m
constexpr const char* kCs25 = "0011100000001010110110010";  // E1-C secondary code, logic 0 = +1

double p2(int e) { return std::ldexp(1.0, e); }
double quant(double v, double lsb) { return std::round(v / lsb) * lsb; }

double wrapPi(double a)
{
    return a - kTwoPi * std::floor((a + kPi) / kTwoPi);
}

// The receiver's clock from its GPS solution: true time as a function of sample index, smoothed by a polynomial because
// single fixes carry metres of noise. Beyond the fixes it continues in a straight line.
class RxClock
{
public:
    double fit(const std::vector<NavSolution>& sols, double fs)
    {
        fs_ = fs;
        std::vector<double> u, y;
        double lo = 1e300, hi = -1e300;
        for (const NavSolution& s : sols)
            if (s.valid) { lo = std::min(lo, s.rxSample); hi = std::max(hi, s.rxSample); }
        mid_ = (lo + hi) / 2;
        half_ = (hi - lo) / 2;
        for (const NavSolution& s : sols) {
            if (!s.valid) continue;
            if (y.empty()) t0_ = s.gpsTime - s.rxSample / fs_;
            u.push_back((s.rxSample - mid_) / half_);
            y.push_back(s.gpsTime - s.rxSample / fs_ - t0_);
        }
        double a[kTerms][kTerms + 1] = {};
        for (size_t n = 0; n < u.size(); ++n) {
            double pi[kTerms];
            pi[0] = 1;
            for (int i = 1; i < kTerms; ++i) pi[i] = pi[i - 1] * u[n];
            for (int i = 0; i < kTerms; ++i) {
                for (int j = 0; j < kTerms; ++j) a[i][j] += pi[i] * pi[j];
                a[i][kTerms] += pi[i] * y[n];
            }
        }
        for (int i = 0; i < kTerms; ++i) {
            int p = i;
            for (int r = i + 1; r < kTerms; ++r)
                if (std::fabs(a[r][i]) > std::fabs(a[p][i])) p = r;
            std::swap_ranges(a[i], a[i] + kTerms + 1, a[p]);
            for (int r = 0; r < kTerms; ++r) {
                if (r == i) continue;
                const double f = a[r][i] / a[i][i];
                for (int c = i; c <= kTerms; ++c) a[r][c] -= f * a[i][c];
            }
        }
        for (int i = 0; i < kTerms; ++i) c_[i] = a[i][kTerms] / a[i][i];
        double e2 = 0;
        for (size_t n = 0; n < u.size(); ++n) e2 += std::pow(y[n] - value(u[n]), 2);
        return std::sqrt(e2 / double(u.size())) * kSpeedOfLight;
    }

    // Time of week if the sample clock were perfect
    double nominal(double k) const { return t0_ + k / fs_; }
    // True time minus that. Kept apart because a time of week in a double only resolves 9 mm, which is 5 % of a carrier cycle
    double offset(double k) const
    {
        const double u = (k - mid_) / half_;
        if (u > 1) return value(1) + slope(1) * (u - 1);
        if (u < -1) return value(-1) + slope(-1) * (u + 1);
        return value(u);
    }
    double trueTime(double k) const { return nominal(k) + offset(k); }

private:
    static constexpr int kTerms = 6;

    double value(double u) const
    {
        double v = 0;
        for (int i = kTerms - 1; i >= 0; --i) v = v * u + c_[i];
        return v;
    }
    double slope(double u) const
    {
        double v = 0;
        for (int i = kTerms - 1; i >= 1; --i) v = v * u + i * c_[i];
        return v;
    }

    double fs_ = 1, mid_ = 0, half_ = 1, t0_ = 0;
    double c_[kTerms] = {};
};

struct Scene
{
    double fs = 0, ifFreq = 0;
    double sign = 1;             // Doppler sign of the front end
    double carrierDrift = 0;     // m/s by which the carrier's clock drift differs from the sample clock's
    Vec3 rx;
    Geodetic llh;
    double isb = 0;              // m: Galileo pseudoranges are longer by this
    bool tropo = true;
    bool iono = false;
    IonoUtc ionoUtc;
    RxClock clock;
};

struct Satellite
{
    int svid = 0;
    Ephemeris eph;
    double el = 0, az = 0;       // degrees, in the middle of the file
    double cn0 = 0;
    double amplitude = 0;        // of the carrier, in input counts
    std::vector<int8_t> bocB, bocC;
    std::vector<int8_t> symbols; // +1/-1 from baseTime on
    double baseTime = 0;         // whole, even second
};

// ---- orbit ------------------------------------------------------------------------------------------------------

// Rounds to what the words can carry, so that the receiver computes the same orbit as the simulation
Ephemeris broadcastEphemeris(int svid, int plane, int slot, double toe, int week, double raan)
{
    const double sc = kGpsPi;
    const double e = 0.0002 + 0.00002 * ((plane * 8 + slot) % 7);
    const double omega = wrapPi((20.0 + 13.0 * slot) * kPi / 180);
    const double u = (45.0 * slot + 15.0 * plane) * kPi / 180;
    Ephemeris g;
    g.prn = kGalileoIdBase + svid;
    g.weekNumber = week;
    g.toe = g.toc = toe;
    g.iode2 = g.iode3 = g.iodc = 40 + svid;
    g.uraIndex = 107;
    g.sqrtA = quant(std::sqrt(kOrbitRadius), p2(-19));
    g.e = quant(e, p2(-33));
    g.m0 = quant(wrapPi(u - omega), p2(-31) * sc);
    g.omega = quant(omega, p2(-31) * sc);
    g.i0 = quant(56.0 * kPi / 180, p2(-31) * sc);
    g.omega0 = quant(wrapPi(raan + plane * 120.0 * kPi / 180 + kOmegaEDot * toe), p2(-31) * sc);
    g.omegaDot = quant(-2.9e-9, p2(-43) * sc);
    g.idot = quant(1.2e-10, p2(-43) * sc);
    g.deltaN = quant(3.1e-9, p2(-43) * sc);
    g.cuc = quant(-2.1e-6, p2(-29));
    g.cus = quant(8.3e-6, p2(-29));
    g.crc = quant(151.0, p2(-5));
    g.crs = quant(-42.0, p2(-5));
    g.cic = quant(3.0e-8, p2(-29));
    g.cis = quant(-2.0e-8, p2(-29));
    g.af0 = quant((svid % 2 ? 1.0 : -1.0) * 1.1e-4 * (1 + svid % 5), p2(-34));
    g.af1 = quant((svid % 3 ? 1.0 : -1.0) * 2.4e-12, p2(-46));
    g.sf1 = g.sf2 = g.sf3 = true;
    return g;
}

// ---- navigation message -----------------------------------------------------------------------------------------

struct Word
{
    std::array<uint8_t, 128> bits{};
    void put(int start, int len, qint64 v)
    {
        for (int i = 0; i < len; ++i) bits[size_t(start + i)] = uint8_t((v >> (len - 1 - i)) & 1);
    }
    void put(int start, int len, double v, double lsb) { put(start, len, qint64(std::llround(v / lsb))); }
};

// Field positions are those of inav::decodeFields
Word makeWord(int type, const Ephemeris& g, int week, int tow)
{
    const double sc = kGpsPi;
    Word w;
    w.put(0, 6, qint64(type));
    switch (type) {
    case 0:
        w.put(6, 2, qint64(2));
        w.put(96, 12, qint64(week));
        w.put(108, 20, qint64(tow));
        break;
    case 1:
        w.put(6, 10, qint64(g.iode2));
        w.put(16, 14, qint64(std::llround(g.toe / 60)));
        w.put(30, 32, g.m0, p2(-31) * sc);
        w.put(62, 32, g.e, p2(-33));
        w.put(94, 32, g.sqrtA, p2(-19));
        break;
    case 2:
        w.put(6, 10, qint64(g.iode2));
        w.put(16, 32, g.omega0, p2(-31) * sc);
        w.put(48, 32, g.i0, p2(-31) * sc);
        w.put(80, 32, g.omega, p2(-31) * sc);
        w.put(112, 14, g.idot, p2(-43) * sc);
        break;
    case 3:
        w.put(6, 10, qint64(g.iode2));
        w.put(16, 24, g.omegaDot, p2(-43) * sc);
        w.put(40, 16, g.deltaN, p2(-43) * sc);
        w.put(56, 16, g.cuc, p2(-29));
        w.put(72, 16, g.cus, p2(-29));
        w.put(88, 16, g.crc, p2(-5));
        w.put(104, 16, g.crs, p2(-5));
        w.put(120, 8, qint64(g.uraIndex));
        break;
    case 4:
        w.put(6, 10, qint64(g.iode2));
        w.put(16, 6, qint64(svidOf(g.prn)));
        w.put(22, 16, g.cic, p2(-29));
        w.put(38, 16, g.cis, p2(-29));
        w.put(54, 14, qint64(std::llround(g.toc / 60)));
        w.put(68, 31, g.af0, p2(-34));
        w.put(99, 21, g.af1, p2(-46));
        w.put(120, 6, g.af2, p2(-59));
        break;
    case 5:
        w.put(6, 11, 60.0, p2(-2));             // a_i0: NeQuick, which this receiver does not use
        w.put(73, 12, qint64(week));
        w.put(85, 20, qint64(tow));
        break;
    case 6:
        w.put(62, 8, qint64(16));               // leap seconds
        w.put(105, 20, qint64(tow));
        break;
    }
    return w;
}

// The word sent at a given even second of the 30 s subframe
int wordType(int tow)
{
    switch (tow % 30) {
    case 0: return 2;
    case 2: return 4;
    case 4: return 6;
    case 20: return 1;
    case 22: return 3;
    case 24: return 5;
    default: return 0;
    }
}

int parity(unsigned v)
{
    int p = 0;
    for (; v; v &= v - 1) p ^= 1;
    return p;
}

// A page part: 10 sync symbols, then the 120 bits coded at rate 1/2 and interleaved. The inverse of inav::viterbi and
// inav::deinterleave.
void encodePart(const uint8_t* bits, int8_t* symbols)
{
    static const int sync[inav::kSyncSymbols] = {0, 1, 0, 1, 1, 0, 0, 0, 0, 0};
    for (int i = 0; i < inav::kSyncSymbols; ++i) symbols[i] = int8_t(1 - 2 * sync[i]);
    uint8_t coded[inav::kCodedSymbols];
    unsigned st = 0;
    for (int t = 0; t < inav::kPartBits; ++t) {
        const unsigned reg = (unsigned(bits[t]) << 6) | st;
        coded[2 * t] = uint8_t(parity(reg & 0171));
        coded[2 * t + 1] = uint8_t(parity(reg & 0133) ^ 1);
        st = reg >> 1;
    }
    for (int r = 0; r < 8; ++r)
        for (int c = 0; c < 30; ++c)
            symbols[inav::kSyncSymbols + r * 30 + c] = int8_t(1 - 2 * coded[c * 8 + r]);
}

void encodePage(const Word& w, int8_t* symbols)
{
    uint8_t even[inav::kPartBits] = {}, odd[inav::kPartBits] = {};
    std::copy_n(w.bits.begin(), 112, even + 2);
    odd[0] = 1;
    std::copy_n(w.bits.begin() + 112, 16, odd + 2);
    uint8_t covered[196];
    std::copy_n(even, 114, covered);
    std::copy_n(odd, 82, covered + 114);
    const uint32_t crc = inav::crc24q(covered, 196);
    for (int i = 0; i < 24; ++i) odd[82 + i] = uint8_t((crc >> (23 - i)) & 1);
    encodePart(even, symbols);
    encodePart(odd, symbols + kInavPartSymbols);
}

// ---- signal -----------------------------------------------------------------------------------------------------

struct Emission
{
    double code;                 // time on the satellite's clock of what arrives at this sample, from baseTime (s)
    double cycles;               // carrier phase at IF
};

// The receiver's measurement equation run backwards: see buildObservations() and evaluate() in Pvt.cpp
Emission emission(const Scene& sc, const Satellite& sat, double k)
{
    const double t = sc.clock.trueTime(k);
    double tau = 0.08, satClock = 0, ionoDelay = 0;
    for (int i = 0; i < 4; ++i) {
        const SatState st = orbit::satState(sat.eph, t - tau);
        const Vec3 rot = geo::rotateEarth(st.pos, (st.pos - sc.rx).norm() / kSpeedOfLight);
        double az = 0, el = 0;
        geo::azimuthElevation(sc.rx, rot, &az, &el);
        const double tropo = sc.tropo ? geo::tropoSaastamoinen(sc.llh.lat, sc.llh.h, el) : 0.0;
        ionoDelay = sc.iono ? geo::ionoKlobuchar(sc.ionoUtc.alpha, sc.ionoUtc.beta, sc.llh.lat, sc.llh.lon, az, el, t - tau) : 0.0;
        satClock = st.clockBias;
        tau = ((rot - sc.rx).norm() + tropo + ionoDelay + sc.isb) / kSpeedOfLight;
    }
    // Carrier phase follows the pseudorange on the receiver's own clock, and the ionosphere advances it
    const double range = kSpeedOfLight * (tau - satClock - sc.clock.offset(k)) - 2 * ionoDelay + sc.carrierDrift * k / sc.fs;
    return {t - sat.baseTime - tau + satClock, sc.ifFreq * k / sc.fs - sc.sign * range / kL1Wavelength};
}

// Adds one satellite to out[0..n), which holds samples first... of the file
void addSatellite(const Scene& sc, const Satellite& sat, qint64 first, int n, float* out)
{
    static const std::vector<float> cosine = [] {
        std::vector<float> c(4096);
        for (int i = 0; i < 4096; ++i) c[size_t(i)] = float(std::cos(kTwoPi * (i + 0.5) / 4096));
        return c;
    }();
    const float gain = float(sat.amplitude / std::sqrt(2.0));   // the two components share the power
    const int block = std::max(1, int(sc.fs / 1000));
    Emission a = emission(sc, sat, double(first));
    for (int at = 0; at < n; at += block) {
        const int len = std::min(block, n - at);
        const Emission b = emission(sc, sat, double(first + at + len));
        const double h0 = a.code * kHalfChipsPerSec;
        const double dh = (b.code - a.code) * kHalfChipsPerSec / len;
        const double frac = a.cycles - std::floor(a.cycles);
        const uint32_t ph0 = uint32_t(frac * 4294967296.0);
        const uint32_t dph = uint32_t(qint64(std::llround((b.cycles - a.cycles) / len * 4294967296.0)));
        float* y = out + at;
        for (int i = 0; i < len; ++i) {
            const qint64 h = qint64(std::floor(h0 + i * dh));
            const int chip = int(h % kHalfChipsPerCode);
            const qint64 epoch = h / kHalfChipsPerCode;
            const int data = sat.symbols[size_t(epoch)];
            const int pilot = kCs25[epoch % 25] == '0' ? 1 : -1;
            const int v = sat.bocB[size_t(chip)] * data - sat.bocC[size_t(chip)] * pilot;
            y[i] += gain * float(v) * cosine[(ph0 + uint32_t(i) * dph) >> 20];
        }
        a = b;
    }
}

// ---- recording --------------------------------------------------------------------------------------------------

struct Chunk
{
    qint64 first = 0;
    QByteArray in, out;
    qint64 clipped = 0;
};

// Noise density of the recording around the IF (counts^2/Hz), which sets the signal levels
double noiseDensity(const SignalSource& src, double fs, double fif)
{
    const Psd p = analysis::welchPsd(src, fs, 0, qint64(fs), 8192);
    double sum = 0;
    int n = 0;
    for (size_t i = 0; i < p.freq.size(); ++i)
        if (std::fabs(p.freq[i] - fif) < 1.2e6) { sum += std::pow(10.0, p.powerDb[i] / 10); ++n; }
    return n ? sum / n : 0;
}

int largestSample(QFile& f)
{
    int most = 0;
    for (int part = 0; part < 16; ++part) {
        f.seek(f.size() / 16 * part);
        const QByteArray b = f.read(1 << 22);
        for (char v : b) most = std::max(most, std::abs(int(static_cast<signed char>(v))));
    }
    f.seek(0);
    return most;
}

int usage()
{
    std::printf("usage: add_galileo <in.bin> <out.bin> [--sats N] [--cn0 dB-Hz] [--isb metres]\n");
    return 2;
}

} // namespace

int main(int argc, char** argv)
{
    QCoreApplication app(argc, argv);
    const QStringList args = app.arguments();
    if (args.size() < 3) return usage();
    const QString inPath = args[1], outPath = args[2];
    int wanted = 6;
    double cn0Zenith = 47.0, isb = 3.0;
    for (int i = 3; i + 1 < args.size(); i += 2) {
        if (args[i] == "--sats") wanted = args[i + 1].toInt();
        else if (args[i] == "--cn0") cn0Zenith = args[i + 1].toDouble();
        else if (args[i] == "--isb") isb = args[i + 1].toDouble();
        else return usage();
    }
    if (QFileInfo(inPath).absoluteFilePath() == QFileInfo(outPath).absoluteFilePath()) {
        std::printf("The output must be a new file\n");
        return 2;
    }

    // ---- what the recording says about the receiver
    Receiver rx;
    QString err;
    if (!rx.openFile(inPath, &err)) {
        std::printf("%s\n", qPrintable(err));
        return 1;
    }
    if (rx.settings.format != SampleFormat::Int8Real) {
        std::printf("Only int8 real recordings are supported\n");
        return 1;
    }
    rx.settings.constellations = Constellations::GpsOnly;
    std::printf("Solving the recording with GPS...\n");
    std::fflush(stdout);
    if (!rx.runAll() || rx.pvt.nValid == 0) {
        std::printf("No GPS solution: %s\n", qPrintable(rx.pvt.error));
        return 1;
    }
    Scene sc;
    sc.fs = rx.settings.samplingFreq;
    sc.ifFreq = rx.settings.ifFreq;
    sc.sign = rx.pvt.dopplerSign;
    sc.carrierDrift = rx.pvt.meanDrift - rx.pvt.biasDrift;
    sc.rx = rx.pvt.meanPos;
    sc.llh = rx.pvt.meanLlh;
    sc.isb = isb;
    sc.tropo = rx.settings.useTropo;
    sc.iono = rx.settings.useIono && rx.gnav.ionoUtc.valid;
    sc.ionoUtc = rx.gnav.ionoUtc;
    const double clockRms = sc.clock.fit(rx.pvt.sols, sc.fs);
    const int gstWeek = rx.pvt.weekNumber;                  // GST weeks count from the first GPS rollover
    const qint64 total = QFileInfo(inPath).size();
    const double midTime = sc.clock.trueTime(double(total) / 2);
    const double endTime = sc.clock.trueTime(double(total));
    const double toe = std::floor(sc.clock.trueTime(0) / 600) * 600;
    std::printf("Receiver at %.7f, %.7f, %.1f m; clock fit to %.2f m RMS; carrier drift differs by %.3f m/s\n",
                sc.llh.lat * 180 / kPi, sc.llh.lon * 180 / kPi, sc.llh.h, clockRms, sc.carrierDrift);

    // ---- the satellites in view: the highest, up to the number asked for
    std::vector<Satellite> sats;
    for (int plane = 0, svid = 1; plane < 3; ++plane)
        for (int slot = 0; slot < 8; ++slot, ++svid) {
            Satellite s;
            s.svid = svid;
            s.eph = broadcastEphemeris(svid, plane, slot, toe, gstWeek, 40.0 * kPi / 180);
            const SatState st = orbit::satState(s.eph, midTime);
            double az = 0, el = 0;
            geo::azimuthElevation(sc.rx, st.pos, &az, &el);
            s.el = el * 180 / kPi;
            s.az = az * 180 / kPi;
            if (s.el > 10) sats.push_back(std::move(s));
        }
    std::sort(sats.begin(), sats.end(), [](const Satellite& a, const Satellite& b) { return a.el > b.el; });
    if (int(sats.size()) > wanted) sats.resize(size_t(wanted));
    if (sats.size() < 4) std::printf("Warning: only %d satellites in view\n", int(sats.size()));

    const double n0 = noiseDensity(rx.source, sc.fs, sc.ifFreq);
    const double base = 2 * std::floor((sc.clock.trueTime(0) - 2) / 2);
    const int pages = int((endTime + 4 - base) / 2);
    double peak = 0;
    for (Satellite& s : sats) {
        s.cn0 = cn0Zenith - 6 * (1 - std::sin(s.el * kPi / 180));
        s.amplitude = std::sqrt(2 * n0 * std::pow(10.0, s.cn0 / 10));
        peak += std::sqrt(2.0) * s.amplitude;
        s.bocB = GalileoCode::boc11(GalileoCode::e1b(s.svid));
        s.bocC = GalileoCode::boc11(GalileoCode::e1c(s.svid));
        s.baseTime = base;
        s.symbols.resize(size_t(pages) * 2 * kInavPartSymbols);
        for (int p = 0; p < pages; ++p) {
            const int tow = int(base) + 2 * p;
            encodePage(makeWord(wordType(tow), s.eph, gstWeek, tow), s.symbols.data() + size_t(p) * 2 * kInavPartSymbols);
        }
    }

    QFile in(inPath), out(outPath);
    if (!in.open(QIODevice::ReadOnly) || !out.open(QIODevice::WriteOnly)) {
        std::printf("Cannot open the files: %s %s\n", qPrintable(in.errorString()), qPrintable(out.errorString()));
        return 1;
    }
    const int scale = std::max(1, int(120.0 / (largestSample(in) + peak)));

    std::printf("Noise density %.2f dB re counts^2/Hz; samples multiplied by %d; GST week %d, ephemeris reference %.0f s\n",
                10 * std::log10(n0), scale, gstWeek, toe);
    std::printf("Satellite  elevation  azimuth  C/N0   amplitude (counts)\n");
    for (const Satellite& s : sats)
        std::printf("   E%02d      %5.1f     %5.1f   %4.1f   %.3f\n", s.svid, s.el, s.az, s.cn0, s.amplitude);
    std::printf("Galileo minus GPS time offset: %.2f m\n", isb);
    std::fflush(stdout);

    // ---- a second of samples at a time, on all processors
    const qint64 chunkBytes = qint64(sc.fs);
    const int threads = std::max(1, QThread::idealThreadCount());
    std::vector<Chunk> batch(static_cast<size_t>(threads));
    qint64 done = 0, clipped = 0;
    const auto work = [&](Chunk& c) {
        const int n = int(c.in.size());
        std::vector<float> sum(static_cast<size_t>(n), 0.f);
        for (const Satellite& s : sats) addSatellite(sc, s, c.first, n, sum.data());
        c.out.resize(n);
        c.clipped = 0;
        for (int i = 0; i < n; ++i) {
            const int v = int(std::lround(scale * (float(static_cast<signed char>(c.in[i])) + sum[size_t(i)])));
            c.clipped += v > 127 || v < -128 ? 1 : 0;
            c.out[i] = char(std::clamp(v, -128, 127));
        }
    };
    while (done < total) {
        int used = 0;
        for (Chunk& c : batch) {
            c.first = done;
            c.in = in.read(chunkBytes);
            if (c.in.isEmpty()) break;
            done += c.in.size();
            ++used;
        }
        QtConcurrent::blockingMap(batch.begin(), batch.begin() + used, work);
        for (int i = 0; i < used; ++i) {
            if (out.write(batch[size_t(i)].out) != batch[size_t(i)].out.size()) {
                std::printf("Writing failed: %s\n", qPrintable(out.errorString()));
                return 1;
            }
            clipped += batch[size_t(i)].clipped;
        }
        std::printf("%.0f s of %.0f s\n", double(done) / sc.fs, double(total) / sc.fs);
        std::fflush(stdout);
    }
    out.close();
    std::printf("Wrote %s (%lld bytes), %lld samples clipped\n", qPrintable(outPath), (long long)total, (long long)clipped);
    return 0;
}
