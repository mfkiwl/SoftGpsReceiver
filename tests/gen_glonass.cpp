// Writes a fully simulated GLONASS L1OF recording: complex int8 samples centred on channel 0 (1602 MHz), with white noise.
//
//   gen_glonass <out-fs10-if0-int8iq.bin> [--seconds N] [--cn0 dB-Hz]
//
// The satellites are made up: a circular 24-satellite constellation seen from a fixed point. The receiver's position is
// printed, so a solution can be compared with it. Only the troposphere is added, as there is no ionosphere data to remove it.

#include "core/Constants.h"
#include "core/Ephemeris.h"
#include "core/Geodesy.h"
#include "core/GlonassCode.h"
#include "core/GlonassNav.h"

#include <QCoreApplication>
#include <QFile>
#include <QtConcurrent>

#include <cmath>
#include <cstdio>
#include <map>
#include <random>
#include <vector>

using namespace gnss;

namespace {

constexpr double kFs = 10e6;
constexpr int kStartTime = 39615;          // seconds of the day
constexpr double kTb = 39600.0;
constexpr double kRxBias = 1.5e-4;         // receiver clock error, s
constexpr double kRxDrift = 1e-7;          // s/s
constexpr double kNoiseSigma = 20.0;       // counts per rail

struct Sat
{
    GlonassEphemeris eph;
    std::map<int, std::array<glo::StringBits, 5>> frames;
    std::vector<std::array<int8_t, glo::kStringSymbols>> symbols;   // per string since 'firstString'
    int firstString = 0;
    double elevation = 0;
};

GlonassEphemeris makeOrbit(int plane, int slot)
{
    constexpr double a = 25510e3, inc = 64.8 * kPi / 180, mu = 398600.4418e9, w = 7.292115e-5;
    const double raan = plane * 2 * kPi / 3, u = slot * kPi / 4 + plane * kPi / 12;
    const double v = std::sqrt(mu / a);
    const double cu = std::cos(u), su = std::sin(u), ci = std::cos(inc), si = std::sin(inc), co = std::cos(raan), so = std::sin(raan);
    const double r[3] = {a * (co * cu - so * su * ci), a * (so * cu + co * su * ci), a * su * si};
    const double vi[3] = {v * (-co * su - so * cu * ci), v * (-so * su + co * cu * ci), v * cu * si};
    GlonassEphemeris e;
    e.tb = kTb;
    e.dayNumber = 321;
    e.fourYear = 7;
    // Rounded to what the message can carry, so that what is transmitted is what is simulated
    const double lp = 1000 * std::ldexp(1.0, -11), lv = 1000 * std::ldexp(1.0, -20);
    e.pos[0] = std::round(r[0] / lp) * lp;
    e.pos[1] = std::round(r[1] / lp) * lp;
    e.pos[2] = std::round(r[2] / lp) * lp;
    e.vel[0] = std::round((vi[0] + w * r[1]) / lv) * lv;
    e.vel[1] = std::round((vi[1] - w * r[0]) / lv) * lv;
    e.vel[2] = std::round(vi[2] / lv) * lv;
    return e;
}

} // namespace

int main(int argc, char** argv)
{
    QCoreApplication app(argc, argv);
    if (argc < 2) {
        std::printf("usage: gen_glonass <out-fs10-if0-int8iq.bin> [--seconds N] [--cn0 dB-Hz]\n");
        return 2;
    }
    int seconds = 90;
    double cn0 = 46;
    for (int i = 2; i + 1 < argc; i += 2) {
        if (!qstrcmp(argv[i], "--seconds")) seconds = atoi(argv[i + 1]);
        if (!qstrcmp(argv[i], "--cn0")) cn0 = atof(argv[i + 1]);
    }
    Geodetic truth;
    truth.lat = 40.0 * kPi / 180;
    truth.lon = -105.0 * kPi / 180;
    truth.h = 1600;
    const Vec3 rx = geo::geodeticToEcef(truth);

    // ---- the satellites in view, each given its own frequency channel
    static const int channels[] = {-5, -2, 0, 1, 3, 4, 6};
    std::vector<Sat> sats;
    for (int p = 0; p < 3; ++p)
        for (int s = 0; s < 8 && sats.size() < 7; ++s) {
            Sat sat;
            sat.eph = makeOrbit(p, s);
            double az, el;
            geo::azimuthElevation(rx, orbit::glonassState(sat.eph, kStartTime + seconds / 2.0).pos, &az, &el);
            if (el < 15 * kPi / 180) continue;
            sat.elevation = el * 180 / kPi;
            sat.eph.slot = p * 8 + s + 1;
            sat.eph.channel = channels[sats.size()];
            sat.eph.tauN = (double(sats.size()) - 3) * 2.5e-6;
            sats.push_back(sat);
        }
    if (sats.size() < 4) { std::printf("only %d satellites in view\n", int(sats.size())); return 1; }

    const int firstString = (kStartTime - 2) / 2;
    const int strings = seconds / 2 + 3;
    for (Sat& s : sats) {
        s.firstString = firstString;
        for (int i = 0; i < strings; ++i) {
            const int second = (firstString + i) * 2, frame = second / 30 * 30, k = (second - frame) / 2;
            if (!s.frames.count(frame)) s.frames[frame] = glo::encodeFrame(s.eph, frame);
            glo::StringBits b{};
            if (k < 5) b = s.frames[frame][size_t(k)];
            else {
                glo::putField(b, 84, 4, quint64(k + 1));
                glo::setHamming(b);
            }
            s.symbols.push_back(glo::modulate(b));
        }
        std::printf("R%+d slot %d elevation %.1f\n", s.eph.channel, s.eph.slot, s.elevation);
    }

    const auto chips = GlonassCode::chips();
    const double amp = std::sqrt(std::pow(10.0, cn0 / 10) * 2 * kNoiseSigma * kNoiseSigma / kFs);

    // System time of the signal arriving at sample k, as the receiver will compute it back
    const auto arrival = [&](const Sat& s, double trueTime) {
        double tau = 0.07;
        for (int i = 0; i < 4; ++i) {
            const SatState st = orbit::glonassState(s.eph, trueTime - tau);
            const Vec3 rot = geo::rotateEarth(st.pos, tau);
            double az, el;
            geo::azimuthElevation(rx, rot, &az, &el);
            tau = ((rot - rx).norm() + geo::tropoSaastamoinen(truth.lat, truth.h, el)) / kSpeedOfLight;
        }
        const double te = trueTime - tau;
        return te + orbit::glonassState(s.eph, te).clockBias;
    };

    const auto makeSecond = [&](int sec) {
        const int n = int(kFs);
        std::vector<float> I(size_t(n), 0.0f), Q(size_t(n), 0.0f);
        const int block = 10000;
        for (const Sat& s : sats) {
            const double df = s.eph.channel * kGlonassL1Step;
            const double lambda = kSpeedOfLight / glonassFreq(s.eph.channel);
            for (int b = 0; b < n; b += block) {
                const double t0 = sec + double(b) / kFs, t1 = t0 + block / kFs;
                const auto trueTime = [](double t) { return kStartTime + t - (kRxBias + kRxDrift * t); };
                const double T0 = arrival(s, trueTime(t0)), T1 = arrival(s, trueTime(t1));
                const double rho0 = kSpeedOfLight * (kStartTime + t0 - T0), rho1 = kSpeedOfLight * (kStartTime + t1 - T1);
                const double dT = (T1 - T0) / block;
                const double ph0 = df * t0 - rho0 / lambda, dph = (df * (t1 - t0) - (rho1 - rho0) / lambda) / block;
                for (int i = 0; i < block; ++i) {
                    const double T = T0 + i * dT;
                    const double ms = T * 1000.0, whole = std::floor(ms);
                    const int chip = std::min(kGlonassCodeLength - 1, int((ms - whole) * kGlonassCodeLength));
                    const int str = int(std::floor(T / 2));
                    const int sym = std::min(glo::kStringSymbols - 1, int((T - str * 2.0) * 100.0));
                    const double a = amp * chips[size_t(chip)] * s.symbols[size_t(str - s.firstString)][size_t(sym)];
                    const double p = kTwoPi * (ph0 + i * dph);
                    I[size_t(b + i)] += float(a * std::cos(p));
                    Q[size_t(b + i)] += float(a * std::sin(p));
                }
            }
        }
        std::mt19937 gen(unsigned(sec + 1));
        std::normal_distribution<float> noise(0.0f, float(kNoiseSigma));
        QByteArray out(2 * n, Qt::Uninitialized);
        for (int i = 0; i < n; ++i) {
            out[2 * i] = char(std::clamp(int(std::lround(I[size_t(i)] + noise(gen))), -127, 127));
            out[2 * i + 1] = char(std::clamp(int(std::lround(Q[size_t(i)] + noise(gen))), -127, 127));
        }
        return out;
    };

    QFile f(QString::fromLocal8Bit(argv[1]));
    if (!f.open(QIODevice::WriteOnly)) { std::printf("cannot write %s\n", argv[1]); return 1; }
    for (int s = 0; s < seconds;) {
        QList<int> batch;
        for (int i = 0; i < 16 && s < seconds; ++i) batch << s++;
        const QList<QByteArray> parts = QtConcurrent::blockingMapped(batch, makeSecond);
        for (const QByteArray& p : parts) f.write(p);
        std::printf("%d s\n", s);
        std::fflush(stdout);
    }
    std::printf("Truth: lat %.8f lon %.8f h %.2f, %d satellites, C/N0 %.1f dB-Hz\n", truth.lat * 180 / kPi, truth.lon * 180 / kPi, truth.h, int(sats.size()), cn0);
    return 0;
}
