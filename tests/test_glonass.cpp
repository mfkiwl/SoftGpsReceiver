// Checks the GLONASS code and the satellite id scheme.

#include "core/Acquisition.h"
#include "core/Constants.h"
#include "core/Ephemeris.h"
#include "core/GlonassCode.h"
#include "core/GlonassNav.h"
#include "core/Settings.h"
#include "core/SignalSource.h"
#include "core/Tracking.h"

#include <QDir>
#include <QFile>
#include <QTemporaryDir>
#include <cmath>
#include <random>
#include <vector>

#include <cstdio>

using namespace gnss;

static int failures = 0;

static void check(bool ok, const char* what)
{
    std::printf("%s  %s\n", ok ? "ok    " : "FAILED", what);
    failures += ok ? 0 : 1;
}

// 4 s of one satellite on channel +2 with 50 Hz data, at 48 dB-Hz in white noise
static void testSearchAndTrack()
{
    QTemporaryDir dir;
    const QString path = QDir(dir.path()).filePath(QStringLiteral("t-fs20-if0-int16iq.bin"));
    const double fs = 20e6, doppler = 1730, sigma = 300, delayChips = 123.4;
    // complex noise power 2 sigma^2 over fs gives N0; carrier power is amp^2
    const double amp = std::sqrt(std::pow(10.0, 4.8) * 2 * sigma * sigma / fs);
    const auto c = GlonassCode::chips();
    std::mt19937 gen(1);
    std::normal_distribution<float> noise(0.0f, float(sigma));
    QFile f(path);
    f.open(QIODevice::WriteOnly);
    std::vector<qint16> buf(2000000);
    const double carrier = 2 * kGlonassL1Step + doppler;
    const double chipRate = kGlonassChipRate * (1 + doppler / glonassFreq(2));
    for (qint64 k = 0, n = qint64(4 * fs); k < n;) {
        const int block = int(std::min<qint64>(n - k, 1000000));
        for (int i = 0; i < block; ++i, ++k) {
            const double t = k / fs, chip = chipRate * t + kGlonassCodeLength - delayChips;
            const int bit = (int(t * 50) * 7 % 3) ? 1 : -1;
            const double a = amp * bit * c[size_t(qint64(chip) % kGlonassCodeLength)];
            const double p = kTwoPi * carrier * t;
            buf[size_t(2 * i)] = qint16(std::lround(a * std::cos(p) + noise(gen)));
            buf[size_t(2 * i + 1)] = qint16(std::lround(a * std::sin(p) + noise(gen)));
        }
        f.write(reinterpret_cast<const char*>(buf.data()), block * 4);
    }
    f.close();

    ReceiverSettings s;
    s.samplingFreq = fs;
    s.ifFreq = 0;
    s.format = SampleFormat::Int16IQ;
    s.constellations = Constellations::GlonassOnly;
    SignalSource src;
    QString err;
    check(src.open(path, s.format, 0, &err), "opened the test signal");
    const AcquisitionOutput out = Acquisition::run(src, s, nullptr, nullptr);
    const AcqResult* hit = nullptr;
    int count = 0;
    for (const AcqResult& r : out.results)
        if (r.acquired) { ++count; if (r.prn == glonassId(2)) hit = &r; }
    std::printf("acquired %d channel(s)\n", count);
    check(hit != nullptr, "found channel +2");
    check(count == 1, "no false channels");
    if (!hit) return;
    std::printf("Doppler %.1f Hz\n", hit->doppler);
    check(std::fabs(hit->doppler - doppler) < 15, "fine Doppler within 15 Hz");
    TrackingChannel ch;
    ChannelTracker tr(s, *hit, hit->codePhase, true, ch);
    while (tr.step(src, src.numSamples())) {}
    const double finalDoppler = ch.carrFreq.back() - s.glonassIf(2);
    std::printf("%d ms tracked, C/N0 %.1f, Doppler %.1f, PLI %.2f\n", int(ch.carrFreq.size()), ch.cn0.back(), finalDoppler, ch.pli.back());
    check(std::fabs(finalDoppler - doppler) < 5 && ch.pli.back() > 0.7, "tracking holds phase lock on the right frequency");
}

// Three minutes of prompt correlator output carrying frames, with noise, a cycle slip's polarity and an offset start
static void testMessage()
{
    GlonassEphemeris e;
    e.pos[0] = 12345678.5; e.pos[1] = -9876543.0; e.pos[2] = 20000000.25;
    e.vel[0] = -1234.5; e.vel[1] = 2345.25; e.vel[2] = 345.75;
    e.acc[0] = 1.86e-6; e.acc[1] = -2.79e-6; e.acc[2] = 0;
    e.tb = 900 * 44;
    e.tauN = -1.25e-5;
    e.gammaN = 9.094947e-13 * 3;
    e.slot = 7;
    e.dayNumber = 321;
    glo::StringBits flip = glo::encodeFrame(e, 39600)[0];
    check(glo::hammingOk(flip), "Hamming check accepts an encoded string");
    bool caught = true;
    for (int i = 1; i <= 84; ++i) {
        flip[size_t(i)] ^= 1;
        caught = caught && !glo::hammingOk(flip);
        flip[size_t(i)] ^= 1;
    }
    check(caught, "Hamming check rejects every single-bit error");

    std::mt19937 gen(2);
    std::normal_distribution<float> noise(0.0f, 1.0f);
    TrackingChannel t;
    t.valid = true;
    t.prn = glonassId(-3);
    const int offsetMs = 1234;
    for (int frame = 0; frame < 6; ++frame) {
        const auto strings = glo::encodeFrame(e, 39600 + 30 * frame);
        for (int k = 0; k < 15; ++k) {
            glo::StringBits spare{};
            glo::putField(spare, 84, 4, quint64(k + 1));
            glo::setHamming(spare);
            const auto sym = glo::modulate(k < 5 ? strings[size_t(k)] : spare);
            for (int i = 0; i < glo::kStringSymbols; ++i)
                for (int ms = 0; ms < 10; ++ms)
                    if (frame * 30000 + k * 2000 + i * 10 + ms >= offsetMs)
                        t.IP.push_back(-1.5f * sym[size_t(i)] + noise(gen));
        }
    }
    t.numMs = int(t.IP.size());
    const NavChannel n = glo::decodeChannel(t, ReceiverSettings());
    std::printf("%d of %d strings ok, timing %d, ref ms %d, tow %.0f\n", n.parityOkWords, n.parityTotalWords, n.timingValid, n.refMs, n.refTow);
    check(n.parityOkWords >= n.parityTotalWords - 1 && n.parityTotalWords > 80, "strings pass in noise and inverted");
    check(n.glo.valid && n.glo.slot == 7 && n.glo.dayNumber == 321, "ephemeris assembled");
    check(std::fabs(n.glo.pos[0] - e.pos[0]) < 0.3 && std::fabs(n.glo.pos[1] - e.pos[1]) < 0.3 && std::fabs(n.glo.pos[2] - e.pos[2]) < 0.3, "position");
    check(std::fabs(n.glo.vel[1] - e.vel[1]) < 0.001 && std::fabs(n.glo.acc[1] - e.acc[1]) < 1e-6 && n.glo.tb == e.tb, "velocity, acceleration, tb");
    check(std::fabs(n.glo.tauN - e.tauN) < 1e-9, "clock");
    check(n.timingValid && std::fabs(n.txTimeAtMs(n.refMs + 4000) - (n.refTow + 4)) < 1e-9 && n.refMs == 30000 - offsetMs && n.refTow == 39630, "time of frame start");
}

static void testOrbit()
{
    GlonassEphemeris e;
    e.pos[0] = 25510e3; e.vel[1] = 1000.0; e.vel[2] = 3000.0;
    e.tb = 1000;
    const SatState a = orbit::glonassState(e, 1900);
    GlonassEphemeris back = e;
    back.tb = 1900;
    for (int i = 0; i < 3; ++i) {
        back.pos[i] = i == 0 ? a.pos.x : i == 1 ? a.pos.y : a.pos.z;
        back.vel[i] = i == 0 ? a.vel.x : i == 1 ? a.vel.y : a.vel.z;
    }
    const SatState b = orbit::glonassState(back, 1000);
    const double err = (b.pos - Vec3(25510e3, 0, 0)).norm();
    std::printf("orbit radius %.0f km after 900 s, return error %.4f m\n", a.pos.norm() / 1000, err);
    check(err < 0.01, "integrating forward and back returns to the start");
    check(a.pos.norm() > 24000e3 && a.pos.norm() < 27000e3, "radius stays near the orbit");
    double bias = 0;
    e.tauN = 1e-5;
    e.gammaN = 1e-11;
    const double t = orbit::glonassTime(e, 1100, &bias);
    check(std::fabs(t - (1100 + 1e-5 - 1e-9)) < 1e-12, "clock correction follows t = tsv + tauN - gamma (t - tb)");
}

int main()
{
    testOrbit();
    testMessage();
    testSearchAndTrack();
    const auto c = GlonassCode::chips();
    bool sidelobes = true;
    for (int lag = 1; lag < kGlonassCodeLength; ++lag) {
        int r = 0;
        for (int i = 0; i < kGlonassCodeLength; ++i) r += c[size_t(i)] * c[size_t((i + lag) % kGlonassCodeLength)];
        sidelobes = sidelobes && r == -1;
    }
    check(sidelobes, "code is a maximal sequence: every sidelobe is -1");
    check(!isGalileo(glonassId(0)) && isGlonass(glonassId(-7)) && isGalileo(136) && !isGlonass(136), "ids do not overlap");
    check(glonassChannel(glonassId(-7)) == -7 && glonassChannel(glonassId(6)) == 6, "channel round trip");
    check(glonassFreq(1) == 1602.5625e6, "channel frequency");
    check(satName(glonassId(-3)) == QLatin1String("R-3") && satName(glonassId(2)) == QLatin1String("R+2"), "names");
    std::printf("%d failure(s)\n", failures);
    return failures ? 1 : 0;
}
