// Command-line front end: runs the full receiver pipeline on a recording and
// prints the intermediate results. Useful for batch processing and testing.
//
//   gnss_cli <file.bin> [--ms N] [--channels N] [--fs Hz] [--if Hz] [--gnss all|gps|gal] [--csv out.csv]
//
// Live mode processes samples as they arrive, printing one line per update:
//
//   gnss_cli <file.bin> --live [--speed X] [--seconds N]       plays the recording in real time (or X times that)
//   gnss_cli --usrp [--args addr=192.168.10.2] [--rate 5e6] [--center 1575.42e6] [--gain 30] [--antenna RX2]
//            [--subdev A:0] [--clock internal] [--lo-offset Hz] [--uhd path/uhd.dll] [--record dir] [--seconds N]
//
// Further live options: --update-ms N, --window s, --acq-interval s

#include "core/Constants.h"
#include "core/Geodesy.h"
#include "core/LiveEngine.h"
#include "core/Receiver.h"

#include <QCoreApplication>
#include <QFile>
#include <QTextStream>
#include <QTimer>
#include <cmath>
#include <cstdio>

using namespace gnss;

static QTextStream& out()
{
    static QTextStream s(stdout);
    return s;
}

static void writeCsv(const QString& csv, const PvtOutput& p)
{
    if (csv.isEmpty()) return;
    QFile f(csv);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Text)) return;
    QTextStream ts(&f);
    ts << "epoch,gpsTow,nSat,lat,lon,h,x,y,z,bias,pdop,hdop,vdop,ve,vn,vu\n";
    for (const auto& s : p.sols)
        if (s.valid)
            ts << QString::asprintf("%d,%.4f,%d,%.9f,%.9f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                                    s.index, s.gpsTime, s.nUsed, s.llh.lat * 180 / kPi, s.llh.lon * 180 / kPi,
                                    s.llh.h, s.pos.x, s.pos.y, s.pos.z, s.bias, s.pdop, s.hdop, s.vdop,
                                    s.velEnu.x, s.velEnu.y, s.velEnu.z);
}

static int runLive(Receiver& rx, int seconds, const QString& csv)
{
    LiveEngine live;
    LiveStatus last;
    QObject::connect(&live, &LiveEngine::logMessage, [](const QString& t) { out() << t << "\n"; out().flush(); });
    QObject::connect(&live, &LiveEngine::snapshotReady, [&] {
        LiveSnapshot snap;
        if (!live.takeSnapshot(snap)) return;
        const LiveStatus st = snap.status;
        rx.applyLiveSnapshot(std::move(snap));
        last = st;
        const NavSolution* fix = nullptr;
        for (auto it = rx.pvt.sols.rbegin(); it != rx.pvt.sols.rend() && !fix; ++it)
            if (it->valid) fix = &*it;
        QString line = QString::asprintf("t=%6.1f ch=%2d usable=%2d lag=%6.1fms rate=%.3fMS/s ovf=%lld lost=%lld level=%.1f",
                                         st.streamSec, st.channels, st.usable, st.lagMs, st.measuredRate / 1e6,
                                         st.overflows, st.lostSamples, st.levelRms);
        if (fix)
            line += QString::asprintf(" | fix %.7f %.7f %.1f m sats=%d pdop=%.2f rms=%.1f tow=%.1f", fix->llh.lat * 180 / kPi,
                                      fix->llh.lon * 180 / kPi, fix->llh.h, fix->nUsed, fix->pdop, fix->rms, fix->gpsTime);
        out() << line << "\n";
        out().flush();
    });
    QObject::connect(&live, &LiveEngine::finished, QCoreApplication::instance(), &QCoreApplication::quit, Qt::QueuedConnection);
    QString err;
    if (!live.start(rx.settings, &rx.source, &err)) {
        out() << err << "\n";
        return 7;
    }
    if (seconds > 0) QTimer::singleShot(seconds * 1000, &live, [&live] { live.stop(); });
    QCoreApplication::exec();
    live.stop();
    LiveSnapshot snap;
    if (live.takeSnapshot(snap)) {
        last = snap.status;
        rx.applyLiveSnapshot(std::move(snap));
    }
    const auto& p = rx.pvt;
    out() << QString::asprintf("\nLive summary: %.1f s of signal, %d fixes kept, first fix at %.1f s, %d overflows, %lld samples lost, %d channels at the end\n",
                               last.streamSec, p.nValid, last.ttffSec, int(last.overflows), last.lostSamples, last.channels);
    if (p.nValid > 0) {
        out() << QString::asprintf("Mean: lat %.8f lon %.8f h %.2f  (ECEF %.2f %.2f %.2f)\n", p.meanLlh.lat * 180 / kPi,
                                   p.meanLlh.lon * 180 / kPi, p.meanLlh.h, p.meanPos.x, p.meanPos.y, p.meanPos.z);
        out() << QString::asprintf("std E/N/U %.2f %.2f %.2f m, CEP50 %.2f m, sats %.1f, PDOP %.2f, effective IF %.1f Hz\n", p.stdE, p.stdN, p.stdU,
                                   p.cep50, p.meanSats, p.meanPdop, p.effectiveIf);
    }
    writeCsv(csv, p);
    out().flush();
    return p.nValid > 0 ? 0 : 8;
}

int main(int argc, char** argv)
{
    QCoreApplication app(argc, argv);
    const QStringList args = app.arguments();
    if (args.size() < 2) {
        out() << "usage: gnss_cli <file.bin> [--ms N] [--channels N] [--fs Hz] [--if Hz] [--gnss all|gps|gal] [--csv out.csv]\n"
                 "       gnss_cli <file.bin> --live [--speed X] [--seconds N]\n"
                 "       gnss_cli --usrp [--args addr=192.168.10.2] [--rate 5e6] [--gain 30] [--seconds N] [--record dir]\n";
        return 1;
    }
    Receiver rx;
    QObject::connect(&rx, &Receiver::logMessage, [](const QString& t) { out() << t << "\n"; out().flush(); });
    const bool sm = args.contains("--sm");
    const bool usrp = args.contains("--usrp") || sm;
    QString err;
    if (!usrp && !rx.openFile(args[1], &err)) {
        out() << err << "\n";
        return 2;
    }
    QString csv;
    bool live = usrp;
    int seconds = 0;
    LiveSettings& ls = rx.settings.live;
    ls.source = sm ? LiveSourceKind::SignalHound : usrp ? LiveSourceKind::Usrp : LiveSourceKind::Replay;
    for (int i = 1; i < args.size(); ++i) {
        const bool more = i + 1 < args.size();
        if (args[i] == "--live") live = true;
        else if (!more) break;
        else if (args[i] == "--ms") rx.settings.msToProcess = args[++i].toInt();
        else if (args[i] == "--channels") rx.settings.numChannels = args[++i].toInt();
        else if (args[i] == "--fs") rx.settings.samplingFreq = args[++i].toDouble();
        else if (args[i] == "--if") rx.settings.ifFreq = args[++i].toDouble();
        else if (args[i] == "--gnss") {
            const QString g = args[++i];
            rx.settings.constellations = g == "gps" ? Constellations::GpsOnly
                                         : g == "gal" ? Constellations::GalileoOnly : g == "glo" ? Constellations::GlonassOnly : Constellations::GpsGalileo;
        }
        else if (args[i] == "--csv") csv = args[++i];
        else if (args[i] == "--speed") ls.replaySpeed = args[++i].toDouble();
        else if (args[i] == "--seconds") seconds = args[++i].toInt();
        else if (args[i] == "--update-ms") ls.updateMs = args[++i].toInt();
        else if (args[i] == "--window") ls.windowSeconds = args[++i].toInt();
        else if (args[i] == "--acq-interval") ls.acqIntervalSec = args[++i].toInt();
        else if (args[i] == "--args") ls.deviceArgs = args[++i];
        else if (args[i] == "--rate") ls.sampleRate = args[++i].toDouble();
        else if (args[i] == "--center") ls.centerFreq = args[++i].toDouble();
        else if (args[i] == "--gain") ls.gainDb = args[++i].toDouble();
        else if (args[i] == "--antenna") ls.antenna = args[++i];
        else if (args[i] == "--subdev") ls.subdev = args[++i];
        else if (args[i] == "--clock") ls.clockSource = args[++i];
        else if (args[i] == "--lo-offset") ls.loOffset = args[++i].toDouble();
        else if (args[i] == "--uhd") ls.uhdLibrary = args[++i];
        else if (args[i] == "--sm-dll") ls.smLibrary = args[++i];
        else if (args[i] == "--sm-center") ls.smCenter = args[++i].toDouble();
        else if (args[i] == "--ref-level") ls.smRefLevel = args[++i].toDouble();
        else if (args[i] == "--record") { ls.record = true; ls.recordDir = args[++i]; }
    }
    if (live) return runLive(rx, seconds, csv);

    if (!rx.runAcquisition()) return 3;
    out() << "\n PRN  metric   code(samp)  Doppler(Hz)  fine-coarse  C/N0\n";
    for (const auto& r : rx.acq.results)
        out() << QString::asprintf(" %3s  %6.2f  %10d  %11.1f  %11.1f  %5.1f %s\n",
                                   qPrintable(isGalileo(r.prn) ? satName(r.prn) : QString::number(r.prn)), r.metric, r.codePhase,
                                   r.doppler, r.fineDelta, r.cn0, r.acquired ? "ACQ" : "");
    out().flush();

    if (!rx.runTracking()) return 4;
    if (!rx.runNavigation()) return 5;
    for (const auto& n : rx.nav) {
        if (isGalileo(n.prn)) {
            out() << QString::asprintf(" %s words:", qPrintable(satName(n.prn)));
            for (size_t i = 0; i < n.inavWords.size() && i < 24; ++i)
                out() << QString::asprintf(" %d%s", n.inavWords[i].type, n.inavWords[i].crcOk ? "" : "*");
        } else {
            out() << QString::asprintf(" PRN %2d subframes:", n.prn);
            for (const auto& sf : n.subframes)
                out() << QString::asprintf(" %d%s(%d)", sf.id, sf.allParityOk() ? "" : "*", sf.towCount);
        }
        out() << "\n";
        if (n.eph.complete())
            out() << QString::asprintf("   WN %d toe %.0f toc %.0f IODE %d/%d IODC %d health %d URA %d e %.6f sqrtA %.3f\n",
                                       n.eph.weekNumber, n.eph.toe, n.eph.toc, n.eph.iode2, n.eph.iode3, n.eph.iodc,
                                       n.eph.svHealth, n.eph.uraIndex, n.eph.e, n.eph.sqrtA);
    }
    out().flush();
    if (!rx.runPvt()) return 6;

    const auto& p = rx.pvt;
    out() << "\nEpoch   rxTime        nSat   lat(deg)      lon(deg)      h(m)      E(m)    N(m)    U(m)   bias(m)      PDOP  vE     vN     vU\n";
    for (const auto& s : p.sols) {
        if (!s.valid) { out() << QString::asprintf("%4d  invalid: %s\n", s.index, qPrintable(s.message)); continue; }
        out() << QString::asprintf("%4d  %12.4f  %2d  %12.8f  %12.8f  %8.2f  %6.2f  %6.2f  %6.2f  %12.2f  %4.2f  %5.2f  %5.2f  %5.2f\n",
                                   s.index, s.gpsTime, s.nUsed, s.llh.lat * 180 / kPi, s.llh.lon * 180 / kPi, s.llh.h,
                                   s.enu.x, s.enu.y, s.enu.z, s.bias, s.pdop, s.velEnu.x, s.velEnu.y, s.velEnu.z);
    }
    out() << QString::asprintf("\nMean: lat %.8f lon %.8f h %.2f  (ECEF %.2f %.2f %.2f)\n", p.meanLlh.lat * 180 / kPi,
                               p.meanLlh.lon * 180 / kPi, p.meanLlh.h, p.meanPos.x, p.meanPos.y, p.meanPos.z);
    out() << QString::asprintf("UTM zone %d%c  E %.2f  N %.2f\n", p.utm.zone, p.utm.band.toLatin1(), p.utm.easting, p.utm.northing);
    out() << QString::asprintf("std E/N/U %.2f %.2f %.2f m, CEP50 %.2f m, 2DRMS %.2f m\n", p.stdE, p.stdN, p.stdU, p.cep50, p.drms2);
    out() << QString::asprintf("clock bias drift %.3f m/s, Doppler clock drift %.3f m/s, common freq offset %.1f Hz -> effective IF %.1f Hz\n",
                               p.biasDrift, p.meanDrift, p.commonFreqOffset, p.effectiveIf);
    if (p.weekNumber >= 0) {
        for (int k = 0; k < 3; ++k) {
            const QDateTime dt = geo::gpsToDateTime(p.weekNumber + 1024 * k, p.firstGpsTime);
            out() << QString::asprintf("If %d week rollovers: %s (GPS time)\n", k, qPrintable(dt.toString(Qt::ISODate)));
        }
    }
    writeCsv(csv, p);
    out().flush();
    return 0;
}
