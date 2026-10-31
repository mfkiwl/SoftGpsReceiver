#include "Receiver.h"
#include "Constants.h"
#include "LiveEngine.h"

#include <QElapsedTimer>
#include <QFileInfo>
#include <QThread>
#include <QtConcurrent/QtConcurrentMap>

#include <algorithm>
#include <numeric>

namespace gnss {

Receiver::Receiver(QObject* parent) : QObject(parent) {}

void Receiver::log(const QString& t)
{
    emit logMessage(t);
}

bool Receiver::openFile(const QString& path, QString* error)
{
    resetResults();
    liveDescription.clear();
    double fs = settings.samplingFreq, fif = settings.ifFreq;
    SampleFormat fmt = settings.format;
    const bool parsed = ReceiverSettings::parseFileName(path, &fs, &fif, &fmt);
    if (parsed) {
        settings.samplingFreq = fs;
        settings.ifFreq = fif;
        settings.format = fmt;
    }
    settings.fileName = path;
    if (!source.open(path, settings.format, settings.skipBytes, error)) return false;
    log(QStringLiteral("Opened %1 (%2 MB, %3 samples, %4 s)")
            .arg(QFileInfo(path).fileName())
            .arg(source.fileSize() / 1048576.0, 0, 'f', 1)
            .arg(source.numSamples())
            .arg(fileDurationSec(), 0, 'f', 3));
    if (parsed)
        log(QStringLiteral("From file name: fs = %1 MHz, IF = %2 MHz, format = %3")
                .arg(fs / 1e6, 0, 'f', 6).arg(fif / 1e6, 0, 'f', 6).arg(sampleFormatName(fmt)));
    else
        log(QStringLiteral("Sampling/IF frequency not found in the file name - check the settings"));
    return true;
}

bool Receiver::reopen(QString* error)
{
    resetResults();
    if (settings.fileName.isEmpty()) return false;
    return source.open(settings.fileName, settings.format, settings.skipBytes, error);
}

void Receiver::resetResults()
{
    acq = AcquisitionOutput();
    trk.clear();
    nav.clear();
    gnav = GlobalNavData();
    pvt = PvtOutput();
    stage_ = StageNone;
    for (double& t : stageTime_) t = 0;
}

void Receiver::closeFile()
{
    resetResults();
    source.close();
    liveDescription.clear();
    settings.fileName.clear();
}

void Receiver::applyLiveSnapshot(LiveSnapshot&& snap)
{
    settings.samplingFreq = snap.samplingFreq;
    settings.ifFreq = snap.ifFreq;
    settings.format = source.format();
    if (snap.hasAcq) acq = std::move(snap.acq);
    trk = std::move(snap.trk);
    nav = std::move(snap.nav);
    gnav = std::move(snap.gnav);
    pvt = std::move(snap.pvt);
    stage_ = snap.stage;
}

double Receiver::fileDurationSec() const
{
    return settings.samplingFreq > 0 ? double(source.numSamples()) / settings.samplingFreq : 0;
}

int Receiver::channelOfPrn(int prn) const
{
    for (size_t i = 0; i < trk.size(); ++i)
        if (trk[i].prn == prn) return int(i);
    return -1;
}

bool Receiver::runAcquisition()
{
    if (!source.isOpen()) { log(QStringLiteral("No file open")); return false; }
    clearCancel();
    QElapsedTimer t;
    t.start();
    if (settings.useGps())
        log(QStringLiteral("GPS acquisition: %1 PRNs, +/-%2 Hz in %3 Hz steps, %4 ms non-coherent")
                .arg(settings.prnList().size()).arg(settings.acqSearchBand).arg(settings.acqStep).arg(settings.acqNonCoherent));
    if (settings.useGalileo())
        log(QStringLiteral("Galileo acquisition: E1-B and E1-C of %1 satellites, %2 x 4 ms non-coherent")
                .arg(kGalileoMaxSv).arg(settings.galNonCoherent));
    emit progressChanged(QStringLiteral("Acquisition"), 0);
    acq = Acquisition::run(source, settings, &cancel_, [this](int p) { emit progressChanged(QStringLiteral("Acquisition"), p); });
    trk.clear(); nav.clear(); pvt = PvtOutput(); gnav = GlobalNavData();
    if (!acq.error.isEmpty()) {
        log(QStringLiteral("Acquisition failed: %1").arg(acq.error));
        stage_ = StageNone;
        return false;
    }
    QStringList found;
    for (const auto& r : acq.results)
        if (r.acquired) found << QStringLiteral("%1 (%2)").arg(isGalileo(r.prn) ? satName(r.prn) : QString::number(r.prn)).arg(r.metric, 0, 'f', 1);
    log(QStringLiteral("Acquisition finished in %1 s: %2 satellites acquired: %3")
            .arg(acq.elapsedSec, 0, 'f', 2).arg(found.size()).arg(found.join(", ")));
    stage_ = StageAcquired;
    stageTime_[StageAcquired] = t.elapsed() / 1000.0;
    emit stageCompleted(stage_);
    return true;
}

bool Receiver::runTracking()
{
    if (stage_ < StageAcquired && !runAcquisition()) return false;
    clearCancel();
    QElapsedTimer t;
    t.start();
    std::vector<const AcqResult*> sel;
    for (const auto& r : acq.results)
        if (r.acquired) sel.push_back(&r);
    // Strongest first, up to numChannels per constellation: the two metrics are not comparable
    std::sort(sel.begin(), sel.end(), [](const AcqResult* a, const AcqResult* b) {
        if (isGalileo(a->prn) != isGalileo(b->prn)) return isGalileo(b->prn);
        return a->metric > b->metric;
    });
    int nGps = 0, nGal = 0;
    sel.erase(std::remove_if(sel.begin(), sel.end(), [&](const AcqResult* r) {
        return ++(isGalileo(r->prn) ? nGal : nGps) > settings.numChannels;
    }), sel.end());
    if (sel.empty()) { log(QStringLiteral("No satellites to track")); return false; }

    trk.assign(sel.size(), TrackingChannel());
    nav.clear(); pvt = PvtOutput(); gnav = GlobalNavData();
    const double msTotalEst = std::max(1.0, fileDurationSec() * 1000.0 - settings.acqStartMs);
    const double msAll = (settings.msToProcess > 0 ? std::min<double>(settings.msToProcess, msTotalEst) : msTotalEst) * sel.size();
    log(QStringLiteral("Tracking %1 channels over %2 s of data (%3)")
            .arg(sel.size()).arg(msAll / sel.size() / 1000.0, 0, 'f', 1).arg(settings.summary()));

    std::atomic<qint64> counter{0};
    std::vector<int> idx(sel.size());
    std::iota(idx.begin(), idx.end(), 0);
    // progress reporter
    QFuture<void> fut = QtConcurrent::map(idx, [&](const int& i) {
        TrackingChannel& ch = trk[size_t(i)];
        ch.channel = i;
        ch.prn = sel[size_t(i)]->prn;
        const qint64 start = acq.startSample + sel[size_t(i)]->codePhase;
        Tracking::trackChannel(source, settings, *sel[size_t(i)], start, ch, &cancel_, &counter);
    });
    int lastPct = -1;
    while (!fut.isFinished()) {
        QThread::msleep(100);
        const int pct = int(std::min(100.0, 100.0 * double(counter.load()) / msAll));
        if (pct != lastPct) { emit progressChanged(QStringLiteral("Tracking"), pct); lastPct = pct; }
    }
    fut.waitForFinished();
    if (isCancelled()) { log(QStringLiteral("Tracking cancelled")); stage_ = StageAcquired; return false; }
    for (const auto& ch : trk)
        log(QStringLiteral("  Ch %1 %2: %3 ms, mean C/N0 %4 dB-Hz, phase lock %5 %, final Doppler %6 Hz, %7 s")
                .arg(ch.channel).arg(satName(ch.prn)).arg(ch.numMs * ch.periodMs).arg(ch.meanCn0, 0, 'f', 1)
                .arg(ch.lockFraction * 100, 0, 'f', 1).arg(ch.finalDoppler, 0, 'f', 1).arg(ch.elapsedSec, 0, 'f', 1));
    for (const auto& ch : trk)
        if (ch.sidePeakJumps > 0)
            log(QStringLiteral("  Ch %1 %2: moved off a BOC side peak %3 time(s)").arg(ch.channel).arg(satName(ch.prn)).arg(ch.sidePeakJumps));
    stage_ = StageTracked;
    stageTime_[StageTracked] = t.elapsed() / 1000.0;
    log(QStringLiteral("Tracking finished in %1 s").arg(stageTime_[StageTracked], 0, 'f', 1));
    emit stageCompleted(stage_);
    return true;
}

bool Receiver::runNavigation()
{
    if (stage_ < StageTracked && !runTracking()) return false;
    QElapsedTimer t;
    t.start();
    emit progressChanged(QStringLiteral("Navigation message"), 0);
    nav.assign(trk.size(), NavChannel());
    std::vector<int> idx(trk.size());
    std::iota(idx.begin(), idx.end(), 0);
    QtConcurrent::blockingMap(idx, [&](const int& i) { nav[size_t(i)] = nav::decodeChannel(trk[size_t(i)], settings); });
    nav::collectGlobal(nav, gnav);
    for (const auto& n : nav) {
        if (isGalileo(n.prn)) {
            log(QStringLiteral("  %1: page sync %2, symbol SNR %3 dB, %4 of %5 I/NAV words pass the CRC, %6 TOW errors, %7")
                    .arg(satName(n.prn)).arg(n.bitSyncOk ? "ok" : "FAILED").arg(n.bitSnrDb, 0, 'f', 1)
                    .arg(n.parityOkWords).arg(n.parityTotalWords).arg(n.inavTowErrors).arg(n.status));
            continue;
        }
        int ok = 0;
        for (const auto& sf : n.subframes) ok += sf.allParityOk() ? 1 : 0;
        log(QStringLiteral("  %1: bit sync %2 (offset %3 ms, %4 %), bit SNR %5 dB, %6 subframes (%7 fully parity-OK), words %8/%9, %10")
                .arg(satName(n.prn)).arg(n.bitSyncOk ? "ok" : "FAILED").arg(n.bitOffset)
                .arg(n.bitSyncConfidence * 100, 0, 'f', 0).arg(n.bitSnrDb, 0, 'f', 1)
                .arg(n.subframes.size()).arg(ok).arg(n.parityOkWords).arg(n.parityTotalWords).arg(n.status));
    }
    if (gnav.ionoUtc.valid) log(QStringLiteral("Ionospheric/UTC parameters decoded from PRN %1").arg(gnav.ionoUtc.sourcePrn));
    else log(QStringLiteral("Subframe 4 page 18 (iono/UTC) not in the recording - no ionospheric correction"));
    if (gnav.galileo.ggtoValid)
        log(QStringLiteral("Broadcast GPS-Galileo time offset: %1 ns").arg(gnav.galileo.a0g * 1e9, 0, 'f', 2));
    int nAlm = 0;
    for (int i = 1; i <= 32; ++i) nAlm += gnav.almanac[i].valid ? 1 : 0;
    log(QStringLiteral("Almanac entries decoded: %1").arg(nAlm));
    stage_ = StageDecoded;
    stageTime_[StageDecoded] = t.elapsed() / 1000.0;
    emit progressChanged(QStringLiteral("Navigation message"), 100);
    emit stageCompleted(stage_);
    return true;
}

bool Receiver::runPvt()
{
    if (stage_ < StageDecoded && !runNavigation()) return false;
    QElapsedTimer t;
    t.start();
    emit progressChanged(QStringLiteral("Position solution"), 0);
    pvt = pvt::solve(trk, nav, gnav, settings);
    for (const QString& l : pvt.log) log(QStringLiteral("  ") + l);
    if (!pvt.error.isEmpty()) {
        log(QStringLiteral("Position solution failed: %1").arg(pvt.error));
        stage_ = StageDecoded;
        emit progressChanged(QStringLiteral("Position solution"), 100);
        emit stageCompleted(stage_);
        return false;
    }
    if (pvt.nGps > 0 && pvt.nGalileo > 0)
        log(QStringLiteral("Combined fix: %1 GPS and %2 Galileo satellites, mean Galileo-GPS bias %3 m")
                .arg(pvt.nGps).arg(pvt.nGalileo).arg(pvt.meanIsb, 0, 'f', 2));
    log(QStringLiteral("Position: %1 valid fixes, lat %2 deg, lon %3 deg, h %4 m; std E/N/U = %5/%6/%7 m; mean PDOP %8")
            .arg(pvt.nValid)
            .arg(pvt.meanLlh.lat * 180.0 / kPi, 0, 'f', 7).arg(pvt.meanLlh.lon * 180.0 / kPi, 0, 'f', 7)
            .arg(pvt.meanLlh.h, 0, 'f', 2).arg(pvt.stdE, 0, 'f', 2).arg(pvt.stdN, 0, 'f', 2).arg(pvt.stdU, 0, 'f', 2)
            .arg(pvt.meanPdop, 0, 'f', 2));
    stage_ = StageSolved;
    stageTime_[StageSolved] = t.elapsed() / 1000.0;
    emit progressChanged(QStringLiteral("Position solution"), 100);
    emit stageCompleted(stage_);
    return true;
}

bool Receiver::runAll()
{
    stage_ = StageNone;
    return runAcquisition() && runTracking() && runNavigation() && runPvt();
}

} // namespace gnss
