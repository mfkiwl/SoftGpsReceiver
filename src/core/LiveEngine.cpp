#include "LiveEngine.h"
#include "Constants.h"
#include "Receiver.h"

#include <QDateTime>
#include <QDir>
#include <QElapsedTimer>
#include <QFileInfo>
#include <QThread>
#include <QtConcurrent/QtConcurrentMap>

#include <algorithm>
#include <cmath>

namespace gnss {

namespace {

constexpr double kWeakCn0 = 22.0;        // dB-Hz: below this a channel is not really tracking anything
constexpr double kLostAfterSec = 3.0;    // ... if it stays there, or out of phase lock, this long
constexpr double kUnverifiedSec = 25.0;  // tracking without data that passes its check is a false lock

} // namespace

struct LiveEngine::Channel
{
    int slot = 0;
    int prn = 0;
    qint64 startSample = 0;
    std::unique_ptr<TrackingChannel> data;
    std::unique_ptr<ChannelTracker> tracker;
    double weakSec = 0;
    bool outOfBuffer = false;
    bool remove = false;
};

LiveEngine::LiveEngine(QObject* parent) : QObject(parent) {}

LiveEngine::~LiveEngine()
{
    stop();
}

bool LiveEngine::start(const ReceiverSettings& settings, SignalSource* source, QString* error)
{
    if (running_.load()) return false;
    s_ = settings;
    src_ = source;
    stop_ = false;
    clearState();

    s_.live.windowSeconds = std::max(s_.live.windowSeconds, 36);    // SF1-3 of one frame must fit
    s_.msToProcess = 0;
    ring_.reset();
    if (s_.live.source == LiveSourceKind::Replay) {
        input_ = std::make_unique<ReplayInput>(source, s_);
    } else if (s_.live.source == LiveSourceKind::GpsSim) {
        auto tcp = std::make_unique<TcpInput>(s_.live);
        if (!tcp->open(error)) return false;
        const LiveInputInfo& info = tcp->info();
        s_.samplingFreq = info.sampleRate;
        s_.ifFreq = kL1Freq - info.centerFreq;
        s_.format = SampleFormat::Int16IQ;
        ring_ = std::make_shared<LiveRing>(qint64(info.sampleRate * std::max(2, s_.live.ringSeconds)));
        source->attachRing(ring_);
        tcp->setRing(ring_);
        input_ = std::move(tcp);
    } else if (s_.live.source == LiveSourceKind::SignalHound) {
        auto sm = std::make_unique<SmInput>(s_.live, s_.useGlonass());
        if (!sm->open(error)) return false;
        const LiveInputInfo& info = sm->info();
        s_.samplingFreq = info.sampleRate;
        s_.ifFreq = 0;                            // the splitter centres the band
        s_.format = SampleFormat::Int16IQ;
        ring_ = std::make_shared<LiveRing>(qint64(info.sampleRate * std::max(2, s_.live.ringSeconds)));
        source->attachRing(ring_);
        sm->setRing(ring_);
        log(info.details);
        input_ = std::move(sm);
    } else if (s_.live.source == LiveSourceKind::HackRf) {
        auto hackrf = std::make_unique<HackRfInput>(s_.live);
        if (!hackrf->open(error)) return false;
        const LiveInputInfo& info = hackrf->info();
        s_.samplingFreq = info.sampleRate;
        s_.ifFreq = kL1Freq - info.centerFreq;    // with an LO offset the band sits off centre
        s_.format = SampleFormat::Int16IQ;
        ring_ = std::make_shared<LiveRing>(qint64(info.sampleRate * std::max(2, s_.live.ringSeconds)));
        source->attachRing(ring_);
        hackrf->setRing(ring_);
        log(info.details);
        input_ = std::move(hackrf);
    } else {
        auto uhd = std::make_unique<UhdInput>(s_.live);
        if (!uhd->open(error)) return false;
        const LiveInputInfo& info = uhd->info();
        s_.samplingFreq = info.sampleRate;
        s_.ifFreq = kL1Freq - info.centerFreq;    // the band sits there in the complex baseband stream
        s_.format = SampleFormat::Int16IQ;
        ring_ = std::make_shared<LiveRing>(qint64(info.sampleRate * std::max(2, s_.live.ringSeconds)));
        source->attachRing(ring_);
        uhd->setRing(ring_);
        log(info.details);
        input_ = std::move(uhd);
    }
    if (s_.samplingFreq < 4e6 && s_.useGalileo()) {
        log(QStringLiteral("Galileo E1 needs about 4 MS/s or more: searching GPS only at %1 MS/s").arg(s_.samplingFreq / 1e6));
        s_.constellations = Constellations::GpsOnly;
    }
    log(QStringLiteral("Live: %1").arg(input_->info().description));

    if (ring_ && s_.live.record) {
        QDir dir(s_.live.recordDir.isEmpty() ? QDir::currentPath() : s_.live.recordDir);
        dir.mkpath(QStringLiteral("."));
        // The name carries what is needed to open the recording again later
        const QString fsText = QString::number(s_.samplingFreq / 1e6, 'f', 6).replace('.', '_');
        const QString ifText = QString::number(std::fabs(s_.ifFreq) / 1e6, 'f', 6).replace('.', '_');
        const QString name = QStringLiteral("usrp-%1-fs%2-if%3-int16iq.bin")
                                 .arg(QDateTime::currentDateTime().toString("yyyyMMdd-HHmmss"), fsText, ifText);
        recordFile_.setFileName(dir.filePath(name));
        if (recordFile_.open(QIODevice::WriteOnly)) log(QStringLiteral("Recording to %1").arg(recordFile_.fileName()));
        else log(QStringLiteral("Cannot record to %1: %2").arg(recordFile_.fileName(), recordFile_.errorString()));
    }

    if (!input_->start(error)) {
        input_.reset();
        return false;
    }
    running_ = true;
    pipelineThread_ = QThread::create([this] { pipeline(); });
    pipelineThread_->start(QThread::HighPriority);
    acqThread_ = QThread::create([this] { acquisitionLoop(); });
    acqThread_->start(QThread::LowPriority);
    if (recordFile_.isOpen()) {
        recordThread_ = QThread::create([this] { recorderLoop(); });
        recordThread_->start();
    }
    return true;
}

void LiveEngine::clearState()
{
    channels_.clear();
    trackedSats_.clear();
    newlyAcquired_.clear();
    acqBySat_.clear();
    gpsBins_.clear();
    acqCycles_ = 0;
    acqDirty_ = false;
    haveCommonDoppler_ = false;
    gnav_ = GlobalNavData();
    pvt_ = PvtOutput();
    solver_ = pvt::LiveSolver();
    ephCache_.clear();
    ttff_ = -1;
    lastHead_ = 0;
    lastTickNs_ = 0;
    rateClock_.start();
    measuredRate_ = 0;
    recordedBytes_ = 0;
    recordDropped_ = 0;
    QMutexLocker lock(&snapMutex_);
    latest_ = LiveSnapshot();
    haveLatest_ = false;
}

void LiveEngine::release()
{
    stop();
    clearState();
    ring_.reset();
    src_ = nullptr;
}

void LiveEngine::stop()
{
    stop_ = true;
    for (QThread** t : {&pipelineThread_, &acqThread_, &recordThread_}) {
        if (*t) {
            (*t)->wait();
            delete *t;
            *t = nullptr;
        }
    }
    if (input_) input_->stop();
    input_.reset();
    if (recordFile_.isOpen()) recordFile_.close();
    running_ = false;
}

bool LiveEngine::takeSnapshot(LiveSnapshot& out)
{
    QMutexLocker lock(&snapMutex_);
    if (!haveLatest_) return false;
    out = std::move(latest_);
    latest_ = LiveSnapshot();
    haveLatest_ = false;
    return true;
}

// ---------------------------------------------------------------------------
// Acquisition thread
// ---------------------------------------------------------------------------
void LiveEngine::acquisitionLoop()
{
    const qint64 need = qint64(0.2 * s_.samplingFreq);    // the search reads about 90 ms of the newest data
    QElapsedTimer sinceLast;
    bool first = true;
    while (!stop_.load() && !input_->finished()) {
        const qint64 head = src_->numSamples();
        const bool due = first || searchNow_.load() || sinceLast.elapsed() >= s_.live.acqIntervalSec * 1000;
        if (head < need + src_->oldestSample() || !due) {
            QThread::msleep(20);
            continue;
        }
        first = false;
        searchNow_ = false;
        sinceLast.start();
        // GPS first and on its own: it takes a fraction of a second, and tells the much
        // longer Galileo search where to look. Each starts from the newest samples.
        if (s_.useGps()) searchFor(false);
        if (s_.useGalileo() && !stop_.load()) searchFor(true);
        if (s_.useGlonass() && !stop_.load()) searchGlonass();
    }
}

void LiveEngine::searchGlonass()
{
    const double fs = s_.samplingFreq;
    ReceiverSettings a = s_;
    a.acqStartMs = int(double(src_->numSamples() - qint64(0.2 * fs)) / fs * 1000.0) + 1;
    AcquisitionOutput out = Acquisition::run(*src_, a, &stop_, nullptr);
    if (stop_.load() || !out.error.isEmpty()) return;
    const qint64 first = qint64(std::llround(a.acqStartMs * fs / 1000.0));
    QMutexLocker lock(&acqMutex_);
    for (AcqResult& r : out.results) {
        r.startSample = first;
        if (r.acquired && !trackedSats_.contains(r.prn)) newlyAcquired_.push_back(r);
        acqBySat_[r.prn] = r;
    }
    ++acqCycles_;
    acqDirty_ = true;
}

void LiveEngine::searchFor(bool galileo)
{
    const double fs = s_.samplingFreq;
    const qint64 head = src_->numSamples();
    ReceiverSettings a = s_;
    a.acqStartMs = int(double(head - qint64(0.2 * fs)) / fs * 1000.0) + 1;
    a.acqPrns.clear();
    a.acqGalSvids.clear();
    a.constellations = galileo ? Constellations::GalileoOnly : Constellations::GpsOnly;
    {
        QMutexLocker lock(&acqMutex_);
        int nGps = 0, nGal = 0;
        for (int id : trackedSats_) (isGalileo(id) ? nGal : nGps)++;
        if (galileo) {
            if (nGal >= s_.numChannels) return;
            for (int v = 1; v <= kGalileoMaxSv; ++v)
                if (!trackedSats_.contains(kGalileoIdBase + v)) a.acqGalSvids.append(v);
            if (a.acqGalSvids.isEmpty()) return;
        } else {
            if (nGps >= s_.numChannels) return;
            for (int p : s_.prnList())
                if (!trackedSats_.contains(p)) a.acqPrns.append(p);
            if (a.acqPrns.isEmpty()) return;
        }
        a.acqHasDopplerHint = haveCommonDoppler_;
        a.acqDopplerHint = commonDoppler_;
    }
    AcquisitionOutput out = Acquisition::run(*src_, a, &stop_, nullptr);
    if (stop_.load()) return;
    if (!out.error.isEmpty()) {
        log(QStringLiteral("Acquisition: %1").arg(out.error));
        return;
    }
    QMutexLocker lock(&acqMutex_);
    double sum = 0;
    int cnt = 0;
    for (AcqResult& r : out.results) {
        if (r.acquired) {
            r.startSample = out.startSample;
            newlyAcquired_.push_back(r);
            sum += r.doppler;
            ++cnt;
        }
        acqBySat_[r.prn] = std::move(r);
    }
    if (!galileo && cnt > 0) {
        commonDoppler_ = sum / cnt;
        haveCommonDoppler_ = true;
    }
    if (!out.binFreqs.empty()) {
        gpsBins_ = out.binFreqs;
        acqSamplesPerCode_ = out.samplesPerCode;
        acqNonCoherent_ = out.nonCoherent;
        acqFftSize_ = out.fftSize;
        acqFftInternal_ = out.fftInternalSize;
    }
    acqStartSample_ = out.startSample;
    acqElapsed_ = out.elapsedSec;
    if (acqCycles_ < 2)
        log(QStringLiteral("%1 search took %2 s").arg(galileo ? "Galileo" : "GPS").arg(out.elapsedSec, 0, 'f', 2));
    ++acqCycles_;
    acqDirty_ = true;
}

// ---------------------------------------------------------------------------
// Recorder thread: the sample ring is its queue, so a slow disk never holds up the radio
// ---------------------------------------------------------------------------
void LiveEngine::recorderLoop()
{
    const int chunk = 1 << 18;
    std::vector<int16_t> buf(size_t(chunk) * 2);
    qint64 pos = 0;
    while (true) {
        const qint64 head = ring_->head();
        const qint64 oldest = ring_->oldest();
        if (pos < oldest) {
            recordDropped_.fetch_add(oldest - pos);
            pos = oldest;
        }
        if (pos >= head) {
            if (stop_.load() || input_->finished()) break;
            QThread::msleep(10);
            continue;
        }
        const int n = ring_->readRaw(pos, int(std::min<qint64>(chunk, head - pos)), buf.data());
        if (n <= 0) continue;
        const qint64 bytes = qint64(n) * 4;
        if (recordFile_.write(reinterpret_cast<const char*>(buf.data()), bytes) != bytes) {
            log(QStringLiteral("Recording stopped: %1").arg(recordFile_.errorString()));
            return;
        }
        recordedBytes_.fetch_add(bytes);
        pos += n;
    }
}

// ---------------------------------------------------------------------------
// Pipeline thread
// ---------------------------------------------------------------------------
void LiveEngine::pipeline()
{
    QElapsedTimer tick;
    tick.start();
    qint64 lastPublish = 0;
    const qint64 period = std::max(50, s_.live.updateMs);
    QString reason;
    while (!stop_.load()) {
        const qint64 head = src_->numSamples();
        const bool ended = input_->finished();
        adoptAcquisitions();
        advanceChannels(head);
        if (ended) {
            const QString why = input_->failure();
            reason = why.isEmpty() ? QStringLiteral("End of recording") : why;
            break;
        }
        if (tick.elapsed() - lastPublish >= period) {
            lastPublish = tick.elapsed();
            publish(head, false);
        } else {
            QThread::msleep(5);
        }
    }
    publish(src_->numSamples(), true);
    if (reason.isEmpty()) reason = QStringLiteral("Stopped");
    log(QStringLiteral("Live finished: %1").arg(reason));
    running_ = false;
    emit finished(reason);
}

void LiveEngine::adoptAcquisitions()
{
    std::vector<AcqResult> found;
    {
        QMutexLocker lock(&acqMutex_);
        if (newlyAcquired_.empty()) return;
        found.swap(newlyAcquired_);
    }
    const bool cpx = src_->isComplex();
    for (const AcqResult& r : found) {
        const bool already = std::any_of(channels_.begin(), channels_.end(),
                                         [&r](const std::unique_ptr<Channel>& c) { return c->prn == r.prn; });
        if (already) continue;
        int used = 0;
        for (const auto& c : channels_) used += isGalileo(c->prn) == isGalileo(r.prn) && isGlonass(c->prn) == isGlonass(r.prn) ? 1 : 0;
        if (used >= s_.numChannels) continue;
        const qint64 startSample = r.startSample + r.codePhase;
        if (startSample < src_->oldestSample()) {                   // the data scrolled out while searching
            log(QStringLiteral("%1 was found too late to follow up: the search took longer than the %2 s sample buffer")
                    .arg(satName(r.prn)).arg(s_.live.ringSeconds));
            continue;
        }

        std::vector<char> taken(channels_.size() + 1, 0);
        for (const auto& c : channels_)
            if (size_t(c->slot) < taken.size()) taken[size_t(c->slot)] = 1;
        auto ch = std::make_unique<Channel>();
        ch->slot = int(std::find(taken.begin(), taken.end(), 0) - taken.begin());
        ch->prn = r.prn;
        ch->startSample = startSample;
        ch->data = std::make_unique<TrackingChannel>();
        ch->data->channel = ch->slot;
        ch->tracker = std::make_unique<ChannelTracker>(s_, r, startSample, cpx, *ch->data);
        log(QStringLiteral("Channel %1: tracking %2, Doppler %3 Hz, metric %4")
                .arg(ch->slot).arg(satName(r.prn)).arg(r.doppler, 0, 'f', 1).arg(r.metric, 0, 'f', 1));
        channels_.push_back(std::move(ch));
    }
    std::sort(channels_.begin(), channels_.end(),
              [](const std::unique_ptr<Channel>& a, const std::unique_ptr<Channel>& b) { return a->slot < b->slot; });
    QMutexLocker lock(&acqMutex_);
    trackedSats_.clear();
    for (const auto& c : channels_) trackedSats_.append(c->prn);
}

void LiveEngine::advanceChannels(qint64 head)
{
    if (channels_.empty()) return;
    std::vector<Channel*> list;
    list.reserve(channels_.size());
    for (const auto& c : channels_) list.push_back(c.get());
    const qint64 oldest = src_->oldestSample();
    QtConcurrent::blockingMap(&pool_, list, [this, head, oldest](Channel* const& c) {
        if (c->tracker->nextSample() < oldest) {
            c->outOfBuffer = true;                 // processing fell too far behind the radio
            return;
        }
        while (!stop_.load(std::memory_order_relaxed) && c->tracker->step(*src_, head)) {}
    });
}

// Drops channels that no longer follow a signal, and limits the history of the others
void LiveEngine::updateChannels()
{
    const double dt = std::max(50, s_.live.updateMs) * 1e-3;
    for (auto& cp : channels_) {
        Channel& c = *cp;
        TrackingChannel& t = *c.data;
        const int n = int(t.IP.size());
        const int perSec = 1000 / t.periodMs;
        if (n > 2 * perSec) {
            double cn0 = 0, pli = 0;
            for (int i = n - perSec; i < n; ++i) { cn0 += t.cn0[size_t(i)]; pli += t.pli[size_t(i)]; }
            const bool weak = cn0 / perSec < kWeakCn0 || pli / perSec < pvt::kLivePhaseLock;
            c.weakSec = weak ? c.weakSec + dt : 0.0;
        }
        if (c.weakSec >= kLostAfterSec) {
            log(QStringLiteral("Channel %1: lost %2").arg(c.slot).arg(satName(c.prn)));
            c.remove = true;
        }
        if (c.outOfBuffer) {
            log(QStringLiteral("Channel %1: %2 fell behind the sample buffer").arg(c.slot).arg(satName(c.prn)));
            c.remove = true;
        }
        const int keep = s_.live.windowSeconds * perSec;
        if (n > keep + 10 * perSec) t.trimFront(n - keep);
    }
    const auto gone = std::remove_if(channels_.begin(), channels_.end(), [](const std::unique_ptr<Channel>& c) { return c->remove; });
    if (gone != channels_.end()) {
        channels_.erase(gone, channels_.end());
        searchNow_ = true;                     // it may be back already, or be replaced by another satellite
        QMutexLocker lock(&acqMutex_);
        trackedSats_.clear();
        for (const auto& c : channels_) trackedSats_.append(c->prn);
    }
}

void LiveEngine::measureLevel(qint64 head, LiveStatus& st)
{
    const int n = int(std::min<qint64>(head - src_->oldestSample(), qint64(s_.samplingFreq * 0.02)));
    if (n <= 0) return;
    std::vector<float> a(static_cast<size_t>(n)), b(static_cast<size_t>(n));
    const int got = src_->read(head - n, n, a.data(), b.data());
    if (got <= 0) return;
    // Without the DC offset, which says nothing about the signal: a HackRF's alone looks like a sound level
    double meanA = 0, meanB = 0;
    for (int i = 0; i < got; ++i) {
        meanA += a[size_t(i)];
        meanB += b[size_t(i)];
    }
    meanA /= got;
    meanB /= got;
    double p = 0;
    int clip = 0;
    for (int i = 0; i < got; ++i) {
        const double x = a[size_t(i)] - meanA, y = b[size_t(i)] - meanB;
        p += x * x + y * y;
        if (std::fabs(a[size_t(i)]) >= 32000.0f || std::fabs(b[size_t(i)]) >= 32000.0f) ++clip;
    }
    st.levelRms = std::sqrt(p / got);
    st.levelDbfs = st.levelRms > 0 ? 20.0 * std::log10(st.levelRms / 32768.0) : -120.0;
    // Satellites sit below the noise, so it has to span a few steps of the converter: about 4 per rail
    st.lowDbfs = input_ ? 20.0 * std::log10(4.0 * std::sqrt(2.0) / double(1 << (input_->info().bits - 1))) : -65.0;
    st.clipPercent = 100.0 * clip / got;
}

void LiveEngine::publish(qint64 head, bool last)
{
    updateChannels();

    // ---- a copy of the tracking window, on which everything below works
    LiveSnapshot snap;
    snap.samplingFreq = s_.samplingFreq;
    snap.ifFreq = s_.ifFreq;
    snap.trk.reserve(channels_.size());
    for (const auto& c : channels_) {
        snap.trk.push_back(*c->data);
        snap.trk.back().summarise(s_);
        snap.trk.back().status = QStringLiteral("Tracking");
    }

    // ---- navigation data
    std::vector<int> idx(snap.trk.size());
    for (size_t i = 0; i < idx.size(); ++i) idx[i] = int(i);
    snap.nav.assign(snap.trk.size(), NavChannel());
    QtConcurrent::blockingMap(&pool_, idx, [&](const int& i) { snap.nav[size_t(i)] = nav::decodeChannel(snap.trk[size_t(i)], s_); });
    GlobalNavData fresh;
    nav::collectGlobal(snap.nav, fresh);
    nav::mergeGlobal(fresh, gnav_);

    snap.channels.resize(snap.trk.size());
    double commonSum = 0;
    int commonCnt = 0;
    for (size_t i = 0; i < snap.trk.size(); ++i) {
        const TrackingChannel& t = snap.trk[i];
        NavChannel& n = snap.nav[i];
        LiveChannelStatus& cs = snap.channels[i];
        cs.channel = t.channel;
        cs.prn = t.prn;
        cs.ageSec = double(channels_[i]->tracker->nextSample() - channels_[i]->startSample) / s_.samplingFreq;
        if (t.numMs > 0) {
            cs.cn0 = t.cn0.back();
            cs.doppler = t.carrFreq.back() - s_.carrierIf(t.prn);
            cs.pli = t.pli.back();
        }
        // A satellite picked up again does not have to wait a whole frame for its orbit
        if (n.eph.complete() && n.eph.consistent()) {
            ephCache_[t.prn] = n.eph;
        } else if (n.timingValid) {
            const auto it = ephCache_.find(t.prn);
            if (it != ephCache_.end()) {
                n.eph = it->second;
                cs.cachedEphemeris = true;
            }
        }
        cs.ephemeris = n.eph.complete();
        cs.subframes = n.parityOkWords / (isGalileo(t.prn) || isGlonass(t.prn) ? 1 : 10);
        if (cs.ageSec < 1.5) cs.state = QStringLiteral("Pull-in");
        else if (!n.bitSyncOk) cs.state = QStringLiteral("Bit sync");
        else if (!n.timingValid) cs.state = QStringLiteral("Finding frame");
        else if (!n.eph.complete()) cs.state = QStringLiteral("Collecting orbit");
        else cs.state = QStringLiteral("Ready");

        // Decoded data passing its check is what proves a channel follows the right signal
        const double lastOk = n.verifiedMs.empty() ? -1.0 : n.verifiedMs.back();
        const double unverified = (t.numMs - 1 - lastOk) * t.periodMs * 1e-3;
        if (cs.ageSec > kUnverifiedSec && unverified > kUnverifiedSec) {
            log(QStringLiteral("Channel %1: dropping %2, no verified data for %3 s").arg(t.channel).arg(satName(t.prn)).arg(unverified, 0, 'f', 0));
            channels_[i]->remove = true;
        }
        if (!isGalileo(t.prn) && !isGlonass(t.prn) && t.numMs > 0 && n.bitSyncOk) {
            commonSum += cs.doppler;
            ++commonCnt;
        }
    }

    // ---- position
    const double nav0 = std::max(1, s_.navSolPeriodMs) * 1e-3;
    solver_.update(snap.trk, snap.nav, gnav_, s_, int(s_.live.historySeconds / nav0), pvt_);
    if (ttff_ < 0 && pvt_.nValid > 0) {
        ttff_ = double(head) / s_.samplingFreq;
        log(QStringLiteral("First fix after %1 s of signal").arg(ttff_, 0, 'f', 1));
    }

    // ---- status
    LiveStatus& st = snap.status;
    st.running = !last;
    st.source = input_->info().description;
    st.sampleRate = s_.samplingFreq;
    st.centerFreq = input_->info().centerFreq;
    st.gainDb = input_->info().gainDb;
    st.samples = head;
    st.streamSec = double(head) / s_.samplingFreq;
    st.overflows = input_->overflows();
    st.lostSamples = input_->lostSamples();
    qint64 slowest = head;
    for (const auto& c : channels_) slowest = std::min(slowest, c->tracker->nextSample());
    st.lagMs = double(head - slowest) / s_.samplingFreq * 1000.0;
    st.bufferUsed = ring_ ? double(head - slowest) / double(ring_->capacity()) : 0.0;
    measureLevel(head, st);
    if (!ring_) st.levelDbfs = 0;                  // a recording has no absolute scale
    st.channels = int(channels_.size());
    st.usable = int(pvt_.usableChannels.size());
    st.haveFix = pvt_.nValid > 0;
    st.ttffSec = ttff_;
    st.recordFile = recordFile_.isOpen() ? recordFile_.fileName() : QString();
    st.recordedBytes = recordedBytes_.load();

    const qint64 t = rateClock_.nsecsElapsed();
    if (lastTickNs_ > 0 && t > lastTickNs_) {
        const double rate = double(head - lastHead_) / (double(t - lastTickNs_) * 1e-9);
        measuredRate_ = measuredRate_ > 0 ? 0.8 * measuredRate_ + 0.2 * rate : rate;
    }
    lastTickNs_ = t;
    lastHead_ = head;
    st.measuredRate = measuredRate_;

    // ---- the Doppler all satellites share gives the Galileo search a starting point
    {
        QMutexLocker lock(&acqMutex_);
        if (commonCnt > 0) {
            commonDoppler_ = commonSum / commonCnt;
            haveCommonDoppler_ = true;
        }
        st.acqCycles = acqCycles_;
        st.lastAcqSec = acqElapsed_;
        if (acqDirty_) {
            acqDirty_ = false;
            snap.hasAcq = true;
            snap.acq.binFreqs = gpsBins_;
            snap.acq.samplesPerCode = acqSamplesPerCode_;
            snap.acq.nonCoherent = acqNonCoherent_;
            snap.acq.fftSize = acqFftSize_;
            snap.acq.fftInternalSize = acqFftInternal_;
            snap.acq.startSample = acqStartSample_;
            snap.acq.elapsedSec = acqElapsed_;
            snap.acq.results.reserve(acqBySat_.size());
            for (const auto& kv : acqBySat_) snap.acq.results.push_back(kv.second);
        }
    }

    snap.gnav = gnav_;
    snap.pvt = pvt_;
    snap.stage = pvt_.nValid > 0 ? Receiver::StageSolved
                 : !snap.nav.empty() ? Receiver::StageDecoded
                 : st.acqCycles > 0 ? Receiver::StageAcquired : Receiver::StageNone;
    {
        QMutexLocker lock(&snapMutex_);
        const bool keepAcq = haveLatest_ && latest_.hasAcq && !snap.hasAcq;
        if (keepAcq) {                  // the display has not taken the previous one yet
            snap.acq = std::move(latest_.acq);
            snap.hasAcq = true;
        }
        latest_ = std::move(snap);
        haveLatest_ = true;
    }
    emit snapshotReady();
}

} // namespace gnss
