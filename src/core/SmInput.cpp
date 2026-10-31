#include "BandSplitter.h"
#include "LiveInput.h"
#include "Constants.h"

#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include <QLibrary>
#include <QThread>

#include <cmath>
#include <cstdint>
#include <vector>

namespace gnss {

namespace {

// The part of sm_api.h used here. At the native 50 MS/s the usable bandwidth is 41.5 MHz (SM API manual).
constexpr int kSmNoError = 0;
constexpr int kSmModeIQ = 3, kSmRateNative = 0, kSmData32fc = 0, kSmTrue = 1, kSmFalse = 0;
constexpr double kInputRate = 50e6, kUsableBandwidth = 41.5e6;
constexpr int kDecimation = 5;
constexpr double kBandCutoff = 4.5e6;
constexpr int kBlockSamples = 1 << 16;

} // namespace

struct SmInput::Api
{
    QLibrary lib;
    int (*open)(int*) = nullptr;
    int (*close)(int) = nullptr;
    int (*setRefLevel)(int, double) = nullptr;
    int (*setCenter)(int, double) = nullptr;
    int (*setBaseRate)(int, int) = nullptr;
    int (*setDecimation)(int, int) = nullptr;
    int (*setBandwidth)(int, int, double) = nullptr;
    int (*setDataType)(int, int) = nullptr;
    int (*configure)(int, int) = nullptr;
    int (*getParameters)(int, double*, double*) = nullptr;
    int (*getIQ)(int, void*, int, double*, int, int64_t*, int, int*, int*) = nullptr;
    const char* (*errorString)(int) = nullptr;
};

double smBandOffset(double centerFreq, bool glonass)
{
    return (glonass ? kGlonassL1Base : kL1Freq) - centerFreq;
}

SmInput::SmInput(const LiveSettings& s, bool glonass) : cfg_(s), glonass_(glonass), api_(new Api) {}

SmInput::~SmInput()
{
    stop();
    if (handle_ >= 0 && api_->close) api_->close(handle_);
}

bool SmInput::open(QString* error)
{
    Api& a = *api_;
    QStringList candidates;
    if (!cfg_.smLibrary.isEmpty()) candidates << cfg_.smLibrary;
    candidates << QDir(QCoreApplication::applicationDirPath()).filePath(QStringLiteral("sm_api.dll")) << QStringLiteral("sm_api");
    QString tried;
    bool loaded = false;
    for (const QString& c : candidates) {
        const QFileInfo fi(c);
        if (fi.isAbsolute() && !fi.exists()) continue;
        if (fi.isAbsolute())                                 // its own dependencies sit next to it
            qputenv("PATH", QDir::toNativeSeparators(fi.absolutePath()).toLocal8Bit() + ';' + qgetenv("PATH"));
        a.lib.setFileName(c);
        if ((loaded = a.lib.load())) break;
        tried += QStringLiteral("\n  %1: %2").arg(c, a.lib.errorString());
    }
    if (!loaded) {
        if (error) *error = QStringLiteral("sm_api.dll was not found. Set its path in Settings > Live; it is in the Signal Hound SDK under device_apis/sm_series/lib.%1").arg(tried);
        return false;
    }
    QStringList missing;
    const auto get = [&](auto& fn, const char* name) {
        fn = reinterpret_cast<std::remove_reference_t<decltype(fn)>>(a.lib.resolve(name));
        if (!fn) missing << QLatin1String(name);
    };
    get(a.open, "smOpenDevice");
    get(a.close, "smCloseDevice");
    get(a.setRefLevel, "smSetRefLevel");
    get(a.setCenter, "smSetIQCenterFreq");
    get(a.setBaseRate, "smSetIQBaseSampleRate");
    get(a.setDecimation, "smSetIQSampleRate");
    get(a.setBandwidth, "smSetIQBandwidth");
    get(a.setDataType, "smSetIQDataType");
    get(a.configure, "smConfigure");
    get(a.getParameters, "smGetIQParameters");
    get(a.getIQ, "smGetIQ");
    get(a.errorString, "smGetErrorString");
    if (!missing.isEmpty()) {
        if (error) *error = QStringLiteral("%1 lacks %2").arg(a.lib.fileName(), missing.join(", "));
        return false;
    }
    const auto ok = [&](int status, const char* what) {
        if (status >= kSmNoError) return true;               // positive values are warnings
        if (error) *error = QStringLiteral("%1: %2").arg(QLatin1String(what), QString::fromLocal8Bit(a.errorString(status)));
        return false;
    };

    // Everything wanted has to lie inside the usable bandwidth
    const double offset = smBandOffset(cfg_.smCenter, glonass_);
    // Refused only if a carrier is lost: a channel at the edge loses some of its signal but can still be tried.
    const double top = glonass_ ? glonassFreq(kGlonassMaxChannel) - kGlonassL1Base : 0.0;
    const double bottom = glonass_ ? glonassFreq(kGlonassMinChannel) - kGlonassL1Base : 0.0;
    const double signalHalf = glonass_ ? kGlonassChipRate : 2.0e6;
    if (offset + top > kUsableBandwidth / 2 || offset + bottom < -kUsableBandwidth / 2) {
        if (error) *error = QStringLiteral("With the centre at %1 MHz the band falls outside the %2 MHz usable at 50 MS/s")
                                .arg(cfg_.smCenter / 1e6, 0, 'f', 3).arg(kUsableBandwidth / 1e6, 0, 'f', 1);
        return false;
    }
    const bool clipped = offset + top + signalHalf > kUsableBandwidth / 2 || offset + bottom - signalHalf < -kUsableBandwidth / 2;

    if (!ok(a.open(&handle_), "Opening the SM200")) { handle_ = -1; return false; }
    if (!ok(a.setRefLevel(handle_, cfg_.smRefLevel), "Reference level")) return false;
    if (!ok(a.setCenter(handle_, cfg_.smCenter), "Centre frequency")) return false;
    if (!ok(a.setBaseRate(handle_, kSmRateNative), "Base sample rate")) return false;
    if (!ok(a.setDecimation(handle_, 1), "Decimation")) return false;
    if (!ok(a.setBandwidth(handle_, kSmFalse, kUsableBandwidth), "Bandwidth")) return false;
    if (!ok(a.setDataType(handle_, kSmData32fc), "Data type")) return false;
    if (!ok(a.configure(handle_, kSmModeIQ), "Starting I/Q")) return false;
    double rate = 0, bandwidth = 0;
    if (!ok(a.getParameters(handle_, &rate, &bandwidth), "Reading the stream settings")) return false;
    if (std::fabs(rate - kInputRate) > 1.0) {
        if (error) *error = QStringLiteral("The SM200 reports %1 MS/s, not 50").arg(rate / 1e6);
        return false;
    }
    info_.sampleRate = rate / kDecimation;
    info_.centerFreq = cfg_.smCenter + offset;
    info_.gainDb = 0;
    info_.description = QStringLiteral("SM200 at %1 MHz, %2 band, %3 MS/s").arg(cfg_.smCenter / 1e6, 0, 'f', 3)
                            .arg(glonass_ ? QStringLiteral("GLONASS") : QStringLiteral("GPS/Galileo")).arg(info_.sampleRate / 1e6, 0, 'f', 1);
    info_.details = QStringLiteral("SM200: %1 MS/s, %2 MHz bandwidth, reference level %3 dBm")
                        .arg(rate / 1e6).arg(bandwidth / 1e6).arg(cfg_.smRefLevel);
    if (clipped) info_.details += QStringLiteral(". Warning: the outermost channels reach past the usable bandwidth at this centre");
    return true;
}

bool SmInput::start(QString*)
{
    stop_ = false;
    thread_ = QThread::create([this] { run(); });
    thread_->start(QThread::TimeCriticalPriority);
    return true;
}

void SmInput::stop()
{
    stop_ = true;
    if (thread_) {
        thread_->wait();
        delete thread_;
        thread_ = nullptr;
    }
}

void SmInput::run()
{
    Api& a = *api_;
    BandSplitter band(kInputRate, kDecimation, smBandOffset(cfg_.smCenter, glonass_), kBandCutoff);
    // Samples are in square root of milliwatts, so 1.0 is 0 dBm: the reference level becomes full scale
    const float scale = float(32767.0 / std::pow(10.0, cfg_.smRefLevel / 20.0));
    std::vector<float> iq(size_t(2 * kBlockSamples));
    std::vector<int16_t> out(size_t(2 * (kBlockSamples / kDecimation + 2)));
    qint64 lastNs = 0;
    while (!stop_.load()) {
        int64_t ns = 0;
        int loss = 0;
        const int status = a.getIQ(handle_, iq.data(), kBlockSamples, nullptr, 0, &ns, kSmFalse, &loss, nullptr);
        if (status < kSmNoError) {
            fail(QStringLiteral("SM200: %1").arg(QString::fromLocal8Bit(a.errorString(status))));
            return;
        }
        if (loss) {
            // Keep later samples at the right time, as with a USRP: the time stamp says how much went missing
            ++overflows_;
            const qint64 expected = lastNs + qint64(kBlockSamples * 1e9 / kInputRate);
            const qint64 gap = lastNs ? qint64(double(ns - expected) * 1e-9 * info_.sampleRate) : 0;
            if (gap > 0) {
                ring_->writeZeros(gap);
                lost_ += gap;
            }
        }
        lastNs = ns;
        ring_->write(out.data(), band.process(iq.data(), kBlockSamples, scale, out.data()));
    }
}

} // namespace gnss
