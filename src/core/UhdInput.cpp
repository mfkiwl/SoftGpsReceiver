#include "LiveInput.h"

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

// ---------------------------------------------------------------------------
// The part of the UHD C API used here, from uhd/usrp/usrp.h, uhd/types/tune_request.h, tune_result.h, metadata.h and
// uhd/usrp/subdev_spec.h. These layouts are the same from UHD 3.15 up to the current 4.x releases, which is what
// allows a program to use the driver without being built against it.
// ---------------------------------------------------------------------------
using uhd_error = int;
constexpr uhd_error kUhdNoError = 0;

using UsrpHandle = void*;
using StreamerHandle = void*;
using MetadataHandle = void*;
using SubdevHandle = void*;

enum { kStreamStart = 97, kStreamStop = 111 };                 // uhd_stream_mode_t
enum { kPolicyAuto = 65, kPolicyManual = 77 };                 // uhd_tune_request_policy_t
enum {                                                          // uhd_rx_metadata_error_code_t
    kMdNone = 0x0, kMdTimeout = 0x1, kMdOverflow = 0x8
};

struct StreamArgs
{
    char* cpu_format;
    char* otw_format;
    char* args;
    size_t* channel_list;
    int n_channels;
};

struct StreamCmd
{
    int stream_mode;
    size_t num_samps;
    bool stream_now;
    int64_t time_spec_full_secs;
    double time_spec_frac_secs;
};

struct TuneRequest
{
    double target_freq;
    int rf_freq_policy;
    double rf_freq;
    int dsp_freq_policy;
    double dsp_freq;
    char* args;
};

struct TuneResult
{
    double clipped_rf_freq;
    double target_rf_freq;
    double actual_rf_freq;
    double target_dsp_freq;
    double actual_dsp_freq;
};

} // namespace

struct UhdInput::Api
{
    QLibrary lib;
    uhd_error (*usrp_make)(UsrpHandle*, const char*) = nullptr;
    uhd_error (*usrp_free)(UsrpHandle*) = nullptr;
    uhd_error (*get_last_error)(char*, size_t) = nullptr;
    uhd_error (*usrp_last_error)(UsrpHandle, char*, size_t) = nullptr;
    uhd_error (*rx_streamer_make)(StreamerHandle*) = nullptr;
    uhd_error (*rx_streamer_free)(StreamerHandle*) = nullptr;
    uhd_error (*rx_metadata_make)(MetadataHandle*) = nullptr;
    uhd_error (*rx_metadata_free)(MetadataHandle*) = nullptr;
    uhd_error (*get_rx_stream)(UsrpHandle, StreamArgs*, StreamerHandle) = nullptr;
    uhd_error (*max_num_samps)(StreamerHandle, size_t*) = nullptr;
    uhd_error (*rx_recv)(StreamerHandle, void**, size_t, MetadataHandle*, double, bool, size_t*) = nullptr;
    uhd_error (*issue_stream_cmd)(StreamerHandle, const StreamCmd*) = nullptr;
    uhd_error (*set_rx_rate)(UsrpHandle, double, size_t) = nullptr;
    uhd_error (*get_rx_rate)(UsrpHandle, size_t, double*) = nullptr;
    uhd_error (*set_rx_freq)(UsrpHandle, TuneRequest*, size_t, TuneResult*) = nullptr;
    uhd_error (*get_rx_freq)(UsrpHandle, size_t, double*) = nullptr;
    uhd_error (*set_rx_gain)(UsrpHandle, double, size_t, const char*) = nullptr;
    uhd_error (*get_rx_gain)(UsrpHandle, size_t, const char*, double*) = nullptr;
    uhd_error (*set_rx_antenna)(UsrpHandle, const char*, size_t) = nullptr;
    uhd_error (*set_rx_bandwidth)(UsrpHandle, double, size_t) = nullptr;
    uhd_error (*set_clock_source)(UsrpHandle, const char*, size_t) = nullptr;
    uhd_error (*set_time_now)(UsrpHandle, int64_t, double, size_t) = nullptr;
    uhd_error (*get_pp_string)(UsrpHandle, char*, size_t) = nullptr;
    uhd_error (*set_rx_subdev_spec)(UsrpHandle, SubdevHandle, size_t) = nullptr;
    uhd_error (*subdev_spec_make)(SubdevHandle*, const char*) = nullptr;
    uhd_error (*subdev_spec_free)(SubdevHandle*) = nullptr;
    uhd_error (*md_error_code)(MetadataHandle, int*) = nullptr;
    uhd_error (*md_has_time_spec)(MetadataHandle, bool*) = nullptr;
    uhd_error (*md_time_spec)(MetadataHandle, int64_t*, double*) = nullptr;
    uhd_error (*get_version_string)(char*, size_t) = nullptr;   // optional
};

UhdInput::UhdInput(const LiveSettings& s) : cfg_(s), api_(new Api) {}

UhdInput::~UhdInput()
{
    stop();
    closeDevice();
}

QString UhdInput::lastError(int code) const
{
    char text[1024] = {0};
    if (usrp_ && api_->usrp_last_error) api_->usrp_last_error(usrp_, text, sizeof(text));
    if (!text[0] && api_->get_last_error) api_->get_last_error(text, sizeof(text));
    const QString s = QString::fromLocal8Bit(text).trimmed();
    return s.isEmpty() ? QStringLiteral("UHD error %1").arg(code) : s;
}

static bool loadLibrary(QLibrary& lib, const QString& configured, QString* tried)
{
    QStringList candidates;
    if (!configured.isEmpty()) candidates << configured;
    const QByteArray pkg = qgetenv("UHD_PKG_PATH");
    if (!pkg.isEmpty()) candidates << QDir(QString::fromLocal8Bit(pkg)).filePath(QStringLiteral("bin/uhd.dll"));
#ifdef Q_OS_WIN
    candidates << QStringLiteral("C:/Program Files/UHD/bin/uhd.dll") << QStringLiteral("C:/Program Files (x86)/UHD/bin/uhd.dll");
#endif
    candidates << QStringLiteral("uhd");     // the system search path: PATH, or libuhd.so on Linux
    for (const QString& c : candidates) {
        const QFileInfo fi(c);
        if (fi.isAbsolute() && !fi.exists()) continue;
#ifdef Q_OS_WIN
        // uhd.dll's own dependencies sit next to it
        if (fi.isAbsolute())
            qputenv("PATH", QDir::toNativeSeparators(fi.absolutePath()).toLocal8Bit() + ';' + qgetenv("PATH"));
#endif
        lib.setFileName(c);
        if (lib.load()) return true;
        tried->append(QStringLiteral("\n  %1: %2").arg(c, lib.errorString()));
    }
    return false;
}

bool UhdInput::open(QString* error)
{
    QString tried;
    if (!loadLibrary(api_->lib, cfg_.uhdLibrary, &tried)) {
        if (error)
            *error = QStringLiteral("The UHD driver was not found. Install UHD from https://files.ettus.com/binaries/uhd/ "
                                    "or set the path of uhd.dll in Settings > Live.%1").arg(tried);
        return false;
    }
    QStringList missing;
    const auto get = [&](auto& fn, const char* name) {
        fn = reinterpret_cast<std::remove_reference_t<decltype(fn)>>(api_->lib.resolve(name));
        if (!fn) missing << QLatin1String(name);
    };
    Api& a = *api_;
    get(a.usrp_make, "uhd_usrp_make");
    get(a.usrp_free, "uhd_usrp_free");
    get(a.get_last_error, "uhd_get_last_error");
    get(a.usrp_last_error, "uhd_usrp_last_error");
    get(a.rx_streamer_make, "uhd_rx_streamer_make");
    get(a.rx_streamer_free, "uhd_rx_streamer_free");
    get(a.rx_metadata_make, "uhd_rx_metadata_make");
    get(a.rx_metadata_free, "uhd_rx_metadata_free");
    get(a.get_rx_stream, "uhd_usrp_get_rx_stream");
    get(a.max_num_samps, "uhd_rx_streamer_max_num_samps");
    get(a.rx_recv, "uhd_rx_streamer_recv");
    get(a.issue_stream_cmd, "uhd_rx_streamer_issue_stream_cmd");
    get(a.set_rx_rate, "uhd_usrp_set_rx_rate");
    get(a.get_rx_rate, "uhd_usrp_get_rx_rate");
    get(a.set_rx_freq, "uhd_usrp_set_rx_freq");
    get(a.get_rx_freq, "uhd_usrp_get_rx_freq");
    get(a.set_rx_gain, "uhd_usrp_set_rx_gain");
    get(a.get_rx_gain, "uhd_usrp_get_rx_gain");
    get(a.set_rx_antenna, "uhd_usrp_set_rx_antenna");
    get(a.set_rx_bandwidth, "uhd_usrp_set_rx_bandwidth");
    get(a.set_clock_source, "uhd_usrp_set_clock_source");
    get(a.set_time_now, "uhd_usrp_set_time_now");
    get(a.get_pp_string, "uhd_usrp_get_pp_string");
    get(a.set_rx_subdev_spec, "uhd_usrp_set_rx_subdev_spec");
    get(a.subdev_spec_make, "uhd_subdev_spec_make");
    get(a.subdev_spec_free, "uhd_subdev_spec_free");
    get(a.md_error_code, "uhd_rx_metadata_error_code");
    get(a.md_has_time_spec, "uhd_rx_metadata_has_time_spec");
    get(a.md_time_spec, "uhd_rx_metadata_time_spec");
    if (!missing.isEmpty()) {
        if (error) *error = QStringLiteral("%1 lacks %2. Is it a UHD 3.15 or later library?").arg(a.lib.fileName(), missing.join(", "));
        return false;
    }
    a.get_version_string = reinterpret_cast<decltype(a.get_version_string)>(a.lib.resolve("uhd_get_version_string"));

    const auto check = [&](uhd_error e, const QString& what) {
        if (e == kUhdNoError) return true;
        if (error) *error = QStringLiteral("%1: %2").arg(what, lastError(e));
        return false;
    };

    const QByteArray args = cfg_.deviceArgs.toLocal8Bit();
    if (!check(a.usrp_make(&usrp_, args.constData()), QStringLiteral("Opening USRP \"%1\"").arg(cfg_.deviceArgs))) {
        usrp_ = nullptr;
        return false;
    }
    if (!cfg_.clockSource.isEmpty() && cfg_.clockSource != QLatin1String("internal")) {
        if (!check(a.set_clock_source(usrp_, cfg_.clockSource.toLocal8Bit().constData(), 0), QStringLiteral("Selecting the clock source")))
            return false;
    }
    if (!cfg_.subdev.isEmpty()) {
        SubdevHandle spec = nullptr;
        if (!check(a.subdev_spec_make(&spec, cfg_.subdev.toLocal8Bit().constData()), QStringLiteral("Subdevice \"%1\"").arg(cfg_.subdev)))
            return false;
        const uhd_error e = a.set_rx_subdev_spec(usrp_, spec, 0);
        a.subdev_spec_free(&spec);
        if (!check(e, QStringLiteral("Selecting subdevice \"%1\"").arg(cfg_.subdev))) return false;
    }
    if (!check(a.set_rx_rate(usrp_, cfg_.sampleRate, 0), QStringLiteral("Setting the sample rate"))) return false;
    if (!check(a.get_rx_rate(usrp_, 0, &info_.sampleRate), QStringLiteral("Reading the sample rate"))) return false;

    // With an LO offset the digital down-converter moves the band back to the centre, so the samples stay at zero IF
    QByteArray tuneArgs("");
    TuneRequest req{};
    req.target_freq = cfg_.centerFreq;
    req.rf_freq_policy = cfg_.loOffset != 0.0 ? kPolicyManual : kPolicyAuto;
    req.rf_freq = cfg_.centerFreq + cfg_.loOffset;
    req.dsp_freq_policy = kPolicyAuto;
    req.dsp_freq = 0;
    req.args = tuneArgs.data();
    TuneResult res{};
    if (!check(a.set_rx_freq(usrp_, &req, 0, &res), QStringLiteral("Tuning"))) return false;
    if (!check(a.get_rx_freq(usrp_, 0, &info_.centerFreq), QStringLiteral("Reading the tuned frequency"))) return false;

    if (!check(a.set_rx_gain(usrp_, cfg_.gainDb, 0, ""), QStringLiteral("Setting the gain"))) return false;
    a.get_rx_gain(usrp_, 0, "", &info_.gainDb);
    if (!cfg_.antenna.isEmpty() && !check(a.set_rx_antenna(usrp_, cfg_.antenna.toLocal8Bit().constData(), 0), QStringLiteral("Selecting antenna \"%1\"").arg(cfg_.antenna)))
        return false;
    if (cfg_.bandwidth > 0 && !check(a.set_rx_bandwidth(usrp_, cfg_.bandwidth, 0), QStringLiteral("Setting the bandwidth"))) return false;
    if (!check(a.set_time_now(usrp_, 0, 0.0, 0), QStringLiteral("Setting the device time"))) return false;

    if (!check(a.rx_streamer_make(&streamer_), QStringLiteral("Creating the receive streamer"))) return false;
    if (!check(a.rx_metadata_make(&metadata_), QStringLiteral("Creating the metadata"))) return false;
    QByteArray cpu("sc16"), otw("sc16"), sargs("");
    size_t channel = 0;
    StreamArgs sa{cpu.data(), otw.data(), sargs.data(), &channel, 1};
    if (!check(a.get_rx_stream(usrp_, &sa, streamer_), QStringLiteral("Opening the receive stream"))) return false;
    if (!check(a.max_num_samps(streamer_, &maxSamps_), QStringLiteral("Reading the packet size"))) return false;

    char version[128] = {0};
    if (a.get_version_string) a.get_version_string(version, sizeof(version));
    char pp[4096] = {0};
    a.get_pp_string(usrp_, pp, sizeof(pp));
    info_.details = QStringLiteral("UHD %1 (%2)\n%3").arg(QString::fromLatin1(version), a.lib.fileName(), QString::fromLocal8Bit(pp).trimmed());
    info_.description = QStringLiteral("USRP %1, %2 MS/s at %3 MHz, gain %4 dB")
                            .arg(cfg_.deviceArgs).arg(info_.sampleRate / 1e6, 0, 'f', 3)
                            .arg(info_.centerFreq / 1e6, 0, 'f', 6).arg(info_.gainDb, 0, 'f', 1);
    return true;
}

bool UhdInput::start(QString* error)
{
    if (!usrp_ || !ring_) {
        if (error) *error = QStringLiteral("USRP not open");
        return false;
    }
    StreamCmd cmd{};
    cmd.stream_mode = kStreamStart;
    cmd.num_samps = 0;
    cmd.stream_now = true;
    const uhd_error e = api_->issue_stream_cmd(streamer_, &cmd);
    if (e != kUhdNoError) {
        if (error) *error = QStringLiteral("Starting the stream: %1").arg(lastError(e));
        return false;
    }
    thread_ = QThread::create([this] { run(); });
    thread_->start(QThread::TimeCriticalPriority);    // the network buffers only ride out short delays
    return true;
}

void UhdInput::stop()
{
    stop_ = true;
    if (thread_) {
        thread_->wait();
        delete thread_;
        thread_ = nullptr;
        StreamCmd cmd{};
        cmd.stream_mode = kStreamStop;
        cmd.stream_now = true;
        api_->issue_stream_cmd(streamer_, &cmd);
    }
}

void UhdInput::closeDevice()
{
    if (metadata_) api_->rx_metadata_free(&metadata_);
    if (streamer_) api_->rx_streamer_free(&streamer_);
    if (usrp_) api_->usrp_free(&usrp_);
    metadata_ = streamer_ = usrp_ = nullptr;
}

void UhdInput::run()
{
    const size_t chunk = std::max<size_t>(maxSamps_, 1) * 8;
    std::vector<int16_t> buf(chunk * 2);
    void* buffs[1] = {buf.data()};
    const double rate = info_.sampleRate;
    bool haveStart = false;
    double t0 = 0;
    int silent = 0;
    while (!stop_.load()) {
        size_t got = 0;
        const uhd_error e = api_->rx_recv(streamer_, buffs, chunk, &metadata_, 0.5, false, &got);
        if (e != kUhdNoError) {
            fail(QStringLiteral("Receiving: %1").arg(lastError(e)));
            return;
        }
        int code = kMdNone;
        api_->md_error_code(metadata_, &code);
        if (code == kMdOverflow) {
            overflows_.fetch_add(1);      // the next packet's time stamp tells how much was lost
        } else if (code == kMdTimeout) {
            if (++silent >= 10) {         // 5 s without a single packet
                fail(QStringLiteral("The USRP stopped sending samples"));
                return;
            }
            continue;
        } else if (code != kMdNone) {
            fail(QStringLiteral("Receive error, metadata code 0x%1").arg(code, 0, 16));
            return;
        }
        if (got == 0) continue;
        silent = 0;

        bool hasTime = false;
        api_->md_has_time_spec(metadata_, &hasTime);
        if (hasTime) {
            int64_t full = 0;
            double frac = 0;
            api_->md_time_spec(metadata_, &full, &frac);
            const double t = double(full) + frac;
            if (!haveStart) { t0 = t; haveStart = true; }
            // Keep every sample at the index that matches its time, so a dropped packet does not shift later ones
            const qint64 gap = std::llround((t - t0) * rate) - ring_->head();
            if (gap > 0) {
                ring_->writeZeros(gap);
                lost_.fetch_add(gap);
            }
        }
        ring_->write(buf.data(), qint64(got));
    }
}

} // namespace gnss
