#include "LiveInput.h"

#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include <QLibrary>
#include <QThread>

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace gnss {

namespace {

// ---------------------------------------------------------------------------
// The part of libhackrf's hackrf.h used here. It has kept these since 2018. GPSSim's HackRfOutput.cpp uses the same.
// ---------------------------------------------------------------------------
constexpr int kSuccess = 0, kTrue = 1;

struct Transfer
{
    void* device;
    uint8_t* buffer;
    int buffer_length;
    int valid_length;
    void* rx_ctx;
    void* tx_ctx;
};

} // namespace

struct HackRfInput::Api
{
    QLibrary lib;
    int (*init)() = nullptr;
    int (*exit)() = nullptr;
    int (*open_by_serial)(const char*, void**) = nullptr;
    int (*close)(void*) = nullptr;
    int (*start_rx)(void*, int (*)(void*), void*) = nullptr;
    int (*stop_rx)(void*) = nullptr;
    int (*is_streaming)(void*) = nullptr;
    int (*set_freq)(void*, uint64_t) = nullptr;
    int (*set_sample_rate)(void*, double) = nullptr;
    int (*set_baseband_filter_bandwidth)(void*, uint32_t) = nullptr;
    uint32_t (*compute_baseband_filter_bw)(uint32_t) = nullptr;
    int (*set_amp_enable)(void*, uint8_t) = nullptr;
    int (*set_lna_gain)(void*, uint32_t) = nullptr;
    int (*set_vga_gain)(void*, uint32_t) = nullptr;
    int (*set_antenna_enable)(void*, uint8_t) = nullptr;
    const char* (*error_name)(int) = nullptr;
    int (*board_id_read)(void*, uint8_t*) = nullptr;
    const char* (*board_id_name)(int) = nullptr;
    int (*version_string_read)(void*, char*, uint8_t) = nullptr;
    int (*get_clkin_status)(void*, uint8_t*) = nullptr;     // optional, like the one below
    const char* (*library_version)() = nullptr;
};

HackRfInput::HackRfInput(const LiveSettings& s) : cfg_(s), api_(new Api) {}

HackRfInput::~HackRfInput()
{
    stop();
    if (device_) {
        api_->set_antenna_enable(device_, 0);      // the HackRF keeps its settings: no power left on the port
        api_->set_amp_enable(device_, 0);
        api_->close(device_);
    }
    if (initialised_) api_->exit();
}

QString HackRfInput::failure(const QString& what, int code) const
{
    const char* name = api_->error_name ? api_->error_name(code) : nullptr;
    return QStringLiteral("%1: %2").arg(what, name ? QString::fromLatin1(name) : QStringLiteral("error %1").arg(code));
}

bool HackRfInput::open(QString* error)
{
    if (cfg_.sampleRate < 2e6 || cfg_.sampleRate > 20e6) {
        if (error) *error = QStringLiteral("A HackRF samples at 2 to 20 MS/s");
        return false;
    }
    Api& a = *api_;
    QStringList candidates;
    if (!cfg_.hackrfLibrary.isEmpty())
        candidates << cfg_.hackrfLibrary;
    else
        candidates << QDir(QCoreApplication::applicationDirPath()).filePath(QStringLiteral("hackrf.dll")) << QStringLiteral("hackrf");
    QString tried;
    bool loaded = false;
    for (const QString& c : candidates) {
        const QFileInfo fi(c);
        if (fi.isAbsolute() && !fi.exists() && c != cfg_.hackrfLibrary) continue;
        if (fi.isAbsolute())                                 // libusb-1.0.dll sits next to it
            qputenv("PATH", QDir::toNativeSeparators(fi.absolutePath()).toLocal8Bit() + ';' + qgetenv("PATH"));
        a.lib.setFileName(c);
        if ((loaded = a.lib.load())) break;
        tried += QStringLiteral("\n  %1: %2").arg(c, a.lib.errorString());
    }
    if (!loaded) {
        if (error)
            *error = QStringLiteral("hackrf.dll was not found. Put it and libusb-1.0.dll next to the program, or set its path in "
                                    "Settings > Live (see the README).%1").arg(tried);
        return false;
    }
    QStringList missing;
    const auto get = [&](auto& fn, const char* name, bool required = true) {
        fn = reinterpret_cast<std::remove_reference_t<decltype(fn)>>(a.lib.resolve(name));
        if (!fn && required) missing << QLatin1String(name);
    };
    get(a.init, "hackrf_init");
    get(a.exit, "hackrf_exit");
    get(a.open_by_serial, "hackrf_open_by_serial");
    get(a.close, "hackrf_close");
    get(a.start_rx, "hackrf_start_rx");
    get(a.stop_rx, "hackrf_stop_rx");
    get(a.is_streaming, "hackrf_is_streaming");
    get(a.set_freq, "hackrf_set_freq");
    get(a.set_sample_rate, "hackrf_set_sample_rate");
    get(a.set_baseband_filter_bandwidth, "hackrf_set_baseband_filter_bandwidth");
    get(a.compute_baseband_filter_bw, "hackrf_compute_baseband_filter_bw");
    get(a.set_amp_enable, "hackrf_set_amp_enable");
    get(a.set_lna_gain, "hackrf_set_lna_gain");
    get(a.set_vga_gain, "hackrf_set_vga_gain");
    get(a.set_antenna_enable, "hackrf_set_antenna_enable");
    get(a.error_name, "hackrf_error_name");
    get(a.board_id_read, "hackrf_board_id_read");
    get(a.board_id_name, "hackrf_board_id_name");
    get(a.version_string_read, "hackrf_version_string_read");
    get(a.get_clkin_status, "hackrf_get_clkin_status", false);
    get(a.library_version, "hackrf_library_version", false);
    if (!missing.isEmpty()) {
        if (error) *error = QStringLiteral("%1 lacks %2. Is it libhackrf 2018 or later?").arg(a.lib.fileName(), missing.join(", "));
        return false;
    }

    const auto check = [&](int code, const QString& what) {
        if (code == kSuccess) return true;
        if (error) *error = failure(what, code);
        return false;
    };
    if (!check(a.init(), QStringLiteral("Starting libhackrf"))) return false;
    initialised_ = true;
    const QByteArray serial = cfg_.hackrfSerial.trimmed().toLatin1();
    if (!check(a.open_by_serial(serial.isEmpty() ? nullptr : serial.constData(), &device_),
               serial.isEmpty() ? QStringLiteral("Opening the HackRF") : QStringLiteral("Opening HackRF %1").arg(cfg_.hackrfSerial))) {
        device_ = nullptr;
        return false;
    }

    // Asking for less makes a fast crystal give the exact frequency and sample rate
    const double scale = 1.0 + cfg_.hackrfPpm * 1e-6;
    const double tuned = cfg_.centerFreq + cfg_.loOffset;      // no down-converter to move the band back: it sits at -offset
    const uint32_t lna = uint32_t(std::clamp(cfg_.hackrfLnaGain, 0, 40) / 8 * 8);
    const uint32_t vga = uint32_t(std::clamp(cfg_.hackrfVgaGain, 0, 62) / 2 * 2);
    if (!check(a.set_sample_rate(device_, cfg_.sampleRate / scale), QStringLiteral("Setting the sample rate"))) return false;
    // After the rate, which picks a filter of its own; at least the C/A main lobe
    const uint32_t filter = a.compute_baseband_filter_bw(uint32_t(std::max(0.75 * cfg_.sampleRate, 2.5e6)));
    if (!check(a.set_baseband_filter_bandwidth(device_, filter), QStringLiteral("Setting the filter"))) return false;
    if (!check(a.set_freq(device_, uint64_t(std::llround(tuned / scale))), QStringLiteral("Tuning"))) return false;
    if (!check(a.set_lna_gain(device_, lna), QStringLiteral("Setting the LNA gain"))) return false;
    if (!check(a.set_vga_gain(device_, vga), QStringLiteral("Setting the VGA gain"))) return false;
    if (!check(a.set_amp_enable(device_, cfg_.hackrfAmp ? 1 : 0), QStringLiteral("Setting the amplifier"))) return false;
    if (!check(a.set_antenna_enable(device_, cfg_.hackrfAntennaPower ? 1 : 0), QStringLiteral("Setting the antenna power"))) return false;

    uint8_t board = 0, clkin = 0;
    a.board_id_read(device_, &board);
    char version[255] = {0};
    a.version_string_read(device_, version, sizeof(version) - 1);
    const bool locked = a.get_clkin_status && a.get_clkin_status(device_, &clkin) == kSuccess && clkin;
    info_.sampleRate = cfg_.sampleRate;
    info_.centerFreq = tuned;
    info_.gainDb = double(lna + vga) + (cfg_.hackrfAmp ? 14.0 : 0.0);
    info_.bits = 8;
    info_.description = QStringLiteral("HackRF, %1 MS/s at %2 MHz, LNA %3 + VGA %4 dB, amplifier %5, correction %6 ppm%7")
                            .arg(cfg_.sampleRate / 1e6, 0, 'f', 3).arg(tuned / 1e6, 0, 'f', 6).arg(lna).arg(vga)
                            .arg(cfg_.hackrfAmp ? QStringLiteral("on") : QStringLiteral("off")).arg(cfg_.hackrfPpm, 0, 'f', 2)
                            .arg(cfg_.hackrfAntennaPower ? QStringLiteral(", 3.3 V on the antenna port") : QString());
    info_.details = QStringLiteral("%1, firmware %2, libhackrf %3, filter %4 MHz, %5, antenna power %6")
                        .arg(QString::fromLatin1(a.board_id_name(board)), QString::fromLatin1(version),
                             QString::fromLatin1(a.library_version ? a.library_version() : ""))
                        .arg(filter / 1e6, 0, 'f', 2)
                        .arg(locked ? QStringLiteral("locked to CLKIN") : QStringLiteral("own crystal"),
                             cfg_.hackrfAntennaPower ? QStringLiteral("on") : QStringLiteral("off"));
    return true;
}

bool HackRfInput::start(QString* error)
{
    if (!device_ || !ring_) {
        if (error) *error = QStringLiteral("HackRF not open");
        return false;
    }
    const int e = api_->start_rx(device_, &HackRfInput::onTransfer, this);
    if (e != kSuccess) {
        if (error) *error = failure(QStringLiteral("Starting the stream"), e);
        return false;
    }
    streaming_ = true;
    watchdog_ = QThread::create([this] { watch(); });
    watchdog_->start();
    return true;
}

void HackRfInput::stop()
{
    stop_ = true;
    if (watchdog_) {
        watchdog_->wait();
        delete watchdog_;
        watchdog_ = nullptr;
    }
    if (streaming_) {
        api_->stop_rx(device_);
        streaming_ = false;
    }
}

// On the HackRF's thread
int HackRfInput::onTransfer(void* transfer)
{
    auto* t = static_cast<Transfer*>(transfer);
    auto* self = static_cast<HackRfInput*>(t->rx_ctx);
    if (self->stop_.load()) return 0;
    const qint64 samples = t->valid_length / 2;
    self->iq_.resize(size_t(samples) * 2);
    const auto* in = reinterpret_cast<const int8_t*>(t->buffer);
    for (size_t i = 0; i < self->iq_.size(); ++i) self->iq_[i] = int16_t(in[i] * 256);
    self->ring_->write(self->iq_.data(), samples);
    self->received_.fetch_add(samples);
    return 0;
}

// The library reports nothing when the HackRF goes away: its samples just stop
void HackRfInput::watch()
{
    qint64 last = -1;
    int quiet = 0;
    while (!stop_.load()) {
        QThread::msleep(100);
        const qint64 now = received_.load();
        quiet = now == last ? quiet + 1 : 0;
        last = now;
        if (quiet >= 30 || api_->is_streaming(device_) != kTrue) {
            fail(QStringLiteral("The HackRF stopped sending samples. Was it unplugged?"));
            return;
        }
    }
}

} // namespace gnss
