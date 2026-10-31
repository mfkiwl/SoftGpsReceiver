#include "LiveInput.h"

#include <QTcpSocket>
#include <QThread>

#include <cstring>
#include <vector>

namespace gnss {

namespace {
// The stream of GPSSim: this header, then samples. Keep in step with IqOutput.h there.
//   char[8] "GPSSIMIQ", u32 version = 1, u32 format (0 = int8 I/Q, 1 = int16 I/Q), f64 fs, f64 if, u32 week, u32 tow
constexpr int kHeaderSize = 40;
}

TcpInput::TcpInput(const LiveSettings& s) : cfg_(s) {}

TcpInput::~TcpInput()
{
    stop();
}

bool TcpInput::open(QString* error)
{
    thread_ = QThread::create([this] { run(); });
    thread_->start();
    while (!opened_.load() && !failed_.load()) QThread::msleep(5);
    if (failed_.load()) {
        if (error) *error = failure();
        stop();
        return false;
    }
    return true;
}

bool TcpInput::start(QString*)
{
    ready_ = true;
    return true;
}

void TcpInput::stop()
{
    stop_ = true;
    if (thread_) {
        thread_->wait();
        delete thread_;
        thread_ = nullptr;
    }
}

// The socket lives here from start to end, as it must stay on one thread
void TcpInput::run()
{
    QTcpSocket socket;
    socket.setReadBufferSize(64 << 20);
    socket.connectToHost(cfg_.tcpHost, quint16(cfg_.tcpPort));
    if (!socket.waitForConnected(3000)) {
        fail(QStringLiteral("Cannot connect to GPSSim at %1:%2: %3").arg(cfg_.tcpHost).arg(cfg_.tcpPort).arg(socket.errorString()));
        return;
    }
    while (socket.bytesAvailable() < kHeaderSize) {
        if (stop_.load() || !socket.waitForReadyRead(3000)) {
            fail(QStringLiteral("No stream header from %1:%2").arg(cfg_.tcpHost).arg(cfg_.tcpPort));
            return;
        }
    }
    char h[kHeaderSize];
    socket.read(h, kHeaderSize);
    uint32_t version = 0, format = 0;
    double fs = 0, ifFreq = 0;
    std::memcpy(&version, h + 8, 4);
    std::memcpy(&format, h + 12, 4);
    std::memcpy(&fs, h + 16, 8);
    std::memcpy(&ifFreq, h + 24, 8);
    if (std::memcmp(h, "GPSSIMIQ", 8) != 0 || version != 1 || format > 1 || fs <= 0) {
        fail(QStringLiteral("%1:%2 is not a GPSSim stream").arg(cfg_.tcpHost).arg(cfg_.tcpPort));
        return;
    }
    info_.sampleRate = fs;
    info_.centerFreq = kL1Freq - ifFreq;
    info_.description = QStringLiteral("GPSSim at %1:%2, %3 MS/s, %4").arg(cfg_.tcpHost).arg(cfg_.tcpPort).arg(fs / 1e6)
                            .arg(format == 0 ? QStringLiteral("int8 I/Q") : QStringLiteral("int16 I/Q"));
    info_.details = info_.description;
    opened_ = true;

    while (!ready_.load()) {
        if (stop_.load()) return;
        QThread::msleep(1);
    }

    const int bytesPerSample = format == 0 ? 2 : 4;
    std::vector<char> raw(1 << 20);
    std::vector<int16_t> iq;
    qint64 held = 0;
    while (!stop_.load()) {
        if (socket.bytesAvailable() == 0 && !socket.waitForReadyRead(100)) {
            if (socket.state() != QAbstractSocket::ConnectedState) break;      // the simulator finished
            continue;
        }
        const qint64 got = socket.read(raw.data() + held, qint64(raw.size()) - held);
        if (got <= 0) continue;
        held += got;
        const qint64 samples = held / bytesPerSample;
        if (format == 0) {
            iq.resize(size_t(samples) * 2);
            const auto* in = reinterpret_cast<const int8_t*>(raw.data());
            for (size_t i = 0; i < iq.size(); ++i) iq[i] = int16_t(in[i]) * 256;
            ring_->write(iq.data(), samples);
        } else {
            ring_->write(reinterpret_cast<const int16_t*>(raw.data()), samples);
        }
        const qint64 used = samples * bytesPerSample;
        std::memmove(raw.data(), raw.data() + used, size_t(held - used));
        held -= used;
    }
    finished_ = true;
}

} // namespace gnss
