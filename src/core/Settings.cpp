#include "Settings.h"
#include "Constants.h"

#include <QFileInfo>
#include <QRegularExpression>
#include <QSettings>
#include <cmath>

namespace gnss {

QString sampleFormatName(SampleFormat f)
{
    switch (f) {
    case SampleFormat::Int8Real:  return QStringLiteral("int8 real");
    case SampleFormat::UInt8Real: return QStringLiteral("uint8 real (offset binary)");
    case SampleFormat::Int16Real: return QStringLiteral("int16 real (little endian)");
    case SampleFormat::Int8IQ:    return QStringLiteral("int8 complex I/Q interleaved");
    case SampleFormat::Int16IQ:   return QStringLiteral("int16 complex I/Q interleaved");
    case SampleFormat::Packed2Real: return QStringLiteral("2-bit real, packed");
    }
    return QStringLiteral("?");
}

QString satName(int id)
{
    if (isGlonass(id)) return QStringLiteral("R%1%2").arg(svidOf(id) < 0 ? '-' : '+').arg(std::abs(svidOf(id)));
    if (isGalileo(id)) return QStringLiteral("E%1").arg(svidOf(id), 2, 10, QLatin1Char('0'));
    return QStringLiteral("PRN %1").arg(id);
}

ReceiverSettings::ReceiverSettings()
{
    for (int p = 1; p <= 32; ++p) acqPrns.append(p);
}

int ReceiverSettings::samplesPerCode() const
{
    return int(std::lround(samplingFreq / 1000.0));
}

bool ReceiverSettings::isComplex() const
{
    return format == SampleFormat::Int8IQ || format == SampleFormat::Int16IQ;
}

int ReceiverSettings::bytesPerSample() const
{
    switch (format) {
    case SampleFormat::Int8Real:
    case SampleFormat::UInt8Real:
    case SampleFormat::Packed2Real: return 1;
    case SampleFormat::Int16Real: return 2;
    case SampleFormat::Int8IQ:    return 2;
    case SampleFormat::Int16IQ:   return 4;
    }
    return 1;
}

QVector<int> ReceiverSettings::prnList() const
{
    if (!acqPrns.isEmpty()) return acqPrns;
    QVector<int> v;
    for (int p = 1; p <= 32; ++p) v.append(p);
    return v;
}

double ReceiverSettings::dopplerSignValue() const
{
    return dopplerSign == DopplerSign::Inverted ? -1.0 : 1.0;
}

static double numberFromCaptures(const QString& a, const QString& b)
{
    QString s = a;
    if (!b.isEmpty()) s += QLatin1Char('.') + b;
    double v = s.toDouble();
    if (v < 1.0e4) v *= 1.0e6;   // values without units are in MHz
    return v;
}

bool ReceiverSettings::parseFileName(const QString& path, double* fs, double* fif, SampleFormat* fmt)
{
    const QString base = QFileInfo(path).fileName();
    static const QRegularExpression reFs(QStringLiteral("(?:^|[^A-Za-z])fs[_\\-=]?(\\d+)(?:[_.p](\\d+))?"),
                                         QRegularExpression::CaseInsensitiveOption);
    static const QRegularExpression reIf(QStringLiteral("(?:^|[^A-Za-z])if[_\\-=]?(\\d+)(?:[_.p](\\d+))?"),
                                         QRegularExpression::CaseInsensitiveOption);
    const auto mFs = reFs.match(base);
    const auto mIf = reIf.match(base);
    if (mFs.hasMatch() && fs) *fs = numberFromCaptures(mFs.captured(1), mFs.captured(2));
    if (mIf.hasMatch() && fif) *fif = numberFromCaptures(mIf.captured(1), mIf.captured(2));

    if (fmt) {
        const QString lower = base.toLower();
        const bool iq = lower.contains(QLatin1String("iq")) || lower.contains(QLatin1String("complex"));
        if (lower.contains(QLatin1String("2bit")))
            *fmt = SampleFormat::Packed2Real;
        else if (lower.contains(QLatin1String("int16")) || lower.contains(QLatin1String("i16")))
            *fmt = iq ? SampleFormat::Int16IQ : SampleFormat::Int16Real;
        else if (lower.contains(QLatin1String("uint8")) || lower.contains(QLatin1String("u8")))
            *fmt = SampleFormat::UInt8Real;
        else
            *fmt = iq ? SampleFormat::Int8IQ : SampleFormat::Int8Real;
    }
    return mFs.hasMatch() && mIf.hasMatch();
}

QString ReceiverSettings::summary() const
{
    return QStringLiteral("fs = %1 MHz, IF = %2 MHz, format = %3, channels = %4, PLL %5 Hz, DLL %6 Hz, FLL %7 Hz")
        .arg(samplingFreq / 1e6, 0, 'f', 4)
        .arg(ifFreq / 1e6, 0, 'f', 4)
        .arg(sampleFormatName(format))
        .arg(numChannels)
        .arg(pllBandwidth).arg(dllBandwidth).arg(fllBandwidth);
}

void ReceiverSettings::save(QSettings& s) const
{
    s.beginGroup(QStringLiteral("receiver"));
    s.setValue("constellations", int(constellations));
    s.setValue("galNonCoherent", galNonCoherent);
    s.setValue("galDllSpacing", galDllSpacing);
    s.setValue("acqStartMs", acqStartMs);
    s.setValue("acqSearchBand", acqSearchBand);
    s.setValue("acqStep", acqStep);
    s.setValue("acqNonCoherent", acqNonCoherent);
    s.setValue("acqThreshold", acqThreshold);
    s.setValue("fineFreqMs", fineFreqMs);
    s.setValue("numChannels", numChannels);
    s.setValue("dllBandwidth", dllBandwidth);
    s.setValue("dllDamping", dllDamping);
    s.setValue("dllSpacing", dllSpacing);
    s.setValue("pllBandwidth", pllBandwidth);
    s.setValue("pllDamping", pllDamping);
    s.setValue("fllBandwidth", fllBandwidth);
    s.setValue("fllAssistMs", fllAssistMs);
    s.setValue("carrierAiding", carrierAiding);
    s.setValue("cn0WindowMs", cn0WindowMs);
    s.setValue("bitSyncStartMs", bitSyncStartMs);
    s.setValue("navSolPeriodMs", navSolPeriodMs);
    s.setValue("elevationMask", elevationMask);
    s.setValue("minCn0", minCn0);
    s.setValue("useIono", useIono);
    s.setValue("useTropo", useTropo);
    s.setValue("useEarthRotation", useEarthRotation);
    s.setValue("elevationWeighting", elevationWeighting);
    s.setValue("dopplerSign", int(dopplerSign));
    s.endGroup();

    s.beginGroup(QStringLiteral("live"));
    s.setValue("source", int(live.source));
    s.setValue("deviceArgs", live.deviceArgs);
    s.setValue("subdev", live.subdev);
    s.setValue("antenna", live.antenna);
    s.setValue("clockSource", live.clockSource);
    s.setValue("uhdLibrary", live.uhdLibrary);
    s.setValue("smLibrary", live.smLibrary);
    s.setValue("smCenter", live.smCenter);
    s.setValue("smRefLevel", live.smRefLevel);
    s.setValue("centerFreq", live.centerFreq);
    s.setValue("sampleRate", live.sampleRate);
    s.setValue("gainDb", live.gainDb);
    s.setValue("bandwidth", live.bandwidth);
    s.setValue("loOffset", live.loOffset);
    s.setValue("hackrfSerial", live.hackrfSerial);
    s.setValue("hackrfLibrary", live.hackrfLibrary);
    s.setValue("hackrfLnaGain", live.hackrfLnaGain);
    s.setValue("hackrfVgaGain", live.hackrfVgaGain);
    s.setValue("hackrfAmp", live.hackrfAmp);
    s.setValue("hackrfAntennaPower", live.hackrfAntennaPower);
    s.setValue("hackrfPpm", live.hackrfPpm);
    s.setValue("ringSeconds", live.ringSeconds);
    s.setValue("windowSeconds", live.windowSeconds);
    s.setValue("historySeconds", live.historySeconds);
    s.setValue("updateMs", live.updateMs);
    s.setValue("acqIntervalSec", live.acqIntervalSec);
    s.setValue("replaySpeed", live.replaySpeed);
    s.setValue("tcpHost", live.tcpHost);
    s.setValue("tcpPort", live.tcpPort);
    s.setValue("record", live.record);
    s.setValue("recordDir", live.recordDir);
    s.endGroup();
}

void ReceiverSettings::load(QSettings& s)
{
    ReceiverSettings d;
    s.beginGroup(QStringLiteral("receiver"));
    constellations = Constellations(s.value("constellations", int(d.constellations)).toInt());
    galNonCoherent = s.value("galNonCoherent", d.galNonCoherent).toInt();
    galDllSpacing = s.value("galDllSpacing", d.galDllSpacing).toDouble();
    acqStartMs = s.value("acqStartMs", d.acqStartMs).toInt();
    acqSearchBand = s.value("acqSearchBand", d.acqSearchBand).toDouble();
    acqStep = s.value("acqStep", d.acqStep).toDouble();
    acqNonCoherent = s.value("acqNonCoherent", d.acqNonCoherent).toInt();
    acqThreshold = s.value("acqThreshold", d.acqThreshold).toDouble();
    fineFreqMs = s.value("fineFreqMs", d.fineFreqMs).toInt();
    numChannels = s.value("numChannels", d.numChannels).toInt();
    dllBandwidth = s.value("dllBandwidth", d.dllBandwidth).toDouble();
    dllDamping = s.value("dllDamping", d.dllDamping).toDouble();
    dllSpacing = s.value("dllSpacing", d.dllSpacing).toDouble();
    pllBandwidth = s.value("pllBandwidth", d.pllBandwidth).toDouble();
    pllDamping = s.value("pllDamping", d.pllDamping).toDouble();
    fllBandwidth = s.value("fllBandwidth", d.fllBandwidth).toDouble();
    fllAssistMs = s.value("fllAssistMs", d.fllAssistMs).toInt();
    carrierAiding = s.value("carrierAiding", d.carrierAiding).toBool();
    cn0WindowMs = s.value("cn0WindowMs", d.cn0WindowMs).toInt();
    bitSyncStartMs = s.value("bitSyncStartMs", d.bitSyncStartMs).toInt();
    navSolPeriodMs = s.value("navSolPeriodMs", d.navSolPeriodMs).toInt();
    elevationMask = s.value("elevationMask", d.elevationMask).toDouble();
    minCn0 = s.value("minCn0", d.minCn0).toDouble();
    useIono = s.value("useIono", d.useIono).toBool();
    useTropo = s.value("useTropo", d.useTropo).toBool();
    useEarthRotation = s.value("useEarthRotation", d.useEarthRotation).toBool();
    elevationWeighting = s.value("elevationWeighting", d.elevationWeighting).toBool();
    dopplerSign = DopplerSign(s.value("dopplerSign", int(d.dopplerSign)).toInt());
    s.endGroup();

    const LiveSettings ld;
    s.beginGroup(QStringLiteral("live"));
    live.source = LiveSourceKind(s.value("source", int(ld.source)).toInt());
    live.deviceArgs = s.value("deviceArgs", ld.deviceArgs).toString();
    live.subdev = s.value("subdev", ld.subdev).toString();
    live.antenna = s.value("antenna", ld.antenna).toString();
    live.clockSource = s.value("clockSource", ld.clockSource).toString();
    live.uhdLibrary = s.value("uhdLibrary", ld.uhdLibrary).toString();
    live.smLibrary = s.value("smLibrary", ld.smLibrary).toString();
    live.smCenter = s.value("smCenter", ld.smCenter).toDouble();
    live.smRefLevel = s.value("smRefLevel", ld.smRefLevel).toDouble();
    live.centerFreq = s.value("centerFreq", ld.centerFreq).toDouble();
    live.sampleRate = s.value("sampleRate", ld.sampleRate).toDouble();
    live.gainDb = s.value("gainDb", ld.gainDb).toDouble();
    live.bandwidth = s.value("bandwidth", ld.bandwidth).toDouble();
    live.loOffset = s.value("loOffset", ld.loOffset).toDouble();
    live.hackrfSerial = s.value("hackrfSerial", ld.hackrfSerial).toString();
    live.hackrfLibrary = s.value("hackrfLibrary", ld.hackrfLibrary).toString();
    live.hackrfLnaGain = s.value("hackrfLnaGain", ld.hackrfLnaGain).toInt();
    live.hackrfVgaGain = s.value("hackrfVgaGain", ld.hackrfVgaGain).toInt();
    live.hackrfAmp = s.value("hackrfAmp", ld.hackrfAmp).toBool();
    live.hackrfAntennaPower = s.value("hackrfAntennaPower", ld.hackrfAntennaPower).toBool();
    live.hackrfPpm = s.value("hackrfPpm", ld.hackrfPpm).toDouble();
    live.ringSeconds = s.value("ringSeconds", ld.ringSeconds).toInt();
    live.windowSeconds = s.value("windowSeconds", ld.windowSeconds).toInt();
    live.historySeconds = s.value("historySeconds", ld.historySeconds).toInt();
    live.updateMs = s.value("updateMs", ld.updateMs).toInt();
    live.acqIntervalSec = s.value("acqIntervalSec", ld.acqIntervalSec).toInt();
    live.replaySpeed = s.value("replaySpeed", ld.replaySpeed).toDouble();
    live.tcpHost = s.value("tcpHost", ld.tcpHost).toString();
    live.tcpPort = s.value("tcpPort", ld.tcpPort).toInt();
    live.record = s.value("record", ld.record).toBool();
    live.recordDir = s.value("recordDir", ld.recordDir).toString();
    s.endGroup();
}

} // namespace gnss
