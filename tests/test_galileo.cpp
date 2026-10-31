// Checks the I/NAV decoder against the worked example in Annex D.2 of the Galileo OS SIS ICD Issue 2.1,
// and the first chips of the code tables.

#include "core/Constants.h"
#include "core/GalileoCodes.h"
#include "core/GalileoNav.h"
#include "core/Tracking.h"

#include <QDir>
#include <QFile>
#include <QTemporaryDir>

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

using namespace gnss;

namespace {

int failures = 0;

void check(bool ok, const char* what)
{
    std::printf("%s  %s\n", ok ? "pass" : "FAIL", what);
    failures += ok ? 0 : 1;
}

// Tracks a clean E1-C signal delayed by 'delay' chips from a replica that starts at zero delay. Returns the code loop's
// final error in chips.
double trackFrom(double delay, int* jumps)
{
    const double fs = 16.3676e6, fif = 4.1304e6;
    const auto table = GalileoCode::boc11(GalileoCode::e1c(1));
    QTemporaryDir dir;
    const QString path = QDir(dir.path()).filePath(QStringLiteral("e1.bin"));
    QByteArray data(int(fs), Qt::Uninitialized);
    for (int k = 0; k < data.size(); ++k) {
        const double t = k / fs;
        const int half = int(std::floor((t * kCaChipRate - delay) * 2.0 + 8184.0)) % 8184;
        data[k] = char(std::lround(60.0 * table[size_t(half)] * std::cos(kTwoPi * fif * t)));
    }
    QFile out(path);
    if (!out.open(QIODevice::WriteOnly)) return 1e9;
    out.write(data);
    out.close();

    ReceiverSettings s;
    s.samplingFreq = fs;
    s.ifFreq = fif;
    s.format = SampleFormat::Int8Real;
    SignalSource src;
    src.open(path, s.format, 0, nullptr);
    AcqResult acq;
    acq.prn = kGalileoIdBase + 1;
    acq.fineFreq = fif;
    TrackingChannel ch;
    Tracking::trackChannel(src, s, acq, 0, ch, nullptr, nullptr);
    src.close();
    *jumps = ch.sidePeakJumps;
    const double spc = fs / kCaChipRate;
    double chips = (ch.codeEpochSample.back() / spc - delay) / kE1CodeLength;
    chips -= std::round(chips);
    return chips * kE1CodeLength;
}

std::vector<int> bitsOf(const std::string& text)
{
    std::vector<int> v;
    for (char c : text)
        if (c == '0' || c == '1') v.push_back(c - '0');
    return v;
}

} // namespace

int main()
{
    const auto input = bitsOf("11111111 11110000 11001100 10101010 00000000 00001111 00110011 01010101 "
                              "11100011 11101100 11011111 10001010 00011100 00010011 01000000");
    const auto encoded = bitsOf("10001100 00011010 10101010 01110011 00110001 01011010 01101111 01011001 "
                                "01111000 10010101 01010101 10001100 11001110 10100101 10010000 10100110 "
                                "10000100 00000010 00000010 00011011 10011001 11101011 01011100 00011000 "
                                "10111011 11111101 11111101 11100100 01010011 00100010");
    const auto interleaved = bitsOf("10100000 01011111 10001100 11110000 01011110 10100000 00011001 11100011 "
                                    "10101000 01010000 01001111 01010111 01111000 10000110 11111010 11100111 "
                                    "10011000 00011111 11100010 00001001 11110110 00001001 11000111 01100000 "
                                    "10010111 01001000 11000110 11011001 00000111 00111010");
    check(input.size() == 120 && encoded.size() == 240 && interleaved.size() == 240, "example vectors have the ICD lengths");

    std::vector<float> rx(240), deint(240);
    for (int i = 0; i < 240; ++i) rx[size_t(i)] = interleaved[size_t(i)] ? -1.0f : 1.0f;
    inav::deinterleave(rx.data(), deint.data());
    bool same = true;
    for (int i = 0; i < 240; ++i) same = same && (deint[size_t(i)] < 0) == (encoded[size_t(i)] != 0);
    check(same, "deinterleaver reproduces the encoded symbols");

    std::vector<uint8_t> out(120);
    inav::viterbi(deint.data(), out.data());
    same = true;
    for (int i = 0; i < 120; ++i) same = same && out[size_t(i)] == input[size_t(i)];
    check(same, "Viterbi decoder reproduces the input bits");

    for (int i : {3, 41, 97, 150, 201}) deint[size_t(i)] = -deint[size_t(i)];
    inav::viterbi(deint.data(), out.data());
    same = true;
    for (int i = 0; i < 120; ++i) same = same && out[size_t(i)] == input[size_t(i)];
    check(same, "Viterbi decoder corrects five spread symbol errors");

    // Standard CRC-24Q check value for the ASCII string "123456789"
    std::vector<uint8_t> msg;
    for (char c : std::string("123456789"))
        for (int b = 7; b >= 0; --b) msg.push_back(uint8_t((c >> b) & 1));
    check(inav::crc24q(msg.data(), int(msg.size())) == 0xCDE703, "CRC-24Q check value");

    const auto e1b = GalileoCode::e1b(1);
    const auto e1c = GalileoCode::e1c(1);
    check(e1b.size() == 4092 && e1c.size() == 4092, "primary code length");
    // F5D7 = 1111 0101 1101 0111
    const int expect[16] = {1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1};
    same = true;
    for (int i = 0; i < 16; ++i) same = same && (e1b[size_t(i)] < 0) == (expect[i] != 0);
    check(same, "E1-B SVID 1 starts with F5D7, first chip in the top bit");

    // The code loop settles 0.5 + d/2 chips from the true peak when it sits on a side peak
    int jumps = 0;
    double error = trackFrom(0.0, &jumps);
    check(jumps == 0 && std::fabs(error) < 0.02, "E1 tracker stays on the true BOC peak");
    error = trackFrom(0.575, &jumps);
    check(jumps == 1 && std::fabs(error) < 0.02, "E1 tracker leaves the side peak after the true one");
    error = trackFrom(-0.575, &jumps);
    check(jumps == 1 && std::fabs(error) < 0.02, "E1 tracker leaves the side peak before the true one");

    std::printf("%d failure(s)\n", failures);
    return failures ? 1 : 0;
}
