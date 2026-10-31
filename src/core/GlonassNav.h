#pragma once

#include "NavMessage.h"

#include <array>

namespace gnss {
namespace glo {

// GLONASS L1OF navigation message (ICD Edition 5.1). A 2 s string is 1.7 s of data followed by a 0.3 s time mark. Its 85
// bits, b85 first, go out relative-coded at 50 bit/s, each bit as a Manchester pair of 10 ms symbols. The mark is 30 of
// those symbols. Strings 1 to 4 of each 30 s frame hold the satellite's own position, velocity and clock.
constexpr int kStringBits = 85;
constexpr int kStringSymbols = 200;     // of 10 ms, including the time mark
using StringBits = std::array<uint8_t, kStringBits + 1>;    // indexed by ICD bit number, 1..85

// True if the 8 check bits b8..b1 agree with the data
bool hammingOk(const StringBits& b);
// Sets b8..b1 from the data bits
void setHamming(StringBits& b);
// The 200 +/-1 symbols of a string. 'last' is the relative-coding state, normally 0 as b85 is always 0.
std::array<int8_t, kStringSymbols> modulate(const StringBits& b);

// Writes a field whose top bit is ICD bit 'hi'. Signed fields are sign and magnitude.
void putField(StringBits& b, int hi, int len, quint64 value);
void putSigned(StringBits& b, int hi, int len, double value, double lsb);

// The strings of one frame for an ephemeris; frameStart is the time of string 1 in seconds of the day
std::array<StringBits, 5> encodeFrame(const GlonassEphemeris& e, int frameStart);

NavChannel decodeChannel(const TrackingChannel& trk, const ReceiverSettings& s);

} // namespace glo
} // namespace gnss
