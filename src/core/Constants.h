#pragma once

namespace gnss {

constexpr double kSpeedOfLight   = 299792458.0;          // m/s
constexpr double kGpsPi          = 3.1415926535898;      // value of pi mandated by IS-GPS-200
constexpr double kPi             = 3.14159265358979323846;
constexpr double kTwoPi          = 2.0 * kPi;
constexpr double kMu             = 3.986005e14;          // WGS-84 earth gravitational constant (m^3/s^2)
constexpr double kOmegaEDot      = 7.2921151467e-5;      // WGS-84 earth rotation rate (rad/s)
constexpr double kRelF           = -4.442807633e-10;     // relativistic constant F (s/m^1/2)
constexpr double kL1Freq         = 1575.42e6;            // Hz
constexpr double kL1Wavelength   = kSpeedOfLight / kL1Freq;
constexpr double kCaChipRate     = 1.023e6;              // chips/s
constexpr int    kCaCodeLength   = 1023;                 // chips
constexpr double kCarrierToCode  = 1540.0;               // L1 carrier cycles per C/A chip
constexpr double kWgs84A         = 6378137.0;
constexpr double kWgs84F         = 1.0 / 298.257223563;
constexpr int    kSecondsPerWeek = 604800;
constexpr int    kHalfWeek       = 302400;
constexpr int    kBitsPerSubframe = 300;
constexpr int    kMsPerBit       = 20;

// Galileo E1 Open Service (OS SIS ICD Issue 2.1)
constexpr double kMuGal          = 3.986004418e14;       // geocentric gravitational constant (m^3/s^2)
constexpr double kRelFGal        = -4.442807309e-10;     // relativistic constant F (s/m^1/2)
constexpr int    kE1CodeLength   = 4092;                 // primary code chips, also 1.023 Mchip/s
constexpr int    kE1CodeMs       = 4;                    // primary code period (ms)
constexpr int    kInavPartSymbols = 250;                 // symbols in a 1 s I/NAV page part

// Satellite ids: a GPS satellite keeps its PRN, a Galileo satellite is 100 + SVID (E01 = 101).
constexpr int    kGalileoIdBase  = 100;
constexpr int    kGalileoMaxSv   = 36;                   // SVIDs with assigned codes

// GLONASS L1OF shares one code and separates satellites by frequency, so a GLONASS id is 200 + channel + 8: the
// frequency channels -7..+6 become 201..214. Satellites on opposite sides of the Earth share a channel.
constexpr int    kGlonassIdBase  = 200;
constexpr int    kGlonassMinChannel = -7, kGlonassMaxChannel = 6;
constexpr int    kGlonassCodeLength = 511;               // chips
constexpr double kGlonassChipRate = 0.511e6;             // chips/s
constexpr double kGlonassL1Base  = 1602.0e6;             // Hz, channel 0
constexpr double kGlonassL1Step  = 0.5625e6;             // Hz per channel

constexpr bool isGlonass(int id) { return id > kGlonassIdBase; }
constexpr bool isGalileo(int id) { return id > kGalileoIdBase && id <= kGlonassIdBase; }
constexpr int glonassId(int channel) { return kGlonassIdBase + channel + 8; }
constexpr int glonassChannel(int id) { return id - kGlonassIdBase - 8; }
constexpr double glonassFreq(int channel) { return kGlonassL1Base + channel * kGlonassL1Step; }
constexpr int svidOf(int id) { return isGlonass(id) ? glonassChannel(id) : isGalileo(id) ? id - kGalileoIdBase : id; }

} // namespace gnss
