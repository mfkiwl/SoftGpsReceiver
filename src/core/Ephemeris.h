#pragma once

#include "MathUtil.h"
#include "NavMessage.h"

#include <QString>
#include <vector>

namespace gnss {

// One line of the step-by-step IS-GPS-200 Table 20-IV computation.
struct KeplerStep
{
    QString symbol;
    QString description;
    double value;
    QString units;
};

struct SatState
{
    Vec3 pos;                // ECEF at time t (m)
    Vec3 vel;                // ECEF velocity (m/s)
    double clockBias = 0;    // a_f0 + a_f1 dt + a_f2 dt^2 + relativistic - T_GD (s)
    double clockDrift = 0;   // s/s
    double relativistic = 0; // s
    double tk = 0;           // time from ephemeris reference epoch (s)
    double Ek = 0;           // eccentric anomaly (rad)
    int keplerIterations = 0;
};

namespace orbit {

// Satellite position/velocity/clock from the broadcast ephemeris at GPS time t (s of week).
SatState satState(const Ephemeris& e, double t, std::vector<KeplerStep>* steps = nullptr);

// Satellite clock correction for a satellite-time tag t_sv; returns the corrected GPS time.
double correctedTransmitTime(const Ephemeris& e, double tsv, double* clockBias = nullptr);

// GLONASS: state at system time t (s of day) by integrating the broadcast state vector from tb, in PZ-90, which is
// taken as equal to WGS-84 (they differ by decimetres).
SatState glonassState(const GlonassEphemeris& e, double t);
// System time from a satellite-time tag; *clockBias is tsv minus that time
double glonassTime(const GlonassEphemeris& e, double tsv, double* clockBias = nullptr);

// Almanac-based position (no harmonic corrections) for sky-plot prediction.
Vec3 almanacPosition(const AlmanacEntry& a, double t);

} // namespace orbit
} // namespace gnss
