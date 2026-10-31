#include "Ephemeris.h"
#include "Constants.h"

#include <cmath>

namespace gnss {
namespace orbit {

SatState satState(const Ephemeris& e, double t, std::vector<KeplerStep>* steps)
{
    SatState st;
    auto log = [&](const char* sym, const char* desc, double v, const char* units) {
        if (steps) steps->push_back({QString::fromUtf8(sym), QString::fromUtf8(desc), v, QString::fromUtf8(units)});
    };

    const bool gal = isGalileo(e.prn);
    const double mu = gal ? kMuGal : kMu;
    const double F = gal ? kRelFGal : kRelF;

    const double A = e.sqrtA * e.sqrtA;
    log("A", "Semi-major axis  A = (sqrt A)^2", A, "m");
    const double n0 = std::sqrt(mu / (A * A * A));
    log("n0", "Computed mean motion  n0 = sqrt(mu / A^3)", n0, "rad/s");
    const double tk = wrapHalfWeek(t - e.toe);
    log("t", "GPS time of transmission", t, "s");
    log("tk", "Time from ephemeris reference epoch  tk = t - toe", tk, "s");
    const double n = n0 + e.deltaN;
    log("n", "Corrected mean motion  n = n0 + delta n", n, "rad/s");
    const double M = e.m0 + n * tk;
    log("Mk", "Mean anomaly  Mk = M0 + n tk", M, "rad");

    double E = M;
    int it = 0;
    for (; it < 30; ++it) {
        const double En = M + e.e * std::sin(E);
        if (steps) steps->push_back({QStringLiteral("E%1").arg(it + 1),
                                     QStringLiteral("Kepler iteration %1: E = Mk + e sin E").arg(it + 1), En, "rad"});
        if (std::fabs(En - E) < 1e-13) { E = En; ++it; break; }
        E = En;
    }
    st.Ek = E;
    st.keplerIterations = it;
    log("Ek", "Eccentric anomaly (converged)", E, "rad");

    const double sinE = std::sin(E), cosE = std::cos(E);
    const double nu = std::atan2(std::sqrt(1.0 - e.e * e.e) * sinE, cosE - e.e);
    log("vk", "True anomaly  vk = atan2(sqrt(1-e^2) sin E, cos E - e)", nu, "rad");
    const double phi = nu + e.omega;
    log("Phik", "Argument of latitude  Phik = vk + omega", phi, "rad");
    const double s2 = std::sin(2 * phi), c2 = std::cos(2 * phi);
    const double du = e.cus * s2 + e.cuc * c2;
    const double dr = e.crs * s2 + e.crc * c2;
    const double di = e.cis * s2 + e.cic * c2;
    log("duk", "Argument of latitude correction  Cus sin2Phi + Cuc cos2Phi", du, "rad");
    log("drk", "Radius correction  Crs sin2Phi + Crc cos2Phi", dr, "m");
    log("dik", "Inclination correction  Cis sin2Phi + Cic cos2Phi", di, "rad");
    const double u = phi + du;
    const double r = A * (1.0 - e.e * cosE) + dr;
    const double inc = e.i0 + di + e.idot * tk;
    log("uk", "Corrected argument of latitude", u, "rad");
    log("rk", "Corrected radius  A(1 - e cos E) + drk", r, "m");
    log("ik", "Corrected inclination  i0 + dik + IDOT tk", inc, "rad");
    const double xp = r * std::cos(u), yp = r * std::sin(u);
    log("xk'", "Position in orbital plane x", xp, "m");
    log("yk'", "Position in orbital plane y", yp, "m");
    const double Om = e.omega0 + (e.omegaDot - kOmegaEDot) * tk - kOmegaEDot * e.toe;
    log("Omegak", "Corrected longitude of ascending node", Om, "rad");
    const double cO = std::cos(Om), sO = std::sin(Om), ci = std::cos(inc), si = std::sin(inc);
    st.pos = Vec3(xp * cO - yp * ci * sO, xp * sO + yp * ci * cO, yp * si);
    log("X", "ECEF X = x' cos Omega - y' cos i sin Omega", st.pos.x, "m");
    log("Y", "ECEF Y = x' sin Omega + y' cos i cos Omega", st.pos.y, "m");
    log("Z", "ECEF Z = y' sin i", st.pos.z, "m");
    log("|r|", "Geocentric distance", st.pos.norm(), "m");

    // velocity (time derivatives of the above)
    const double Edot = n / (1.0 - e.e * cosE);
    const double nudot = Edot * std::sqrt(1.0 - e.e * e.e) / (1.0 - e.e * cosE);
    const double udot = nudot * (1.0 + 2.0 * (e.cus * c2 - e.cuc * s2));
    const double rdot = A * e.e * sinE * Edot + 2.0 * nudot * (e.crs * c2 - e.crc * s2);
    const double idot = e.idot + 2.0 * nudot * (e.cis * c2 - e.cic * s2);
    const double Omdot = e.omegaDot - kOmegaEDot;
    const double xpd = rdot * std::cos(u) - r * udot * std::sin(u);
    const double ypd = rdot * std::sin(u) + r * udot * std::cos(u);
    st.vel.x = xpd * cO - ypd * ci * sO + yp * si * sO * idot - st.pos.y * Omdot;
    st.vel.y = xpd * sO + ypd * ci * cO - yp * si * idot * cO + st.pos.x * Omdot;
    st.vel.z = ypd * si + yp * ci * idot;
    log("Vx", "ECEF velocity X", st.vel.x, "m/s");
    log("Vy", "ECEF velocity Y", st.vel.y, "m/s");
    log("Vz", "ECEF velocity Z", st.vel.z, "m/s");

    // clock
    const double dt = wrapHalfWeek(t - e.toc);
    st.relativistic = F * e.e * e.sqrtA * sinE;
    st.clockBias = e.af0 + e.af1 * dt + e.af2 * dt * dt + st.relativistic - e.tgd;
    st.clockDrift = e.af1 + 2.0 * e.af2 * dt + F * e.e * e.sqrtA * cosE * Edot;
    st.tk = tk;
    log("dtr", "Relativistic correction  F e sqrt(A) sin E", st.relativistic, "s");
    log("dtsv", "SV clock correction  af0 + af1 dt + af2 dt^2 + dtr - TGD", st.clockBias, "s");
    log("c*dtsv", "SV clock correction as range", st.clockBias * kSpeedOfLight, "m");
    return st;
}

namespace {

void glonassRates(const double y[6], const double acc[3], double out[6])
{
    constexpr double mu = 398600.4418e9, ae = 6378136.0, j2 = 1.08262575e-3, w = 7.292115e-5;
    const double r2 = y[0] * y[0] + y[1] * y[1] + y[2] * y[2], r = std::sqrt(r2), r3 = r2 * r;
    const double a = 1.5 * j2 * mu * ae * ae / (r2 * r3);
    const double b = 5.0 * y[2] * y[2] / r2;
    const double k = -mu / r3 - a * (1.0 - b);
    out[0] = y[3]; out[1] = y[4]; out[2] = y[5];
    out[3] = (k + w * w) * y[0] + 2.0 * w * y[4] + acc[0];
    out[4] = (k + w * w) * y[1] - 2.0 * w * y[3] + acc[1];
    out[5] = (k - 2.0 * a) * y[2] + acc[2];
}

} // namespace

SatState glonassState(const GlonassEphemeris& e, double t)
{
    double y[6] = {e.pos[0], e.pos[1], e.pos[2], e.vel[0], e.vel[1], e.vel[2]};
    const double span = t - e.tb;
    const int n = std::max(1, int(std::ceil(std::fabs(span) / 30.0)));
    const double h = span / n;
    for (int s = 0; s < n; ++s) {                            // fourth-order Runge-Kutta
        double k1[6], k2[6], k3[6], k4[6], w[6];
        glonassRates(y, e.acc, k1);
        for (int i = 0; i < 6; ++i) w[i] = y[i] + 0.5 * h * k1[i];
        glonassRates(w, e.acc, k2);
        for (int i = 0; i < 6; ++i) w[i] = y[i] + 0.5 * h * k2[i];
        glonassRates(w, e.acc, k3);
        for (int i = 0; i < 6; ++i) w[i] = y[i] + h * k3[i];
        glonassRates(w, e.acc, k4);
        for (int i = 0; i < 6; ++i) y[i] += h / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
    }
    SatState st;
    st.pos = Vec3(y[0], y[1], y[2]);
    st.vel = Vec3(y[3], y[4], y[5]);
    st.tk = span;
    st.clockBias = -e.tauN + e.gammaN * span;
    st.clockDrift = e.gammaN;
    return st;
}

double glonassTime(const GlonassEphemeris& e, double tsv, double* clockBias)
{
    const double bias = -e.tauN + e.gammaN * (tsv - e.tb);
    if (clockBias) *clockBias = bias;
    return tsv - bias;
}

double correctedTransmitTime(const Ephemeris& e, double tsv, double* clockBias)
{
    double t = tsv;
    double bias = 0;
    for (int i = 0; i < 3; ++i) {
        const SatState s = satState(e, t);
        bias = s.clockBias;
        t = tsv - bias;
    }
    if (clockBias) *clockBias = bias;
    return t;
}

Vec3 almanacPosition(const AlmanacEntry& a, double t)
{
    const double A = a.sqrtA * a.sqrtA;
    if (A <= 0) return Vec3();
    const double n = std::sqrt(kMu / (A * A * A));
    const double tk = wrapHalfWeek(t - a.toa);
    const double M = a.m0 + n * tk;
    double E = M;
    for (int i = 0; i < 20; ++i) E = M + a.e * std::sin(E);
    const double nu = std::atan2(std::sqrt(1 - a.e * a.e) * std::sin(E), std::cos(E) - a.e);
    const double u = nu + a.omega;
    const double r = A * (1 - a.e * std::cos(E));
    const double inc = 0.30 * kGpsPi + a.deltaI;
    const double Om = a.omega0 + (a.omegaDot - kOmegaEDot) * tk - kOmegaEDot * a.toa;
    const double xp = r * std::cos(u), yp = r * std::sin(u);
    return Vec3(xp * std::cos(Om) - yp * std::cos(inc) * std::sin(Om),
                xp * std::sin(Om) + yp * std::cos(inc) * std::cos(Om),
                yp * std::sin(inc));
}

} // namespace orbit
} // namespace gnss
