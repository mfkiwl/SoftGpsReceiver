#include "Pvt.h"
#include "Constants.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace gnss {
namespace pvt {

namespace {

constexpr double c = kSpeedOfLight;

bool locate(const TrackingChannel& t, double R, int* m, double* frac)
{
    const auto& e = t.codeEpochSample;
    if (e.size() < 2 || R < e.front() || R >= e.back()) return false;
    const auto it = std::upper_bound(e.begin(), e.end(), R);
    const int idx = int(it - e.begin()) - 1;
    if (idx < 0 || idx + 1 >= int(e.size())) return false;
    *m = idx;
    *frac = (R - e[size_t(idx)]) / (e[size_t(idx + 1)] - e[size_t(idx)]);
    return true;
}

// Builds the measurements of one epoch (everything that does not depend on
// the receiver position).
void buildObservations(NavSolution& sol, const std::vector<int>& usable, const std::vector<TrackingChannel>& trk,
                       const std::vector<NavChannel>& nav, const ReceiverSettings& s, double minPli = -2.0)
{
    sol.obs.clear();
    for (int i : usable) {
        const TrackingChannel& t = trk[size_t(i)];
        const NavChannel& n = nav[size_t(i)];
        SatObservation o;
        o.channel = i;
        o.prn = t.prn;
        o.health = n.eph.svHealth;
        int m = 0;
        double fr = 0;
        if (!locate(t, sol.rxSample, &m, &fr)) {
            o.status = QStringLiteral("Outside tracked interval");
            sol.obs.push_back(o);
            continue;
        }
        o.txTime = n.txTimeAtMs(m + fr);
        o.cn0 = t.cn0[size_t(m)];
        o.pli = t.pli[size_t(m)];
        o.carrFreq = t.carrFreq[size_t(m)] + fr * (t.carrFreq[size_t(m + 1)] - t.carrFreq[size_t(m)]);
        o.adrCycles = t.adr[size_t(m)] + fr * (t.adr[size_t(m + 1)] - t.adr[size_t(m)]);
        const bool glo = isGlonass(t.prn);
        o.doppler = o.carrFreq - (glo ? s.glonassIf(glonassChannel(t.prn)) : s.ifFreq);
        if (glo) o.wavelength = c / glonassFreq(glonassChannel(t.prn));

        // GLONASS-only solutions run on GLONASS time, in seconds of the day, in place of GPS time of week
        double bias = 0;
        o.txTimeGps = glo ? orbit::glonassTime(n.glo, o.txTime, &bias) : orbit::correctedTransmitTime(n.eph, o.txTime, &bias);
        const SatState st = glo ? orbit::glonassState(n.glo, o.txTimeGps) : orbit::satState(n.eph, o.txTimeGps);
        o.satClock = st.clockBias;
        o.satClockRel = st.relativistic;
        o.tgd = n.eph.tgd;
        o.satClkDrift = st.clockDrift;
        o.satPosTx = st.pos;
        o.satVelTx = st.vel;
        o.rawPr = c * (sol.rxTime - o.txTime);
        o.prClockCorrected = o.rawPr + c * o.satClock;

        // The C/N0 estimate stays well up on pure noise for several seconds. Phase lock does not: with the
        // signal gone, this mean is close to zero. See kLivePhaseLock.
        double recentPli = 1.0;
        if (minPli > -1.0) {
            const int span = std::min(m + 1, 500 / t.periodMs);
            recentPli = 0;
            for (int k = m - span + 1; k <= m; ++k) recentPli += t.pli[size_t(k)];
            recentPli /= span;
        }
        if (o.cn0 < s.minCn0) o.status = QStringLiteral("C/N0 %1 dB-Hz below threshold").arg(o.cn0, 0, 'f', 1);
        else if (recentPli < minPli) o.status = QStringLiteral("Not phase locked");
        else if (!n.verifiedAt(m + fr)) o.status = QStringLiteral("Tracking not verified: no decoded data passed its check near this epoch");
        else if (o.health != 0) o.status = QStringLiteral("SV unhealthy (health %1)").arg(o.health);
        else { o.available = true; o.status = QStringLiteral("OK"); }
        sol.obs.push_back(o);
    }
}

// Evaluates corrections, residuals and design matrix at position x / bias b.
int evaluate(NavSolution& sol, const Vec3& x, double b, double isb, const ReceiverSettings& s, const GlobalNavData& g,
             const std::vector<char>& include, std::vector<std::array<double, 5>>& H,
             std::vector<double>& res, std::vector<double>& w, std::vector<int>& rows)
{
    H.clear(); res.clear(); w.clear(); rows.clear();
    const bool posValid = x.norm() > 6.0e6;
    Geodetic gx;
    if (posValid) gx = geo::ecefToGeodetic(x);
    for (int k = 0; k < int(sol.obs.size()); ++k) {
        SatObservation& o = sol.obs[size_t(k)];
        if (!o.available || !include[size_t(k)]) continue;
        const double tau = posValid ? (o.satPosTx - x).norm() / c : 0.075;
        const Vec3 rot = s.useEarthRotation ? geo::rotateEarth(o.satPosTx, tau) : o.satPosTx;
        o.satPos = rot;
        o.satVel = s.useEarthRotation ? geo::rotateEarth(o.satVelTx, tau) : o.satVelTx;
        const double r = (rot - x).norm();
        o.sagnac = r - (o.satPosTx - x).norm();
        o.tropo = 0;
        o.iono = 0;
        if (posValid) {
            geo::azimuthElevation(x, rot, &o.az, &o.el);
            if (s.useTropo) o.tropo = geo::tropoSaastamoinen(gx.lat, gx.h, o.el);
            if (s.useIono && g.ionoUtc.valid)
                o.iono = geo::ionoKlobuchar(g.ionoUtc.alpha, g.ionoUtc.beta, gx.lat, gx.lon, o.az, o.el, o.txTimeGps);
        }
        o.correctedPr = o.prClockCorrected - o.tropo - o.iono;
        o.geomRange = r;
        const double gal = isGalileo(o.prn) ? 1.0 : 0.0;
        o.residual = o.correctedPr - r - b - gal * isb;
        o.weight = (s.elevationWeighting && posValid) ? std::max(0.01, std::pow(std::sin(o.el), 2)) : 1.0;
        const Vec3 u = (x - rot) / r;
        H.push_back({u.x, u.y, u.z, 1.0, gal});
        res.push_back(o.residual);
        w.push_back(o.weight);
        rows.push_back(k);
    }
    return int(rows.size());
}

// Weighted least squares on the first 'states' columns of H
template <size_t Cols>
bool solveLsq(const std::vector<std::array<double, Cols>>& H, const std::vector<double>& y,
              const std::vector<double>& w, int states, double* dx, Matrix* Q = nullptr)
{
    Matrix N(states, states), Qm;
    double rhs[Cols] = {};
    for (size_t r = 0; r < H.size(); ++r)
        for (int i = 0; i < states; ++i) {
            rhs[i] += H[r][i] * w[r] * y[r];
            for (int j = 0; j < states; ++j) N(i, j) += H[r][i] * w[r] * H[r][j];
        }
    if (!N.inverse(&Qm)) return false;
    for (int i = 0; i < states; ++i) {
        dx[i] = 0;
        for (int j = 0; j < states; ++j) dx[i] += Qm(i, j) * rhs[j];
    }
    if (Q) *Q = Qm;
    return true;
}

// 'states' is 4, or 5 when GPS and Galileo are mixed: the extra unknown is the bias between the two
bool positionFix(NavSolution& sol, Vec3 x, double b, double isb, int states, const ReceiverSettings& s,
                 const GlobalNavData& g, bool record)
{
    std::vector<char> include(sol.obs.size(), 1);
    std::vector<std::array<double, 5>> H;
    std::vector<double> res, w;
    std::vector<int> rows;
    sol.iterations.clear();
    bool converged = false;
    int maskPass = 0;
    for (int iter = 0; iter < 20; ++iter) {
        const int n = evaluate(sol, x, b, isb, s, g, include, H, res, w, rows);
        if (n < states) {
            sol.message = QStringLiteral("Only %1 satellites available").arg(n);
            return false;
        }
        double dx[5] = {};
        if (!solveLsq(H, res, w, states, dx)) {
            sol.message = QStringLiteral("Singular geometry");
            return false;
        }
        LsqIteration li;
        li.iter = int(sol.iterations.size()) + 1;
        li.dpos = Vec3(dx[0], dx[1], dx[2]);
        li.dbias = dx[3];
        x += li.dpos;
        b += dx[3];
        isb += dx[4];
        li.pos = x;
        li.bias = b;
        li.isb = isb;
        li.stepNorm = li.dpos.norm();
        double ss = 0;
        for (double r : res) ss += r * r;
        li.rms = std::sqrt(ss / n);
        li.nSats = n;
        for (int k : rows) li.prns.push_back(sol.obs[size_t(k)].prn);
        if (record) { li.residuals = res; li.H = H; }
        sol.iterations.push_back(li);
        if (li.stepNorm < 1e-4 && std::fabs(dx[3]) < 1e-4) {
            // elevation mask (needs a converged position)
            bool changed = false;
            int remaining = 0;
            for (size_t k = 0; k < sol.obs.size(); ++k)
                if (sol.obs[k].available && include[k]) ++remaining;
            for (size_t k = 0; k < sol.obs.size() && maskPass == 0; ++k) {
                SatObservation& o = sol.obs[k];
                if (o.available && include[k] && o.el * 180.0 / kPi < s.elevationMask && remaining > states) {
                    include[k] = 0;
                    --remaining;
                    o.status = QStringLiteral("Below elevation mask (%1 deg)").arg(o.el * 180.0 / kPi, 0, 'f', 1);
                    changed = true;
                }
            }
            ++maskPass;
            if (!changed) { converged = true; break; }
        }
    }
    if (!converged) {
        sol.message = QStringLiteral("Least squares did not converge");
        return false;
    }
    // final evaluation, residuals and DOP
    evaluate(sol, x, b, isb, s, g, include, H, res, w, rows);
    for (auto& o : sol.obs) o.used = false;
    for (int k : rows) sol.obs[size_t(k)].used = true;
    std::vector<double> ones(H.size(), 1.0);
    double dummy[5];
    Matrix Q;
    if (!solveLsq(H, res, ones, states, dummy, &Q)) return false;
    sol.pos = x;
    sol.bias = b;
    sol.isb = isb;
    sol.nUsed = int(rows.size());
    double ss = 0;
    for (double r : res) ss += r * r;
    sol.rms = std::sqrt(ss / std::max<size_t>(1, res.size()));
    sol.llh = geo::ecefToGeodetic(x);
    const double sl = std::sin(sol.llh.lat), cl = std::cos(sol.llh.lat);
    const double so = std::sin(sol.llh.lon), co = std::cos(sol.llh.lon);
    const double R[4][4] = {{-so, co, 0, 0}, {-sl * co, -sl * so, cl, 0}, {cl * co, cl * so, sl, 0}, {0, 0, 0, 1}};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) {
            double v = 0;
            for (int k = 0; k < 4; ++k)
                for (int l = 0; l < 4; ++l) v += R[i][k] * Q(k, l) * R[j][l];
            sol.qEnu[i][j] = v;
        }
    sol.gdop = std::sqrt(Q(0, 0) + Q(1, 1) + Q(2, 2) + Q(3, 3));
    sol.pdop = std::sqrt(Q(0, 0) + Q(1, 1) + Q(2, 2));
    sol.hdop = std::sqrt(std::max(0.0, sol.qEnu[0][0] + sol.qEnu[1][1]));
    sol.vdop = std::sqrt(std::max(0.0, sol.qEnu[2][2]));
    sol.tdop = std::sqrt(std::max(0.0, Q(3, 3)));
    sol.valid = true;
    return true;
}

// Doppler velocity solution; returns residual RMS (or -1 on failure)
double velocityFix(NavSolution& sol, double sign, bool store)
{
    std::vector<std::array<double, 4>> H;
    std::vector<double> y, w;
    std::vector<int> rows;
    for (int k = 0; k < int(sol.obs.size()); ++k) {
        SatObservation& o = sol.obs[size_t(k)];
        if (!o.used) continue;
        const Vec3 d = o.satPos - sol.pos;
        const Vec3 u = d / d.norm();
        const double prRate = -o.wavelength * sign * o.doppler + c * o.satClkDrift;
        H.push_back({-u.x, -u.y, -u.z, 1.0});
        y.push_back(prRate - o.satVel.dot(u));
        w.push_back(1.0);
        rows.push_back(k);
    }
    if (rows.size() < 4) return -1;
    double x[4];
    if (!solveLsq(H, y, w, 4, x)) return -1;
    double ss = 0;
    for (size_t r = 0; r < rows.size(); ++r) {
        double pred = 0;
        for (int i = 0; i < 4; ++i) pred += H[r][i] * x[i];
        ss += (y[r] - pred) * (y[r] - pred);
    }
    const double rms = std::sqrt(ss / rows.size());
    if (store) {
        sol.vel = Vec3(x[0], x[1], x[2]);
        sol.drift = x[3];
        sol.velEnu = geo::ecefToEnu(sol.vel, sol.llh.lat, sol.llh.lon);
        sol.velRms = rms;
        sol.velValid = true;
        for (size_t r = 0; r < rows.size(); ++r) {
            SatObservation& o = sol.obs[size_t(rows[r])];
            const Vec3 d = o.satPos - sol.pos;
            const Vec3 u = d / d.norm();
            o.prRate = -o.wavelength * sign * o.doppler + c * o.satClkDrift;
            o.predRangeRate = (o.satVel - sol.vel).dot(u);
            o.predDoppler = -o.predRangeRate / o.wavelength;
            double pred = 0;
            for (int i = 0; i < 4; ++i) pred += H[r][i] * x[i];
            o.rateResidual = y[r] - pred;
        }
    }
    return rms;
}

// Mean position, scatter and clock statistics over out.sols
void statistics(PvtOutput& out, const ReceiverSettings& s, double sign)
{
    std::vector<const NavSolution*> valid;
    for (const auto& sol : out.sols)
        if (sol.valid) valid.push_back(&sol);
    out.nValid = int(valid.size());
    if (valid.empty()) {
        out.error = QStringLiteral("No valid position fixes");
        return;
    }
    Vec3 mean;
    for (const auto* v : valid) mean += v->pos;
    mean = mean / double(valid.size());
    out.meanPos = mean;
    out.meanLlh = geo::ecefToGeodetic(mean);
    out.utm = geo::toUtm(out.meanLlh.lat, out.meanLlh.lon);
    double se = 0, sn = 0, su = 0, sp = 0, sh = 0, ns = 0, md = 0;
    int nd = 0;
    std::vector<double> horiz, sph;
    out.meanIsb = 0;
    for (auto& sol : out.sols) {
        if (!sol.valid) continue;
        sol.enu = geo::ecefToEnu(sol.pos - mean, out.meanLlh.lat, out.meanLlh.lon);
        se += sol.enu.x * sol.enu.x; sn += sol.enu.y * sol.enu.y; su += sol.enu.z * sol.enu.z;
        horiz.push_back(std::hypot(sol.enu.x, sol.enu.y));
        sph.push_back(sol.enu.norm());
        sp += sol.pdop; sh += sol.hdop; ns += sol.nUsed;
        out.meanIsb += sol.isb;
        if (sol.velValid) { md += sol.drift; ++nd; }
    }
    const double N = double(valid.size());
    out.meanIsb /= N;
    out.stdE = std::sqrt(se / N); out.stdN = std::sqrt(sn / N); out.stdU = std::sqrt(su / N);
    out.drms2 = 2.0 * std::sqrt((se + sn) / N);
    std::sort(horiz.begin(), horiz.end());
    std::sort(sph.begin(), sph.end());
    out.cep50 = horiz[horiz.size() / 2];
    out.sep50 = sph[sph.size() / 2];
    out.meanPdop = sp / N; out.meanHdop = sh / N; out.meanSats = ns / N;
    out.meanDrift = nd ? md / nd : 0;
    out.commonFreqOffset = -out.meanDrift * sign / kL1Wavelength;
    out.effectiveIf = s.ifFreq + out.commonFreqOffset;
    // clock bias slope
    out.biasDrift = 0;
    if (valid.size() >= 2) {
        double st = 0, sb = 0, stt = 0, stb = 0;
        for (const auto* v : valid) {
            const double t = v->rxTime - valid.front()->rxTime;
            st += t; sb += v->bias; stt += t * t; stb += t * v->bias;
        }
        const double den = N * stt - st * st;
        if (den != 0) out.biasDrift = (N * stb - st * sb) / den;
    }
    out.firstGpsTime = valid.front()->gpsTime;
}

} // namespace

void LiveSolver::update(const std::vector<TrackingChannel>& trk, const std::vector<NavChannel>& nav,
                        const GlobalNavData& gnav, const ReceiverSettings& s, int maxFixes, PvtOutput& out)
{
    const double fs = s.samplingFreq;
    out.error.clear();
    out.log.clear();
    out.usableChannels.clear();
    out.nGps = out.nGalileo = 0;
    for (size_t i = 0; i < trk.size() && i < nav.size(); ++i) {
        const NavChannel& n = nav[i];
        if (!trk[i].valid || !n.timingValid || !n.eph.complete() || trk[i].codeEpochSample.size() < 2) continue;
        out.usableChannels.push_back(int(i));
        (isGalileo(n.prn) ? out.nGalileo : out.nGps)++;
        if (out.weekNumber < 0) out.weekNumber = n.eph.weekNumber % 1024;
    }
    if (out.usableChannels.size() < 4) {
        out.error = QStringLiteral("Need at least 4 satellites with ephemeris and timing (have %1)").arg(out.usableChannels.size());
        return;
    }

    // ---- epochs: as far as every usable channel has data. At the very first fix also go back over the
    // history that is already tracked, skipping each channel's pull-in.
    double R1 = std::numeric_limits<double>::infinity();
    double R0 = -std::numeric_limits<double>::infinity();
    for (int i : out.usableChannels) {
        const auto& e = trk[size_t(i)].codeEpochSample;
        const int k = std::clamp(s.bitSyncStartMs / trk[size_t(i)].periodMs, 0, int(e.size()) - 2);
        R0 = std::max(R0, e[size_t(k)]);
        R1 = std::min(R1, e[e.size() - 2]);
    }
    if (!haveEpoch_) {
        nextR_ = std::ceil(R0);
        haveEpoch_ = true;
    }
    const double step = std::max(1, s.navSolPeriodMs) * fs / 1000.0;
    signKnown_ = signKnown_ || s.dopplerSign != DopplerSign::Auto;
    if (s.dopplerSign != DopplerSign::Auto) sign_ = s.dopplerSignValue();
    out.signAuto = s.dopplerSign == DopplerSign::Auto;

    for (; nextR_ < R1; nextR_ += step) {
        NavSolution sol;
        sol.rxSample = nextR_;
        // Which satellites are usable changes from epoch to epoch here, and with them the unknowns
        const auto statesOf = [](const NavSolution& n) {
            int nGps = 0, nGal = 0;
            for (const auto& o : n.obs)
                if (o.available) (isGalileo(o.prn) ? nGal : nGps)++;
            return nGps > 0 && nGal > 0 ? 5 : 4;
        };
        if (!haveRef_) {
            buildObservations(sol, out.usableChannels, trk, nav, s, kLivePhaseLock);
            double maxTx = -1e300;
            for (const auto& o : sol.obs)
                if (o.available) maxTx = std::max(maxTx, o.txTime);
            if (maxTx < -1e299) continue;
            sol.rxTime = maxTx + 0.0688;
            buildObservations(sol, out.usableChannels, trk, nav, s, kLivePhaseLock);
            if (!positionFix(sol, Vec3(), 0.0, 0.0, statesOf(sol), s, gnav, false)) continue;
            trxRef_ = sol.rxTime - sol.bias / c;
            Rref_ = nextR_;
            haveRef_ = true;
        }
        sol = NavSolution();
        sol.index = nextIndex_++;
        sol.rxSample = nextR_;
        sol.rxTime = trxRef_ + (nextR_ - Rref_) / fs;
        buildObservations(sol, out.usableChannels, trk, nav, s, kLivePhaseLock);
        if (positionFix(sol, havePrev_ ? xPrev_ : Vec3(), havePrev_ ? bPrev_ : 0.0, isbPrev_, statesOf(sol), s, gnav, true)) {
            xPrev_ = sol.pos;
            bPrev_ = sol.bias;
            isbPrev_ = sol.isb;
            havePrev_ = true;
            sol.gpsTime = sol.rxTime - sol.bias / c;
            if (!signKnown_ && sol.nUsed >= 5) {
                out.signRms[0] = velocityFix(sol, +1.0, false);
                out.signRms[1] = velocityFix(sol, -1.0, false);
                sign_ = (out.signRms[1] >= 0 && out.signRms[1] < out.signRms[0]) ? -1.0 : 1.0;
                signKnown_ = true;
            }
            if (signKnown_) velocityFix(sol, sign_, true);
        }
        out.sols.push_back(std::move(sol));
    }
    out.dopplerSign = int(sign_);
    const size_t keep = size_t(std::max(1, maxFixes));
    if (out.sols.size() > keep + keep / 8 + 1) out.sols.erase(out.sols.begin(), out.sols.end() - keep);
    statistics(out, s, sign_);
}

PvtOutput solve(const std::vector<TrackingChannel>& trk, const std::vector<NavChannel>& nav,
                const GlobalNavData& gnav, const ReceiverSettings& s)
{
    PvtOutput out;
    const double fs = s.samplingFreq;
    for (size_t i = 0; i < trk.size() && i < nav.size(); ++i) {
        const NavChannel& n = nav[i];
        if (!trk[i].valid) continue;
        if (!n.timingValid) { out.log << QStringLiteral("%1: no time reference (no verified subframe or page)").arg(satName(n.prn)); continue; }
        if (!n.eph.complete()) { out.log << QStringLiteral("%1: ephemeris incomplete").arg(satName(n.prn)); continue; }
        if (!n.eph.consistent()) out.log << QStringLiteral("%1: warning - issue of data mismatch").arg(satName(n.prn));
        out.usableChannels.push_back(int(i));
        (isGalileo(n.prn) ? out.nGalileo : out.nGps)++;
        if (out.weekNumber < 0) out.weekNumber = n.eph.weekNumber % 1024;
    }
    const int states = out.nGps > 0 && out.nGalileo > 0 ? 5 : 4;
    if (int(out.usableChannels.size()) < states) {
        out.error = QStringLiteral("Need at least %1 satellites with ephemeris and timing (have %2)").arg(states).arg(out.usableChannels.size());
        return out;
    }

    // ---- measurement epochs common to all usable channels
    double R0 = -std::numeric_limits<double>::infinity();
    double R1 = std::numeric_limits<double>::infinity();
    for (int i : out.usableChannels) {
        const auto& e = trk[size_t(i)].codeEpochSample;
        const int k = std::clamp(s.bitSyncStartMs / trk[size_t(i)].periodMs, 0, int(e.size()) - 2);
        R0 = std::max(R0, e[size_t(k)]);
        R1 = std::min(R1, e[e.size() - 2]);
    }
    const double step = std::max(1, s.navSolPeriodMs) * fs / 1000.0;
    std::vector<double> epochs;
    for (double R = std::ceil(R0); R < R1; R += step) epochs.push_back(R);
    if (epochs.empty()) {
        out.error = QStringLiteral("No common measurement interval");
        return out;
    }
    out.log << QStringLiteral("%1 measurement epochs every %2 ms using %3 satellites")
                   .arg(epochs.size()).arg(s.navSolPeriodMs).arg(out.usableChannels.size());

    // ---- establish the receiver time scale from the first solvable epoch
    double trxRef = 0, Rref = 0;
    bool haveRef = false;
    for (size_t e = 0; e < epochs.size() && !haveRef; ++e) {
        NavSolution sol;
        sol.rxSample = epochs[e];
        sol.rxTime = 0;
        buildObservations(sol, out.usableChannels, trk, nav, s);
        double maxTx = -1e300;
        for (const auto& o : sol.obs)
            if (o.available) maxTx = std::max(maxTx, o.txTime);
        if (maxTx < -1e299) continue;
        sol.rxTime = maxTx + 0.0688;   // nominal ~69 ms travel time for the nearest satellite
        buildObservations(sol, out.usableChannels, trk, nav, s);
        if (positionFix(sol, Vec3(), 0.0, 0.0, states, s, gnav, false)) {
            trxRef = sol.rxTime - sol.bias / c;
            Rref = epochs[e];
            haveRef = true;
            out.log << QStringLiteral("Receiver clock initialised at epoch %1: initial bias %2 m removed")
                           .arg(e).arg(sol.bias, 0, 'f', 1);
        }
    }
    if (!haveRef) {
        out.error = QStringLiteral("Could not compute an initial position");
        return out;
    }

    // ---- all epochs
    Vec3 xPrev;
    double bPrev = 0, isbPrev = 0;
    bool havePrev = false;
    bool signKnown = s.dopplerSign != DopplerSign::Auto;
    double sign = s.dopplerSignValue();
    out.signAuto = !signKnown;
    for (size_t e = 0; e < epochs.size(); ++e) {
        NavSolution sol;
        sol.index = int(e);
        sol.rxSample = epochs[e];
        sol.rxTime = trxRef + (epochs[e] - Rref) / fs;
        buildObservations(sol, out.usableChannels, trk, nav, s);
        const bool ok = positionFix(sol, havePrev ? xPrev : Vec3(), havePrev ? bPrev : 0.0, isbPrev, states, s, gnav, true);
        if (ok) {
            xPrev = sol.pos;
            bPrev = sol.bias;
            isbPrev = sol.isb;
            havePrev = true;
            sol.gpsTime = sol.rxTime - sol.bias / c;
            if (!signKnown && sol.nUsed >= 5) {
                out.signRms[0] = velocityFix(sol, +1.0, false);
                out.signRms[1] = velocityFix(sol, -1.0, false);
                sign = (out.signRms[1] >= 0 && out.signRms[1] < out.signRms[0]) ? -1.0 : 1.0;
                signKnown = true;
                out.log << QStringLiteral("Doppler sign auto-detected: %1 (velocity residual RMS %2 m/s normal vs %3 m/s inverted)")
                               .arg(sign > 0 ? "normal" : "inverted spectrum")
                               .arg(out.signRms[0], 0, 'f', 2).arg(out.signRms[1], 0, 'f', 2);
            }
            if (signKnown) velocityFix(sol, sign, true);
        }
        out.sols.push_back(sol);
    }
    out.dopplerSign = int(sign);

    statistics(out, s, sign);
    return out;
}

} // namespace pvt
} // namespace gnss
