#include "Tracking.h"
#include "CaCode.h"
#include "Constants.h"
#include "GalileoCodes.h"
#include "GlonassCode.h"

#include <QElapsedTimer>

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace gnss {

namespace {

constexpr int kLutBits = 10;
constexpr int kLutSize = 1 << kLutBits;

struct CarrierLut
{
    float c[kLutSize];
    float s[kLutSize];
    CarrierLut()
    {
        for (int i = 0; i < kLutSize; ++i) {
            const double a = kTwoPi * (i + 0.5) / kLutSize;
            c[i] = float(std::cos(a));
            s[i] = float(std::sin(a));
        }
    }
};

const CarrierLut& lut()
{
    static const CarrierLut l;
    return l;
}

inline double frac(double x) { return x - std::floor(x); }

inline uint32_t phaseToFixed(double cycles)
{
    return uint32_t(uint64_t(frac(cycles) * 4294967296.0));
}

// Early/prompt/late correlation over one block. The carrier NCO is a 32-bit
// phase accumulator addressing a sine/cosine table; the code NCO is a
// double-precision phase, sampled with a padded code table so that the
// early and late replicas can run off either end of the code period.
// With Data set, a second table supplies an extra prompt correlator (E1-B), and two more correlators watch the
// pilot half a chip, one table entry, either side of prompt.
template <bool Complex, bool Data>
void correlate(const float* sI, const float* sQ, int n, double remCode, double codeStep, double d,
               const float* codeP, const float* dataP, double remCarr, double carrStep, float acc[12])
{
    const CarrierLut& L = lut();
    uint32_t ph = phaseToFixed(remCarr);
    const uint32_t dph = uint32_t(uint64_t(std::llround(frac(carrStep) * 4294967296.0)));
    float ie = 0, qe = 0, ip = 0, qp = 0, il = 0, ql = 0, ib = 0, qb = 0, ive = 0, qve = 0, ivl = 0, qvl = 0;
    for (int i = 0; i < n; ++i) {
        const double cp = remCode + double(i) * codeStep;
        const float ce = codeP[int(cp + d + 1.0)];
        const float cpv = codeP[int(cp + 1.0)];
        const float cl = codeP[int(cp - d + 1.0)];
        const unsigned idx = ph >> (32 - kLutBits);
        ph += dph;
        const float c = L.c[idx], sn = L.s[idx];
        float bi, bq;
        if (Complex) {
            bi = sI[i] * c + sQ[i] * sn;
            bq = sQ[i] * c - sI[i] * sn;
        } else {
            bi = sI[i] * c;
            bq = -sI[i] * sn;
        }
        ie += ce * bi;  qe += ce * bq;
        ip += cpv * bi; qp += cpv * bq;
        il += cl * bi;  ql += cl * bq;
        if (Data) {
            const float db = dataP[int(cp + 1.0)];
            ib += db * bi; qb += db * bq;
            const float cve = codeP[int(cp + 2.0)], cvl = codeP[int(cp)];
            ive += cve * bi; qve += cve * bq;
            ivl += cvl * bi; qvl += cvl * bq;
        }
    }
    acc[0] = ie; acc[1] = qe; acc[2] = ip; acc[3] = qp; acc[4] = il; acc[5] = ql; acc[6] = ib; acc[7] = qb;
    acc[8] = ive; acc[9] = qve; acc[10] = ivl; acc[11] = qvl;
}

// Replica table used by the correlators: one value per chip for C/A, two per chip for BOC(1,1)
std::vector<int8_t> replicaTable(int prn, bool dataComponent)
{
    if (isGlonass(prn)) return GlonassCode::chips();
    if (!isGalileo(prn)) return CaCode::chips(prn);
    return GalileoCode::boc11(dataComponent ? GalileoCode::e1b(svidOf(prn)) : GalileoCode::e1c(svidOf(prn)));
}

std::vector<float> paddedTable(const std::vector<int8_t>& table)
{
    const int len = int(table.size());
    std::vector<float> p(len + 3);
    for (int k = 0; k < len + 3; ++k) p[k] = table[(k - 1 + len) % len];
    return p;
}

inline double safeAtan(double num, double den)
{
    if (den == 0.0) return num >= 0 ? kPi / 2 : -kPi / 2;
    return std::atan(num / den);
}

} // namespace

void TrackingChannel::reserve(int n)
{
    blockStart.reserve(n); blockSize.reserve(n); codeEpochSample.reserve(n);
    carrFreq.reserve(n); codeFreq.reserve(n); remCarrPhase.reserve(n); remCodePhase.reserve(n);
    IE.reserve(n); IP.reserve(n); IL.reserve(n); QE.reserve(n); QP.reserve(n); QL.reserve(n);
    if (isGalileo(prn)) { IB.reserve(n); QB.reserve(n); }
    pllDisc.reserve(n); fllDisc.reserve(n); dllDisc.reserve(n); carrNco.reserve(n); codeNco.reserve(n);
    cn0.reserve(n); pli.reserve(n); adr.reserve(n); lock.reserve(n);
}

void TrackingChannel::trimFront(int n)
{
    n = std::min(n, int(IP.size()));
    if (n <= 0) return;
    const auto cut = [n](auto& v) { if (!v.empty()) v.erase(v.begin(), v.begin() + n); };
    cut(blockStart); cut(blockSize); cut(codeEpochSample);
    cut(carrFreq); cut(codeFreq); cut(remCarrPhase); cut(remCodePhase);
    cut(IE); cut(IP); cut(IL); cut(QE); cut(QP); cut(QL); cut(IB); cut(QB);
    cut(pllDisc); cut(fllDisc); cut(dllDisc); cut(carrNco); cut(codeNco);
    cut(cn0); cut(pli); cut(adr); cut(lock);
}

void TrackingChannel::summarise(const ReceiverSettings& s)
{
    numMs = int(IP.size());
    valid = numMs > 0;
    const int from = std::min(numMs, std::max(0, s.bitSyncStartMs / periodMs));
    double sc = 0;
    int nl = 0, cnt = 0;
    for (int i = from; i < numMs; ++i) { sc += cn0[i]; nl += lock[i]; ++cnt; }
    meanCn0 = cnt ? sc / cnt : 0;
    lockFraction = cnt ? double(nl) / cnt : 0;
    finalDoppler = numMs ? carrFreq.back() - s.ifFreq : 0;
}

ChannelTracker::ChannelTracker(const ReceiverSettings& s, const AcqResult& acq, qint64 startSample, bool complexInput,
                               TrackingChannel& ch)
    : ch_(ch), fs_(s.samplingFreq), ifFreq_(s.ifFreq), cpx_(complexInput), fllAssistMs_(s.fllAssistMs), pos_(startSample)
{
    ch.prn = acq.prn;
    ch.startSample = startSample;
    ch.initCarrFreq = acq.fineFreq;
    gal_ = isGalileo(ch.prn);
    periodMs_ = gal_ ? kE1CodeMs : 1;
    const bool glo = isGlonass(ch.prn);
    codeLength_ = gal_ ? kE1CodeLength : glo ? kGlonassCodeLength : kCaCodeLength;
    chipRate_ = glo ? kGlonassChipRate : kCaChipRate;
    if (glo) {
        ifFreq_ = s.glonassIf(glonassChannel(ch.prn));
        carrToCode_ = glonassFreq(glonassChannel(ch.prn)) / kGlonassChipRate;
    }
    tableScale_ = gal_ ? 2.0 : 1.0;                       // table entries per chip
    ch.periodMs = periodMs_;
    ch.codeLength = codeLength_;

    // padded code: codeP[k] = table[(k-1) mod length], index = floor(phase) + 1
    codeP_ = paddedTable(replicaTable(ch.prn, false));
    dataP_ = gal_ ? paddedTable(replicaTable(ch.prn, true)) : std::vector<float>();

    // ---- loop filter coefficients (2nd-order PLL + 1st-order FLL assist, 2nd-order DLL)
    zp_ = s.pllDamping;
    wnp_ = s.pllBandwidth * 8.0 * zp_ / (4.0 * zp_ * zp_ + 1.0);
    w0f_ = s.fllBandwidth / 0.25;
    zd_ = s.dllDamping;
    wnd_ = s.dllBandwidth * 8.0 * zd_ / (4.0 * zd_ * zd_ + 1.0);
    ch.pllWn = wnp_; ch.pllK1 = 2.0 * zp_ * wnp_; ch.pllK2 = wnp_ * wnp_; ch.fllW0 = w0f_;
    ch.dllWn = wnd_; ch.dllK1 = 2.0 * zd_ * wnd_; ch.dllK2 = wnd_ * wnd_;
    d_ = gal_ ? s.galDllSpacing : s.dllSpacing;
    dopSign_ = s.dopplerSignValue();
    // Discriminator gain. C/A: triangle R = 1 - |t|; BOC(1,1) near the peak: R = 1 - 3|t|
    dllGain_ = gal_ ? (1.0 - 3.0 * d_) / 3.0 : 1.0 - d_;

    carrBase_ = acq.fineFreq;
    carrFreq_ = carrBase_;
    // Code Doppler reaches about 2 chips/s. A narrow DLL would lag it by more than the width of the
    // BOC(1,1) peak, so E1 always uses carrier aiding, from the first epoch on.
    aiding_ = gal_ || s.carrierAiding;
    aidRef_ = ifFreq_ + (gal_ ? acq.commonDoppler : 0.0);
    codeFreq_ = chipRate_ + (gal_ ? dopSign_ * (carrFreq_ - aidRef_) / carrToCode_ : 0.0);
    cn0Win_ = std::max(20, s.cn0WindowMs / periodMs_);
    pwr_.assign(cn0Win_, 0.0);
    bufI_.resize(size_t(periodMs_ * fs_ / 1000.0 * 1.2) + 64);
    bufQ_.resize(bufI_.size());
}

bool ChannelTracker::step(const SignalSource& src, qint64 availableSamples)
{
    TrackingChannel& ch = ch_;
    const double codeStep = codeFreq_ / fs_;
    const int blk = int(std::ceil((double(codeLength_) - remCode_) / codeStep));
    if (blk <= 0 || pos_ + blk > availableSamples) return false;
    if (size_t(blk) > bufI_.size()) { bufI_.resize(blk); bufQ_.resize(blk); }
    src.read(pos_, blk, bufI_.data(), cpx_ ? bufQ_.data() : nullptr);

    const double carrStep = carrFreq_ / fs_;
    const double tRem = remCode_ * tableScale_, tStep = codeStep * tableScale_, tD = d_ * tableScale_;
    float acc[12];
    if (gal_) {
        if (cpx_) correlate<true, true>(bufI_.data(), bufQ_.data(), blk, tRem, tStep, tD, codeP_.data(), dataP_.data(), remCarr_, carrStep, acc);
        else      correlate<false, true>(bufI_.data(), nullptr, blk, tRem, tStep, tD, codeP_.data(), dataP_.data(), remCarr_, carrStep, acc);
        ch.IB.push_back(acc[6]);
        ch.QB.push_back(acc[7]);
    } else {
        if (cpx_) correlate<true, false>(bufI_.data(), bufQ_.data(), blk, tRem, tStep, tD, codeP_.data(), nullptr, remCarr_, carrStep, acc);
        else      correlate<false, false>(bufI_.data(), nullptr, blk, tRem, tStep, tD, codeP_.data(), nullptr, remCarr_, carrStep, acc);
    }
    const float IE = acc[0], QE = acc[1], IP = acc[2], QP = acc[3], IL = acc[4], QL = acc[5];
    const double T = double(blk) / fs_;

    ch.blockStart.push_back(pos_);
    ch.blockSize.push_back(blk);
    ch.codeEpochSample.push_back(double(pos_) - remCode_ / codeStep);
    ch.remCodePhase.push_back(remCode_);
    ch.remCarrPhase.push_back(remCarr_);
    ch.carrFreq.push_back(carrFreq_);
    ch.codeFreq.push_back(codeFreq_);
    ch.adr.push_back(adr_);
    ch.IE.push_back(IE); ch.IP.push_back(IP); ch.IL.push_back(IL);
    ch.QE.push_back(QE); ch.QP.push_back(QP); ch.QL.push_back(QL);

    // advance NCO phases to the start of the next block
    remCode_ = remCode_ + blk * codeStep - codeLength_;
    remCarr_ = frac(remCarr_ + carrFreq_ * blk / fs_);
    adr_ += (carrFreq_ - ifFreq_) * T;
    pos_ += blk;

    // A block of nothing but zeros is a gap in a live stream (samples lost on the network). The discriminators
    // would read it as a large error and throw the loops out of lock, so hold the NCOs and coast across it.
    const bool gap = IP == 0.0f && QP == 0.0f && IE == 0.0f && IL == 0.0f;

    // ---- carrier loop: Costas PLL with FLL assist during pull-in
    const double ep = gap ? 0.0 : safeAtan(QP, IP);          // rad
    double efHz = 0.0;
    if (epoch_ > 0 && !gap && (prevI_ != 0.0f || prevQ_ != 0.0f)) {
        const double cross = double(prevI_) * QP - double(prevQ_) * IP;
        const double dot = double(prevI_) * IP + double(prevQ_) * QP;
        efHz = safeAtan(cross, dot) / (kTwoPi * T);
    }
    prevI_ = IP; prevQ_ = QP;
    const bool fllOn = epoch_ * periodMs_ < fllAssistMs_;
    carrInteg_ += T * (wnp_ * wnp_ * ep + (fllOn ? w0f_ * kTwoPi * efHz : 0.0));
    const double ncoHz = (carrInteg_ + 2.0 * zp_ * wnp_ * ep) / kTwoPi;
    carrFreq_ = carrBase_ + ncoHz;

    // ---- code loop: normalised early-minus-late envelope DLL
    const double E = std::hypot(double(IE), double(QE));
    const double Lm = std::hypot(double(IL), double(QL));
    const double ed = (E + Lm) > 0 ? dllGain_ * (E - Lm) / (E + Lm) : 0.0;  // chips
    codeInteg_ += T * wnd_ * wnd_ * ed;
    const double codeNco = codeInteg_ + 2.0 * zd_ * wnd_ * ed;              // chips/s
    const double aid = aiding_ ? dopSign_ * (carrFreq_ - aidRef_) / carrToCode_ : 0.0;
    codeFreq_ = chipRate_ + codeNco + aid;

    // ---- BOC(1,1) side peaks. The correlation has smaller peaks half a chip either side of the true one, and the
    // discriminator holds on those too, 0.5 + d/2 chips off. There the correlator nearer the true peak beats the
    // prompt, which never lasts on the true peak, so a sustained win moves the code across ("bump jumping").
    if (gal_ && !gap) {
        constexpr int kBumpEpochs = 30;                      // 120 ms
        const double P = std::hypot(double(IP), double(QP));
        const double Ve = std::hypot(double(acc[8]), double(acc[9]));
        const double Vl = std::hypot(double(acc[10]), double(acc[11]));
        bumpEarly_ = Ve > P && Ve > Vl ? bumpEarly_ + 1 : std::max(0, bumpEarly_ - 1);
        bumpLate_ = Vl > P && Vl > Ve ? bumpLate_ + 1 : std::max(0, bumpLate_ - 1);
        if (bumpEarly_ >= kBumpEpochs || bumpLate_ >= kBumpEpochs) {
            remCode_ += bumpEarly_ >= kBumpEpochs ? 0.5 : -0.5;
            bumpEarly_ = bumpLate_ = 0;
            ++ch.sidePeakJumps;
        }
    }

    ch.pllDisc.push_back(float(ep));
    ch.fllDisc.push_back(float(efHz));
    ch.dllDisc.push_back(float(ed));
    ch.carrNco.push_back(float(ncoHz));
    ch.codeNco.push_back(float(codeNco));

    // ---- C/N0 (M2M4 moment method) and phase lock indicator
    const double p = double(IP) * IP + double(QP) * QP;
    pwr_[size_t(epoch_ % cn0Win_)] = p;
    if (epoch_ + 1 >= cn0Win_ && (epoch_ + 1) % 20 == 0) {
        double m2 = 0, m4 = 0;
        for (double v : pwr_) { m2 += v; m4 += v * v; }
        m2 /= cn0Win_; m4 /= cn0Win_;
        const double pd = std::sqrt(std::max(0.0, 2.0 * m2 * m2 - m4));
        const double pn = m2 - pd;
        if (pd > 0 && pn > 0) cn0_ = float(10.0 * std::log10(pd / (pn * T)) + (gal_ ? 3.01 : 0.0));   // the pilot has half the E1 power
        else if (pd > 0) cn0_ = 60.0f;
        else cn0_ = 0.0f;
    }
    const double a = 0.02 * periodMs_;
    sI2_ = (1 - a) * sI2_ + a * double(IP) * IP;
    sQ2_ = (1 - a) * sQ2_ + a * double(QP) * QP;
    const float pli = (sI2_ + sQ2_) > 0 ? float((sI2_ - sQ2_) / (sI2_ + sQ2_)) : 0.0f;
    ch.cn0.push_back(cn0_);
    ch.pli.push_back(pli);
    ch.lock.push_back(pli > 0.6f ? 1 : 0);
    ++epoch_;
    return true;
}

void Tracking::trackChannel(const SignalSource& src, const ReceiverSettings& s, const AcqResult& acq,
                            qint64 startSample, TrackingChannel& ch,
                            const std::atomic<bool>* cancel, std::atomic<qint64>* msCounter)
{
    QElapsedTimer timer;
    timer.start();
    const double fs = s.samplingFreq;
    const qint64 totalSamples = src.numSamples();
    ChannelTracker tracker(s, acq, startSample, src.isComplex(), ch);
    const int periodMs = ch.periodMs;

    int maxMs = int(double(totalSamples - startSample) / (fs / 1000.0)) - 2 * periodMs;
    if (s.msToProcess > 0) maxMs = std::min(maxMs, s.msToProcess);
    maxMs /= periodMs;                                    // from here on "ms" counts code periods
    if (maxMs <= 0) { ch.status = QStringLiteral("No data"); return; }
    ch.reserve(maxMs);

    int reported = 0;
    for (int ms = 0; ms < maxMs; ++ms) {
        if (cancel && cancel->load(std::memory_order_relaxed)) { ch.status = QStringLiteral("Cancelled"); break; }
        if (!tracker.step(src, totalSamples)) break;
        if (msCounter && (ms - reported) >= 200) {
            msCounter->fetch_add((ms - reported) * periodMs, std::memory_order_relaxed);
            reported = ms;
        }
    }
    if (msCounter) msCounter->fetch_add((int(ch.IP.size()) - reported) * periodMs, std::memory_order_relaxed);

    ch.summarise(s);
    if (ch.status.isEmpty()) ch.status = ch.valid ? QStringLiteral("Tracked") : QStringLiteral("Failed");
    ch.elapsedSec = timer.elapsed() / 1000.0;
}

std::vector<CorrShapePoint> Tracking::correlationShape(const SignalSource& src, const ReceiverSettings& s,
                                                       const TrackingChannel& ch, int ms,
                                                       double span, int taps, int avgMs)
{
    std::vector<CorrShapePoint> out;
    if (!ch.valid || ms < 0 || ms >= ch.numMs || taps < 2) return out;
    const double fs = s.samplingFreq;
    const std::vector<int8_t> table = replicaTable(ch.prn, false);
    const int len = int(table.size());
    const double tableScale = double(len) / ch.codeLength;
    out.resize(taps);
    for (int t = 0; t < taps; ++t) {
        out[t].offset = -span + 2.0 * span * t / (taps - 1);
        out[t].magnitude = out[t].inPhase = out[t].quadrature = 0;
    }
    const int last = std::min(ch.numMs, ms + std::max(1, avgMs));
    int used = 0;
    std::vector<float> bi, bq, rI, rQ;
    for (int m = ms; m < last; ++m) {
        const int blk = ch.blockSize[m];
        bi.resize(blk); bq.resize(blk); rI.resize(blk); rQ.resize(blk);
        if (src.read(ch.blockStart[m], blk, rI.data(), rQ.data()) < blk) break;
        const double carrStep = ch.carrFreq[m] / fs;
        for (int i = 0; i < blk; ++i) {
            const double ph = kTwoPi * (ch.remCarrPhase[m] + i * carrStep);
            const double c = std::cos(ph), sn = std::sin(ph);
            bi[i] = float(rI[i] * c + rQ[i] * sn);
            bq[i] = float(rQ[i] * c - rI[i] * sn);
        }
        const double codeStep = ch.codeFreq[m] / fs;
        const double sgn = ch.IP[m] >= 0 ? 1.0 : -1.0;
        for (int t = 0; t < taps; ++t) {
            double ai = 0, aq = 0;
            for (int i = 0; i < blk; ++i) {
                const double cp = ch.remCodePhase[m] + i * codeStep + out[t].offset;
                int k = int(std::floor(cp * tableScale)) % len;
                if (k < 0) k += len;
                ai += table[k] * bi[i];
                aq += table[k] * bq[i];
            }
            out[t].magnitude += std::hypot(ai, aq);
            out[t].inPhase += sgn * ai;
            out[t].quadrature += sgn * aq;
        }
        ++used;
    }
    if (used > 0)
        for (auto& p : out) { p.magnitude /= used; p.inPhase /= used; p.quadrature /= used; }
    return out;
}

} // namespace gnss
