#include "Acquisition.h"
#include "CaCode.h"
#include "Constants.h"
#include "Fft.h"
#include "GalileoCodes.h"
#include "GlonassCode.h"

#include <QElapsedTimer>
#include <QtConcurrent/QtConcurrentMap>

#include <algorithm>
#include <cmath>
#include <complex>
#include <numeric>

namespace gnss {

namespace {

// Fine frequency estimation robust to navigation-bit transitions:
// the code is wiped off with the acquired code phase, the residual carrier is
// integrated coherently over 0.5 ms blocks P_k, and the spectrum of P_k^2
// (which removes the +/-1 data modulation and doubles the frequency) is
// evaluated on a 1 Hz grid. The frequency offset is half of the peak location.
void fineFrequency(const std::vector<float>& sI, const std::vector<float>& sQ, int available,
                   double fs, int fineMs, AcqResult& r)
{
    const int N = int(std::lround(fs / 1000.0));
    const int half = N / 2;
    int nBlk = 2 * fineMs;
    while (nBlk > 0 && r.codePhase + nBlk * half > available) --nBlk;
    r.fineFreq = r.coarseFreq;
    if (nBlk < 8) return;

    const auto chips = CaCode::chips(r.prn);
    const double w = -kTwoPi * r.coarseFreq / fs;
    std::vector<cplx> z(nBlk);
    for (int k = 0; k < nBlk; ++k) {
        double ai = 0, aq = 0;
        for (int n = k * half; n < (k + 1) * half; ++n) {
            const int si = r.codePhase + n;
            const int ci = int(std::floor(n * kCaChipRate / fs)) % kCaCodeLength;
            const double a = w * double(si);
            const double c = std::cos(a), s = std::sin(a);
            const double x = sI[si], y = sQ[si];
            ai += chips[ci] * (x * c - y * s);
            aq += chips[ci] * (x * s + y * c);
        }
        const cplx p(ai, aq);
        z[k] = cmul(p, p);
    }
    const double tBlk = double(half) / fs;
    const int nF = 2001;      // 2*df from -1000..+1000 Hz in 1 Hz steps
    r.fineAxis.resize(nF);
    r.fineSpectrum.resize(nF);
    int best = 0;
    for (int i = 0; i < nF; ++i) {
        const double f2 = -1000.0 + i;
        const cplx rot(std::cos(-kTwoPi * f2 * tBlk), std::sin(-kTwoPi * f2 * tBlk));
        cplx ph(1, 0), acc(0, 0);
        for (int k = 0; k < nBlk; ++k) { acc += cmul(z[k], ph); ph = cmul(ph, rot); }
        r.fineAxis[i] = f2 / 2.0;
        r.fineSpectrum[i] = float(std::abs(acc));
        if (r.fineSpectrum[i] > r.fineSpectrum[best]) best = i;
    }
    double f2 = -1000.0 + best;
    if (best > 0 && best < nF - 1) {
        const double y0 = r.fineSpectrum[best - 1], y1 = r.fineSpectrum[best], y2 = r.fineSpectrum[best + 1];
        const double den = y0 - 2 * y1 + y2;
        if (den != 0) f2 += 0.5 * (y0 - y2) / den;
    }
    r.fineDelta = f2 / 2.0;
    r.fineFreq = r.coarseFreq + r.fineDelta;
}

// Fine frequency for E1: as above. The E1-C pilot replica wipes the code, and squaring removes the secondary code.
// Blocks are half a code period, so they never straddle a sign change. Squaring halves the unambiguous range, and
// 2 ms blocks keep it at +/-125 Hz, wider than the Doppler bins: with 4 ms blocks a true offset beyond +/-62.5 Hz
// would alias, and a Costas loop with T = 4 ms then locks happily on the wrong line, 125 Hz away and 3.9 dB down.
void fineFrequencyE1(const std::vector<float>& sI, const std::vector<float>& sQ, int available,
                     double fs, double ifFreq, int fineMs, AcqResult& r)
{
    const double period = 0.5 * kE1CodeMs * fs / 1000.0;
    int nBlk = 2 * fineMs / kE1CodeMs;
    while (nBlk > 0 && r.codePhase + int(std::ceil(nBlk * period)) > available) --nBlk;
    r.fineFreq = r.coarseFreq;
    if (nBlk < 8) return;

    const auto table = GalileoCode::boc11(GalileoCode::e1c(svidOf(r.prn)));
    const int len = int(table.size());
    const double step = 2.0 * kCaChipRate * (1.0 + (r.coarseFreq - ifFreq) / kL1Freq) / fs;   // half chips per sample
    const double w = -kTwoPi * r.coarseFreq / fs;
    std::vector<cplx> z(nBlk);
    for (int k = 0; k < nBlk; ++k) {
        double ai = 0, aq = 0;
        const int n0 = int(std::lround(k * period)), n1 = int(std::lround((k + 1) * period));
        for (int n = n0; n < n1; ++n) {
            const int si = r.codePhase + n;
            const int ci = int(std::floor(n * step)) % len;
            const double a = w * double(si);
            const double c = std::cos(a), sn = std::sin(a);
            const double x = sI[si], y = sQ[si];
            ai += table[ci] * (x * c - y * sn);
            aq += table[ci] * (x * sn + y * c);
        }
        const cplx p(ai, aq);
        z[k] = cmul(p, p);
    }
    const double tBlk = period / fs;
    const int nF = 501;      // 2*df from -250..+250 Hz, the unambiguous range for 2 ms blocks
    r.fineAxis.resize(nF);
    r.fineSpectrum.resize(nF);
    int best = 0;
    for (int i = 0; i < nF; ++i) {
        const double f2 = -250.0 + i;
        const cplx rot(std::cos(-kTwoPi * f2 * tBlk), std::sin(-kTwoPi * f2 * tBlk));
        cplx ph(1, 0), acc(0, 0);
        for (int k = 0; k < nBlk; ++k) { acc += cmul(z[k], ph); ph = cmul(ph, rot); }
        r.fineAxis[i] = f2 / 2.0;
        r.fineSpectrum[i] = float(std::abs(acc));
        if (r.fineSpectrum[i] > r.fineSpectrum[best]) best = i;
    }
    double f2 = -250.0 + best;
    if (best > 0 && best < nF - 1) {
        const double y0 = r.fineSpectrum[best - 1], y1 = r.fineSpectrum[best], y2 = r.fineSpectrum[best + 1];
        const double den = y0 - 2 * y1 + y2;
        if (den != 0) f2 += 0.5 * (y0 - y2) / den;
    }
    r.fineDelta = f2 / 2.0;
    r.fineFreq = r.coarseFreq + r.fineDelta;
}

// Galileo E1 search. One 4 ms BOC(1,1) replica is correlated against an 8 ms window, so the window always holds a
// whole code period and a data or secondary-code sign change cannot cancel the peak. Doppler steps are circular
// shifts of one spectrum, and only the bins around the carrier are kept, which holds both BOC main lobes.
// E1-B and E1-C are searched separately: a satellite counts as found only when these two unrelated codes put their
// peaks at the same delay and Doppler.
void acquireE1(const SignalSource& src, const ReceiverSettings& s, AcquisitionOutput& out,
               const std::atomic<bool>* cancel, const std::function<void(int)>& progress)
{
    const double fs = s.samplingFreq;
    const double period = kE1CodeMs * fs / 1000.0;
    const int N = int(std::lround(period));
    const int K = std::max(1, s.galNonCoherent);
    int L = 1;
    while (L < 2 * N) L *= 2;
    int M = L;
    while (fs * (M / 2) / L >= 4.5e6) M /= 2;
    const int nLag = int(std::ceil(double(N) * M / L));
    const int fineMs = std::max(20, s.fineFreqMs);

    const qint64 start = qint64(std::llround(s.acqStartMs * fs / 1000.0));
    const int total = int(std::lround((K + 2) * period)) + 2 * N + int(std::lround(fineMs * fs / 1000.0));
    std::vector<float> sI(total), sQ(total);
    const int got = src.read(start, total, sI.data(), sQ.data());
    if (got < int(std::lround(K * period)) + 2 * N) {
        out.error = QStringLiteral("Not enough data for acquisition");
        return;
    }
    out.startSample = start;
    double mi = 0, mq = 0;
    for (int i = 0; i < got; ++i) { mi += sI[i]; mq += sQ[i]; }
    mi /= got; mq /= got;
    for (int i = 0; i < got; ++i) { sI[i] -= float(mi); sQ[i] -= float(mq); }

    // GPS has already measured the receiver clock offset, so a narrower band around it is enough
    double centre = s.ifFreq, span = s.acqSearchBand;
    double sum = 0;
    int cnt = 0;
    for (const AcqResult& r : out.results)
        if (r.acquired) { sum += r.doppler; ++cnt; }
    if (cnt == 0 && s.acqHasDopplerHint) { sum = s.acqDopplerHint; cnt = 1; }
    if (cnt > 0) {
        centre += sum / cnt;
        span = std::min(span, 5000.0);
    }
    const double binHz = fs / L;
    const int step = std::max(1, int(std::lround(125.0 / binHz)));   // about 1/(2T) for T = 4 ms
    const int nd = int(span / (step * binHz));
    const int nBins = 2 * nd + 1;
    const int centreBin = int(std::lround(centre / binHz));
    std::vector<double> binFreqs(nBins);
    for (int b = 0; b < nBins; ++b) binFreqs[b] = (centreBin + (b - nd) * step) * binHz;

    std::vector<int> kIdx(K);
    std::iota(kIdx.begin(), kIdx.end(), 0);
    std::vector<std::vector<cplx>> blocks(K);
    QtConcurrent::blockingMap(kIdx, [&](const int& k) {
        Fft fft(L);
        std::vector<cplx> x(L, cplx(0, 0));
        const int off = int(std::lround(k * period));
        for (int n = 0; n < 2 * N; ++n) x[n] = cplx(sI[off + n], sQ[off + n]);
        fft.forward(x);
        blocks[k] = std::move(x);
    });

    std::vector<int> svids(s.acqGalSvids.begin(), s.acqGalSvids.end());
    if (svids.empty())
        for (int v = 1; v <= kGalileoMaxSv; ++v) svids.push_back(v);
    const int P = int(svids.size());
    std::vector<AcqResult> results(P);
    std::vector<int> pIdx(P);
    std::iota(pIdx.begin(), pIdx.end(), 0);
    std::atomic<int> done{0};
    const double spc = fs / kCaChipRate;                    // samples per chip
    const double lagSamples = double(L) / M;
    QtConcurrent::blockingMap(pIdx, [&](const int& p) {
        if (cancel && cancel->load()) return;
        AcqResult& r = results[p];
        const int svid = svids[size_t(p)];
        r.prn = kGalileoIdBase + svid;

        Fft fftL(L), fftM(M);
        std::vector<cplx> rep(L), C(M), Y(M);
        std::vector<float> surfB(size_t(nBins) * nLag, 0.0f), surfC(size_t(nBins) * nLag, 0.0f);
        for (int comp = 0; comp < 2; ++comp) {
            const auto table = GalileoCode::boc11(comp == 0 ? GalileoCode::e1b(svid) : GalileoCode::e1c(svid));
            const int len = int(table.size());
            std::fill(rep.begin(), rep.end(), cplx(0, 0));
            for (int n = 0; n < N; ++n) rep[n] = cplx(table[int(std::floor(2.0 * n * kCaChipRate / fs)) % len], 0);
            fftL.forward(rep);
            for (int j = 0; j < M; ++j) {
                const int o = j < M / 2 ? j : j - M;
                C[j] = std::conj(rep[o & (L - 1)]);
            }
            std::vector<float>& surf = comp == 0 ? surfB : surfC;
            for (const auto& X : blocks) {
                for (int b = 0; b < nBins; ++b) {
                    const int fi = centreBin + (b - nd) * step;
                    for (int j = 0; j < M; ++j) {
                        const int o = j < M / 2 ? j : j - M;
                        Y[j] = cmul(X[(fi + o) & (L - 1)], C[j]);
                    }
                    fftM.inverse(Y);
                    float* dst = surf.data() + size_t(b) * nLag;
                    for (int t = 0; t < nLag; ++t) dst[t] += float(std::norm(Y[t]));
                }
            }
        }

        const auto mean = [](const std::vector<float>& v) {
            double m = 0;
            for (float x : v) m += x;
            return m / double(v.size());
        };
        const double mb = mean(surfB), mc = mean(surfC);
        const size_t peakB = size_t(std::max_element(surfB.begin(), surfB.end()) - surfB.begin());
        const size_t peakC = size_t(std::max_element(surfC.begin(), surfC.end()) - surfC.begin());
        const int dLag = std::abs(int(peakB % nLag) - int(peakC % nLag));
        const bool agree = std::abs(int(peakB / nLag) - int(peakC / nLag)) <= 1 && std::min(dLag, nLag - dLag) <= 2;

        std::vector<float> c(surfB.size());
        for (size_t i = 0; i < c.size(); ++i) c[i] = float(surfB[i] / mb + surfC[i] / mc);
        const size_t best = size_t(std::max_element(c.begin(), c.end()) - c.begin());
        const int bStar = int(best / nLag);
        const int tau = int(best % nLag);
        r.peak = c[best];
        r.peakBin = bStar;
        r.codePhase = int(std::lround(tau * lagSamples));
        r.codePhaseChips = r.codePhase / spc;
        r.coarseFreq = binFreqs[bStar];
        r.binFreqs = binFreqs;

        const float* row = c.data() + size_t(bStar) * nLag;
        const int excl = int(std::ceil(spc / lagSamples));
        double second = 0;
        for (int n = 0; n < nLag; ++n) {
            int dist = std::abs(n - tau);
            dist = std::min(dist, nLag - dist);
            if (dist > excl) second = std::max(second, double(row[n]));
        }
        r.secondPeak = second;
        r.noiseMean = 2.0;                                  // each surface is normalised to mean 1
        r.metric = second > 0 ? r.peak / second : 0;
        // Each component holds half of the signal power; T = 4 ms
        const double snr = (r.peak - r.noiseMean) / r.noiseMean;
        r.cn0 = 10.0 * std::log10(std::max(2.0 * snr, 1e-3) / 0.004);
        r.acquired = agree;

        r.codeSlice.assign(row, row + nLag);
        r.dopplerSlice.resize(nBins);
        for (int bb = 0; bb < nBins; ++bb) r.dopplerSlice[bb] = c[size_t(bb) * nLag + tau];
        const int cols = std::min(nLag, 2046);
        r.surfaceCols = cols;
        r.surface.assign(size_t(nBins) * cols, 0.0f);
        for (int bb = 0; bb < nBins; ++bb) {
            const float* src_ = c.data() + size_t(bb) * nLag;
            for (int col = 0; col < cols; ++col) {
                const int a0 = int((long long)col * nLag / cols);
                const int a1 = std::max(a0 + 1, int((long long)(col + 1) * nLag / cols));
                float m = 0;
                for (int n = a0; n < a1; ++n) m = std::max(m, src_[n]);
                r.surface[size_t(bb) * cols + col] = m;
            }
        }
        if (r.acquired) fineFrequencyE1(sI, sQ, got, fs, s.ifFreq, fineMs, r);
        else r.fineFreq = r.coarseFreq;
        r.doppler = r.fineFreq - s.ifFreq;

        const int d = ++done;
        if (progress) progress(d * 100 / P);
    });
    if (cancel && cancel->load()) {
        out.error = QStringLiteral("Cancelled");
        return;
    }

    // Satellite Dopplers roughly average out, leaving the IF and clock error
    if (cnt == 0)
        for (const AcqResult& r : results)
            if (r.acquired) { sum += r.doppler; ++cnt; }
    for (AcqResult& r : results) r.commonDoppler = cnt > 0 ? sum / cnt : 0.0;
    out.results.insert(out.results.end(), results.begin(), results.end());
}

// GPS C/A search
void acquireCa(const SignalSource& src, const ReceiverSettings& s, AcquisitionOutput& out,
               const std::atomic<bool>* cancel, const std::function<void(int)>& progress)
{
    const double fs = s.samplingFreq;
    const int N = s.samplesPerCode();
    const int K = std::max(1, s.acqNonCoherent);
    const int fineMs = std::max(10, s.fineFreqMs);
    const qint64 start = qint64(std::llround(s.acqStartMs * fs / 1000.0));
    const int total = N * (K + fineMs + 2);

    out.samplesPerCode = N;
    out.startSample = start;
    out.nonCoherent = K;
    out.fftSize = N;

    std::vector<float> sI(total), sQ(total);
    const int got = src.read(start, total, sI.data(), sQ.data());
    if (got < N * K) {
        out.error = QStringLiteral("Not enough data for acquisition");
        return;
    }
    // Remove any DC offset (matters for zero-IF complex recordings)
    double mi = 0, mq = 0;
    for (int i = 0; i < got; ++i) { mi += sI[i]; mq += sQ[i]; }
    mi /= got; mq /= got;
    for (int i = 0; i < got; ++i) { sI[i] -= float(mi); sQ[i] -= float(mq); }

    const int nBins = int(std::floor(2.0 * s.acqSearchBand / s.acqStep + 1e-6)) + 1;
    out.binFreqs.resize(nBins);
    for (int b = 0; b < nBins; ++b) out.binFreqs[b] = s.ifFreq - s.acqSearchBand + b * s.acqStep;

    const QVector<int> prnList = s.prnList();
    std::vector<int> prns(prnList.begin(), prnList.end());
    const int P = int(prns.size());
    out.results.resize(P);
    for (int p = 0; p < P; ++p) out.results[p].prn = prns[p];

    // ---- FFT of each sampled C/A code replica (conjugated)
    std::vector<std::vector<cplx>> codeSpec(P);
    std::vector<int> pIdx(P);
    std::iota(pIdx.begin(), pIdx.end(), 0);
    QtConcurrent::blockingMap(pIdx, [&](const int& p) {
        Fft fft(N);
        const auto chips = CaCode::chips(prns[p]);
        std::vector<cplx> c(N);
        for (int n = 0; n < N; ++n) {
            const int ci = int(std::floor(n * kCaChipRate / fs)) % kCaCodeLength;
            c[n] = cplx(chips[ci], 0);
        }
        fft.forward(c);
        for (auto& v : c) v = std::conj(v);
        codeSpec[p] = std::move(c);
    });
    {
        Fft probe(N);
        out.fftInternalSize = probe.internalSize();
    }

    // ---- correlation surfaces: corr[p][bin * N + tau]
    std::vector<std::vector<float>> corr(P, std::vector<float>(size_t(nBins) * N, 0.0f));
    std::atomic<int> done{0};
    std::vector<int> bins(nBins);
    std::iota(bins.begin(), bins.end(), 0);
    QtConcurrent::blockingMap(bins, [&](const int& b) {
        if (cancel && cancel->load()) return;
        Fft fft(N);
        std::vector<cplx> X(N), Y(N);
        const double w = -kTwoPi * out.binFreqs[b] / fs;
        for (int k = 0; k < K; ++k) {
            const int off = k * N;
            for (int n = 0; n < N; ++n) {
                const double a = w * double(off + n);
                const double c = std::cos(a), sn = std::sin(a);
                const double x = sI[off + n], y = sQ[off + n];
                X[n] = cplx(x * c - y * sn, x * sn + y * c);
            }
            fft.forward(X);
            for (int p = 0; p < P; ++p) {
                const cplx* C = codeSpec[p].data();
                for (int n = 0; n < N; ++n) Y[n] = cmul(X[n], C[n]);
                fft.inverse(Y);
                float* dst = corr[p].data() + size_t(b) * N;
                for (int n = 0; n < N; ++n) dst[n] += float(std::norm(Y[n]));
            }
        }
        const int d = ++done;
        if (progress) progress(d * 90 / nBins);
    });
    if (cancel && cancel->load()) {
        out.error = QStringLiteral("Cancelled");
        return;
    }

    // ---- peak search, metrics, diagnostics and fine frequency
    const double spc = double(N) / kCaCodeLength;   // samples per chip
    const int excl = int(std::ceil(spc));
    QtConcurrent::blockingMap(pIdx, [&](const int& p) {
        AcqResult& r = out.results[p];
        std::vector<float>& c = corr[p];
        const size_t best = size_t(std::max_element(c.begin(), c.end()) - c.begin());
        const int bStar = int(best / N);
        const int tau = int(best % N);
        r.peak = c[best];
        r.peakBin = bStar;
        r.codePhase = tau;
        r.codePhaseChips = tau / spc;
        r.coarseFreq = out.binFreqs[bStar];

        const float* row = c.data() + size_t(bStar) * N;
        double second = 0, sum = 0;
        int cnt = 0;
        for (int n = 0; n < N; ++n) {
            int dist = std::abs(n - tau);
            dist = std::min(dist, N - dist);
            if (dist > excl) second = std::max(second, double(row[n]));
            if (dist > 2 * excl) { sum += row[n]; ++cnt; }
        }
        r.secondPeak = second;
        r.noiseMean = cnt > 0 ? sum / cnt : 0;
        r.metric = second > 0 ? r.peak / second : 0;
        const double snr = r.noiseMean > 0 ? (r.peak - r.noiseMean) / r.noiseMean : 0;
        r.cn0 = 10.0 * std::log10(std::max(snr, 1e-3) / 1e-3);
        r.acquired = r.metric > s.acqThreshold;

        r.codeSlice.assign(row, row + N);
        r.dopplerSlice.resize(nBins);
        for (int bb = 0; bb < nBins; ++bb) r.dopplerSlice[bb] = c[size_t(bb) * N + tau];

        const int cols = std::min(N, 2046);
        r.surfaceCols = cols;
        r.surface.assign(size_t(nBins) * cols, 0.0f);
        for (int bb = 0; bb < nBins; ++bb) {
            const float* src_ = c.data() + size_t(bb) * N;
            for (int col = 0; col < cols; ++col) {
                const int a0 = int((long long)col * N / cols);
                const int a1 = std::max(a0 + 1, int((long long)(col + 1) * N / cols));
                float m = 0;
                for (int n = a0; n < a1; ++n) m = std::max(m, src_[n]);
                r.surface[size_t(bb) * cols + col] = m;
            }
        }
        std::vector<float>().swap(c);    // free full-resolution surface

        fineFrequency(sI, sQ, got, fs, fineMs, r);
        r.doppler = r.fineFreq - s.ifFreq;
    });
}

// GLONASS L1OF search: one code, so each frequency channel is searched for it around its own IF
void acquireG1(const SignalSource& src, const ReceiverSettings& s, AcquisitionOutput& out, const std::atomic<bool>* cancel)
{
    const double fs = s.samplingFreq;
    const int N = int(std::llround(fs / 1000.0));
    const int K = std::max(1, s.acqNonCoherent);
    constexpr int kFineMs = 40, kFineBlockMs = 10;
    const qint64 start = qint64(std::llround(s.acqStartMs * fs / 1000.0));
    const int total = N * std::max(K, kFineMs);
    std::vector<float> sI(total), sQ(total);
    const int got = src.read(start, total, sI.data(), sQ.data());
    if (got < total) {
        out.error = QStringLiteral("Not enough data for acquisition");
        return;
    }
    double mi = 0, mq = 0;
    for (int i = 0; i < got; ++i) { mi += sI[i]; mq += sQ[i]; }
    for (int i = 0; i < got; ++i) { sI[i] -= float(mi / got); sQ[i] -= float(mq / got); }

    out.samplesPerCode = N;
    out.startSample = start;
    out.nonCoherent = K;
    out.fftSize = N;
    const auto chips = GlonassCode::chips();
    std::vector<float> rep(N);
    std::vector<cplx> spec(N);
    for (int n = 0; n < N; ++n) {
        rep[n] = chips[size_t(int(std::floor(n * kGlonassChipRate / fs)) % kGlonassCodeLength)];
        spec[n] = cplx(rep[n], 0);
    }
    { Fft fft(N); fft.forward(spec); }
    for (cplx& v : spec) v = std::conj(v);

    const int nBins = int(std::floor(2.0 * s.acqSearchBand / s.acqStep + 1e-6)) + 1;
    std::vector<AcqResult> found(kGlonassMaxChannel - kGlonassMinChannel + 1);
    std::vector<int> idx(found.size());
    std::iota(idx.begin(), idx.end(), 0);
    QtConcurrent::blockingMap(idx, [&](const int& i) {
        if (cancel && cancel->load()) return;
        AcqResult& r = found[size_t(i)];
        const int channel = kGlonassMinChannel + i;
        const double centre = s.glonassIf(channel);
        r.prn = glonassId(channel);
        Fft fft(N);
        std::vector<cplx> X(N), Y(N);
        std::vector<float> row(N), best(N);
        double bestPeak = -1, bestFreq = centre;
        for (int b = 0; b < nBins; ++b) {
            const double f = centre - s.acqSearchBand + b * s.acqStep;
            const double w = -kTwoPi * f / fs;
            std::fill(row.begin(), row.end(), 0.0f);
            for (int k = 0; k < K; ++k) {
                for (int n = 0; n < N; ++n) {
                    const double p = w * double(k * N + n), c = std::cos(p), sn = std::sin(p);
                    const double x = sI[k * N + n], y = sQ[k * N + n];
                    X[n] = cplx(x * c - y * sn, x * sn + y * c);
                }
                fft.forward(X);
                for (int n = 0; n < N; ++n) Y[n] = cmul(X[n], spec[n]);
                fft.inverse(Y);
                for (int n = 0; n < N; ++n) row[n] += float(std::norm(Y[n]));
            }
            const double peak = *std::max_element(row.begin(), row.end());
            if (peak > bestPeak) { bestPeak = peak; bestFreq = f; best = row; r.peakBin = b; }
        }
        const int tau = int(std::max_element(best.begin(), best.end()) - best.begin());
        const double spc = N / double(kGlonassCodeLength);
        const int excl = int(std::ceil(spc));
        double second = 0, sum = 0;
        int cnt = 0;
        for (int n = 0; n < N; ++n) {
            int d = std::abs(n - tau);
            d = std::min(d, N - d);
            if (d > excl) second = std::max(second, double(best[n]));
            if (d > 2 * excl) { sum += best[n]; ++cnt; }
        }
        r.peak = bestPeak;
        r.secondPeak = second;
        r.noiseMean = cnt ? sum / cnt : 0;
        r.metric = second > 0 ? bestPeak / second : 0;
        r.cn0 = 10.0 * std::log10(std::max(r.noiseMean > 0 ? (bestPeak - r.noiseMean) / r.noiseMean : 0, 1e-3) / 1e-3);
        r.codePhase = tau;
        r.codePhaseChips = tau / spc;
        r.coarseFreq = bestFreq;
        r.acquired = r.metric > s.acqThreshold;
        r.fineFreq = bestFreq;
        if (r.acquired) {
            // Fine frequency from 1 ms correlations, in 10 ms blocks summed by power so that bit edges matter little
            std::vector<std::complex<double>> ms(kFineMs);
            const double w = -kTwoPi * bestFreq / fs;
            for (int m = 0; m < kFineMs; ++m) {
                std::complex<double> v = 0;
                for (int n = 0; n < N; ++n) {
                    const int q = m * N + n;
                    const double p = w * double(q);
                    const double code = rep[size_t(((n - tau) % N + N) % N)];
                    v += code * std::complex<double>(sI[q], sQ[q]) * std::complex<double>(std::cos(p), std::sin(p));
                }
                ms[size_t(m)] = v;
            }
            double top = -1;
            for (double df = -350; df <= 350; df += 5) {
                double power = 0;
                for (int b = 0; b + kFineBlockMs <= kFineMs; b += kFineBlockMs) {
                    std::complex<double> v = 0;
                    for (int m = 0; m < kFineBlockMs; ++m) {
                        const double p = -kTwoPi * df * (b + m) * 1e-3;
                        v += ms[size_t(b + m)] * std::complex<double>(std::cos(p), std::sin(p));
                    }
                    power += std::norm(v);
                }
                if (power > top) { top = power; r.fineFreq = bestFreq + df; r.fineDelta = df; }
            }
        }
        r.doppler = r.fineFreq - centre;
    });
    out.results.insert(out.results.end(), std::make_move_iterator(found.begin()), std::make_move_iterator(found.end()));
}

} // namespace

AcquisitionOutput Acquisition::run(const SignalSource& src, const ReceiverSettings& s,
                                   const std::atomic<bool>* cancel,
                                   const std::function<void(int)>& progress)
{
    AcquisitionOutput out;
    QElapsedTimer timer;
    timer.start();

    const bool both = s.useGps() && s.useGalileo();
    const auto part = [&](int base) {
        return [&progress, base, both](int p) {
            if (progress) progress(both ? base + p / 2 : p);
        };
    };
    if (s.useGps()) acquireCa(src, s, out, cancel, part(0));
    if (out.error.isEmpty() && s.useGalileo()) acquireE1(src, s, out, cancel, part(50));
    if (out.error.isEmpty() && s.useGlonass()) acquireG1(src, s, out, cancel);
    if (progress) progress(100);
    out.elapsedSec = timer.elapsed() / 1000.0;
    return out;
}

} // namespace gnss
