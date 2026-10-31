#include "NavMessage.h"
#include "GalileoNav.h"
#include "Constants.h"

#include <QHash>
#include <algorithm>
#include <cmath>

namespace gnss {

GlobalNavData::GlobalNavData()
{
    for (int i = 0; i < 33; ++i) {
        health6[i] = 0;
        health6Valid[i] = false;
        svConfig[i] = 0;
        svConfigValid[i] = false;
    }
}

quint64 InavWord::field(int start, int len) const
{
    quint64 v = 0;
    for (int i = 0; i < len; ++i) v = (v << 1) | bits[size_t(start + i)];
    return v;
}

quint64 Subframe::bits(int start1, int len) const
{
    quint64 v = 0;
    for (int i = 0; i < len; ++i) v = (v << 1) | dataBits[size_t(start1 - 1 + i)];
    return v;
}

namespace nav {

namespace {

double p2(int e) { return std::ldexp(1.0, e); }
QString p2t(int e) { return QStringLiteral("2^%1").arg(e); }

// Layout builder
struct LB
{
    std::vector<NavFieldDef> v;
    LB& add(const QString& key, const QString& name, int s1, int l1, int s2, int l2, bool sg, double sc,
            const QString& st, const QString& u, FieldKind k)
    {
        NavFieldDef f;
        f.key = key; f.name = name; f.s1 = s1; f.l1 = l1; f.s2 = s2; f.l2 = l2;
        f.isSigned = sg; f.scale = sc; f.scaleText = st; f.units = u; f.kind = k;
        v.push_back(f);
        return *this;
    }
    LB& num(const QString& key, const QString& name, int s1, int l1, bool sg = false, int e = 0,
            const QString& u = QString(), int s2 = 0, int l2 = 0)
    {
        return add(key, name, s1, l1, s2, l2, sg, p2(e), e == 0 ? QStringLiteral("1") : p2t(e), u, FieldKind::Number);
    }
    LB& numScale(const QString& key, const QString& name, int s1, int l1, double sc, const QString& st, const QString& u)
    {
        return add(key, name, s1, l1, 0, 0, false, sc, st, u, FieldKind::Number);
    }
    LB& semi(const QString& key, const QString& name, int s1, int l1, int e, const QString& u, int s2 = 0, int l2 = 0)
    {
        return add(key, name, s1, l1, s2, l2, true, p2(e), p2t(e), u, FieldKind::Semicircle);
    }
    LB& code(const QString& key, const QString& name, int s1, int l1)
    {
        return add(key, name, s1, l1, 0, 0, false, 1.0, QString(), QString(), FieldKind::Code);
    }
    LB& res(const QString& name, int s1, int l1, const QString& key = QStringLiteral("reserved"))
    {
        return add(key, name, s1, l1, 0, 0, false, 1.0, QString(), QString(), FieldKind::Reserved);
    }
    LB& text(const QString& key, const QString& name, int s1, int l1)
    {
        return add(key, name, s1, l1, 0, 0, false, 1.0, QString(), QString(), FieldKind::Text);
    }
};

void header(LB& b)
{
    b.code("preamble", "Preamble", 1, 8)
     .num("tlm", "TLM message", 9, 14)
     .code("isf", "Integrity status flag (ISF)", 23, 1)
     .res("Reserved", 24, 1)
     .numScale("tow", "TOW count (truncated Z-count)", 31, 17, 6.0, "6", "s")
     .code("alert", "Alert flag", 48, 1)
     .code("as", "Anti-spoof (A-S) flag", 49, 1)
     .code("sfid", "Subframe ID", 50, 3)
     .res("Parity-solving bits (t)", 53, 2, "t");
}

void subframe1(LB& b)
{
    b.num("wn", "Week number (mod 1024)", 61, 10, false, 0, "weeks")
     .code("l2code", "Codes on L2", 71, 2)
     .code("ura", "URA index", 73, 4)
     .code("health", "SV health", 77, 6)
     .num("iodc", "IODC (Issue of Data, Clock)", 83, 2, false, 0, "", 211, 8)
     .code("l2p", "L2 P data flag", 91, 1)
     .res("Reserved", 92, 23)
     .res("Reserved", 121, 24)
     .res("Reserved", 151, 24)
     .res("Reserved", 181, 16)
     .num("tgd", "T_GD (group delay)", 197, 8, true, -31, "s")
     .num("toc", "t_oc (clock reference time)", 219, 16, false, 4, "s")
     .num("af2", "a_f2 (clock drift rate)", 241, 8, true, -55, "s/s^2")
     .num("af1", "a_f1 (clock drift)", 249, 16, true, -43, "s/s")
     .num("af0", "a_f0 (clock bias)", 271, 22, true, -31, "s");
}

void subframe2(LB& b)
{
    b.num("iode", "IODE (Issue of Data, Ephemeris)", 61, 8)
     .num("crs", "C_rs (radius sine harmonic)", 69, 16, true, -5, "m")
     .semi("dn", "Delta n (mean motion difference)", 91, 16, -43, "semicircles/s")
     .semi("m0", "M_0 (mean anomaly at t_oe)", 107, 8, -31, "semicircles", 121, 24)
     .num("cuc", "C_uc (latitude cosine harmonic)", 151, 16, true, -29, "rad")
     .num("e", "e (eccentricity)", 167, 8, false, -33, "", 181, 24)
     .num("cus", "C_us (latitude sine harmonic)", 211, 16, true, -29, "rad")
     .num("sqrta", "sqrt(A) (square root of semi-major axis)", 227, 8, false, -19, "m^1/2", 241, 24)
     .num("toe", "t_oe (ephemeris reference time)", 271, 16, false, 4, "s")
     .code("fit", "Fit interval flag", 287, 1)
     .numScale("aodo", "AODO (age of data offset)", 288, 5, 900.0, "900", "s");
}

void subframe3(LB& b)
{
    b.num("cic", "C_ic (inclination cosine harmonic)", 61, 16, true, -29, "rad")
     .semi("omega0", "Omega_0 (longitude of ascending node)", 77, 8, -31, "semicircles", 91, 24)
     .num("cis", "C_is (inclination sine harmonic)", 121, 16, true, -29, "rad")
     .semi("i0", "i_0 (inclination at t_oe)", 137, 8, -31, "semicircles", 151, 24)
     .num("crc", "C_rc (radius cosine harmonic)", 181, 16, true, -5, "m")
     .semi("omega", "omega (argument of perigee)", 197, 8, -31, "semicircles", 211, 24)
     .semi("omegadot", "OMEGA DOT (rate of right ascension)", 241, 24, -43, "semicircles/s")
     .num("iode", "IODE (Issue of Data, Ephemeris)", 271, 8)
     .semi("idot", "IDOT (rate of inclination)", 279, 14, -43, "semicircles/s");
}

void almanacPage(LB& b)
{
    b.num("alm_e", "e (eccentricity)", 69, 16, false, -21)
     .num("alm_toa", "t_oa (almanac reference time)", 91, 8, false, 12, "s")
     .semi("alm_di", "delta i (relative to i0 = 0.30 semicircles)", 99, 16, -19, "semicircles")
     .semi("alm_omegadot", "OMEGA DOT", 121, 16, -38, "semicircles/s")
     .code("alm_health", "SV health (8 bits)", 137, 8)
     .num("alm_sqrta", "sqrt(A)", 151, 24, false, -11, "m^1/2")
     .semi("alm_omega0", "Omega_0", 181, 24, -23, "semicircles")
     .semi("alm_omega", "omega (argument of perigee)", 211, 24, -23, "semicircles")
     .semi("alm_m0", "M_0", 241, 24, -23, "semicircles")
     .num("alm_af0", "a_f0", 271, 8, true, -20, "s", 290, 3)
     .num("alm_af1", "a_f1", 279, 11, true, -38, "s/s");
}

void sf5Page25(LB& b)
{
    b.num("p25_toa", "t_oa (almanac reference time)", 69, 8, false, 12, "s")
     .num("p25_wna", "WN_a (almanac week, mod 256)", 77, 8, false, 0, "weeks");
    for (int sv = 1; sv <= 24; ++sv) {
        const int word = 4 + (sv - 1) / 4;
        const int pos = 30 * (word - 1) + 1 + 6 * ((sv - 1) % 4);
        b.code(QStringLiteral("h_sv%1").arg(sv), QStringLiteral("SV %1 health (6 bits)").arg(sv), pos, 6);
    }
    b.res("Reserved", 271, 22);
}

void sf4Page25(LB& b)
{
    auto cfgPos = [](int sv) {
        if (sv <= 4) return 69 + 4 * (sv - 1);
        if (sv <= 28) {
            const int k = sv - 5;
            const int word = 4 + k / 6;
            return 30 * (word - 1) + 1 + 4 * (k % 6);
        }
        return 211 + 4 * (sv - 29);
    };
    for (int sv = 1; sv <= 32; ++sv)
        b.code(QStringLiteral("cfg_sv%1").arg(sv), QStringLiteral("SV %1 A-S flag & configuration").arg(sv), cfgPos(sv), 4);
    b.res("Reserved", 227, 2);
    const int hpos[8] = {229, 241, 247, 253, 259, 271, 277, 283};
    for (int i = 0; i < 8; ++i)
        b.code(QStringLiteral("h_sv%1").arg(25 + i), QStringLiteral("SV %1 health (6 bits)").arg(25 + i), hpos[i], 6);
    b.res("Reserved", 289, 4);
}

void sf4Page18(LB& b)
{
    b.num("a0", "alpha_0", 69, 8, true, -30, "s")
     .num("a1", "alpha_1", 77, 8, true, -27, "s/semicircle")
     .num("a2", "alpha_2", 91, 8, true, -24, "s/semicircle^2")
     .num("a3", "alpha_3", 99, 8, true, -24, "s/semicircle^3")
     .num("b0", "beta_0", 107, 8, true, 11, "s")
     .num("b1", "beta_1", 121, 8, true, 14, "s/semicircle")
     .num("b2", "beta_2", 129, 8, true, 16, "s/semicircle^2")
     .num("b3", "beta_3", 137, 8, true, 16, "s/semicircle^3")
     .num("A1", "A_1 (UTC drift)", 151, 24, true, -50, "s/s")
     .num("A0", "A_0 (UTC bias)", 181, 24, true, -30, "s", 211, 8)
     .num("tot", "t_ot (UTC reference time)", 219, 8, false, 12, "s")
     .num("wnt", "WN_t (UTC reference week)", 227, 8, false, 0, "weeks")
     .num("dtls", "Delta t_LS (current leap seconds)", 241, 8, true, 0, "s")
     .num("wnlsf", "WN_LSF (leap second week)", 249, 8, false, 0, "weeks")
     .num("dn", "DN (leap second day)", 257, 8, false, 0, "days")
     .num("dtlsf", "Delta t_LSF (future leap seconds)", 271, 8, true, 0, "s")
     .res("Reserved", 279, 14);
}

void sf4Page17(LB& b)
{
    const int pos[22] = {69, 77, 91, 99, 107, 121, 129, 137, 151, 159, 167, 181, 189, 197,
                         211, 219, 227, 241, 249, 257, 271, 279};
    for (int i = 0; i < 22; ++i)
        b.text(QStringLiteral("ch%1").arg(i + 1), QStringLiteral("Character %1").arg(i + 1), pos[i], 8);
    b.res("Reserved", 287, 6);
}

void sf4Page13(LB& b)
{
    std::vector<int> positions;
    for (int w = 3; w <= 10; ++w) {
        const int ws = 30 * (w - 1) + 1;
        const int first = (w == 3) ? ws + 8 : ws;
        const int last = (w == 10) ? ws + 21 : ws + 23;
        for (int p = first; p <= last; ++p) positions.push_back(p);
    }
    auto addRun = [&](const QString& key, const QString& name, int idx, int len, bool sg, double sc,
                      const QString& st, const QString& u, FieldKind k) {
        int s1 = positions[idx], l1 = 1, s2 = 0, l2 = 0;
        for (int i = 1; i < len; ++i) {
            const int p = positions[idx + i];
            if (l2 == 0 && p == s1 + l1) ++l1;
            else if (l2 == 0) { s2 = p; l2 = 1; }
            else ++l2;
        }
        b.add(key, name, s1, l1, s2, l2, sg, sc, st, u, k);
    };
    addRun("ai", "Availability indicator (AI)", 0, 2, false, 1.0, QString(), QString(), FieldKind::Code);
    for (int i = 0; i < 30; ++i)
        addRun(QStringLiteral("erd%1").arg(i + 1), QStringLiteral("ERD %1 (estimated range deviation)").arg(i + 1),
               2 + 6 * i, 6, true, 0.3, "0.3", "m", FieldKind::Number);
}

void reservedPage(LB& b)
{
    b.res("Reserved / system use", 69, 16);
    for (int w = 4; w <= 9; ++w) b.res("Reserved / system use", 30 * (w - 1) + 1, 24);
    b.res("Reserved / system use", 271, 22);
}

QString dayHms(double t)
{
    static const char* days[7] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
    double tt = std::fmod(t, 604800.0);
    if (tt < 0) tt += 604800.0;
    const int d = int(tt / 86400.0);
    const double r = tt - d * 86400.0;
    const int h = int(r / 3600.0);
    const int m = int((r - h * 3600.0) / 60.0);
    const double s = r - h * 3600.0 - m * 60.0;
    return QStringLiteral("%1 %2:%3:%4").arg(QLatin1String(days[std::clamp(d, 0, 6)]))
        .arg(h, 2, 10, QLatin1Char('0')).arg(m, 2, 10, QLatin1Char('0'))
        .arg(s, 6, 'f', 3, QLatin1Char('0'));
}

QString interpret(const QString& key, quint64 raw, double value, const Subframe& sf)
{
    if (key == "preamble") return raw == 0x8B ? QStringLiteral("10001011 - valid TLM preamble") : QStringLiteral("INVALID preamble");
    if (key == "isf") return raw ? QStringLiteral("Enhanced level of integrity assurance") : QStringLiteral("Legacy level of integrity assurance");
    if (key == "tow") return QStringLiteral("Next subframe starts at TOW %1 s (%2); this subframe at %3 s")
                                 .arg(value, 0, 'f', 0).arg(dayHms(value)).arg(value - 6.0, 0, 'f', 0);
    if (key == "alert") return raw ? QStringLiteral("ALERT: URA may be worse than indicated - use at own risk") : QStringLiteral("No alert");
    if (key == "as") return raw ? QStringLiteral("A-S mode ON (P code encrypted to Y code)") : QStringLiteral("A-S mode off");
    if (key == "sfid") return (raw >= 1 && raw <= 5) ? QStringLiteral("Subframe %1").arg(raw) : QStringLiteral("INVALID subframe ID");
    if (key == "wn") return QStringLiteral("GPS week %1 + n x 1024 (rollover ambiguity)").arg(raw);
    if (key == "l2code") return l2CodeText(int(raw));
    if (key == "ura") return QStringLiteral("URA %1 (nominal %2 m)").arg(uraText(int(raw))).arg(uraNominalMeters(int(raw)));
    if (key == "health" || key.startsWith("h_sv")) return health6Text(int(raw));
    if (key == "alm_health") return health8Text(int(raw));
    if (key.startsWith("cfg_sv")) return svConfigText(int(raw));
    if (key == "l2p") return raw ? QStringLiteral("NAV data OFF on L2 P code") : QStringLiteral("NAV data ON on L2 P code");
    if (key == "fit") return raw ? QStringLiteral("Curve fit interval greater than 4 hours") : QStringLiteral("Curve fit interval 4 hours");
    if (key == "toc" || key == "toe" || key == "alm_toa" || key == "p25_toa" || key == "tot") return dayHms(value);
    if (key == "tgd") return QStringLiteral("%1 ns").arg(value * 1e9, 0, 'f', 3);
    if (key == "af0") return QStringLiteral("%1 us").arg(value * 1e6, 0, 'f', 4);
    if (key == "af1") return QStringLiteral("%1 ns/s (%2 m/s)").arg(value * 1e9, 0, 'g', 6).arg(value * kSpeedOfLight, 0, 'g', 5);
    if (key == "alm_af0") return QStringLiteral("%1 us").arg(value * 1e6, 0, 'f', 3);
    if (key == "sqrta" || key == "alm_sqrta") {
        const double A = value * value;
        const double T = kTwoPi * std::sqrt(A * A * A / kMu);
        return QStringLiteral("A = %1 km, period = %2 h").arg(A / 1000.0, 0, 'f', 3).arg(T / 3600.0, 0, 'f', 4);
    }
    if (key == "dataid") return raw == 1 ? QStringLiteral("01 - current data structure") : QStringLiteral("Data ID %1").arg(raw);
    if (key == "svid") return pageDescription(sf.id, int(raw));
    if (key == "iodc") return QStringLiteral("8 LSBs = %1 (must equal IODE)").arg(raw & 0xFF);
    if (key == "dtls" || key == "dtlsf") return QStringLiteral("GPS - UTC = %1 s").arg(value);
    if (key == "ai") return raw == 0 ? QStringLiteral("NMCT available (unencrypted)") :
                           (raw == 1 ? QStringLiteral("NMCT encrypted") : QStringLiteral("Reserved"));
    if (key.startsWith("erd")) return raw == 0x20 ? QStringLiteral("No correction available") : QString();
    if (key.startsWith("ch")) {
        const char c = char(raw);
        return (c >= 32 && c < 127) ? QStringLiteral("'%1'").arg(QChar(c)) : QStringLiteral("(non-printable)");
    }
    if (key == "aodo") return QStringLiteral("NMCT age of data offset");
    return QString();
}

} // namespace

std::vector<NavFieldDef> layoutFor(int sfId, int svId)
{
    LB b;
    header(b);
    switch (sfId) {
    case 1: subframe1(b); break;
    case 2: subframe2(b); break;
    case 3: subframe3(b); break;
    case 4:
    case 5:
        b.code("dataid", "Data ID", 61, 2).code("svid", "SV (page) ID", 63, 6);
        if ((sfId == 5 && svId >= 1 && svId <= 24) || (sfId == 4 && svId >= 25 && svId <= 32)) almanacPage(b);
        else if (sfId == 5 && svId == 51) sf5Page25(b);
        else if (sfId == 4 && svId == 63) sf4Page25(b);
        else if (sfId == 4 && svId == 56) sf4Page18(b);
        else if (sfId == 4 && svId == 55) sf4Page17(b);
        else if (sfId == 4 && svId == 52) sf4Page13(b);
        else reservedPage(b);
        break;
    default:
        for (int w = 3; w <= 10; ++w) b.res("Unknown", 30 * (w - 1) + 1, w == 10 ? 22 : 24);
        break;
    }
    if (sfId >= 1 && sfId <= 5) b.res("Parity-solving bits (t)", 293, 2, "t");
    for (int w = 1; w <= 10; ++w)
        b.add("parity", QStringLiteral("Parity word %1 (D25-D30)").arg(w), 30 * (w - 1) + 25, 6, 0, 0,
              false, 1.0, QString(), QString(), FieldKind::Parity);
    return b.v;
}

QString pageDescription(int sfId, int svId)
{
    if (svId == 0) return QStringLiteral("Dummy SV (no data)");
    if (sfId == 5) {
        if (svId >= 1 && svId <= 24) return QStringLiteral("Almanac for SV %1").arg(svId);
        if (svId == 51) return QStringLiteral("Page 25: SV health 1-24, t_oa, WN_a");
        return QStringLiteral("SV ID %1 (unexpected in subframe 5)").arg(svId);
    }
    if (sfId == 4) {
        if (svId >= 25 && svId <= 32) return QStringLiteral("Almanac for SV %1").arg(svId);
        switch (svId) {
        case 52: return QStringLiteral("Page 13: Navigation Message Correction Table");
        case 53: return QStringLiteral("Page 14: reserved for system use");
        case 54: return QStringLiteral("Page 15: reserved for system use");
        case 55: return QStringLiteral("Page 17: special message");
        case 56: return QStringLiteral("Page 18: ionospheric and UTC parameters");
        case 57: return QStringLiteral("Pages 1, 6, 11, 16, 21: reserved");
        case 58: return QStringLiteral("Pages 12, 24: reserved");
        case 59: return QStringLiteral("Page 19: reserved");
        case 60: return QStringLiteral("Page 20: reserved");
        case 61: return QStringLiteral("Page 22: reserved");
        case 62: return QStringLiteral("Page 23: reserved");
        case 63: return QStringLiteral("Page 25: A-S flags / SV configurations, health SV 25-32");
        default: return QStringLiteral("SV ID %1").arg(svId);
        }
    }
    return QString();
}

std::vector<DecodedField> decodeFields(const Subframe& sf)
{
    std::vector<DecodedField> out;
    const auto layout = layoutFor(sf.id, sf.svId);
    out.reserve(layout.size());
    for (const NavFieldDef& f : layout) {
        DecodedField d;
        d.def = f;
        quint64 raw = 0;
        QString bs;
        for (int i = 0; i < f.l1; ++i) {
            const int b = sf.dataBits[size_t(f.s1 - 1 + i)];
            raw = (raw << 1) | quint64(b);
            bs += QLatin1Char(b ? '1' : '0');
        }
        if (f.l2 > 0) bs += QLatin1Char(' ');
        for (int i = 0; i < f.l2; ++i) {
            const int b = sf.dataBits[size_t(f.s2 - 1 + i)];
            raw = (raw << 1) | quint64(b);
            bs += QLatin1Char(b ? '1' : '0');
        }
        const int n = f.totalBits();
        d.raw = raw;
        d.rawSigned = qint64(raw);
        if (f.isSigned && n > 0 && (raw >> (n - 1)) & 1) d.rawSigned = qint64(raw) - (qint64(1) << n);
        d.value = double(f.isSigned ? d.rawSigned : qint64(raw)) * f.scale;
        d.bitString = bs;
        switch (f.kind) {
        case FieldKind::Number:
            d.valueText = QString::number(d.value, 'g', 12);
            break;
        case FieldKind::Semicircle:
            d.valueText = QString::number(d.value, 'g', 12);
            break;
        case FieldKind::Code:
            d.valueText = QString::number(raw);
            break;
        case FieldKind::Reserved:
        case FieldKind::Parity:
            d.valueText = QStringLiteral("0x%1").arg(raw, 0, 16);
            break;
        case FieldKind::Text:
            d.valueText = QString::number(raw);
            break;
        }
        d.interpretation = interpret(f.key, raw, d.value, sf);
        if (f.kind == FieldKind::Semicircle) {
            const double rad = d.value * kGpsPi;
            const QString u = f.units.contains("/s") ? QStringLiteral("rad/s") : QStringLiteral("rad");
            const QString deg = f.units.contains("/s") ? QStringLiteral("deg/s") : QStringLiteral("deg");
            d.interpretation = QStringLiteral("%1 %2 = %3 %4").arg(rad, 0, 'g', 12).arg(u)
                                   .arg(rad * 180.0 / kGpsPi, 0, 'g', 10).arg(deg);
        }
        if (f.key == "e" || f.key == "alm_e")
            d.interpretation = d.value < 0.03 ? QStringLiteral("near-circular orbit") : QStringLiteral("eccentric orbit");
        if (f.key == "parity") {
            const int w = (f.s1 - 1) / 30;
            const NavWord& nw = sf.words[w];
            d.interpretation = nw.parityOk ? QStringLiteral("Parity OK")
                                           : QStringLiteral("PARITY FAIL (computed %1)").arg(nw.parityCalc, 6, 2, QLatin1Char('0'));
        }
        out.push_back(d);
    }
    return out;
}

// ---------------------------------------------------------------------------
QString uraText(int i)
{
    static const char* t[16] = {
        "0.00 - 2.40 m", "2.40 - 3.40 m", "3.40 - 4.85 m", "4.85 - 6.85 m", "6.85 - 9.65 m",
        "9.65 - 13.65 m", "13.65 - 24.00 m", "24.00 - 48.00 m", "48.00 - 96.00 m", "96.00 - 192.00 m",
        "192.00 - 384.00 m", "384.00 - 768.00 m", "768.00 - 1536.00 m", "1536.00 - 3072.00 m",
        "3072.00 - 6144.00 m", "> 6144 m (no accuracy prediction - use at own risk)"};
    return QLatin1String(t[std::clamp(i, 0, 15)]);
}

double uraNominalMeters(int i)
{
    static const double v[16] = {2.0, 2.8, 4.0, 5.7, 8.0, 11.3, 16.0, 32.0, 64.0, 128.0,
                                 256.0, 512.0, 1024.0, 2048.0, 4096.0, 6144.0};
    return v[std::clamp(i, 0, 15)];
}

QString signalHealthText(int c)
{
    static const char* t[32] = {
        "All signals OK", "All signals weak", "All signals dead", "All signals have no data modulation",
        "L1 P signal weak", "L1 P signal dead", "L1 P signal has no data modulation",
        "L2 P signal weak", "L2 P signal dead", "L2 P signal has no data modulation",
        "L1 C signal weak", "L1 C signal dead", "L1 C signal has no data modulation",
        "L2 C signal weak", "L2 C signal dead", "L2 C signal has no data modulation",
        "L1 & L2 P signal weak", "L1 & L2 P signal dead", "L1 & L2 P signal has no data modulation",
        "L1 & L2 C signal weak", "L1 & L2 C signal dead", "L1 & L2 C signal has no data modulation",
        "L1 signal weak", "L1 signal dead", "L1 signal has no data modulation",
        "L2 signal weak", "L2 signal dead", "L2 signal has no data modulation",
        "SV is temporarily out", "SV will be temporarily out",
        "One or more signals are deformed, URA parameters valid",
        "More than one combination would be required to describe anomalies"};
    return QLatin1String(t[c & 31]);
}

QString navDataHealthText(int c)
{
    static const char* t[8] = {"All data OK", "Parity failure - some or all parity bad",
                               "TLM/HOW format problem", "Z-count in HOW bad",
                               "Subframes 1, 2, 3 - one or more elements bad",
                               "Subframes 4, 5 - one or more elements bad",
                               "All uploaded data bad", "All data bad"};
    return QLatin1String(t[c & 7]);
}

QString health6Text(int h)
{
    const QString nav = (h & 0x20) ? QStringLiteral("NAV data: some or all BAD") : QStringLiteral("NAV data: all OK");
    return QStringLiteral("%1; signals: %2").arg(nav, signalHealthText(h & 31));
}

QString health8Text(int h)
{
    return QStringLiteral("NAV: %1; signals: %2").arg(navDataHealthText((h >> 5) & 7), signalHealthText(h & 31));
}

QString l2CodeText(int c)
{
    switch (c & 3) {
    case 1: return QStringLiteral("P code ON");
    case 2: return QStringLiteral("C/A code ON");
    default: return QStringLiteral("Reserved / invalid");
    }
}

QString svConfigText(int c4)
{
    const bool as = (c4 >> 3) & 1;
    QString cfg;
    switch (c4 & 7) {
    case 0: cfg = QStringLiteral("reserved"); break;
    case 1: cfg = QStringLiteral("Block II/IIA/IIR (A-S capable)"); break;
    case 2: cfg = QStringLiteral("Block IIR-M (M-code, L2C)"); break;
    case 3: cfg = QStringLiteral("Block IIF (M-code, L2C, L5)"); break;
    case 4: cfg = QStringLiteral("GPS III (M-code, L2C, L5, L1C, no SA)"); break;
    case 5: cfg = QStringLiteral("GPS IIIF"); break;
    default: cfg = QStringLiteral("reserved"); break;
    }
    return QStringLiteral("A-S %1; %2").arg(as ? QStringLiteral("ON") : QStringLiteral("off"), cfg);
}

QString towText(double tow) { return dayHms(tow); }

uint8_t computeParity(const uint8_t* D, int d29s, int d30s, uint8_t* out)
{
    uint8_t d[25];
    for (int i = 1; i <= 24; ++i) d[i] = D[i - 1] ^ uint8_t(d30s);
    if (out) for (int i = 0; i < 24; ++i) out[i] = d[i + 1];
    const int p25 = d29s ^ d[1] ^ d[2] ^ d[3] ^ d[5] ^ d[6] ^ d[10] ^ d[11] ^ d[12] ^ d[13] ^ d[14] ^ d[17] ^ d[18] ^ d[20] ^ d[23];
    const int p26 = d30s ^ d[2] ^ d[3] ^ d[4] ^ d[6] ^ d[7] ^ d[11] ^ d[12] ^ d[13] ^ d[14] ^ d[15] ^ d[18] ^ d[19] ^ d[21] ^ d[24];
    const int p27 = d29s ^ d[1] ^ d[3] ^ d[4] ^ d[5] ^ d[7] ^ d[8] ^ d[12] ^ d[13] ^ d[14] ^ d[15] ^ d[16] ^ d[19] ^ d[20] ^ d[22];
    const int p28 = d30s ^ d[2] ^ d[4] ^ d[5] ^ d[6] ^ d[8] ^ d[9] ^ d[13] ^ d[14] ^ d[15] ^ d[16] ^ d[17] ^ d[20] ^ d[21] ^ d[23];
    const int p29 = d30s ^ d[1] ^ d[3] ^ d[5] ^ d[6] ^ d[7] ^ d[9] ^ d[10] ^ d[14] ^ d[15] ^ d[16] ^ d[17] ^ d[18] ^ d[21] ^ d[22] ^ d[24];
    const int p30 = d29s ^ d[3] ^ d[5] ^ d[6] ^ d[8] ^ d[9] ^ d[10] ^ d[11] ^ d[13] ^ d[15] ^ d[19] ^ d[22] ^ d[23] ^ d[24];
    return uint8_t((p25 << 5) | (p26 << 4) | (p27 << 3) | (p28 << 2) | (p29 << 1) | p30);
}

namespace {

uint32_t pack(const uint8_t* b, int n)
{
    uint32_t v = 0;
    for (int i = 0; i < n; ++i) v = (v << 1) | b[i];
    return v;
}

void decodeSubframe(const NavChannel& nc, int j, bool inv, Subframe& sf)
{
    sf.startBit = j;
    sf.startMs = nc.firstBitMs + kMsPerBit * j;
    sf.inverted = inv;
    for (int i = 0; i < 300; ++i) {
        sf.rxBits[i] = uint8_t(nc.hardBits[size_t(j + i)] ^ (inv ? 1 : 0));
        sf.softBits[i] = nc.softBits[size_t(j + i)] * (inv ? -1.0f : 1.0f);
    }
    int p29 = j >= 2 ? (nc.hardBits[size_t(j - 2)] ^ (inv ? 1 : 0)) : 0;
    int p30 = j >= 1 ? (nc.hardBits[size_t(j - 1)] ^ (inv ? 1 : 0)) : 0;
    sf.parityOkCount = 0;
    for (int w = 0; w < 10; ++w) {
        const uint8_t* D = &sf.rxBits[size_t(30 * w)];
        uint8_t dd[24];
        NavWord& nw = sf.words[w];
        nw.d29s = uint8_t(p29);
        nw.d30s = uint8_t(p30);
        nw.parityCalc = computeParity(D, p29, p30, dd);
        nw.parityRx = uint8_t(pack(D + 24, 6));
        nw.raw30 = pack(D, 30);
        nw.data24 = pack(dd, 24);
        nw.parityOk = nw.parityCalc == nw.parityRx;
        if (nw.parityOk) ++sf.parityOkCount;
        for (int i = 0; i < 24; ++i) sf.dataBits[size_t(30 * w + i)] = dd[i];
        for (int i = 24; i < 30; ++i) sf.dataBits[size_t(30 * w + i)] = D[i];
        p29 = D[28];
        p30 = D[29];
    }
    sf.tlmMessage = int(sf.bits(9, 14));
    sf.isf = sf.bits(23, 1) != 0;
    sf.towCount = int(sf.bits(31, 17));
    sf.alert = sf.bits(48, 1) != 0;
    sf.antiSpoof = sf.bits(49, 1) != 0;
    sf.id = int(sf.bits(50, 3));
    sf.towStart = sf.towCount * 6.0 - 6.0;
    if (sf.id == 4 || sf.id == 5) {
        sf.dataId = int(sf.bits(61, 2));
        sf.svId = int(sf.bits(63, 6));
        const long long frame = (long long)std::floor(sf.towStart / 30.0);
        sf.page = int(((frame % 25) + 25) % 25) + 1;
    }
}

QHash<QString, double> fieldValues(const Subframe& sf)
{
    QHash<QString, double> h;
    for (const DecodedField& f : decodeFields(sf)) {
        if (f.def.kind == FieldKind::Reserved || f.def.kind == FieldKind::Parity) continue;
        h.insert(f.def.key, f.def.kind == FieldKind::Semicircle ? f.value * kGpsPi : f.value);
    }
    return h;
}

void assembleEphemeris(NavChannel& nc)
{
    std::vector<int> s1, s2, s3;
    for (int i = 0; i < int(nc.subframes.size()); ++i) {
        const Subframe& sf = nc.subframes[size_t(i)];
        if (!sf.allParityOk()) continue;
        if (sf.id == 1) s1.push_back(i);
        else if (sf.id == 2) s2.push_back(i);
        else if (sf.id == 3) s3.push_back(i);
    }
    int pick2 = -1, pick3 = -1, pick1 = -1;
    // newest consistent pair of subframes 2 & 3
    for (int a = int(s2.size()) - 1; a >= 0 && pick2 < 0; --a) {
        const int iode2 = int(nc.subframes[size_t(s2[size_t(a)])].bits(61, 8));
        for (int b = int(s3.size()) - 1; b >= 0; --b) {
            const int iode3 = int(nc.subframes[size_t(s3[size_t(b)])].bits(271, 8));
            if (iode3 == iode2) { pick2 = s2[size_t(a)]; pick3 = s3[size_t(b)]; break; }
        }
    }
    if (pick2 < 0 && !s2.empty()) pick2 = s2.back();
    if (pick3 < 0 && !s3.empty()) pick3 = s3.back();
    const int iode = pick2 >= 0 ? int(nc.subframes[size_t(pick2)].bits(61, 8)) : -1;
    for (int a = int(s1.size()) - 1; a >= 0; --a) {
        const Subframe& sf = nc.subframes[size_t(s1[size_t(a)])];
        const int iodc = int((sf.bits(83, 2) << 8) | sf.bits(211, 8));
        if ((iodc & 0xFF) == iode) { pick1 = s1[size_t(a)]; break; }
    }
    if (pick1 < 0 && !s1.empty()) pick1 = s1.back();

    Ephemeris& e = nc.eph;
    e.prn = nc.prn;
    if (pick1 >= 0) {
        const auto v = fieldValues(nc.subframes[size_t(pick1)]);
        e.sf1 = true; e.sfIndex[1] = pick1;
        e.weekNumber = int(v.value("wn")); e.l2Code = int(v.value("l2code")); e.uraIndex = int(v.value("ura"));
        e.svHealth = int(v.value("health")); e.iodc = int(v.value("iodc")); e.l2pFlag = int(v.value("l2p"));
        e.tgd = v.value("tgd"); e.toc = v.value("toc"); e.af2 = v.value("af2"); e.af1 = v.value("af1"); e.af0 = v.value("af0");
    }
    if (pick2 >= 0) {
        const auto v = fieldValues(nc.subframes[size_t(pick2)]);
        e.sf2 = true; e.sfIndex[2] = pick2;
        e.iode2 = int(v.value("iode")); e.crs = v.value("crs"); e.deltaN = v.value("dn"); e.m0 = v.value("m0");
        e.cuc = v.value("cuc"); e.e = v.value("e"); e.cus = v.value("cus"); e.sqrtA = v.value("sqrta");
        e.toe = v.value("toe"); e.fitFlag = int(v.value("fit")); e.aodo = int(v.value("aodo"));
    }
    if (pick3 >= 0) {
        const auto v = fieldValues(nc.subframes[size_t(pick3)]);
        e.sf3 = true; e.sfIndex[3] = pick3;
        e.cic = v.value("cic"); e.omega0 = v.value("omega0"); e.cis = v.value("cis"); e.i0 = v.value("i0");
        e.crc = v.value("crc"); e.omega = v.value("omega"); e.omegaDot = v.value("omegadot");
        e.iode3 = int(v.value("iode")); e.idot = v.value("idot");
    }
}

} // namespace

NavChannel decodeChannel(const TrackingChannel& trk, const ReceiverSettings& s)
{
    if (isGalileo(trk.prn)) return inav::decodeChannel(trk, s);
    NavChannel nc;
    nc.channel = trk.channel;
    nc.prn = trk.prn;
    nc.eph.prn = trk.prn;
    const int n = trk.numMs;
    if (!trk.valid || n < 2000) {
        nc.status = QStringLiteral("Too little tracking data");
        return nc;
    }

    // ---- bit synchronisation: histogram of prompt sign changes modulo 20 ms
    const int start = std::clamp(s.bitSyncStartMs, 1, n / 2);
    nc.bitSyncStartMs = start;
    for (int k = start + 1; k < n; ++k)
        if ((trk.IP[size_t(k)] >= 0) != (trk.IP[size_t(k - 1)] >= 0)) nc.transitionHist[size_t(k % 20)]++;
    int total = 0, best = 0;
    for (int i = 0; i < 20; ++i) {
        total += nc.transitionHist[size_t(i)];
        if (nc.transitionHist[size_t(i)] > nc.transitionHist[size_t(best)]) best = i;
    }
    int second = 0;
    for (int i = 0; i < 20; ++i)
        if (i != best) second = std::max(second, nc.transitionHist[size_t(i)]);
    nc.bitOffset = best;
    nc.bitSyncConfidence = total > 0 ? double(nc.transitionHist[size_t(best)]) / total : 0.0;
    nc.bitSyncOk = nc.transitionHist[size_t(best)] >= 20 && nc.transitionHist[size_t(best)] > 2 * second;
    nc.firstBitMs = start + ((best - start % 20) + 20) % 20;

    // ---- bit integration (20 ms)
    const int nb = (n - nc.firstBitMs) / kMsPerBit;
    nc.softBits.resize(size_t(std::max(0, nb)));
    nc.hardBits.resize(size_t(std::max(0, nb)));
    double m1 = 0, m2 = 0;
    for (int j = 0; j < nb; ++j) {
        double sum = 0;
        const int k0 = nc.firstBitMs + kMsPerBit * j;
        for (int i = 0; i < kMsPerBit; ++i) sum += trk.IP[size_t(k0 + i)];
        nc.softBits[size_t(j)] = float(sum);
        nc.hardBits[size_t(j)] = sum > 0 ? 1 : 0;
        m1 += std::fabs(sum);
        m2 += sum * sum;
    }
    if (nb > 0) {
        m1 /= nb; m2 /= nb;
        const double var = std::max(1e-30, m2 - m1 * m1);
        const double snr = m1 * m1 / var;
        nc.bitSnrDb = 10.0 * std::log10(snr);
        nc.estimatedBer = 0.5 * std::erfc(std::sqrt(snr / 2.0));
    }
    if (!nc.bitSyncOk) {
        nc.status = QStringLiteral("Bit synchronisation failed");
        return nc;
    }

    // ---- preamble search with parity verification of TLM and HOW
    static const uint8_t pre[8] = {1, 0, 0, 0, 1, 0, 1, 1};
    const int nc8 = std::max(0, nb - 7);
    nc.preambleCorr.resize(size_t(nc8));
    for (int j = 0; j < nc8; ++j) {
        int c = 0;
        for (int i = 0; i < 8; ++i) c += (2 * nc.hardBits[size_t(j + i)] - 1) * (2 * pre[i] - 1);
        nc.preambleCorr[size_t(j)] = c;
        if (std::abs(c) != 8) continue;
        nc.preambleMatches.push_back(j);
        if (j + 60 > nb) continue;
        const int inv = c < 0 ? 1 : 0;
        uint8_t D[60];
        for (int i = 0; i < 60; ++i) D[i] = uint8_t(nc.hardBits[size_t(j + i)] ^ inv);
        const int p29 = j >= 2 ? (nc.hardBits[size_t(j - 2)] ^ inv) : 0;
        const int p30 = j >= 1 ? (nc.hardBits[size_t(j - 1)] ^ inv) : 0;
        uint8_t d1[24], d2[24];
        const bool ok1 = computeParity(D, p29, p30, d1) == pack(D + 24, 6);
        const bool ok2 = computeParity(D + 30, D[28], D[29], d2) == pack(D + 54, 6);
        const int sfid = (d2[19] << 2) | (d2[20] << 1) | d2[21];
        if (ok1 && ok2 && sfid >= 1 && sfid <= 5) nc.preambleVerified.push_back(j);
    }
    if (nc.preambleVerified.empty()) {
        nc.status = QStringLiteral("No verified preamble found");
        return nc;
    }
    std::map<int, int> phaseCount;
    for (int j : nc.preambleVerified) phaseCount[j % kBitsPerSubframe]++;
    int phase = nc.preambleVerified.front() % kBitsPerSubframe, bestCount = 0;
    for (const auto& kv : phaseCount)
        if (kv.second > bestCount) { bestCount = kv.second; phase = kv.first; }
    bool dominantInv = false;
    for (int j : nc.preambleVerified)
        if (j % kBitsPerSubframe == phase) { dominantInv = nc.preambleCorr[size_t(j)] < 0; break; }

    // ---- decode every subframe slot at the verified phase
    for (int j = phase; j + kBitsPerSubframe <= nb; j += kBitsPerSubframe) {
        const int c = j < nc8 ? nc.preambleCorr[size_t(j)] : 0;
        const bool inv = c == -8 ? true : (c == 8 ? false : dominantInv);
        Subframe sf;
        decodeSubframe(nc, j, inv, sf);
        nc.subframes.push_back(sf);
        nc.parityOkWords += sf.parityOkCount;
        nc.parityTotalWords += 10;
    }

    // ---- TOW consistency and timing reference
    int refIdx = -1;
    for (int i = 0; i < int(nc.subframes.size()); ++i) {
        const Subframe& sf = nc.subframes[size_t(i)];
        if (sf.words[0].parityOk && sf.words[1].parityOk && sf.id >= 1 && sf.id <= 5) { refIdx = i; break; }
    }
    if (refIdx >= 0) {
        const Subframe& ref = nc.subframes[size_t(refIdx)];
        nc.timingValid = true;
        nc.refMs = ref.startMs;
        nc.refTow = ref.towStart;
        for (Subframe& sf : nc.subframes) {
            const double expected = nc.refTow + (sf.startMs - nc.refMs) * 1e-3;
            sf.towConsistent = !(sf.words[1].parityOk) || std::fabs(expected - sf.towStart) < 1e-6;
        }
    }
    // Whole subframes only: the 6-bit parity of a single word passes by chance one time in 64 on a lost channel
    for (const Subframe& sf : nc.subframes)
        if (sf.allParityOk())
            for (int w = 0; w < 10; ++w) nc.verifiedMs.push_back(sf.startMs + w * 600);
    assembleEphemeris(nc);
    nc.status = nc.eph.complete() ? QStringLiteral("Ephemeris complete")
                                  : QStringLiteral("Ephemeris incomplete (SF1:%1 SF2:%2 SF3:%3)")
                                        .arg(nc.eph.sf1 ? "ok" : "-").arg(nc.eph.sf2 ? "ok" : "-").arg(nc.eph.sf3 ? "ok" : "-");
    return nc;
}

static void collectGalileo(const NavChannel& nc, GalileoGlobal& g)
{
    const auto signedField = [](const InavWord& w, int start, int len) {
        const quint64 u = w.field(start, len);
        return (u >> (len - 1)) & 1 ? qint64(u) - (qint64(1) << len) : qint64(u);
    };
    for (const InavWord& w : nc.inavWords) {
        if (!w.crcOk) continue;
        if (w.type == 5 && !g.ionoValid) {
            g.ionoValid = true;
            g.sourcePrn = nc.prn;
            g.ai0 = double(w.field(6, 11)) * std::ldexp(1.0, -2);
            g.ai1 = double(signedField(w, 17, 11)) * std::ldexp(1.0, -8);
            g.ai2 = double(signedField(w, 28, 14)) * std::ldexp(1.0, -15);
        } else if (w.type == 6 && !g.utcValid) {
            g.utcValid = true;
            g.A0 = double(signedField(w, 6, 32)) * std::ldexp(1.0, -30);
            g.A1 = double(signedField(w, 38, 24)) * std::ldexp(1.0, -50);
            g.dtLs = int(signedField(w, 62, 8));
            g.t0t = int(w.field(70, 8)) * 3600;
            g.wn0t = int(w.field(78, 8));
            g.wnLsf = int(w.field(86, 8));
            g.dn = int(w.field(94, 3));
            g.dtLsf = int(signedField(w, 97, 8));
        } else if (w.type == 10 && !g.ggtoValid) {
            g.ggtoValid = true;
            g.a0g = double(signedField(w, 86, 16)) * std::ldexp(1.0, -35);
            g.a1g = double(signedField(w, 102, 12)) * std::ldexp(1.0, -51);
            g.t0g = int(w.field(114, 8)) * 3600;
            g.wn0g = int(w.field(122, 6));
        }
    }
}

void mergeGlobal(const GlobalNavData& in, GlobalNavData& t)
{
    for (int sv = 1; sv <= 32; ++sv) {
        if (in.almanac[sv].valid) t.almanac[sv] = in.almanac[sv];
        if (in.health6Valid[sv]) { t.health6[sv] = in.health6[sv]; t.health6Valid[sv] = true; }
        if (in.svConfigValid[sv]) { t.svConfig[sv] = in.svConfig[sv]; t.svConfigValid[sv] = true; }
    }
    if (in.sf5p25) { t.sf5p25 = true; t.p25Toa = in.p25Toa; t.p25Wna = in.p25Wna; }
    t.sf4p25 = t.sf4p25 || in.sf4p25;
    if (in.ionoUtc.valid) t.ionoUtc = in.ionoUtc;
    if (!in.specialMessage.isEmpty()) { t.specialMessage = in.specialMessage; t.specialMessagePrn = in.specialMessagePrn; }
    for (const auto& kv : in.nmct) t.nmct[kv.first] = kv.second;
    if (in.weekNumber >= 0) t.weekNumber = in.weekNumber;
    const GalileoGlobal& g = in.galileo;
    if (g.sourcePrn) t.galileo.sourcePrn = g.sourcePrn;
    if (g.ionoValid) { t.galileo.ionoValid = true; t.galileo.ai0 = g.ai0; t.galileo.ai1 = g.ai1; t.galileo.ai2 = g.ai2; }
    if (g.utcValid) {
        t.galileo.utcValid = true;
        t.galileo.A0 = g.A0; t.galileo.A1 = g.A1; t.galileo.dtLs = g.dtLs; t.galileo.t0t = g.t0t; t.galileo.wn0t = g.wn0t;
        t.galileo.wnLsf = g.wnLsf; t.galileo.dn = g.dn; t.galileo.dtLsf = g.dtLsf;
    }
    if (g.ggtoValid) {
        t.galileo.ggtoValid = true;
        t.galileo.a0g = g.a0g; t.galileo.a1g = g.a1g; t.galileo.t0g = g.t0g; t.galileo.wn0g = g.wn0g;
    }
    for (const PageRecord& p : in.pages) {
        const auto same = [&p](const PageRecord& o) { return o.prn == p.prn && o.subframe == p.subframe && o.tow == p.tow; };
        if (std::find_if(t.pages.begin(), t.pages.end(), same) == t.pages.end()) t.pages.push_back(p);
    }
    const size_t maxPages = 2000;
    if (t.pages.size() > maxPages) t.pages.erase(t.pages.begin(), t.pages.end() - maxPages);
}

void collectGlobal(const std::vector<NavChannel>& channels, GlobalNavData& g)
{
    g = GlobalNavData();
    for (const NavChannel& nc : channels) {
        if (nc.eph.sf1 && g.weekNumber < 0) g.weekNumber = nc.eph.weekNumber % 1024;
        collectGalileo(nc, g.galileo);
        for (const Subframe& sf : nc.subframes) {
            if (sf.id != 4 && sf.id != 5) continue;
            PageRecord pr;
            pr.prn = nc.prn; pr.subframe = sf.id; pr.svId = sf.svId; pr.page = sf.page;
            pr.tow = sf.towStart; pr.parityOk = sf.allParityOk();
            g.pages.push_back(pr);
            if (!sf.allParityOk()) continue;
            const auto v = fieldValues(sf);
            const int sv = sf.svId;
            if ((sf.id == 5 && sv >= 1 && sv <= 24) || (sf.id == 4 && sv >= 25 && sv <= 32)) {
                AlmanacEntry& a = g.almanac[sv];
                a.valid = true; a.svId = sv; a.sourcePrn = nc.prn; a.page = sf.page; a.subframe = sf.id;
                a.health = int(v.value("alm_health"));
                a.e = v.value("alm_e"); a.toa = v.value("alm_toa"); a.deltaI = v.value("alm_di");
                a.omegaDot = v.value("alm_omegadot"); a.sqrtA = v.value("alm_sqrta");
                a.omega0 = v.value("alm_omega0"); a.omega = v.value("alm_omega"); a.m0 = v.value("alm_m0");
                a.af0 = v.value("alm_af0"); a.af1 = v.value("alm_af1");
            } else if (sf.id == 5 && sv == 51) {
                g.sf5p25 = true;
                g.p25Toa = int(v.value("p25_toa"));
                g.p25Wna = int(v.value("p25_wna"));
                for (int k = 1; k <= 24; ++k) {
                    g.health6[k] = int(v.value(QStringLiteral("h_sv%1").arg(k)));
                    g.health6Valid[k] = true;
                }
            } else if (sf.id == 4 && sv == 63) {
                g.sf4p25 = true;
                for (int k = 1; k <= 32; ++k) {
                    g.svConfig[k] = int(v.value(QStringLiteral("cfg_sv%1").arg(k)));
                    g.svConfigValid[k] = true;
                }
                for (int k = 25; k <= 32; ++k) {
                    g.health6[k] = int(v.value(QStringLiteral("h_sv%1").arg(k)));
                    g.health6Valid[k] = true;
                }
            } else if (sf.id == 4 && sv == 56) {
                IonoUtc& u = g.ionoUtc;
                u.valid = true; u.sourcePrn = nc.prn;
                u.alpha[0] = v.value("a0"); u.alpha[1] = v.value("a1"); u.alpha[2] = v.value("a2"); u.alpha[3] = v.value("a3");
                u.beta[0] = v.value("b0"); u.beta[1] = v.value("b1"); u.beta[2] = v.value("b2"); u.beta[3] = v.value("b3");
                u.A0 = v.value("A0"); u.A1 = v.value("A1");
                u.tot = int(v.value("tot")); u.wnt = int(v.value("wnt")); u.dtls = int(v.value("dtls"));
                u.wnlsf = int(v.value("wnlsf")); u.dn = int(v.value("dn")); u.dtlsf = int(v.value("dtlsf"));
            } else if (sf.id == 4 && sv == 55) {
                QString msg;
                for (int k = 1; k <= 22; ++k) {
                    const int c = int(v.value(QStringLiteral("ch%1").arg(k)));
                    msg += (c >= 32 && c < 127) ? QChar(c) : QChar('.');
                }
                g.specialMessage = msg;
                g.specialMessagePrn = nc.prn;
            } else if (sf.id == 4 && sv == 52) {
                std::vector<int> t;
                t.push_back(int(v.value("ai")));
                for (int k = 1; k <= 30; ++k) t.push_back(int(std::lround(v.value(QStringLiteral("erd%1").arg(k)) / 0.3)));
                g.nmct[nc.prn] = t;
            }
        }
    }
}

} // namespace nav
} // namespace gnss
