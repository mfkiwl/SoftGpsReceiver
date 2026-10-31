// A stand-in for uhd.dll, for testing the USRP path without a radio. It exports the same C API (the structures below
// are copied from the UHD headers) and streams a complex 16-bit file at its real rate, stamping packets with their
// time like a device does. It also checks what the receiver asks for, and can simulate lost packets.
//
//   FAKE_UHD_FILE     file of interleaved int16 I/Q
//   FAKE_UHD_RATE     its sample rate in Hz (the rate any request is coerced to)
//   FAKE_UHD_DROP_AT  seconds into the stream: report an overflow and lose samples there (several, comma separated),
//                     as when the computer stops reading for a moment
//   FAKE_UHD_DROP_MS  how much is lost each time
//   FAKE_UHD_BLANK_AT, FAKE_UHD_BLANK_SEC  replace the signal by noise for a while, as with the antenna disconnected
//   FAKE_UHD_NO_DEVICE  when set, opening the device fails as it does with no USRP on the network

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#define API extern "C" __declspec(dllexport)

using uhd_error = int;
enum { UHD_ERROR_NONE = 0, UHD_ERROR_RUNTIME = 41 };

typedef struct { char* cpu_format; char* otw_format; char* args; size_t* channel_list; int n_channels; } uhd_stream_args_t;
typedef struct { int stream_mode; size_t num_samps; bool stream_now; int64_t time_spec_full_secs; double time_spec_frac_secs; } uhd_stream_cmd_t;
typedef struct { double target_freq; int rf_freq_policy; double rf_freq; int dsp_freq_policy; double dsp_freq; char* args; } uhd_tune_request_t;
typedef struct { double clipped_rf_freq; double target_rf_freq; double actual_rf_freq; double target_dsp_freq; double actual_dsp_freq; } uhd_tune_result_t;

namespace {

struct Device
{
    double rate = 1e6, freq = 0, gain = 0;
    std::string antenna, clock;
    FILE* file = nullptr;
    bool streaming = false;
    std::chrono::steady_clock::time_point start;
    long long sent = 0;                 // samples consumed from the file, including lost ones
    std::vector<double> dropAt;
    double dropSec = 0;
    size_t nextDrop = 0;
    double blankAt = -1, blankSec = 0;
    unsigned noise = 12345;
};

struct Metadata { int code = 0; bool hasTime = false; int64_t full = 0; double frac = 0; };

Device g;
std::string lastError;

uhd_error failure(const std::string& what)
{
    lastError = what;
    return UHD_ERROR_RUNTIME;
}

uhd_error copyString(const std::string& s, char* out, size_t len)
{
    if (len == 0) return UHD_ERROR_NONE;
    std::strncpy(out, s.c_str(), len - 1);
    out[len - 1] = 0;
    return UHD_ERROR_NONE;
}

} // namespace

API uhd_error uhd_get_last_error(char* out, size_t len) { return copyString(lastError, out, len); }
API uhd_error uhd_usrp_last_error(void*, char* out, size_t len) { return copyString(lastError, out, len); }
API uhd_error uhd_get_version_string(char* out, size_t len) { return copyString("fake-1.0", out, len); }

API uhd_error uhd_usrp_make(void** h, const char* args)
{
    std::printf("FAKE_UHD: usrp_make args=\"%s\"\n", args);
    if (std::getenv("FAKE_UHD_NO_DEVICE")) return failure("LookupError: KeyError: No devices found for ----->\nDevice Address:\n    addr: 192.168.10.2");
    const char* name = std::getenv("FAKE_UHD_FILE");
    if (!name) return failure("FAKE_UHD_FILE not set");
    g.file = std::fopen(name, "rb");
    if (!g.file) return failure(std::string("cannot open ") + name);
    if (const char* r = std::getenv("FAKE_UHD_RATE")) g.rate = std::atof(r);
    if (const char* d = std::getenv("FAKE_UHD_DROP_AT")) {
        std::string s(d);
        size_t pos = 0;
        while (pos < s.size()) {
            g.dropAt.push_back(std::atof(s.c_str() + pos));
            pos = s.find(',', pos);
            if (pos == std::string::npos) break;
            ++pos;
        }
    }
    if (const char* d = std::getenv("FAKE_UHD_DROP_MS")) g.dropSec = std::atof(d) * 1e-3;
    if (const char* d = std::getenv("FAKE_UHD_BLANK_AT")) g.blankAt = std::atof(d);
    if (const char* d = std::getenv("FAKE_UHD_BLANK_SEC")) g.blankSec = std::atof(d);
    *h = &g;
    return UHD_ERROR_NONE;
}

API uhd_error uhd_usrp_free(void** h)
{
    if (g.file) std::fclose(g.file);
    g.file = nullptr;
    *h = nullptr;
    return UHD_ERROR_NONE;
}

API uhd_error uhd_usrp_set_rx_rate(void*, double rate, size_t chan)
{
    std::printf("FAKE_UHD: set_rx_rate %.1f chan %zu\n", rate, chan);
    return UHD_ERROR_NONE;
}
API uhd_error uhd_usrp_get_rx_rate(void*, size_t, double* out) { *out = g.rate; return UHD_ERROR_NONE; }

API uhd_error uhd_usrp_set_rx_freq(void*, uhd_tune_request_t* req, size_t, uhd_tune_result_t* res)
{
    std::printf("FAKE_UHD: tune target %.1f rf policy %c rf %.1f dsp policy %c args \"%s\"\n", req->target_freq,
                req->rf_freq_policy, req->rf_freq, req->dsp_freq_policy, req->args ? req->args : "(null)");
    g.freq = req->target_freq;
    res->target_rf_freq = res->actual_rf_freq = req->target_freq;
    return UHD_ERROR_NONE;
}
API uhd_error uhd_usrp_get_rx_freq(void*, size_t, double* out) { *out = g.freq; return UHD_ERROR_NONE; }

API uhd_error uhd_usrp_set_rx_gain(void*, double gain, size_t, const char* name)
{
    std::printf("FAKE_UHD: gain %.1f name \"%s\"\n", gain, name);
    g.gain = gain;
    return UHD_ERROR_NONE;
}
API uhd_error uhd_usrp_get_rx_gain(void*, size_t, const char*, double* out) { *out = g.gain; return UHD_ERROR_NONE; }
API uhd_error uhd_usrp_set_rx_antenna(void*, const char* a, size_t) { std::printf("FAKE_UHD: antenna %s\n", a); return UHD_ERROR_NONE; }
API uhd_error uhd_usrp_set_rx_bandwidth(void*, double, size_t) { return UHD_ERROR_NONE; }
API uhd_error uhd_usrp_set_clock_source(void*, const char* c, size_t) { std::printf("FAKE_UHD: clock %s\n", c); return UHD_ERROR_NONE; }
API uhd_error uhd_usrp_set_time_now(void*, int64_t, double, size_t) { return UHD_ERROR_NONE; }
API uhd_error uhd_usrp_get_pp_string(void*, char* out, size_t len) { return copyString("  Device: Fake USRP2/N200\n  RX dboard: none", out, len); }
API uhd_error uhd_usrp_set_rx_subdev_spec(void*, void*, size_t) { return UHD_ERROR_NONE; }
API uhd_error uhd_subdev_spec_make(void** h, const char* m) { std::printf("FAKE_UHD: subdev %s\n", m); *h = &g; return UHD_ERROR_NONE; }
API uhd_error uhd_subdev_spec_free(void** h) { *h = nullptr; return UHD_ERROR_NONE; }

API uhd_error uhd_rx_streamer_make(void** h) { *h = &g; return UHD_ERROR_NONE; }
API uhd_error uhd_rx_streamer_free(void** h) { *h = nullptr; return UHD_ERROR_NONE; }
API uhd_error uhd_rx_metadata_make(void** h) { *h = new Metadata; return UHD_ERROR_NONE; }
API uhd_error uhd_rx_metadata_free(void** h) { delete static_cast<Metadata*>(*h); *h = nullptr; return UHD_ERROR_NONE; }
API uhd_error uhd_rx_metadata_error_code(void* h, int* out) { *out = static_cast<Metadata*>(h)->code; return UHD_ERROR_NONE; }
API uhd_error uhd_rx_metadata_has_time_spec(void* h, bool* out) { *out = static_cast<Metadata*>(h)->hasTime; return UHD_ERROR_NONE; }
API uhd_error uhd_rx_metadata_time_spec(void* h, int64_t* full, double* frac)
{
    *full = static_cast<Metadata*>(h)->full;
    *frac = static_cast<Metadata*>(h)->frac;
    return UHD_ERROR_NONE;
}

API uhd_error uhd_usrp_get_rx_stream(void*, uhd_stream_args_t* a, void*)
{
    std::printf("FAKE_UHD: stream cpu=%s otw=%s args=\"%s\" channels=%d chan0=%zu\n", a->cpu_format, a->otw_format, a->args,
                a->n_channels, a->channel_list[0]);
    if (std::string(a->cpu_format) != "sc16") return failure("expected sc16");
    return UHD_ERROR_NONE;
}
API uhd_error uhd_rx_streamer_max_num_samps(void*, size_t* out) { *out = 363; return UHD_ERROR_NONE; }

API uhd_error uhd_rx_streamer_issue_stream_cmd(void*, const uhd_stream_cmd_t* cmd)
{
    std::printf("FAKE_UHD: stream command '%c' now=%d\n", cmd->stream_mode, cmd->stream_now);
    g.streaming = cmd->stream_mode == 'a';
    g.start = std::chrono::steady_clock::now();
    g.sent = 0;
    return UHD_ERROR_NONE;
}

API uhd_error uhd_rx_streamer_recv(void*, void** buffs, size_t samps, void** md, double timeout, bool, size_t* got)
{
    Metadata& m = *static_cast<Metadata*>(*md);
    *got = 0;
    m.code = 0;
    m.hasTime = false;
    const double now = g.sent / g.rate;
    if (g.nextDrop < g.dropAt.size() && now >= g.dropAt[g.nextDrop]) {
        ++g.nextDrop;
        const long long lost = (long long)(g.dropSec * g.rate);
        std::this_thread::sleep_for(std::chrono::duration<double>(g.dropSec));
        _fseeki64(g.file, lost * 4, SEEK_CUR);
        g.sent += lost;
        m.code = 0x8;                      // overflow
        return UHD_ERROR_NONE;
    }
    // The samples of this buffer exist once real time has passed them
    const auto due = g.start + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                   std::chrono::duration<double>((g.sent + (long long)samps) / g.rate));
    const auto wait = due - std::chrono::steady_clock::now();
    if (wait > std::chrono::duration<double>(timeout)) {
        std::this_thread::sleep_for(std::chrono::duration<double>(timeout));
        m.code = 0x1;                      // timeout
        return UHD_ERROR_NONE;
    }
    if (wait.count() > 0) std::this_thread::sleep_until(due);
    const size_t n = std::fread(buffs[0], 4, samps, g.file);
    if (n == 0) {
        std::this_thread::sleep_for(std::chrono::duration<double>(timeout));
        m.code = 0x1;
        return UHD_ERROR_NONE;
    }
    const double t = g.sent / g.rate;
    if (g.blankAt >= 0 && t >= g.blankAt && t < g.blankAt + g.blankSec) {
        int16_t* s = static_cast<int16_t*>(buffs[0]);
        for (size_t i = 0; i < 2 * n; ++i) {
            g.noise = g.noise * 1103515245u + 12345u;
            s[i] = int16_t(int((g.noise >> 16) & 0x7FF) - 1024);
        }
    }
    m.hasTime = true;
    m.full = (int64_t)t;
    m.frac = t - (double)m.full;
    g.sent += (long long)n;
    *got = n;
    return UHD_ERROR_NONE;
}
