// Stands in for sm_api.dll, to test the Signal Hound path without the instrument. It delivers 50 MS/s of 32-bit complex
// floats in real time, made from a file of complex int8 samples at 10 MS/s by repeating each five times and shifting it.
//
//   FAKE_SM_FILE       the file
//   FAKE_SM_CENTRE_HZ  frequency the file is centred on
//   FAKE_SM_NO_DEVICE  when set, opening fails as it does with nothing connected
//
// Repeating samples leaves images of the band 10 MHz away, which the splitter's filter has to remove.

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <thread>
#include <vector>

#define API extern "C" __declspec(dllexport)

namespace {

constexpr double kRate = 50e6;
constexpr double kScale = 1e-6;    // file counts to square root of milliwatts: -120 dBm per count
std::FILE* g_file = nullptr;
double g_center = 0, g_shift = 0;
long long g_sent = 0;
std::chrono::steady_clock::time_point g_start;

} // namespace

API int smOpenDevice(int* device)
{
    if (std::getenv("FAKE_SM_NO_DEVICE")) return -2;
    const char* name = std::getenv("FAKE_SM_FILE");
    if (!name || !(g_file = std::fopen(name, "rb"))) return -2;
    *device = 0;
    g_sent = 0;
    return 0;
}
API int smCloseDevice(int) { if (g_file) std::fclose(g_file); g_file = nullptr; return 0; }
API int smSetRefLevel(int, double) { return 0; }
API int smSetIQCenterFreq(int, double f) { g_center = f; return 0; }
API int smSetIQBaseSampleRate(int, int) { return 0; }
API int smSetIQSampleRate(int, int) { return 0; }
API int smSetIQBandwidth(int, int, double) { return 0; }
API int smSetIQDataType(int, int) { return 0; }
API int smConfigure(int, int)
{
    const char* c = std::getenv("FAKE_SM_CENTRE_HZ");
    g_shift = (c ? std::atof(c) : 0.0) - g_center;
    g_start = std::chrono::steady_clock::now();
    return 0;
}
API int smGetIQParameters(int, double* rate, double* bw) { *rate = kRate; *bw = 41.5e6; return 0; }
API const char* smGetErrorString(int) { return "Unable to open device"; }

API int smGetIQ(int, void* buf, int n, double*, int, int64_t* ns, int, int* loss, int* remaining)
{
    std::this_thread::sleep_until(g_start + std::chrono::nanoseconds((long long)((g_sent + n) / kRate * 1e9)));
    float* out = static_cast<float*>(buf);
    static int8_t held[2];
    const double w = 2 * 3.14159265358979323846 * g_shift / kRate;
    double c = std::cos(w * double(g_sent)), s = std::sin(w * double(g_sent));
    const double cd = std::cos(w), sd = std::sin(w);
    for (int i = 0; i < n; ++i) {
        if ((g_sent + i) % 5 == 0 && std::fread(held, 1, 2, g_file) != 2) return -3;
        const double x = held[0] * kScale, y = held[1] * kScale;
        out[2 * i] = float(x * c - y * s);
        out[2 * i + 1] = float(x * s + y * c);
        const double nc = c * cd - s * sd;
        s = c * sd + s * cd;
        c = nc;
    }
    if (ns) *ns = (long long)(g_sent / kRate * 1e9);
    if (loss) *loss = 0;
    if (remaining) *remaining = 0;
    g_sent += n;
    return 0;
}
