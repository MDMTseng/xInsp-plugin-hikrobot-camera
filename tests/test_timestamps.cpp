//
// test_timestamps.cpp — verify the four timestamp streams.
//
// Fires N triggers from the ESP32 at a known HZ. For each drained
// process() output collects ts_trigger_ns, ts_received_ns, tstats_host_ns,
// ts_device_ns. Reports interval statistics (mean / min / max / stddev)
// vs the expected 1/HZ interval.
//
// A healthy plugin should show:
//   - trigger intervals close to 1/HZ (≤ a few ms jitter)
//   - received intervals close to trigger intervals (latency ~ constant)
//   - device intervals identical to trigger intervals (camera's own clock)
//   - host intervals similar but possibly noisier
//

#include <xi/xi_abi.hpp>
#include <xi/xi_baseline.hpp>
#include <xi/xi_image_pool.hpp>

#include <windows.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#ifndef HIKROBOT_CAMERA_DLL_PATH
#define HIKROBOT_CAMERA_DLL_PATH "hikrobot_camera.dll"
#endif

static int die(const char* w) { std::fprintf(stderr, "FAIL: %s\n", w); return 1; }

static bool has_lit(const char* j, const char* k, const char* v) {
    char n[128]; std::snprintf(n, sizeof(n), "\"%s\":%s", k, v);
    return std::strstr(j, n) != nullptr;
}
static int64_t peek_i64(const char* j, const char* k) {
    char n[64]; std::snprintf(n, sizeof(n), "\"%s\":", k);
    const char* p = std::strstr(j, n);
    if (!p) return 0;
    return (int64_t)std::atoll(p + std::strlen(n));
}

class Serial {
public:
    bool open(const char* port, int baud) {
        char n[32]; std::snprintf(n, sizeof(n), "\\\\.\\%s", port);
        h_ = CreateFileA(n, GENERIC_READ|GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, 0, nullptr);
        if (h_ == INVALID_HANDLE_VALUE) return false;
        DCB d{}; d.DCBlength = sizeof(d); GetCommState(h_, &d);
        d.BaudRate = baud; d.ByteSize = 8; d.Parity = NOPARITY; d.StopBits = ONESTOPBIT;
        d.fDtrControl = DTR_CONTROL_DISABLE; d.fRtsControl = RTS_CONTROL_DISABLE;
        SetCommState(h_, &d);
        COMMTIMEOUTS t{}; t.ReadIntervalTimeout = MAXDWORD; t.ReadTotalTimeoutConstant = 200;
        SetCommTimeouts(h_, &t);
        return true;
    }
    void write_line(const std::string& s) {
        std::string x = s + "\n";
        DWORD w; WriteFile(h_, x.data(), (DWORD)x.size(), &w, nullptr);
    }
    ~Serial() { if (h_ != INVALID_HANDLE_VALUE) CloseHandle(h_); }
private:
    HANDLE h_ = INVALID_HANDLE_VALUE;
};

struct Stats {
    double mean_ms, min_ms, max_ms, stddev_ms;
};

static Stats interval_stats(const std::vector<int64_t>& ts) {
    std::vector<double> intervals_ms;
    intervals_ms.reserve(ts.size());
    for (size_t i = 1; i < ts.size(); ++i) {
        intervals_ms.push_back((ts[i] - ts[i-1]) / 1e6);
    }
    if (intervals_ms.empty()) return {0,0,0,0};
    double sum = 0;
    double mn = intervals_ms[0], mx = intervals_ms[0];
    for (double v : intervals_ms) { sum += v; if (v < mn) mn = v; if (v > mx) mx = v; }
    double mean = sum / intervals_ms.size();
    double var = 0;
    for (double v : intervals_ms) var += (v - mean) * (v - mean);
    var /= intervals_ms.size();
    return { mean, mn, mx, std::sqrt(var) };
}

int main() {
    HMODULE dll = LoadLibraryA(HIKROBOT_CAMERA_DLL_PATH);
    if (!dll) return die("LoadLibrary");
    auto syms = xi::baseline::load_symbols(dll);
    if (!syms.ok()) return die("syms");
    auto host = xi::ImagePool::make_host_api();

    void* inst = syms.create(&host, "ts");
    if (!inst) return die("create");
    char buf[16384];

    syms.exchange(inst, R"({"command":"set_pixel_format","value":"Mono8"})",             buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"set_roi","x":0,"y":0,"w":640,"h":480})",          buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"set_exposure","value":1000})",                    buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"set_trigger_mode","mode":"on","source":"Line0"})",buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"set_strict_trigger_mode","value":true,"timeout_ms":3000})",
                  buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"discover"})", buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"connect","index":0})", buf, sizeof(buf));
    if (!has_lit(buf, "connected", "true")) return die("connect");

    Serial trig;
    if (!trig.open("COM4", 115200)) return die("open COM4");
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    const int    N            = 20;
    const int    HZ           = 10;
    const double expected_ms  = 1000.0 / HZ;

    std::printf("firing %d triggers at %d Hz (expected interval %.2f ms)…\n",
                N, HZ, expected_ms);
    char fire[80];
    std::snprintf(fire, sizeof(fire), "BURST N=%d HZ=%d WIDTH=200 MASK=0x3", N, HZ);
    trig.write_line(fire);
    std::this_thread::sleep_for(std::chrono::seconds(N / HZ + 2));

    std::vector<int64_t> ts_trig, ts_rx, tstats_host, ts_dev;
    for (int i = 0; i < N; ++i) {
        xi_record in{}; in.json = "{}";
        xi_record_out out{}; xi_record_out_init(&out);
        for (int r = 0; r < 5; ++r) {
            syms.process(inst, &in, &out);
            if (!std::strstr(out.json, "\"error\":\"no_pending_trigger\"")) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            xi_record_out_init(&out);
        }
        if (!out.json || std::strstr(out.json, "\"missed\":true")) continue;
        ts_trig.push_back(peek_i64(out.json, "ts_trigger_ns"));
        ts_rx  .push_back(peek_i64(out.json, "ts_received_ns"));
        tstats_host.push_back(peek_i64(out.json, "tstats_host_ns"));
        ts_dev .push_back(peek_i64(out.json, "ts_device_ns"));
    }

    auto s_trig = interval_stats(ts_trig);
    auto s_rx   = interval_stats(ts_rx);
    auto stats_host = interval_stats(tstats_host);
    auto s_dev  = interval_stats(ts_dev);

    auto print_row = [&](const char* name, Stats s) {
        std::printf("  %-12s mean=%7.2f ms  min=%7.2f  max=%7.2f  stddev=%6.2f  err=%+6.2f ms\n",
                    name, s.mean_ms, s.min_ms, s.max_ms, s.stddev_ms,
                    s.mean_ms - expected_ms);
    };
    std::printf("\nexpected interval: %.2f ms (from %d Hz request to ESP32)\n\n", expected_ms, HZ);
    std::printf("interval stats across %zu intervals between captured frames:\n", ts_trig.size() - 1);
    print_row("trigger",  s_trig);
    print_row("received", s_rx);
    print_row("host",     stats_host);
    print_row("device",   s_dev);

    syms.exchange(inst, R"({"command":"set_trigger_mode","mode":"off"})", buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"disconnect"})",                     buf, sizeof(buf));
    syms.destroy(inst);

    // Quality gate. trigger/received are in steady_clock ns (what we
    // control) — require them within 5% of expected. host_ns is never
    // populated by this firmware (always 0), device_ns is in
    // camera-specific tick units (10 MHz = 100ns on MV-CA050), so
    // neither is directly comparable to wall-time ms without per-camera
    // calibration — see README caveat on timestamps.
    auto within = [&](double x, double tol_pct) {
        return std::fabs(x - expected_ms) <= expected_ms * (tol_pct / 100.0);
    };
    if (!within(s_trig.mean_ms, 5)) return die("trigger interval > 5% off expected");
    if (!within(s_rx.mean_ms,   5)) return die("received interval > 5% off expected");
    std::printf("\nOK — trigger/received timestamps match ESP32 firing rate within 5%%.\n");
    return 0;
}
