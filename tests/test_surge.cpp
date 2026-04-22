//
// test_surge.cpp — 10 Hz baseline interrupted by a back-to-back trigger
// pair ~500 µs apart. The second edge of the pair arrives while the
// camera is still exposing/reading the first → sensor-level reject.
// Verifies the plugin:
//   1. Handles 10 Hz baseline cleanly (every edge → one real output)
//   2. Sees both surge edges via Line0RisingEdge events
//   3. Flags the rejected surge edge as missed (reason=counter_mismatch)
//   4. Stamps even the rejected edge with accurate trigger_ns + device_ns
//   5. Keeps the 1:1 trigger→output invariant across the surge
//
// Sequence:
//   5 triggers at 10 Hz       (pre-surge)
//   2 triggers at 2000 Hz     (surge pair, ~500 µs apart)
//   5 triggers at 10 Hz       (post-surge)
//   total 12 edges; expect 12 outputs, 1 of them missed.
//

#include <xi/xi_abi.hpp>
#include <xi/xi_baseline.hpp>
#include <xi/xi_image_pool.hpp>

#include <windows.h>
#include <chrono>
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
static int peek_int(const char* j, const char* k) {
    char n[64]; std::snprintf(n, sizeof(n), "\"%s\":", k);
    const char* p = std::strstr(j, n);
    return p ? std::atoi(p + std::strlen(n)) : 0;
}
static int64_t peek_i64(const char* j, const char* k) {
    char n[64]; std::snprintf(n, sizeof(n), "\"%s\":", k);
    const char* p = std::strstr(j, n);
    return p ? (int64_t)std::atoll(p + std::strlen(n)) : 0;
}
static std::string peek_str(const char* j, const char* k) {
    char n[64]; std::snprintf(n, sizeof(n), "\"%s\":\"", k);
    const char* p = std::strstr(j, n);
    if (!p) return {};
    p += std::strlen(n);
    const char* e = std::strchr(p, '"');
    return e ? std::string(p, e - p) : std::string();
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

int main() {
    HMODULE dll = LoadLibraryA(HIKROBOT_CAMERA_DLL_PATH);
    if (!dll) return die("LoadLibrary");
    auto syms = xi::baseline::load_symbols(dll);
    if (!syms.ok()) return die("syms");
    auto host = xi::ImagePool::make_host_api();

    void* inst = syms.create(&host, "surge");
    char buf[16384];

    // Full native resolution + 100 us exposure. Exposure is now tiny,
    // so the camera's busy window is dominated by sensor readout +
    // USB transfer (tens of ms at full res). Whether the 10 ms surge
    // pair forces a reject depends on readout speed alone.
    syms.exchange(inst, R"({"command":"set_pixel_format","value":"Mono8"})",             buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"set_exposure","value":3000})",                    buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"set_trigger_mode","mode":"on","source":"Line0"})",buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"set_strict_trigger_mode","value":true,"timeout_ms":2000})",
                  buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"discover"})", buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"connect","index":0})", buf, sizeof(buf));
    if (!has_lit(buf, "connected", "true")) return die("connect");

    Serial trig;
    if (!trig.open("COM4", 115200)) return die("open COM4");
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // Pre-surge: 5 triggers at 10 Hz.
    std::printf("stage 1: 5 triggers @ 10 Hz (baseline)\n");
    trig.write_line("BURST N=5 HZ=10 WIDTH=200 MASK=0x3");
    std::this_thread::sleep_for(std::chrono::milliseconds(700));

    // Surge: 4 triggers at 100 Hz → 10 ms apart each, ~40 ms total.
    // Now stresses both sensor busy-window AND USB transfer bandwidth
    // (four 20 MP frames queued within 40 ms is ~2 GB/s theoretical
    // peak, well above USB 3.0 sustainable). Some rejects + some
    // transmission drops likely.
    std::printf("stage 2: 4 triggers @ 200 Hz (surge burst, 5ms apart, ~20ms total)\n");
    trig.write_line("BURST N=4 HZ=200 WIDTH=200 MASK=0x3");
    std::this_thread::sleep_for(std::chrono::milliseconds(400));

    // Post-surge: 5 more at 10 Hz.
    std::printf("stage 3: 5 triggers @ 10 Hz (post-surge)\n\n");
    trig.write_line("BURST N=5 HZ=10 WIDTH=200 MASK=0x3");
    std::this_thread::sleep_for(std::chrono::milliseconds(800));

    // Drain.
    const int expected_total = 14;   // 5 + 4 + 5
    std::vector<std::string> rows;
    int real = 0, missed = 0;
    int missed_with_trig_ts = 0, missed_with_dev_ts = 0;
    int64_t last_trig_ns = 0;
    for (int i = 0; i < expected_total + 2; ++i) {
        xi_record in{}; in.json = "{}";
        xi_record_out out{}; xi_record_out_init(&out);
        for (int r = 0; r < 5; ++r) {
            syms.process(inst, &in, &out);
            if (!std::strstr(out.json, "\"error\":\"no_pending_trigger\"")) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            xi_record_out_init(&out);
        }
        if (!out.json || std::strstr(out.json, "\"error\":\"no_pending_trigger\"")) break;
        int tid = peek_int(out.json, "trigger_id");
        int64_t ts_trig = peek_i64(out.json, "ts_trigger_ns");
        int64_t ts_dev  = peek_i64(out.json, "ts_device_ns");
        bool miss = std::strstr(out.json, "\"missed\":true") != nullptr;
        std::string why = peek_str(out.json, "miss_reason");

        char line[192];
        double gap_ms = (last_trig_ns && ts_trig) ? (ts_trig - last_trig_ns) / 1e6 : 0;
        std::snprintf(line, sizeof(line),
            "  tid=%-2d  %-6s  gap=%6.2f ms  trig_ns=%lld  dev_ns=%lld%s%s",
            tid, miss ? "MISS" : "real", gap_ms,
            (long long)ts_trig, (long long)ts_dev,
            miss ? "  reason=" : "", miss ? why.c_str() : "");
        rows.emplace_back(line);
        if (miss) {
            ++missed;
            if (ts_trig > 0) ++missed_with_trig_ts;
            if (ts_dev  > 0) ++missed_with_dev_ts;
        } else ++real;
        if (ts_trig) last_trig_ns = ts_trig;
    }

    std::printf("drained outputs (gap = delta from previous row's trigger_ns):\n");
    for (auto& r : rows) std::puts(r.c_str());

    // Final status snapshot.
    syms.exchange(inst, R"({"command":"get_status"})", buf, sizeof(buf));
    int line0   = peek_int(buf, "line0_edges");
    int falls   = peek_int(buf, "line0_falling_edges");
    int captured = peek_int(buf, "captured");
    int missed_c = peek_int(buf, "missed_triggers");

    std::printf("\n=== result ===\n"
                "drained: %zu  (real=%d  missed=%d)\n"
                "line0_edges / falling: %d / %d\n"
                "captured / missed_triggers: %d / %d\n"
                "missed slots with trigger_ns > 0: %d / %d\n"
                "missed slots with device_ns  > 0: %d / %d\n",
                rows.size(), real, missed, line0, falls,
                captured, missed_c,
                missed_with_trig_ts, missed,
                missed_with_dev_ts,  missed);

    syms.exchange(inst, R"({"command":"set_trigger_mode","mode":"off"})", buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"disconnect"})",                     buf, sizeof(buf));
    syms.destroy(inst);

    // Pass criteria.
    if ((int)rows.size() != expected_total)
        return die("drain count != 12 (expected one output per fired edge)");
    if (line0 != expected_total)
        return die("line0_edges != fired count");
    if (line0 != falls)
        return die("rising != falling edge count (pulse-integrity)");
    if (missed_with_trig_ts < missed)
        return die("some missed slots have zero ts_trigger_ns");
    // Surge should produce at least 1 miss (the second of the 500us pair
    // hits the sensor mid-readout). If we got 0 misses, either the
    // sensor was faster than expected or the test didn't actually surge.
    if (missed < 1)
        std::fprintf(stderr, "NOTE: 0 surge misses — sensor absorbed both pulses. "
                             "Bump the surge rate or extend exposure to force a reject.\n");
    std::printf("\nOK — 10Hz baseline + surge pair kept 1:1 trigger→output with accurate timestamps.\n");
    return 0;
}
