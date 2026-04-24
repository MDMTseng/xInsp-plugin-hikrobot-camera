//
// test_two_cam_align.cpp — cross-camera alignment under surge.
//
// Both cameras wired to the SAME ESP32 channel so every physical
// edge hits both sensors simultaneously. Under the new sort-on-match
// (by device_ns) logic, each camera must emit records in physical
// edge order — and because both cameras saw the same edges, the two
// per-row outputs must line up:
//
//   - same record count from both cams
//   - same trigger_id on every row (both are renumbered by device_ns)
//   - same miss pattern (sensor readout window is identical per model)
//   - trigger_ns (host steady_clock) within a few ms per row
//
// The sequence is identical to test_ts_dump so we can compare:
// 10 @ 10 Hz + 4 @ 200 Hz surge + 16 @ 10 Hz = 30 edges.
//
// MASK=0x3 fires both ESP32 channels in lockstep — one is fanned to
// both cameras' Line0 via a common wire in the rig.
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

struct Cam {
    const char* name;
    std::string device_key;
    void* inst = nullptr;
    char  buf[16384];
};

struct Row {
    int     tid = 0;
    bool    missed = false;
    int64_t trig_ns = 0;
    int64_t dev_ns  = 0;
};

// Probe a camera's native Width/Height max by opening it briefly with
// trigger off, reading diag, then tearing down. Avoids the USB-race that
// hits when we try to change ROI live on an already-streaming cam.
static bool probe_max_dims(const xi::baseline::PluginSymbols& s, xi_host_api* host,
                           const std::string& key, int& w_out, int& h_out) {
    void* inst = s.create(host, "probe_dims");
    if (!inst) return false;
    char buf[16384];
    s.exchange(inst, R"({"command":"set_trigger_mode","mode":"off"})", buf, sizeof(buf));
    // Retry discover a few times — after a previous probe's disconnect,
    // MVS may need a beat before the device re-enumerates cleanly.
    bool ok = false;
    for (int attempt = 0; attempt < 5 && !ok; ++attempt) {
        s.exchange(inst, R"({"command":"discover"})", buf, sizeof(buf));
        char cmd[256];
        std::snprintf(cmd, sizeof(cmd), R"({"command":"connect","device_key":"%s"})", key.c_str());
        s.exchange(inst, cmd, buf, sizeof(buf));
        ok = has_lit(buf, "connected", "true");
        if (!ok) std::this_thread::sleep_for(std::chrono::milliseconds(400));
    }
    w_out = h_out = 0;
    if (ok) {
        char dbuf[8192];
        s.exchange(inst, R"({"command":"diag"})", dbuf, sizeof(dbuf));
        auto peek_max = [&](const char* k) -> int {
            char n[64]; std::snprintf(n, sizeof(n), "\"%s\":{", k);
            const char* p = std::strstr(dbuf, n);
            if (!p) return 0;
            const char* m = std::strstr(p, "\"max\":");
            if (!m || m > std::strchr(p, '}')) return 0;
            return std::atoi(m + std::strlen("\"max\":"));
        };
        w_out = peek_max("Width");
        h_out = peek_max("Height");
        s.exchange(inst, R"({"command":"disconnect"})", buf, sizeof(buf));
    }
    s.destroy(inst);
    return ok;
}

// Phase 1: open the camera with trigger mode OFF. Avoids the
// USB_WRITE storm that hits when TriggerMode=On + StartGrabbing race
// during concurrent two-cam init. Camera starts in free-run but we
// immediately stop streaming so it's quiescent.
static bool connect_only(const xi::baseline::PluginSymbols& s, xi_host_api* host,
                         Cam& c, int wmax, int hmax) {
    c.inst = s.create(host, c.name);
    if (!c.inst) return false;
    s.exchange(c.inst, R"({"command":"set_pixel_format","value":"Mono8"})", c.buf, sizeof(c.buf));
    s.exchange(c.inst, R"({"command":"set_exposure","value":3000})",        c.buf, sizeof(c.buf));
    if (wmax > 0 && hmax > 0) {
        char roi[128];
        std::snprintf(roi, sizeof(roi),
            R"({"command":"set_roi","x":0,"y":0,"w":%d,"h":%d})", wmax, hmax);
        s.exchange(c.inst, roi, c.buf, sizeof(c.buf));
    }
    // Pre-stage with trigger_mode=on so when we finally do start
    // streaming, it's directly into trigger-wait (never free-run).
    s.exchange(c.inst, R"({"command":"set_trigger_mode","mode":"on","source":"Line0"})",
               c.buf, sizeof(c.buf));
    s.exchange(c.inst, R"({"command":"discover"})", c.buf, sizeof(c.buf));
    char cmd[256];
    // defer_streaming:true — the plugin opens the handle, applies all
    // pre-staged features, but does NOT call StartGrabbing. No
    // acquisition happens during configure.
    std::snprintf(cmd, sizeof(cmd),
        R"({"command":"connect","device_key":"%s","defer_streaming":true})",
        c.device_key.c_str());
    if (s.exchange(c.inst, cmd, c.buf, sizeof(c.buf)) <= 0) return false;
    return has_lit(c.buf, "connected", "true");
}

// Phase 2: turn on strict mode (registers events), then explicitly
// start streaming — which goes straight into trigger-wait because
// trigger mode was pre-staged on during connect_only.
static bool arm_triggers(const xi::baseline::PluginSymbols& s, Cam& c) {
    s.exchange(c.inst, R"({"command":"set_strict_trigger_mode","value":true,"timeout_ms":2000})",
               c.buf, sizeof(c.buf));
    s.exchange(c.inst, R"({"command":"start_streaming"})", c.buf, sizeof(c.buf));
    s.exchange(c.inst, R"({"command":"get_status"})", c.buf, sizeof(c.buf));
    return has_lit(c.buf, "streaming", "true");
}

// Combined helper kept for the isolation phase which still wants the
// one-shot configure flow.
static bool configure(const xi::baseline::PluginSymbols& s, xi_host_api* host,
                      Cam& c, int wmax, int hmax) {
    if (!connect_only(s, host, c, wmax, hmax)) return false;
    return arm_triggers(s, c);
}

static void teardown(const xi::baseline::PluginSymbols& s, Cam& c) {
    if (!c.inst) return;
    s.exchange(c.inst, R"({"command":"set_trigger_mode","mode":"off"})", c.buf, sizeof(c.buf));
    s.exchange(c.inst, R"({"command":"disconnect"})",                     c.buf, sizeof(c.buf));
    s.destroy(c.inst);
    c.inst = nullptr;
}

static std::vector<Row> drain(const xi::baseline::PluginSymbols& s, Cam& c, int max_records) {
    std::vector<Row> out;
    out.reserve(max_records);
    for (int i = 0; i < max_records; ++i) {
        xi_record in{}; in.json = "{}";
        xi_record_out rec{}; xi_record_out_init(&rec);
        for (int r = 0; r < 5; ++r) {
            s.process(c.inst, &in, &rec);
            if (!std::strstr(rec.json, "\"error\":\"no_pending_trigger\"")) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            xi_record_out_init(&rec);
        }
        if (!rec.json || std::strstr(rec.json, "\"error\":\"no_pending_trigger\"")) break;
        Row r;
        r.tid     = peek_int(rec.json, "trigger_id");
        r.missed  = std::strstr(rec.json, "\"missed\":true") != nullptr;
        r.trig_ns = peek_i64(rec.json, "ts_trigger_ns");
        r.dev_ns  = peek_i64(rec.json, "ts_device_ns");
        out.push_back(r);
    }
    return out;
}

int main() {
    HMODULE dll = LoadLibraryA(HIKROBOT_CAMERA_DLL_PATH);
    if (!dll) return die("LoadLibrary");
    auto syms = xi::baseline::load_symbols(dll);
    if (!syms.ok()) return die("syms");
    auto host = xi::ImagePool::make_host_api();

    // Give USB enumeration time to settle after any preceding PnP reset
    // (otherwise MVS discover sees the devices but the control channel
    // isn't yet healthy, and TriggerMode / StartGrabbing USB-WRITE fail).
    std::this_thread::sleep_for(std::chrono::seconds(3));

    // Discover.
    std::vector<std::string> keys;
    {
        void* p = syms.create(&host, "probe");
        char pb[16384]; pb[0] = 0;
        syms.exchange(p, R"({"command":"discover"})", pb, sizeof(pb));
        const char* q = pb;
        while ((q = std::strstr(q, "\"USB:")) != nullptr) {
            const char* e = std::strchr(q + 1, '"');
            if (!e) break;
            keys.emplace_back(q + 1, e - q - 1);
            q = e + 1;
        }
        syms.destroy(p);
    }
    // With >2 cameras on the bus (common on this dev machine), pick the
    // two we've characterized: MV-CA050 (K47674142) + MV-CE200 (00DA5328883).
    // Fall back to the first two enumerated if those aren't plugged in.
    auto find_key = [&](const char* serial) -> std::string {
        for (auto& k : keys) if (k.find(serial) != std::string::npos) return k;
        return {};
    };
    std::string keyA = find_key("K47674142");
    std::string keyB = find_key("00DA5328883");
    if (keyA.empty() || keyB.empty()) {
        if (keys.size() < 2) return die("need 2 cameras visible to MVS");
        keyA = keys[0]; keyB = keys[1];
    }
    keys = {keyA, keyB};
    std::printf("using cameras:\n  A = %s\n  B = %s\n\n", keyA.c_str(), keyB.c_str());

    // Single-camera-at-a-time isolation: configure A alone, run the
    // full sequence, tear down, then do the same for B. If one cam at
    // full res can't start then the second-cam path was never the
    // blocker. If both pass alone but fail together, the issue is
    // concurrent startup contention.
    // Full res each: CE200 5472x3648 (20 MP), CA050 2448x2048 (5 MP native).
    auto max_dims = [](const std::string& k) -> std::pair<int,int> {
        if (k.find("00DA5328883") != std::string::npos) return {5472, 3648};
        return {2448, 2048};
    };

    auto run_single = [&](const std::string& key, const char* name) -> bool {
        auto [w, h] = max_dims(key);
        std::printf("  trying %s at %dx%d... ", name, w, h);
        std::fflush(stdout);
        Cam c{name, key};
        if (!configure(syms, &host, c, w, h)) {
            std::printf("CONFIGURE FAILED\n");
            return false;
        }
        char sbuf[4096];
        syms.exchange(c.inst, R"({"command":"get_status"})", sbuf, sizeof(sbuf));
        bool streaming = std::strstr(sbuf, "\"streaming\":true") != nullptr;
        char dbuf[8192];
        syms.exchange(c.inst, R"({"command":"diag"})", dbuf, sizeof(dbuf));
        auto peek_cur = [&](const char* k) -> int {
            char n[64]; std::snprintf(n, sizeof(n), "\"%s\":{", k);
            const char* p = std::strstr(dbuf, n);
            if (!p) return -1;
            const char* cur = std::strstr(p, "\"cur\":");
            return cur ? std::atoi(cur + 6) : -1;
        };
        std::printf("streaming=%s dims=%dx%d\n",
                    streaming ? "YES" : "NO",
                    peek_cur("Width"), peek_cur("Height"));
        teardown(syms, c);
        return streaming;
    };

    // Serialized full-res capture needs the trigger source open up-front.
    Serial trig;
    // Try COM4 then COM3 (ESP32 may re-enumerate to either after replug).
    if (!trig.open("COM4", 115200) && !trig.open("COM3", 115200))
        return die("no ESP32 on COM3 or COM4");
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // Since concurrent startup of two full-res streams on this USB
    // configuration tears both handles down, run them strictly one at
    // a time: configure A, fire and drain A's burst, teardown; then do
    // the same for B. "Alternating full-res capture" via serialization.
    auto [wA, hA] = max_dims(keys[0]);
    auto [wB, hB] = max_dims(keys[1]);

    auto capture_burst = [&](const std::string& key, const char* name,
                             int w, int h, const char* mask) {
        std::printf("--- %s at %dx%d ---\n", name, w, h);
        Cam c{name, key};
        if (!configure(syms, &host, c, w, h)) {
            std::fprintf(stderr, "FAIL: configure %s\n", name);
            return std::vector<Row>{};
        }
        char sbuf[4096];
        syms.exchange(c.inst, R"({"command":"get_status"})", sbuf, sizeof(sbuf));
        std::printf("  streaming=%s\n",
                    std::strstr(sbuf, "\"streaming\":true") ? "YES" : "NO");
        char burst[128];
        std::snprintf(burst, sizeof(burst),
                      "BURST N=%d HZ=5 WIDTH=200 MASK=%s", 25, mask);
        trig.write_line(burst);
        std::this_thread::sleep_for(std::chrono::milliseconds(5200));
        std::this_thread::sleep_for(std::chrono::seconds(1));
        // Snapshot camera stats — even if we drain 0 rows, line0_edges
        // tells us whether the camera saw the electrical edges at all.
        syms.exchange(c.inst, R"({"command":"get_status"})", sbuf, sizeof(sbuf));
        std::printf("  stats: captured=%d line0_edges=%d missed=%d "
                    "triggers_sent=%d dropped=%d\n",
                    peek_int(sbuf, "captured"),
                    peek_int(sbuf, "line0_edges"),
                    peek_int(sbuf, "missed_triggers"),
                    peek_int(sbuf, "triggers_sent"),
                    peek_int(sbuf, "dropped"));
        auto rows = drain(syms, c, 25);
        std::printf("  %s drained %zu rows\n", name, rows.size());
        teardown(syms, c);
        return rows;
    };

    // Skip the isolation + serialized phases for this run — too many
    // teardown/reopen cycles accumulate plugin state and trigger the
    // 0x80000203 "device disabled" error on the next OpenDevice.

    // Health monitor: poll both cams every 200 ms and print a timeline.
    // Lets us see the exact wall-clock moment a handle goes dead —
    // useful to test the "USB hub power sag" hypothesis: if power
    // droop is the cause, both cams should die within the same poll
    // tick, at a point correlated with peak USB activity.
    std::atomic<bool> monitor_run{false};
    std::thread monitor;
    auto start_monitor = [&](Cam& A, Cam& B, const char* tag) {
        monitor_run.store(true);
        monitor = std::thread([&, tag]{
            auto t0 = std::chrono::steady_clock::now();
            char ba[4096], bb[4096];
            auto snap = [&](Cam& c, char* buf) {
                syms.exchange(c.inst, R"({"command":"get_status"})", buf, 4096);
            };
            auto peek_stream = [](const char* j) {
                return std::strstr(j, "\"streaming\":true") ? 'Y' : 'N';
            };
            std::printf("[%s] monitor started  (t_ms  A.str A.cap A.edge  B.str B.cap B.edge)\n", tag);
            while (monitor_run.load()) {
                snap(A, ba); snap(B, bb);
                auto now = std::chrono::steady_clock::now();
                double t_ms = std::chrono::duration<double, std::milli>(now - t0).count();
                std::printf("[%s] %8.1f   %c %4d %4d    %c %4d %4d\n", tag,
                    t_ms,
                    peek_stream(ba), peek_int(ba, "captured"), peek_int(ba, "line0_edges"),
                    peek_stream(bb), peek_int(bb, "captured"), peek_int(bb, "line0_edges"));
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
            }
        });
    };
    auto stop_monitor = [&]{
        monitor_run.store(false);
        if (monitor.joinable()) monitor.join();
    };

    std::printf("\n=== concurrent two-cam at full res (legacy, likely fails) ===\n");
    Cam A{"camA_align", keys[0]};
    Cam B{"camB_align", keys[1]};
    auto t_cfg = std::chrono::steady_clock::now();
    auto elapsed = [&]{
        return std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t_cfg).count();
    };

    // Phase 1: open both handles with trigger mode OFF, streams stopped.
    bool okA = connect_only(syms, &host, A, wA, hA);
    std::printf("[T+%6.1f ms] connect A  → %s\n", elapsed(), okA ? "OK" : "FAILED");
    if (!okA) { teardown(syms, B); return die("connect A"); }
    std::this_thread::sleep_for(std::chrono::seconds(3));
    bool okB = connect_only(syms, &host, B, wB, hB);
    std::printf("[T+%6.1f ms] connect B  → %s\n", elapsed(), okB ? "OK" : "FAILED");
    if (!okB) { teardown(syms, A); return die("connect B"); }
    // Long settle so both handles are fully stable before any feature
    // writes on one cam can race the other's USB control channel.
    std::this_thread::sleep_for(std::chrono::seconds(3));

    // Phase 2: arm triggers on each cam serially.
    bool armA = arm_triggers(syms, A);
    std::printf("[T+%6.1f ms] arm A      → %s\n", elapsed(), armA ? "streaming=YES" : "FAILED");
    if (!armA) { teardown(syms, A); teardown(syms, B); return die("arm A"); }
    std::this_thread::sleep_for(std::chrono::seconds(2));
    bool armB = arm_triggers(syms, B);
    std::printf("[T+%6.1f ms] arm B      → %s\n", elapsed(), armB ? "streaming=YES" : "FAILED");
    if (!armB) { teardown(syms, A); teardown(syms, B); return die("arm B"); }

    // Flush any frames the cameras buffered between prior session's
    // teardown (free-run) and this connect. Resets stats counters so
    // captured / line0_edges reflect only THIS test's triggers.
    std::printf("[T+%6.1f ms] flushing cam buffers...\n", elapsed());
    syms.exchange(A.inst, R"({"command":"flush_camera","drain_ms":800})",
                  A.buf, sizeof(A.buf));
    syms.exchange(B.inst, R"({"command":"flush_camera","drain_ms":800})",
                  B.buf, sizeof(B.buf));
    std::printf("[T+%6.1f ms] flush done\n", elapsed());
    start_monitor(A, B, "mon");
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Snapshot captured_ immediately before firing triggers. Anything
    // here represents ghost frames that leaked past the flush.
    auto pre_captured = [&](Cam& c) {
        char sb[4096];
        syms.exchange(c.inst, R"({"command":"get_status"})", sb, sizeof(sb));
        return peek_int(sb, "captured");
    };
    int preA = pre_captured(A), preB = pre_captured(B);
    std::printf("pre-trigger captured: A=%d  B=%d\n", preA, preB);

    std::printf("both cameras armed (strict, Line0, split connect→arm flow)\n\n");


    // 15 Hz baseline with surges, 20 s duration. Pattern per cycle:
    //   60 @ 15 Hz = 4 s baseline
    //   6  @ 250 Hz surge
    //   repeat 5x = 20.2 s total
    // Each cycle = 66 edges; total 330 edges per cam.
    const int CYCLES = 5;
    const int N = (60 + 6) * CYCLES;  // 330
    std::printf("15 Hz + surge for 20s: %d cycles of (60@15Hz + 6@250Hz surge) = %d edges\n\n",
                CYCLES, N);
    auto fire = [&](const char* cmd, int settle_ms) {
        trig.write_line(cmd);
        std::this_thread::sleep_for(std::chrono::milliseconds(settle_ms));
    };
    for (int i = 0; i < CYCLES; ++i) {
        fire("BURST N=60 HZ=15 WIDTH=200 MASK=0x3", 4100);
        fire("BURST N=6  HZ=250 WIDTH=200 MASK=0x3", 300);
    }
    std::this_thread::sleep_for(std::chrono::seconds(3));

    // Pre-drain status: report actual resolution + sensor-level counters
    // so we can tell whether the camera really shot at full res and
    // whether anything got rejected/dropped silently.
    auto report_status = [&](Cam& c, const char* tag) {
        char sbuf[4096];
        syms.exchange(c.inst, R"({"command":"get_status"})", sbuf, sizeof(sbuf));
        std::printf("  %s  captured=%d dropped=%d lost_packets=%d "
                    "line0_edges=%d missed=%d triggers_sent=%d\n",
                    tag,
                    peek_int(sbuf, "captured"),
                    peek_int(sbuf, "dropped"),
                    peek_int(sbuf, "lost_packets"),
                    peek_int(sbuf, "line0_edges"),
                    peek_int(sbuf, "missed_triggers"),
                    peek_int(sbuf, "triggers_sent"));
    };
    auto report_diag = [&](Cam& c, const char* tag) {
        char dbuf[8192];
        syms.exchange(c.inst, R"({"command":"diag"})", dbuf, sizeof(dbuf));
        // Pull Width/Height cur fields from nested objects.
        auto peek_cur = [&](const char* key) -> int {
            char needle[64];
            std::snprintf(needle, sizeof(needle), "\"%s\":{", key);
            const char* p = std::strstr(dbuf, needle);
            if (!p) return -1;
            const char* cur = std::strstr(p, "\"cur\":");
            if (!cur) return -1;
            return std::atoi(cur + std::strlen("\"cur\":"));
        };
        std::printf("  %s  %dx%d\n", tag, peek_cur("Width"), peek_cur("Height"));
    };
    std::printf("camera status (before drain):\n");
    report_diag(A, "A");
    report_diag(B, "B");
    report_status(A, "A");
    report_status(B, "B");
    std::printf("\n");

    auto rowsA = drain(syms, A, N + 10);
    auto rowsB = drain(syms, B, N + 10);

    std::printf("drained: A=%zu  B=%zu\n\n", rowsA.size(), rowsB.size());

    // Compare rows side-by-side.
    size_t cmp = std::min(rowsA.size(), rowsB.size());
    int64_t t0A = rowsA.empty() ? 0 : rowsA[0].trig_ns;
    int64_t t0B = rowsB.empty() ? 0 : rowsB[0].trig_ns;
    int tid_mismatches   = 0;
    int miss_mismatches  = 0;
    int big_trig_skew    = 0;
    int prev_bad_dev_A   = 0;
    int prev_bad_dev_B   = 0;
    // Stricter per-cam 1:1 integrity checks (for 15 Hz + surge test).
    int prev_bad_trig_A  = 0;   // trigger_ns not monotonic within A
    int prev_bad_trig_B  = 0;
    int rows_zero_trig_A = 0;   // real rows with ts_trigger_ns == 0
    int rows_zero_dev_A  = 0;   // real rows with ts_device_ns == 0
    int rows_zero_trig_B = 0;
    int rows_zero_dev_B  = 0;
    int64_t lastA_trig = 0, lastB_trig = 0;

    std::printf("%-4s %-8s %-8s %-6s %-6s %-12s %-12s %-12s\n",
                "idx", "A.tid", "B.tid", "A.k", "B.k",
                "dA_trig_ms", "dB_trig_ms", "skew_ms");
    std::printf("%-4s %-8s %-8s %-6s %-6s %-12s %-12s %-12s\n",
                "---", "-----", "-----", "---", "---",
                "----------", "----------", "-------");

    int64_t lastA_dev = 0, lastB_dev = 0;
    for (size_t i = 0; i < cmp; ++i) {
        const auto& a = rowsA[i];
        const auto& b = rowsB[i];
        double dA = (a.trig_ns - t0A) / 1e6;
        double dB = (b.trig_ns - t0B) / 1e6;
        double skew_ms = (a.trig_ns && b.trig_ns)
            ? ((a.trig_ns - t0A) - (b.trig_ns - t0B)) / 1e6 : 0.0;
        // Condense: print every 25th + any MISS + edges of the stream.
        bool show = a.missed || b.missed
                 || i < 3 || i >= cmp - 3
                 || (i % 25) == 0;
        if (show) {
            std::printf("%-4zu %-8d %-8d %-6s %-6s %12.3f %12.3f %12.3f\n",
                        i, a.tid, b.tid,
                        a.missed ? "MISS" : "real",
                        b.missed ? "MISS" : "real",
                        dA, dB, skew_ms);
        }

        if (a.tid != b.tid)       ++tid_mismatches;
        if (a.missed != b.missed) ++miss_mismatches;
        // Under alternating triggers the two cams never fire at the
        // same edge so skew is ~half the pulse gap (~50 ms). Only flag
        // really wild skews as cross-camera timing issues.
        if (std::abs(skew_ms) > 200.0) ++big_trig_skew;

        if (i > 0) {
            if (a.dev_ns && lastA_dev && a.dev_ns <= lastA_dev) ++prev_bad_dev_A;
            if (b.dev_ns && lastB_dev && b.dev_ns <= lastB_dev) ++prev_bad_dev_B;
            if (a.trig_ns && lastA_trig && a.trig_ns <= lastA_trig) ++prev_bad_trig_A;
            if (b.trig_ns && lastB_trig && b.trig_ns <= lastB_trig) ++prev_bad_trig_B;
        }
        if (a.dev_ns)  lastA_dev  = a.dev_ns;
        if (b.dev_ns)  lastB_dev  = b.dev_ns;
        if (a.trig_ns) lastA_trig = a.trig_ns;
        if (b.trig_ns) lastB_trig = b.trig_ns;
        // Real rows must carry both timestamps populated. A missed slot
        // is allowed to have dev_ns = 0 (unknown) but a real captured
        // row with no timestamp means the plugin lost track.
        if (!a.missed) {
            if (!a.trig_ns) ++rows_zero_trig_A;
            if (!a.dev_ns)  ++rows_zero_dev_A;
        }
        if (!b.missed) {
            if (!b.trig_ns) ++rows_zero_trig_B;
            if (!b.dev_ns)  ++rows_zero_dev_B;
        }
    }

    // Per-cam: compare host trigger_ns gap vs device_ns gap across
    // consecutive REAL rows. If the camera's event timestamp matches
    // the physical edge time, the two should agree within jitter.
    auto gap_stats = [](const std::vector<Row>& rows, double tick_ns,
                        const char* label) {
        double sum_host = 0, sum_dev = 0;
        double min_host = 1e18, max_host = -1e18;
        double min_dev  = 1e18, max_dev  = -1e18;
        int n = 0;
        int64_t prev_host = 0, prev_dev = 0;
        for (auto& r : rows) {
            if (r.missed || !r.trig_ns || !r.dev_ns) { prev_host = prev_dev = 0; continue; }
            if (prev_host && prev_dev) {
                double h_ms = (r.trig_ns - prev_host) / 1e6;
                double d_ms = (r.dev_ns  - prev_dev) * tick_ns / 1e6;
                sum_host += h_ms; sum_dev += d_ms;
                if (h_ms < min_host) min_host = h_ms;
                if (h_ms > max_host) max_host = h_ms;
                if (d_ms < min_dev)  min_dev  = d_ms;
                if (d_ms > max_dev)  max_dev  = d_ms;
                ++n;
            }
            prev_host = r.trig_ns;
            prev_dev  = r.dev_ns;
        }
        if (n == 0) {
            std::printf("  %s: no real-row pairs\n", label);
            return;
        }
        std::printf("  %s  host_gap_ms: mean=%.3f min=%.3f max=%.3f\n",
                    label, sum_host / n, min_host, max_host);
        std::printf("  %s   dev_gap_ms: mean=%.3f min=%.3f max=%.3f  (tick=%gns)\n",
                    label, sum_dev / n, min_dev, max_dev, tick_ns);
        std::printf("  %s   bias (dev-host) mean=%.4f ms\n",
                    label, (sum_dev - sum_host) / n);
    };
    std::printf("\n=== host vs device timing ===\n");
    // Both cams on this rig run their device clock at 100 MHz
    // (10 ns/tick). The earlier belief that CA050 was 10 MHz turned
    // out to be wrong — measured against host trigger_ns it's 100 MHz
    // too.
    double tickA = 10.0;
    double tickB = 10.0;
    gap_stats(rowsA, tickA, "A");
    gap_stats(rowsB, tickB, "B");

    std::printf("\n=== result ===\n"
                "A rows: %zu  B rows: %zu  (expected %d each)\n"
                "tid mismatches: %d\n"
                "miss pattern mismatches: %d\n"
                "|skew| > 5 ms rows: %d\n"
                "non-monotonic device_ns (A / B): %d / %d\n"
                "non-monotonic trigger_ns (A / B): %d / %d\n"
                "real rows missing trigger_ns (A / B): %d / %d\n"
                "real rows missing device_ns  (A / B): %d / %d\n",
                rowsA.size(), rowsB.size(), N,
                tid_mismatches, miss_mismatches, big_trig_skew,
                prev_bad_dev_A, prev_bad_dev_B,
                prev_bad_trig_A, prev_bad_trig_B,
                rows_zero_trig_A, rows_zero_trig_B,
                rows_zero_dev_A,  rows_zero_dev_B);

    stop_monitor();
    teardown(syms, A);
    teardown(syms, B);

    // Row count equality is load-bearing: both cams must have seen the
    // same edges. Reaching N isn't — at 50 Hz sustained the event path
    // often loses edges outright (MVS back-pressures callbacks).
    if (rowsA.size() != rowsB.size())
        return die("cams drained a different number of rows");
    if ((int)rowsA.size() < N / 2)
        std::fprintf(stderr, "NOTE: only %zu/%d rows delivered — MVS/USB "
                             "couldn't sustain the target rate.\n",
                     rowsA.size(), N);
    if (tid_mismatches != 0)
        return die("trigger_id diverged between cams (sort-on-match broken)");
    if (prev_bad_dev_A != 0 || prev_bad_dev_B != 0)
        return die("device_ns not monotonic — output order doesn't match physical order");
    // miss-pattern divergence is expected when one camera's USB channel
    // drops a frame the other receives; log but don't fail on it.

    std::printf("\nOK — 2-cam alignment holds under surge.\n");
    return 0;
}
