//
// test_ts_dump.cpp — fire 30 triggers at 10 Hz, dump each record's
// timestamps normalized to the first frame so the offsets are readable.
//
//  idx  trigger_id  status  Δtrigger_ns (from idx 0)  Δdevice_ns (from idx 0)  gap_trig gap_dev
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

int main() {
    HMODULE dll = LoadLibraryA(HIKROBOT_CAMERA_DLL_PATH);
    auto syms  = xi::baseline::load_symbols(dll);
    auto host  = xi::ImagePool::make_host_api();
    void* inst = syms.create(&host, "tsdump");
    if (!inst) return die("create");
    char buf[16384];

    syms.exchange(inst, R"({"command":"set_pixel_format","value":"Mono8"})",             buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"set_exposure","value":3000})",                    buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"set_trigger_mode","mode":"on","source":"Line0"})",buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"set_strict_trigger_mode","value":true,"timeout_ms":2000})",
                  buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"discover"})", buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"connect","index":0})", buf, sizeof(buf));
    if (!has_lit(buf, "connected", "true")) return die("connect");

    Serial trig;
    if (!trig.open("COM4", 115200)) return die("COM4");
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // Sequence: 10 @ 10Hz  +  4 @ 200Hz (surge)  +  16 @ 10Hz  = 30 total.
    const int N = 30;
    std::printf("firing 30 edges: 10 @ 10Hz + 4 @ 200Hz (surge) + 16 @ 10Hz"
                " (full-res, 3ms exposure)\n\n");
    trig.write_line("BURST N=10 HZ=10 WIDTH=200 MASK=0x3");
    std::this_thread::sleep_for(std::chrono::milliseconds(1100));   // 10 * 100ms + slack
    trig.write_line("BURST N=4 HZ=200 WIDTH=200 MASK=0x3");
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    trig.write_line("BURST N=16 HZ=10 WIDTH=200 MASK=0x3");
    std::this_thread::sleep_for(std::chrono::seconds(2));

    struct Row {
        int tid; bool missed; int64_t trig_ns, dev_ns;
    };
    std::vector<Row> rows;
    rows.reserve(N);
    for (int i = 0; i < N; ++i) {
        xi_record in{}; in.json = "{}";
        xi_record_out out{}; xi_record_out_init(&out);
        for (int r = 0; r < 5; ++r) {
            syms.process(inst, &in, &out);
            if (!std::strstr(out.json, "\"error\":\"no_pending_trigger\"")) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            xi_record_out_init(&out);
        }
        if (!out.json || std::strstr(out.json, "\"error\":\"no_pending_trigger\"")) break;
        Row r;
        r.tid     = peek_int(out.json, "trigger_id");
        r.missed  = std::strstr(out.json, "\"missed\":true") != nullptr;
        r.trig_ns = peek_i64(out.json, "ts_trigger_ns");
        r.dev_ns  = peek_i64(out.json, "ts_device_ns");
        rows.push_back(r);
    }

    if (rows.empty()) return die("no rows");

    int64_t t0 = rows[0].trig_ns;
    int64_t d0 = rows[0].dev_ns;
    std::printf("normalized to first frame (trigger_ns and device_ns both offset-zero'd)\n\n");
    std::printf("%-4s %-10s %-6s %16s %16s %10s %10s\n",
                "idx", "trigger_id", "kind",
                "Δtrig (ms)", "Δdev (ms*)", "gap_trig", "gap_dev");
    std::printf("%-4s %-10s %-6s %16s %16s %10s %10s\n",
                "---", "----------", "------",
                "----------", "----------", "--------", "-------");
    int64_t prev_trig = t0, prev_dev = d0;
    // MV-CE200 clock = 100 MHz → 10 ns/tick; MV-CA050 = 10 MHz → 100 ns/tick.
    // We normalize but print the raw delta, annotated as "ticks" since
    // we don't know which camera without probing. "*ms" in the header is
    // a reminder that the device delta is in *camera ticks*, not ms.
    for (size_t i = 0; i < rows.size(); ++i) {
        double dtrig_ms = (rows[i].trig_ns - t0) / 1e6;
        double ddev     = (double)(rows[i].dev_ns  - d0);   // ticks, not ms
        double gap_trig = (rows[i].trig_ns - prev_trig) / 1e6;
        double gap_dev  = (double)(rows[i].dev_ns  - prev_dev);
        std::printf("%-4zu %-10d %-6s %16.3f %16.0f %10.3f %10.0f\n",
                    i, rows[i].tid, rows[i].missed ? "MISS" : "real",
                    dtrig_ms, ddev, gap_trig, gap_dev);
        prev_trig = rows[i].trig_ns;
        prev_dev  = rows[i].dev_ns;
    }

    syms.exchange(inst, R"({"command":"set_trigger_mode","mode":"off"})", buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"disconnect"})",                     buf, sizeof(buf));
    syms.destroy(inst);

    std::printf("\nnotes:\n"
                "  Δtrig is host steady_clock ns (→ ms); gap_trig same unit.\n"
                "  Δdev  is camera-clock ticks (MV-CE200 = 10ns/tick, MV-CA050 = 100ns/tick).\n"
                "  For MV-CE200:  Δdev ticks × 10 ns → wall ms = Δdev / 1e5\n");
    return 0;
}
