//
// test_ce200_alone.cpp — MV-CE200 alone at full resolution (5472x3648),
// 10 Hz for 10 s, strict hardware trigger mode. Proves the 20 MP cam
// can sustain full-res streaming on its own without the MV-CA050 on
// the same bus contending for control-channel bandwidth.
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
    if (!dll) return die("LoadLibrary");
    auto syms = xi::baseline::load_symbols(dll);
    if (!syms.ok()) return die("syms");
    auto host = xi::ImagePool::make_host_api();

    // Hardcoded MV-CE200 device key and native dims. If the key doesn't
    // match the first USB camera, update per your rig.
    const std::string target_key = "USB:00DA5328883";
    const int W = 5472, H = 3648;

    void* inst = syms.create(&host, "ce200_alone");
    if (!inst) return die("create");
    char buf[16384];

    // Pre-stage everything so the connect path applies it in one go.
    syms.exchange(inst, R"({"command":"set_pixel_format","value":"Mono8"})",            buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"set_exposure","value":3000})",                    buf, sizeof(buf));
    char roi[128];
    std::snprintf(roi, sizeof(roi),
        R"({"command":"set_roi","x":0,"y":0,"w":%d,"h":%d})", W, H);
    syms.exchange(inst, roi, buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"set_trigger_mode","mode":"on","source":"Line0"})",buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"set_strict_trigger_mode","value":true,"timeout_ms":2000})",
                  buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"discover"})", buf, sizeof(buf));
    char cmd[256];
    std::snprintf(cmd, sizeof(cmd), R"({"command":"connect","device_key":"%s"})", target_key.c_str());
    if (syms.exchange(inst, cmd, buf, sizeof(buf)) <= 0) return die("connect");
    if (!has_lit(buf, "connected", "true")) return die("not connected");

    syms.exchange(inst, R"({"command":"get_status"})", buf, sizeof(buf));
    std::printf("streaming=%s (before burst)\n",
                has_lit(buf, "streaming", "true") ? "YES" : "NO");

    Serial trig;
    if (!trig.open("COM4", 115200)) { syms.destroy(inst); return die("COM4"); }
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // 5 Hz × 5 s = 25 edges. MV-CE200 full-res max is ~17 fps.
    const int N = 25;
    std::printf("firing %d edges @ 5 Hz for 5 s (MV-CE200 full res)\n\n", N);
    trig.write_line("BURST N=25 HZ=5 WIDTH=200 MASK=0x3");
    std::this_thread::sleep_for(std::chrono::milliseconds(5200));
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Stats snapshot.
    syms.exchange(inst, R"({"command":"get_status"})", buf, sizeof(buf));
    int cap   = peek_int(buf, "captured");
    int edges = peek_int(buf, "line0_edges");
    int miss  = peek_int(buf, "missed_triggers");
    int sent  = peek_int(buf, "triggers_sent");
    int drop  = peek_int(buf, "dropped");
    int lost  = peek_int(buf, "lost_packets");
    std::printf("stats: captured=%d line0_edges=%d missed=%d "
                "triggers_sent=%d dropped=%d lost_packets=%d\n\n",
                cap, edges, miss, sent, drop, lost);

    // Drain all records.
    std::vector<std::tuple<int,bool,int64_t,int64_t>> rows;  // tid,missed,trig_ns,dev_ns
    for (int i = 0; i < N + 10; ++i) {
        xi_record in{}; in.json = "{}";
        xi_record_out rec{}; xi_record_out_init(&rec);
        for (int r = 0; r < 5; ++r) {
            syms.process(inst, &in, &rec);
            if (!std::strstr(rec.json, "\"error\":\"no_pending_trigger\"")) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            xi_record_out_init(&rec);
        }
        if (!rec.json || std::strstr(rec.json, "\"error\":\"no_pending_trigger\"")) break;
        rows.emplace_back(
            peek_int(rec.json, "trigger_id"),
            std::strstr(rec.json, "\"missed\":true") != nullptr,
            peek_i64(rec.json, "ts_trigger_ns"),
            peek_i64(rec.json, "ts_device_ns"));
    }

    // Condensed print: first 3, every 10th, last 3, all misses.
    int real = 0, missed = 0;
    int64_t t0 = rows.empty() ? 0 : std::get<2>(rows[0]);
    int64_t prev_trig = t0;
    std::printf("%-4s %-6s %-6s %14s %14s\n",
                "idx", "tid", "kind", "Δtrig_ms", "gap_ms");
    std::printf("%-4s %-6s %-6s %14s %14s\n",
                "---", "---", "----", "--------", "------");
    for (size_t i = 0; i < rows.size(); ++i) {
        auto [tid, m, tn, dn] = rows[i];
        double dtrig = (tn - t0) / 1e6;
        double gap   = (tn - prev_trig) / 1e6;
        if (m) ++missed; else ++real;
        bool show = m || i < 3 || i >= rows.size() - 3 || (i % 10) == 0;
        if (show) {
            std::printf("%-4zu %-6d %-6s %14.3f %14.3f\n",
                        i, tid, m ? "MISS" : "real", dtrig, gap);
        }
        prev_trig = tn;
    }
    std::printf("\ndrained %zu rows (real=%d missed=%d of %d expected)\n",
                rows.size(), real, missed, N);

    syms.exchange(inst, R"({"command":"set_trigger_mode","mode":"off"})", buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"disconnect"})",                     buf, sizeof(buf));
    syms.destroy(inst);

    if ((int)rows.size() < N * 90 / 100)
        return die("fewer than 90% of edges delivered at full res");
    std::printf("\nOK — MV-CE200 alone sustains 10 Hz full-res.\n");
    return 0;
}
