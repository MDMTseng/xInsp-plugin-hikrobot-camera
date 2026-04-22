//
// test_counter_probe.cpp — figure out which CounterEventSource values
// actually make Counter0 tick on this firmware.
//
// For each candidate source string, rebind + reset the counter, fire a
// known burst, read the counter back, report. The one that matches the
// burst count is the source we should use.
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

#ifndef HIKROBOT_CAMERA_DLL_PATH
#define HIKROBOT_CAMERA_DLL_PATH "hikrobot_camera.dll"
#endif

static bool has_lit(const char* j, const char* k, const char* v) {
    char n[128]; std::snprintf(n, sizeof(n), "\"%s\":%s", k, v);
    return std::strstr(j, n) != nullptr;
}
static int peek_int(const char* j, const char* k) {
    char n[64]; std::snprintf(n, sizeof(n), "\"%s\":", k);
    const char* p = std::strstr(j, n);
    return p ? std::atoi(p + std::strlen(n)) : 0;
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
    auto syms = xi::baseline::load_symbols(dll);
    auto host = xi::ImagePool::make_host_api();
    void* inst = syms.create(&host, "probe_cnt");
    char buf[8192];

    syms.exchange(inst, R"({"command":"set_pixel_format","value":"Mono8"})",             buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"set_roi","x":0,"y":0,"w":640,"h":480})",          buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"set_exposure","value":1000})",                    buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"set_trigger_mode","mode":"on","source":"Line0"})",buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"discover"})",                                      buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"connect","index":0})",                             buf, sizeof(buf));
    if (!has_lit(buf, "connected", "true")) { std::fprintf(stderr, "connect fail\n"); return 1; }

    Serial trig;
    if (!trig.open("COM4", 115200)) { std::fprintf(stderr, "no COM4\n"); return 1; }
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    const char* candidates[] = {
        "Line0",           "Line0RisingEdge", "Line0FallingEdge",
        "Line1",           "Line1RisingEdge", "Line2",
        "FrameStart",      "FrameEnd",        "ExposureStart",
        "ExposureEnd",     "FrameTrigger",    "AcquisitionStart",
        "AcquisitionEnd",  "AcquisitionTrigger",
    };

    const int N = 10;
    for (const char* src : candidates) {
        char cmd[128];
        std::snprintf(cmd, sizeof(cmd),
            R"({"command":"debug_counter_src","value":"%s"})", src);
        syms.exchange(inst, cmd, buf, sizeof(buf));
        bool ok = has_lit(buf, "ok", "true");
        if (!ok) {
            std::printf("  %-22s  rejected by firmware\n", src);
            continue;
        }
        // Fire N edges, wait for them to arrive, then read the counter.
        char fire[80];
        std::snprintf(fire, sizeof(fire), "BURST N=%d HZ=5 WIDTH=500 MASK=0x3", N);
        trig.write_line(fire);
        std::this_thread::sleep_for(std::chrono::milliseconds(N * 200 + 800));
        syms.exchange(inst, R"({"command":"get_edge_counter"})", buf, sizeof(buf));
        int value = peek_int(buf, "value");
        std::printf("  %-22s  ticked=%d  (fired %d edges)\n", src, value, N);
    }

    syms.exchange(inst, R"({"command":"disconnect"})", buf, sizeof(buf));
    syms.destroy(inst);
    return 0;
}
