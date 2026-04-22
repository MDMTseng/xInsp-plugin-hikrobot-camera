//
// test_event_probe.cpp — enumerate camera events that actually fire,
// with TriggerSource=Line0. For each known GenICam/HikRobot event name
// we attempt EventSelector+EventNotification+RegisterCallback, then fire
// a burst via the ESP32 and tally how many times each event fired.
// Also dumps the stat APIs before and after to see which counters move.
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

static int die(const char* w) { std::fprintf(stderr, "FAIL: %s\n", w); return 1; }
static bool has_lit(const char* j, const char* k, const char* v) {
    char n[128]; std::snprintf(n, sizeof(n), "\"%s\":%s", k, v);
    return std::strstr(j, n) != nullptr;
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
    void* inst = syms.create(&host, "evp");
    if (!inst) return die("create");

    char buf[16384];

    // Pre-stage config then connect with TriggerSource=Line0 so trigger
    // events have the best chance of firing (Anyway suppresses them).
    syms.exchange(inst, R"({"command":"set_pixel_format","value":"Mono8"})",             buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"set_roi","x":0,"y":0,"w":640,"h":480})",          buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"set_exposure","value":5000})",                    buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"set_trigger_mode","mode":"on","source":"Line0"})",buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"discover"})", buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"connect","index":0})", buf, sizeof(buf));
    if (!has_lit(buf, "connected", "true")) return die("connect");

    // Probe all known event names.
    const char* probe_cmd = R"({"command":"probe_events","names":[
        "AcquisitionStart","AcquisitionEnd","AcquisitionTrigger","AcquisitionTriggerMiss",
        "FrameStart","FrameEnd",
        "ExposureStart","ExposureEnd",
        "FrameTrigger","FrameTriggerMiss",
        "FrameBurstStart","FrameBurstEnd",
        "LineRisingEdge","LineFallingEdge",
        "Line0RisingEdge","Line0FallingEdge",
        "Line1RisingEdge","Line2RisingEdge",
        "DeviceTemperature","ControlLost","StreamControlLost","Error"
    ]})";
    syms.exchange(inst, probe_cmd, buf, sizeof(buf));
    std::printf("=== probe results (which events this firmware accepts) ===\n%s\n\n", buf);

    // Stat snapshot BEFORE burst.
    syms.exchange(inst, R"({"command":"get_status"})", buf, sizeof(buf));
    std::printf("=== stats BEFORE burst ===\n%s\n\n", buf);

    // Fire 10 hardware triggers at 5 Hz, both channels (in case one
    // cam's Line0 is wired to GPIO32 and the other to GPIO33).
    Serial trig;
    if (!trig.open("COM4", 115200)) return die("open COM4");
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    std::printf("=== firing 10 edges at 5 Hz on MASK=0x3 ===\n\n");
    trig.write_line("BURST N=10 HZ=5 WIDTH=500 MASK=0x3");
    std::this_thread::sleep_for(std::chrono::seconds(4));

    // Read the event counts.
    syms.exchange(inst, R"({"command":"get_event_counts"})", buf, sizeof(buf));
    std::printf("=== event counts after burst ===\n%s\n\n", buf);

    // Stat snapshot AFTER burst.
    syms.exchange(inst, R"({"command":"get_status"})", buf, sizeof(buf));
    std::printf("=== stats AFTER burst ===\n%s\n\n", buf);

    // diag: GenICam node readouts (ResultingFrameRate, throughput, etc.)
    syms.exchange(inst, R"({"command":"diag"})", buf, sizeof(buf));
    std::printf("=== diag (GenICam node readbacks) ===\n%.1500s\n\n", buf);

    syms.exchange(inst, R"({"command":"set_trigger_mode","mode":"off"})", buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"disconnect"})",                     buf, sizeof(buf));
    syms.destroy(inst);
    return 0;
}
