//
// test_trigger_sweep.cpp — probe which TriggerSource + ESP32 channel
// combo actually wakes a HikRobot camera up.
//
// For each camera, we set TriggerSource to Line0 / Line1 / Line2 / Line3
// in turn, then fire 5 pulses on ESP32 Ch0 AND 5 on Ch1 (one at a time),
// then read captured count. Whichever combo yields captured > 0 tells us:
//   - which camera is wired to which ESP32 GPIO (32=Ch0, 33=Ch1)
//   - which Line number the camera is actually listening on
// Polarity defaults to camera's RisingEdge; we can extend to sweep that
// too if nothing trips here.
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
static int get_int(const char* j, const char* k) {
    char needle[64]; std::snprintf(needle, sizeof(needle), "\"%s\":", k);
    const char* p = std::strstr(j, needle);
    return p ? std::atoi(p + std::strlen(needle)) : 0;
}
static bool has_lit(const char* j, const char* k, const char* v) {
    char needle[128]; std::snprintf(needle, sizeof(needle), "\"%s\":%s", k, v);
    return std::strstr(j, needle) != nullptr;
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
        DWORD w = 0;
        WriteFile(h_, x.data(), (DWORD)x.size(), &w, nullptr);
    }
    ~Serial() { if (h_ != INVALID_HANDLE_VALUE) CloseHandle(h_); }
private:
    HANDLE h_ = INVALID_HANDLE_VALUE;
};

struct Cam {
    std::string key;
    void*       inst = nullptr;
    char        buf[8192];

    bool setup(const xi::baseline::PluginSymbols& s, xi_host_api* host, const char* name) {
        inst = s.create(host, name);
        if (!inst) return false;
        s.exchange(inst, R"({"command":"set_pixel_format","value":"Mono8"})",             buf, sizeof(buf));
        s.exchange(inst, R"({"command":"set_roi","x":0,"y":0,"w":640,"h":480})",          buf, sizeof(buf));
        s.exchange(inst, R"({"command":"set_exposure","value":5000})",                    buf, sizeof(buf));
        s.exchange(inst, R"({"command":"set_trigger_mode","mode":"on","source":"Line0"})",buf, sizeof(buf));
        s.exchange(inst, R"({"command":"discover"})",                                      buf, sizeof(buf));
        char cmd[256];
        std::snprintf(cmd, sizeof(cmd), R"({"command":"connect","device_key":"%s"})", key.c_str());
        if (s.exchange(inst, cmd, buf, sizeof(buf)) <= 0) return false;
        return has_lit(buf, "connected", "true");
    }
    void set_source(const xi::baseline::PluginSymbols& s, const char* src) {
        char cmd[128];
        std::snprintf(cmd, sizeof(cmd),
            R"({"command":"set_trigger_mode","mode":"on","source":"%s"})", src);
        s.exchange(inst, cmd, buf, sizeof(buf));
    }
    int captured(const xi::baseline::PluginSymbols& s) {
        s.exchange(inst, R"({"command":"get_status"})", buf, sizeof(buf));
        return get_int(buf, "captured");
    }
    void teardown(const xi::baseline::PluginSymbols& s) {
        if (!inst) return;
        s.exchange(inst, R"({"command":"set_trigger_mode","mode":"off"})", buf, sizeof(buf));
        s.exchange(inst, R"({"command":"disconnect"})",                    buf, sizeof(buf));
        s.destroy(inst);
        inst = nullptr;
    }
};

int main() {
    HMODULE dll = LoadLibraryA(HIKROBOT_CAMERA_DLL_PATH);
    if (!dll) return die("LoadLibrary");
    auto syms = xi::baseline::load_symbols(dll);
    if (!syms.ok()) return die("syms");
    auto host = xi::ImagePool::make_host_api();

    // Discover.
    std::vector<std::string> keys;
    {
        void* probe = syms.create(&host, "probe");
        char pbuf[16384]; pbuf[0] = 0;
        syms.exchange(probe, R"({"command":"discover"})", pbuf, sizeof(pbuf));
        const char* p = pbuf;
        while ((p = std::strstr(p, "\"USB:")) != nullptr) {
            const char* e = std::strchr(p + 1, '"');
            if (!e) break;
            keys.emplace_back(p + 1, e - p - 1);
            p = e + 1;
        }
        syms.destroy(probe);
    }
    std::printf("cameras: %zu\n", keys.size());
    for (auto& k : keys) std::printf("  %s\n", k.c_str());
    std::printf("\n");
    if (keys.empty()) return die("no cameras");

    // Open ESP32.
    Serial trig;
    if (!trig.open("COM4", 115200)) return die("open COM4");
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // Set up each camera on Line0 initially; we'll switch lines in the sweep.
    std::vector<Cam> cams(keys.size());
    for (size_t i = 0; i < keys.size(); ++i) {
        cams[i].key = keys[i];
        char name[32]; std::snprintf(name, sizeof(name), "probe_%zu", i);
        if (!cams[i].setup(syms, &host, name)) {
            std::fprintf(stderr, "cam %zu setup failed\n", i);
        }
    }

    const char* sources[] = { "Line0", "Line1", "Line2", "Line3" };
    const int   masks[]   = { 0x1, 0x2 };  // Ch0 only, then Ch1 only
    const char* mask_name[] = { "Ch0 (GPIO32)", "Ch1 (GPIO33)" };

    std::printf("sweep: for each camera, try each TriggerSource with each ESP32 channel\n");
    std::printf("       5 pulses at 2 Hz per trial, 100us width\n\n");

    for (size_t ci = 0; ci < cams.size(); ++ci) {
        auto& c = cams[ci];
        if (!c.inst) continue;
        std::printf("── camera %zu (%s) ──\n", ci, c.key.c_str());
        for (const char* src : sources) {
            c.set_source(syms, src);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            for (size_t mi = 0; mi < 2; ++mi) {
                int before = c.captured(syms);
                char cmd[80];
                std::snprintf(cmd, sizeof(cmd),
                    "BURST N=5 HZ=2 WIDTH=100 MASK=0x%x", masks[mi]);
                trig.write_line(cmd);
                std::this_thread::sleep_for(std::chrono::seconds(4));   // 5 pulses @ 2Hz + margin
                int after = c.captured(syms);
                int got = after - before;
                const char* verdict = got > 0 ? "✓ GOT FRAMES" : "·";
                std::printf("  %-6s  %s  got=%d  %s\n",
                            src, mask_name[mi], got, verdict);
            }
        }
        std::printf("\n");
    }

    for (auto& c : cams) c.teardown(syms);
    std::printf("sweep done.\n");
    return 0;
}
