//
// test_trig_probe.cpp — verify ESP32 triggers reach a specific camera.
// Fire 5 pulses on each mask (0x1, 0x2) and check the camera's
// line0_edges counter. Tells us which cam is wired to which pin.
//
#include <xi/xi_abi.hpp>
#include <xi/xi_baseline.hpp>
#include <xi/xi_image_pool.hpp>
#include <windows.h>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
#include <chrono>

#ifndef HIKROBOT_CAMERA_DLL_PATH
#define HIKROBOT_CAMERA_DLL_PATH "hikrobot_camera.dll"
#endif

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

int main(int argc, char** argv) {
    const char* key = argc > 1 ? argv[1] : "USB:K47674142";

    HMODULE dll = LoadLibraryA(HIKROBOT_CAMERA_DLL_PATH);
    auto syms = xi::baseline::load_symbols(dll);
    auto host = xi::ImagePool::make_host_api();
    void* inst = syms.create(&host, "trig_probe");
    char buf[16384];

    syms.exchange(inst, R"({"command":"set_pixel_format","value":"Mono8"})",             buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"set_trigger_mode","mode":"on","source":"Line0"})",buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"set_strict_trigger_mode","value":true,"timeout_ms":2000})",
                  buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"discover"})", buf, sizeof(buf));
    char cmd[256];
    std::snprintf(cmd, sizeof(cmd), R"({"command":"connect","device_key":"%s"})", key);
    syms.exchange(inst, cmd, buf, sizeof(buf));
    std::printf("%s  connected=%s  streaming=%s\n", key,
        std::strstr(buf, "\"connected\":true") ? "YES" : "NO",
        std::strstr(buf, "\"streaming\":true") ? "YES" : "NO");

    Serial trig;
    if (!trig.open("COM4", 115200)) {
        std::fprintf(stderr, "COM4 open failed\n");
        syms.exchange(inst, R"({"command":"disconnect"})", buf, sizeof(buf));
        syms.destroy(inst);
        return 1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    auto edges = [&]() {
        syms.exchange(inst, R"({"command":"get_status"})", buf, sizeof(buf));
        return peek_int(buf, "line0_edges");
    };

    int e0 = edges();
    std::printf("\n--- MASK=0x1 (GPIO 32) ---\n");
    trig.write_line("BURST N=5 HZ=5 WIDTH=200 MASK=0x1");
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    int e1 = edges();
    std::printf("edges received: %d\n", e1 - e0);

    std::printf("\n--- MASK=0x2 (GPIO 33) ---\n");
    trig.write_line("BURST N=5 HZ=5 WIDTH=200 MASK=0x2");
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    int e2 = edges();
    std::printf("edges received: %d\n", e2 - e1);

    std::printf("\n--- MASK=0x3 (both) ---\n");
    trig.write_line("BURST N=5 HZ=5 WIDTH=200 MASK=0x3");
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    int e3 = edges();
    std::printf("edges received: %d\n", e3 - e2);

    syms.exchange(inst, R"({"command":"set_trigger_mode","mode":"off"})", buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"disconnect"})", buf, sizeof(buf));
    syms.destroy(inst);
    return 0;
}
