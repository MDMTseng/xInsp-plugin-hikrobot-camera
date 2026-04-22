//
// test_any_cam_noroi.cpp — same as test_any_cam but DOES NOT set ROI.
// Uses whatever ROI the camera currently has persisted.
//
#include <xi/xi_abi.hpp>
#include <xi/xi_baseline.hpp>
#include <xi/xi_image_pool.hpp>
#include <windows.h>
#include <cstdio>
#include <cstdlib>
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
static double diag_cur(const char* db, const char* key) {
    char n[64]; std::snprintf(n, sizeof(n), "\"%s\":{", key);
    const char* p = std::strstr(db, n);
    if (!p) return -1;
    const char* c = std::strstr(p, "\"cur\":");
    return c ? std::atof(c + 6) : -1;
}

int main(int argc, char** argv) {
    const char* key = argc > 1 ? argv[1] : "USB:00K79315117";
    HMODULE dll = LoadLibraryA(HIKROBOT_CAMERA_DLL_PATH);
    auto syms  = xi::baseline::load_symbols(dll);
    auto host  = xi::ImagePool::make_host_api();
    void* inst = syms.create(&host, "any_cam_noroi");
    char buf[16384], db[16384];

    // Skip pixel format — let camera use whatever it has persisted.
    // (15117 is a color CE200-UC that rejects Mono8.)
    syms.exchange(inst, R"({"command":"set_trigger_mode","mode":"off"})",    buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"set_frame_rate","value":10})",        buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"discover"})", buf, sizeof(buf));
    char cmd[256];
    std::snprintf(cmd, sizeof(cmd), R"({"command":"connect","device_key":"%s"})", key);
    syms.exchange(inst, cmd, buf, sizeof(buf));
    bool ok = std::strstr(buf, "\"connected\":true") && std::strstr(buf, "\"streaming\":true");

    syms.exchange(inst, R"({"command":"diag"})", db, sizeof(db));
    int W = (int)diag_cur(db, "Width");
    int H = (int)diag_cur(db, "Height");
    double resulting = diag_cur(db, "ResultingFrameRate");
    double prog      = diag_cur(db, "AcquisitionFrameRate");

    auto cap = [&]() {
        syms.exchange(inst, R"({"command":"get_status"})", buf, sizeof(buf));
        return peek_int(buf, "captured");
    };
    auto err = [&]() {
        syms.exchange(inst, R"({"command":"get_status"})", buf, sizeof(buf));
        return peek_int(buf, "mv_error_frames");
    };
    int c0 = cap(), e0 = err();
    std::this_thread::sleep_for(std::chrono::seconds(5));
    int c1 = cap(), e1 = err();

    // Also pull the actual pixel format (enum index, not symbol — the
    // noroi diag path doesn't resolve symbol here).
    char sb[4096];
    syms.exchange(inst, R"({"command":"get_status"})", sb, sizeof(sb));
    const char* pf = std::strstr(sb, "\"pixel_format\":\"");
    std::string fmt = "?";
    if (pf) {
        pf += std::strlen("\"pixel_format\":\"");
        const char* end = std::strchr(pf, '"');
        if (end) fmt.assign(pf, end - pf);
    }
    std::printf("%-20s  %dx%-4d  fmt=%s  prog=%.1f  resulting=%.1f  captured=%-3d errors=%-3d  %s\n",
        key, W, H, fmt.c_str(), prog, resulting, c1 - c0, e1 - e0,
        ok ? ((c1 - c0 >= 45 && e1 - e0 == 0) ? "OK" :
              (c1 - c0 == 0) ? "NO_FRAMES" : "DEGRADED") : "OPEN_FAIL");

    syms.exchange(inst, R"({"command":"stop_streaming"})", buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"disconnect"})",     buf, sizeof(buf));
    syms.destroy(inst);
    return 0;
}
