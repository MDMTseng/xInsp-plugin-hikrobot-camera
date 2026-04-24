//
// test_any_cam.cpp — target one camera by device_key, probe its native
// max dims, stream at full res @ 10 fps for 5 s, report captured /
// errors / actual resulting fps. Used to sweep every camera on the
// bus quickly.
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
static double diag_max(const char* db, const char* key) {
    char n[64]; std::snprintf(n, sizeof(n), "\"%s\":{", key);
    const char* p = std::strstr(db, n);
    if (!p) return -1;
    const char* c = std::strstr(p, "\"max\":");
    return c ? std::atof(c + 6) : -1;
}

int main(int argc, char** argv) {
    const char* key = argc > 1 ? argv[1] : nullptr;
    // Optional 2nd arg: target fps. 0 = uncapped (disable AFR) to measure
    // the camera's max sensor rate at whatever USB topology it's on.
    double target_fps = argc > 2 ? std::atof(argv[2]) : 10.0;
    if (!key) { std::fprintf(stderr, "usage: %s <USB:SERIAL> [fps (0=uncap)]\n", argv[0]); return 1; }

    HMODULE dll = LoadLibraryA(HIKROBOT_CAMERA_DLL_PATH);
    auto syms  = xi::baseline::load_symbols(dll);
    auto host  = xi::ImagePool::make_host_api();

    void* inst = syms.create(&host, "any_cam");
    char buf[16384], db[16384];

    // Phase 1: probe — open at default dims, read max, close.
    int wmax = 0, hmax = 0;
    {
        syms.exchange(inst, R"({"command":"set_pixel_format","value":"Mono8"})", buf, sizeof(buf));
        syms.exchange(inst, R"({"command":"set_trigger_mode","mode":"off"})",    buf, sizeof(buf));
        syms.exchange(inst, R"({"command":"discover"})", buf, sizeof(buf));
        char cmd[256];
        std::snprintf(cmd, sizeof(cmd), R"({"command":"connect","device_key":"%s"})", key);
        syms.exchange(inst, cmd, buf, sizeof(buf));
        if (!std::strstr(buf, "\"connected\":true")) {
            std::printf("%-20s  ERR       open failed\n", key);
            syms.destroy(inst);
            return 1;
        }
        syms.exchange(inst, R"({"command":"diag"})", db, sizeof(db));
        wmax = (int)diag_max(db, "Width");
        hmax = (int)diag_max(db, "Height");
        syms.exchange(inst, R"({"command":"disconnect"})", buf, sizeof(buf));
    }
    if (wmax <= 0 || hmax <= 0) {
        std::printf("%-20s  ERR       could not probe dims\n", key);
        syms.destroy(inst);
        return 1;
    }

    // Phase 2: reconnect with full-res ROI pre-staged + AFR=10.
    std::this_thread::sleep_for(std::chrono::seconds(1));
    syms.exchange(inst, R"({"command":"set_pixel_format","value":"Mono8"})", buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"set_trigger_mode","mode":"off"})",    buf, sizeof(buf));
    char roi[128];
    std::snprintf(roi, sizeof(roi),
        R"({"command":"set_roi","x":0,"y":0,"w":%d,"h":%d})", wmax, hmax);
    syms.exchange(inst, roi, buf, sizeof(buf));
    char fpscmd[128];
    std::snprintf(fpscmd, sizeof(fpscmd),
        R"({"command":"set_frame_rate","value":%g})", target_fps);
    syms.exchange(inst, fpscmd, buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"discover"})", buf, sizeof(buf));
    char cmd[256];
    std::snprintf(cmd, sizeof(cmd), R"({"command":"connect","device_key":"%s"})", key);
    syms.exchange(inst, cmd, buf, sizeof(buf));
    bool connected = std::strstr(buf, "\"connected\":true") != nullptr;
    bool streaming = std::strstr(buf, "\"streaming\":true") != nullptr;
    if (!connected || !streaming) {
        std::printf("%-20s  %dx%-4d  ERR conn=%d stream=%d\n",
            key, wmax, hmax, connected, streaming);
        syms.destroy(inst);
        return 1;
    }

    // Phase 3: sample captured over 5 s.
    auto snap_cap = [&]() {
        syms.exchange(inst, R"({"command":"get_status"})", buf, sizeof(buf));
        return peek_int(buf, "captured");
    };
    auto snap_err = [&]() {
        syms.exchange(inst, R"({"command":"get_status"})", buf, sizeof(buf));
        return peek_int(buf, "mv_error_frames");
    };
    int cap0 = snap_cap();
    int err0 = snap_err();
    std::this_thread::sleep_for(std::chrono::seconds(5));
    int cap1 = snap_cap();
    int err1 = snap_err();

    // Read resulting fps from diag.
    syms.exchange(inst, R"({"command":"diag"})", db, sizeof(db));
    double resulting_fps = diag_cur(db, "ResultingFrameRate");
    double programmed_fps = diag_cur(db, "AcquisitionFrameRate");

    // One-line verdict: key | dims | target | actual | captured | err
    int captured = cap1 - cap0;
    int errors   = err1 - err0;
    // Under "uncapped" mode the resulting fps is whatever the sensor does;
    // verdict should be OK as long as we're delivering close to it.
    const int expected = target_fps > 0 ? (int)(target_fps * 5 * 0.9)
                                        : (int)(resulting_fps * 5 * 0.9);
    const char* verdict =
        (captured >= expected && errors == 0) ? "OK"        :
        (captured == 0)                       ? "NO_FRAMES" :
                                                "DEGRADED";
    std::printf("%-20s  %dx%-4d  prog=%.1f  resulting=%.1f  captured=%-3d errors=%-3d  %s\n",
        key, wmax, hmax, programmed_fps, resulting_fps,
        captured, errors, verdict);

    syms.exchange(inst, R"({"command":"stop_streaming"})", buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"disconnect"})",     buf, sizeof(buf));
    syms.destroy(inst);
    return 0;
}
