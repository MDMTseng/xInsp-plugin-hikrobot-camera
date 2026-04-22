//
// test_ca050_alone.cpp — CA050 alone, free-run 10 fps, 10 s.
// Isolates the CA050 to see whether it can sustain capture by itself
// (i.e. is the failure CA050-specific or a two-cam issue).
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
static double peek_d(const char* j, const char* k) {
    char n[64]; std::snprintf(n, sizeof(n), "\"%s\":", k);
    const char* p = std::strstr(j, n);
    return p ? std::atof(p + std::strlen(n)) : 0.0;
}

int main(int argc, char** argv) {
    // Default to K47674142 if no arg. Pass a device key (e.g.
    // "USB:J40531452") to target a different camera.
    const char* key = argc > 1 ? argv[1] : "USB:K47674142";
    HMODULE dll = LoadLibraryA(HIKROBOT_CAMERA_DLL_PATH);
    auto syms  = xi::baseline::load_symbols(dll);
    auto host  = xi::ImagePool::make_host_api();

    std::printf("=== testing %s ===\n", key);

    void* inst = syms.create(&host, "ca050_alone");
    char buf[16384];

    syms.exchange(inst, R"({"command":"set_pixel_format","value":"Mono8"})", buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"set_trigger_mode","mode":"off"})",    buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"set_frame_rate","value":10})",        buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"discover"})", buf, sizeof(buf));
    char cmd[256];
    std::snprintf(cmd, sizeof(cmd),
        R"({"command":"connect","device_key":"%s"})", key);
    syms.exchange(inst, cmd, buf, sizeof(buf));
    // Trim the reply — most fields are noise; keep just the key flags.
    std::printf("connected=%s streaming=%s last_error=",
        std::strstr(buf, "\"connected\":true")  ? "YES" : "NO",
        std::strstr(buf, "\"streaming\":true")  ? "YES" : "NO");
    const char* e = std::strstr(buf, "\"last_error\":\"");
    if (e) {
        e += std::strlen("\"last_error\":\"");
        const char* q = std::strchr(e, '"');
        std::fwrite(e, 1, q ? q - e : 0, stdout);
    }
    std::printf("\n\n");

    // Pull link diag.
    char db[16384];
    syms.exchange(inst, R"({"command":"diag"})", db, sizeof(db));
    auto cur = [&](const char* key) -> double {
        char n[64]; std::snprintf(n, sizeof(n), "\"%s\":{", key);
        const char* p = std::strstr(db, n);
        if (!p) return -1;
        const char* c = std::strstr(p, "\"cur\":");
        return c ? std::atof(c + 6) : -1;
    };
    std::printf("link: LinkSpeed=%.0f  Throughput=%.0f  AFR=%.2f  Resulting=%.2f  Width=%.0f Height=%.0f\n\n",
        cur("DeviceLinkSpeed"), cur("DeviceLinkCurrentThroughput"),
        cur("AcquisitionFrameRate"), cur("ResultingFrameRate"),
        cur("Width"), cur("Height"));

    // Sample the captured counter over 10 seconds.
    auto snap = [&]() {
        syms.exchange(inst, R"({"command":"get_status"})", buf, sizeof(buf));
        return std::make_tuple(
            peek_int(buf, "captured"),
            peek_int(buf, "mv_received_frames"),
            peek_int(buf, "mv_error_frames"));
    };
    auto [c0, mr0, me0] = snap();
    std::printf("t  captured  mv_received  mv_errors\n");
    std::printf("-  --------  -----------  ---------\n");
    for (int t = 1; t <= 10; ++t) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        auto [c, mr, me] = snap();
        std::printf("%d  %-8d  %-11d  %-9d\n",
            t, c - c0, mr - mr0, me - me0);
    }

    syms.exchange(inst, R"({"command":"disconnect"})", buf, sizeof(buf));
    syms.destroy(inst);
    return 0;
}
