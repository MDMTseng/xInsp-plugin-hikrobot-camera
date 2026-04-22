//
// test_four_cam_freerun.cpp — four cameras in free-run @ 10 fps.
//   A = MV-CA050-11UM  (00F50664114)   5 MP  mono
//   B = MV-CE200-11UC  (00K79315117)  15 MP  color "TopView"
//   C = MV-CA050-12UC  (J40531452)     5 MP  color
//   D = MV-CE200-11UM  (00DA5328883)  20 MP  mono
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

struct Cam { const char* name; std::string key; void* inst = nullptr; char buf[16384]; };

static bool configure_freerun(const xi::baseline::PluginSymbols& s, xi_host_api* host,
                              Cam& c, double fps) {
    c.inst = s.create(host, c.name);
    if (!c.inst) return false;
    s.exchange(c.inst, R"({"command":"set_trigger_mode","mode":"off"})",    c.buf, sizeof(c.buf));
    char fpscmd[128];
    std::snprintf(fpscmd, sizeof(fpscmd),
        R"({"command":"set_frame_rate","value":%g})", fps);
    s.exchange(c.inst, fpscmd, c.buf, sizeof(c.buf));
    s.exchange(c.inst, R"({"command":"discover"})", c.buf, sizeof(c.buf));
    char cmd[256];
    std::snprintf(cmd, sizeof(cmd),
        R"({"command":"connect","device_key":"%s","defer_streaming":true})", c.key.c_str());
    if (s.exchange(c.inst, cmd, c.buf, sizeof(c.buf)) <= 0) return false;
    return std::strstr(c.buf, "\"connected\":true") != nullptr;
}

static void teardown(const xi::baseline::PluginSymbols& s, Cam& c) {
    if (!c.inst) return;
    s.exchange(c.inst, R"({"command":"disconnect"})", c.buf, sizeof(c.buf));
    s.destroy(c.inst);
    c.inst = nullptr;
}

int main() {
    HMODULE dll = LoadLibraryA(HIKROBOT_CAMERA_DLL_PATH);
    auto syms  = xi::baseline::load_symbols(dll);
    auto host  = xi::ImagePool::make_host_api();

    const double TARGET_FPS = 0.0;  // 0 = uncapped (AFR disabled)
    const int    DURATION_S = 10;

    std::vector<Cam> cams = {
        {"camA", "USB:00F50664114"},
        {"camB", "USB:00K79315117"},
        {"camC", "USB:J40531452"},
        {"camD", "USB:00DA5328883"},
    };
    const char* LABELS[] = {
        "A CA050-UM  5MP",
        "B CE200-UC 15MP TopView",
        "C CA050-UC  5MP",
        "D CE200-UM 20MP",
    };

    std::printf("cameras:\n");
    for (int i = 0; i < 4; ++i) std::printf("  %s  %s\n", LABELS[i], cams[i].key.c_str());
    std::printf("\ntarget: %.1f fps each, duration %d s\n\n", TARGET_FPS, DURATION_S);

    for (int i = 0; i < 4; ++i) {
        if (!configure_freerun(syms, &host, cams[i], TARGET_FPS)) {
            std::fprintf(stderr, "FAIL: configure cam %c\n", 'A' + i);
            for (int j = 0; j < i; ++j) teardown(syms, cams[j]);
            return 1;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));

    std::printf("starting streams...\n");
    for (int i = 0; i < 4; ++i) {
        syms.exchange(cams[i].inst, R"({"command":"start_streaming"})",
                      cams[i].buf, sizeof(cams[i].buf));
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    auto snap = [&](Cam& c) {
        char sb[4096];
        syms.exchange(c.inst, R"({"command":"get_status"})", sb, sizeof(sb));
        return peek_int(sb, "captured");
    };

    int base[4], prev[4];
    for (int i = 0; i < 4; ++i) base[i] = prev[i] = snap(cams[i]);
    std::printf("\n%-6s  %-5s %-5s %-5s %-5s    %-4s %-4s %-4s %-4s\n",
        "t(s)", "A", "B", "C", "D", "A.f", "B.f", "C.f", "D.f");
    std::printf("%-6s  %-5s %-5s %-5s %-5s    %-4s %-4s %-4s %-4s\n",
        "----", "-----", "-----", "-----", "-----", "----", "----", "----", "----");

    for (int t = 1; t <= DURATION_S; ++t) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        int cur[4];
        for (int i = 0; i < 4; ++i) cur[i] = snap(cams[i]);
        std::printf("%-6d  %-5d %-5d %-5d %-5d    %-4d %-4d %-4d %-4d\n",
            t,
            cur[0] - base[0], cur[1] - base[1], cur[2] - base[2], cur[3] - base[3],
            cur[0] - prev[0], cur[1] - prev[1], cur[2] - prev[2], cur[3] - prev[3]);
        for (int i = 0; i < 4; ++i) prev[i] = cur[i];
    }

    std::printf("\n=== final ===\n");
    for (int i = 0; i < 4; ++i) {
        char sb[4096], db[16384];
        syms.exchange(cams[i].inst, R"({"command":"get_status"})", sb, sizeof(sb));
        syms.exchange(cams[i].inst, R"({"command":"diag"})", db, sizeof(db));
        std::printf("%s  captured=%-3d dropped=%-3d mv_rcvd=%-3d mv_err=%-3d "
                    "throughput=%.0f MB/s  resulting=%.1f fps\n",
            LABELS[i],
            peek_int(sb, "captured") - base[i],
            peek_int(sb, "dropped"),
            peek_int(sb, "mv_received_frames"),
            peek_int(sb, "mv_error_frames"),
            diag_cur(db, "DeviceLinkCurrentThroughput") / 1e6,
            diag_cur(db, "ResultingFrameRate"));
    }

    for (int i = 0; i < 4; ++i) {
        syms.exchange(cams[i].inst, R"({"command":"stop_streaming"})",
                      cams[i].buf, sizeof(cams[i].buf));
    }
    for (int i = 0; i < 4; ++i) teardown(syms, cams[i]);
    return 0;
}
