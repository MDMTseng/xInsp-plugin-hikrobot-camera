//
// test_overrun.cpp — deliberate trigger-overrun to demonstrate drop
// detection.
//
// Puts the camera into software-trigger mode, sets a 100 ms exposure,
// then fires 200 triggers in a tight loop (≈instant). A correctly
// behaving HikRobot firmware accepts triggers until the sensor is busy
// and ignores the rest. After a ~3 s settle, the plugin's counters
// should show:
//     triggers_sent      = 200
//     triggers_accepted  ≪ 200    (camera honoured only a handful)
//     captured           ≈ triggers_accepted
//     dropped            = 0        (no nFrameNum gaps)
//

#include <xi/xi_abi.hpp>
#include <xi/xi_baseline.hpp>
#include <xi/xi_image_pool.hpp>

#include <windows.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <thread>

#ifndef HIKROBOT_CAMERA_DLL_PATH
#define HIKROBOT_CAMERA_DLL_PATH "hikrobot_camera.dll"
#endif

static int die(const char* what, const char* detail = "") {
    std::fprintf(stderr, "FAIL: %s %s\n", what, detail);
    return 1;
}

static int get_int(const char* json, const char* key) {
    // naive "key":N parser; good enough for our diag fields.
    char needle[64];
    std::snprintf(needle, sizeof(needle), "\"%s\":", key);
    const char* p = std::strstr(json, needle);
    if (!p) return -1;
    return std::atoi(p + std::strlen(needle));
}

int main() {
    HMODULE dll = LoadLibraryA(HIKROBOT_CAMERA_DLL_PATH);
    if (!dll) return die("LoadLibrary", HIKROBOT_CAMERA_DLL_PATH);
    auto syms = xi::baseline::load_symbols(dll);
    if (!syms.ok()) return die("missing C ABI exports");
    auto host = xi::ImagePool::make_host_api();
    void* inst = syms.create(&host, "overrun");
    if (!inst) return die("create");

    char buf[8192];

    // 1) Discover + connect
    syms.exchange(inst, R"({"command":"discover"})", buf, sizeof(buf));
    if (!std::strstr(buf, "\"devices\":[\"")) {
        std::fprintf(stderr, "no devices\n");
        syms.destroy(inst);
        return 1;
    }
    syms.exchange(inst, R"({"command":"connect","index":0})", buf, sizeof(buf));
    if (!std::strstr(buf, "\"connected\":true")) return die("connect");

    // 2) Small ROI (fast readout), long exposure so triggers *will* overlap
    syms.exchange(inst, R"({"command":"set_roi","x":0,"y":0,"w":640,"h":480})",
                  buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"set_exposure","value":100000})",  // 100 ms
                  buf, sizeof(buf));

    // 3) Software trigger mode
    syms.exchange(inst,
        R"({"command":"set_trigger_mode","mode":"on","source":"Software"})",
        buf, sizeof(buf));
    std::this_thread::sleep_for(std::chrono::milliseconds(400));
    std::printf("after set_trigger_mode on,source=Software:\n%.600s\n\n", buf);

    // 4) Blast 200 software triggers as fast as possible.
    const int N = 200;
    std::printf("firing %d software triggers in a tight loop...\n", N);
    auto t0 = std::chrono::steady_clock::now();
    char cmd[80];
    std::snprintf(cmd, sizeof(cmd),
                  R"({"command":"software_trigger","count":%d})", N);
    syms.exchange(inst, cmd, buf, sizeof(buf));
    auto tFired = std::chrono::steady_clock::now();
    std::printf("  (firing took %.3f s)\n",
                std::chrono::duration<double>(tFired - t0).count());

    // 5) Let any accepted frames flow to us — longest possible wait is
    //    (captured × (exposure + readout)) ≈ a few seconds if the camera
    //    accepted ~20 triggers at 100 ms each.
    for (int i = 0; i < 8; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        int n = syms.get_def(inst, buf, sizeof(buf));
        int cap  = get_int(buf, "captured");
        int drop = get_int(buf, "dropped");
        int acc  = get_int(buf, "triggers_accepted");
        int rej  = get_int(buf, "triggers_rejected");
        int sent = get_int(buf, "triggers_sent");
        std::printf("  t+%.1fs sent=%d accepted=%d rejected=%d captured=%d dropped=%d\n",
                    (i + 1) * 0.5, sent, acc, rej, cap, drop);
    }

    // 6) Final stats
    int n = syms.get_def(inst, buf, sizeof(buf));
    int sent = get_int(buf, "triggers_sent");
    int acc  = get_int(buf, "triggers_accepted");
    int rej  = get_int(buf, "triggers_rejected");
    int cap  = get_int(buf, "captured");
    int drop = get_int(buf, "dropped");

    int over = get_int(buf, "overruns");
    std::printf("\n=== overrun test result ===\n"
                "triggers_sent      = %d\n"
                "triggers_accepted  = %d    (= captured + dropped)\n"
                "triggers_rejected  = %d    (MVS busy/error at fire time)\n"
                "captured           = %d\n"
                "dropped            = %d    (nFrameNum gaps)\n"
                "overruns           = %d    (sent - accepted)\n"
                "overrun ratio      = %.1f%%\n",
                sent, acc, rej, cap, drop, over,
                sent > 0 ? 100.0 * over / sent : 0.0);

    // Cleanup
    syms.exchange(inst,
        R"({"command":"set_trigger_mode","mode":"off"})", buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"disconnect"})", buf, sizeof(buf));
    syms.destroy(inst);

    // Pass if we demonstrably saw more sent than accepted (that's what
    // "overrun detection works" looks like).
    if (sent <= 0)  return die("no triggers sent");
    if (over <= 0)  return die("no overruns detected (all triggers accepted?)");
    std::printf("\nOK — overrun detection works.\n");
    return 0;
}
