//
// test_native.cpp — native baseline + smoke tests for hikrobot_camera.
//
// Hardware is touched only by the discover/connect tests, which tolerate
// the no-camera case (they just log the device count). The baseline set
// never touches the camera.
//

#include <xi/xi_abi.hpp>
#include <xi/xi_baseline.hpp>
#include <xi/xi_cert.hpp>
#include <xi/xi_image_pool.hpp>
#include <xi/xi_test.hpp>

#ifdef _WIN32
  #include <windows.h>
#endif

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>

#ifndef HIKROBOT_CAMERA_DLL_PATH
#define HIKROBOT_CAMERA_DLL_PATH "hikrobot_camera.dll"
#endif

static HMODULE g_dll = nullptr;
static xi::baseline::PluginSymbols g_syms;
static xi_host_api g_host = xi::ImagePool::make_host_api();

static void load_dll() {
    if (g_dll) return;
    g_dll = LoadLibraryA(HIKROBOT_CAMERA_DLL_PATH);
    if (!g_dll) {
        std::fprintf(stderr, "failed to load %s (err %lu)\n",
                     HIKROBOT_CAMERA_DLL_PATH, GetLastError());
        std::exit(2);
    }
    g_syms = xi::baseline::load_symbols(g_dll);
    if (!g_syms.ok()) {
        std::fprintf(stderr, "%s missing required C ABI exports\n", HIKROBOT_CAMERA_DLL_PATH);
        std::exit(2);
    }
}

XI_TEST(baseline_all_pass) {
    load_dll();
    auto summary = xi::baseline::run_all(g_syms, &g_host);
    for (auto& r : summary.results) {
        if (!r.passed) std::fprintf(stderr, "  baseline fail: %s: %s\n",
                                    r.name.c_str(), r.error.c_str());
    }
    XI_EXPECT(summary.all_passed);
    if (summary.all_passed) {
        auto folder = std::filesystem::path(HIKROBOT_CAMERA_DLL_PATH).parent_path();
        xi::cert::certify(folder, HIKROBOT_CAMERA_DLL_PATH, "hikrobot_camera", g_syms, &g_host);
    }
}

XI_TEST(hikrobot_get_def_reports_not_connected) {
    load_dll();
    void* inst = g_syms.create(&g_host, "cam0");
    XI_EXPECT(inst != nullptr);

    char buf[4096];
    int n = g_syms.get_def(inst, buf, sizeof(buf));
    XI_EXPECT(n > 0);
    XI_EXPECT(std::strstr(buf, "\"connected\":false") != nullptr);

    g_syms.destroy(inst);
}

XI_TEST(hikrobot_discover_reports_devices) {
    load_dll();
    void* inst = g_syms.create(&g_host, "cam_disc");

    char rsp[16384];
    int n = g_syms.exchange(inst, R"({"command":"discover"})", rsp, sizeof(rsp));
    XI_EXPECT(n > 0);
    XI_EXPECT(std::strstr(rsp, "\"devices\":") != nullptr);
    std::fprintf(stderr, "  discover output: %.300s\n", rsp);

    g_syms.destroy(inst);
}

XI_TEST(hikrobot_connect_fails_without_target) {
    load_dll();
    void* inst = g_syms.create(&g_host, "cam_bogus");

    char rsp[8192];
    int n = g_syms.exchange(inst,
        R"({"command":"connect","device_key":"USB:__NONE__"})",
        rsp, sizeof(rsp));
    XI_EXPECT(n > 0);
    XI_EXPECT(std::strstr(rsp, "\"connected\":false") != nullptr);
    XI_EXPECT(std::strstr(rsp, "\"last_error\":") != nullptr);

    g_syms.destroy(inst);
}

XI_TEST(hikrobot_set_params_before_connect_persists) {
    load_dll();
    void* inst = g_syms.create(&g_host, "cam_params");

    char rsp[4096];
    g_syms.exchange(inst, R"({"command":"set_frame_rate","value":5.5})",    rsp, sizeof(rsp));
    g_syms.exchange(inst, R"({"command":"set_exposure","value":4321})",     rsp, sizeof(rsp));
    g_syms.exchange(inst, R"({"command":"set_gain","value":1.5})",          rsp, sizeof(rsp));
    g_syms.exchange(inst, R"({"command":"set_pixel_format","value":"Mono8"})", rsp, sizeof(rsp));

    char def[4096];
    int n = g_syms.get_def(inst, def, sizeof(def));
    XI_EXPECT(n > 0);
    XI_EXPECT(std::strstr(def, "\"fps\":5.5") != nullptr);
    XI_EXPECT(std::strstr(def, "\"exposure_us\":4321") != nullptr);
    XI_EXPECT(std::strstr(def, "\"gain\":1.5") != nullptr);

    g_syms.destroy(inst);
}

int main() {
    auto results = xi::test::run_all();
    for (auto& r : results) if (!r.passed) return 1;
    return 0;
}
