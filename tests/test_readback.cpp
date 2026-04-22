//
// test_readback.cpp — verify that apply_exposure / apply_gain /
// apply_frame_rate update the plugin's shadow state to the CAMERA's
// actual accepted value (after min/max clamp or increment snap).
//
#include <xi/xi_abi.hpp>
#include <xi/xi_baseline.hpp>
#include <xi/xi_image_pool.hpp>
#include <windows.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#ifndef HIKROBOT_CAMERA_DLL_PATH
#define HIKROBOT_CAMERA_DLL_PATH "hikrobot_camera.dll"
#endif

static double peek_d(const char* j, const char* k) {
    char n[64]; std::snprintf(n, sizeof(n), "\"%s\":", k);
    const char* p = std::strstr(j, n);
    return p ? std::atof(p + std::strlen(n)) : 0.0;
}

int main() {
    HMODULE dll = LoadLibraryA(HIKROBOT_CAMERA_DLL_PATH);
    auto syms = xi::baseline::load_symbols(dll);
    auto host = xi::ImagePool::make_host_api();
    void* inst = syms.create(&host, "readback");
    char buf[32768];
    syms.exchange(inst, R"({"command":"discover"})", buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"connect","index":0})", buf, sizeof(buf));

    auto show = [&](const char* label) {
        syms.exchange(inst, R"({"command":"get_status"})", buf, sizeof(buf));
        std::printf("%s:  exposure_us=%.1f  gain=%.3f  fps=%.3f\n",
            label,
            peek_d(buf, "exposure_us"),
            peek_d(buf, "gain"),
            peek_d(buf, "fps"));
    };

    show("baseline");

    // Under min → camera should clamp to min (15 us on CA050).
    syms.exchange(inst, R"({"command":"set_exposure","value":1})", buf, sizeof(buf));
    show("set exposure=1      -> clamped to min");

    // In range.
    syms.exchange(inst, R"({"command":"set_exposure","value":5000})", buf, sizeof(buf));
    show("set exposure=5000   -> verbatim");

    // Above max (~10M us) → clamped to max.
    syms.exchange(inst, R"({"command":"set_exposure","value":99999999})", buf, sizeof(buf));
    show("set exposure=99M    -> clamped to max");

    // Gain out of range.
    syms.exchange(inst, R"({"command":"set_gain","value":100})", buf, sizeof(buf));
    show("set gain=100        -> clamped to max");

    syms.exchange(inst, R"({"command":"set_gain","value":-5})", buf, sizeof(buf));
    show("set gain=-5         -> clamped to min");

    // Enable AFR + request fps=10, read back exact.
    syms.exchange(inst, R"({"command":"set_frame_rate","value":10})", buf, sizeof(buf));
    show("set fps=10");

    syms.exchange(inst, R"({"command":"disconnect"})", buf, sizeof(buf));
    syms.destroy(inst);
    return 0;
}
