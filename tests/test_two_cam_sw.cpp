//
// test_two_cam_sw.cpp — two-camera software-trigger sanity check.
//
// Same structure as test_two_cam but uses TriggerSource=Software
// (MV_CC_SetCommandValue) so we don't depend on ESP32 wiring or opto
// signalling. Proves that two cameras can be configured and delivered
// frames simultaneously in strict trigger mode, independent of the
// hardware trigger path.
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

static int die(const char* what) {
    std::fprintf(stderr, "FAIL: %s\n", what);
    return 1;
}
static bool has_literal(const char* json, const char* key, const char* val) {
    char needle[128];
    std::snprintf(needle, sizeof(needle), "\"%s\":%s", key, val);
    return std::strstr(json, needle) != nullptr;
}
static int get_int(const char* json, const char* key) {
    char needle[64];
    std::snprintf(needle, sizeof(needle), "\"%s\":", key);
    const char* p = std::strstr(json, needle);
    if (!p) return -1;
    return std::atoi(p + std::strlen(needle));
}

struct Cam {
    const char* name;
    std::string device_key;
    void*       inst = nullptr;
    char        buf[16384];

    bool configure(const xi::baseline::PluginSymbols& s, xi_host_api* host) {
        inst = s.create(host, name);
        if (!inst) return false;
        s.exchange(inst, R"({"command":"set_pixel_format","value":"Mono8"})", buf, sizeof(buf));
        s.exchange(inst, R"({"command":"set_roi","x":0,"y":0,"w":640,"h":480})", buf, sizeof(buf));
        s.exchange(inst, R"({"command":"set_exposure","value":5000})",        buf, sizeof(buf));
        s.exchange(inst,
            R"({"command":"set_trigger_mode","mode":"on","source":"Software"})", buf, sizeof(buf));
        s.exchange(inst,
            R"({"command":"set_strict_trigger_mode","value":true,"timeout_ms":2000})",
            buf, sizeof(buf));
        s.exchange(inst, R"({"command":"discover"})", buf, sizeof(buf));
        char cmd[256];
        std::snprintf(cmd, sizeof(cmd),
            R"({"command":"connect","device_key":"%s"})", device_key.c_str());
        if (s.exchange(inst, cmd, buf, sizeof(buf)) <= 0) return false;
        if (!has_literal(buf, "connected", "true")) return false;
        return true;
    }
    void teardown(const xi::baseline::PluginSymbols& s) {
        if (!inst) return;
        s.exchange(inst, R"({"command":"set_trigger_mode","mode":"off"})", buf, sizeof(buf));
        s.exchange(inst, R"({"command":"disconnect"})",                     buf, sizeof(buf));
        s.destroy(inst);
        inst = nullptr;
    }
};

int main() {
    HMODULE dll = LoadLibraryA(HIKROBOT_CAMERA_DLL_PATH);
    if (!dll) return die("LoadLibrary");
    auto syms = xi::baseline::load_symbols(dll);
    if (!syms.ok()) return die("load_symbols");
    auto host = xi::ImagePool::make_host_api();

    // Probe.
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
        std::printf("discovered %zu device(s):\n", keys.size());
        for (auto& k : keys) std::printf("  %s\n", k.c_str());
        std::printf("\n");
        if (keys.size() < 2) return die("need 2 cameras");
    }

    Cam cam0{"cam0_sw", keys[0]};
    Cam cam1{"cam1_sw", keys[1]};
    if (!cam0.configure(syms, &host)) { cam1.teardown(syms); return die("configure cam0"); }
    if (!cam1.configure(syms, &host)) { cam0.teardown(syms); return die("configure cam1"); }
    std::printf("both cameras connected + armed for software trigger\n\n");

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    const int N = 5;
    std::printf("firing %d software triggers on each camera (spaced 300 ms)…\n", N);
    for (int i = 0; i < N; ++i) {
        char tmp[64];
        syms.exchange(cam0.inst, R"({"command":"software_trigger","count":1})", tmp, sizeof(tmp));
        syms.exchange(cam1.inst, R"({"command":"software_trigger","count":1})", tmp, sizeof(tmp));
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }

    auto drain = [&](Cam& c, int& real, int& miss) {
        for (int i = 0; i < N; ++i) {
            xi_record in{}; in.json = "{}";
            xi_record_out out{}; xi_record_out_init(&out);
            // Retry a couple of times in case the slot hasn't filled yet.
            for (int retry = 0; retry < 3; ++retry) {
                syms.process(c.inst, &in, &out);
                if (!std::strstr(out.json, "\"error\":\"no_pending_trigger\"")) break;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                xi_record_out_init(&out);
            }
            bool missed = std::strstr(out.json, "\"missed\":true") != nullptr;
            int  tid = get_int(out.json, "trigger_id");
            if (missed) ++miss; else ++real;
            std::printf("  %s tid=%-2d %s image=%d\n",
                        c.name, tid, missed ? "MISSED" : "real  ", out.image_count);
        }
    };

    int real0 = 0, miss0 = 0, real1 = 0, miss1 = 0;
    drain(cam0, real0, miss0);
    drain(cam1, real1, miss1);

    std::printf("\n=== result ===\n"
                "cam0: real=%d missed=%d (of %d)\n"
                "cam1: real=%d missed=%d (of %d)\n",
                real0, miss0, N, real1, miss1, N);

    cam0.teardown(syms);
    cam1.teardown(syms);

    if (real0 == 0 || real1 == 0)
        return die("at least one camera produced zero real frames");
    std::printf("\nOK — two-camera strict software-trigger works.\n");
    return 0;
}
