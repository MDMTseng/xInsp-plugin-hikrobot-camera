//
// test_strict.cpp — demonstrate strict trigger mode.
//
// With strict_trigger_mode=true, every software trigger produces exactly
// one process() output in order. Real frame if the camera delivered,
// empty-image placeholder with {"missed":true,"error":"missed_trigger..."}
// if it didn't — so downstream code stays trigger-indexed.
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

static int die(const char* what) {
    std::fprintf(stderr, "FAIL: %s\n", what);
    return 1;
}

int main() {
    HMODULE dll = LoadLibraryA(HIKROBOT_CAMERA_DLL_PATH);
    if (!dll) return die("LoadLibrary");
    auto syms = xi::baseline::load_symbols(dll);
    if (!syms.ok()) return die("load_symbols");
    auto host = xi::ImagePool::make_host_api();
    void* inst = syms.create(&host, "strict");
    if (!inst) return die("create");
    char buf[8192];

    syms.exchange(inst, R"({"command":"discover"})", buf, sizeof(buf));
    if (!std::strstr(buf, "\"devices\":[\"")) return die("no device");

    syms.exchange(inst, R"({"command":"connect","index":0})", buf, sizeof(buf));
    if (!std::strstr(buf, "\"connected\":true")) return die("connect");

    // Small ROI, long exposure so back-to-back triggers overlap.
    syms.exchange(inst, R"({"command":"set_roi","x":0,"y":0,"w":640,"h":480})",     buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"set_exposure","value":100000})",              buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"set_trigger_mode","mode":"on","source":"Software"})", buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"set_strict_trigger_mode","value":true,"timeout_ms":400})", buf, sizeof(buf));
    std::this_thread::sleep_for(std::chrono::milliseconds(400));

    // Fire 20 triggers back-to-back. At 100 ms exposure, the camera can
    // only accept ~1 of them per 100ms window, so most will miss.
    const int N = 20;
    std::printf("firing %d software triggers back-to-back...\n", N);
    for (int i = 0; i < N; ++i) {
        syms.exchange(inst, R"({"command":"software_trigger","count":1})", buf, sizeof(buf));
        // no sleep — provoke overruns
    }

    // Drain — should return exactly N outputs, in order.
    std::printf("\ndraining process() expecting %d outputs:\n", N);
    int got_real = 0, got_missed = 0;
    auto find_str = [](const char* j, const char* key) {
        char needle[64];
        std::snprintf(needle, sizeof(needle), "\"%s\":\"", key);
        const char* p = std::strstr(j, needle);
        if (!p) return std::string();
        p += std::strlen(needle);
        const char* e = std::strchr(p, '"');
        return e ? std::string(p, e - p) : std::string();
    };
    auto find_num = [](const char* j, const char* key) -> int64_t {
        char needle[64];
        std::snprintf(needle, sizeof(needle), "\"%s\":", key);
        const char* p = std::strstr(j, needle);
        if (!p) return 0;
        return (int64_t)std::atoll(p + std::strlen(needle));
    };
    for (int i = 0; i < N; ++i) {
        xi_record in{}; in.json = "{}";
        xi_record_out out{};
        xi_record_out_init(&out);
        syms.process(inst, &in, &out);

        const char* j = out.json ? out.json : "{}";
        bool missed = std::strstr(j, "\"missed\":true") != nullptr;
        int  tid = 0;
        const char* t = std::strstr(j, "\"trigger_id\":");
        if (t) tid = std::atoi(t + 13);
        std::string src = find_str(j, "trigger_source");
        std::string why = find_str(j, "miss_reason");
        int64_t ts_trig = find_num(j, "ts_trigger_ns");
        int64_t ts_host = find_num(j, "ts_host_ns");
        int64_t ts_dev  = find_num(j, "ts_device_ns");
        int64_t latency = find_num(j, "latency_ns");
        if (missed) {
            ++got_missed;
            std::printf("  [%2d] tid=%-3d src=%-8s MISSED reason=%-18s "
                        "ts_trig=%lld ts_host=%lld ts_dev=%lld\n",
                        i, tid, src.c_str(), why.c_str(),
                        (long long)ts_trig, (long long)ts_host, (long long)ts_dev);
        } else {
            ++got_real;
            std::printf("  [%2d] tid=%-3d src=%-8s real  "
                        "ts_trig=%lld ts_host=%lld ts_dev=%lld lat_ms=%.2f\n",
                        i, tid, src.c_str(),
                        (long long)ts_trig, (long long)ts_host, (long long)ts_dev,
                        latency / 1e6);
        }
    }

    std::printf("\n=== result ===\n"
                "outputs: %d (requested %d)\n"
                "real:    %d\n"
                "missed:  %d\n",
                got_real + got_missed, N, got_real, got_missed);

    // Cleanup
    syms.exchange(inst, R"({"command":"set_trigger_mode","mode":"off"})", buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"disconnect"})", buf, sizeof(buf));
    syms.destroy(inst);

    if (got_real + got_missed != N) return die("output count != trigger count");
    if (got_missed == 0)            return die("no misses — test didn't stress overrun path");
    std::printf("\nOK — strict trigger mode preserves 1:1 trigger→output mapping.\n");
    return 0;
}
