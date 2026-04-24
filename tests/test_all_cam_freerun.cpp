//
// test_all_cam_freerun.cpp — connect every discoverable HikRobot cam
// in free-run, uncapped, and report each cam's throughput. Highlights
// the "test" camera (00DA5328883) at the top.
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

struct Cam { std::string name; std::string key; void* inst = nullptr; char buf[16384]; };

int main() {
    HMODULE dll = LoadLibraryA(HIKROBOT_CAMERA_DLL_PATH);
    auto syms  = xi::baseline::load_symbols(dll);
    auto host  = xi::ImagePool::make_host_api();

    const int DURATION_S = 10;
    const char* TEST_CAM = "00DA5328883";   // CE200-UM 20 MP

    // Discover all.
    std::vector<std::string> all_keys;
    std::vector<std::string> all_labels;
    {
        void* p = syms.create(&host, "probe");
        char pb[16384]; pb[0] = 0;
        syms.exchange(p, R"({"command":"discover"})", pb, sizeof(pb));
        const char* q = pb;
        while ((q = std::strstr(q, "\"USB:")) != nullptr) {
            const char* e = std::strchr(q + 1, '"');
            if (!e) break;
            all_keys.emplace_back(q + 1, e - q - 1);
            q = e + 1;
        }
        // Also parse labels in order (zip-aligned with keys).
        const char* d = pb;
        while ((d = std::strstr(d, "\"[USB3]")) != nullptr) {
            const char* s = d + 1;
            const char* e = std::strchr(s, '"');
            if (!e) break;
            all_labels.emplace_back(s, e - s);
            d = e + 1;
        }
        syms.destroy(p);
    }
    if (all_keys.empty()) { std::fprintf(stderr, "no cameras\n"); return 1; }
    std::printf("discovered %zu cameras:\n", all_keys.size());
    for (size_t i = 0; i < all_keys.size(); ++i) {
        const char* tag = (all_keys[i].find(TEST_CAM) != std::string::npos)
            ? " [TEST]" : "";
        std::printf("  %s  %s%s\n",
            all_keys[i].c_str(),
            i < all_labels.size() ? all_labels[i].c_str() : "(unknown)",
            tag);
    }
    std::printf("\n");

    std::vector<Cam> cams;
    for (size_t i = 0; i < all_keys.size(); ++i) {
        Cam c;
        c.name = "cam" + std::to_string(i);
        c.key  = all_keys[i];
        cams.push_back(std::move(c));
    }

    // TEST cam → uncapped (fps=0). Every other cam → 2 fps.
    // Goal: see whether slowing the background cams frees bandwidth
    // for the TEST cam.
    for (auto& c : cams) {
        c.inst = syms.create(&host, c.name.c_str());
        if (!c.inst) { std::fprintf(stderr, "create fail %s\n", c.key.c_str()); continue; }
        syms.exchange(c.inst, R"({"command":"set_trigger_mode","mode":"off"})",    c.buf, sizeof(c.buf));
        syms.exchange(c.inst, R"({"command":"set_frame_rate","value":0})",
                      c.buf, sizeof(c.buf));
        syms.exchange(c.inst, R"({"command":"discover"})", c.buf, sizeof(c.buf));
        char cmd[256];
        std::snprintf(cmd, sizeof(cmd),
            R"({"command":"connect","device_key":"%s","defer_streaming":true})", c.key.c_str());
        syms.exchange(c.inst, cmd, c.buf, sizeof(c.buf));
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Start streaming all cams in close succession.
    std::printf("starting %zu streams uncapped...\n", cams.size());
    for (auto& c : cams) {
        if (!c.inst) continue;
        syms.exchange(c.inst, R"({"command":"start_streaming"})", c.buf, sizeof(c.buf));
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));

    auto snap = [&](Cam& c) {
        if (!c.inst) return 0;
        char sb[4096];
        syms.exchange(c.inst, R"({"command":"get_status"})", sb, sizeof(sb));
        return peek_int(sb, "captured");
    };

    // Baseline.
    std::vector<int> base(cams.size());
    for (size_t i = 0; i < cams.size(); ++i) base[i] = snap(cams[i]);
    std::this_thread::sleep_for(std::chrono::seconds(DURATION_S));

    // Final.
    std::printf("\n%-20s  %-6s  %-8s  %-12s  %-12s  %-8s\n",
        "device", "frames", "fps",    "throughput",  "dims",         "tag");
    std::printf("%-20s  %-6s  %-8s  %-12s  %-12s  %-8s\n",
        "------", "------", "---",    "----------",  "----",         "---");
    int total_fps = 0;
    double total_mbps = 0;
    for (size_t i = 0; i < cams.size(); ++i) {
        if (!cams[i].inst) continue;
        char sb[4096], db[16384];
        syms.exchange(cams[i].inst, R"({"command":"get_status"})", sb, sizeof(sb));
        syms.exchange(cams[i].inst, R"({"command":"diag"})", db, sizeof(db));
        int frames = peek_int(sb, "captured") - base[i];
        double fps = (double)frames / DURATION_S;
        double throughput = diag_cur(db, "DeviceLinkCurrentThroughput");
        double w = diag_cur(db, "Width"), h = diag_cur(db, "Height");
        const char* tag = (cams[i].key.find(TEST_CAM) != std::string::npos)
            ? "[TEST]" : "";
        std::printf("%-20s  %-6d  %-8.1f  %-12.0f  %4.0fx%-4.0f    %-8s\n",
            cams[i].key.c_str(), frames, fps, throughput / 1e6,
            w, h, tag);
        total_fps += (int)fps;
        total_mbps += throughput / 1e6;
    }
    std::printf("\ntotal: ~%d fps aggregate, %.0f MB/s summed across all cams\n",
        total_fps, total_mbps);

    for (auto& c : cams) {
        if (!c.inst) continue;
        syms.exchange(c.inst, R"({"command":"stop_streaming"})", c.buf, sizeof(c.buf));
        syms.exchange(c.inst, R"({"command":"disconnect"})",     c.buf, sizeof(c.buf));
        syms.destroy(c.inst);
    }
    return 0;
}
