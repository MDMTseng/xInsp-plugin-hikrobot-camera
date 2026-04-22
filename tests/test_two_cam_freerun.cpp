//
// test_two_cam_freerun.cpp — two cameras in free-run streaming at
// 10 fps each. No hardware trigger, no ESP32. Just AFR-gated
// continuous capture. Verifies both cams coexist on the bus at
// their programmed rate.
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

static int die(const char* w) { std::fprintf(stderr, "FAIL: %s\n", w); return 1; }
static bool has_lit(const char* j, const char* k, const char* v) {
    char n[128]; std::snprintf(n, sizeof(n), "\"%s\":%s", k, v);
    return std::strstr(j, n) != nullptr;
}
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

struct Cam { const char* name; std::string key; void* inst = nullptr; char buf[16384]; };

// Free-run configure: trigger mode OFF, AFR enabled at requested fps,
// defer_streaming so we can arm both cams then start them together.
static bool configure_freerun(const xi::baseline::PluginSymbols& s, xi_host_api* host,
                              Cam& c, double fps) {
    c.inst = s.create(host, c.name);
    if (!c.inst) return false;
    // Don't set pixel format — the mono MV-CA050-UM and color MV-CE200-UC
    // have disjoint supported pixel formats, so stamping a common one
    // would fail on one of them. Use whatever each camera has persisted.
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
    return has_lit(c.buf, "connected", "true");
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

    // Discover and pick our two known cameras.
    std::vector<std::string> keys;
    {
        void* p = syms.create(&host, "probe");
        char pb[16384]; pb[0] = 0;
        syms.exchange(p, R"({"command":"discover"})", pb, sizeof(pb));
        const char* q = pb;
        while ((q = std::strstr(q, "\"USB:")) != nullptr) {
            const char* e = std::strchr(q + 1, '"');
            if (!e) break;
            keys.emplace_back(q + 1, e - q - 1);
            q = e + 1;
        }
        syms.destroy(p);
    }
    auto find_key = [&](const char* serial) -> std::string {
        for (auto& k : keys) if (k.find(serial) != std::string::npos) return k;
        return {};
    };
    // Pair: working 5 MP CA050 + 20 MP CE200-UC "TopView".
    std::string keyA = find_key("00F50664114");  // MV-CA050-11UM
    std::string keyB = find_key("00K79315117");  // MV-CE200-11UC "TopView"
    if (keyA.empty() || keyB.empty()) {
        if (keys.size() < 2) return die("need 2 cameras visible to MVS");
        keyA = keys[0]; keyB = keys[1];
    }
    std::printf("using cameras:\n  A = %s\n  B = %s\n\n", keyA.c_str(), keyB.c_str());

    Cam A{"camA_freerun", keyA};
    Cam B{"camB_freerun", keyB};
    const double TARGET_FPS = 10.0;
    const int    DURATION_S = 10;

    if (!configure_freerun(syms, &host, A, TARGET_FPS)) { teardown(syms, B); return die("configure A"); }
    std::this_thread::sleep_for(std::chrono::seconds(2));
    if (!configure_freerun(syms, &host, B, TARGET_FPS)) { teardown(syms, A); return die("configure B"); }
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Start streaming on both cameras in close succession.
    std::printf("starting streams at %.1f fps...\n", TARGET_FPS);
    syms.exchange(A.inst, R"({"command":"start_streaming"})", A.buf, sizeof(A.buf));
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    syms.exchange(B.inst, R"({"command":"start_streaming"})", B.buf, sizeof(B.buf));
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // Verify both went live and report trigger mode / AFR state.
    auto report = [&](Cam& c, const char* label) {
        char sb[4096];
        syms.exchange(c.inst, R"({"command":"get_status"})", sb, sizeof(sb));
        std::printf("  %s streaming=%s trigger_mode=%s fps=%.2f exposure=%.0f err=%s\n",
            label,
            has_lit(sb, "streaming", "true") ? "YES" : "NO",
            std::strstr(sb, "\"trigger_mode\":\"on\"") ? "on" : "off",
            peek_d(sb, "fps"),
            peek_d(sb, "exposure_us"),
            std::strstr(sb, "\"last_error\":\"\"") ? "none" :
                (std::strstr(sb, "\"last_error\":") ? "SEE_BELOW" : "?"));
    };
    report(A, "A");
    report(B, "B");

    // Snapshot starting captured counts.
    auto snap = [&](Cam& c) {
        char sb[4096];
        syms.exchange(c.inst, R"({"command":"get_status"})", sb, sizeof(sb));
        return peek_int(sb, "captured");
    };
    int capA0 = snap(A), capB0 = snap(B);
    std::printf("initial captured: A=%d B=%d (should be 0)\n\n", capA0, capB0);

    // Poll every second for DURATION_S seconds.
    std::printf("%-6s  %-10s  %-10s  %-12s  %-12s\n",
        "t(s)", "A.captured", "B.captured", "A.fps_inst", "B.fps_inst");
    std::printf("%-6s  %-10s  %-10s  %-12s  %-12s\n",
        "----", "----------", "----------", "----------", "----------");

    int prevA = capA0, prevB = capB0;
    for (int t = 1; t <= DURATION_S; ++t) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        int capA = snap(A), capB = snap(B);
        double fA = capA - prevA, fB = capB - prevB;
        std::printf("%-6d  %-10d  %-10d  %-12.2f  %-12.2f\n",
            t, capA - capA0, capB - capB0, fA, fB);
        prevA = capA; prevB = capB;
    }

    // Final snapshot with resulting / programmed fps via diag.
    auto diagf = [&](Cam& c, const char* label) {
        char db[16384];
        syms.exchange(c.inst, R"({"command":"diag"})", db, sizeof(db));
        auto get_cur = [&](const char* key) -> double {
            char n[64]; std::snprintf(n, sizeof(n), "\"%s\":{", key);
            const char* p = std::strstr(db, n);
            if (!p) return -1;
            const char* cur = std::strstr(p, "\"cur\":");
            return cur ? std::atof(cur + 6) : -1;
        };
        char sb[4096];
        syms.exchange(c.inst, R"({"command":"get_status"})", sb, sizeof(sb));
        std::printf("%s  programmed=%.2f  resulting=%.2f  captured=%d  dropped=%d  lost_pkts=%d\n",
            label,
            get_cur("AcquisitionFrameRate"),
            get_cur("ResultingFrameRate"),
            peek_int(sb, "captured"),
            peek_int(sb, "dropped"),
            peek_int(sb, "lost_packets"));
    };
    std::printf("\n=== final ===\n");
    diagf(A, "A");
    diagf(B, "B");

    // Deep dive: USB transport stats.
    auto deep = [&](Cam& c, const char* label) {
        char sb[8192], db[16384];
        syms.exchange(c.inst, R"({"command":"get_status"})", sb, sizeof(sb));
        syms.exchange(c.inst, R"({"command":"diag"})", db, sizeof(db));
        auto get_cur = [&](const char* key) -> double {
            char n[64]; std::snprintf(n, sizeof(n), "\"%s\":{", key);
            const char* p = std::strstr(db, n);
            if (!p) return -1;
            const char* cur = std::strstr(p, "\"cur\":");
            return cur ? std::atof(cur + 6) : -1;
        };
        std::printf("%s  link_speed=%.0f  throughput(B/s)=%.0f  "
                    "mv_received=%d  mv_errors=%d  mv_lost_frames=%d\n",
            label,
            get_cur("DeviceLinkSpeed"),
            get_cur("DeviceLinkCurrentThroughput"),
            peek_int(sb, "mv_received_frames"),
            peek_int(sb, "mv_error_frames"),
            peek_int(sb, "mv_lost_frames"));
    };
    std::printf("\n=== transport ===\n");
    deep(A, "A");
    deep(B, "B");

    syms.exchange(A.inst, R"({"command":"stop_streaming"})", A.buf, sizeof(A.buf));
    syms.exchange(B.inst, R"({"command":"stop_streaming"})", B.buf, sizeof(B.buf));
    teardown(syms, A);
    teardown(syms, B);
    return 0;
}
