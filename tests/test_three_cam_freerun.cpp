//
// test_three_cam_freerun.cpp — three cameras in free-run at 10 fps:
//   A = MV-CA050-11UM  (00F50664114)   5 MP mono
//   B = MV-CE200-11UC  (00K79315117)  15 MP color "TopView"
//   C = MV-CA050-12UC  (J40531452)    5 MP color
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

    const double TARGET_FPS = 10.0;
    const int    DURATION_S = 10;

    Cam A{"camA", "USB:00F50664114"};
    Cam B{"camB", "USB:00K79315117"};
    Cam C{"camC", "USB:J40531452"};
    std::printf("cameras:\n  A = %s (CA050-UM 5MP)\n  B = %s (CE200-UC 15MP)\n  C = %s (CA050-UC 5MP)\n\n",
        A.key.c_str(), B.key.c_str(), C.key.c_str());

    // All three uncapped — AFR disabled → each camera runs at its
    // sensor max. See how they share the bus at full throttle.
    if (!configure_freerun(syms, &host, A, 0.0)) { return 1; }
    std::this_thread::sleep_for(std::chrono::seconds(1));
    if (!configure_freerun(syms, &host, B, 0.0)) { teardown(syms, A); return 1; }
    std::this_thread::sleep_for(std::chrono::seconds(1));
    if (!configure_freerun(syms, &host, C, 0.0)) { teardown(syms, A); teardown(syms, B); return 1; }
    std::this_thread::sleep_for(std::chrono::seconds(2));

    std::printf("starting streams at %.1f fps...\n", TARGET_FPS);
    syms.exchange(A.inst, R"({"command":"start_streaming"})", A.buf, sizeof(A.buf));
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    syms.exchange(B.inst, R"({"command":"start_streaming"})", B.buf, sizeof(B.buf));
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    syms.exchange(C.inst, R"({"command":"start_streaming"})", C.buf, sizeof(C.buf));
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    auto snap = [&](Cam& c) {
        char sb[4096];
        syms.exchange(c.inst, R"({"command":"get_status"})", sb, sizeof(sb));
        return peek_int(sb, "captured");
    };
    int a0 = snap(A), b0 = snap(B), c0 = snap(C);
    std::printf("initial captured: A=%d B=%d C=%d\n\n", a0, b0, c0);

    std::printf("%-6s  %-8s %-8s %-8s  %-6s %-6s %-6s\n",
        "t(s)", "A.cap", "B.cap", "C.cap", "A.fps", "B.fps", "C.fps");
    std::printf("%-6s  %-8s %-8s %-8s  %-6s %-6s %-6s\n",
        "----", "-----", "-----", "-----", "-----", "-----", "-----");

    int pa = a0, pb = b0, pc = c0;
    for (int t = 1; t <= DURATION_S; ++t) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        int a = snap(A), b = snap(B), c = snap(C);
        std::printf("%-6d  %-8d %-8d %-8d  %-6d %-6d %-6d\n",
            t, a - a0, b - b0, c - c0, a - pa, b - pb, c - pc);
        pa = a; pb = b; pc = c;
    }

    auto report = [&](Cam& c, const char* label) {
        char sb[4096], db[16384];
        syms.exchange(c.inst, R"({"command":"get_status"})", sb, sizeof(sb));
        syms.exchange(c.inst, R"({"command":"diag"})", db, sizeof(db));
        std::printf("%s  captured=%-3d dropped=%-3d mv_received=%-3d mv_errors=%-3d  "
                    "throughput=%.0f MB/s  resulting_fps=%.1f\n",
            label,
            peek_int(sb, "captured") - (label[0]=='A' ? a0 : label[0]=='B' ? b0 : c0),
            peek_int(sb, "dropped"),
            peek_int(sb, "mv_received_frames"),
            peek_int(sb, "mv_error_frames"),
            diag_cur(db, "DeviceLinkCurrentThroughput") / 1e6,
            diag_cur(db, "ResultingFrameRate"));
    };
    std::printf("\n=== final ===\n");
    report(A, "A");
    report(B, "B");
    report(C, "C");

    syms.exchange(A.inst, R"({"command":"stop_streaming"})", A.buf, sizeof(A.buf));
    syms.exchange(B.inst, R"({"command":"stop_streaming"})", B.buf, sizeof(B.buf));
    syms.exchange(C.inst, R"({"command":"stop_streaming"})", C.buf, sizeof(C.buf));
    teardown(syms, A);
    teardown(syms, B);
    teardown(syms, C);
    return 0;
}
