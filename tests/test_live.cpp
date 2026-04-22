//
// test_live.cpp — end-to-end live-capture check.
//
// Requires a real HikRobot camera (USB3 or GigE) to be connected.
// Not part of CTest — build and run manually:
//
//   cmake --build build --config Release --target hikrobot_camera_live
//   ./hikrobot_camera_live.exe
//
// Exits 0 on success, 1 on any failure. Prints the result of discover(),
// the device it chose, and a frame/sec count over a 3-second capture.
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

int main() {
    HMODULE dll = LoadLibraryA(HIKROBOT_CAMERA_DLL_PATH);
    if (!dll) return die("LoadLibrary", HIKROBOT_CAMERA_DLL_PATH);

    auto syms = xi::baseline::load_symbols(dll);
    if (!syms.ok()) return die("missing C ABI exports");

    auto host = xi::ImagePool::make_host_api();
    void* inst = syms.create(&host, "live");
    if (!inst) return die("create");

    char buf[16384];

    // 1) Discover
    int n = syms.exchange(inst, R"({"command":"discover"})", buf, sizeof(buf));
    if (n <= 0) return die("discover exchange");
    std::printf("discover: %.500s...\n\n", buf);
    if (!std::strstr(buf, "\"devices\":[\"")) {
        std::fprintf(stderr, "no devices found — check MVS and cable.\n");
        syms.destroy(inst);
        return 1;
    }

    // 2) Connect to index 0
    n = syms.exchange(inst, R"({"command":"connect","index":0})", buf, sizeof(buf));
    if (n <= 0) return die("connect exchange");
    if (!std::strstr(buf, "\"connected\":true")) {
        std::fprintf(stderr, "connect failed. Reply:\n%s\n", buf);
        syms.destroy(inst);
        return 1;
    }
    std::printf("connected. status: %.400s...\n\n", buf);

    // 2b) Push for max fps: 1 ms exposure, moderate gain, ask for 200 Hz
    //     (camera will clamp to its bandwidth ceiling). Drop ROI to a
    //     small window so we can see whether the ceiling is USB-bandwidth
    //     or something else.
    // Full-res on a USB 2.0 link is the worst case — the camera will
    // expose at ResultingFrameRate but the host pipeline can't drain
    // fast enough, so this exercise surfaces drop detection.
    const char* roi_cmd = R"({"command":"set_roi","x":0,"y":0,"w":2448,"h":2048})";
    syms.exchange(inst, roi_cmd,                                          buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"set_exposure","value":1000})",    buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"set_gain","value":5.0})",         buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"set_frame_rate","value":200.0})", buf, sizeof(buf));
    std::this_thread::sleep_for(std::chrono::milliseconds(800));

    // 3) Warm up (discard first burst which may contain buffered frames
    //    from before our settings took effect), then measure 5 s.
    n = syms.get_def(inst, buf, sizeof(buf));
    const char* c0 = std::strstr(buf, "\"captured\":");
    int warmup_cap = c0 ? std::atoi(c0 + 11) : 0;
    std::printf("warmup done at captured=%d, measuring 5 s...\n", warmup_cap);

    int64_t first_captured = warmup_cap;
    int64_t last_captured  = warmup_cap;
    auto t_start = std::chrono::steady_clock::now();
    for (int t = 0; t < 10; ++t) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        n = syms.get_def(inst, buf, sizeof(buf));
        const char* c = std::strstr(buf, "\"captured\":");
        int cap = c ? std::atoi(c + 11) : 0;
        last_captured = cap;
        auto elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t_start).count();
        std::printf("  t=%.2fs captured=%d  (Δ=%d, inst. fps=%.1f)\n",
                    elapsed, cap, (int)(cap - first_captured),
                    elapsed > 0 ? (cap - first_captured) / elapsed : 0.0);
    }

    auto elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - t_start).count();
    int delta = (int)(last_captured - first_captured);

    // Fetch dropped/lost from get_def
    n = syms.get_def(inst, buf, sizeof(buf));
    const char* cd = std::strstr(buf, "\"dropped\":");
    const char* cl = std::strstr(buf, "\"lost_packets\":");
    int dropped = cd ? std::atoi(cd + 10) : 0;
    int lost    = cl ? std::atoi(cl + 15) : 0;

    std::printf("\n=== result ===\n"
                "frames:       %d  in %.2fs  (%.2f fps)\n"
                "dropped:      %d   (camera-side frame-num gaps)\n"
                "lost_packets: %d   (per-frame packet loss)\n"
                "drop rate:    %.1f%%\n\n",
                delta, elapsed, delta / elapsed,
                dropped, lost,
                (delta + dropped) > 0 ? 100.0 * dropped / (delta + dropped) : 0.0);

    // Dump what the camera actually has configured so we can see where
    // the bottleneck is when requested != achieved fps.
    n = syms.exchange(inst, R"({"command":"diag"})", buf, sizeof(buf));
    if (n > 0) std::printf("=== camera diagnostics ===\n%s\n\n", buf);

    // 4) One process() call — verify an image comes back.
    xi_record empty_in{};
    empty_in.json = "{}";
    xi_record_out out{};
    xi_record_out_init(&out);
    syms.process(inst, &empty_in, &out);

    bool got_image = (out.image_count > 0);
    std::printf("process() returned image_count=%d json=%.200s\n",
                out.image_count, out.json ? out.json : "(null)");

    // 5) Clean up
    syms.exchange(inst, R"({"command":"disconnect"})", buf, sizeof(buf));
    syms.destroy(inst);

    if (delta <= 0) return die("no frames received in 3s");
    if (!got_image) return die("process() returned no image");
    std::printf("\nOK — live capture works.\n");
    return 0;
}
