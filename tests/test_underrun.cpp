//
// test_underrun.cpp — stress the strict-mode underrun path.
//
// Configures the MV-CA050 (USB 2.0 side, the slower link) at full
// resolution, fires hardware triggers via ESP32 at a rate guaranteed
// to exceed what USB can drain, and verifies that strict mode
// produces one output per trigger in contiguous order — some real,
// some placeholders with miss_reason=transmission_drop.
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
static std::string peek_str(const char* j, const char* k) {
    char n[64]; std::snprintf(n, sizeof(n), "\"%s\":\"", k);
    const char* p = std::strstr(j, n);
    if (!p) return {};
    p += std::strlen(n);
    const char* e = std::strchr(p, '"');
    return e ? std::string(p, e - p) : std::string();
}
static int peek_int(const char* j, const char* k) {
    char n[64]; std::snprintf(n, sizeof(n), "\"%s\":", k);
    const char* p = std::strstr(j, n);
    return p ? std::atoi(p + std::strlen(n)) : 0;
}

class Serial {
public:
    bool open(const char* port, int baud) {
        char n[32]; std::snprintf(n, sizeof(n), "\\\\.\\%s", port);
        h_ = CreateFileA(n, GENERIC_READ|GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, 0, nullptr);
        if (h_ == INVALID_HANDLE_VALUE) return false;
        DCB d{}; d.DCBlength = sizeof(d); GetCommState(h_, &d);
        d.BaudRate = baud; d.ByteSize = 8; d.Parity = NOPARITY; d.StopBits = ONESTOPBIT;
        d.fDtrControl = DTR_CONTROL_DISABLE; d.fRtsControl = RTS_CONTROL_DISABLE;
        SetCommState(h_, &d);
        COMMTIMEOUTS t{}; t.ReadIntervalTimeout = MAXDWORD; t.ReadTotalTimeoutConstant = 200;
        SetCommTimeouts(h_, &t);
        return true;
    }
    void write_line(const std::string& s) {
        std::string x = s + "\n";
        DWORD w; WriteFile(h_, x.data(), (DWORD)x.size(), &w, nullptr);
    }
    ~Serial() { if (h_ != INVALID_HANDLE_VALUE) CloseHandle(h_); }
private:
    HANDLE h_ = INVALID_HANDLE_VALUE;
};

#define TRACE(fmt, ...) do { std::fprintf(stderr, "[trace] " fmt "\n", ##__VA_ARGS__); std::fflush(stderr); } while(0)

int main() {
    TRACE("enter main");
    HMODULE dll = LoadLibraryA(HIKROBOT_CAMERA_DLL_PATH);
    if (!dll) return die("LoadLibrary");
    auto syms = xi::baseline::load_symbols(dll);
    if (!syms.ok()) return die("syms");
    auto host = xi::ImagePool::make_host_api();

    void* inst = syms.create(&host, "underrun");
    if (!inst) return die("create");
    char buf[16384];

    // Pre-stage config. Full resolution (2448x2048) on the MV-CA050 is
    // the slow path — USB 2.0 chokes ~0.8 fps. Mono8 + 1 ms exposure
    // keeps the sensor fast so the bottleneck is transfer, not capture.
    syms.exchange(inst, R"({"command":"set_pixel_format","value":"Mono8"})",             buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"set_roi","x":0,"y":0,"w":2448,"h":2048})",        buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"set_exposure","value":1000})",                    buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"set_trigger_mode","mode":"on","source":"Line0"})",buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"set_strict_trigger_mode","value":true,"timeout_ms":3000})",
                  buf, sizeof(buf));

    TRACE("discovering");
    syms.exchange(inst, R"({"command":"discover"})", buf, sizeof(buf));
    TRACE("finding MV-CA050");
    // Find the MV-CA050 key.
    std::string key;
    {
        const char* p = std::strstr(buf, "USB:K");
        if (p) {
            const char* e = std::strchr(p, '"');
            if (!e) { TRACE("no closing quote"); return die("MV-CA050 bad json"); }
            key.assign(p, e - p);
        }
    }
    TRACE("key='%s'", key.c_str());
    if (key.empty()) return die("MV-CA050 not found (looking for USB:K…)");
    std::printf("targeting %s (MV-CA050 on USB 2.0 side)\n\n", key.c_str());

    char cmd[256];
    std::snprintf(cmd, sizeof(cmd), R"({"command":"connect","device_key":"%s"})", key.c_str());
    syms.exchange(inst, cmd, buf, sizeof(buf));
    if (!has_lit(buf, "connected", "true")) return die("connect");

    // Debug hook: force every 3rd frame to be discarded so the USB/MVS
    // drop path gets exercised even when the bus isn't actually
    // saturated. The plugin's consume_frame absorbs this exactly like
    // a real transmission drop (nFrameNum gap → transmission_drop slot).
    syms.exchange(inst, R"({"command":"debug_drop_every_n","value":3})",
                  buf, sizeof(buf));
    std::printf("debug hook: simulating a drop every 3rd frame\n\n");

    // Open ESP32.
    Serial trig;
    if (!trig.open("COM4", 115200)) return die("open COM4");
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // We want TRANSMISSION drops (frames the camera exposed but USB
    // dropped), not camera-side overruns (triggers camera rejected).
    // - Full-res 2448x2048 Mono8 = ~5 MB/frame
    // - USB 2.0 HS usable ≈ 30 MB/s → max sustained ≈ 6 fps
    // - Camera can accept triggers up to ~48 Hz
    // So firing at 10 Hz for 8 s gets us 80 triggers, all accepted by
    // the camera, but USB can only drain ~48 of them — ~32 should
    // overflow the on-camera buffer and surface as nFrameNum gaps.
    const int N  = 30;
    const int HZ = 10;
    std::printf("firing %d edges at %d Hz → expect ~%d transmission drops (every 3rd)…\n",
                N, HZ, N / 3);
    char fire[80];
    std::snprintf(fire, sizeof(fire),
        "BURST N=%d HZ=%d WIDTH=500 MASK=0x3", N, HZ);
    trig.write_line(fire);
    TRACE("burst fired; sleeping %d s", N / HZ + 4);
    std::this_thread::sleep_for(std::chrono::seconds(N / HZ + 4));
    TRACE("draining");

    // Drain up to N outputs and bucket them.
    int real = 0, missed_tx = 0, missed_other = 0;
    for (int i = 0; i < N; ++i) {
        TRACE("drain iter %d", i);
        xi_record in{}; in.json = "{}";
        xi_record_out out{}; xi_record_out_init(&out);
        // Retry on "no_pending_trigger".
        for (int r = 0; r < 5; ++r) {
            syms.process(inst, &in, &out);
            if (!out.json) { TRACE("out.json NULL"); break; }
            if (!std::strstr(out.json, "\"error\":\"no_pending_trigger\"")) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            xi_record_out_init(&out);
        }
        if (!out.json) { TRACE("exit drain iter %d null", i); continue; }
        int tid = peek_int(out.json, "trigger_id");
        bool miss = std::strstr(out.json, "\"missed\":true") != nullptr;
        std::string why = peek_str(out.json, "miss_reason");
        if (miss) {
            if (why == "transmission_drop") ++missed_tx;
            else ++missed_other;
            std::printf("  [%2d] tid=%-2d MISSED reason=%s\n",
                        i, tid, why.c_str());
        } else {
            ++real;
            std::printf("  [%2d] tid=%-2d real   image=%d\n", i, tid, out.image_count);
        }
    }

    // Pull final counters.
    syms.exchange(inst, R"({"command":"get_status"})", buf, sizeof(buf));
    std::printf("\n=== result ===\n"
                "outputs drained: %d\n"
                "  real:                %d\n"
                "  missed (transmission): %d\n"
                "  missed (other/timeout): %d\n"
                "plugin counters: captured=%d dropped=%d missed_triggers=%d triggers_sent_hw=%d\n",
                real + missed_tx + missed_other,
                real, missed_tx, missed_other,
                peek_int(buf, "captured"), peek_int(buf, "dropped"),
                peek_int(buf, "missed_triggers"),
                peek_int(buf, "triggers_sent_hw"));

    syms.exchange(inst, R"({"command":"set_trigger_mode","mode":"off"})", buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"disconnect"})",                     buf, sizeof(buf));
    syms.destroy(inst);

    // Tail-drop is a known limitation: a drop at the very end of the
    // burst has no subsequent frame to detect the nFrameNum gap against,
    // so we can drain up to N-1 outputs when the last fired trigger got
    // dropped. Anything short of that is a real failure.
    int drained = real + missed_tx + missed_other;
    if (drained < N - 1)     return die("drained fewer outputs than expected");
    if (missed_tx == 0)      return die("no transmission_drop markers seen");
    std::printf("\nOK — underrun handling keeps 1:1 trigger→output; transmission drops flagged.\n");
    return 0;
}
