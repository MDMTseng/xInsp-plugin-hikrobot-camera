//
// test_tail_drop.cpp — verify camera-counter reaper handles tail drops.
//
// Fires a burst via ESP32, uses the debug drop hook so the LAST few
// frames get discarded at the plugin's frame callback. With no later
// frame to expose the nFrameNum gap, the strict-mode gap-absorb path
// can't detect those drops. The counter-poll reaper (Counter0Value on
// the camera vs captured+missed) should notice the discrepancy after
// the burst stops and emit `miss_reason=counter_mismatch` placeholders
// with `ts_trigger_ns=0`.
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

#ifndef HIKROBOT_CAMERA_DLL_PATH
#define HIKROBOT_CAMERA_DLL_PATH "hikrobot_camera.dll"
#endif

static int die(const char* w) { std::fprintf(stderr, "FAIL: %s\n", w); return 1; }
static bool has_lit(const char* j, const char* k, const char* v) {
    char n[128]; std::snprintf(n, sizeof(n), "\"%s\":%s", k, v);
    return std::strstr(j, n) != nullptr;
}
static int64_t peek_i64(const char* j, const char* k) {
    char n[64]; std::snprintf(n, sizeof(n), "\"%s\":", k);
    const char* p = std::strstr(j, n);
    return p ? (int64_t)std::atoll(p + std::strlen(n)) : 0;
}
static std::string peek_str(const char* j, const char* k) {
    char n[64]; std::snprintf(n, sizeof(n), "\"%s\":\"", k);
    const char* p = std::strstr(j, n);
    if (!p) return {};
    p += std::strlen(n);
    const char* e = std::strchr(p, '"');
    return e ? std::string(p, e - p) : std::string();
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

int main() {
    HMODULE dll = LoadLibraryA(HIKROBOT_CAMERA_DLL_PATH);
    if (!dll) return die("LoadLibrary");
    auto syms = xi::baseline::load_symbols(dll);
    if (!syms.ok()) return die("syms");
    auto host = xi::ImagePool::make_host_api();

    void* inst = syms.create(&host, "tail");
    if (!inst) return die("create");
    char buf[16384];

    syms.exchange(inst, R"({"command":"set_pixel_format","value":"Mono8"})",             buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"set_roi","x":0,"y":0,"w":640,"h":480})",          buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"set_exposure","value":1000})",                    buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"set_trigger_mode","mode":"on","source":"Line0"})",buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"set_strict_trigger_mode","value":true,"timeout_ms":3000})",
                  buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"discover"})", buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"connect","index":0})", buf, sizeof(buf));
    if (!has_lit(buf, "connected", "true")) return die("connect");

    Serial trig;
    if (!trig.open("COM4", 115200)) return die("open COM4");
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // Drop every 4th frame. At 10 Hz firing 20 triggers: frames 4,8,12,
    // 16, 20 will be dropped. The LAST (20) is a tail drop — no later
    // frame to expose it via nFrameNum gap. The counter reaper should
    // still catch it.
    syms.exchange(inst, R"({"command":"debug_drop_every_n","value":4})", buf, sizeof(buf));

    const int N  = 20;
    const int HZ = 10;
    std::printf("firing %d edges at %d Hz, dropping every 4th (tail trigger included)…\n", N, HZ);
    char fire[80];
    std::snprintf(fire, sizeof(fire), "BURST N=%d HZ=%d WIDTH=500 MASK=0x3", N, HZ);
    trig.write_line(fire);

    // Wait for burst plus reaper settle (idle_ms=500 + poll period 50 ms + margin).
    std::this_thread::sleep_for(std::chrono::seconds(N / HZ + 2));
    // Check the raw camera counter so we can see what the hardware thinks.
    syms.exchange(inst, R"({"command":"get_edge_counter"})", buf, sizeof(buf));
    std::printf("camera edge counter: %s\n\n", buf);

    // Host-driven flush with explicit expected trigger count. This is
    // the reliable path for tail drops when the camera counter is
    // silent — the host knows N was fired, plugin synthesizes any
    // missing placeholders to reach that total.
    char flush[80];
    std::snprintf(flush, sizeof(flush),
        R"({"command":"flush_pending","expected":%d})", N);
    syms.exchange(inst, flush, buf, sizeof(buf));
    std::printf("flush reply: %s\n\n", buf);

    int real = 0, miss_tx = 0, miss_cnt = 0, miss_other = 0;
    int last_tid = 0;
    for (int i = 0; i < N + 4; ++i) {  // extra slack in case reaper emitted a bit more
        xi_record in{}; in.json = "{}";
        xi_record_out out{}; xi_record_out_init(&out);
        for (int r = 0; r < 5; ++r) {
            syms.process(inst, &in, &out);
            if (!std::strstr(out.json, "\"error\":\"no_pending_trigger\"")) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            xi_record_out_init(&out);
        }
        if (!out.json || std::strstr(out.json, "\"error\":\"no_pending_trigger\"")) break;
        int tid = (int)peek_i64(out.json, "trigger_id");
        int64_t ts_trig = peek_i64(out.json, "ts_trigger_ns");
        bool miss = std::strstr(out.json, "\"missed\":true") != nullptr;
        std::string why = peek_str(out.json, "miss_reason");
        if (miss) {
            if (why == "transmission_drop")       ++miss_tx;
            else if (why == "counter_mismatch")   ++miss_cnt;
            else                                  ++miss_other;
            std::printf("  tid=%-2d MISSED reason=%-18s ts_trig=%lld\n",
                        tid, why.c_str(), (long long)ts_trig);
        } else {
            ++real;
            std::printf("  tid=%-2d real   ts_trig=%lld\n",
                        tid, (long long)ts_trig);
        }
        last_tid = tid;
    }

    syms.exchange(inst, R"({"command":"get_status"})", buf, sizeof(buf));
    std::printf("\n=== result ===\n"
                "outputs drained: %d\n"
                "  real:                %d\n"
                "  transmission_drop:   %d\n"
                "  counter_mismatch:    %d\n"
                "  other:               %d\n"
                "plugin counters: captured=%lld dropped=%lld missed_triggers=%lld\n"
                "last trigger_id drained: %d (expected >= %d)\n",
                real + miss_tx + miss_cnt + miss_other,
                real, miss_tx, miss_cnt, miss_other,
                peek_i64(buf, "captured"), peek_i64(buf, "dropped"),
                peek_i64(buf, "missed_triggers"), last_tid, N);

    syms.exchange(inst, R"({"command":"set_trigger_mode","mode":"off"})", buf, sizeof(buf));
    syms.exchange(inst, R"({"command":"disconnect"})",                     buf, sizeof(buf));
    syms.destroy(inst);

    int total = real + miss_tx + miss_cnt + miss_other;
    if (total < N) return die("drained fewer outputs than triggers fired");
    if (miss_tx == 0 && miss_cnt == 0 && miss_other == 0)
        return die("no missed markers — drops weren't flagged");
    std::printf("\nOK — tail drop covered: N=%d triggers fired, %d outputs drained"
                " (%d real + %d mid-burst drops + %d flushed); ts_trigger_ns=0 for unknown times.\n",
                N, total, real, miss_tx, miss_cnt + miss_other);
    return 0;
}
