//
// test_event_probe_all.cpp — same as test_event_probe but iterates every
// connected camera. Reports per-model which events fire and whether
// strict-mode detection (line0_edges, counter_mismatch reaper, etc.)
// lights up on that firmware.
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
static std::string peek_str_field(const char* j, const char* k) {
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

struct CamResult {
    std::string key;
    std::string label;
    std::vector<std::string> events_fired;
    int captured = 0;
    int line0_edges = 0;
    int line0_falling = 0;
    int mv_received = 0;
    int triggers_sent_hw = 0;
};

int main() {
    HMODULE dll = LoadLibraryA(HIKROBOT_CAMERA_DLL_PATH);
    if (!dll) return die("LoadLibrary");
    auto syms = xi::baseline::load_symbols(dll);
    if (!syms.ok()) return die("syms");
    auto host = xi::ImagePool::make_host_api();

    // Discover all cameras.
    std::vector<std::pair<std::string,std::string>> devs;  // key, label
    {
        void* probe = syms.create(&host, "probe");
        char pbuf[16384]; pbuf[0] = 0;
        syms.exchange(probe, R"({"command":"discover"})", pbuf, sizeof(pbuf));
        // pull device_keys and devices in parallel
        const char* k_start = std::strstr(pbuf, "\"device_keys\":[");
        const char* l_start = std::strstr(pbuf, "\"devices\":[");
        if (!k_start || !l_start) { syms.destroy(probe); return die("discover"); }
        k_start += std::strlen("\"device_keys\":[");
        l_start += std::strlen("\"devices\":[");
        const char* k_end = std::strchr(k_start, ']');
        const char* l_end = std::strchr(l_start, ']');
        auto extract_quoted = [](const char* p, const char* e,
                                  std::vector<std::string>& out) {
            while (p < e) {
                p = std::strchr(p, '"');
                if (!p || p >= e) break;
                ++p;
                const char* q = std::strchr(p, '"');
                if (!q || q >= e) break;
                out.emplace_back(p, q - p);
                p = q + 1;
            }
        };
        std::vector<std::string> keys, labels;
        extract_quoted(k_start, k_end, keys);
        extract_quoted(l_start, l_end, labels);
        for (size_t i = 0; i < keys.size(); ++i) {
            devs.emplace_back(keys[i],
                              i < labels.size() ? labels[i] : keys[i]);
        }
        syms.destroy(probe);
    }
    std::printf("found %zu cameras:\n", devs.size());
    for (auto& d : devs) std::printf("  %s   (%s)\n", d.first.c_str(), d.second.c_str());
    std::printf("\n");
    if (devs.empty()) return die("no cameras");

    Serial trig;
    if (!trig.open("COM4", 115200)) return die("open COM4");
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    std::vector<CamResult> results;

    for (auto& [key, label] : devs) {
        std::printf("══════════════════════════════════════════════════════════\n");
        std::printf(" %s  (%s)\n", key.c_str(), label.c_str());
        std::printf("══════════════════════════════════════════════════════════\n");

        void* inst = syms.create(&host, ("cam_" + key).c_str());
        char buf[16384];

        // Configure for Line0 hardware trigger.
        syms.exchange(inst, R"({"command":"set_pixel_format","value":"Mono8"})",             buf, sizeof(buf));
        syms.exchange(inst, R"({"command":"set_roi","x":0,"y":0,"w":640,"h":480})",          buf, sizeof(buf));
        syms.exchange(inst, R"({"command":"set_exposure","value":5000})",                    buf, sizeof(buf));
        syms.exchange(inst, R"({"command":"set_trigger_mode","mode":"on","source":"Line0"})",buf, sizeof(buf));
        syms.exchange(inst, R"({"command":"discover"})", buf, sizeof(buf));
        char cmd[256];
        std::snprintf(cmd, sizeof(cmd),
            R"({"command":"connect","device_key":"%s"})", key.c_str());
        syms.exchange(inst, cmd, buf, sizeof(buf));
        if (!has_lit(buf, "connected", "true")) {
            std::printf("  ❌ connect failed — skipping\n\n");
            syms.destroy(inst);
            continue;
        }

        // Probe all known events.
        const char* probe_cmd = R"({"command":"probe_events","names":[
            "AcquisitionStart","AcquisitionEnd","AcquisitionTrigger","AcquisitionTriggerMiss",
            "FrameStart","FrameEnd","ExposureStart","ExposureEnd",
            "FrameTrigger","FrameTriggerMiss","FrameBurstStart","FrameBurstEnd",
            "LineRisingEdge","LineFallingEdge",
            "Line0RisingEdge","Line0FallingEdge","Line1RisingEdge","Line2RisingEdge",
            "DeviceTemperature","ControlLost","StreamControlLost","Error"
        ]})";
        syms.exchange(inst, probe_cmd, buf, sizeof(buf));

        // Fire a burst and wait.
        trig.write_line("BURST N=10 HZ=5 WIDTH=500 MASK=0x3");
        std::this_thread::sleep_for(std::chrono::seconds(3));

        // Collect.
        CamResult r;
        r.key = key;
        r.label = label;
        syms.exchange(inst, R"({"command":"get_event_counts"})", buf, sizeof(buf));
        std::printf("  events fired: %s\n", buf);

        syms.exchange(inst, R"({"command":"get_status"})", buf, sizeof(buf));
        r.captured         = peek_int(buf, "captured");
        r.line0_edges      = peek_int(buf, "line0_edges");
        r.line0_falling    = peek_int(buf, "line0_falling_edges");
        r.mv_received      = peek_int(buf, "mv_received_frames");
        r.triggers_sent_hw = peek_int(buf, "triggers_sent_hw");
        std::printf("  captured=%d line0_edges=%d line0_falling=%d mv_received=%d triggers_sent_hw=%d\n",
                    r.captured, r.line0_edges, r.line0_falling,
                    r.mv_received, r.triggers_sent_hw);

        results.push_back(r);

        syms.exchange(inst, R"({"command":"set_trigger_mode","mode":"off"})", buf, sizeof(buf));
        syms.exchange(inst, R"({"command":"disconnect"})",                     buf, sizeof(buf));
        syms.destroy(inst);
        std::printf("\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // Summary: do they all support the key detection signals?
    std::printf("══════════════════════════════════════════════════════════\n");
    std::printf(" SUMMARY — does strict-mode detection work on each model?\n");
    std::printf("══════════════════════════════════════════════════════════\n");
    std::printf(" %-20s %-22s %-8s %-8s %-8s\n",
                "serial", "captured/expected", "edges", "falls", "hw_slots");
    bool all_ok = true;
    for (auto& r : results) {
        bool ok = r.line0_edges == 10 && r.captured == 10;
        std::printf(" %-20s %-22s %-8d %-8d %-8d   %s\n",
                    r.key.c_str(),
                    (std::to_string(r.captured) + "/10").c_str(),
                    r.line0_edges, r.line0_falling, r.triggers_sent_hw,
                    ok ? "✅" : "⚠");
        if (!ok) all_ok = false;
    }

    std::printf("\n%s\n", all_ok
        ? "OK — all cameras expose Line0RisingEdge reliably; strict-mode detection works on every model."
        : "one or more cameras don't give us the Line0RisingEdge signal — "
          "detection would fall back to flush_pending / nFrameNum gap only");
    return all_ok ? 0 : 1;
}
