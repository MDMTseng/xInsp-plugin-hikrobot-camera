//
// test_two_cam.cpp — two-camera hardware-trigger E2E.
//
// Requires:
//   - Two HikRobot USB3 cameras visible to MVS.
//   - Each camera's Line0 trigger wired to the ESP32 running xtrig
//     (defaults Ch0 = GPIO32, Ch1 = GPIO33).
//   - xtrig firmware reachable on COM4 at 115200 baud.
//
// Flow:
//   1. Discover → expect 2 devices.
//   2. Create two plugin instances, connect to index 0 and index 1.
//   3. Configure both for TriggerMode=On, TriggerSource=Line0,
//      strict_trigger_mode=true.
//   4. Command the ESP32 to fire 10 pulses at 2 Hz on MASK=0x3 (both
//      channels simultaneously).
//   5. Drain 10 outputs from each camera in parallel; verify each slot
//      produces a real frame with matching trigger_id across both cams.
//   6. Save the last frame from each camera as a JPEG next to the test.
//

#include <xi/xi_abi.hpp>
#include <xi/xi_baseline.hpp>
#include <xi/xi_image_pool.hpp>

#include <windows.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>
#include <thread>

#ifndef HIKROBOT_CAMERA_DLL_PATH
#define HIKROBOT_CAMERA_DLL_PATH "hikrobot_camera.dll"
#endif

// --- tiny helpers -------------------------------------------------
static int die(const char* what) {
    std::fprintf(stderr, "FAIL: %s\n", what);
    return 1;
}

static int get_int(const char* json, const char* key) {
    char needle[64];
    std::snprintf(needle, sizeof(needle), "\"%s\":", key);
    const char* p = std::strstr(json, needle);
    if (!p) return -1;
    return std::atoi(p + std::strlen(needle));
}

static bool has_literal(const char* json, const char* key, const char* val) {
    char needle[128];
    std::snprintf(needle, sizeof(needle), "\"%s\":%s", key, val);
    return std::strstr(json, needle) != nullptr;
}

// Minimal base64 decoder for extracting the preview JPEG bytes.
static std::vector<uint8_t> b64_decode(const std::string& s) {
    static int8_t T[256];
    static bool init = false;
    if (!init) {
        std::memset(T, -1, sizeof(T));
        const char* A = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
        for (int i = 0; i < 64; ++i) T[(unsigned)A[i]] = (int8_t)i;
        init = true;
    }
    std::vector<uint8_t> out;
    out.reserve(s.size() * 3 / 4);
    int buf = 0, n = 0;
    for (char c : s) {
        if (c == '=' || T[(unsigned char)c] < 0) continue;
        buf = (buf << 6) | T[(unsigned char)c];
        n += 6;
        if (n >= 8) { n -= 8; out.push_back((uint8_t)((buf >> n) & 0xFF)); }
    }
    return out;
}

// --- minimal Windows serial wrapper -------------------------------
class Serial {
public:
    bool open(const char* port, int baud) {
        char name[32];
        std::snprintf(name, sizeof(name), "\\\\.\\%s", port);
        h_ = CreateFileA(name, GENERIC_READ | GENERIC_WRITE,
                         0, nullptr, OPEN_EXISTING, 0, nullptr);
        if (h_ == INVALID_HANDLE_VALUE) return false;
        DCB dcb{}; dcb.DCBlength = sizeof(dcb);
        GetCommState(h_, &dcb);
        dcb.BaudRate = baud;
        dcb.ByteSize = 8;
        dcb.Parity   = NOPARITY;
        dcb.StopBits = ONESTOPBIT;
        dcb.fDtrControl = DTR_CONTROL_DISABLE;  // avoid ESP32 auto-reset on open
        dcb.fRtsControl = RTS_CONTROL_DISABLE;
        SetCommState(h_, &dcb);
        COMMTIMEOUTS t{}; t.ReadIntervalTimeout = MAXDWORD;
        t.ReadTotalTimeoutConstant = 200;
        SetCommTimeouts(h_, &t);
        return true;
    }
    void write_line(const std::string& s) {
        std::string line = s + "\n";
        DWORD w = 0;
        WriteFile(h_, line.data(), (DWORD)line.size(), &w, nullptr);
    }
    ~Serial() { if (h_ != INVALID_HANDLE_VALUE) CloseHandle(h_); }
private:
    HANDLE h_ = INVALID_HANDLE_VALUE;
};

// --- per-camera setup --------------------------------------------
struct Cam {
    const char* name;
    std::string device_key;   // "USB:<serial>" — passed in, no index lookup
    void*       inst = nullptr;
    char        buf[16384];

    bool configure(const xi::baseline::PluginSymbols& s, xi_host_api* host) {
        inst = s.create(host, name);
        if (!inst) return false;

        // Pre-stage ALL config — including trigger mode — so connect()
        // opens the camera with everything in place and StartGrabbing
        // enters trigger-wait state immediately (no free-run flicker).
        // A free-run StartGrabbing on full-res 20 MP over shared USB
        // fails outright; small ROI + trigger-from-the-start avoids it.
        s.exchange(inst, R"({"command":"set_pixel_format","value":"Mono8"})", buf, sizeof(buf));
        s.exchange(inst, R"({"command":"set_roi","x":0,"y":0,"w":640,"h":480})", buf, sizeof(buf));
        s.exchange(inst, R"({"command":"set_exposure","value":5000})",        buf, sizeof(buf));
        s.exchange(inst,
            R"({"command":"set_trigger_mode","mode":"on","source":"Line0"})", buf, sizeof(buf));
        s.exchange(inst,
            R"({"command":"set_strict_trigger_mode","value":true,"timeout_ms":1500})",
            buf, sizeof(buf));

        // Discover so the plugin's internal enumerate list has entries.
        s.exchange(inst, R"({"command":"discover"})", buf, sizeof(buf));

        char cmd[256];
        std::snprintf(cmd, sizeof(cmd),
            R"({"command":"connect","device_key":"%s"})", device_key.c_str());
        if (s.exchange(inst, cmd, buf, sizeof(buf)) <= 0) return false;
        if (!has_literal(buf, "connected", "true")) return false;
        return true;
    }

    bool save_preview(const xi::baseline::PluginSymbols& s, const std::string& path) {
        if (s.exchange(inst,
            R"({"command":"get_preview","max_side":640,"quality":80})",
            buf, sizeof(buf)) <= 0) return false;
        const char* j = buf;
        const char* k = std::strstr(j, "\"jpeg_b64\":\"");
        if (!k) return false;
        k += std::strlen("\"jpeg_b64\":\"");
        const char* e = std::strchr(k, '"');
        if (!e) return false;
        std::string b64(k, e - k);
        auto bytes = b64_decode(b64);
        std::ofstream f(path, std::ios::binary);
        f.write((const char*)bytes.data(), (std::streamsize)bytes.size());
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

// --- main --------------------------------------------------------
int main() {
    std::fprintf(stderr, "[trace] start\n"); std::fflush(stderr);
    HMODULE dll = LoadLibraryA(HIKROBOT_CAMERA_DLL_PATH);
    if (!dll) return die("LoadLibrary");
    auto syms = xi::baseline::load_symbols(dll);
    if (!syms.ok()) return die("load_symbols");
    auto host = xi::ImagePool::make_host_api();

    // Probe: discover once, extract the two device_keys.
    std::vector<std::string> device_keys;
    {
        void* probe = syms.create(&host, "probe");
        if (!probe) return die("probe create");
        char pbuf[16384]; pbuf[0] = 0;
        syms.exchange(probe, R"({"command":"discover"})", pbuf, sizeof(pbuf));
        const char* p = pbuf;
        while ((p = std::strstr(p, "\"USB:")) != nullptr) {
            const char* e = std::strchr(p + 1, '"');
            if (!e) break;
            device_keys.emplace_back(p + 1, e - p - 1);
            p = e + 1;
        }
        const char* q = pbuf;
        while ((q = std::strstr(q, "\"GIGE:")) != nullptr) {
            const char* e = std::strchr(q + 1, '"');
            if (!e) break;
            device_keys.emplace_back(q + 1, e - q - 1);
            q = e + 1;
        }
        std::printf("discovered %zu device(s):\n", device_keys.size());
        for (auto& k : device_keys) std::printf("  %s\n", k.c_str());
        std::printf("\n");
        syms.destroy(probe);
        if (device_keys.size() < 2) return die("need at least 2 cameras visible to MVS");
    }

    // Open ESP32.
    Serial trig;
    if (!trig.open("COM4", 115200)) return die("open COM4");
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // Two camera instances, each bound to a specific device by key.
    Cam cam0{"cam0_hw", device_keys[0]};
    Cam cam1{"cam1_hw", device_keys[1]};
    if (!cam0.configure(syms, &host)) { cam1.teardown(syms); return die("configure cam0"); }
    if (!cam1.configure(syms, &host)) { cam0.teardown(syms); return die("configure cam1"); }
    std::printf("both cameras connected + armed for strict hw trigger on Line0\n\n");

    auto diag_print = [&](Cam& c) {
        char dbuf[4096];
        syms.exchange(c.inst, R"({"command":"diag"})", dbuf, sizeof(dbuf));
        // Extract the "cur" inside the nested object for each feature.
        auto peek_cur = [&](const char* key) {
            char needle[64];
            std::snprintf(needle, sizeof(needle), "\"%s\":{", key);
            const char* p = std::strstr(dbuf, needle);
            if (!p) return std::string("<none>");
            const char* cur = std::strstr(p, "\"cur\":");
            if (!cur) return std::string("<none>");
            cur += std::strlen("\"cur\":");
            const char* e = std::strpbrk(cur, ",}");
            return e ? std::string(cur, e - cur) : std::string();
        };
        // Also pull the plugin-level state.
        char st[4096];
        syms.exchange(c.inst, R"({"command":"get_status"})", st, sizeof(st));
        auto peek_raw = [&](const char* json, const char* key) {
            char needle[64];
            std::snprintf(needle, sizeof(needle), "\"%s\":", key);
            const char* p = std::strstr(json, needle);
            if (!p) return std::string("<none>");
            p += std::strlen(needle);
            const char* e = std::strpbrk(p, ",}");
            return e ? std::string(p, e - p) : std::string();
        };
        std::printf("  %s:\n", c.name);
        std::printf("    plugin: connected=%s streaming=%s trigger_mode=%s trigger_source=%s\n",
                    peek_raw(st, "connected").c_str(),
                    peek_raw(st, "streaming").c_str(),
                    peek_raw(st, "trigger_mode").c_str(),
                    peek_raw(st, "trigger_source").c_str());
        std::printf("    camera: TriggerMode=%s Width=%s Height=%s Exposure=%s\n",
                    peek_cur("TriggerMode").c_str(),
                    peek_cur("Width").c_str(),
                    peek_cur("Height").c_str(),
                    peek_cur("ExposureTime").c_str());
    };
    diag_print(cam0);
    diag_print(cam1);
    std::printf("\n");

    // Let the cameras settle before firing.
    std::this_thread::sleep_for(std::chrono::milliseconds(600));

    // Fire bursts from ESP32. 10 pulses at 2 Hz, 500us wide, both channels.
    const int N = 10;
    std::printf("firing %d triggers at 2 Hz on ch0+ch1 via esp32…\n", N);
    trig.write_line("BURST N=10 HZ=2 WIDTH=500 MASK=0x3");
    // Give the burst + camera transfer a moment to complete.
    std::this_thread::sleep_for(std::chrono::milliseconds(6000));

    // Snapshot capture counters — tells us whether frames reached the
    // plugin's frame callback (independent of strict-mode pairing).
    auto snap_capture = [&](Cam& c) {
        char sbuf[4096];
        syms.exchange(c.inst, R"({"command":"get_status"})", sbuf, sizeof(sbuf));
        auto peek = [&](const char* k) {
            char needle[64]; std::snprintf(needle, sizeof(needle), "\"%s\":", k);
            const char* p = std::strstr(sbuf, needle);
            if (!p) return std::string("<none>");
            p += std::strlen(needle);
            const char* e = std::strpbrk(p, ",}");
            return e ? std::string(p, e - p) : std::string();
        };
        std::printf("  %s: captured=%s dropped=%s triggers_sent_hw=%s missed=%s\n",
                    c.name, peek("captured").c_str(), peek("dropped").c_str(),
                    peek("triggers_sent_hw").c_str(),
                    peek("missed_triggers").c_str());
    };
    std::printf("\npost-burst counter snapshot:\n");
    snap_capture(cam0);
    snap_capture(cam1);
    std::printf("\n");

    // Drain: pull one process() output per trigger from each camera.
    // Loop with a small sleep if a camera reports "no_pending_trigger"
    // (slot hasn't arrived yet).
    int real0 = 0, real1 = 0, miss0 = 0, miss1 = 0;
    int matched_ids = 0;
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);

    auto drain_one = [&](Cam& c, int& real, int& miss, int& last_id) -> bool {
        xi_record in{}; in.json = "{}";
        xi_record_out out{}; xi_record_out_init(&out);
        syms.process(c.inst, &in, &out);
        const char* j = out.json ? out.json : "{}";
        if (std::strstr(j, "\"error\":\"no_pending_trigger\"")) return false;
        int tid = get_int(j, "trigger_id");
        bool missed = std::strstr(j, "\"missed\":true") != nullptr;
        if (missed) ++miss; else ++real;
        last_id = tid;
        std::printf("  %s tid=%-3d %s  image=%d\n",
                    c.name, tid, missed ? "MISSED" : "real  ", out.image_count);
        return true;
    };

    int done0 = 0, done1 = 0, last0 = 0, last1 = 0;
    while ((done0 < N || done1 < N) &&
           std::chrono::steady_clock::now() < deadline) {
        if (done0 < N && drain_one(cam0, real0, miss0, last0)) {
            ++done0;
            if (done0 <= done1) {
                if (last0 == last1) ++matched_ids;
            }
        }
        if (done1 < N && drain_one(cam1, real1, miss1, last1)) {
            ++done1;
            if (done1 <= done0) {
                if (last1 == last0) ++matched_ids;
            }
        }
        if (done0 < N || done1 < N)
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }

    // Save last frame from each camera (best-effort).
    cam0.save_preview(syms, "test_two_cam_cam0.jpg");
    cam1.save_preview(syms, "test_two_cam_cam1.jpg");

    std::printf("\n=== result ===\n"
                "cam0:  real=%d missed=%d  (got %d/%d)\n"
                "cam1:  real=%d missed=%d  (got %d/%d)\n"
                "matched trigger_ids per pair: %d/%d\n"
                "preview frames saved to test_two_cam_cam{0,1}.jpg\n",
                real0, miss0, done0, N,
                real1, miss1, done1, N,
                matched_ids, N);

    cam0.teardown(syms);
    cam1.teardown(syms);

    if (done0 != N || done1 != N) return die("did not drain N outputs per camera");
    if (real0 == 0 || real1 == 0) return die("neither camera received any real frames");
    std::printf("\nOK — two-camera hw trigger E2E works.\n");
    return 0;
}
