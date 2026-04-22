//
// test_schema.cpp — connect one camera, call get_schema, dump the
// result. Sanity-check for the schema-driven UI.
//
#include <xi/xi_abi.hpp>
#include <xi/xi_baseline.hpp>
#include <xi/xi_image_pool.hpp>
#include <windows.h>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
#include <chrono>

#ifndef HIKROBOT_CAMERA_DLL_PATH
#define HIKROBOT_CAMERA_DLL_PATH "hikrobot_camera.dll"
#endif

static int die(const char* w) { std::fprintf(stderr, "FAIL: %s\n", w); return 1; }

int main(int argc, char** argv) {
    HMODULE dll = LoadLibraryA(HIKROBOT_CAMERA_DLL_PATH);
    if (!dll) return die("LoadLibrary");
    auto syms = xi::baseline::load_symbols(dll);
    if (!syms.ok()) return die("syms");
    auto host = xi::ImagePool::make_host_api();

    void* inst = syms.create(&host, "schema_probe");
    if (!inst) return die("create");
    char buf[65536];

    const char* key = (argc > 1) ? argv[1] : nullptr;
    syms.exchange(inst, R"({"command":"discover"})", buf, sizeof(buf));
    if (key) {
        char cmd[256];
        std::snprintf(cmd, sizeof(cmd), R"({"command":"connect","device_key":"%s"})", key);
        syms.exchange(inst, cmd, buf, sizeof(buf));
    } else {
        syms.exchange(inst, R"({"command":"connect","index":0})", buf, sizeof(buf));
    }
    if (!std::strstr(buf, "\"connected\":true")) return die("connect");

    syms.exchange(inst, R"({"command":"get_schema"})", buf, sizeof(buf));
    std::printf("%s\n", buf);

    syms.exchange(inst, R"({"command":"disconnect"})", buf, sizeof(buf));
    syms.destroy(inst);
    return 0;
}
