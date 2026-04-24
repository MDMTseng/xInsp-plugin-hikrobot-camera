//
// Quick: select Line1, dump LineSource's full enum value list.
//
#include <xi/xi_abi.hpp>
#include <xi/xi_baseline.hpp>
#include <xi/xi_image_pool.hpp>
#include <windows.h>
#include <cstdio>
#include <cstring>

#ifndef HIKROBOT_CAMERA_DLL_PATH
#define HIKROBOT_CAMERA_DLL_PATH "hikrobot_camera.dll"
#endif

int main() {
    HMODULE dll = LoadLibraryA(HIKROBOT_CAMERA_DLL_PATH);
    auto s = xi::baseline::load_symbols(dll);
    auto host = xi::ImagePool::make_host_api();
    void* inst = s.create(&host, "io_line1");
    char buf[65536];
    s.exchange(inst, R"({"command":"discover"})", buf, sizeof(buf));
    s.exchange(inst, R"({"command":"connect","device_key":"USB:00DA5328883"})", buf, sizeof(buf));
    s.exchange(inst, R"({"command":"set_feature","name":"LineSelector","type":"enum","value":"Line1"})",
               buf, sizeof(buf));
    s.exchange(inst, R"({"command":"get_schema"})", buf, sizeof(buf));
    // Just print the whole thing for Python to parse.
    std::printf("%s\n", buf);
    s.exchange(inst, R"({"command":"disconnect"})", buf, sizeof(buf));
    s.destroy(inst);
    return 0;
}
