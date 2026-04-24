//
// test_io_probe.cpp — iterate LineSelector values + UserOutputSelector
// values and dump which nodes the camera exposes for each, so we can
// see which IO features come and go per selector state.
//
#include <xi/xi_abi.hpp>
#include <xi/xi_baseline.hpp>
#include <xi/xi_image_pool.hpp>
#include <windows.h>
#include <cstdio>
#include <cstring>
#include <string>

#ifndef HIKROBOT_CAMERA_DLL_PATH
#define HIKROBOT_CAMERA_DLL_PATH "hikrobot_camera.dll"
#endif

int main(int argc, char** argv) {
    const char* key = argc > 1 ? argv[1] : "USB:00DA5328883";
    HMODULE dll = LoadLibraryA(HIKROBOT_CAMERA_DLL_PATH);
    auto s = xi::baseline::load_symbols(dll);
    auto host = xi::ImagePool::make_host_api();
    void* inst = s.create(&host, "io_probe");
    char buf[65536];
    s.exchange(inst, R"({"command":"discover"})", buf, sizeof(buf));
    char cmd[256];
    std::snprintf(cmd, sizeof(cmd), R"({"command":"connect","device_key":"%s"})", key);
    s.exchange(inst, cmd, buf, sizeof(buf));

    auto dump_io = [&](const char* label) {
        s.exchange(inst, R"({"command":"get_schema"})", buf, sizeof(buf));
        std::printf("--- %s ---\n", label);
        // Find each feature whose group == "io" and print name / type / cur.
        const char* p = buf;
        while ((p = std::strstr(p, "{\"name\":\"")) != nullptr) {
            const char* obj_end = p + 1;
            int depth = 1;
            for (; *obj_end && depth > 0; ++obj_end) {
                if (*obj_end == '{') ++depth;
                else if (*obj_end == '}') --depth;
            }
            if (!*obj_end) break;
            std::string obj(p, obj_end - p);
            if (obj.find("\"group\":\"io\"") != std::string::npos) {
                // Extract name / type / cur quickly.
                auto get = [&](const char* key) -> std::string {
                    char n[64]; std::snprintf(n, sizeof(n), "\"%s\":\"", key);
                    auto ki = obj.find(n);
                    if (ki == std::string::npos) {
                        // fall back to unquoted value (bool/int/float cur)
                        std::snprintf(n, sizeof(n), "\"%s\":", key);
                        ki = obj.find(n);
                        if (ki == std::string::npos) return "";
                        ki += std::strlen(n);
                        auto ke = obj.find_first_of(",}", ki);
                        return obj.substr(ki, ke - ki);
                    }
                    ki += std::strlen(n);
                    auto ke = obj.find('"', ki);
                    return obj.substr(ki, ke - ki);
                };
                std::printf("    %-26s %-7s cur=%s\n",
                            get("name").c_str(),
                            get("type").c_str(),
                            get("cur").c_str());
            }
            p = obj_end;
        }
    };

    // Iterate each Line selector + each UserOutput selector.
    for (const char* line : {"Line0", "Line1", "Line2"}) {
        std::snprintf(cmd, sizeof(cmd),
            R"({"command":"set_feature","name":"LineSelector","type":"enum","value":"%s"})", line);
        s.exchange(inst, cmd, buf, sizeof(buf));
        dump_io((std::string("LineSelector=") + line).c_str());
    }
    for (const char* uo : {"UserOutput0", "UserOutput1", "UserOutput2"}) {
        std::snprintf(cmd, sizeof(cmd),
            R"({"command":"set_feature","name":"UserOutputSelector","type":"enum","value":"%s"})", uo);
        s.exchange(inst, cmd, buf, sizeof(buf));
        dump_io((std::string("UserOutputSelector=") + uo).c_str());
    }

    s.exchange(inst, R"({"command":"disconnect"})", buf, sizeof(buf));
    s.destroy(inst);
    return 0;
}
