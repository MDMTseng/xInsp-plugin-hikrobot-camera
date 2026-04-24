cmake_minimum_required(VERSION 3.16)
project(hikrobot_camera CXX C)
set(CMAKE_CXX_STANDARD 20)

# XINSP2_ROOT — env var preferred; auto-detect sibling xInsp2 checkout.
if(NOT DEFINED XINSP2_ROOT AND DEFINED ENV{XINSP2_ROOT})
    set(XINSP2_ROOT $ENV{XINSP2_ROOT})
endif()
if(NOT DEFINED XINSP2_ROOT)
    foreach(rel "../../xInsp2" "../../../xInsp2" "../../../../xInsp2")
        get_filename_component(_p "${CMAKE_CURRENT_SOURCE_DIR}/${rel}" ABSOLUTE)
        if(EXISTS "${_p}/sdk/cmake/xinsp2_plugin.cmake")
            set(XINSP2_ROOT ${_p})
            break()
        endif()
    endforeach()
endif()
include(${XINSP2_ROOT}/sdk/cmake/xinsp2_plugin.cmake)

# --- Find MVS SDK -----------------------------------------------------
#
# Order of attempts:
#   1. -DMVS_ROOT=<path> override
#   2. $MVCAM_COMMON_RUNENV env var (MVS installer sets this)
#   3. Standard install locations
#
if(NOT DEFINED MVS_ROOT AND DEFINED ENV{MVCAM_COMMON_RUNENV})
    set(MVS_ROOT $ENV{MVCAM_COMMON_RUNENV})
endif()
if(NOT DEFINED MVS_ROOT)
    foreach(p
        "C:/Program Files (x86)/MVS/Development"
        "C:/Program Files/MVS/Development")
        if(EXISTS "${p}/Includes/MvCameraControl.h")
            set(MVS_ROOT "${p}")
            break()
        endif()
    endforeach()
endif()

if(NOT MVS_ROOT OR NOT EXISTS "${MVS_ROOT}/Includes/MvCameraControl.h")
    message(WARNING
        "hikrobot_camera: MVS SDK not found — plugin will not be built.\n"
        "  Install HikRobot MVS (https://www.hikrobotics.com/) or pass\n"
        "  -DMVS_ROOT=<Development-folder-path> on the cmake command line.")
    return()
endif()

set(MVS_INCLUDE "${MVS_ROOT}/Includes")
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(MVS_LIB_DIR "${MVS_ROOT}/Libraries/win64")
else()
    set(MVS_LIB_DIR "${MVS_ROOT}/Libraries/win32")
endif()

find_library(MVS_LIBRARY NAMES MvCameraControl PATHS "${MVS_LIB_DIR}" NO_DEFAULT_PATH)
if(NOT MVS_LIBRARY)
    message(WARNING "hikrobot_camera: MvCameraControl.lib not found in ${MVS_LIB_DIR} — skipping.")
    return()
endif()

message(STATUS "hikrobot_camera: MVS include = ${MVS_INCLUDE}")
message(STATUS "hikrobot_camera: MVS lib     = ${MVS_LIBRARY}")

# --- Plugin target ----------------------------------------------------
xinsp2_add_plugin(hikrobot_camera hikrobot_camera.cpp)
target_include_directories(hikrobot_camera PRIVATE ${MVS_INCLUDE})
target_link_libraries(hikrobot_camera PRIVATE ${MVS_LIBRARY})

if(MSVC)
    target_compile_options(hikrobot_camera PRIVATE /utf-8 /wd4244 /wd4267)
    target_compile_definitions(hikrobot_camera PRIVATE _CRT_SECURE_NO_WARNINGS NOMINMAX)
endif()

# --- Native test ------------------------------------------------------
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_native.cpp)
    xinsp2_add_plugin_test(hikrobot_camera_test tests/test_native.cpp)
    target_compile_definitions(hikrobot_camera_test PRIVATE
        HIKROBOT_CAMERA_DLL_PATH="${CMAKE_CURRENT_SOURCE_DIR}/hikrobot_camera.dll")
    add_dependencies(hikrobot_camera_test hikrobot_camera)
    enable_testing()
    add_test(NAME hikrobot_camera_test
             COMMAND hikrobot_camera_test
             WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
endif()

# --- Live capture smoke (requires a real camera; run manually) --------
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_live.cpp)
    xinsp2_add_plugin_test(hikrobot_camera_live tests/test_live.cpp)
    target_compile_definitions(hikrobot_camera_live PRIVATE
        HIKROBOT_CAMERA_DLL_PATH="${CMAKE_CURRENT_SOURCE_DIR}/hikrobot_camera.dll")
    add_dependencies(hikrobot_camera_live hikrobot_camera)
endif()

# --- Trigger-overrun demo (requires a real camera; run manually) ------
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_overrun.cpp)
    xinsp2_add_plugin_test(hikrobot_camera_overrun tests/test_overrun.cpp)
    target_compile_definitions(hikrobot_camera_overrun PRIVATE
        HIKROBOT_CAMERA_DLL_PATH="${CMAKE_CURRENT_SOURCE_DIR}/hikrobot_camera.dll")
    add_dependencies(hikrobot_camera_overrun hikrobot_camera)
endif()

# --- Strict trigger mode demo (requires a real camera; run manually) --
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_strict.cpp)
    xinsp2_add_plugin_test(hikrobot_camera_strict tests/test_strict.cpp)
    target_compile_definitions(hikrobot_camera_strict PRIVATE
        HIKROBOT_CAMERA_DLL_PATH="${CMAKE_CURRENT_SOURCE_DIR}/hikrobot_camera.dll")
    add_dependencies(hikrobot_camera_strict hikrobot_camera)
endif()

# --- Two-camera hardware-trigger E2E (real cameras + ESP32) -----------
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_two_cam.cpp)
    xinsp2_add_plugin_test(hikrobot_two_cam tests/test_two_cam.cpp)
    target_compile_definitions(hikrobot_two_cam PRIVATE
        HIKROBOT_CAMERA_DLL_PATH="${CMAKE_CURRENT_SOURCE_DIR}/hikrobot_camera.dll")
    add_dependencies(hikrobot_two_cam hikrobot_camera)
endif()

# --- Two-camera software-trigger sanity (no ESP32 required) -----------
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_two_cam_sw.cpp)
    xinsp2_add_plugin_test(hikrobot_two_cam_sw tests/test_two_cam_sw.cpp)
    target_compile_definitions(hikrobot_two_cam_sw PRIVATE
        HIKROBOT_CAMERA_DLL_PATH="${CMAKE_CURRENT_SOURCE_DIR}/hikrobot_camera.dll")
    add_dependencies(hikrobot_two_cam_sw hikrobot_camera)
endif()

# --- Underrun stress (requires 1 real camera on slow USB + ESP32) -----
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_underrun.cpp)
    xinsp2_add_plugin_test(hikrobot_underrun tests/test_underrun.cpp)
    target_compile_definitions(hikrobot_underrun PRIVATE
        HIKROBOT_CAMERA_DLL_PATH="${CMAKE_CURRENT_SOURCE_DIR}/hikrobot_camera.dll")
    add_dependencies(hikrobot_underrun hikrobot_camera)
endif()

# --- Event probe (1 camera + ESP32) -----------------------------------
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_event_probe.cpp)
    xinsp2_add_plugin_test(hikrobot_event_probe tests/test_event_probe.cpp)
    target_compile_definitions(hikrobot_event_probe PRIVATE
        HIKROBOT_CAMERA_DLL_PATH="${CMAKE_CURRENT_SOURCE_DIR}/hikrobot_camera.dll")
    add_dependencies(hikrobot_event_probe hikrobot_camera)
endif()

# --- Timestamp dump: 30 triggers @ 10 Hz, normalized deltas -----------
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_ts_dump.cpp)
    xinsp2_add_plugin_test(hikrobot_ts_dump tests/test_ts_dump.cpp)
    target_compile_definitions(hikrobot_ts_dump PRIVATE
        HIKROBOT_CAMERA_DLL_PATH="${CMAKE_CURRENT_SOURCE_DIR}/hikrobot_camera.dll")
    add_dependencies(hikrobot_ts_dump hikrobot_camera)
endif()

# --- Trigger probe: which ESP32 pin reaches which camera --------------
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_trig_probe.cpp)
    xinsp2_add_plugin_test(hikrobot_trig_probe tests/test_trig_probe.cpp)
    target_compile_definitions(hikrobot_trig_probe PRIVATE
        HIKROBOT_CAMERA_DLL_PATH="${CMAKE_CURRENT_SOURCE_DIR}/hikrobot_camera.dll")
    add_dependencies(hikrobot_trig_probe hikrobot_camera)
endif()

# --- IO Line1 probe ---------------------------------------------------
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_io_line1.cpp)
    xinsp2_add_plugin_test(hikrobot_io_line1 tests/test_io_line1.cpp)
    target_compile_definitions(hikrobot_io_line1 PRIVATE
        HIKROBOT_CAMERA_DLL_PATH="${CMAKE_CURRENT_SOURCE_DIR}/hikrobot_camera.dll")
    add_dependencies(hikrobot_io_line1 hikrobot_camera)
endif()

# --- IO probe: iterate LineSelector / UserOutputSelector --------------
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_io_probe.cpp)
    xinsp2_add_plugin_test(hikrobot_io_probe tests/test_io_probe.cpp)
    target_compile_definitions(hikrobot_io_probe PRIVATE
        HIKROBOT_CAMERA_DLL_PATH="${CMAKE_CURRENT_SOURCE_DIR}/hikrobot_camera.dll")
    add_dependencies(hikrobot_io_probe hikrobot_camera)
endif()

# --- Probe any cam at its persisted ROI (no ROI change) ---------------
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_any_cam_noroi.cpp)
    xinsp2_add_plugin_test(hikrobot_any_cam_noroi tests/test_any_cam_noroi.cpp)
    target_compile_definitions(hikrobot_any_cam_noroi PRIVATE
        HIKROBOT_CAMERA_DLL_PATH="${CMAKE_CURRENT_SOURCE_DIR}/hikrobot_camera.dll")
    add_dependencies(hikrobot_any_cam_noroi hikrobot_camera)
endif()

# --- Sweep any camera by device_key: full-res 10 fps 5 s --------------
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_any_cam.cpp)
    xinsp2_add_plugin_test(hikrobot_any_cam tests/test_any_cam.cpp)
    target_compile_definitions(hikrobot_any_cam PRIVATE
        HIKROBOT_CAMERA_DLL_PATH="${CMAKE_CURRENT_SOURCE_DIR}/hikrobot_camera.dll")
    add_dependencies(hikrobot_any_cam hikrobot_camera)
endif()

# --- CA050 alone free-run ---------------------------------------------
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_ca050_alone.cpp)
    xinsp2_add_plugin_test(hikrobot_ca050_alone tests/test_ca050_alone.cpp)
    target_compile_definitions(hikrobot_ca050_alone PRIVATE
        HIKROBOT_CAMERA_DLL_PATH="${CMAKE_CURRENT_SOURCE_DIR}/hikrobot_camera.dll")
    add_dependencies(hikrobot_ca050_alone hikrobot_camera)
endif()

# --- Free-run ALL cams streaming uncapped, measure throughput ---------
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_all_cam_freerun.cpp)
    xinsp2_add_plugin_test(hikrobot_all_cam_freerun tests/test_all_cam_freerun.cpp)
    target_compile_definitions(hikrobot_all_cam_freerun PRIVATE
        HIKROBOT_CAMERA_DLL_PATH="${CMAKE_CURRENT_SOURCE_DIR}/hikrobot_camera.dll")
    add_dependencies(hikrobot_all_cam_freerun hikrobot_camera)
endif()

# --- Free-run four-cam streaming at 10 fps each -----------------------
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_four_cam_freerun.cpp)
    xinsp2_add_plugin_test(hikrobot_four_cam_freerun tests/test_four_cam_freerun.cpp)
    target_compile_definitions(hikrobot_four_cam_freerun PRIVATE
        HIKROBOT_CAMERA_DLL_PATH="${CMAKE_CURRENT_SOURCE_DIR}/hikrobot_camera.dll")
    add_dependencies(hikrobot_four_cam_freerun hikrobot_camera)
endif()

# --- Free-run three-cam streaming at 10 fps each ----------------------
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_three_cam_freerun.cpp)
    xinsp2_add_plugin_test(hikrobot_three_cam_freerun tests/test_three_cam_freerun.cpp)
    target_compile_definitions(hikrobot_three_cam_freerun PRIVATE
        HIKROBOT_CAMERA_DLL_PATH="${CMAKE_CURRENT_SOURCE_DIR}/hikrobot_camera.dll")
    add_dependencies(hikrobot_three_cam_freerun hikrobot_camera)
endif()

# --- Free-run two-cam streaming at 10 fps each ------------------------
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_two_cam_freerun.cpp)
    xinsp2_add_plugin_test(hikrobot_two_cam_freerun tests/test_two_cam_freerun.cpp)
    target_compile_definitions(hikrobot_two_cam_freerun PRIVATE
        HIKROBOT_CAMERA_DLL_PATH="${CMAKE_CURRENT_SOURCE_DIR}/hikrobot_camera.dll")
    add_dependencies(hikrobot_two_cam_freerun hikrobot_camera)
endif()

# --- Readback probe: verify apply_* shadow-state is camera truth ------
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_readback.cpp)
    xinsp2_add_plugin_test(hikrobot_readback tests/test_readback.cpp)
    target_compile_definitions(hikrobot_readback PRIVATE
        HIKROBOT_CAMERA_DLL_PATH="${CMAKE_CURRENT_SOURCE_DIR}/hikrobot_camera.dll")
    add_dependencies(hikrobot_readback hikrobot_camera)
endif()

# --- Schema probe: dump get_schema JSON -------------------------------
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_schema.cpp)
    xinsp2_add_plugin_test(hikrobot_schema tests/test_schema.cpp)
    target_compile_definitions(hikrobot_schema PRIVATE
        HIKROBOT_CAMERA_DLL_PATH="${CMAKE_CURRENT_SOURCE_DIR}/hikrobot_camera.dll")
    add_dependencies(hikrobot_schema hikrobot_camera)
endif()

# --- CE200 alone full-res 10 Hz / 10 s --------------------------------
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_ce200_alone.cpp)
    xinsp2_add_plugin_test(hikrobot_ce200_alone tests/test_ce200_alone.cpp)
    target_compile_definitions(hikrobot_ce200_alone PRIVATE
        HIKROBOT_CAMERA_DLL_PATH="${CMAKE_CURRENT_SOURCE_DIR}/hikrobot_camera.dll")
    add_dependencies(hikrobot_ce200_alone hikrobot_camera)
endif()

# --- Two-cam alignment under surge: sort-on-match verification -------
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_two_cam_align.cpp)
    xinsp2_add_plugin_test(hikrobot_two_cam_align tests/test_two_cam_align.cpp)
    target_compile_definitions(hikrobot_two_cam_align PRIVATE
        HIKROBOT_CAMERA_DLL_PATH="${CMAKE_CURRENT_SOURCE_DIR}/hikrobot_camera.dll")
    add_dependencies(hikrobot_two_cam_align hikrobot_camera)
endif()

# --- Surge: 10 Hz baseline + back-to-back pair (tests sensor-reject) --
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_surge.cpp)
    xinsp2_add_plugin_test(hikrobot_surge tests/test_surge.cpp)
    target_compile_definitions(hikrobot_surge PRIVATE
        HIKROBOT_CAMERA_DLL_PATH="${CMAKE_CURRENT_SOURCE_DIR}/hikrobot_camera.dll")
    add_dependencies(hikrobot_surge hikrobot_camera)
endif()

# --- Stress: both cameras, high rate, forced drops ---------------------
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_stress.cpp)
    xinsp2_add_plugin_test(hikrobot_stress tests/test_stress.cpp)
    target_compile_definitions(hikrobot_stress PRIVATE
        HIKROBOT_CAMERA_DLL_PATH="${CMAKE_CURRENT_SOURCE_DIR}/hikrobot_camera.dll")
    add_dependencies(hikrobot_stress hikrobot_camera)
endif()

# --- Event probe across all cameras (sequential; all cams + ESP32) ----
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_event_probe_all.cpp)
    xinsp2_add_plugin_test(hikrobot_event_probe_all tests/test_event_probe_all.cpp)
    target_compile_definitions(hikrobot_event_probe_all PRIVATE
        HIKROBOT_CAMERA_DLL_PATH="${CMAKE_CURRENT_SOURCE_DIR}/hikrobot_camera.dll")
    add_dependencies(hikrobot_event_probe_all hikrobot_camera)
endif()

# --- Counter source probe (1 camera + ESP32) --------------------------
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_counter_probe.cpp)
    xinsp2_add_plugin_test(hikrobot_counter_probe tests/test_counter_probe.cpp)
    target_compile_definitions(hikrobot_counter_probe PRIVATE
        HIKROBOT_CAMERA_DLL_PATH="${CMAKE_CURRENT_SOURCE_DIR}/hikrobot_camera.dll")
    add_dependencies(hikrobot_counter_probe hikrobot_camera)
endif()

# --- Tail-drop detection via camera edge counter (1 camera + ESP32) ---
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_tail_drop.cpp)
    xinsp2_add_plugin_test(hikrobot_tail_drop tests/test_tail_drop.cpp)
    target_compile_definitions(hikrobot_tail_drop PRIVATE
        HIKROBOT_CAMERA_DLL_PATH="${CMAKE_CURRENT_SOURCE_DIR}/hikrobot_camera.dll")
    add_dependencies(hikrobot_tail_drop hikrobot_camera)
endif()

# --- Timestamp interval verification (1 camera + ESP32) ----------------
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_timestamps.cpp)
    xinsp2_add_plugin_test(hikrobot_timestamps tests/test_timestamps.cpp)
    target_compile_definitions(hikrobot_timestamps PRIVATE
        HIKROBOT_CAMERA_DLL_PATH="${CMAKE_CURRENT_SOURCE_DIR}/hikrobot_camera.dll")
    add_dependencies(hikrobot_timestamps hikrobot_camera)
endif()
