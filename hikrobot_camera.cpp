//
// hikrobot_camera.cpp — HikRobot camera source via MVS SDK.
//
// Uses MV_CC_RegisterImageCallBackEx2 for free-run streaming; the
// callback runs on an MVS-owned thread and drops the latest frame into
// a single mutex-protected slot. process() returns that slot.
//
// Non-Mono8 / non-RGB8 frames are converted in-place via
// MV_CC_ConvertPixelType — Bayer cameras "just work" as color.
//

// Pull in stb_image_write's implementation — xi::encode_jpeg falls back
// to stb's JPEG writer when IPP/OpenCV aren't available.
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include <xi/xi_abi.hpp>
#include <xi/xi_json.hpp>
#include <xi/xi_jpeg.hpp>

#include <windows.h>
#include "MvCameraControl.h"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <deque>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace {

const char* layer_name(unsigned t) {
    if (t == MV_GIGE_DEVICE) return "GigE";
    if (t == MV_USB_DEVICE)  return "USB3";
    if (t == MV_CAMERALINK_DEVICE) return "CameraLink";
    return "Other";
}

// Stringify an MVS error code so the UI user sees something useful.
std::string mv_err(int code, const char* what) {
    char buf[160];
    std::snprintf(buf, sizeof(buf), "%s: MVS err=0x%08X", what, (unsigned)code);
    return buf;
}

std::string dev_label(const MV_CC_DEVICE_INFO* d) {
    if (!d) return "(null)";
    char buf[256];
    const char* model = "?";
    const char* serial = "?";
    const char* user = "";
    if (d->nTLayerType == MV_USB_DEVICE) {
        model  = (const char*)d->SpecialInfo.stUsb3VInfo.chModelName;
        serial = (const char*)d->SpecialInfo.stUsb3VInfo.chSerialNumber;
        user   = (const char*)d->SpecialInfo.stUsb3VInfo.chUserDefinedName;
    } else if (d->nTLayerType == MV_GIGE_DEVICE) {
        model  = (const char*)d->SpecialInfo.stGigEInfo.chModelName;
        serial = (const char*)d->SpecialInfo.stGigEInfo.chSerialNumber;
        user   = (const char*)d->SpecialInfo.stGigEInfo.chUserDefinedName;
    }
    std::snprintf(buf, sizeof(buf), "[%s] %s (%s)%s%s",
                  layer_name(d->nTLayerType),
                  model, serial,
                  (user && user[0]) ? " " : "",
                  (user && user[0]) ? user : "");
    return buf;
}

std::string base64_encode(const uint8_t* data, size_t n) {
    static const char tbl[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string out;
    out.reserve(((n + 2) / 3) * 4);
    size_t i = 0;
    for (; i + 3 <= n; i += 3) {
        uint32_t v = (data[i] << 16) | (data[i+1] << 8) | data[i+2];
        out += tbl[(v >> 18) & 63];
        out += tbl[(v >> 12) & 63];
        out += tbl[(v >> 6) & 63];
        out += tbl[v & 63];
    }
    if (i < n) {
        uint32_t v = data[i] << 16;
        if (i + 1 < n) v |= data[i+1] << 8;
        out += tbl[(v >> 18) & 63];
        out += tbl[(v >> 12) & 63];
        out += (i + 1 < n) ? tbl[(v >> 6) & 63] : '=';
        out += '=';
    }
    return out;
}

// Nearest-neighbour downsample to cap the preview payload. Operates on
// 1- or 3-channel 8-bit images (the only formats we ever put in latest_).
xi::Image downsample_to(const xi::Image& src, int max_side) {
    if (src.empty()) return {};
    if (src.width <= max_side && src.height <= max_side) return src;
    double s = (double)max_side / (double)std::max(src.width, src.height);
    int dw = std::max(1, (int)(src.width * s));
    int dh = std::max(1, (int)(src.height * s));
    xi::Image out(dw, dh, src.channels);
    const uint8_t* sp = src.data();
    uint8_t*       dp = out.data();
    for (int y = 0; y < dh; ++y) {
        int sy = (int)((y + 0.5) / s);
        if (sy >= src.height) sy = src.height - 1;
        for (int x = 0; x < dw; ++x) {
            int sx = (int)((x + 0.5) / s);
            if (sx >= src.width) sx = src.width - 1;
            const uint8_t* s_px = sp + (sy * src.width + sx) * src.channels;
            uint8_t*       d_px = dp + (y  * dw       + x ) * src.channels;
            for (int c = 0; c < src.channels; ++c) d_px[c] = s_px[c];
        }
    }
    return out;
}

std::string dev_key(const MV_CC_DEVICE_INFO* d) {
    // Stable key used by the UI for reconnect across discover() calls.
    if (!d) return "";
    if (d->nTLayerType == MV_USB_DEVICE)
        return std::string("USB:") + (const char*)d->SpecialInfo.stUsb3VInfo.chSerialNumber;
    if (d->nTLayerType == MV_GIGE_DEVICE)
        return std::string("GIGE:") + (const char*)d->SpecialInfo.stGigEInfo.chSerialNumber;
    return "OTHER";
}

} // namespace

class HikRobotCamera : public xi::Plugin {
public:
    using xi::Plugin::Plugin;

    ~HikRobotCamera() override {
        disconnect();
    }

    xi::Record process(const xi::Record& input) override {
        // Strict trigger mode: emit one frame-or-placeholder per pending
        // trigger, in FIFO order. Blocks briefly up to the slot deadline
        // so the inspection pipeline stays trigger-indexed even if some
        // frames never arrive (overrun or transmission drop).
        if (strict_trigger_mode_ && trigger_mode_ == "on" && connected_.load()) {
            return drain_one_trigger_slot(input);
        }

        xi::Image frame;
        int64_t frame_id = 0;
        int64_t host_ns = 0, device_ns = 0;
        {
            std::lock_guard<std::mutex> lk(latest_mu_);
            frame     = latest_;
            frame_id  = latest_id_;
            host_ns   = latest_host_ns_;
            device_ns = latest_device_ns_;
        }

        xi::Record out;
        if (!frame.empty()) {
            out.image("out", frame);
            out.set("frame_id", (int)frame_id);
            out.set("width",    frame.width);
            out.set("height",   frame.height);
            out.set("channels", frame.channels);
            // Timestamps: nanoseconds. host_ns is MVS's nHostTimeStamp
            // (host steady time when the SDK surfaced the frame), device_ns
            // is the camera-internal clock — use the latter for precise
            // cross-frame timing as it's not jittered by USB scheduling.
            out.set("ts_host_ns",   (double)host_ns);
            out.set("ts_device_ns", (double)device_ns);
        } else {
            const xi::Image& passthrough = input.get_image("out");
            if (!passthrough.empty()) out.image("out", passthrough);
            out.set("error", connected_.load() ? "no_frame" : "not_connected");
        }
        out.set("connected", connected_.load());
        out.set("streaming", streaming_.load());
        out.set("captured",  (int)captured_.load());
        out.set("dropped",   (int)dropped_.load());
        out.set("lost_packets", (int)lost_packets_.load());
        return out;
    }

    // Returns iterator to the unfilled/unmissed slot with the smallest
    // device_ns (real edge time). Slots with device_ns==0 (software or
    // synthesized) rank last so real hardware edges match first. Within
    // zero-device_ns slots, push order is preserved (stable FIFO tail).
    auto pick_oldest_unfilled() {
        auto best = pending_.end();
        int64_t best_key = INT64_MAX;
        for (auto it = pending_.begin(); it != pending_.end(); ++it) {
            if (it->filled || it->missed) continue;
            int64_t key = it->device_ns > 0 ? it->device_ns : INT64_MAX;
            if (key < best_key) { best_key = key; best = it; }
        }
        return best;
    }

    // Returns iterator to the slot (filled OR missed) with the smallest
    // device_ns — the next one ready to emit in physical order. Again
    // device_ns==0 ranks last, with stable push-order tiebreak.
    auto pick_oldest_ready() {
        auto best = pending_.end();
        int64_t best_key = INT64_MAX;
        for (auto it = pending_.begin(); it != pending_.end(); ++it) {
            if (!it->filled && !it->missed) continue;
            int64_t key = it->device_ns > 0 ? it->device_ns : INT64_MAX;
            if (key < best_key) { best_key = key; best = it; }
        }
        return best;
    }

    xi::Record drain_one_trigger_slot(const xi::Record& /*input*/) {
        std::unique_lock<std::mutex> lk(pending_mu_);

        // If nothing's pending, return a lightweight status so downstream
        // inspections can idle without error noise.
        if (pending_.empty()) {
            xi::Record out;
            out.set("error", "no_pending_trigger");
            out.set("connected", true);
            out.set("streaming", streaming_.load());
            out.set("strict",    true);
            return out;
        }

        // Wait until SOME slot is ready (filled or missed), or the nearest
        // deadline passes. We no longer wait specifically on the front —
        // events can arrive out of order, so we emit in physical
        // (device_ns) order once a candidate is ready.
        auto nearest_deadline = [this]() {
            auto dl = std::chrono::steady_clock::time_point::max();
            for (auto& s : pending_) {
                if (s.filled || s.missed) continue;
                if (s.deadline < dl) dl = s.deadline;
            }
            return dl;
        };
        pending_cv_.wait_until(lk,
            nearest_deadline(),
            [this]{
                if (pending_.empty()) return true;
                for (auto& s : pending_) if (s.filled || s.missed) return true;
                return false;
            });

        // Prefer the oldest (min device_ns) ready slot. If none is ready
        // yet but deadlines have passed on some slot, fall back to the
        // oldest unfilled and classify it as timeout below.
        auto pick = pick_oldest_ready();
        if (pick == pending_.end()) pick = pick_oldest_unfilled();
        if (pick == pending_.end()) {
            xi::Record out;
            out.set("error", "no_pending_trigger");
            return out;
        }
        TriggerSlot slot = std::move(*pick);
        pending_.erase(pick);
        if (!slot.filled && !slot.missed) {
            // Deadline passed without a frame arriving and firmware
            // didn't report an explicit miss — classify as timeout.
            slot.missed      = true;
            slot.miss_reason = "timeout";
            missed_count_.fetch_add(1);
        }
        // Reassign trigger_id at emit time so output order is monotonic in
        // physical (device_ns) order — the camera hardware's view — rather
        // than event-callback order (which MVS may reorder during bursts).
        slot.trigger_id = next_output_tid_.fetch_add(1);
        lk.unlock();

        xi::Record out;
        if (slot.filled && !slot.frame.empty()) {
            out.image("out", slot.frame);
            out.set("frame_id", (int)slot.trigger_id);
            out.set("width",    slot.frame.width);
            out.set("height",   slot.frame.height);
            out.set("channels", slot.frame.channels);
            out.set("missed",   false);
        } else {
            // Placeholder: empty image + explicit miss marker. Downstream
            // can check `missed` / `miss_reason` / `trigger_source` to
            // decide how to handle (log, re-trigger, substitute previous
            // frame, etc.) — but the frame sequence index stays aligned
            // with the trigger index.
            out.image("out", xi::Image{});
            out.set("frame_id", (int)slot.trigger_id);
            out.set("missed",   true);
            out.set("miss_reason", slot.miss_reason.empty() ? "unknown"
                                                            : slot.miss_reason);
            out.set("error",
                    slot.miss_reason == "firmware_reported"
                        ? (slot.source == "hardware"
                              ? "missed_hw_trigger_firmware_reported"
                              : "missed_sw_trigger_firmware_reported")
                    : /* timeout */
                        (slot.source == "hardware"
                              ? "missed_hw_trigger_timeout"
                              : "missed_sw_trigger_timeout"));
        }
        out.set("trigger_id",     (int)slot.trigger_id);
        out.set("trigger_source", slot.source);       // "software" or "hardware"
        // Timestamps (nanoseconds). For misses only the trigger time is
        // real; the frame timestamps stay zero so downstream can still
        // detect "no frame arrived" via missed=true while keeping a valid
        // trigger time for index correlation.
        out.set("ts_trigger_ns",  (double)slot.trigger_ns);
        out.set("ts_received_ns", (double)slot.received_ns);
        out.set("ts_host_ns",     (double)slot.host_ns);
        out.set("ts_device_ns",   (double)slot.device_ns);
        if (slot.filled && slot.received_ns && slot.trigger_ns) {
            // Both are steady_clock → this delta is meaningful.
            out.set("latency_ns", (double)(slot.received_ns - slot.trigger_ns));
        }
        out.set("connected",      true);
        out.set("streaming",      streaming_.load());
        out.set("strict",         true);
        out.set("captured",       (int)captured_.load());
        out.set("dropped",        (int)dropped_.load());
        out.set("missed_triggers",(int)missed_count_.load());
        return out;
    }

    std::string exchange(const std::string& cmd) override {
        auto p = xi::Json::parse(cmd);
        auto command = p["command"].as_string();

        if      (command == "discover")          discover();
        else if (command == "connect")           connect_by(p);
        else if (command == "disconnect")        disconnect();
        else if (command == "start_streaming")   start_streaming();
        else if (command == "stop_streaming")    stop_streaming();
        else if (command == "flush_camera") {
            // Drain any frames the camera has buffered in its on-board
            // DDR from a prior session (teardown leaves the cam in
            // free-run, so it accumulates frames until the next
            // StartGrabbing). Approach: if streaming is off, start it
            // in free-run briefly and let MVS deliver the backlog;
            // otherwise just wait in-place. Then zero the stats
            // counters so the caller's next capture starts from 0.
            int drain_ms = p["drain_ms"].as_int(500);
            bool was_trigger_on = (trigger_mode_ == "on");
            bool was_streaming  = streaming_.load();
            if (was_trigger_on) {
                // Free-run mode needed so buffered frames actually
                // flow out without triggers.
                apply_trigger_mode("off", trigger_source_);
            }
            if (!streaming_.load()) start_streaming();
            std::this_thread::sleep_for(std::chrono::milliseconds(drain_ms));
            // Restore trigger state BEFORE resetting counters — the
            // stop/start cycle during apply_trigger_mode("on") delivers
            // one more batch of stale buffered frames; if we zero
            // counters before that, those stale frames end up counted.
            if (was_trigger_on) {
                apply_trigger_mode("on", trigger_source_);
            }
            if (!was_streaming) stop_streaming();
            // Give MVS a beat to deliver the last buffered frames from
            // the stop/start cycle, then reset counters so the caller
            // sees a clean slate from here on.
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            captured_.store(0);
            dropped_.store(0);
            lost_packets_.store(0);
            missed_count_.store(0);
            triggers_sent_.store(0);
            triggers_sent_sw_.store(0);
            triggers_sent_hw_.store(0);
            triggers_rejected_.store(0);
            triggers_accepted_.store(0);
            line0_edges_.store(0);
            line0_falling_edges_.store(0);
            last_seq_valid_.store(false);
            next_trigger_id_.store(1);
            next_output_tid_.store(1);
            {
                std::lock_guard<std::mutex> lk(pending_mu_);
                pending_.clear();
            }
            pending_cv_.notify_all();
        }
        else if (command == "set_roi")           apply_roi(p["x"].as_int(0), p["y"].as_int(0),
                                                           p["w"].as_int(0), p["h"].as_int(0));
        else if (command == "set_frame_rate")    apply_frame_rate(p["value"].as_double(fps_));
        else if (command == "set_exposure")      apply_exposure(p["value"].as_double(exposure_us_));
        else if (command == "set_gain")          apply_gain(p["value"].as_double(gain_));
        else if (command == "set_mirror")        apply_mirror(p["mirror_x"].as_bool(mirror_x_),
                                                              p["mirror_y"].as_bool(mirror_y_));
        else if (command == "set_pixel_format")  apply_pixel_format(p["value"].as_string(pixel_format_));
        else if (command == "set_trigger_mode")  apply_trigger_mode(p["mode"].as_string("off"),
                                                                    p["source"].as_string("Software"));
        else if (command == "software_trigger")  software_trigger(p["count"].as_int(1));
        else if (command == "set_strict_trigger_mode") {
            strict_trigger_mode_ = p["value"].as_bool(false);
            strict_timeout_ms_   = p["timeout_ms"].as_int(0);
            {
                // On any toggle, clear pending + reset the nFrameNum
                // anchor so the first frame after the toggle doesn't
                // look like a giant gap.
                std::lock_guard<std::mutex> lk(pending_mu_);
                pending_.clear();
            }
            last_seq_valid_.store(false);
            pending_cv_.notify_all();
            if (strict_trigger_mode_ && handle_ && trigger_source_ != "Software") {
                register_hw_events();
            }
        }
        else if (command == "get_preview")       return get_preview_json(p["max_side"].as_int(480),
                                                                          p["quality"].as_int(70));
        else if (command == "diag")              return diag_json();
        else if (command == "debug_drop_every_n") {
            debug_drop_every_n_ = p["value"].as_int(0);
            return xi::Json::object()
                .set("ok", true)
                .set("drop_every_n", debug_drop_every_n_)
                .dump();
        }
        else if (command == "get_edge_counter") {
            int64_t c = read_edge_counter();
            return xi::Json::object()
                .set("available", counter_available_)
                .set("node", counter_value_node_)
                .set("value", (int)c)
                .dump();
        }
        else if (command == "probe_events") {
            // Enable + register a counter callback for each named event.
            // Request: {"command":"probe_events","names":["FrameStart","FrameEnd",...]}
            // Reply:   {"probes":[{"name":"...","selector_ok":true,
            //                       "notification_ok":true,"register_ok":true}, ...]}
            if (!handle_) return R"({"error":"not_connected"})";
            auto probes = xi::Json::array();
            p["names"].for_each([&](const char*, xi::Json v) {
                std::string name = v.as_string();
                if (name.empty()) return;
                auto entry = xi::Json::object().set("name", name);
                int r = MV_CC_SetEnumValueByString(handle_, "EventSelector", name.c_str());
                entry.set("selector_ok", r == MV_OK);
                if (r == MV_OK) {
                    int r2 = MV_CC_SetEnumValueByString(handle_, "EventNotification", "On");
                    entry.set("notification_ok", r2 == MV_OK);
                    int r3 = MV_CC_RegisterEventCallBackEx(
                        handle_, name.c_str(),
                        &HikRobotCamera::s_event_generic, this);
                    entry.set("register_ok", r3 == MV_OK);
                } else {
                    entry.set("notification_ok", false)
                         .set("register_ok",     false);
                }
                probes.push(entry);
            });
            {
                std::lock_guard<std::mutex> lk(event_counts_mu_);
                event_counts_.clear();
            }
            return xi::Json::object().set("probes", probes).dump();
        }
        else if (command == "get_event_counts") {
            auto out = xi::Json::object();
            std::lock_guard<std::mutex> lk(event_counts_mu_);
            for (auto& kv : event_counts_) {
                out.set(kv.first.c_str(), (int)kv.second);
            }
            return out.dump();
        }
        else if (command == "debug_counter_src") {
            // Debug: set CounterEventSource to a specific string so we
            // can probe which enum values this firmware honours.
            std::string src = p["value"].as_string("FrameStart");
            if (!handle_) return R"({"error":"not_connected"})";
            MV_CC_SetEnumValueByString(handle_, "CounterSelector", "Counter0");
            int r = MV_CC_SetEnumValueByString(handle_, "CounterEventSource", src.c_str());
            MV_CC_SetCommandValue(handle_, "CounterReset");
            counter_baseline_.store(0);
            return xi::Json::object()
                .set("ok", r == MV_OK)
                .set("src", src)
                .set("mvs_err", r)
                .dump();
        }
        else if (command == "flush_pending") {
            // Host-driven tail-drop cleanup. Two modes, usable together:
            //
            //   (1) Mark existing unfilled slots as missed. Covers the
            //       case where events / the gap-absorber put slots in
            //       pending_ but no frame ever arrived to fill them.
            //
            //   (2) Synthesize missed placeholders up to `expected`
            //       total outputs since the last flush/connect. Covers
            //       the true tail-drop case where the plugin never
            //       learned the triggers happened (no event, no gap).
            //       Host passes `expected` = trigger count it fired.
            //
            // Timestamps on these slots stay 0 — unknown by definition.
            int64_t expected = p["expected"].as_int(0);
            int flushed = 0;
            int synth   = 0;
            {
                std::lock_guard<std::mutex> lk(pending_mu_);
                for (auto& slot : pending_) {
                    if (!slot.filled && !slot.missed) {
                        slot.missed      = true;
                        slot.miss_reason = "flushed";
                        missed_count_.fetch_add(1);
                        ++flushed;
                    }
                }
                if (expected > 0) {
                    int64_t have = captured_.load() + missed_count_.load();
                    int64_t need = expected - have;
                    for (int64_t i = 0; i < need; ++i) {
                        TriggerSlot syn;
                        syn.trigger_id  = next_trigger_id_.fetch_add(1);
                        syn.source      = "hardware";
                        syn.missed      = true;
                        syn.miss_reason = "flushed";
                        syn.deadline    = std::chrono::steady_clock::now();
                        pending_.push_back(std::move(syn));
                        triggers_sent_.fetch_add(1);
                        triggers_sent_hw_.fetch_add(1);
                        missed_count_.fetch_add(1);
                        ++synth;
                    }
                }
                pending_cv_.notify_all();
            }
            return xi::Json::object()
                .set("ok", true)
                .set("flushed_existing", flushed)
                .set("synthesized", synth)
                .dump();
        }
        return get_def();
    }

    std::string get_def() const override {
        auto devices = xi::Json::array();
        {
            std::lock_guard<std::mutex> lk(devices_mu_);
            for (auto& d : device_labels_) devices.push(d);
        }
        auto keys = xi::Json::array();
        {
            std::lock_guard<std::mutex> lk(devices_mu_);
            for (auto& k : device_keys_) keys.push(k);
        }
        return xi::Json::object()
            .set("device_key",    device_key_)
            .set("device_label",  device_label_)
            .set("pixel_format",  pixel_format_)
            .set("roi_x",         roi_x_)
            .set("roi_y",         roi_y_)
            .set("roi_w",         roi_w_)
            .set("roi_h",         roi_h_)
            .set("fps",           fps_)
            .set("exposure_us",   exposure_us_)
            .set("gain",          gain_)
            .set("mirror_x",      mirror_x_)
            .set("mirror_y",      mirror_y_)
            .set("connected",     connected_.load())
            .set("streaming",     streaming_.load())
            .set("captured",      (int)captured_.load())
            .set("dropped",       (int)dropped_.load())
            .set("lost_packets",  (int)lost_packets_.load())
            .set("trigger_mode",  trigger_mode_)
            .set("trigger_source",trigger_source_)
            .set("strict_trigger_mode", strict_trigger_mode_)
            .set("strict_timeout_ms",   strict_timeout_ms_)
            .set("missed_triggers", (int)missed_count_.load())
            // MVS transport stats (refreshed ~1 Hz by the counter thread).
            // `mv_received_frames` includes both good and error frames
            // so equals approximately `captured + mv_error_frames` for
            // USB. Delta of `mv_error_frames` since last poll is frames
            // the transport noticed go wrong that our callback never
            // saw — a silent-drop signal beyond nFrameNum gaps.
            .set("mv_received_frames", (int)mv_received_frames_.load())
            .set("mv_error_frames",    (int)(mv_error_frames_.load() - mv_err_baseline_.load()))
            .set("mv_lost_frames",     (int)mv_lost_frames_.load())
            .set("mv_lost_packets",    (int)mv_lost_packets_.load())
            .set("mv_resend_packets",  (int)mv_resend_packets_.load())
            // Per-frame counts lifted from the latest MV_FRAME_OUT_INFO_EX.
            // Firmware-dependent — frequently 0 on USB3 cameras that
            // don't populate these chunks. When non-zero they give you
            // a second opinion on camera-side counts.
            .set("mv_frame_counter",   (int)mv_frame_counter_.load())
            .set("mv_trigger_index",   (int)triggers_accepted_.load())
            // Firmware-level edge counter (from Line0RisingEdge event).
            // Matches the physical edge count on the Line0 input; the
            // reaper uses this to catch tail drops autonomously.
            .set("line0_edges",         (int)line0_edges_.load())
            .set("line0_falling_edges", (int)line0_falling_edges_.load())
            .set("line0_last_dev_ns",  (double)line0_last_edge_device_ns_.load())
            .set("last_event_block_id",  (double)last_event_block_id_.load())
            .set("triggers_sent",     (int)triggers_sent_.load())
            .set("triggers_sent_sw",  (int)triggers_sent_sw_.load())
            .set("triggers_sent_hw",  (int)triggers_sent_hw_.load())
            .set("triggers_rejected", (int)triggers_rejected_.load())
            // "accepted" = frames that actually left the sensor (captured
            // + dropped-in-transit). Derived, so always accurate even
            // when nTriggerIndex / chunk-mode counters aren't populated
            // (which they aren't on the HikRobot USB3 firmwares we've
            // tested). The raw chunk value is also exposed as
            // `mv_trigger_index` for completeness.
            .set("triggers_accepted", (int)(captured_.load() + dropped_.load()))
            // "overruns" = triggers the camera ignored because the sensor
            // was already busy. The canonical HikRobot overrun signal.
            .set("overruns",
                 (int)std::max<int64_t>(0,
                    triggers_sent_.load() - (captured_.load() + dropped_.load())))
            .set("last_error",    last_error_)
            .set("devices",       devices)
            .set("device_keys",   keys)
            .dump();
    }

    bool set_def(const std::string& json) override {
        auto p = xi::Json::parse(json);
        if (!p.valid()) return false;
        device_key_   = p["device_key"].as_string(device_key_);
        pixel_format_ = p["pixel_format"].as_string(pixel_format_);
        roi_x_        = p["roi_x"].as_int(roi_x_);
        roi_y_        = p["roi_y"].as_int(roi_y_);
        roi_w_        = p["roi_w"].as_int(roi_w_);
        roi_h_        = p["roi_h"].as_int(roi_h_);
        fps_          = p["fps"].as_double(fps_);
        exposure_us_  = p["exposure_us"].as_double(exposure_us_);
        gain_         = p["gain"].as_double(gain_);
        mirror_x_     = p["mirror_x"].as_bool(mirror_x_);
        mirror_y_     = p["mirror_y"].as_bool(mirror_y_);
        strict_trigger_mode_ = p["strict_trigger_mode"].as_bool(strict_trigger_mode_);
        strict_timeout_ms_   = p["strict_timeout_ms"].as_int(strict_timeout_ms_);
        return true;
    }

private:
    // --- discovery ------------------------------------------------------
    // Process-global discovery cache. Multiple plugin instances share
    // one MV_CC_EnumDevices result so a second cam's discover() doesn't
    // re-enumerate while the first cam has an open handle — which on
    // this SDK silently breaks the first handle's control channel.
    //
    // Access through shared_enum() / shared_refresh(); callers are
    // themselves serialized by the shared mutex so two instances
    // can't race each other here.
    struct SharedEnum {
        std::mutex                      mu;
        std::vector<MV_CC_DEVICE_INFO*> list;
        std::vector<std::string>        labels;
        std::vector<std::string>        keys;
        bool                            populated = false;
    };
    static SharedEnum& shared_enum() {
        static SharedEnum s;
        return s;
    }
    // Refresh the shared list. Caller must NOT hold any per-instance
    // locks. Safe to skip if any plugin instance currently has an open
    // device — we detect by caller passing `allow_when_any_open=false`,
    // which is the default. Unconditional refresh on the first call.
    static void shared_refresh() {
        MV_CC_DEVICE_INFO_LIST list{};
        int r = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &list);
        if (r != MV_OK) return;
        auto& se = shared_enum();
        std::lock_guard<std::mutex> lk(se.mu);
        se.list.assign(list.pDeviceInfo, list.pDeviceInfo + list.nDeviceNum);
        se.labels.clear();
        se.keys.clear();
        for (unsigned i = 0; i < list.nDeviceNum; ++i) {
            se.labels.emplace_back(dev_label(list.pDeviceInfo[i]));
            se.keys  .emplace_back(dev_key  (list.pDeviceInfo[i]));
        }
        se.populated = true;
    }

    void discover() {
        auto& se = shared_enum();
        bool need = false;
        {
            std::lock_guard<std::mutex> lk(se.mu);
            need = !se.populated;
        }
        // First discover() call across the whole process actually
        // enumerates. Subsequent calls reuse the cached list unless the
        // caller explicitly wants a refresh (not exposed today — add a
        // "rediscover" command if needed).
        if (need) shared_refresh();

        // Mirror shared cache into per-instance fields so existing
        // get_status / get_def paths keep working unchanged.
        std::lock_guard<std::mutex> lk_src(se.mu);
        last_enum_ = se.list;
        std::lock_guard<std::mutex> lk(devices_mu_);
        device_labels_ = se.labels;
        device_keys_   = se.keys;
    }

    void connect_by(xi::Json& p) {
        if (connected_.load()) disconnect();

        // discover() must have populated last_enum_ for index/key to work.
        // Accept either {"index": n} or {"device_key": "USB:..."}.
        MV_CC_DEVICE_INFO* target = nullptr;
        int index = p["index"].as_int(-1);
        std::string key = p["device_key"].as_string("");
        if (key.empty()) key = device_key_;
        // Two-cam workaround: caller can pass "defer_streaming":true to
        // suppress the end-of-connect StartGrabbing. Lets the caller
        // open both handles without entering acquisition, then arm
        // triggers + start streaming later once USB bus is stable.
        bool defer_streaming = p["defer_streaming"].as_bool(false);

        if (last_enum_.empty()) discover();

        if (index >= 0 && index < (int)last_enum_.size()) {
            target = last_enum_[index];
        } else if (!key.empty()) {
            for (size_t i = 0; i < last_enum_.size(); ++i)
                if (dev_key(last_enum_[i]) == key) { target = last_enum_[i]; break; }
        }
        if (!target) {
            set_error("connect: no matching device (run discover first)");
            return;
        }

        void* h = nullptr;
        int r = MV_CC_CreateHandle(&h, target);
        if (r != MV_OK) { set_error(mv_err(r, "CreateHandle")); return; }

        r = MV_CC_OpenDevice(h);
        if (r != MV_OK) {
            set_error(mv_err(r, "OpenDevice"));
            MV_CC_DestroyHandle(h);
            return;
        }
        // HikRobot USB3 cameras need a short beat after OpenDevice before
        // feature sets will accept — otherwise the first few writes can
        // fail with MV_E_USB_WRITE (0x80000301). 500 ms is generous
        // enough that big sensors (20 MP MV-CE200) finish whatever
        // handshake they need after open.
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // Defensive: if a previous owner crashed mid-stream, the sensor
        // may still be grabbing. Feature writes fail while streaming, so
        // stop it first. Return value is ignored — StopGrabbing on a
        // non-streaming device is a no-op.
        MV_CC_StopGrabbing(h);

        // Tune GigE packet size for throughput; no-op for USB.
        if (target->nTLayerType == MV_GIGE_DEVICE) {
            int best = MV_CC_GetOptimalPacketSize(h);
            if (best > 0) MV_CC_SetIntValueEx(h, "GevSCPSPacketSize", best);
        }

        // Enlarge the SDK's internal ring so frames aren't dropped while
        // Skipping MV_CC_SetImageNodeNum — at high ROIs (5+ MP) the
        // large pinned-memory allocation (buffers × width × height)
        // happens in the middle of the control-channel init sequence
        // and blocks subsequent feature writes (TriggerMode, etc.),
        // producing the MV_E_USB_WRITE 0x80000301 storm we see on
        // concurrent two-cam startup. The reference HikRobot plugin
        // (visSele) doesn't set this at all and runs reliably — we
        // follow suit and accept MVS's default buffer depth.

        // Continuous / free-run.
        {
            int rr = MV_CC_SetEnumValueByString(h, "AcquisitionMode", "Continuous");
            if (rr != MV_OK) set_error(mv_err(rr, "AcquisitionMode=Continuous"));
        }
        {
            int rr = MV_CC_SetEnumValueByString(h, "TriggerMode", "Off");
            if (rr != MV_OK) set_error(mv_err(rr, "TriggerMode=Off(init)"));
        }

        // Apply stored config — errors are surfaced in last_error_ but
        // we don't bail; the user can fix them from the UI.
        handle_ = h;
        device_key_   = dev_key(target);
        device_label_ = dev_label(target);
        target_layer_ = target->nTLayerType;
        captured_.store(0);
        dropped_.store(0);
        lost_packets_.store(0);
        last_seq_valid_.store(false);
        triggers_sent_.store(0);
        triggers_sent_sw_.store(0);
        triggers_sent_hw_.store(0);
        triggers_rejected_.store(0);
        triggers_accepted_.store(0);
        missed_count_.store(0);

        apply_pixel_format_on_handle(pixel_format_);
        if (roi_w_ > 0 && roi_h_ > 0) apply_roi_on_handle(roi_x_, roi_y_, roi_w_, roi_h_);
        apply_exposure_on_handle(exposure_us_);
        apply_gain_on_handle(gain_);
        apply_frame_rate_on_handle(fps_);
        apply_mirror_on_handle(mirror_x_, mirror_y_);

        // If the user pre-configured trigger mode before connecting
        // (via set_trigger_mode while handle was null), honor it now —
        // otherwise StartGrabbing below runs the camera in free-run,
        // which for full-res streams on shared USB is exactly where
        // things go wrong. Applied here means the camera StartGrabs
        // into trigger-wait state on the very first go.
        if (trigger_mode_ == "on") {
            // TriggerSelector decides WHICH trigger type TriggerMode/Source
            // refer to. HikRobot USB3 cameras expose FrameBurstStart as
            // the primary option (matches their MVS GUI default); some
            // older / GigE models use FrameStart — try that as fallback.
            int ts = MV_CC_SetEnumValueByString(handle_, "TriggerSelector", "FrameBurstStart");
            if (ts != MV_OK) {
                ts = MV_CC_SetEnumValueByString(handle_, "TriggerSelector", "FrameStart");
            }
            if (ts != MV_OK) set_error(mv_err(ts, "TriggerSelector"));
            // Force 1 frame per trigger edge. With FrameBurstStart the
            // camera can be configured to emit N frames per edge via
            // AcquisitionBurstFrameCount; on some firmwares the default
            // is >1, which causes captured_ to grow faster than the
            // trigger count and breaks 1:1 edge→frame accounting.
            // No-op (or MV_E_SUPPORT) on firmwares that don't expose
            // this node — safe to ignore the return code.
            MV_CC_SetIntValue(handle_, "AcquisitionBurstFrameCount", 1);

            // Same USB_WRITE race as StartGrabbing — retry with backoff.
            int tr = MV_OK;
            for (int attempt = 0; attempt < 20; ++attempt) {
                tr = MV_CC_SetEnumValueByString(handle_, "TriggerMode", "On");
                if (tr == MV_OK) break;
                std::this_thread::sleep_for(std::chrono::milliseconds(300));
            }
            if (tr != MV_OK) set_error(mv_err(tr, "TriggerMode=On(init)"));
            tr = MV_CC_SetEnumValueByString(handle_, "TriggerSource",
                                            trigger_source_.c_str());
            if (tr != MV_OK) set_error(mv_err(tr,
                (std::string("TriggerSource=") + trigger_source_).c_str()));
            // Default activation; explicit for clarity / consistency with
            // the MVS GUI test the user ran.
            MV_CC_SetEnumValueByString(handle_, "TriggerActivation", "RisingEdge");
            if (trigger_source_ != "Software") register_hw_events();
        }

        // Read back ROI in case camera snapped to its increments.
        read_back_roi();

        r = MV_CC_RegisterImageCallBackEx2(handle_, &HikRobotCamera::s_frame_callback, this, true);
        if (r != MV_OK) {
            set_error(mv_err(r, "RegisterImageCallBackEx2"));
            MV_CC_CloseDevice(handle_);
            MV_CC_DestroyHandle(handle_);
            handle_ = nullptr;
            return;
        }

        connected_.store(true);
        last_error_.clear();
        last_activity_ns_.store(now_ns());
        // Snapshot MVS's cumulative counts at this moment so subsequent
        // polls read deltas within this session.
        {
            MV_MATCH_INFO_USB_DETECT u{};
            MV_ALL_MATCH_INFO info{};
            info.nType = MV_MATCH_TYPE_USB_DETECT;
            info.pInfo = &u;
            info.nInfoSize = sizeof(u);
            if (target_layer_ == MV_USB_DEVICE &&
                MV_CC_GetAllMatchInfo(handle_, &info) == MV_OK) {
                mv_err_baseline_.store((int64_t)u.nErrorFrameCount);
            } else {
                mv_err_baseline_.store(0);
            }
        }
        mv_received_frames_.store(0);
        mv_error_frames_.store(0);
        mv_lost_frames_.store(0);
        mv_lost_packets_.store(0);
        mv_resend_packets_.store(0);
        line0_edges_.store(0);
        line0_falling_edges_.store(0);
        line0_last_edge_device_ns_.store(0);

        setup_edge_counter();
        start_counter_thread();
        if (!defer_streaming) start_streaming();
        // Prime the transport-stats cache so a get_status right after
        // connect already shows valid counters (the 1 Hz poller in the
        // counter thread may not have run yet).
        poll_transport_stats();
    }

    void disconnect() {
        stop_counter_thread();
        stop_streaming();
        if (handle_) {
            unregister_hw_events();
            MV_CC_CloseDevice(handle_);
            MV_CC_DestroyHandle(handle_);
            handle_ = nullptr;
        }
        connected_.store(false);
        {
            std::lock_guard<std::mutex> lk(latest_mu_);
            latest_ = {};
            latest_id_        = 0;
            latest_host_ns_   = 0;
            latest_device_ns_ = 0;
        }
        {
            std::lock_guard<std::mutex> lk(pending_mu_);
            pending_.clear();
        }
        pending_cv_.notify_all();
        missed_count_.store(0);
    }

    void start_streaming() {
        if (!handle_ || streaming_.load()) return;
        // Retry on MV_E_USB_WRITE — concurrent two-camera startups
        // sometimes race at the USB control channel and the first
        // StartGrabbing fails. At full resolution on a shared bus the
        // initial handshake can take 1–2 s to settle, so we back off
        // aggressively rather than give up fast.
        int r = MV_OK;
        for (int attempt = 0; attempt < 20; ++attempt) {
            r = MV_CC_StartGrabbing(handle_);
            if (r == MV_OK) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
        }
        if (r != MV_OK) { set_error(mv_err(r, "StartGrabbing")); return; }
        streaming_.store(true);
    }

    void stop_streaming() {
        if (!handle_ || !streaming_.load()) return;
        MV_CC_StopGrabbing(handle_);
        streaming_.store(false);
    }

    // --- parameter plumbing -------------------------------------------
    void apply_roi(int x, int y, int w, int h) {
        roi_x_ = x; roi_y_ = y; roi_w_ = w; roi_h_ = h;
        if (!handle_) return;
        bool was = streaming_.load();
        stop_streaming();
        apply_roi_on_handle(x, y, w, h);
        read_back_roi();
        if (was) start_streaming();
    }

    void apply_roi_on_handle(int x, int y, int w, int h) {
        // MVS requires OffsetX/Y to be writable, which means reducing
        // Width/Height to fit first. Order matters.
        int r;
        r = MV_CC_SetIntValueEx(handle_, "OffsetX", 0); if (r != MV_OK) set_error(mv_err(r, "OffsetX=0"));
        r = MV_CC_SetIntValueEx(handle_, "OffsetY", 0); if (r != MV_OK) set_error(mv_err(r, "OffsetY=0"));
        if (w > 0) { r = MV_CC_SetIntValueEx(handle_, "Width",  w); if (r != MV_OK) set_error(mv_err(r, "Width")); }
        if (h > 0) { r = MV_CC_SetIntValueEx(handle_, "Height", h); if (r != MV_OK) set_error(mv_err(r, "Height")); }
        if (x > 0) { r = MV_CC_SetIntValueEx(handle_, "OffsetX", x); if (r != MV_OK) set_error(mv_err(r, "OffsetX")); }
        if (y > 0) { r = MV_CC_SetIntValueEx(handle_, "OffsetY", y); if (r != MV_OK) set_error(mv_err(r, "OffsetY")); }
    }

    void read_back_roi() {
        MVCC_INTVALUE v{};
        if (MV_CC_GetIntValue(handle_, "Width",   &v) == MV_OK) roi_w_ = (int)v.nCurValue;
        if (MV_CC_GetIntValue(handle_, "Height",  &v) == MV_OK) roi_h_ = (int)v.nCurValue;
        if (MV_CC_GetIntValue(handle_, "OffsetX", &v) == MV_OK) roi_x_ = (int)v.nCurValue;
        if (MV_CC_GetIntValue(handle_, "OffsetY", &v) == MV_OK) roi_y_ = (int)v.nCurValue;
    }

    void apply_frame_rate(double fps) {
        fps_ = fps;
        if (handle_) apply_frame_rate_on_handle(fps);
    }

    void apply_frame_rate_on_handle(double fps) {
        if (fps <= 0) {
            MV_CC_SetBoolValue(handle_, "AcquisitionFrameRateEnable", false);
            return;
        }
        MV_CC_SetBoolValue(handle_, "AcquisitionFrameRateEnable", true);
        int r = MV_CC_SetFloatValue(handle_, "AcquisitionFrameRate", (float)fps);
        if (r != MV_OK) set_error(mv_err(r, "AcquisitionFrameRate"));
    }

    void apply_exposure(double us) {
        exposure_us_ = us;
        if (handle_) apply_exposure_on_handle(us);
    }

    void apply_exposure_on_handle(double us) {
        if (us <= 0) return;
        MV_CC_SetEnumValueByString(handle_, "ExposureAuto", "Off");
        int r = MV_CC_SetFloatValue(handle_, "ExposureTime", (float)us);
        if (r != MV_OK) set_error(mv_err(r, "ExposureTime"));
    }

    void apply_gain(double db) {
        gain_ = db;
        if (handle_) apply_gain_on_handle(db);
    }

    void apply_gain_on_handle(double db) {
        MV_CC_SetEnumValueByString(handle_, "GainAuto", "Off");
        int r = MV_CC_SetFloatValue(handle_, "Gain", (float)db);
        if (r != MV_OK) set_error(mv_err(r, "Gain"));
    }

    void apply_mirror(bool x, bool y) {
        mirror_x_ = x; mirror_y_ = y;
        if (!handle_) return;
        bool was = streaming_.load();
        stop_streaming();
        apply_mirror_on_handle(x, y);
        if (was) start_streaming();
    }

    void apply_mirror_on_handle(bool x, bool y) {
        MV_CC_SetBoolValue(handle_, "ReverseX", x);
        MV_CC_SetBoolValue(handle_, "ReverseY", y);
    }

    // --- trigger mode ------------------------------------------------
    //
    // mode   ∈ {"off", "on"} — TriggerMode
    // source ∈ {"Software", "Line0", "Line1", "Line2", ...} — TriggerSource.
    // In "off" (free-run), source is ignored but still stored.
    void apply_trigger_mode(const std::string& mode, const std::string& source) {
        trigger_mode_   = (mode == "on") ? "on" : "off";
        trigger_source_ = source;
        if (!handle_) return;
        bool was = streaming_.load();
        stop_streaming();

        if (trigger_mode_ == "on") {
            // Select FrameStart selector first (see connect_by for why).
            MV_CC_SetEnumValueByString(handle_, "TriggerSelector", "FrameBurstStart");
        }
        int r1 = MV_CC_SetEnumValueByString(handle_, "TriggerMode",
                                            trigger_mode_ == "on" ? "On" : "Off");
        if (r1 != MV_OK) set_error(mv_err(r1, "TriggerMode"));

        if (trigger_mode_ == "on") {
            int r2 = MV_CC_SetEnumValueByString(handle_, "TriggerSource", source.c_str());
            if (r2 != MV_OK) set_error(mv_err(r2, "TriggerSource"));
            MV_CC_SetEnumValueByString(handle_, "TriggerActivation", "RisingEdge");
        }

        // Strict mode with a hardware source means we rely on HikRobot's
        // FrameTrigger / FrameTriggerMiss events to know when a trigger
        // edge was accepted / rejected. Re-register whenever mode flips.
        unregister_hw_events();
        if (strict_trigger_mode_ && trigger_mode_ == "on" && trigger_source_ != "Software") {
            register_hw_events();
        }

        if (was) start_streaming();
    }

    // --- hardware trigger events --------------------------------------
    //
    // HikRobot cameras expose the trigger detector via two named events:
    //   "FrameTrigger"      — every edge the camera accepted as a trigger
    //   "FrameTriggerMiss"  — every edge that arrived while the sensor
    //                         was still busy (the overrun signal)
    // Enable each via EventSelector + EventNotification, then register
    // their callbacks. Not all firmwares expose FrameTriggerMiss — if
    // it fails, the code silently falls back to deadline-based misses.
    void register_hw_events() {
        if (!handle_) return;
        // FrameTrigger/Miss aren't supported on this HikRobot firmware
        // (confirmed via event probe). Line0RisingEdge IS, and fires for
        // every electrical edge — including ones the sensor would later
        // reject because it was busy. Register both so we have coverage
        // regardless of which firmware variant is in use.
        // Line0FallingEdge is counted for pulse-width sanity but doesn't
        // push slots (one slot per pair of edges would double-count).
        enable_event("FrameTrigger");
        enable_event("FrameTriggerMiss");
        enable_event("Line0RisingEdge");
        enable_event("Line0FallingEdge");
        MV_CC_RegisterEventCallBackEx(handle_, "FrameTrigger",
                                      &HikRobotCamera::s_event_frame_trigger, this);
        MV_CC_RegisterEventCallBackEx(handle_, "FrameTriggerMiss",
                                      &HikRobotCamera::s_event_frame_trigger_miss, this);
        MV_CC_RegisterEventCallBackEx(handle_, "Line0RisingEdge",
                                      &HikRobotCamera::s_event_line_rising_edge, this);
        MV_CC_RegisterEventCallBackEx(handle_, "Line0FallingEdge",
                                      &HikRobotCamera::s_event_line_falling_edge, this);
    }
    void unregister_hw_events() {
        if (!handle_) return;
        MV_CC_RegisterEventCallBackEx(handle_, "FrameTrigger",      nullptr, nullptr);
        MV_CC_RegisterEventCallBackEx(handle_, "FrameTriggerMiss",  nullptr, nullptr);
        MV_CC_RegisterEventCallBackEx(handle_, "Line0RisingEdge",   nullptr, nullptr);
        MV_CC_RegisterEventCallBackEx(handle_, "Line0FallingEdge",  nullptr, nullptr);
    }
    void enable_event(const char* name) {
        int r = MV_CC_SetEnumValueByString(handle_, "EventSelector", name);
        if (r != MV_OK) return; // firmware may not list this event
        MV_CC_SetEnumValueByString(handle_, "EventNotification", "On");
    }

    static void __stdcall s_event_frame_trigger(MV_EVENT_OUT_INFO* info, void* user) {
        auto* self = static_cast<HikRobotCamera*>(user);
        if (!self || !info) return;
        self->bump_event_count(info->EventName);
        self->on_hw_trigger_edge(/*missed=*/false);
    }
    static void __stdcall s_event_frame_trigger_miss(MV_EVENT_OUT_INFO* info, void* user) {
        auto* self = static_cast<HikRobotCamera*>(user);
        if (!self || !info) return;
        self->bump_event_count(info->EventName);
        self->on_hw_trigger_edge(/*missed=*/true);
    }
    // Generic catch-all for the event probe. Counts the fire per
    // EventName AND dispatches to the specific plugin-state updates
    // for events we care about — so `probe_events` can replace all
    // callbacks with this one and the plugin still behaves correctly
    // (slot-pushing for edges, strict-mode hooks for trigger events).
    static void __stdcall s_event_generic(MV_EVENT_OUT_INFO* info, void* user) {
        auto* self = static_cast<HikRobotCamera*>(user);
        if (!self || !info) return;
        self->bump_event_count(info->EventName);
        std::string name = info->EventName;
        if (name == "Line0RisingEdge") {
            self->line0_edges_.fetch_add(1);
            int64_t dev_ns = ((int64_t)info->nTimestampHigh << 32)
                           | (int64_t)info->nTimestampLow;
            self->line0_last_edge_device_ns_.store(dev_ns);
            self->push_edge_slot(dev_ns);
        } else if (name == "Line0FallingEdge") {
            self->line0_falling_edges_.fetch_add(1);
        } else if (name == "FrameTrigger") {
            self->on_hw_trigger_edge(false);
        } else if (name == "FrameTriggerMiss") {
            self->on_hw_trigger_edge(true);
        }
    }
    // Line0RisingEdge fires per electrical edge, firmware-reliable.
    // Each edge pushes a pending slot with device_ns captured from the
    // event's camera-clock timestamp — so even slots that later stay
    // missed (sensor-rejected or tail-dropped) carry an accurate
    // per-edge `device_ns`.
    static void __stdcall s_event_line_rising_edge(MV_EVENT_OUT_INFO* info, void* user) {
        auto* self = static_cast<HikRobotCamera*>(user);
        if (!self || !info) return;
        self->bump_event_count(info->EventName);
        self->line0_edges_.fetch_add(1);
        int64_t dev_ns = ((int64_t)info->nTimestampHigh << 32)
                       | (int64_t)info->nTimestampLow;
        self->line0_last_edge_device_ns_.store(dev_ns);
        // MVS headers mark nBlockIdHigh/Low as "no firmware support" and
        // the struct has no FrameNum field — capture BlockId anyway so a
        // probe can confirm it stays zero on this firmware.
        self->last_event_block_id_.store(
            ((int64_t)info->nBlockIdHigh << 32) | (int64_t)info->nBlockIdLow);
        self->push_edge_slot(dev_ns);
    }
    // Line0FallingEdge — counted for pulse-width sanity / double-check
    // signal integrity. Doesn't push slots.
    static void __stdcall s_event_line_falling_edge(MV_EVENT_OUT_INFO* info, void* user) {
        auto* self = static_cast<HikRobotCamera*>(user);
        if (!self || !info) return;
        self->bump_event_count(info->EventName);
        self->line0_falling_edges_.fetch_add(1);
    }

    // Push a pristine slot tagged with the edge's camera-clock time.
    // Called from Line0RisingEdge handler. If strict mode is off we
    // still keep the slot for the reaper's bookkeeping — it just won't
    // be drained by process() in non-strict mode.
    void push_edge_slot(int64_t device_ns) {
        if (trigger_mode_ != "on") return;  // only meaningful in triggered
        if (trigger_source_ == "Software") return;
        std::lock_guard<std::mutex> lk(pending_mu_);
        TriggerSlot slot;
        slot.trigger_id = next_trigger_id_.fetch_add(1);
        slot.source     = "hardware";
        slot.trigger_ns = now_ns();
        slot.device_ns  = device_ns;          // ← event's camera-clock time
        slot.deadline   = std::chrono::steady_clock::now()
                        + std::chrono::milliseconds(
                            strict_timeout_ms_ > 0 ? strict_timeout_ms_
                                                    : ((int)(exposure_us_ / 1000) + 200));
        pending_.push_back(std::move(slot));
        triggers_sent_.fetch_add(1);
        triggers_sent_hw_.fetch_add(1);
        pending_cv_.notify_all();
    }

    void bump_event_count(const char* name) {
        if (!name) return;
        std::lock_guard<std::mutex> lk(event_counts_mu_);
        event_counts_[name]++;
    }

    void on_hw_trigger_edge(bool missed) {
        // Each hardware edge is counted as a "sent" trigger.
        triggers_sent_.fetch_add(1);
        triggers_sent_hw_.fetch_add(1);

        if (!strict_trigger_mode_) {
            if (missed) missed_count_.fetch_add(1);
            return;
        }

        auto now = std::chrono::steady_clock::now();
        int timeout_ms = strict_timeout_ms_ > 0
            ? strict_timeout_ms_
            : (int)(exposure_us_ / 1000.0) + 200;
        auto deadline = now + std::chrono::milliseconds(timeout_ms);

        TriggerSlot slot;
        slot.trigger_id      = next_trigger_id_.fetch_add(1);
        slot.source          = "hardware";
        slot.trigger_ns = now_ns();
        slot.deadline        = missed ? now : deadline;  // missed slots expire immediately
        if (missed) {
            slot.filled      = false;
            slot.missed      = true;
            slot.miss_reason = "firmware_reported";
            missed_count_.fetch_add(1);
        }
        {
            std::lock_guard<std::mutex> lk(pending_mu_);
            pending_.push_back(std::move(slot));
        }
        pending_cv_.notify_all();
    }

    void software_trigger(int count) {
        if (!handle_) return;
        if (count <= 0) count = 1;
        auto now = std::chrono::steady_clock::now();
        // deadline = exposure + readout slack. 200 ms slack covers USB
        // queueing on slow links; bump via strict_timeout_ms if needed.
        int timeout_ms = strict_timeout_ms_ > 0
            ? strict_timeout_ms_
            : (int)(exposure_us_ / 1000.0) + 200;
        auto deadline = now + std::chrono::milliseconds(timeout_ms);

        for (int i = 0; i < count; ++i) {
            if (strict_trigger_mode_ && trigger_mode_ == "on") {
                TriggerSlot slot;
                slot.trigger_id      = next_trigger_id_.fetch_add(1);
                slot.deadline        = deadline;
                slot.source          = "software";
                slot.trigger_ns = now_ns();
                std::lock_guard<std::mutex> lk(pending_mu_);
                pending_.push_back(std::move(slot));
            }

            int r = MV_CC_SetCommandValue(handle_, "TriggerSoftware");
            triggers_sent_.fetch_add(1);
            triggers_sent_sw_.fetch_add(1);
            if (r != MV_OK) triggers_rejected_.fetch_add(1);
        }
    }

    void apply_pixel_format(const std::string& fmt) {
        pixel_format_ = fmt;
        if (!handle_) return;
        bool was = streaming_.load();
        stop_streaming();
        apply_pixel_format_on_handle(fmt);
        if (was) start_streaming();
    }

    void apply_pixel_format_on_handle(const std::string& fmt) {
        // "Mono8", "RGB8Packed", "BGR8Packed" are the canonical GenICam
        // names. We also accept short forms from the UI.
        std::string genicam = fmt;
        if (fmt == "RGB8") genicam = "RGB8Packed";
        if (fmt == "BGR8") genicam = "BGR8Packed";
        int r = MV_CC_SetEnumValueByString(handle_, "PixelFormat", genicam.c_str());
        if (r != MV_OK) set_error(mv_err(r, (std::string("PixelFormat=") + genicam).c_str()));
    }

    // --- frame callback ------------------------------------------------
    static void __stdcall s_frame_callback(MV_FRAME_OUT* pstFrame, void* pUser, bool /*bAutoFree*/) {
        auto* self = static_cast<HikRobotCamera*>(pUser);
        if (!self || !pstFrame || !pstFrame->pBufAddr) return;
        self->consume_frame(pstFrame);
    }

    void consume_frame(MV_FRAME_OUT* pstFrame) {
        const auto& info = pstFrame->stFrameInfo;

        // Debug hook: discard every Nth frame to simulate USB/MVS drops
        // for testing the strict-mode transmission-drop handling. Returns
        // before updating any counters — the next frame's nFrameNum jump
        // is the only signal the drop happened, which is exactly the
        // real-world failure mode.
        if (debug_drop_every_n_ > 0) {
            debug_drop_counter_++;
            if (debug_drop_counter_ % debug_drop_every_n_ == 0) return;
        }
        int w = info.nExtendWidth  ? (int)info.nExtendWidth  : (int)info.nWidth;
        int h = info.nExtendHeight ? (int)info.nExtendHeight : (int)info.nHeight;
        if (w <= 0 || h <= 0) { dropped_.fetch_add(1); return; }

        // Drop detection via camera-side sequence numbers. nFrameNum
        // increments for every *exposed* frame regardless of whether it
        // made it through USB/host to us — so a gap means frames were
        // dropped somewhere between the sensor and this callback (on-
        // camera buffer, USB, or SDK node pool). nLostPacket > 0 means
        // this frame itself is partially corrupt.
        unsigned seq      = info.nFrameNum;
        unsigned expected = last_seq_.load();
        int      gap      = 0;
        if (last_seq_valid_.load() && seq > expected) {
            gap = (int)(seq - expected);
            dropped_.fetch_add(gap);
        }
        last_seq_.store(seq + 1);
        last_seq_valid_.store(true);

        if (info.nLostPacket) lost_packets_.fetch_add(info.nLostPacket);
        // nTriggerIndex is the camera's count of *accepted* triggers at
        // the moment this frame was exposed (often zero on USB3
        // firmwares; works when populated). nFrameCounter is a separate
        // cumulative counter that sometimes survives across trigger
        // toggles. Keep the highest observed value.
        if (info.nTriggerIndex > triggers_accepted_.load()) {
            triggers_accepted_.store(info.nTriggerIndex);
        }
        if ((int64_t)info.nFrameCounter > mv_frame_counter_.load()) {
            mv_frame_counter_.store((int64_t)info.nFrameCounter);
        }

        MvGvspPixelType src_fmt = info.enPixelType;
        int channels = 0;
        const uint8_t* src = pstFrame->pBufAddr;
        std::vector<uint8_t> converted;

        switch (src_fmt) {
            case PixelType_Gvsp_Mono8:
                channels = 1;
                break;
            case PixelType_Gvsp_RGB8_Packed:
            case PixelType_Gvsp_BGR8_Packed:
                channels = 3;
                break;
            default: {
                // Convert anything else (Bayer, 10/12-bit, YUV) to BGR8.
                channels = 3;
                converted.resize((size_t)w * h * 3);
                MV_CC_PIXEL_CONVERT_PARAM cp{};
                cp.nWidth         = (unsigned short)w;
                cp.nHeight        = (unsigned short)h;
                cp.pSrcData       = pstFrame->pBufAddr;
                cp.nSrcDataLen    = pstFrame->stFrameInfo.nFrameLen;
                cp.enSrcPixelType = src_fmt;
                cp.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
                cp.pDstBuffer     = converted.data();
                cp.nDstBufferSize = (unsigned)converted.size();
                int r = MV_CC_ConvertPixelType(handle_, &cp);
                if (r != MV_OK) {
                    dropped_.fetch_add(1);
                    return;
                }
                src = converted.data();
                break;
            }
        }

        xi::Image img(w, h, channels, src);
        int64_t frame_host_ns   = (int64_t)info.nHostTimeStamp;
        int64_t frame_device_ns =
            ((int64_t)info.nDevTimeStampHigh << 32) | (int64_t)info.nDevTimeStampLow;
        int64_t id;
        {
            std::lock_guard<std::mutex> lk(latest_mu_);
            latest_           = img;
            id                = captured_.fetch_add(1) + 1;
            latest_id_        = id;
            latest_host_ns_   = frame_host_ns;
            latest_device_ns_ = frame_device_ns;
        }
        last_activity_ns_.store(now_ns());

        // Strict-mode pairing. Goal: the output sequence stays 1:1 with
        // every camera-exposed frame. Two populator paths feed pending_:
        //
        //   (a) FrameTrigger events — pushed by on_hw_trigger_edge() as
        //       edges are detected by the camera. Present when the
        //       firmware supports FrameTrigger events (not with
        //       TriggerSource=Anyway on USB3 — see README caveat #7).
        //   (b) On-arrival synthesis — this code path. When (a) didn't
        //       push anything, we synthesize slots here so every frame
        //       (and every gap-detected drop) becomes a placeholder or
        //       real output.
        //
        // The algorithm ensures pending_ has exactly `gap + 1` unfilled
        // slots after this call: `gap` of them missed=transmission_drop,
        // and the last one filled with the frame we just received.
        if (strict_trigger_mode_ && trigger_mode_ == "on") {
            std::lock_guard<std::mutex> lk(pending_mu_);

            auto count_unfilled = [&]() {
                int n = 0;
                for (auto& s : pending_) if (!s.filled && !s.missed) ++n;
                return n;
            };

            // Stale-slot sweep. When an edge event pushed a slot but the
            // sensor later rejected the trigger (e.g. the second pulse
            // of a close surge pair), no frame will ever arrive for it.
            // Without this sweep, FIFO fill would back-fill such a slot
            // with a later frame and the miss attribution drifts to the
            // tail. Threshold: if an edge is older than (2×exposure +
            // 200 ms) without a frame, we can safely call it rejected.
            // The window is generous enough to never false-positive a
            // slow-arriving frame under normal backpressure.
            {
                const int64_t now    = now_ns();
                const int64_t window = (int64_t)exposure_us_ * 1000LL * 2LL
                                     + 200LL * 1000000LL;
                for (auto& s : pending_) {
                    if (s.filled || s.missed) continue;
                    if (s.trigger_ns == 0) continue;  // pushed without timestamp
                    if (now - s.trigger_ns > window) {
                        s.missed      = true;
                        s.miss_reason = "firmware_reported";
                        missed_count_.fetch_add(1);
                    }
                }
            }

            // Synthesize any slots we still need — `gap` of them will
            // be marked missed below, plus one for this frame.
            //
            // Timestamps: we only know the true trigger time of edges
            // that either fired a FrameTrigger event (pre-pushed slot,
            // already stamped) OR produced a delivered frame (the real
            // slot, stamped with current time). For drops we don't
            // know — the edge happened some unknown amount before
            // `now`. Rather than synthesize a fake time, we leave
            // `trigger_ns = 0` on the missed slots so downstream can
            // tell the timestamp is unknown.
            int total_need = gap + 1;
            int have       = count_unfilled();
            int to_synth   = total_need - have;
            if (to_synth < 0) to_synth = 0;

            const int64_t t_now = now_ns();

            for (int i = 0; i < to_synth; ++i) {
                TriggerSlot syn;
                syn.trigger_id  = next_trigger_id_.fetch_add(1);
                syn.source      = "hardware";
                syn.deadline    = std::chrono::steady_clock::now();
                // Leave trigger_ns = 0 on synthesized slots by default;
                // the final fill step overwrites it on the real one.
                pending_.push_back(std::move(syn));
                triggers_sent_.fetch_add(1);
                triggers_sent_hw_.fetch_add(1);
                ++have;
            }

            // Mark the oldest (by device_ns) `gap` unfilled slots as
            // drops. Sorting by device_ns — not push order — ensures
            // transmission-drop attribution tracks physical edge order
            // even when MVS reorders event callbacks during bursts.
            for (int k = 0; k < gap; ++k) {
                auto it = pick_oldest_unfilled();
                if (it == pending_.end()) break;
                it->missed      = true;
                it->miss_reason = "transmission_drop";
                missed_count_.fetch_add(1);
            }

            // Fill the oldest (by device_ns) pristine slot with this
            // frame — the one that physically corresponds to the next
            // edge the sensor read out.
            {
                auto it = pick_oldest_unfilled();
                if (it != pending_.end()) {
                    it->frame       = std::move(img);
                    it->filled      = true;
                    it->received_ns = t_now;
                    it->host_ns     = frame_host_ns;
                    // Preserve the slot's original device_ns (captured
                    // from the Line0RisingEdge event) — true edge time.
                    if (it->device_ns == 0) it->device_ns = frame_device_ns;
                    if (it->trigger_ns == 0) it->trigger_ns = t_now;
                }
            }
            pending_cv_.notify_all();
        }
    }

    // --- preview encoder ----------------------------------------------
    //
    // Returns a JSON reply with a base64-encoded JPEG of the latest
    // frame, downsampled to `max_side` on the longer axis. The UI polls
    // this at its own cadence (typically 2–4 Hz) — get_status is kept
    // image-free so it stays cheap.
    std::string get_preview_json(int max_side, int quality) {
        if (max_side <= 0)  max_side = 480;
        if (quality < 10)   quality = 10;
        if (quality > 95)   quality = 95;

        xi::Image frame;
        int64_t   frame_id = 0;
        {
            std::lock_guard<std::mutex> lk(latest_mu_);
            frame    = latest_;
            frame_id = latest_id_;
        }
        if (frame.empty()) {
            return xi::Json::object()
                .set("has_frame", false)
                .set("frame_id", 0)
                .dump();
        }

        xi::Image thumb = downsample_to(frame, max_side);
        std::vector<uint8_t> jpeg;
        if (!xi::encode_jpeg(thumb, quality, jpeg)) {
            return xi::Json::object()
                .set("has_frame", false)
                .set("error", "encode_jpeg failed")
                .dump();
        }

        return xi::Json::object()
            .set("has_frame", true)
            .set("frame_id", (int)frame_id)
            .set("width",    thumb.width)
            .set("height",   thumb.height)
            .set("channels", thumb.channels)
            .set("jpeg_b64", base64_encode(jpeg.data(), jpeg.size()))
            .dump();
    }

    // --- live diagnostics ---------------------------------------------
    //
    // Reads back what the camera actually has configured so we can see
    // whether our set_* commands took effect (or whether some feature
    // is capping the framerate).
    std::string diag_json() {
        auto obj = xi::Json::object();
        obj.set("has_handle", handle_ != nullptr);
        if (!handle_) return obj.dump();

        auto read_float = [&](const char* key) {
            MVCC_FLOATVALUE v{};
            int r = MV_CC_GetFloatValue(handle_, key, &v);
            auto row = xi::Json::object();
            row.set("ok", r == MV_OK);
            if (r == MV_OK) {
                row.set("cur", (double)v.fCurValue);
                row.set("min", (double)v.fMin);
                row.set("max", (double)v.fMax);
            } else {
                row.set("err", (int)r);
            }
            obj.set(key, row);
        };
        auto read_int = [&](const char* key) {
            MVCC_INTVALUE v{};
            int r = MV_CC_GetIntValue(handle_, key, &v);
            auto row = xi::Json::object();
            row.set("ok", r == MV_OK);
            if (r == MV_OK) {
                row.set("cur", (int)v.nCurValue);
                row.set("min", (int)v.nMin);
                row.set("max", (int)v.nMax);
            } else {
                row.set("err", (int)r);
            }
            obj.set(key, row);
        };
        auto read_bool = [&](const char* key) {
            bool b = false;
            int r = MV_CC_GetBoolValue(handle_, key, &b);
            auto row = xi::Json::object();
            row.set("ok", r == MV_OK);
            if (r == MV_OK) row.set("cur", b);
            else            row.set("err", (int)r);
            obj.set(key, row);
        };
        auto read_enum = [&](const char* key) {
            MVCC_ENUMVALUE v{};
            int r = MV_CC_GetEnumValue(handle_, key, &v);
            auto row = xi::Json::object();
            row.set("ok", r == MV_OK);
            if (r == MV_OK) row.set("cur", (int)v.nCurValue);
            else            row.set("err", (int)r);
            obj.set(key, row);
        };
        read_float("ExposureTime");
        read_float("Gain");
        read_float("AcquisitionFrameRate");
        read_float("ResultingFrameRate");   // readback — actual achieved fps
        read_bool ("AcquisitionFrameRateEnable");
        read_enum ("TriggerMode");
        read_enum ("AcquisitionMode");
        read_enum ("ExposureAuto");
        read_enum ("GainAuto");
        read_int  ("Width");
        read_int  ("Height");
        read_enum ("PixelFormat");
        read_enum ("DeviceLinkSpeed");                  // 0=USB2 LS/FS, 1=USB2 HS, 2=USB3 SS
        read_int  ("DeviceLinkSpeedInBps");             // actual link speed bytes/s
        read_int  ("DeviceLinkCurrentThroughput");      // bytes/s currently used
        read_int  ("StreamPacketCount");
        return obj.dump();
    }

    // --- camera-side edge counter (Counter0) -------------------------
    //
    // GenICam Counter feature incremented at the camera hardware level
    // on every edge of Line0. Gives us ground truth for how many
    // triggers the camera actually received — independent of whether
    // the sensor exposed a frame, whether the frame transmitted OK, or
    // whether the plugin's frame callback fired. Enables tail-drop
    // detection: `Counter0Value - (captured + missed_count)` = number
    // of triggers still unaccounted for.
    void setup_edge_counter() {
        counter_available_ = false;
        counter_baseline_.store(0);
        if (!handle_ || trigger_mode_ != "on") return;
        if (trigger_source_ == "Software") return;

        int r;
        r = MV_CC_SetEnumValueByString(handle_, "CounterSelector", "Counter0");
        if (r != MV_OK) { set_error(mv_err(r, "CounterSelector=Counter0")); return; }

        // CounterEventSource: try edge-specific names first (HikRobot
        // uses `Line0RisingEdge` as the event that fires per edge),
        // then the level-style name as fallback. Also generic event
        // names for firmwares that only expose higher-level signals.
        std::string edge_name = trigger_source_ + "RisingEdge";
        const char* candidates[] = {
            // FrameStart ticks once per exposure; that's the closest we
            // can get to "triggers the camera saw" on firmwares that
            // don't accept Line0 as a counter source (most HikRobot
            // USB3 cams). Missed-at-sensor triggers won't count, but
            // transmission drops still will — sensor-level rejects
            // need the FrameTriggerMiss event path instead.
            "FrameStart",
            edge_name.c_str(),            // "Line0RisingEdge"
            trigger_source_.c_str(),       // "Line0"
            "AnyLineRisingEdge",
            "FrameTrigger",
            "AcquisitionTrigger"
        };
        bool src_ok = false;
        const char* chosen = "";
        for (const char* c : candidates) {
            r = MV_CC_SetEnumValueByString(handle_, "CounterEventSource", c);
            if (r == MV_OK) { chosen = c; src_ok = true; break; }
        }
        if (!src_ok) {
            set_error(mv_err(r, "CounterEventSource (all variants failed)"));
            return;
        }
        set_info(std::string("CounterEventSource=") + chosen + " OK");

        MV_CC_SetEnumValueByString(handle_, "CounterTriggerActivation", "RisingEdge");
        MV_CC_SetCommandValue(handle_, "CounterReset");

        // GenICam SFNC uses `CounterValue` (selector-driven). HikRobot
        // firmwares sometimes expose `Counter0Value` / `CounterCurrentValue`
        // as aliases; try each.
        const char* value_nodes[] = {
            "CounterValue", "Counter0Value", "CounterCurrentValue"
        };
        bool got_value = false;
        MVCC_INTVALUE v{};
        for (const char* nn : value_nodes) {
            r = MV_CC_GetIntValue(handle_, nn, &v);
            if (r == MV_OK) {
                counter_value_node_ = nn;
                got_value = true;
                break;
            }
        }
        if (got_value) {
            counter_available_ = true;
            counter_baseline_.store((int64_t)v.nCurValue);
            set_info(std::string("Counter0 armed via ") + counter_value_node_);
        } else {
            set_error(mv_err(r, "Counter readback (all node names failed)"));
        }
    }

    // Pull every cumulative count MVS knows about for this session.
    // Updates atomics; returns true if the call succeeded on at least
    // one variant (USB or GigE).
    bool poll_transport_stats() {
        if (!handle_) return false;
        bool any = false;

        if (target_layer_ == MV_USB_DEVICE) {
            MV_MATCH_INFO_USB_DETECT u{};
            MV_ALL_MATCH_INFO info{};
            info.nType     = MV_MATCH_TYPE_USB_DETECT;
            info.pInfo     = &u;
            info.nInfoSize = sizeof(u);
            int r = MV_CC_GetAllMatchInfo(handle_, &info);
            if (r == MV_OK) {
                mv_received_frames_.store((int64_t)u.nReceivedFrameCount);
                mv_error_frames_   .store((int64_t)u.nErrorFrameCount);
                any = true;
            } else {
                char buf[96];
                std::snprintf(buf, sizeof(buf),
                    "poll_transport_stats USB: err=0x%08X", (unsigned)r);
                set_error(buf);
            }
        } else if (target_layer_ == MV_GIGE_DEVICE) {
            MV_MATCH_INFO_NET_DETECT g{};
            MV_ALL_MATCH_INFO info{};
            info.nType     = MV_MATCH_TYPE_NET_DETECT;
            info.pInfo     = &g;
            info.nInfoSize = sizeof(g);
            if (MV_CC_GetAllMatchInfo(handle_, &info) == MV_OK) {
                mv_received_frames_.store((int64_t)g.nNetRecvFrameCount);
                mv_lost_frames_    .store((int64_t)g.nLostFrameCount);
                mv_lost_packets_   .store((int64_t)g.nLostPacketCount);
                mv_resend_packets_ .store((int64_t)g.nResendPacketCount);
                any = true;
            }
        }
        return any;
    }

    int64_t read_edge_counter() {
        if (!counter_available_ || !handle_) return -1;
        // Re-select in case another code path changed CounterSelector.
        MV_CC_SetEnumValueByString(handle_, "CounterSelector", "Counter0");
        MVCC_INTVALUE v{};
        if (MV_CC_GetIntValue(handle_, counter_value_node_, &v) != MV_OK) return -1;
        return (int64_t)v.nCurValue - counter_baseline_.load();
    }

    void start_counter_thread() {
        counter_stop_.store(false);
        if (counter_thread_.joinable()) return;
        counter_thread_ = std::thread([this] {
            uint32_t tick = 0;
            while (!counter_stop_.load()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                if (counter_stop_.load()) break;
                counter_reap_if_idle();
                // Refresh transport stats at ~1 Hz (20 × 50 ms). A little
                // slow for fast change-detection but cheap enough.
                if ((++tick % 20) == 0) poll_transport_stats();
            }
        });
    }

    void stop_counter_thread() {
        counter_stop_.store(true);
        if (counter_thread_.joinable()) counter_thread_.join();
    }

    // Called periodically on the poller thread. If the pipeline has
    // been quiet for `counter_poll_idle_ms_`, ask the camera how many
    // edges it saw and emit missed placeholders for any that never
    // turned into outputs (captured or already-missed). Trigger
    // timestamps stay 0 — we know it happened, not when.
    void counter_reap_if_idle() {
        if (!strict_trigger_mode_ || trigger_mode_ != "on") return;
        if (!streaming_.load()) return;

        int64_t now     = now_ns();
        int64_t idle_ns = now - last_activity_ns_.load();
        if (idle_ns < (int64_t)counter_poll_idle_ms_ * 1000000LL) return;

        // Three parallel signals — take the max:
        //   1. Line0RisingEdge event count (firmware-reliable on MV-CE
        //      family; fires per electrical edge)
        //   2. Camera's Counter0 edge count (if firmware populates)
        //   3. MVS error-frame count (transport-level silent drops)
        int64_t outputs = captured_.load() + missed_count_.load();
        int64_t delta   = 0;

        int64_t edges = line0_edges_.load();
        if (edges > outputs) delta = edges - outputs;

        if (counter_available_) {
            int64_t cam_count = read_edge_counter();
            if (cam_count > outputs && (cam_count - outputs) > delta) {
                delta = cam_count - outputs;
            }
        }

        // MVS's nErrorFrameCount: frames MVS noticed but couldn't
        // decode (partial, CRC, etc.). These never reached consume_frame
        // so they're invisible to nFrameNum gap detection.
        poll_transport_stats();   // refresh before comparing
        int64_t mv_err = mv_error_frames_.load() - mv_err_baseline_.load();
        if (mv_err > 0) {
            // Count every error frame as a missed output if we haven't
            // already attributed that many misses to transmission_drop.
            // (Don't double-count.)
            int64_t already_mv = 0;  // placeholder — we don't track which
                                     // miss reason came from where
            int64_t mv_delta = mv_err - already_mv;
            if (mv_delta > delta) delta = mv_delta;
        }

        if (delta <= 0) return;

        std::lock_guard<std::mutex> lk(pending_mu_);

        // First, mark existing unfilled slots as missed — these are
        // edge-event-pushed slots whose frames never arrived. They
        // already carry the edge's device_ns, so downstream gets the
        // real camera-clock time of each missed edge, not a zero.
        int64_t marked = 0;
        for (auto& s : pending_) {
            if (marked >= delta) break;
            if (!s.filled && !s.missed) {
                s.missed      = true;
                s.miss_reason = "counter_mismatch";
                missed_count_.fetch_add(1);
                ++marked;
            }
        }

        // If the delta is bigger than the unfilled slots we had, the
        // leftover triggers weren't captured by events either (rare —
        // means line0_edges_ reports more than our event callback saw,
        // which shouldn't happen but we handle defensively). Synthesize
        // the remainder; these get the most-recent edge's device_ns as
        // the best available camera-clock approximation.
        int64_t still = delta - marked;
        int64_t last_dev = line0_last_edge_device_ns_.load();
        for (int64_t i = 0; i < still; ++i) {
            TriggerSlot syn;
            syn.trigger_id  = next_trigger_id_.fetch_add(1);
            syn.source      = "hardware";
            syn.missed      = true;
            syn.miss_reason = "counter_mismatch";
            syn.device_ns   = last_dev;   // approximate camera time
            syn.deadline    = std::chrono::steady_clock::now();
            pending_.push_back(std::move(syn));
            triggers_sent_.fetch_add(1);
            triggers_sent_hw_.fetch_add(1);
            missed_count_.fetch_add(1);
        }

        last_activity_ns_.store(now);
        pending_cv_.notify_all();
    }

    void set_error(const std::string& msg) {
        last_error_ = msg;
        log_error("[hikrobot_camera] " + msg);
    }

    // For diagnostic messages that aren't errors (e.g. counter probe
    // succeeded, event enumeration result). These show up in the host
    // log but don't populate `last_error` — keeps the UI/status clean.
    void set_info(const std::string& msg) {
        log_info("[hikrobot_camera] " + msg);
    }

private:
    // --- config (persisted) ---
    std::string device_key_;              // "USB:<serial>" / "GIGE:<serial>"
    std::string device_label_;            // human-readable, discovery-time
    std::string pixel_format_ = "Mono8";
    int    roi_x_       = 0;
    int    roi_y_       = 0;
    int    roi_w_       = 0;
    int    roi_h_       = 0;
    double fps_         = 0.0;            // 0 = don't force, use camera default
    double exposure_us_ = 10000.0;
    double gain_        = 0.0;
    bool   mirror_x_    = false;
    bool   mirror_y_    = false;

    // --- runtime ---
    void*                handle_       = nullptr;
    unsigned             target_layer_ = 0;
    std::atomic<bool>    connected_{false};
    std::atomic<bool>    streaming_{false};
    std::atomic<int64_t> captured_{0};
    std::atomic<int64_t> dropped_{0};
    std::atomic<int64_t> lost_packets_{0};
    std::atomic<unsigned> last_seq_{0};
    std::atomic<bool>     last_seq_valid_{false};
    std::atomic<int64_t>  triggers_sent_{0};
    std::atomic<int64_t>  triggers_sent_sw_{0};
    std::atomic<int64_t>  triggers_sent_hw_{0};
    std::atomic<int64_t>  triggers_rejected_{0};
    std::atomic<unsigned> triggers_accepted_{0};
    std::string trigger_mode_   = "off";
    std::string trigger_source_ = "Software";
    std::string          last_error_;

    // --- discovery cache ---
    mutable std::mutex         devices_mu_;
    std::vector<std::string>   device_labels_;
    std::vector<std::string>   device_keys_;
    std::vector<MV_CC_DEVICE_INFO*> last_enum_;

    // --- latest frame slot ---
    std::mutex latest_mu_;
    xi::Image  latest_;
    int64_t    latest_id_        = 0;
    int64_t    latest_host_ns_   = 0;
    int64_t    latest_device_ns_ = 0;

    // --- strict-trigger pairing ---------------------------------------
    //
    // When strict_trigger_mode_ && trigger_mode_=="on", every software
    // trigger pushes a TriggerSlot and every process() drains the front
    // of the queue (either with a real frame or with a placeholder if
    // the deadline passed). This gives downstream code a 1:1 trigger→
    // process() mapping — no frame skipping — at the cost of process()
    // potentially blocking briefly while the camera finishes exposing.
    struct TriggerSlot {
        int64_t trigger_id = 0;
        std::chrono::steady_clock::time_point deadline{};
        xi::Image frame;         // empty until filled
        bool      filled = false;
        bool      missed = false;
        // "software" if we pushed it from software_trigger(),
        // "hardware" if from a Line* FrameTrigger camera event.
        std::string source = "software";
        // Only meaningful when missed:
        //   "timeout"           — slot deadline expired without a frame
        //   "firmware_reported" — camera's FrameTriggerMiss event (hardware only)
        std::string miss_reason;
        // All times in nanoseconds. Keep them in distinct clock domains
        // so the caller can pick the right one:
        //   trigger_ns  — our steady_clock at the instant we registered
        //                 the trigger. Pairs with received_ns for latency.
        //   received_ns — our steady_clock when the frame callback fired.
        //                 Same domain as trigger_ns.
        //   host_ns     — MVS's nHostTimeStamp on the delivered frame
        //                 (separate host-clock domain, persisted for
        //                 cross-process correlation).
        //   device_ns   — camera-internal timestamp (high<<32 | low).
        //                 Use this for precise inter-frame timing — it's
        //                 immune to USB scheduling jitter.
        int64_t  trigger_ns  = 0;
        int64_t  received_ns = 0;
        int64_t  host_ns     = 0;
        int64_t  device_ns   = 0;
    };

    static int64_t now_ns() {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    }

    bool       strict_trigger_mode_ = false;
    int        strict_timeout_ms_   = 0;   // 0 = derive from exposure
    std::mutex                pending_mu_;
    std::condition_variable   pending_cv_;
    std::deque<TriggerSlot>   pending_;
    std::atomic<int64_t>      next_trigger_id_{1};
    // Separate counter assigned at drain time so output `trigger_id` is
    // monotonic in physical (device_ns) order, not event-callback order.
    // MVS can reorder Line0RisingEdge callbacks during fast bursts; the
    // push-order tid would therefore appear non-monotonic in output.
    std::atomic<int64_t>      next_output_tid_{1};
    std::atomic<int64_t>      missed_count_{0};
    std::atomic<int64_t>      last_activity_ns_{0};

    // Transport-level counts (from MV_CC_GetAllMatchInfo). Refreshed by
    // the counter thread every poll cycle, exposed via get_status +
    // diag, and used as an extra miss-detection signal (nErrorFrameCount
    // delta = frames MVS saw go wrong that we never received as a
    // callback).
    std::atomic<int64_t>      mv_received_frames_{0};
    std::atomic<int64_t>      mv_error_frames_{0};
    // GigE-only (zeros on USB3):
    std::atomic<int64_t>      mv_lost_frames_{0};
    std::atomic<int64_t>      mv_lost_packets_{0};
    std::atomic<int64_t>      mv_resend_packets_{0};
    // Baseline snapshot at connect so our stats are per-session.
    std::atomic<int64_t>      mv_err_baseline_{0};
    // Highest nFrameCounter observed across delivered frames (separate
    // from our `captured_` and from nFrameNum; firmware-dependent).
    std::atomic<int64_t>      mv_frame_counter_{0};
    // Edge count from Line0RisingEdge events — firmware-reliable on
    // MV-CE200; fires per electrical edge regardless of sensor state.
    // Primary signal for the counter-reaper now that we know the
    // on-camera Counter0 node doesn't count continuously.
    std::atomic<int64_t>      line0_edges_{0};
    std::atomic<int64_t>      line0_falling_edges_{0};
    std::atomic<int64_t>      line0_last_edge_device_ns_{0};
    // Debug: last BlockId seen on a Line0RisingEdge callback. MVS headers
    // mark this field as "no firmware support" — we expose it so a probe
    // can verify it really stays 0 on MV-CE200 / MV-CA050 firmware.
    std::atomic<int64_t>      last_event_block_id_{0};

    // Generic event-probe map: names → fire counts. Populated by a
    // catch-all callback registered via `probe_events`. Read back by
    // `get_event_counts`.
    std::mutex                event_counts_mu_;
    std::map<std::string, int64_t> event_counts_;

    // Counter-based reaper for hardware-trigger tail drops. If no frame
    // arrives for `counter_poll_idle_ms_` ms, poll the camera's Counter0
    // (configured at connect to count Line0 edges) and synthesize
    // missed placeholders for any triggers the camera saw but we never
    // received. Timestamps stay 0 — we know how many were missed but
    // not when each one happened.
    int                   counter_poll_idle_ms_ = 500;
    bool                  counter_available_    = false;
    const char*           counter_value_node_   = "CounterValue";
    std::thread           counter_thread_;
    std::atomic<bool>     counter_stop_{false};
    std::atomic<int64_t>  counter_baseline_{0};   // Counter0 value at (re)start

    // Test-only knob: if >0, consume_frame silently discards every Nth
    // frame so the strict-mode transmission-drop handling can be
    // exercised on a link that doesn't naturally saturate.
    int      debug_drop_every_n_ = 0;
    uint32_t debug_drop_counter_  = 0;
};

XI_PLUGIN_IMPL(HikRobotCamera)
