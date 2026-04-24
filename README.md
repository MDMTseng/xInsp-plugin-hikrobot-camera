# hikrobot_camera

xInsp2 image-source plugin for HikRobot / HIKVISION industrial cameras via the MVS SDK. Supports USB3 Vision and GigE Vision devices with both free-run and hardware-triggered acquisition.

**Key feature**: *strict trigger mode* — a 1:1 invariant between external trigger edges and `process()` outputs. Every edge that fires produces exactly one record in order; edges whose frames were lost become placeholder records with `missed:true` and accurate camera-clock timestamps. Downstream pipelines can index by `trigger_id` without ever losing alignment.

---

## Quick build

Requires HikRobot MVS. `MVCAM_COMMON_RUNENV` (set by the MVS installer) or `-DMVS_ROOT=<Development-folder>` points the CMake at the SDK.

```
cmake -S . -B build -A x64
cmake --build build --config Release
```

Outputs the plugin DLL plus all test binaries next to `plugin.json`.

---

## `xi::Record` output shape

Every `process()` call returns a record shaped like:

```json
// Successful frame
{
  "out": <Image>,
  "frame_id":      42,
  "trigger_id":    42,
  "trigger_source": "hardware",
  "missed":        false,
  "width": 640, "height": 480, "channels": 1,
  "ts_trigger_ns":  ...,     // host steady_clock at edge-event receipt
  "ts_received_ns": ...,     // host steady_clock at frame callback
  "ts_host_ns":     ...,     // MVS's nHostTimeStamp (0 on firmwares that don't populate)
  "ts_device_ns":   ...,     // camera-internal clock (units vary per model)
  "latency_ns":     ...,     // ts_received_ns − ts_trigger_ns
  ...counters...
}

// Missed slot (placeholder preserving the 1:1 invariant)
{
  "out": <empty Image>,      // .empty() returns true
  "frame_id":      43,
  "trigger_id":    43,       // still contiguous with surrounding real slots
  "trigger_source": "hardware",
  "missed":        true,
  "miss_reason":   "transmission_drop",  // or "counter_mismatch" / "flushed" / "timeout" / "firmware_reported"
  "error":         "missed_hw_trigger_timeout",
  "ts_trigger_ns":  ...,     // populated by Line0RisingEdge event — NOT zero
  "ts_device_ns":   ...,     // populated from the same event's TimestampHigh/Low
  "ts_host_ns":     0,       // no frame arrived, this stays 0
  "ts_received_ns": 0,
  ...counters...
}
```

Downstream code can always do:

```cpp
auto rec = cam->process({});
if (rec["missed"].as_bool()) {
    // drop / substitute / log / etc — but always advance the pipeline index
} else {
    auto frame = rec.get_image("out");
    // normal inspection path
}
```

---

## Config fields (persisted in `instance.json`)

| Field | Type | Notes |
|---|---|---|
| `device_key` | `"USB:<serial>"` / `"GIGE:<serial>"` | Stable identifier; preferred over index |
| `pixel_format` | `Mono8` \| `RGB8Packed` \| `BGR8Packed` | Bayer etc. auto-converted to BGR8 |
| `roi_x/y/w/h` | int | `0/0/0/0` = full sensor |
| `fps` | double | `0` = disable AFR |
| `exposure_us`, `gain` | double | Auto modes force to Off before applying |
| `mirror_x`, `mirror_y` | bool | `ReverseX` / `ReverseY` |
| `trigger_mode` | `off` \| `on` | |
| `trigger_source` | `Software` \| `Line0..Line3` \| `Anyway` | See caveats |
| `strict_trigger_mode` | bool | 1:1 trigger→output enforcement |
| `strict_timeout_ms` | int | Per-slot deadline; `0` = `exposure_us / 1000 + 200` ms |

---

## Exchange commands

### Connect / configure

| Command | Effect |
|---|---|
| `discover` | Repopulate device list |
| `connect` + `device_key` / `index` | Open + configure + start |
| `disconnect` | Stop + release |
| `start_streaming` / `stop_streaming` | Manual `MV_CC_StartGrabbing` / `StopGrabbing` |
| `set_roi` / `set_frame_rate` / `set_exposure` / `set_gain` / `set_mirror` / `set_pixel_format` | Runtime tuning |
| `reset_roi` | Snap ROI back to the sensor's native max dimensions |
| `set_feature` with `name`, `type` (`int`/`float`/`bool`/`enum`/`command`/`string`), `value` | Generic GenICam node setter; dispatches to the right `MV_CC_Set*` and reads the node back so the reply's `cur` reflects the camera's actually-applied (possibly clamped / snapped) value. Lets the UI drive any node exposed by `get_schema` without a hardcoded command per feature |
| `get_schema` | Dump every supported node grouped by purpose (see [Schema groups](#schema-groups)). The UI uses this to render its control panels |

### Trigger + strict mode

| Command | Effect |
|---|---|
| `set_trigger_mode` with `mode` (`off`/`on`), `source` (`Software`/`Line0..3`/`Anyway`) | Switch free-run / triggered |
| `set_strict_trigger_mode` with `value` (bool), `timeout_ms` | Toggle 1:1 pairing |
| `software_trigger` with `count` | Fire N software triggers |
| **`flush_pending`** with `expected` (int) | Close a burst cleanly — marks unfilled slots as `missed` and, if `expected > captured+missed`, synthesizes additional `flushed` placeholders to reach N |

### Diagnostics / observability

| Command | Effect |
|---|---|
| `get_preview` with `max_side`, `quality` | Base64-JPEG of the latest frame (for UI preview) |
| `diag` | Dump camera GenICam node values (ExposureTime, TriggerMode, ResultingFrameRate, DeviceLinkCurrentThroughput, etc.) |
| `probe_events` with `names` array | Try enabling each named event; reports which ones the firmware accepts. Callbacks forward to the internal handlers for real events (FrameTrigger, Line0RisingEdge, etc.) — probing doesn't disable normal operation |
| `get_event_counts` | Returns fire-count per event since last probe |
| `get_edge_counter` | Reads the camera's `Counter0Value` — usable only if the firmware supports continuous edge counting (not on either of our tested models) |
| `debug_counter_src` with `value` | Probe which `CounterEventSource` enum values the firmware accepts (diagnostic) |
| `debug_drop_every_n` with `value` | Test-only — silently discard every Nth frame in `consume_frame` to exercise drop-handling code paths |

---

## Schema groups

`get_schema` returns a `features` array plus a `groups` array that orders the panels the UI renders. Each feature has `name` / `label` / `group` / `type` (`int`/`float`/`bool`/`enum`/`string`) / `cur`, plus `min`/`max`/`inc`/`unit` for numerics and `values` for enums. Pair with `set_feature` to write any of them.

| Group | Purpose | Notable nodes |
|---|---|---|
| `device` | Read-only identity + thermals (asset-tag fodder) | `DeviceVendorName`, `DeviceModelName`, `DeviceSerialNumber`, `DeviceFirmwareVersion`, `DeviceUserID`, `DeviceTemperature` |
| `image` | Resolution + ROI + pixel format | `PixelFormat`, `Width`, `Height`, `OffsetX`, `OffsetY`, `ReverseX`/`ReverseY` |
| `exposure` | Exposure, gain, and auto-control bounds/target | `ExposureAuto`, `ExposureTime`, `GainAuto`, `Gain`, `AutoExposureTimeLowerLimit`/`UpperLimit`, `AutoGainLowerLimit`/`UpperLimit`, `AETargetValue` |
| `acquisition` | Frame rate + (where supported) shutter mode | `AcquisitionMode`, `AcquisitionFrameRate`, `ResultingFrameRate` (ro), `SensorShutterMode`, `GlobalResetReleaseMode` |
| `trigger` | Trigger selector / mode / source / activation / delay | `TriggerSelector`, `TriggerMode`, `TriggerSource`, `TriggerActivation`, `TriggerDelay` |
| `io` | Per-line GPIO config (selector pattern), strobe, user-output, timer. `LineSelector` / `UserOutputSelector` / `TimerSelector` gate the rest — the UI re-pulls the schema after each selector change so the dependent nodes refresh | `LineSelector`, `LineMode`, `LineSource`, `LineInverter`, `LineStatus` (ro), `LineStatusAll` (ro), `LineDebouncerTime`, `StrobeEnable` / `StrobeLineSource` / `StrobeLineDuration` / `StrobeLineDelay` / `StrobeLinePreDelay`, `UserOutputSelector` / `UserOutputValue` / `UserOutputValueAll` (ro), `TimerSelector` / `TimerDuration` / `TimerDelay` / `TimerTriggerSource` / `TimerTriggerActivation` |
| `color` | Tonemap / white balance / black level (color sensors only — silently skipped on mono) | `GammaEnable`, `Gamma`, `BalanceWhiteAuto`, `BalanceRatioSelector`, `BalanceRatio`, `Saturation`, `Hue`, `Sharpness`, `Brightness`, `BlackLevelEnable`, `BlackLevel`, `DigitalShiftEnable`, `DigitalShift` |
| `transport` | Link / throughput diagnostics | `DeviceLinkSpeedInBps`, `DeviceLinkCurrentThroughput` |
| `userset` | User-set load / save selector | `UserSetSelector` |

Nodes the connected firmware doesn't expose are silently omitted, so the same schema shape works across mono / color and small / large sensors — the UI filters to what's actually present.

---

## Detection mechanisms

Five independent paths, used in combination:

### 1. `nFrameNum` gap detection

Every delivered frame carries the camera's monotonic `nFrameNum`. When the next frame's `nFrameNum` jumps by more than 1, the gap-absorb code pre-marks that many oldest unfilled slots as `missed=true, miss_reason="transmission_drop"`. The missing frames were exposed by the sensor but lost in USB/MVS transport.

**Limitation**: purely retrospective — only fires when a *subsequent* frame arrives. Can't catch tail drops on its own.

### 2. `Line0RisingEdge` event path (**primary for hardware trigger**)

Registered on connect. Every rising edge on the camera's Line0 input fires a callback that:

- Increments `line0_edges` (firmware-reliable edge count)
- Records `TimestampHigh/Low` into `line0_last_edge_device_ns`
- **Pushes a pristine `TriggerSlot`** into the pending queue with `device_ns` set from the event's camera-clock timestamp

Downstream frames (from accepted triggers) then fill these slots FIFO. Slots that never get filled — because the sensor rejected the trigger, or the frame was dropped, or it's the tail of the burst — stay pending and eventually get marked missed, *carrying accurate per-edge `device_ns`*.

**This is why missed slots have non-zero `ts_device_ns`**: the event gave us the camera-clock time of every edge, independent of whether it became a frame.

### 3. `Line0FallingEdge` event — pulse integrity cross-check

Same semantic as rising, counted separately. `line0_edges == line0_falling_edges` means every pulse the ESP32 emitted reached the camera's input cleanly. Divergence flags signal-integrity issues (bounce, level below opto threshold).

### 4. Counter-mismatch reaper (background thread)

A 50 ms-tick thread that, after `counter_poll_idle_ms` of no frame activity, checks whether `line0_edges > captured + missed`. If so, the extra edges fired but produced no output — most commonly the tail of a burst. The reaper marks the oldest unfilled slots as `missed, miss_reason="counter_mismatch"`. If somehow there are fewer pending slots than the edge delta (shouldn't happen with events on but handled defensively), it synthesizes additional placeholders, using `line0_last_edge_device_ns` as the best-available approximate `device_ns`.

Same thread also polls `MV_CC_GetAllMatchInfo` ~1 Hz for USB transport stats (`nReceivedFrameCount`, `nErrorFrameCount`) and GigE stats (`nLostFrameCount`, etc.) — exposed in `get_status`.

### 5. Host-driven `flush_pending` (deterministic backstop)

```
{"command":"flush_pending","expected":N}
```

After the host knows the trigger source has stopped (e.g. ESP32 `DONE` line received), it calls `flush_pending` with the total trigger count. The plugin marks any unfilled slots as `missed, miss_reason="flushed"` and — if `captured + missed` is still below `N` — synthesizes additional placeholders to bring the total to exactly N. This is the guaranteed deterministic path; unlike the reaper, it doesn't wait for an idle window.

Use `flush_pending` when you can't afford the reaper's 500 ms idle wait (e.g., you need the "burst complete" signal to come back immediately).

### Reliability summary

Detection mechanism | Catches | Missed-slot timestamps |
---|---|---
`nFrameNum` gap | mid-burst transmission drop | `device_ns` from `Line0RisingEdge` event ✅
`Line0RisingEdge` slot-push | every edge, including sensor-rejected ones | `device_ns` from event ✅
Line-edge reaper | tail drops + sensor rejects | `device_ns` from event on pre-pushed slots ✅, from `last_edge` on synthesized ones
`flush_pending` | explicit burst end / residual after reaper | 0 (honest "unknown") — flush is for when even the reaper can't help
`FrameTriggerMiss` event | sensor-busy rejects | device timestamp from event (if firmware supports event — **neither MV-CA050 nor MV-CE200 do**)

---

## HikRobot MVS caveats found during development

Every item below cost a debug cycle; the plugin handles each one automatically. Written down so the next person doesn't have to re-learn.

### 1. Feature writes fail with `MV_E_USB_WRITE` (`0x80000301`) right after `OpenDevice`

USB3 cameras — especially 20 MP MV-CE200 — need a settle window after `MV_CC_OpenDevice` before feature sets take. First attempts fail with `0x80000301`. **Fix**: 500 ms sleep after open before any feature set. Plugin does this unconditionally.

### 2. Previous crashed processes can leave the camera streaming

If a prior process exited without `StopGrabbing`+`CloseDevice`, the camera firmware keeps feeding its USB endpoint. A fresh `OpenDevice` succeeds but feature writes silently fail. **Fix**: defensive `MV_CC_StopGrabbing` after the post-open sleep. If it wasn't streaming, it's a no-op.

**Nuclear recovery**: `tools/reset_cameras.ps1` disables + re-enables the HikRobot PnP device (VID 2BDF), tearing down and re-enumerating the USB state machine cleanly. No admin prompt required in an elevated session.

### 3. `StartGrabbing` can fail on big sensors in free-run mode

For 20 MP sensors over shared USB, a free-run `StartGrabbing` can refuse outright. **Fix**: apply `TriggerMode=On` *inside* `connect_by()` before the first `StartGrabbing`, so the camera starts in trigger-wait state directly and never attempts free-run at full resolution.

### 4. `TriggerMode=On` is a no-op without `TriggerSelector` first

`TriggerMode`/`TriggerSource`/`TriggerActivation` all operate on the *currently selected* trigger. Without `TriggerSelector`, the firmware default (often `AcquisitionStart`) is what you configure — wrong for "one frame per edge". **Fix**: set `TriggerSelector=FrameBurstStart` (with `FrameStart` fallback) before `TriggerMode=On`.

### 5. `AcquisitionFrameRate` defaults to disabled

Without `AcquisitionFrameRateEnable=true`, the camera free-runs at whatever rate the sensor can sustain at current exposure — often ≤ 1 fps on big sensors. **Fix**: `set_frame_rate` with a positive value automatically enables AFR.

### 6. USB 2.0 throttling is silent

Camera reports `ResultingFrameRate=11` but you only receive ~0.8 fps with `dropped=0`. The USB controller silently throttles below what the camera thinks it's delivering; MVS doesn't surface it as a drop. **Diagnose**: `diag` → `DeviceLinkCurrentThroughput` in the 50 KB/s – 30 MB/s range means USB 2.0 HighSpeed, not USB 3.0 SuperSpeed. **Fix**: replug into a USB 3.0 port with the official HikRobot cable — no software fix helps.

### 7. `TriggerSource="Anyway"` suppresses the `FrameTrigger` event

With `Anyway`, the camera accepts edges from any Line0/1/2 but silences the named trigger events. Strict mode's pending-slot pushing from events goes dormant. **Fix**: the plugin falls back to synthesis-on-arrival using `nFrameNum`. On `Line0` source (as used in production), `Line0RisingEdge` fires reliably instead.

### 8. Device timestamp units vary by model

`ts_device_ns` is **camera-specific tick units**, not nanoseconds. The MV-CA050-12UC uses a 10 MHz tick (100 ns per reported unit). Other models may use 1 ns or 1 µs per tick. `ts_host_ns` (from `nHostTimeStamp`) is usually 0 on USB3 firmwares. **Use for wall-time work**: `ts_trigger_ns` and `ts_received_ns` (both host `steady_clock` nanoseconds); **use for inter-frame jitter**: `ts_device_ns` deltas, knowing the camera-specific tick size.

### 9. `MV_CC_EnumDevices` doesn't return claimed devices

Once a device is opened in the current process, a subsequent `EnumDevices` call from another plugin instance doesn't see it. **Fix**: discover once globally, then connect each instance by `device_key` (not `index`).

There's also a second reason to avoid `index`: MVS enumerates every supported transport (USB3 *and* GigE), so a non-HikRobot GEV device on the same network — e.g. a WTX1000 code reader — can land at `index=0` and silently steal the connection. The Line0 trigger path then sees zero edges and the test hangs at `drain=0`. Stick to `device_key` + the `USB:` prefix filter.

### 10. `Counter0` is one-shot on HikRobot USB3 firmware

`CounterSelector=Counter0` + `CounterEventSource=Line0` is accepted by the firmware and the counter ticks to 1 on the first edge — then stops. It's structurally a "count to N and signal" primitive, not a free-running tally. `FrameStart`/`ExposureStart`/`Line0RisingEdge` are **rejected** as event sources on our firmware. **Fix**: the plugin uses the `Line0RisingEdge` event for edge counting instead, which works reliably (see Detection #2).

### 11. `FrameTrigger` / `FrameTriggerMiss` events are not supported on MV-CE200 or MV-CA050

`EventSelector=FrameTrigger` is rejected. Registration has no effect. **Fix**: plugin registers them anyway (for forward compatibility with firmwares that support them) and uses `Line0RisingEdge` as the primary per-edge signal.

### 12. Concurrent two-camera startup races at the USB control channel

First `StartGrabbing` on the second camera can return `MV_E_USB_WRITE` when two cameras initialize near-simultaneously. **Fix**: `start_streaming` retries up to 5 times with 150 ms spacing. Observed to succeed on attempt 2 in all our runs.

### 13. Miss attribution drifts to tail without a stale-slot sweep

The Line0RisingEdge path pushes one slot per edge; sensor-rejected edges produce slots that never fill. Naive FIFO fill back-fills the rejected slot with a later edge's frame, so the miss appears at the *tail* of the burst instead of the edge that was actually rejected. **Fix**: before each FIFO fill, sweep pending slots and mark any whose `trigger_ns` age exceeds `2 × exposure + 200 ms` as `miss_reason="firmware_reported"`. That window is comfortably longer than expected frame delivery latency under backpressure but much shorter than "you'd have seen the frame by now if the sensor had accepted the edge". Verified by `hikrobot_surge` (surge-pair in mid-burst now correctly attributed at the reject position with the right reason).

---

## Verified on

Both HikRobot USB3 Vision cameras on our test rig:

| Model | Serial | Role in testing |
|---|---|---|
| **MV-CE200-11UM** (20 MP mono) | `00DA5328883` | Free-run max ~33 fps at 2448×2048 Mono8 on USB 3.0; throughput ~168 MB/s |
| **MV-CA050-12UC** (5 MP color) | `K47674142` | USB-link-speed diagnostic target; self-paces under bandwidth pressure |

Firmware event-support matrix (both cameras show identical behaviour):

```
accepted + firing:  AcquisitionStart, AcquisitionEnd,
                    FrameStart, FrameEnd,
                    ExposureStart, ExposureEnd,
                    FrameBurstStart, FrameBurstEnd,
                    Line0RisingEdge, Line0FallingEdge
accepted but silent: (none)
rejected by fw:     FrameTrigger, FrameTriggerMiss,
                    AcquisitionTrigger, AcquisitionTriggerMiss,
                    LineRisingEdge (generic), LineFallingEdge (generic),
                    Line1RisingEdge, Line2RisingEdge,
                    DeviceTemperature, ControlLost, StreamControlLost, Error
```

---

## Test matrix

| Binary | Purpose | Requirements |
|---|---|---|
| `hikrobot_camera_test` | Baseline C-ABI cert + 4 unit tests | None |
| `hikrobot_camera_live` | Single-cam free-run + fps / throughput diagnostics | 1 camera |
| `hikrobot_camera_overrun` | Software-trigger overrun demo | 1 camera |
| `hikrobot_camera_strict` | Strict-mode 1:1 trigger→frame demo | 1 camera |
| `hikrobot_two_cam_sw` | Two-camera strict software-trigger | 2 cameras |
| `hikrobot_two_cam` | Two-camera hardware-trigger E2E via ESP32 xtrig | 2 cameras + ESP32 |
| `hikrobot_underrun` | Mid-burst transmission-drop handling (uses drop hook) | 1 camera + ESP32 |
| `hikrobot_tail_drop` | Tail-drop detection via reaper + `flush_pending` | 1 camera + ESP32 |
| `hikrobot_timestamps` | Interval accuracy vs ESP32 firing rate | 1 camera + ESP32 |
| `hikrobot_event_probe` | Which events the firmware supports on one camera | 1 camera + ESP32 |
| `hikrobot_event_probe_all` | Same, iterated across all connected cameras | All cameras + ESP32 |
| `hikrobot_counter_probe` | Which `CounterEventSource` values the firmware honours | 1 camera + ESP32 |
| `hikrobot_io_probe` | Iterate `LineSelector` (Line0/1/2) + `UserOutputSelector`; print the `io` features each selector state exposes | 1 camera |
| `hikrobot_io_line1` | Select `Line1`, dump `get_schema` (quick look at `LineSource` enum on an output line) | 1 camera |
| **`hikrobot_stress`** | Multi-cam HW-triggered soak with 25% forced drops; CLI: `hikrobot_stress <N> <HZ>` (defaults `300 30`) | 2 cameras + ESP32 |
| **`hikrobot_surge`** | 10 Hz baseline + 4-edge burst at 200 Hz + 10 Hz recovery; verifies sensor-busy reject gets `missed=firmware_reported` with intact timestamps | 1 camera + ESP32 |

### Measured stress-test result

Torture: both cameras, Line0 hardware trigger, 300 triggers at 30 Hz, `debug_drop_every_n=4` on both (25% synthetic drop rate).

```
cam0: 300/300 outputs drained (225 real + 75 missed)
cam1: 300/300 outputs drained (225 real + 75 missed)
Row-by-row trigger_id match:            300 / 300  (100%)
Missed slots with non-zero trigger_ts:   75 /  75  (100% per camera)
Missed slots with non-zero device_ts:    75 /  75  (100% per camera)
line0_edges == line0_falling_edges:     300 == 300  (both cameras)
Real-frame interval at 30 Hz:           mean 33.32 ms, min 32.29, max 34.26 (stddev ~1 ms)
```

**100% 1:1 trigger→record invariant holds under multi-camera high-rate stress with forced drops.**

---

## Useful associated artifacts

- **`tools/reset_cameras.ps1`** — PnP disable/enable for HikRobot VID (`2BDF`). Run when MVS starts returning `USB_WRITE` on every feature set.
- **`../../trigger_peripheral/`** — ESP32 `xtrig` firmware driving hardware triggers. See its README for wiring + command protocol.
- **`ui/index.html`** — webview with live JPEG preview, trigger / strict-mode / stats panels, schema-driven device-info / auto-exposure-bounds / digital-IO / color panels (all rendered from `get_schema`), plus a drag-to-frame ROI overlay on the preview that maps to sensor-pixel coordinates and calls `set_roi` (paired with a "Reset ROI" button that calls `reset_roi`).

---

## Wiring a hardware trigger

For ESP32 / Arduino driving `Line0` on HikRobot opto-isolated inputs:

- **Voltage**: cameras need 5–24 V on `Line0`; ESP32 GPIO at 3.3 V is usually below the opto threshold. Either level-shift (pull-up to +5 V through ~2.2 kΩ, or a logic-level MOSFET) or use an opto (6N137 / TLP2309) on the ESP32 output side.
- **Selector**: the plugin uses `TriggerSelector=FrameBurstStart` with burst count 1 (one frame per edge).
- **Source**: `Line0` is the proven path (gives `Line0RisingEdge` events). `Anyway` works for frames but suppresses the trigger event stream — prefer `Line0`.

`hikrobot_two_cam.exe` exercises the whole path end-to-end with two cameras, an ESP32 driving both, and strict mode preserving index alignment. Use it as the reference integration test.
