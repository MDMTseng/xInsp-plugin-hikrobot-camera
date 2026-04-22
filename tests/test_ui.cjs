//
// test_ui.cjs — live camera E2E inside VS Code.
//
// Creates a project, adds a hikrobot_camera instance, opens the webview,
// discovers devices, connects to index 0, streams at ~10 fps, waits until
// ≥10 frames are captured, then screenshots the whole VS Code window.
//
// Requires an MVS-visible HikRobot camera to be connected. Fails if the
// discover() list is empty.
//

module.exports = {
    async run(h) {
        const projDir = h.tmp();

        // 1. Project + instance
        await h.createProject(projDir, 'hikrobot_demo');
        await h.addInstance('cam0', 'hikrobot_camera');

        // 2. Webview
        await h.openUI('cam0', 'hikrobot_camera');
        await h.sleep(400);
        h.shot('01_ui_opened');

        // 3. Discover — camera must be present
        await h.sendCmd('cam0', { command: 'discover' });
        await h.sleep(600);
        h.shot('02_after_discover');

        await h.getStatus('cam0');
        const devices = (h.lastStatus && h.lastStatus.devices) || [];
        h.expect(devices.length > 0, `discover found ${devices.length} devices (need ≥1)`);
        if (devices.length === 0) {
            h.shot('99_no_devices');
            return;
        }
        console.log('  devices:', devices);

        // 4. Connect — the plugin auto-starts streaming after a successful connect.
        await h.sendCmd('cam0', { command: 'connect', index: 0 });
        await h.sleep(1500); // give MVS time to open + start grabbing
        await h.getStatus('cam0');
        h.expect(!!h.lastStatus.connected, 'camera connected');
        h.expect(!!h.lastStatus.streaming, 'streaming');
        h.shot('03_connected');

        // 5. Force a sane framerate AFTER connect (the MV-CA050 won't honor
        //    AcquisitionFrameRate until the device is open and the feature
        //    tree is populated; setting it pre-connect just caches a value
        //    that races the camera's own defaults). Exposure is high enough
        //    that the preview is visibly non-black even under office light.
        await h.sendCmd('cam0', { command: 'set_exposure',   value: 500000 });  // 500 ms
        await h.sendCmd('cam0', { command: 'set_gain',       value: 15.0 });
        await h.sendCmd('cam0', { command: 'set_frame_rate', value: 5.0 });
        await h.sleep(400);
        await h.getStatus('cam0');
        if (h.lastStatus.last_error) {
            console.log('  last_error after fps apply:', h.lastStatus.last_error);
        }

        // 6. Poll until captured >= 10 (hard cap: 25 seconds)
        const target = 10;
        const deadline = Date.now() + 25000;
        let lastCaptured = 0;
        while (Date.now() < deadline) {
            await h.getStatus('cam0');
            lastCaptured = (h.lastStatus && h.lastStatus.captured) || 0;
            if (lastCaptured >= target) break;
            await h.sleep(300);
        }
        console.log('  captured frames:', lastCaptured);
        h.expect(lastCaptured >= target,
                 `captured ${lastCaptured} frames (want ≥${target})`);

        // 7. Explicitly poke get_preview so a JPEG is ready to render
        //    independent of the webview's own polling cadence.
        await h.sendCmd('cam0', { command: 'get_preview', max_side: 640, quality: 70 });
        await h.sleep(300);
        h.expect(h.lastStatus && h.lastStatus.has_frame === true,
                 'get_preview returned an encoded frame');

        // 7b. Dump the live JPEG to disk so you can eyeball the actual
        //     camera image without needing to decode the webview screenshot.
        if (h.lastStatus && h.lastStatus.jpeg_b64) {
            const fs   = require('fs');
            const path = require('path');
            const out  = path.join(__dirname, 'screenshots', 'live_preview.jpg');
            fs.writeFileSync(out, Buffer.from(h.lastStatus.jpeg_b64, 'base64'));
            console.log('  wrote live_preview.jpg (' +
                        h.lastStatus.width + 'x' + h.lastStatus.height + ')');
        }

        // 8. Give the webview a beat to paint the <img> then screenshot.
        await h.sleep(1200);
        h.shot('04_captured_10_frames_with_preview');

        // 9. Run one inspection cycle so the webview reflects a fresh process()
        await h.run();
        await h.sleep(600);
        h.shot('05_after_run');

        // 10. Clean up
        await h.sendCmd('cam0', { command: 'disconnect' });
        await h.sleep(400);
        h.shot('06_disconnected');
    },
};
