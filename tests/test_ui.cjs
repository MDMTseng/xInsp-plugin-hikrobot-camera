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

        // 4. Connect to the known 20 MP CE200 test cam by device_key.
        //    Falls back to first USB cam if that specific unit isn't on
        //    the bus (keeps the test runnable on other rigs).
        const usb = devices.filter(d => d.includes('USB3'));
        const targetDev = devices.find(d => d.includes('00DA5328883'))
                       || (usb.length ? usb[0] : devices[0]);
        console.log('  connecting to:', targetDev);
        const key = 'USB:' + targetDev.replace(/^.*\(([^)]+)\).*$/, '$1');
        await h.sendCmd('cam0', { command: 'connect', device_key: key });
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
        // 50 ms exposure — safe under 5 fps (200 ms ceiling) on the 20 MP
        // CE200 and on the smaller USB cams too.
        await h.sendCmd('cam0', { command: 'set_exposure',   value: 50000 });
        await h.sendCmd('cam0', { command: 'set_gain',       value: 5.0 });
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

        // ---------------------------------------------------------------
        // New UI features added in the schema-driven / color / ROI pass.
        // ---------------------------------------------------------------

        // 10. Schema round-trip
        const schemaRsp = await h.sendCmd('cam0', { command: 'get_schema' });
        const schema = typeof schemaRsp.data === 'string'
            ? JSON.parse(schemaRsp.data) : schemaRsp.data;
        h.expect(Array.isArray(schema && schema.features),
                 'get_schema returns a features array');
        const byName = {};
        for (const f of (schema.features || [])) byName[f.name] = f;
        console.log('  schema features by group:');
        for (const g of (schema.groups || [])) {
            const inG = (schema.features || []).filter(f => f.group === g).map(f => f.name);
            console.log('    ' + g + ':', inG.join(', '));
        }
        // Every connected cam should at least expose Width / Height / Exposure.
        h.expect(!!byName.Width && !!byName.Height && !!byName.ExposureTime,
                 'schema has Width / Height / ExposureTime');
        h.shot('10_schema_fetched');

        // 11. Generic set_feature — apply a benign value, read it back.
        // Use GammaEnable if present (color cam) else ExposureTime (every cam).
        if (byName.GammaEnable) {
            const prev = byName.GammaEnable.cur;
            const target = !prev;
            const r = await h.sendCmd('cam0', {
                command: 'set_feature', name: 'GammaEnable', type: 'bool', value: target,
            });
            const d = typeof r.data === 'string' ? JSON.parse(r.data) : r.data;
            h.expect(d && d.ok && d.cur === target,
                     `GammaEnable toggled to ${target} (got cur=${d && d.cur})`);
            console.log('  GammaEnable: ' + prev + ' → ' + d.cur);
        } else {
            // Mono cam — exercise set_feature on an always-present numeric.
            const cur = byName.ExposureTime.cur || 10000;
            const target = Math.max(byName.ExposureTime.min || 15,
                                    Math.min(cur * 0.9, byName.ExposureTime.max || 1e6));
            const r = await h.sendCmd('cam0', {
                command: 'set_feature', name: 'ExposureTime', type: 'float', value: target,
            });
            const d = typeof r.data === 'string' ? JSON.parse(r.data) : r.data;
            h.expect(d && d.ok && Math.abs(d.cur - target) < 1.0,
                     `ExposureTime set to ${target} (got cur=${d && d.cur})`);
            console.log('  ExposureTime: ' + cur + ' → ' + d.cur);
        }
        h.shot('11_set_feature');

        // 12. Reset ROI → full-res.
        const rr = await h.sendCmd('cam0', { command: 'reset_roi' });
        const rrData = typeof rr.data === 'string' ? JSON.parse(rr.data) : rr.data;
        h.expect(rrData && rrData.ok,
                 `reset_roi ok (w=${rrData && rrData.w} h=${rrData && rrData.h})`);
        console.log('  reset_roi → ' + rrData.w + 'x' + rrData.h);
        await h.sleep(600);
        h.shot('12_reset_roi_applied');

        // 13. Set a small centered ROI and verify readback.
        const nativeW = byName.Width ? byName.Width.max : 0;
        const nativeH = byName.Height ? byName.Height.max : 0;
        const wInc = (byName.Width && byName.Width.inc) || 8;
        const hInc = (byName.Height && byName.Height.inc) || 2;
        const roundTo = (v, inc) => Math.floor(v / inc) * inc;
        const roiW = Math.max(640, roundTo(nativeW * 0.4, wInc));
        const roiH = Math.max(480, roundTo(nativeH * 0.4, hInc));
        const roiX = roundTo((nativeW - roiW) / 2, wInc);
        const roiY = roundTo((nativeH - roiH) / 2, hInc);
        await h.sendCmd('cam0', { command: 'set_roi', x: roiX, y: roiY, w: roiW, h: roiH });
        await h.sleep(800);
        const s = await h.sendCmd('cam0', { command: 'get_schema' });
        const schema2 = typeof s.data === 'string' ? JSON.parse(s.data) : s.data;
        const W2 = (schema2.features || []).find(f => f.name === 'Width');
        const H2 = (schema2.features || []).find(f => f.name === 'Height');
        const X2 = (schema2.features || []).find(f => f.name === 'OffsetX');
        const Y2 = (schema2.features || []).find(f => f.name === 'OffsetY');
        console.log('  ROI readback: '
                    + (W2 && W2.cur) + 'x' + (H2 && H2.cur)
                    + ' @ (' + (X2 && X2.cur) + ',' + (Y2 && Y2.cur) + ')'
                    + ' (requested ' + roiW + 'x' + roiH + ' @ ' + roiX + ',' + roiY + ')');
        // Camera snaps to increments, so allow a small delta on all four.
        h.expect(W2 && Math.abs(W2.cur - roiW) < 32,
                 `Width snapped to ~${roiW} (got ${W2 && W2.cur})`);
        h.expect(H2 && Math.abs(H2.cur - roiH) < 32,
                 `Height snapped to ~${roiH} (got ${H2 && H2.cur})`);
        h.expect(X2 && Math.abs(X2.cur - roiX) < 32,
                 `OffsetX snapped to ~${roiX} (got ${X2 && X2.cur})`);
        h.expect(Y2 && Math.abs(Y2.cur - roiY) < 32,
                 `OffsetY snapped to ~${roiY} (got ${Y2 && Y2.cur})`);
        h.shot('13_custom_roi_applied');

        // 14. Final clean up
        await h.sendCmd('cam0', { command: 'disconnect' });
        await h.sleep(400);
        h.shot('14_disconnected');
    },
};
