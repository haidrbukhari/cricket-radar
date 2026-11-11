// node test_engine.js VIDEO TRUTH.json [--side near|far|centre] [--factors auto|1|4] [--readout 0.75] [--width 960]
// Decodes VIDEO with ffmpeg, runs the RadarEngine exactly as the app does, prints JSON.
const { spawn, execFileSync } = require('child_process');
const fs = require('fs');
const E = require('./engine.js');

const args = process.argv.slice(2);
const opt = (n, d) => { const i = args.indexOf('--' + n); return i >= 0 ? args[i + 1] : d; };
const video = args[0], truth = JSON.parse(fs.readFileSync(args[1], 'utf8'));
const side = opt('side', 'near'), factorsArg = opt('factors', 'auto');
const readoutFrac = parseFloat(opt('readout', '0.75'));
const phone = { ...E.PHONES['huawei-y9-prime-2019'] }; if (opt('hfov')) phone.hfovDeg = +opt('hfov'); const NOISE = +opt('noise', '1.5');

const probe = JSON.parse(execFileSync('ffprobe', ['-v', 'error', '-select_streams', 'v:0', '-show_entries',
  'stream=width,height,r_frame_rate', '-of', 'json', video]).toString()).streams[0];
const VW = probe.width, VH = probe.height;
const [fn, fd] = probe.r_frame_rate.split('/').map(Number), containerFps = fn / fd;

// calibration: exact points (in the truth's frame size) + ±1.5 px click error, rescaled to this video
let seed = +opt('seed', String([...video].reduce((h, ch) => (h * 31 + ch.charCodeAt(0)) % 100000, 7) + 1)); const rnd = () => ((seed = (seed * 16807) % 2147483647) / 2147483647) * 3 - 1.5;
const sx = VW / truth.frame_size[0], sy = VH / truth.frame_size[1];
const pts = truth.calib_points_exact.map(([x, y]) => [x * sx + rnd() * NOISE / 1.5 * Math.min(1, sx), y * sy + rnd() * NOISE / 1.5 * Math.min(1, sy)]);

const cam = E.cameraFromCalibration(pts, VW, VH, phone.hfovDeg);
const offset = { near: 0.35, far: -0.35, centre: 0 }[side];
const mapper = E.makeMapper(cam, offset, 0.9);
const AW = Math.min(960, VW), AH = Math.round(VH * AW / VW), s = VW / AW;
const box = E.corridorBox(mapper, VW, VH);
const roi = { x: Math.floor(box.x / s), y: Math.floor(box.y / s), w: Math.ceil(box.w / s), h: Math.ceil(box.h / s) };
roi.w = Math.min(roi.w, AW - roi.x); roi.h = Math.min(roi.h, AH - roi.y);
const ballPx = Math.max(2, E.BALL_D * cam.f / cam.C[1] / s);
const det = new E.MotionDetector({ roiW: roi.w, roiH: roi.h, videoH: VH, mapper,
  toVideo: (x, y) => [(x + roi.x) * s, (y + roi.y) * s], maxArea: Math.max(60, (ballPx * 6) ** 2) });

const t0 = Date.now();
const ff = spawn('ffmpeg', ['-v', 'error', '-i', video, '-vf', `scale=${AW}:${AH}`, '-f', 'rawvideo', '-pix_fmt', 'rgba', '-']);
const frameBytes = AW * AH * 4; let buf = Buffer.alloc(0), idx = 0; const cands = [];
ff.stdout.on('data', chunk => {
  buf = Buffer.concat([buf, chunk]);
  while (buf.length >= frameBytes) {
    const fr = buf.subarray(0, frameBytes);
    const crop = new Uint8ClampedArray(roi.w * roi.h * 4);
    for (let y = 0; y < roi.h; y++) {
      const o = ((y + roi.y) * AW + roi.x) * 4;
      crop.set(fr.subarray(o, o + roi.w * 4), y * roi.w * 4);
    }
    cands.push(...det.push(crop, idx, idx / containerFps));
    idx++; buf = buf.subarray(frameBytes);
  }
});
ff.on('close', () => {
  const tDet = Date.now() - t0;
  const factors = factorsArg === 'auto' ? (containerFps >= 90 ? [1] : [1, ...phone.slowmoFactors]) : [Number(factorsArg)];
  const out = E.analyse(cands, { containerFps, factors, readoutFrac, k: E.dragK(0.42, 1.18), releaseX: 0.9,
    mapper, lateralKnown: side !== 'centre',
    calib: { pts, W: VW, H: VH, hfov: phone.hfovDeg, offset, sigmaPx: 1.0 * VW / 1920 } });
  const r = out.ok ? out.result : null;
  console.log(JSON.stringify({
    video: video.split('/').pop(), true: truth.true_release_speed_kmh, size: `${VW}x${VH}`, containerFps,
    measured: r && +r.releaseKmh.toFixed(1), err: r && +(r.releaseKmh - truth.true_release_speed_kmh).toFixed(1),
    pm: r && +r.uncertaintyKmh.toFixed(1), cal: r && +r.calKmh.toFixed(1), factor: r && r.factor, gRatio: r && +r.gRatio.toFixed(2),
    check: r && r.frameRateCheck, bounceX: r && r.bounceX && +r.bounceX.toFixed(2), trueBounce: truth.bounce_x_m,
    pts: r && r.pointsUsed, hfovUsed: +cam.hfovUsed.toFixed(1), camDist: +mapper.camDist.toFixed(1), rms: +cam.rms.toFixed(2),
    frames: idx, cands: cands.length, detectMsPerFrame: +(tDet / idx).toFixed(1),
    tries: out.tries.map(t => `${t.factor}x:${t.ok ? (t.res.releaseKmh.toFixed(0) + '/g' + t.res.gRatio.toFixed(2)) : 'none'}`).join(' ')
  }));
});
