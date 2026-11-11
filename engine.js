/* CricketRadar engine - ball detection, tracking and release-speed physics.
 * Pure JavaScript, no dependencies. Runs in the browser and in Node (for tests).
 *
 * Conventions: world X = metres down the pitch from the bowler's middle stump,
 * Y = sideways, Z = height. Times are in seconds. Speeds in m/s internally.
 */
(function (root) {
  'use strict';

  // ---------------------------------------------------------------- constants
  const PITCH = 20.12;                 // Laws of Cricket: 22 yd between middle stumps
  const STUMP_H = 0.711;               // 28 in
  const BALL_MASS = 0.1595;            // men's ball, 155.9-163 g
  const BALL_D = 0.2265 / Math.PI;     // 224-229 mm circumference
  const G = 9.81;
  const MIN_V = 12, MAX_V = 50;        // 43-180 km/h

  // Phone camera profiles. hfovDeg = horizontal field of view of the main
  // camera in 16:9 video; readoutFrac = rolling-shutter readout time as a
  // fraction of the frame period.
  const PHONES = {
    'huawei-y9-prime-2019': {
      name: 'Huawei Y9 Prime (2019)',
      mainCamera: '16 MP, f/1.8, PDAF',
      video: '1080p at 30 fps (rear and front)',
      hfovDeg: 65,       // estimate: ~27 mm-equivalent lens, full sensor width used for 16:9 video
      readoutFrac: 0.75, // estimate for a budget CMOS sensor: ~25 ms per 1080p frame at 30 fps
      slowmoFactors: [4, 8],
      notes: 'No electronic stabilisation, so the video is not cropped or warped. ' +
             'Use the main 1x lens; the 0.6x ultra-wide (13 mm, ~120 deg) bends straight lines.'
    },
    generic: {
      name: 'Other phone',
      mainCamera: '', video: '', hfovDeg: 68, readoutFrac: 0.75,
      slowmoFactors: [4, 8, 2, 16, 32], notes: ''
    }
  };

  function dragK(cd, rho) {
    return rho * cd * Math.PI * (BALL_D / 2) ** 2 / (2 * BALL_MASS);
  }

  // ---------------------------------------------------------------- linear algebra
  function solve(A, b) {             // Gaussian elimination, partial pivoting
    const n = b.length, M = A.map((r, i) => r.concat([b[i]]));
    for (let c = 0; c < n; c++) {
      let p = c;
      for (let r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[p][c])) p = r;
      if (Math.abs(M[p][c]) < 1e-14) return null;
      [M[c], M[p]] = [M[p], M[c]];
      for (let r = c + 1; r < n; r++) {
        const f = M[r][c] / M[c][c];
        for (let k = c; k <= n; k++) M[r][k] -= f * M[c][k];
      }
    }
    const x = new Array(n).fill(0);
    for (let r = n - 1; r >= 0; r--) {
      let s = M[r][n];
      for (let k = r + 1; k < n; k++) s -= M[r][k] * x[k];
      x[r] = s / M[r][r];
    }
    return x;
  }
  const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  const norm = a => Math.hypot(a[0], a[1], a[2]);
  const scale = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
  const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
  const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
  const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
  const normalize = a => scale(a, 1 / norm(a));

  function homography(src, dst) {     // maps src[i] -> dst[i], 4 point pairs
    const A = [], b = [];
    for (let i = 0; i < 4; i++) {
      const [x, y] = src[i], [u, v] = dst[i];
      A.push([x, y, 1, 0, 0, 0, -u * x, -u * y]); b.push(u);
      A.push([0, 0, 0, x, y, 1, -v * x, -v * y]); b.push(v);
    }
    const h = solve(A, b);
    if (!h) return null;
    return [[h[0], h[1], h[2]], [h[3], h[4], h[5]], [h[6], h[7], 1]];
  }

  function polyfit(x, y, deg) {       // least squares, returns coeffs high->low in original x
    const n = x.length; let mx = 0;
    for (const v of x) mx += v / n;
    const xs = x.map(v => v - mx), d = deg + 1;
    const A = Array.from({ length: d }, () => new Array(d).fill(0)), b = new Array(d).fill(0);
    for (let i = 0; i < n; i++) {
      const pw = [1]; for (let k = 1; k < 2 * d; k++) pw.push(pw[k - 1] * xs[i]);
      for (let r = 0; r < d; r++) { b[r] += pw[r] * y[i]; for (let c = 0; c < d; c++) A[r][c] += pw[r + c]; }
    }
    const cLow = solve(A, b);       // coefficients in centred x, low->high
    if (!cLow) return null;
    // expand back to original x: sum c_k (x - mx)^k
    const out = new Array(d).fill(0); // low->high
    for (let k = 0; k < d; k++) {
      let binom = 1;
      for (let j = 0; j <= k; j++) {
        out[j] += cLow[k] * binom * Math.pow(-mx, k - j);
        binom = binom * (k - j) / (j + 1);
      }
    }
    return out.reverse();           // high->low like numpy
  }
  const polyval = (c, x) => c.reduce((s, v) => s * x + v, 0);
  const polyder = c => c.slice(0, -1).map((v, i) => v * (c.length - 1 - i));
  const median = a => { const s = [...a].sort((p, q) => p - q), m = s.length >> 1; return s.length % 2 ? s[m] : (s[m - 1] + s[m]) / 2; };

  // ---------------------------------------------------------------- calibration
  // Four points, in this order: bowler's-end middle stump base, its top,
  // batter's-end middle stump base, its top.
  const WORLD_XZ = [[0, 0], [0, STUMP_H], [PITCH, 0], [PITCH, STUMP_H]];

  function cameraFromCalibration(imgPts, frameW, frameH, hfovDeg) {
    const H = homography(WORLD_XZ, imgPts);
    if (!H) throw new Error('The four points are in a line or on top of each other. Mark them again.');
    const cx = frameW / 2, cy = frameH / 2;
    const f0 = (frameW / 2) / Math.tan(hfovDeg * Math.PI / 360);

    function poseFor(f) {
      const col = j => [(H[0][j] - cx * H[2][j]) / f, (H[1][j] - cy * H[2][j]) / f, H[2][j]];
      const a0 = col(0), a1 = col(1), a2 = col(2);
      let lam = 2 / (norm(a0) + norm(a1));
      if (a2[2] * lam < 0) lam = -lam;
      const rX = normalize(scale(a0, lam));
      let rZ = scale(a1, lam);
      rZ = normalize(sub(rZ, scale(rX, dot(rX, rZ))));
      const rY = cross(rZ, rX);
      const R = [[rX[0], rY[0], rZ[0]], [rX[1], rY[1], rZ[1]], [rX[2], rY[2], rZ[2]]];
      const t = scale(a2, lam);
      const cam = { f, cx, cy, R, t, W: frameW, H: frameH };
      let ss = 0;
      for (let i = 0; i < 4; i++) {
        const p = projectWorld(cam, [WORLD_XZ[i][0], 0, WORLD_XZ[i][1]]);
        ss += (p[0] - imgPts[i][0]) ** 2 + (p[1] - imgPts[i][1]) ** 2;
      }
      cam.rms = Math.sqrt(ss / 4);
      return cam;
    }
    // Start from the homography decomposition, then fit a rigid camera
    // (focal length + pose) to the 4 clicks by least squares. A side-on view
    // barely constrains the focal length, so the phone's lens acts as a prior.
    const init = poseFor(f0);
    const p0 = [0, ...rotToVec(init.R), ...init.t];
    const resid = p => {
      const cam = camFromParams(p, f0, cx, cy, frameW, frameH), r = [];
      for (let i = 0; i < 4; i++) {
        const q = projectWorld(cam, [WORLD_XZ[i][0], 0, WORLD_XZ[i][1]]);
        r.push(q[0] - imgPts[i][0], q[1] - imgPts[i][1]);
      }
      r.push(p[0] / 0.15 * 0.5);   // focal-length prior: ~15 % (1 sigma) costs 0.5 px
      return r;
    };
    const p = levenbergMarquardt(resid, p0);
    const cam = camFromParams(p, f0, cx, cy, frameW, frameH);
    const r = resid(p).slice(0, 8);
    cam.rms = Math.sqrt(r.reduce((s, v) => s + v * v, 0) / 4);
    cam.C = scale(matTvec(cam.R, cam.t), -1);   // camera centre in world coords
    cam.hfovUsed = 2 * Math.atan((frameW / 2) / cam.f) * 180 / Math.PI;
    return cam;
  }
  function vecToRot(w) {             // Rodrigues
    const th = norm(w);
    if (th < 1e-12) return [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
    const [x, y, z] = scale(w, 1 / th), c = Math.cos(th), s = Math.sin(th), C = 1 - c;
    return [[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
            [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
            [z * x * C - y * s, z * y * C + x * s, c + z * z * C]];
  }
  function rotToVec(R) {
    const tr = R[0][0] + R[1][1] + R[2][2];
    const th = Math.acos(Math.max(-1, Math.min(1, (tr - 1) / 2)));
    if (th < 1e-9) return [0, 0, 0];
    const v = [R[2][1] - R[1][2], R[0][2] - R[2][0], R[1][0] - R[0][1]];
    if (Math.PI - th < 1e-4) {           // near 180 deg: use the diagonal
      const ax = normalize([Math.sqrt(Math.max(0, (R[0][0] + 1) / 2)), Math.sign(v[0] || 1) * Math.sqrt(Math.max(0, (R[1][1] + 1) / 2)),
        Math.sign(v[1] || 1) * Math.sqrt(Math.max(0, (R[2][2] + 1) / 2))]);
      return scale(ax, th);
    }
    return scale(v, th / (2 * Math.sin(th)));
  }
  function camFromParams(p, f0, cx, cy, W, H) {
    return { f: f0 * Math.exp(p[0]), cx, cy, R: vecToRot([p[1], p[2], p[3]]), t: [p[4], p[5], p[6]], W, H };
  }
  function levenbergMarquardt(F, p0, iters = 200) {
    let p = p0.slice(), r = F(p), cost = r.reduce((s, v) => s + v * v, 0), lam = 1e-3;
    const n = p.length;
    for (let it = 0; it < iters; it++) {
      const J = [];
      for (let j = 0; j < n; j++) {
        const h = 1e-6 * Math.max(1, Math.abs(p[j]));
        const pp = p.slice(); pp[j] += h; const rp = F(pp);
        const pm = p.slice(); pm[j] -= h; const rm = F(pm);
        J.push(rp.map((v, i) => (v - rm[i]) / (2 * h)));
      }
      const A = Array.from({ length: n }, (_, a) => Array.from({ length: n }, (_, b) => J[a].reduce((s, v, i) => s + v * J[b][i], 0)));
      const g = J.map(col => col.reduce((s, v, i) => s + v * r[i], 0));
      let improved = false;
      for (let tries = 0; tries < 10; tries++) {
        const Ad = A.map((row, a) => row.map((v, b) => a === b ? v * (1 + lam) + 1e-12 : v));
        const d = solve(Ad, g.map(v => -v));
        if (!d) { lam *= 10; continue; }
        const pn = p.map((v, i) => v + d[i]), rn = F(pn), cn = rn.reduce((s, v) => s + v * v, 0);
        if (cn < cost) {
          const rel = (cost - cn) / Math.max(cost, 1e-12);
          p = pn; r = rn; cost = cn; lam = Math.max(lam / 10, 1e-9); improved = true;
          if (rel < 1e-12) return p;
          break;
        }
        lam *= 10;
      }
      if (!improved) break;
    }
    return p;
  }
  const matvec = (R, v) => [dot(R[0], v), dot(R[1], v), dot(R[2], v)];
  const matTvec = (R, v) => [R[0][0] * v[0] + R[1][0] * v[1] + R[2][0] * v[2],
                             R[0][1] * v[0] + R[1][1] * v[1] + R[2][1] * v[2],
                             R[0][2] * v[0] + R[1][2] * v[1] + R[2][2] * v[2]];
  function projectWorld(cam, P) {
    const c = add(matvec(cam.R, P), cam.t);
    return [cam.f * c[0] / c[2] + cam.cx, cam.f * c[1] / c[2] + cam.cy];
  }

  // The vertical plane the ball travels in: from the release point (beside the
  // stumps by `releaseOffset` m, + = towards the camera) to the batter's middle stump.
  function makeMapper(cam, releaseOffset = 0, releaseX = 0.9) {
    const side = cam.C[1] >= 0 ? 1 : -1;
    const y0 = side * releaseOffset, x0 = releaseX;
    const slope = (0 - y0) / (PITCH - x0);
    const a = [x0, y0, 0];
    const d = normalize([1, slope, 0]);
    const n = [-d[1], d[0], 0];
    const camDist = Math.abs(dot(n, sub(cam.C, a)));
    return {
      cam, slope, camDist,
      toWorld(u, v) {
        const ray = matTvec(cam.R, [(u - cam.cx) / cam.f, (v - cam.cy) / cam.f, 1]);
        const den = dot(ray, n);
        if (Math.abs(den) < 1e-9) return null;
        const s = dot(n, sub(a, cam.C)) / den;
        if (s <= 0) return null;
        const P = add(cam.C, scale(ray, s));
        return [P[0], P[2]];
      },
      toImage(X, Z) { return projectWorld(cam, [X, y0 + (X - x0) * slope, Z]); }
    };
  }

  // Bounding box (in video pixels) of the corridor above the pitch: only this
  // part of each frame needs to be searched for the ball.
  function corridorBox(mapper, frameW, frameH) {
    let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity;
    for (const X of [-3, PITCH / 2, PITCH + 3]) for (const Z of [-0.4, 3.5]) {
      const [u, v] = mapper.toImage(X, Z);
      x0 = Math.min(x0, u); x1 = Math.max(x1, u); y0 = Math.min(y0, v); y1 = Math.max(y1, v);
    }
    const pad = 0.01 * frameW;
    x0 = Math.max(0, Math.floor(x0 - pad)); y0 = Math.max(0, Math.floor(y0 - pad));
    x1 = Math.min(frameW, Math.ceil(x1 + pad)); y1 = Math.min(frameH, Math.ceil(y1 + pad));
    return { x: x0, y: y0, w: Math.max(1, x1 - x0), h: Math.max(1, y1 - y0) };
  }

  // ---------------------------------------------------------------- detection
  // Three-frame differencing on a region of interest. push() frames in order;
  // each call returns candidates for the previous frame.
  class MotionDetector {
    constructor(opts) {
      // opts: roiW, roiH (analysis pixels), toVideo(x,y)->[u,v], videoH, mapper,
      //       threshold, minArea, maxArea
      Object.assign(this, { threshold: 18, minArea: 2, maxArea: 400, maxPerFrame: 60 }, opts);
      this.frames = []; this.meta = [];
      const n = this.roiW * this.roiH;
      this.mask = new Uint8Array(n); this.label = new Int32Array(n); this.stack = new Int32Array(n);
    }
    push(rgba, frameIndex, tFrame) {
      this.frames.push(rgba); this.meta.push({ frameIndex, tFrame });
      if (this.frames.length > 3) { this.frames.shift(); this.meta.shift(); }
      if (this.frames.length < 3) return [];
      return this._detect();
    }
    _detect() {
      const [p, c, nx] = this.frames, W = this.roiW, Hh = this.roiH, thr = this.threshold;
      const mask = this.mask;
      for (let i = 0, j = 0; i < W * Hh; i++, j += 4) {
        const d1 = Math.max(Math.abs(c[j] - p[j]), Math.abs(c[j + 1] - p[j + 1]), Math.abs(c[j + 2] - p[j + 2]));
        if (d1 < thr) { mask[i] = 0; continue; }
        const d2 = Math.max(Math.abs(c[j] - nx[j]), Math.abs(c[j + 1] - nx[j + 1]), Math.abs(c[j + 2] - nx[j + 2]));
        mask[i] = d2 >= thr ? 1 : 0;
      }
      // connected components (8-neighbour) via flood fill
      const label = this.label, stack = this.stack; label.fill(0);
      const out = []; let nl = 0;
      for (let i = 0; i < W * Hh; i++) {
        if (!mask[i] || label[i]) continue;
        nl++; let sp = 0; stack[sp++] = i; label[i] = nl;
        let area = 0, sx = 0, sy = 0, tooBig = false;
        while (sp) {
          const q = stack[--sp], qx = q % W, qy = (q / W) | 0;
          area++; sx += qx; sy += qy;
          for (let dy = -1; dy <= 1; dy++) {
            const yy = qy + dy; if (yy < 0 || yy >= Hh) continue;
            for (let dx = -1; dx <= 1; dx++) {
              const xx = qx + dx; if (xx < 0 || xx >= W) continue;
              const r = yy * W + xx;
              if (mask[r] && !label[r]) { label[r] = nl; stack[sp++] = r; }
            }
          }
          if (area > this.maxArea * 4) tooBig = true;
        }
        if (tooBig || area < this.minArea || area > this.maxArea) continue;
        out.push({ x: sx / area + 0.5, y: sy / area + 0.5, area });
      }
      if (out.length > this.maxPerFrame) { out.sort((a, b) => a.area - b.area); out.length = this.maxPerFrame; }
      const m = this.meta[1], cands = [];
      for (const o of out) {
        const [u, v] = this.toVideo(o.x, o.y);
        const w = this.mapper.toWorld(u, v);
        if (!w) continue;
        const [X, Z] = w;
        if (X < -3 || X > PITCH + 3 || Z < -0.4 || Z > 3.5) continue;
        cands.push({ frame: m.frameIndex, tFrame: m.tFrame, rowFrac: v / this.videoH, u, v, X, Z, area: o.area });
      }
      return cands;
    }
  }

  // ---------------------------------------------------------------- physics helpers
  const xDrag = (t, a, u, t1, k) => a + Math.log1p(k * u * (t - t1)) / k;

  function fitDrag(t, X, k, t1, a0, u0) {   // Gauss-Newton for (a, u)
    let a = a0, u = u0;
    const n = t.length;
    for (let it = 0; it < 50; it++) {
      let A00 = 0, A01 = 0, A11 = 0, b0 = 0, b1 = 0;
      for (let i = 0; i < n; i++) {
        const q = 1 + k * u * (t[i] - t1);
        const r = X[i] - (a + Math.log(q) / k), j1 = (t[i] - t1) / q;
        A00 += 1; A01 += j1; A11 += j1 * j1; b0 += r; b1 += j1 * r;
      }
      const det = A00 * A11 - A01 * A01;
      if (Math.abs(det) < 1e-18) break;
      let da = (A11 * b0 - A01 * b1) / det, du = (A00 * b1 - A01 * b0) / det;
      let step = 1;
      while (step > 1e-4) {
        const un = u + step * du;
        let ok = un > 0.5;
        for (let i = 0; ok && i < n; i++) if (1 + k * un * (t[i] - t1) <= 0) ok = false;
        if (ok) break;
        step /= 2;
      }
      a += step * da; u += step * du;
      if (Math.abs(du * step) < 1e-8 && Math.abs(da * step) < 1e-9) break;
    }
    let ss = 0, A00 = 0, A01 = 0, A11 = 0;
    for (let i = 0; i < n; i++) {
      const q = 1 + k * u * (t[i] - t1), j1 = (t[i] - t1) / q;
      ss += (X[i] - (a + Math.log(q) / k)) ** 2; A00 += 1; A01 += j1; A11 += j1 * j1;
    }
    const s2 = ss / Math.max(1, n - 2), det = A00 * A11 - A01 * A01;
    const cov = [[s2 * A11 / det, -s2 * A01 / det], [-s2 * A01 / det, s2 * A00 / det]];
    return { a, u, cov, resid: t.map((ti, i) => X[i] - xDrag(ti, a, u, t1, k)) };
  }

  // ---------------------------------------------------------------- tracking
  function bestInFrame(cs, Xp, Zp, tolX, tolZ) {
    let best = null, eb = Infinity;
    for (const c of cs) {
      const ex = Math.abs(c.X - Xp), ez = Zp == null ? 0 : Math.abs(c.Z - Zp);
      if (ex <= tolX && ez <= tolZ) { const e = ex + 0.3 * ez; if (e < eb) { eb = e; best = c; } }
    }
    return best;
  }

  function findTrack(cands, fps, k) {
    const byFrame = new Map();
    for (const c of cands) { if (!byFrame.has(c.frame)) byFrame.set(c.frame, []); byFrame.get(c.frame).push(c); }
    if (!byFrame.size) return [];
    const frames = [...byFrame.keys()].sort((a, b) => a - b);
    const fMin = frames[0], fMax = frames[frames.length - 1];
    const gap = Math.max(2, Math.round(0.03 * fps)), half = Math.max(4, Math.round(0.12 * fps));

    let bestScore = 0, bestSeed = null;
    for (const fi of frames) {
      const fj = fi + gap; if (!byFrame.has(fj)) continue;
      for (const c1 of byFrame.get(fi)) for (const c2 of byFrame.get(fj)) {
        const dt = c2.t - c1.t; if (dt <= 0) continue;
        const v = (c2.X - c1.X) / dt, vz = (c2.Z - c1.Z) / dt;
        if (v < MIN_V || v > MAX_V || Math.abs(vz) > 12) continue;
        const inl = [];
        for (let f = fi - half; f <= fi + half; f++) {
          const cs = byFrame.get(f); if (!cs) continue;
          const tt = cs[0].t - c1.t;
          const c = bestInFrame(cs, c1.X + v * tt, c1.Z + vz * tt,
            0.15 + 0.06 * Math.abs(v * tt), 0.35 + 0.5 * G * tt * tt + 0.1 * Math.abs(vz * tt));
          if (c) inl.push(c);
        }
        if (inl.length <= bestScore || inl.length < 4) continue;
        const vf = polyfit(inl.map(c => c.t), inl.map(c => c.X), 1);
        if (vf && vf[0] >= MIN_V && vf[0] <= MAX_V) { bestScore = inl.length; bestSeed = inl; }
      }
    }
    if (!bestSeed) return [];

    let track = bestSeed;
    for (let iter = 0; iter < 60; iter++) {
      const t = track.map(c => c.t), X = track.map(c => c.X), Z = track.map(c => c.Z);
      const t1 = t[0];
      const lin = polyfit(t, X, 1);
      const fit = fitDrag(t, X, k, t1, X[0], Math.max(MIN_V, lin[0]));
      const { a, u } = fit;
      const nLo = Math.min(t.length, 30);
      const zLo = polyfit(t.slice(0, nLo), Z.slice(0, nLo), t.length >= 6 ? 2 : 1);
      const zHi = polyfit(t.slice(-nLo), Z.slice(-nLo), 1);
      const fFirst = track[0].frame, fLast = track[track.length - 1].frame;
      const span = Math.max(3, Math.round(0.08 * fps));
      const add = [];
      for (let f = fFirst - 1; f >= Math.max(fMin, fFirst - span); f--) {
        const cs = byFrame.get(f); if (!cs) continue;
        const tt = cs[0].t;
        if (k * u * (tt - t1) <= -0.9) break;
        const xp = xDrag(tt, a, u, t1, k); if (xp < -1.5) break;
        const c = bestInFrame(cs, xp, polyval(zLo, tt), 0.25, 0.4);
        if (c) add.push(c);
      }
      const vEnd = u * Math.exp(-k * (X[X.length - 1] - a));
      let lastX = X[X.length - 1];
      for (let f = fLast + 1; f <= Math.min(fMax, fLast + span); f++) {
        const cs = byFrame.get(f); if (!cs) continue;
        const tt = cs[0].t, dtt = tt - t[t.length - 1];
        const xp = X[X.length - 1] + vEnd * dtt; if (xp > PITCH + 2.5) break;
        const c = bestInFrame(cs, xp - 0.1 * vEnd * dtt, null, 0.25 + 0.2 * vEnd * dtt, 9);
        if (c && c.X > lastX - 0.05 && Math.abs(c.Z - polyval(zHi, tt)) < 0.5 + 6 * dtt) { add.push(c); lastX = c.X; }
      }
      if (!add.length) break;
      const m = new Map(track.map(c => [c.frame, c]));
      for (const c of add) if (!m.has(c.frame)) m.set(c.frame, c);
      track = [...m.values()].sort((p, q) => p.frame - q.frame);
    }
    return track.length >= 5 ? track : [];
  }

  function removeOutliers(track, fps) {
    const t = track.map(c => c.t), X = track.map(c => c.X), Z = track.map(c => c.Z);
    const w = Math.max(3, Math.round(0.03 * fps)), keep = [];
    for (let i = 0; i < track.length; i++) {
      const nb = [];
      for (let j = Math.max(0, i - w); j < Math.min(track.length, i + w + 1); j++) if (j !== i) nb.push(j);
      if (nb.length < 3) { keep.push(track[i]); continue; }
      const cx = polyfit(nb.map(j => t[j]), nb.map(j => X[j]), 1), cz = polyfit(nb.map(j => t[j]), nb.map(j => Z[j]), 1);
      if (Math.abs(polyval(cx, t[i]) - X[i]) < 0.2 && Math.abs(polyval(cz, t[i]) - Z[i]) < 0.3) keep.push(track[i]);
    }
    return keep;
  }

  function findBounce(t, Z) {
    const n = t.length; let best = null, bestR = Infinity;
    const ssr = (c, xs, ys) => xs.reduce((s, x, i) => s + (polyval(c, x) - ys[i]) ** 2, 0);
    for (let b = 5; b < n - 2; b++) {
      const pt = t.slice(0, b), pz = Z.slice(0, b), qt = t.slice(b), qz = Z.slice(b);
      const c1 = polyfit(pt, pz, 2), c2 = polyfit(qt, qz, qt.length >= 4 ? 2 : 1);
      if (!c1 || !c2) continue;
      const r = ssr(c1, pt, pz) + ssr(c2, qt, qz);
      if (polyval(polyder(c1), pt[pt.length - 1]) < 0 && polyval(polyder(c2), qt[0]) > 0 && r < bestR) { best = b; bestR = r; }
    }
    const call = polyfit(t, Z, 2), rAll = call ? ssr(call, t, Z) : Infinity;
    if (best == null || rAll < 1.5 * bestR + 1e-6) return null;
    return best;
  }

  function robustDragFit(tp, Xp, k) {
    let mask = tp.map(() => true), res;
    for (let it = 0; it < 4; it++) {
      const ti = tp.filter((_, i) => mask[i]), xi = Xp.filter((_, i) => mask[i]);
      const t1 = ti[0];
      const fit = fitDrag(ti, xi, k, t1, xi[0], (xi[xi.length - 1] - xi[0]) / (ti[ti.length - 1] - ti[0]));
      res = { ...fit, t1, mask };
      const r = tp.map((t, i) => Xp[i] - xDrag(t, fit.a, fit.u, t1, k));
      const rm = r.filter((_, i) => mask[i]), med = median(rm);
      const mad = 1.4826 * median(rm.map(v => Math.abs(v - med)));
      const nm = r.map(v => Math.abs(v) <= Math.max(0.06, 3.5 * mad));
      if (nm.filter(Boolean).length < 4 || nm.every((v, i) => v === mask[i])) break;
      mask = nm;
    }
    return res;
  }

  // ---------------------------------------------------------------- speed
  function measureSpeed(trackIn, fps, k, opts) {
    const { releaseX = 0.9, mapper = null, lateralKnown = false } = opts || {};
    const track = removeOutliers(trackIn, fps);
    const t = track.map(c => c.t), X = track.map(c => c.X), Z = track.map(c => c.Z);
    const b = findBounce(t, Z);
    const nPre = b == null ? t.length : b;
    let tp = t.slice(0, nPre), Xp = X.slice(0, nPre), Zp = Z.slice(0, nPre);
    if (tp.length < 4) throw new Error('Too few ball positions before the bounce. Film in slow-mo, or move closer.');
    const fit = robustDragFit(tp, Xp, k);
    const { a, u, t1, cov, mask } = fit;
    tp = tp.filter((_, i) => mask[i]); Xp = Xp.filter((_, i) => mask[i]); Zp = Zp.filter((_, i) => mask[i]);
    const used = track.filter((_, i) => i >= nPre || mask[i]);

    const relX = Math.min(releaseX, a);
    const vxRel = u * Math.exp(-k * (relX - a));
    const tRel = t1 + (Math.exp(k * (relX - a)) - 1) / (k * u);
    // vertical: Z = z1 + w (t - t1) - g/2 (t - t1)^2
    const tt = tp.map(v => v - t1);
    const lw = polyfit(tt, Zp.map((z, i) => z + 0.5 * G * tt[i] ** 2), 1);
    const w = lw[0], z1 = lw[1];
    const vzRel = w - G * (tRel - t1);
    const slope = mapper ? mapper.slope : 0;
    const vRel = Math.sqrt(vxRel ** 2 * (1 + slope ** 2) + vzRel ** 2);

    // gravity check: free quadratic in time; its curvature should equal g/2
    const q = polyfit(tt, Zp, 2);
    const gRatio = q ? -2 * q[0] / G : NaN;

    // error budget
    const J = [vxRel * k, Math.exp(-k * (relX - a))];
    const varFit = J[0] * (J[0] * cov[0][0] + J[1] * cov[1][0]) + J[1] * (J[0] * cov[0][1] + J[1] * cov[1][1]);
    const sFit = Number.isFinite(varFit) && varFit > 0 ? Math.sqrt(varFit) : 0;
    const vWithK = kk => { const f2 = fitDrag(tp, Xp, kk, t1, a, u); return f2.u * Math.exp(-kk * (Math.min(releaseX, f2.a) - f2.a)); };
    const sCd = Math.abs(vWithK(k * (1 + 0.06 / 0.42)) - vWithK(k * (1 - 0.06 / 0.42))) / 2;
    let sLat = 0, sCal = 0;
    if (mapper) {
      const p0 = mapper.toImage(0, 0), p1 = mapper.toImage(PITCH, 0);
      const pitchPx = Math.hypot(p1[0] - p0[0], p1[1] - p0[1]);
      sLat = vRel * (lateralKnown ? 0.15 : 0.35) / mapper.camDist;
      sCal = vRel * 2 * Math.SQRT2 / pitchPx;
    }
    const sigma = Math.sqrt(sFit ** 2 + sCd ** 2 + sLat ** 2 + sCal ** 2);

    const res = {
      releaseKmh: vRel * 3.6, releaseMph: vRel * 2.236936, uncertaintyKmh: sigma * 3.6, calKmh: sCal * 3.6,
      firstSeenKmh: u * 3.6, bounceKmh: null, bounceX: null,
      pointsUsed: tp.length, pointsTotal: used.length, fps, gRatio, warnings: [],
      track: used, fit: { a, u, t1, z1, w, tRel, relX, k, bounceIndex: b }
    };
    if (b != null) {
      const tb = 0.5 * (t[b - 1] + t[b]);
      const xb = xDrag(tb, a, u, t1, k);
      res.bounceX = xb; res.bounceKmh = u * Math.exp(-k * (xb - a)) * 3.6;
    } else res.warnings.push('No bounce seen: full toss, or the ball left the picture before pitching.');
    if (fps < 50) res.warnings.push(`At ${Math.round(fps)} fps the ball moves ${(vRel / fps).toFixed(1)} m between frames, so this reading is rough. Slow-mo gives much better accuracy.`);
    if (tp.length < 8) res.warnings.push('Few ball positions before the bounce, so the reading is rough.');
    if (a - relX > 6) res.warnings.push("The ball was first seen far from the bowler's hand. Keep the bowler's end in the picture.");
    return res;
  }

  // ---------------------------------------------------------------- full analysis
  // cands: from MotionDetector (tFrame in container seconds). Tries the
  // possible slow-motion factors and keeps the one where the ball falls at g.
  function analyse(cands, opts) {
    const { containerFps, factors = [1], readoutFrac = 0.75, k, releaseX, mapper, lateralKnown } = opts;
    const tries = [];
    for (const f of factors) {
      const fps = containerFps * f;
      const cs = cands.map(c => ({ ...c, t: (c.tFrame + c.rowFrac * readoutFrac / containerFps) / f }));
      const track = findTrack(cs, fps, k);
      if (!track.length) { tries.push({ factor: f, fps, ok: false }); continue; }
      try {
        const res = measureSpeed(track, fps, k, { releaseX, mapper, lateralKnown });
        res.factor = f;
        // a delivery travels down the pitch: bowler's arm, fielders and the
        // umpire never cover 7 m in a straight, fast, falling line
        const tr = res.track, span = tr[tr.length - 1].X - tr[0].X;
        if (span < 7 || res.gRatio <= 0) { tries.push({ factor: f, fps, ok: false, err: 'not a delivery', res }); continue; }
        tries.push({ factor: f, fps, ok: true, res, score: Math.abs(Math.log(Math.max(1e-3, res.gRatio || 1e-3))) });
      } catch (e) { tries.push({ factor: f, fps, ok: false, err: e.message }); }
    }
    const ok = tries.filter(r => r.ok);
    if (!ok.length) return { ok: false, tries };
    ok.sort((p, q) => p.score - q.score);
    const best = ok[0].res;
    if (opts.calib) calibrationSpread(best, opts);
    if (factors.length > 1) {
      best.frameRateCheck = Number.isFinite(best.gRatio) && best.gRatio > 0.55 && best.gRatio < 1.8 ? 'confirmed' : 'unconfirmed';
      if (best.frameRateCheck === 'unconfirmed')
        best.warnings.push('Could not confirm the slow-motion speed from the ball\'s fall. Set it by hand in Settings if this reading looks wrong.');
    }
    return { ok: true, result: best, tries };
  }

  // How much would the reading move if each calibration tap were off by
  // ~sigmaPx? Re-map the tracked ball with jiggled taps and re-measure.
  function calibrationSpread(res, opts) {
    const { calib, k, releaseX, lateralKnown } = opts;
    const { pts, W, H, hfov, offset, sigmaPx = 1 } = calib;
    let seed = 12345;
    const gauss = () => {
      let u = 0, v = 0;
      while (!u) u = (seed = (seed * 16807) % 2147483647) / 2147483647;
      v = (seed = (seed * 16807) % 2147483647) / 2147483647;
      return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
    };
    const speeds = [];
    for (let i = 0; i < 12; i++) {
      try {
        const jp = pts.map(([x, y]) => [x + gauss() * sigmaPx, y + gauss() * sigmaPx]);
        const m = makeMapper(cameraFromCalibration(jp, W, H, hfov), offset, releaseX);
        const tr = res.track.map(c => { const w = m.toWorld(c.u, c.v); return w ? { ...c, X: w[0], Z: w[1] } : null; }).filter(Boolean);
        speeds.push(measureSpeed(tr, res.fps, k, { releaseX, mapper: m, lateralKnown }).releaseKmh);
      } catch (e) { /* skip a failed draw */ }
    }
    if (speeds.length >= 6) {
      const mu = speeds.reduce((a, b) => a + b, 0) / speeds.length;
      const sd = Math.sqrt(speeds.reduce((a, b) => a + (b - mu) ** 2, 0) / (speeds.length - 1));
      const other = res.uncertaintyKmh ** 2 - (res.calKmh || 0) ** 2;
      res.calKmh = sd;
      res.uncertaintyKmh = Math.sqrt(Math.max(0, other) + sd * sd);
    }
  }

  function lengthName(bounceX) {      // Hawk-Eye style zones, distance from batter's stumps
    if (bounceX == null) return 'Full toss';
    const d = PITCH - bounceX;
    if (d < 2) return 'Yorker'; if (d < 6) return 'Full'; if (d < 8) return 'Good length'; return 'Short';
  }

  const api = { PITCH, STUMP_H, BALL_D, G, PHONES, dragK, homography, cameraFromCalibration, makeMapper,
    corridorBox, MotionDetector, findTrack, measureSpeed, analyse, lengthName, polyfit, polyval, xDrag, projectWorld };
  if (typeof module !== 'undefined' && module.exports) module.exports = api; else root.RadarEngine = api;
})(typeof self !== 'undefined' ? self : this);
