/* 문서 스캐너 - 이미지 처리 핵심 (원근 보정, 문서 자동 인식, 문서 필터) */
(function (root) {
  'use strict';

  /* 단위 정사각형 -> 사각형(q) 으로 가는 호모그래피. q = [TL, TR, BR, BL] */
  function homography(q) {
    var x0 = q[0][0], y0 = q[0][1], x1 = q[1][0], y1 = q[1][1];
    var x2 = q[2][0], y2 = q[2][1], x3 = q[3][0], y3 = q[3][1];
    var dx1 = x1 - x2, dx2 = x3 - x2, dx3 = x0 - x1 + x2 - x3;
    var dy1 = y1 - y2, dy2 = y3 - y2, dy3 = y0 - y1 + y2 - y3;
    var g = 0, h = 0;
    if (Math.abs(dx3) > 1e-9 || Math.abs(dy3) > 1e-9) {
      var den = dx1 * dy2 - dx2 * dy1;
      g = (dx3 * dy2 - dx2 * dy3) / den;
      h = (dx1 * dy3 - dx3 * dy1) / den;
    }
    return { a: x1 - x0 + g * x1, b: x3 - x0 + h * x3, c: x0,
             d: y1 - y0 + g * y1, e: y3 - y0 + h * y3, f: y0, g: g, h: h };
  }

  function dist(p, q) { return Math.hypot(p[0] - q[0], p[1] - q[1]); }

  /* 결과 이미지 크기 (긴 변이 maxLong 을 넘지 않게) */
  function outputSize(q, maxLong) {
    var w = Math.max(dist(q[0], q[1]), dist(q[3], q[2]));
    var h = Math.max(dist(q[0], q[3]), dist(q[1], q[2]));
    var s = Math.min(1, maxLong / Math.max(w, h));
    return [Math.max(1, Math.round(w * s)), Math.max(1, Math.round(h * s))];
  }

  /* 원근 보정: src({data,width,height}) 의 사각형 q 를 ow x oh 의 반듯한 이미지로 펴 준다 */
  function warp(src, q, ow, oh) {
    var H = homography(q), sw = src.width, sh = src.height, sd = src.data;
    var out = new Uint8ClampedArray(ow * oh * 4);
    var o = 0, maxX = sw - 1, maxY = sh - 1;
    for (var j = 0; j < oh; j++) {
      var v = (j + 0.5) / oh;
      for (var i = 0; i < ow; i++) {
        var u = (i + 0.5) / ow;
        var w = H.g * u + H.h * v + 1;
        var x = (H.a * u + H.b * v + H.c) / w - 0.5;
        var y = (H.d * u + H.e * v + H.f) / w - 0.5;
        if (x < 0) x = 0; else if (x > maxX) x = maxX;
        if (y < 0) y = 0; else if (y > maxY) y = maxY;
        var x0 = x | 0, y0 = y | 0;
        var x1 = x0 < maxX ? x0 + 1 : x0, y1 = y0 < maxY ? y0 + 1 : y0;
        var fx = x - x0, fy = y - y0;
        var p00 = (y0 * sw + x0) * 4, p10 = (y0 * sw + x1) * 4;
        var p01 = (y1 * sw + x0) * 4, p11 = (y1 * sw + x1) * 4;
        var w00 = (1 - fx) * (1 - fy), w10 = fx * (1 - fy), w01 = (1 - fx) * fy, w11 = fx * fy;
        out[o]     = sd[p00] * w00 + sd[p10] * w10 + sd[p01] * w01 + sd[p11] * w11;
        out[o + 1] = sd[p00 + 1] * w00 + sd[p10 + 1] * w10 + sd[p01 + 1] * w01 + sd[p11 + 1] * w11;
        out[o + 2] = sd[p00 + 2] * w00 + sd[p10 + 2] * w10 + sd[p01 + 2] * w01 + sd[p11 + 2] * w11;
        out[o + 3] = 255;
        o += 4;
      }
    }
    return { data: out, width: ow, height: oh };
  }

  function boxBlur(src, w, h, r) {
    var tmp = new Float32Array(w * h), out = new Float32Array(w * h), x, y, k, sum, cnt;
    for (y = 0; y < h; y++) {
      for (x = 0; x < w; x++) {
        sum = 0; cnt = 0;
        for (k = -r; k <= r; k++) { var xx = x + k; if (xx >= 0 && xx < w) { sum += src[y * w + xx]; cnt++; } }
        tmp[y * w + x] = sum / cnt;
      }
    }
    for (x = 0; x < w; x++) {
      for (y = 0; y < h; y++) {
        sum = 0; cnt = 0;
        for (k = -r; k <= r; k++) { var yy = y + k; if (yy >= 0 && yy < h) { sum += tmp[yy * w + x]; cnt++; } }
        out[y * w + x] = sum / cnt;
      }
    }
    return out;
  }

  /* ================= 문서 자동 인식 =================
     1) 작게 줄인 사진(긴 변 DETECT_SIZE)에서 경계선(엣지)을 찾고
     2) 허프 변환으로 직선 후보를 뽑은 뒤
     3) 직선 4개로 만들 수 있는 사각형마다 "네 변이 정말 종이 가장자리인가"
        (변을 따라 엣지가 이어지는지 + 안쪽이 바깥보다 밝은지)를 점수로 매겨 가장 좋은 것을 고른다.
     4) refineQuad 로 더 큰 사진(긴 변 REFINE_SIZE)에서 각 변을 1픽셀 단위로 다시 맞춘다. */
  var DETECT_SIZE = 640, REFINE_SIZE = 1600;

  function blur5(src, w, h) {                    // [1 4 6 4 1]/16 가우시안 (σ≈1)
    var tmp = new Float32Array(w * h), out = new Float32Array(w * h), x, y, r;
    for (y = 0; y < h; y++) {
      r = y * w;
      for (x = 0; x < w; x++) {
        var a = x > 1 ? x - 2 : 0, b = x > 0 ? x - 1 : 0, c = x < w - 1 ? x + 1 : w - 1, e = x < w - 2 ? x + 2 : w - 1;
        tmp[r + x] = (src[r + a] + 4 * src[r + b] + 6 * src[r + x] + 4 * src[r + c] + src[r + e]) * 0.0625;
      }
    }
    for (y = 0; y < h; y++) {
      var ya = (y > 1 ? y - 2 : 0) * w, yb = (y > 0 ? y - 1 : 0) * w, yc = y * w;
      var yd = (y < h - 1 ? y + 1 : h - 1) * w, ye = (y < h - 2 ? y + 2 : h - 1) * w;
      for (x = 0; x < w; x++) out[yc + x] = (tmp[ya + x] + 4 * tmp[yb + x] + 6 * tmp[yc + x] + 4 * tmp[yd + x] + tmp[ye + x]) * 0.0625;
    }
    return out;
  }

  function angDiff(a, b) { var d = Math.abs(a - b) % 180; return d > 90 ? 180 - d : d; }

  /* 밝기 + 색 채널의 기울기로 엣지(가는 선) 지도를 만든다 */
  function edgeMap(img) {
    var w = img.width, h = img.height, d = img.data, n = w * h, i, x, y;
    var L = new Float32Array(n), C1 = new Float32Array(n), C2 = new Float32Array(n);
    for (i = 0; i < n; i++) {
      var r = d[4 * i], g = d[4 * i + 1], b = d[4 * i + 2];
      L[i] = 0.299 * r + 0.587 * g + 0.114 * b;
      C1[i] = (r - g) * 0.6; C2[i] = ((r + g) * 0.5 - b) * 0.6;
    }
    L = blur5(L, w, h); C1 = blur5(C1, w, h); C2 = blur5(C2, w, h);
    var GX = new Float32Array(n), GY = new Float32Array(n), M = new Float32Array(n), chans = [L, C1, C2];
    var hist = new Int32Array(256), cnt = 0;
    for (y = 1; y < h - 1; y++) {
      for (x = 1; x < w - 1; x++) {
        var p = y * w + x, best = -1, bx = 0, by = 0;
        for (var c = 0; c < 3; c++) {
          var s = chans[c];
          var gx = (s[p - w + 1] + 2 * s[p + 1] + s[p + w + 1] - s[p - w - 1] - 2 * s[p - 1] - s[p + w - 1]) * 0.125;
          var gy = (s[p + w - 1] + 2 * s[p + w] + s[p + w + 1] - s[p - w - 1] - 2 * s[p - w] - s[p - w + 1]) * 0.125;
          var m2 = gx * gx + gy * gy;
          if (m2 > best) { best = m2; bx = gx; by = gy; }
        }
        var m = Math.sqrt(best);
        GX[p] = bx; GY[p] = by; M[p] = m;
        hist[m > 255 ? 255 : m | 0]++; cnt++;
      }
    }
    var acc = 0, med = 0;
    for (i = 0; i < 256; i++) { acc += hist[i]; if (acc >= cnt / 2) { med = i + 0.5; break; } }
    var T = Math.max(3, 1.5 * med);
    var ang = new Uint8Array(n);                 // 0 = 엣지 아님, 1..180 = (기울기 방향 각도 + 1)
    for (y = 2; y < h - 2; y++) {
      for (x = 2; x < w - 2; x++) {
        var q = y * w + x, mm = M[q];
        if (mm < T) continue;
        var a = Math.atan2(GY[q], GX[q]) * 180 / Math.PI; if (a < 0) a += 180; if (a >= 180) a -= 180;
        var o1, o2;
        if (a < 22.5 || a >= 157.5) { o1 = q - 1; o2 = q + 1; }
        else if (a < 67.5) { o1 = q - w - 1; o2 = q + w + 1; }
        else if (a < 112.5) { o1 = q - w; o2 = q + w; }
        else { o1 = q - w + 1; o2 = q + w - 1; }
        if (!(mm >= M[o1] && mm > M[o2])) continue;
        // "계단형" 경계만 남긴다: 기울기 방향으로 4px 양쪽의 색이 달라야 한다.
        // 글자 획·표 선·격자선처럼 가는 선은 양쪽이 같은 색이라 빠진다 → 종이 가장자리만 또렷해짐
        var ux = GX[q] / mm, uy = GY[q] / mm;
        var qa = Math.round(y + 4 * uy) * w + Math.round(x + 4 * ux), qb = Math.round(y - 4 * uy) * w + Math.round(x - 4 * ux);
        if (qa < 0 || qb < 0 || qa >= n || qb >= n) continue;
        var e0 = L[qa] - L[qb], e1 = C1[qa] - C1[qb], e2 = C2[qa] - C2[qb];
        if (e0 * e0 + e1 * e1 + e2 * e2 < 64) continue;
        ang[q] = 1 + (Math.round(a) % 180);
      }
    }
    return { w: w, h: h, L: L, C1: C1, C2: C2, ang: ang };
  }

  /* 엣지 점들을 직선에 다시 맞춰서(최소제곱) 허프의 1도 단위 오차를 없앤다 */
  function fitLineToEdges(E, deg, rho, tol) {
    var w = E.w, h = E.h, ang = E.ang, th = deg * Math.PI / 180, nx = Math.cos(th), ny = Math.sin(th);
    var dx = -ny, dy = nx, x0 = nx * rho, y0 = ny * rho;
    var tmin = 1e9, tmax = -1e9, cs = [[0, 0], [w, 0], [0, h], [w, h]];
    for (var k = 0; k < 4; k++) { var t = (cs[k][0] - x0) * dx + (cs[k][1] - y0) * dy; if (t < tmin) tmin = t; if (t > tmax) tmax = t; }
    var sx = 0, sy = 0, sxx = 0, syy = 0, sxy = 0, n = 0, seen = {};
    for (t = Math.floor(tmin); t <= tmax; t++) {
      for (var o = -tol; o <= tol; o++) {
        var xi = Math.round(x0 + t * dx + o * nx), yi = Math.round(y0 + t * dy + o * ny);
        if (xi < 0 || yi < 0 || xi >= w || yi >= h) continue;
        var p = yi * w + xi, a = ang[p];
        if (!a || seen[p] || angDiff(a - 1, deg) > 10) continue;
        seen[p] = 1; sx += xi; sy += yi; sxx += xi * xi; syy += yi * yi; sxy += xi * yi; n++;
      }
    }
    if (n < 10) return null;
    var mx = sx / n, my = sy / n, cxx = sxx / n - mx * mx, cyy = syy / n - my * my, cxy = sxy / n - mx * my;
    var phi = 0.5 * Math.atan2(2 * cxy, cxx - cyy);           // 직선 방향
    var nnx = -Math.sin(phi), nny = Math.cos(phi);
    if (nnx * nx + nny * ny < 0) { nnx = -nnx; nny = -nny; }
    return { nx: nnx, ny: nny, rho: nnx * mx + nny * my, n: n };
  }

  function houghLines(E, maxLines) {
    var w = E.w, h = E.h, ang = E.ang, diag = Math.ceil(Math.hypot(w, h)), NR = 2 * diag + 1;
    var acc = new Float32Array(180 * NR), cs = new Float32Array(180), sn = new Float32Array(180), x, y, t, k;
    var WT = [0.35, 0.6, 0.85, 1, 0.85, 0.6, 0.35];
    for (t = 0; t < 180; t++) { cs[t] = Math.cos(t * Math.PI / 180); sn[t] = Math.sin(t * Math.PI / 180); }
    for (y = 0; y < h; y++) {
      for (x = 0; x < w; x++) {
        var a = ang[y * w + x]; if (!a) continue; a--;
        for (k = -3; k <= 3; k++) {
          var tt = a + k; if (tt < 0) tt += 180; else if (tt >= 180) tt -= 180;
          acc[tt * NR + Math.round(x * cs[tt] + y * sn[tt]) + diag] += WT[k + 3];
        }
      }
    }
    var minV = 0.07 * Math.min(w, h), peaks = [], r;
    for (t = 0; t < 180; t++) {
      for (r = 0; r < NR; r++) {
        var v = acc[t * NR + r]; if (v < minV) continue;
        var isMax = true;
        for (var dt = -2; dt <= 2 && isMax; dt++) {
          var t2 = t + dt, r0 = r;
          if (t2 < 0) { t2 += 180; r0 = NR - 1 - r; } else if (t2 >= 180) { t2 -= 180; r0 = NR - 1 - r; }
          for (var dr = -3; dr <= 3; dr++) {
            var r2 = r0 + dr; if ((dt === 0 && dr === 0) || r2 < 0 || r2 >= NR) continue;
            var v2 = acc[t2 * NR + r2];
            if (v2 > v || (v2 === v && (t2 * NR + r2) < (t * NR + r))) { isMax = false; break; }
          }
        }
        if (isMax) peaks.push({ deg: t, rho: r - diag, v: v });
      }
    }
    peaks.sort(function (a, b) { return b.v - a.v; });
    var lines = [];
    for (var i = 0; i < peaks.length && lines.length < maxLines; i++) {
      var L = fitLineToEdges(E, peaks[i].deg, peaks[i].rho, 2);
      if (L) L = fitLineToEdges(E, Math.atan2(L.ny, L.nx) * 180 / Math.PI, L.rho, 1) || L;
      if (!L) continue;
      L.deg = Math.atan2(L.ny, L.nx) * 180 / Math.PI; L.v = peaks[i].v;
      var dup = false;
      for (var j = 0; j < lines.length && !dup; j++) {
        var M = lines[j];
        if (angDiff(L.deg, M.deg) > 2) continue;
        var cx = w / 2, cy = h / 2, pL = L.nx * cx + L.ny * cy - L.rho, pM = M.nx * cx + M.ny * cy - M.rho;
        if (L.nx * M.nx + L.ny * M.ny < 0) pM = -pM;
        if (Math.abs(pL - pM) < 3) dup = true;
      }
      if (!dup) lines.push(L);
    }
    return lines;
  }

  /* 직선을 따라가며 "엣지가 있고, 한쪽이 다른 쪽보다 밝은" 지점 수를 누적합으로 저장 */
  var TAU = 7;
  function lineProfile(E, L) {
    var w = E.w, h = E.h, ang = E.ang, LL = E.L, C1 = E.C1, C2 = E.C2;
    var nx = L.nx, ny = L.ny, dx = -ny, dy = nx, x0 = nx * L.rho, y0 = ny * L.rho, deg = L.deg;
    var tmin = 1e9, tmax = -1e9, cs = [[0, 0], [w, 0], [0, h], [w, h]], k;
    for (k = 0; k < 4; k++) { var t = (cs[k][0] - x0) * dx + (cs[k][1] - y0) * dy; if (t < tmin) tmin = t; if (t > tmax) tmax = t; }
    tmin = Math.floor(tmin); tmax = Math.ceil(tmax);
    var len = tmax - tmin + 1, Pin = new Int32Array(len + 1), Pp = new Int32Array(len + 1), Pn = new Int32Array(len + 1), Pa = new Int32Array(len + 1);
    var OFF = [3, 4.5, 6];
    function band(x, y, s, ch) {
      var sum = 0;
      for (var m = 0; m < 3; m++) {
        var xi = Math.round(x + s * OFF[m] * nx), yi = Math.round(y + s * OFF[m] * ny);
        sum += ch[yi * w + xi];
      }
      return sum / 3;
    }
    for (var i = 0; i < len; i++) {
      t = tmin + i;
      var x = x0 + t * dx, y = y0 + t * dy, ins = 0, gp = 0, gn = 0, ga = 0;
      if (x >= 7 && y >= 7 && x < w - 7 && y < h - 7) {
        ins = 1;
        var hit = false;
        for (k = -2; k <= 2 && !hit; k++) {
          var a = ang[Math.round(y + k * ny) * w + Math.round(x + k * nx)];
          if (a && angDiff(a - 1, deg) <= 12) hit = true;
        }
        if (hit) {
          var dl = band(x, y, 1, LL) - band(x, y, -1, LL);
          var d1 = band(x, y, 1, C1) - band(x, y, -1, C1), d2 = band(x, y, 1, C2) - band(x, y, -1, C2);
          var dc = Math.sqrt(dl * dl + d1 * d1 + d2 * d2);
          if (dl > TAU) gp = 1; else if (-dl > TAU) gn = 1;
          if (dc > TAU * 1.5) ga = 1;
        }
      }
      Pin[i + 1] = Pin[i] + ins; Pp[i + 1] = Pp[i] + gp; Pn[i + 1] = Pn[i] + gn; Pa[i + 1] = Pa[i] + ga;
    }
    return (L.prof = { tmin: tmin, len: len, x0: x0, y0: y0, dx: dx, dy: dy, Pin: Pin, Pp: Pp, Pn: Pn, Pa: Pa });
  }

  var BORDER_SCORE = 0.45;
  /* 변(직선 L 위의 c1~c2 구간) 점수: 0~1. g = 사각형 중심 */
  function sideScore(L, c1, c2, g) {
    if (L.border) return BORDER_SCORE;
    var P = L.prof;
    var t1 = (c1[0] - P.x0) * P.dx + (c1[1] - P.y0) * P.dy, t2 = (c2[0] - P.x0) * P.dx + (c2[1] - P.y0) * P.dy;
    var lo = Math.min(t1, t2), hi = Math.max(t1, t2), m = (hi - lo) * 0.06;
    lo += m; hi -= m;
    var segLen = hi - lo;
    var a = Math.max(0, Math.min(P.len, Math.round(lo - P.tmin))), b = Math.max(0, Math.min(P.len, Math.round(hi - P.tmin)));
    var inside = P.Pin[b] - P.Pin[a];
    if (inside < 0.5 * segLen || inside < 5) return 0;
    var s = L.nx * g[0] + L.ny * g[1] - L.rho;    // 중심이 법선(+) 쪽이면 종이도 + 쪽
    var polar = (s > 0 ? P.Pp[b] - P.Pp[a] : P.Pn[b] - P.Pn[a]) / inside;
    var any = (P.Pa[b] - P.Pa[a]) / inside;
    return Math.max(polar, 0.7 * any);
  }

  /* 직선 L 위에서 모서리 c 를 지나 (반대쪽 끝 o 의 반대 방향으로) 더 나아간 구간의 변 점수 */
  function extScore(L, c, o, g) {
    if (L.border) return 0;
    var P = L.prof;
    var tc = (c[0] - P.x0) * P.dx + (c[1] - P.y0) * P.dy, to = (o[0] - P.x0) * P.dx + (o[1] - P.y0) * P.dy;
    var E = Math.max(12, 0.12 * Math.abs(tc - to)), lo, hi;
    if (tc > to) { lo = tc + 4; hi = tc + 4 + E; } else { lo = tc - 4 - E; hi = tc - 4; }
    var a = Math.max(0, Math.min(P.len, Math.round(lo - P.tmin))), b = Math.max(0, Math.min(P.len, Math.round(hi - P.tmin)));
    var inside = P.Pin[b] - P.Pin[a];
    if (inside < 0.5 * E) return 0;
    var s = L.nx * g[0] + L.ny * g[1] - L.rho;
    return (s > 0 ? P.Pp[b] - P.Pp[a] : P.Pn[b] - P.Pn[a]) / inside;
  }

  function intersect(A, B) {
    var det = A.nx * B.ny - A.ny * B.nx;
    if (Math.abs(det) < 1e-9) return null;
    return [(A.rho * B.ny - A.ny * B.rho) / det, (A.nx * B.rho - A.rho * B.nx) / det];
  }

  /* 시계 방향, 왼쪽 위(x+y 최소)부터 */
  function orderQuad(q) {
    var cx = 0, cy = 0, i;
    for (i = 0; i < 4; i++) { cx += q[i][0] / 4; cy += q[i][1] / 4; }
    var s = q.slice().sort(function (a, b) { return Math.atan2(a[1] - cy, a[0] - cx) - Math.atan2(b[1] - cy, b[0] - cx); });
    var k = 0;
    for (i = 1; i < 4; i++) if (s[i][0] + s[i][1] < s[k][0] + s[k][1]) k = i;
    return [s[k], s[(k + 1) % 4], s[(k + 2) % 4], s[(k + 3) % 4]];
  }

  function quadGeom(c, w, h) {
    var mg = 0.04 * Math.max(w, h), i, sgn = 0, area = 0, minSide = 1e9;
    for (i = 0; i < 4; i++) {
      var p = c[i];
      if (p[0] < -mg || p[1] < -mg || p[0] > w + mg || p[1] > h + mg) return null;
      var a = c[(i + 3) % 4], b = c[(i + 1) % 4];
      var v1x = a[0] - p[0], v1y = a[1] - p[1], v2x = b[0] - p[0], v2y = b[1] - p[1];
      var l1 = Math.hypot(v1x, v1y), l2 = Math.hypot(v2x, v2y);
      if (l2 < minSide) minSide = l2;
      var cr = v1x * v2y - v1y * v2x;
      if (sgn === 0) sgn = cr > 0 ? 1 : -1; else if ((cr > 0 ? 1 : -1) !== sgn) return null;   // 볼록해야 함
      var cosA = (v1x * v2x + v1y * v2y) / (l1 * l2 + 1e-9);
      if (cosA > 0.77 || cosA < -0.77) return null;                                        // 내각 40°~140°
      area += p[0] * b[1] - b[0] * p[1];
    }
    area = Math.abs(area) / 2;
    if (area < 0.08 * w * h || minSide < 0.12 * Math.min(w, h)) return null;
    return { area: area };
  }

  /* 작게 줄인 이미지({data,width,height})에서 문서 모서리 [TL,TR,BR,BL] 또는 null */
  function detectQuad(img) {
    var w = img.width, h = img.height, i, j;
    var E = edgeMap(img);
    // 직선 후보를 넉넉히 뽑은 뒤, "한쪽이 다른 쪽보다 밝은" 구간이 충분히 긴 것만 남긴다
    // (커팅매트 격자선, 나뭇결 같은 선은 양쪽 색이 같아서 여기서 빠진다)
    var raw = houghLines(E, 80), lines = [], minGood = 0.1 * Math.min(w, h);
    for (i = 0; i < raw.length; i++) {
      var Pr = lineProfile(E, raw[i]);
      raw[i].good = Math.max(Pr.Pp[Pr.len], Pr.Pn[Pr.len], Pr.Pa[Pr.len]);
      if (raw[i].good >= minGood) lines.push(raw[i]);
    }
    lines.sort(function (a, b) { return b.good - a.good; });
    lines = lines.slice(0, 30);
    // 사진 테두리도 "약한 변" 후보로 넣는다 (종이가 화면 밖으로 나간 경우)
    [[1, 0, 0], [1, 0, w - 1], [0, 1, 0], [0, 1, h - 1]].forEach(function (b) {
      lines.push({ nx: b[0], ny: b[1], rho: b[2], deg: b[0] ? 0 : 90, border: true });
    });
    // 거의 평행한 두 직선 = 마주 보는 두 변 후보
    var cx = w / 2, cy = h / 2, minSep = 0.12 * Math.min(w, h), pairs = [];
    for (i = 0; i < lines.length; i++) {
      for (j = i + 1; j < lines.length; j++) {
        var A = lines[i], B = lines[j];
        if (A.border && B.border && A.deg !== B.deg) continue;
        if (angDiff(A.deg, B.deg) > 30) continue;
        var sa = A.nx * cx + A.ny * cy - A.rho, sb = B.nx * cx + B.ny * cy - B.rho;
        if (A.nx * B.nx + A.ny * B.ny < 0) sb = -sb;
        if (Math.abs(sa - sb) < minSep) continue;
        pairs.push([A, B, (A.border ? 1 : 0) + (B.border ? 1 : 0)]);
      }
    }
    var best = null, bestS = -1, W = w * h;
    for (i = 0; i < pairs.length; i++) {
      var P = pairs[i];
      for (j = i + 1; j < pairs.length; j++) {
        var Q = pairs[j];
        if (P[2] + Q[2] > 2) continue;
        if (P[0] === Q[0] || P[0] === Q[1] || P[1] === Q[0] || P[1] === Q[1]) continue;
        if (angDiff(P[0].deg, Q[0].deg) < 40 || angDiff(P[1].deg, Q[1].deg) < 40) continue;
        var a = P[0], b = P[1], c = Q[0], d = Q[1];
        var p1 = intersect(a, c), p2 = intersect(c, b), p3 = intersect(b, d), p4 = intersect(d, a);
        if (!p1 || !p2 || !p3 || !p4) continue;
        var corners = [p1, p2, p3, p4], gm = quadGeom(corners, w, h);
        if (!gm) continue;
        var g = [(p1[0] + p2[0] + p3[0] + p4[0]) / 4, (p1[1] + p2[1] + p3[1] + p4[1]) / 4];
        // 변 대응: c 위에 p1-p2, b 위에 p2-p3, d 위에 p3-p4, a 위에 p4-p1
        var s1 = sideScore(c, p1, p2, g); if (s1 < 0.2) continue;
        var s2 = sideScore(b, p2, p3, g); if (s2 < 0.2) continue;
        var s3 = sideScore(d, p3, p4, g); if (s3 < 0.2) continue;
        var s4 = sideScore(a, p4, p1, g); if (s4 < 0.2) continue;
        var mn = Math.min(s1, s2, s3, s4), mean = (s1 + s2 + s3 + s4) / 4;
        var S = 0.45 * mean + 0.55 * mn + 0.12 * Math.sqrt(gm.area / W);
        // 진짜 모서리라면 변이 거기서 끝나야 한다. 옆 변이 모서리 너머로 계속 이어지면
        // (예: 종이 안쪽의 표 머리줄을 아래 변으로 잘못 잡은 경우) 감점
        var pen = 0;
        pen += Math.max(0, Math.max(extScore(a, p1, p4, g), extScore(c, p1, p2, g)) - 0.3);
        pen += Math.max(0, Math.max(extScore(c, p2, p1, g), extScore(b, p2, p3, g)) - 0.3);
        pen += Math.max(0, Math.max(extScore(b, p3, p2, g), extScore(d, p3, p4, g)) - 0.3);
        pen += Math.max(0, Math.max(extScore(d, p4, p3, g), extScore(a, p4, p1, g)) - 0.3);
        S -= 0.6 * pen;
        if (S > bestS) { bestS = S; best = corners; }
      }
    }
    if (!best || bestS < 0.4) return null;
    return orderQuad(best);
  }

  /* 큰 이미지에서 각 변을 다시 맞춘다. img: {data,width,height}, q: img 좌표의 [TL,TR,BR,BL] */
  function refineQuad(img, q) {
    var w = img.width, h = img.height, d = img.data;
    function lum(x, y) {
      if (x < 0) x = 0; else if (x > w - 1.001) x = w - 1.001;
      if (y < 0) y = 0; else if (y > h - 1.001) y = h - 1.001;
      var x0 = x | 0, y0 = y | 0, fx = x - x0, fy = y - y0, p = (y0 * w + x0) * 4, p2 = p + w * 4;
      var a = d[p] * 0.299 + d[p + 1] * 0.587 + d[p + 2] * 0.114, b = d[p + 4] * 0.299 + d[p + 5] * 0.587 + d[p + 6] * 0.114;
      var c = d[p2] * 0.299 + d[p2 + 1] * 0.587 + d[p2 + 2] * 0.114, e = d[p2 + 4] * 0.299 + d[p2 + 5] * 0.587 + d[p2 + 6] * 0.114;
      return (a * (1 - fx) + b * fx) * (1 - fy) + (c * (1 - fx) + e * fx) * fy;
    }
    var gx = 0, gy = 0, i, k;
    for (i = 0; i < 4; i++) { gx += q[i][0] / 4; gy += q[i][1] / 4; }
    var R = Math.max(6, Math.round(Math.max(w, h) * 0.008)), N = 48, lines = [];
    for (i = 0; i < 4; i++) {
      var A = q[i], B = q[(i + 1) % 4], len = dist(A, B);
      var ux = (B[0] - A[0]) / len, uy = (B[1] - A[1]) / len, nx = -uy, ny = ux;
      if (nx * (gx - A[0]) + ny * (gy - A[1]) > 0) { nx = -nx; ny = -ny; }          // n = 바깥쪽
      // 안쪽이 밝은지 어두운지
      var pol = 0;
      for (k = 0; k < 9; k++) {
        var tt = 0.1 + 0.8 * k / 8, px = A[0] + (B[0] - A[0]) * tt, py = A[1] + (B[1] - A[1]) * tt;
        pol += lum(px - nx * 4, py - ny * 4) - lum(px + nx * 4, py + ny * 4);
      }
      var sg = pol >= 0 ? 1 : -1, pts = [];
      for (k = 0; k < N; k++) {
        var t = 0.06 + 0.88 * (k + 0.5) / N, bx = A[0] + (B[0] - A[0]) * t, by = A[1] + (B[1] - A[1]) * t;
        var bestS = 0, vals = [];
        for (var s = -R; s <= R + 1e-6; s += 0.5) {
          var v = sg * (lum(bx + nx * (s - 1), by + ny * (s - 1)) - lum(bx + nx * (s + 1), by + ny * (s + 1)));
          vals.push(v);
        }
        // 후보 = 계단형 경계(양쪽 4px 의 밝기가 확실히 다름). 글자 획·격자선 같은 가는 선은 제외.
        // 그중 충분히 강한 것(최댓값의 40% 이상) 가운데 가장 바깥쪽을 고른다
        // (종이 바로 안쪽의 표 선이나 종이 아래 그림자 대신 종이 끝을 잡도록)
        var cand = [], vmax = 0, bi = -1, m;
        for (m = 0; m < vals.length; m++) {
          if (vals[m] < 6 || (m > 0 && vals[m] < vals[m - 1]) || (m < vals.length - 1 && vals[m] < vals[m + 1])) continue;
          var sm = -R + m * 0.5;
          var step = sg * (lum(bx + nx * (sm - 4), by + ny * (sm - 4)) - lum(bx + nx * (sm + 4), by + ny * (sm + 4)));
          if (step < Math.max(6, 0.6 * vals[m])) continue;
          cand.push(m); if (vals[m] > vmax) vmax = vals[m];
        }
        for (m = cand.length - 1; m >= 0; m--) if (vals[cand[m]] >= 0.4 * vmax) { bi = cand[m]; break; }
        if (bi < 0) continue;
        var off = 0;
        if (bi > 0 && bi < vals.length - 1) {
          var den = vals[bi - 1] - 2 * vals[bi] + vals[bi + 1];
          if (den < 0) off = 0.5 * (vals[bi - 1] - vals[bi + 1]) / den;
        }
        bestS = -R + (bi + off) * 0.5;
        pts.push([bx + nx * bestS, by + ny * bestS]);
      }
      var L = robustLine(pts);
      if (!L) { L = { nx: nx, ny: ny, rho: nx * A[0] + ny * A[1] }; }
      lines.push(L);
    }
    var out = [];
    for (i = 0; i < 4; i++) {
      var c = intersect(lines[(i + 3) % 4], lines[i]);
      if (!c || dist(c, q[i]) > 4 * R) return q;
      out.push(c);
    }
    return out;
  }

  function robustLine(pts) {
    if (pts.length < 8) return null;
    var use = pts, L = null;
    for (var it = 0; it < 3; it++) {
      var n = use.length, mx = 0, my = 0, cxx = 0, cyy = 0, cxy = 0, i;
      if (n < 6) return L;
      for (i = 0; i < n; i++) { mx += use[i][0]; my += use[i][1]; }
      mx /= n; my /= n;
      for (i = 0; i < n; i++) { var ex = use[i][0] - mx, ey = use[i][1] - my; cxx += ex * ex; cyy += ey * ey; cxy += ex * ey; }
      var phi = 0.5 * Math.atan2(2 * cxy, cxx - cyy), nx = -Math.sin(phi), ny = Math.cos(phi);
      L = { nx: nx, ny: ny, rho: nx * mx + ny * my };
      var res = pts.map(function (p) { return Math.abs(nx * p[0] + ny * p[1] - L.rho); });
      var sorted = res.slice().sort(function (a, b) { return a - b; });
      var thr = Math.max(0.8, 2.5 * 1.4826 * sorted[sorted.length >> 1]);
      use = pts.filter(function (p, j) { return res[j] <= thr; });
    }
    return L;
  }

  /* 사각형의 각 변을 안쪽으로 px 만큼 평행 이동 (종이 바깥의 어두운 띠가 섞이지 않게) */
  function insetQuad(q, px) {
    var gx = 0, gy = 0, i, lines = [];
    for (i = 0; i < 4; i++) { gx += q[i][0] / 4; gy += q[i][1] / 4; }
    for (i = 0; i < 4; i++) {
      var A = q[i], B = q[(i + 1) % 4], len = dist(A, B) || 1;
      var nx = -(B[1] - A[1]) / len, ny = (B[0] - A[0]) / len;
      if (nx * (gx - A[0]) + ny * (gy - A[1]) < 0) { nx = -nx; ny = -ny; }          // n = 안쪽
      lines.push({ nx: nx, ny: ny, rho: nx * A[0] + ny * A[1] + px });
    }
    var out = [];
    for (i = 0; i < 4; i++) { var c = intersect(lines[(i + 3) % 4], lines[i]); if (!c) return q; out.push(c); }
    return out;
  }

  /* ================= 문서 필터 =================
     mode: 'doc'(그림자 제거 + 선명) | 'bw'(흑백) | 'color'(원본 색, 살짝 선명하게만). img.data 를 직접 수정 */
  function applyFilter(img, mode) {
    var w = img.width, h = img.height, d = img.data, n = w * h, x, y, i;
    if (mode !== 'color') {
      // 1) 종이 밝기(배경) 추정: 블록별 최댓값 → 닫힘 연산(도장·굵은 글씨 같은 큰 어두운 부분을 메움) → 부드럽게
      var B = Math.max(6, Math.round(Math.max(w, h) / 110));
      var bw = Math.ceil(w / B), bh = Math.ceil(h / B);
      var bg = new Float32Array(bw * bh);
      for (y = 0; y < h; y++) {
        var by = ((y / B) | 0) * bw;
        for (x = 0; x < w; x++) {
          var k = (y * w + x) * 4;
          var lum = (d[k] * 77 + d[k + 1] * 150 + d[k + 2] * 29) / 256;
          var bi = by + ((x / B) | 0);
          if (lum > bg[bi]) bg[bi] = lum;
        }
      }
      var RAD = 3;
      bg = minMaxFilter(minMaxFilter(bg, bw, bh, RAD, true), bw, bh, RAD, false);
      bg = boxBlur(boxBlur(bg, bw, bh, 1), bw, bh, 1);
      // 2) 배경으로 나눠서 종이를 하얗게 (그림자·조명 얼룩 제거)
      for (y = 0; y < h; y++) {
        var fy = y / B - 0.5; if (fy < 0) fy = 0; if (fy > bh - 1) fy = bh - 1;
        var y0 = fy | 0, y1 = Math.min(y0 + 1, bh - 1), ty = fy - y0;
        for (x = 0; x < w; x++) {
          var fx = x / B - 0.5; if (fx < 0) fx = 0; if (fx > bw - 1) fx = bw - 1;
          var x0 = fx | 0, x1 = Math.min(x0 + 1, bw - 1), tx = fx - x0;
          var bgv = (bg[y0 * bw + x0] * (1 - tx) + bg[y0 * bw + x1] * tx) * (1 - ty) +
                    (bg[y1 * bw + x0] * (1 - tx) + bg[y1 * bw + x1] * tx) * ty;
          if (bgv < 40) bgv = 40;
          var sc = 255 / bgv;
          i = (y * w + x) * 4;
          d[i] = d[i] * sc; d[i + 1] = d[i + 1] * sc; d[i + 2] = d[i + 2] * sc;    // Uint8ClampedArray 가 255에서 자름
        }
      }
    }
    // 3) 선명하게(언샤프 마스크) + 대비 곡선
    var amount = mode === 'color' ? 0.3 : 0.7;
    var lo = mode === 'bw' ? 100 : mode === 'doc' ? 45 : 0, hi = mode === 'bw' ? 215 : mode === 'doc' ? 228 : 255;
    var LUT = new Uint8ClampedArray(512 + 256);                    // 입력 -256..511 → 출력
    for (i = 0; i < LUT.length; i++) LUT[i] = Math.round((i - 256 - lo) * 255 / (hi - lo));
    var Y = new Uint8ClampedArray(n);
    for (i = 0; i < n; i++) Y[i] = (d[4 * i] * 77 + d[4 * i + 1] * 150 + d[4 * i + 2] * 29) >> 8;
    for (y = 0; y < h; y++) {
      var r0 = (y > 0 ? y - 1 : 0) * w, r1 = y * w, r2 = (y < h - 1 ? y + 1 : h - 1) * w;
      for (x = 0; x < w; x++) {
        var xl = x > 0 ? x - 1 : 0, xr = x < w - 1 ? x + 1 : w - 1, p = r1 + x;
        var blur = (Y[r0 + xl] + 2 * Y[r0 + x] + Y[r0 + xr] + 2 * Y[r1 + xl] + 4 * Y[p] + 2 * Y[r1 + xr] +
                    Y[r2 + xl] + 2 * Y[r2 + x] + Y[r2 + xr]) * 0.0625;
        var delta = amount * (Y[p] - blur), q4 = p * 4;
        if (mode === 'bw') {
          var gv = Y[p] + delta;
          d[q4] = d[q4 + 1] = d[q4 + 2] = LUT[(gv < -256 ? -256 : gv > 511 ? 511 : gv) + 256 | 0];
        } else {
          for (var c = 0; c < 3; c++) {
            var vv = d[q4 + c] + delta;
            d[q4 + c] = LUT[(vv < -256 ? -256 : vv > 511 ? 511 : vv) + 256 | 0];
          }
        }
      }
    }
    return img;
  }

  /* 정사각형 창의 최댓값(isMax) 또는 최솟값 필터 (분리형) */
  function minMaxFilter(src, w, h, r, isMax) {
    var tmp = new Float32Array(w * h), out = new Float32Array(w * h), x, y, k, v, m;
    for (y = 0; y < h; y++) {
      for (x = 0; x < w; x++) {
        m = isMax ? -1e9 : 1e9;
        for (k = -r; k <= r; k++) {
          var xx = x + k; if (xx < 0 || xx >= w) continue;
          v = src[y * w + xx]; if (isMax ? v > m : v < m) m = v;
        }
        tmp[y * w + x] = m;
      }
    }
    for (x = 0; x < w; x++) {
      for (y = 0; y < h; y++) {
        m = isMax ? -1e9 : 1e9;
        for (k = -r; k <= r; k++) {
          var yy = y + k; if (yy < 0 || yy >= h) continue;
          v = tmp[yy * w + x]; if (isMax ? v > m : v < m) m = v;
        }
        out[y * w + x] = m;
      }
    }
    return out;
  }

  var api = { homography: homography, outputSize: outputSize, warp: warp,
              detectQuad: detectQuad, refineQuad: refineQuad, insetQuad: insetQuad, applyFilter: applyFilter,
              DETECT_SIZE: DETECT_SIZE, REFINE_SIZE: REFINE_SIZE };
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  else root.ScanCore = api;
})(typeof self !== 'undefined' ? self : this);
