/* One renderer for every point cloud on the site: the TetraSwarm map and the robots built from their own CAD.
   Points only, coloured by height from --point-lo to --point-hi, a sweep that lights them, lime where something was found.
   Drag to turn; it turns on its own otherwise. Stops off screen, holds still under reduced motion. */
(function () {
  'use strict';
  const reduce = matchMedia('(prefers-reduced-motion: reduce)');
  let palette = null;
  function readPalette() {
    if (palette) return palette;
    const probe = document.createElement('span');
    probe.style.display = 'none';
    document.body.appendChild(probe);
    const steps = [];
    const N = 12;
    for (let i = 0; i < N; i++) {
      const t = i / (N - 1);
      probe.style.color = t < 0.5
        ? `color-mix(in oklch, var(--point-mid) ${Math.round(t * 200)}%, var(--point-lo))`
        : `color-mix(in oklch, var(--point-hi) ${Math.round((t - 0.5) * 200)}%, var(--point-mid))`;
      steps.push(getComputedStyle(probe).color);
    }
    probe.style.color = 'var(--color-accent)';
    const lime = getComputedStyle(probe).color;
    probe.remove();
    palette = { steps, lime };
    return palette;
  }

  function decode(json) {
    const raw = atob(json.pts);
    const buf = new ArrayBuffer(raw.length);
    const u8 = new Uint8Array(buf);
    for (let i = 0; i < raw.length; i++) u8[i] = raw.charCodeAt(i);
    const q = new Int16Array(buf);
    const n = json.n;
    const x = new Float32Array(n), y = new Float32Array(n), z = new Float32Array(n);
    for (let i = 0; i < n; i++) { x[i] = q[i * 3] / 32767; y[i] = q[i * 3 + 1] / 32767; z[i] = q[i * 3 + 2] / 32767; }
    const fr = atob(json.found);
    const found = new Uint8Array(n);
    for (let i = 0; i < n; i++) found[i] = (fr.charCodeAt(i >> 3) >> (7 - (i & 7))) & 1;
    return { x, y, z, found, n, size: json.size_mm };
  }

  const cache = new Map();
  function load(url) {
    if (!cache.has(url)) cache.set(url, fetch(url).then((r) => { if (!r.ok) throw new Error(url); return r.json(); }).then(decode));
    return cache.get(url);
  }

  /* canvas: the element; opts: { pitch, yaw0, period (ms per turn), fill (share of the frame), lift, drag, onFrame } */
  function PointCloud(canvas, opts) {
    const o = Object.assign({ pitch: 0.42, yaw0: 0.6, period: 70000, fill: 1.0, size: 1.25, drag: true, sweep: 7000 }, opts || {});
    const ctx = canvas.getContext('2d');
    let data = null, zMin = 0, zMax = 1, span = 1, W = 0, H = 0, dpr = 1;
    let yaw = o.yaw0, pitch = o.pitch, dragging = false, lastX = 0, lastY = 0, raf = 0, last = 0, t0 = 0, clock = 0, vy = 0, vp = 0;
    const st = { seen: false, held: reduce.matches };
    const pal = readPalette();
    const buckets = Array.from({ length: pal.steps.length }, () => []);
    const lime = [];

    function setData(d) {
      data = d;
      zMin = Infinity; zMax = -Infinity; let r = 0;
      for (let i = 0; i < d.n; i++) { if (d.z[i] < zMin) zMin = d.z[i]; if (d.z[i] > zMax) zMax = d.z[i]; }
      const zc = (zMin + zMax) / 2;
      for (let i = 0; i < d.n; i++) { const rr = Math.hypot(d.x[i], d.y[i], d.z[i] - zc); if (rr > r) r = rr; }
      span = r || 1;
      draw(clock);
    }
    function project(px, py, pz) {
      const cy = Math.cos(yaw), sy = Math.sin(yaw), cp = Math.cos(pitch), sp = Math.sin(pitch);
      const zc = pz - (zMin + zMax) / 2;
      const X = px * cy - py * sy;
      const D = px * sy + py * cy;
      const Y = zc * cp - D * sp;
      const Z = zc * sp + D * cp;
      const scale = (Math.min(W, H) * o.fill) / (2 * span);
      const persp = 9 / (9 - Z / span); // +Z faces the viewer, so near points grow; a long lens keeps proportions at every angle
      return [W / 2 + X * scale * persp, H * 0.5 - Y * scale * persp, persp];
    }
    function draw(t) {
      if (!data || !W || !H) return;
      ctx.clearRect(0, 0, W, H);
      for (const b of buckets) b.length = 0;
      lime.length = 0;
      const ph = (t % o.sweep) / o.sweep;
      const e = ph < 0.5 ? 4 * ph * ph * ph : 1 - Math.pow(-2 * ph + 2, 3) / 2;
      const sweepZ = zMin - 0.15 + e * (zMax - zMin + 0.3);
      const band = (zMax - zMin) * 0.08 || 0.05;
      const K = pal.steps.length - 1;
      const base = o.size * Math.max(1, Math.min(W, H) / 420);
      for (let i = 0; i < data.n; i++) {
        const [sx, sy, d] = project(data.x[i], data.y[i], data.z[i]);
        if (sx < -4 || sx > W + 4 || sy < -4 || sy > H + 4) continue;
        const s = Math.max(1, base * d);
        if (data.found[i]) { lime.push(sx, sy, s * 1.15); continue; }
        const h = (data.z[i] - zMin) / ((zMax - zMin) || 1);
        const lift = Math.max(0, 1 - Math.abs(data.z[i] - sweepZ) / band);
        const k = Math.min(K, Math.round((0.18 + h * 0.5 + lift * 0.45) * K));
        buckets[k].push(sx, sy, s);
      }
      for (let k = 0; k <= K; k++) {
        const b = buckets[k];
        if (!b.length) continue;
        ctx.fillStyle = pal.steps[k];
        for (let i = 0; i < b.length; i += 3) ctx.fillRect(b[i], b[i + 1], b[i + 2], b[i + 2]);
      }
      ctx.fillStyle = pal.lime;
      for (let i = 0; i < lime.length; i += 3) ctx.fillRect(lime[i], lime[i + 1], lime[i + 2], lime[i + 2]);
      if (o.onFrame) o.onFrame(project, W, H);
    }
    function resize() {
      const b = canvas.getBoundingClientRect();
      dpr = Math.min(window.devicePixelRatio || 1, 2);
      W = b.width; H = b.height;
      canvas.width = Math.round(W * dpr); canvas.height = Math.round(H * dpr);
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      draw(clock);
    }
    function tick(now) {
      const dt = Math.min(64, now - last); last = now;
      clock += dt;
      if (!dragging) {
        yaw += (dt / o.period) * Math.PI * 2;
        if (Math.abs(vy) > 0.0005) { yaw += vy; vy *= 0.94; }
        if (Math.abs(vp) > 0.0005) { pitch += vp; vp *= 0.94; }
      }
      draw(clock);
      raf = requestAnimationFrame(tick);
    }
    function sync() {
      const run = st.seen && !st.held && !document.hidden;
      if (run && !raf) { last = performance.now(); raf = requestAnimationFrame(tick); }
      if (!run && raf) { cancelAnimationFrame(raf); raf = 0; }
    }
    if (o.drag) {
      canvas.style.touchAction = 'none';
      if (!canvas.hasAttribute('tabindex')) canvas.tabIndex = 0;
      canvas.addEventListener('keydown', (e) => {
        const k = { ArrowLeft: [-0.15, 0], ArrowRight: [0.15, 0], ArrowUp: [0, -0.15], ArrowDown: [0, 0.15] }[e.key];
        if (!k) return;
        e.preventDefault(); yaw += k[0]; pitch += k[1]; if (!raf) draw(clock);
      });
      canvas.addEventListener('pointerdown', (e) => { vy = 0; vp = 0; dragging = true; lastX = e.clientX; lastY = e.clientY; canvas.setPointerCapture(e.pointerId); canvas.classList.add('is-dragging'); });
      canvas.addEventListener('pointermove', (e) => {
        if (!dragging) return;
        vy = (e.clientX - lastX) * 0.01; vp = (e.clientY - lastY) * 0.008;
        yaw += vy; pitch += vp;
        lastX = e.clientX; lastY = e.clientY;
        if (!raf) draw(clock);
      });
      const end = () => { dragging = false; canvas.classList.remove('is-dragging'); };
      canvas.addEventListener('pointerup', end);
      canvas.addEventListener('pointercancel', end);
    }
    new IntersectionObserver(([e]) => { st.seen = e.isIntersecting; sync(); }).observe(canvas);
    document.addEventListener('visibilitychange', sync);
    reduce.addEventListener && reduce.addEventListener('change', () => { st.held = reduce.matches; sync(); });
    if ('ResizeObserver' in window) new ResizeObserver(resize).observe(canvas); else window.addEventListener('resize', resize);
    resize();
    return {
      setData,
      hold(v) { st.held = v; sync(); },
      get held() { return st.held; },
      reset() { yaw = o.yaw0; pitch = o.pitch; draw(clock); }
    };
  }
  PointCloud.load = load;
  window.PointCloud = PointCloud;
})();
