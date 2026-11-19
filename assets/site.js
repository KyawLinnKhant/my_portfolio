/* Kyaw Linn Khant portfolio: nav, the TetraSwarm point cloud, the preview index, the case stage, the record tabs. */
(function () {
  'use strict';
  const reduce = matchMedia('(prefers-reduced-motion: reduce)');
  const y = document.getElementById('y');
  if (y) y.textContent = new Date().getFullYear();

  /* ── Nav: mobile menu and current section ── */
  const toggle = document.querySelector('.nav__toggle');
  const links = document.getElementById('nav-links');
  if (toggle && links) {
    const setOpen = (open) => { links.classList.toggle('is-open', open); toggle.setAttribute('aria-expanded', String(open)); };
    toggle.addEventListener('click', () => setOpen(!links.classList.contains('is-open')));
    links.addEventListener('click', (e) => { if (e.target.closest('a')) setOpen(false); });
    document.addEventListener('keydown', (e) => { if (e.key === 'Escape' && links.classList.contains('is-open')) { setOpen(false); toggle.focus(); } });
  }
  const navLinks = Array.from(document.querySelectorAll('.nav__links a[data-target]'));
  const sectionFor = (id) => { const el = document.getElementById(id); return el && (el.closest('main > section') || el); };
  if (navLinks.length && 'IntersectionObserver' in window) {
    const map = new Map();
    navLinks.forEach((a) => { const s = sectionFor(a.dataset.target); if (s) map.set(s, a); });
    const io = new IntersectionObserver((entries) => {
      entries.forEach((e) => {
        const a = map.get(e.target);
        if (!a) return;
        if (e.isIntersecting) { navLinks.forEach((l) => l.removeAttribute('aria-current')); a.setAttribute('aria-current', 'true'); }
      });
    }, { rootMargin: '-45% 0px -50% 0px' });
    map.forEach((_, s) => io.observe(s));
  }

  const frame = document.querySelector('.hero .frame');

  /* ── The robots in 3D: one stage, four models ── */
  const MODELS = {
    quadruped: { name: 'Sim2Real quadruped', size: '385 × 236 × 280', joints: '12 revolute joints, three per leg', source: '13 STL parts in sim2real-quad/hardware', caption: 'Lime marks where the four feet meet the ground.', id: 'sim2real-quad', code: 'sim2real-quad', pitch: 0.38 },
    'so-arm': { name: 'SO-ARM100 arm', size: '111 × 304 × 170', joints: 'SO-ARM100, 6 revolute joints from pan to gripper, at its URDF zero pose', source: 'so_arm100.urdf and its 13 STL meshes in IsaacLab_SO-ARM100-101', caption: 'Lime marks the gripper jaws.', id: 'isaaclab-so-arm', code: 'IsaacLab_SO-ARM100-101', pitch: 0.42 },
    balance: { name: 'self-balancing robot', size: '285 × 80 × 167', joints: '2 driven wheels on one axle', source: 'robot_body.stl and two wheel meshes, placed by robot.xml in IMP_MJC_RL', caption: 'Lime marks the two wheels it balances on, placed as the MuJoCo model places them.', id: 'rl-balance', code: 'IMP_MJC_RL', pitch: 0.3 },
    turtlebot: { name: 'TurtleBot3 Burger', size: '138 × 178 × 191', joints: '2 wheels, 160 mm track width', source: 'burger_base, lds and tire STLs, placed by the URDF in slam_turtlebot_ros', caption: 'Lime marks the LDS 360° lidar the EKF SLAM reads.', id: 'slam-turtlebot-ros', code: 'slam_turtlebot_ros', pitch: 0.36 }
  };
  const mCanvas = document.getElementById('model-canvas');
  if (mCanvas && window.PointCloud && mCanvas.getContext) {
    const mTabs = Array.from(document.querySelectorAll('.models__tabs [role="tab"]'));
    const panel = document.getElementById('model-panel');
    const still = document.getElementById('model-still');
    const $ = (id) => document.getElementById(id);
    const cloud = window.PointCloud(mCanvas, { pitch: 0.38, period: 60000 });
    let current = null;
    const show = (key, focus) => {
      const m = MODELS[key]; if (!m || key === current) { if (focus) mTabs.find((t) => t.dataset.model === key).focus(); return; }
      current = key;
      mTabs.forEach((t) => { const on = t.dataset.model === key; t.setAttribute('aria-selected', String(on)); t.tabIndex = on ? 0 : -1; if (on) { panel.setAttribute('aria-labelledby', t.id); if (focus) t.focus(); } });
      $('model-size').innerHTML = m.size + '<span class="u">mm</span>';
      $('model-joints').textContent = m.joints;
      $('model-source').textContent = m.source;
      $('model-caption').textContent = m.caption;
      $('model-link').href = 'project.html?id=' + m.id;
      $('model-code').href = 'https://github.com/KyawLinnKhant/' + m.code;
      mCanvas.setAttribute('aria-label', 'Point cloud of the ' + m.name + ', assembled from its own CAD files. Drag or use the arrow keys to spin it.');
      still.src = 'assets/clouds/' + key + '.png'; still.alt = 'Point cloud of the ' + m.name;
      const swap = !reduce.matches && mCanvas.dataset.ready === '1';
      if (swap) mCanvas.classList.add('is-swapping');
      Promise.all([
        window.PointCloud.load('assets/clouds/' + key + '.json'),
        new Promise((r) => setTimeout(r, swap ? 260 : 0))
      ]).then(([d]) => { if (current === key) { cloud.setData(d); cloud.reset(); mCanvas.dataset.ready = '1'; mCanvas.classList.remove('is-swapping'); } }).catch(() => { document.documentElement.classList.add('no-cloud'); });
    };
    /* show one, then the next: the shown tab's line fills, and when it is full the next robot loads.
       It holds while the pointer or focus is on the stage, while it is off screen, and never runs under reduced motion. */
    const stage = mCanvas.closest('.models__stage');
    const hold = { hover: false, focus: false, off: true };
    const syncCycle = () => {
      stage.classList.toggle('is-cycling', !reduce.matches);
      stage.classList.toggle('is-held', hold.hover || hold.focus || hold.off);
    };
    stage.addEventListener('pointerenter', () => { hold.hover = true; syncCycle(); });
    stage.addEventListener('pointerleave', () => { hold.hover = false; syncCycle(); });
    stage.addEventListener('focusin', () => { hold.focus = true; syncCycle(); });
    stage.addEventListener('focusout', (e) => { if (!stage.contains(e.relatedTarget)) { hold.focus = false; syncCycle(); } });
    new IntersectionObserver(([e]) => { hold.off = !e.isIntersecting; syncCycle(); }, { threshold: 0.35 }).observe(stage);
    reduce.addEventListener && reduce.addEventListener('change', syncCycle);
    stage.addEventListener('animationend', (e) => {
      if (e.animationName !== 'models-tick') return;
      const i = mTabs.findIndex((t) => t.dataset.model === current);
      show(mTabs[(i + 1) % mTabs.length].dataset.model);
    });
    syncCycle();
    mTabs.forEach((t, i) => {
      t.addEventListener('click', () => show(t.dataset.model));
      t.addEventListener('keydown', (e) => {
        const d = (e.key === 'ArrowDown' || e.key === 'ArrowRight') ? 1 : (e.key === 'ArrowUp' || e.key === 'ArrowLeft') ? -1 : 0;
        if (d) { e.preventDefault(); show(mTabs[(i + d + mTabs.length) % mTabs.length].dataset.model, true); }
      });
    });
    const fromHash = () => { const m = location.hash.match(/^#model-(.+)$/); if (m && MODELS[m[1]]) { show(m[1]); document.getElementById('models').scrollIntoView({ behavior: reduce.matches ? 'auto' : 'smooth' }); } };
    window.addEventListener('hashchange', fromHash);
    show('quadruped');
    fromHash();
  }

  /* ── The point cloud: the swarm's own occupancy grid, turning ── */
  const canvas = document.getElementById('cloud');
  const grid = window.MAZE;
  if (!canvas || !grid || !canvas.getContext) return;
  const ctx = canvas.getContext('2d');
  const R = grid.length, C = grid[0].length, S = 1;
  const pts = [];
  // found: the far corner the swarm picked (top right of the map), where the blocks were placed
  const isFound = (r, c) => r >= 1 && r <= 9 && c >= C - 14 && c <= C - 3;
  for (let r = 0; r < R; r++) {
    for (let c = 0; c < C; c++) {
      const ch = grid[r][c];
      const x = (c - C / 2) * S, z = (r - R / 2) * S;
      if (ch === '#') {
        for (let h = 0; h < 4; h++) pts.push({ x, z, y: h * 1.15 + 0.4, lvl: 0.45 + h * 0.18, f: 0 });
      } else if (ch === '.') {
        const j = ((r * 73856093) ^ (c * 19349663)) & 1023;
        pts.push({ x: x + (j % 32) / 64 - 0.25, z: z + (j >> 5) / 64 - 0.25, y: 0, lvl: 0.12 + (j % 7) / 70, f: isFound(r, c) ? 1 : 0 });
      }
    }
  }
  // callout anchors, in grid space
  const anchors = {
    found: { x: (C - 8 - C / 2) * S, z: (4 - R / 2) * S, y: 0 },
    mapped: { x: (8 - C / 2) * S, z: (40 - R / 2) * S, y: 4.5 }
  };
  const callouts = Array.from(document.querySelectorAll('.callout[data-anchor]'));

  // the palette: point-lo to point-hi in steps, read from the tokens
  const probe = document.createElement('span');
  probe.style.display = 'none';
  document.body.appendChild(probe);
  const STEPS = 12;
  const pal = [];
  for (let i = 0; i < STEPS; i++) {
    const t = i / (STEPS - 1);
    const mid = t < 0.5 ? `color-mix(in oklch, var(--point-mid) ${Math.round(t * 200)}%, var(--point-lo))` : `color-mix(in oklch, var(--point-hi) ${Math.round((t - 0.5) * 200)}%, var(--point-mid))`;
    probe.style.color = mid;
    pal.push(getComputedStyle(probe).color);
  }
  probe.style.color = 'var(--color-accent)';
  const lime = getComputedStyle(probe).color;
  probe.remove();

  let W = 0, H = 0, dpr = 1;
  const sizes = new Map();
  const resize = () => {
    callouts.forEach((el) => sizes.set(el, [el.offsetWidth, el.offsetHeight]));
    const b = canvas.getBoundingClientRect();
    dpr = Math.min(window.devicePixelRatio || 1, 2);
    W = b.width; H = b.height;
    canvas.width = Math.round(W * dpr); canvas.height = Math.round(H * dpr);
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    draw(last);
  };

  // spin it: drag (mouse or touch) to turn and tilt, let go and it coasts; arrow keys turn it too
  let spin = 0, tilt = 0, vel = 0, tvel = 0, dragging = false, px = 0, py = 0;
  const redraw = () => { if (!raf) draw(last); };
  canvas.style.touchAction = 'none';
  canvas.addEventListener('pointerdown', (e) => { dragging = true; vel = 0; tvel = 0; px = e.clientX; py = e.clientY; canvas.setPointerCapture(e.pointerId); canvas.classList.add('is-dragging'); });
  canvas.addEventListener('pointermove', (e) => {
    if (!dragging) return;
    const dx = e.clientX - px, dy = e.clientY - py;
    px = e.clientX; py = e.clientY;
    spin += dx * 0.01; vel = dx * 0.01; tvel = dy * 0.008;
    tilt += dy * 0.008;
    redraw();
  });
  const release = () => { dragging = false; canvas.classList.remove('is-dragging'); };
  canvas.addEventListener('pointerup', release);
  canvas.addEventListener('pointercancel', release);
  canvas.addEventListener('keydown', (e) => {
    const k = { ArrowLeft: [-0.15, 0], ArrowRight: [0.15, 0], ArrowUp: [0, -0.15], ArrowDown: [0, 0.15] }[e.key];
    if (!k) return;
    e.preventDefault();
    spin += k[0]; tilt += k[1];
    redraw();
  });

  const buckets = Array.from({ length: STEPS }, () => []);
  const found = [];
  function project(p, cy, sy, cp, sp, scale, ox, oy) {
    const X = p.x * cy - p.z * sy;
    const Z0 = p.x * sy + p.z * cy;
    const Y = p.y * cp - Z0 * sp;
    const Z = p.y * sp + Z0 * cp;
    const d = 1100 / (1100 - Z); // +Z faces the viewer, so near points grow; a long lens keeps it true
    return [ox + X * scale * d, oy - Y * scale * d, d];
  }
  let last = 0;
  function draw(t) {
    if (!W || !H) return;
    ctx.clearRect(0, 0, W, H);
    if (!dragging) { if (Math.abs(vel) > 0.0005) { spin += vel; vel *= 0.94; } if (Math.abs(tvel) > 0.0005) { tilt += tvel; tvel *= 0.94; } }
    const yaw = -0.5 + (t / 120000) * Math.PI * 2 + spin;
    const pitch = 0.92 + tilt;
    const cy = Math.cos(yaw), sy = Math.sin(yaw), cp = Math.cos(pitch), sp = Math.sin(pitch);
    // the map's diagonal is ~157 cells; leave room so it never clips the frame while it turns
    const scale = Math.min(W, H * 1.7) / 172;
    const ox = W / 2, oy = H * 0.5;
    // the sweep: a plane crossing the map every 7 s, eased
    const ph = (t % 7000) / 7000;
    const e = ph < 0.5 ? 4 * ph * ph * ph : 1 - Math.pow(-2 * ph + 2, 3) / 2;
    const sweep = (-0.6 + e * 1.2) * C;
    for (const b of buckets) b.length = 0;
    found.length = 0;
    for (let i = 0; i < pts.length; i++) {
      const p = pts[i];
      const [sx, sy2, d] = project(p, cy, sy, cp, sp, scale, ox, oy);
      if (sx < -4 || sx > W + 4 || sy2 < -4 || sy2 > H + 4) continue;
      const lift = Math.max(0, 1 - Math.abs(p.x - sweep) / 6);
      const s = Math.max(1, (p.y > 0 ? 1.3 : 1.1) * d * (scale / 3.2));
      if (p.f) { found.push(sx, sy2, s * 1.4); continue; }
      const k = Math.min(STEPS - 1, Math.round((p.lvl + lift * 0.55) * (STEPS - 1)));
      buckets[k].push(sx, sy2, s);
    }
    for (let k = 0; k < STEPS; k++) {
      const b = buckets[k];
      if (!b.length) continue;
      ctx.fillStyle = pal[k];
      for (let i = 0; i < b.length; i += 3) ctx.fillRect(b[i], b[i + 1], b[i + 2], b[i + 2]);
    }
    ctx.fillStyle = lime;
    for (let i = 0; i < found.length; i += 3) ctx.fillRect(found[i], found[i + 1], found[i + 2], found[i + 2]);
    // callouts follow their points; the leader is 40px tall under the label
    callouts.forEach((el) => {
      const a = anchors[el.dataset.anchor];
      if (!a) return;
      const [sx, sy2] = project(a, cy, sy, cp, sp, scale, ox, oy);
      const [cw, ch] = sizes.get(el) || [el.offsetWidth, el.offsetHeight];
      const h = ch + 40;
      const x = Math.max(8, Math.min(W - cw - 8, sx - 12));
      const yy = Math.max(8, Math.min(H - h - 8, sy2 - h));
      el.style.transform = `translate(${Math.round(x)}px, ${Math.round(yy)}px)`;
    });
  }

  // run state: on screen, not hovered, not paused, motion allowed
  const loop = canvas.closest('[data-loop]');
  const st = { seen: false, held: reduce.matches };
  let raf = 0, t0 = performance.now(), offset = 26000;
  const tick = (now) => { last = offset + (now - t0); draw(last); raf = requestAnimationFrame(tick); };
  const sync = () => {
    const run = st.seen && !st.held && !document.hidden;
    if (run && !raf) { t0 = performance.now(); raf = requestAnimationFrame(tick); }
    if (!run && raf) { cancelAnimationFrame(raf); raf = 0; offset = last; }
  };
  reduce.addEventListener && reduce.addEventListener('change', () => { st.held = reduce.matches; sync(); });
  document.addEventListener('visibilitychange', sync);
  new IntersectionObserver(([e]) => { st.seen = e.isIntersecting; sync(); }).observe(canvas);
  if ('ResizeObserver' in window) new ResizeObserver(resize).observe(canvas); else window.addEventListener('resize', resize);
  last = offset;
  resize();
})();
