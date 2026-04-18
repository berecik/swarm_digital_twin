import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

// ── Scene setup ────────────────────────────────────────────────────

const canvas = document.getElementById('viewport');
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x04081a);
scene.fog = new THREE.Fog(0x04081a, 60, 220);

const camera = new THREE.PerspectiveCamera(
  60,
  window.innerWidth / window.innerHeight,
  0.1,
  2000
);
camera.position.set(25, 25, 25);

const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);
renderer.setSize(window.innerWidth, window.innerHeight);

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.08;
controls.target.set(0, 0, 0);

// ── Lighting ───────────────────────────────────────────────────────

scene.add(new THREE.HemisphereLight(0x8899ff, 0x0c0f22, 0.6));
const dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
dirLight.position.set(30, 40, 20);
scene.add(dirLight);

// ── Ground grid ────────────────────────────────────────────────────

const grid = new THREE.GridHelper(200, 40, 0x2a3360, 0x161b3d);
grid.position.y = 0;
scene.add(grid);

const groundGeo = new THREE.PlaneGeometry(200, 200);
const groundMat = new THREE.MeshStandardMaterial({
  color: 0x0b1130,
  metalness: 0.1,
  roughness: 0.95,
  transparent: true,
  opacity: 0.85,
});
const ground = new THREE.Mesh(groundGeo, groundMat);
ground.rotation.x = -Math.PI / 2;
scene.add(ground);

// ── Drone mesh (low-poly quadrotor) ───────────────────────────────

const drone = new THREE.Group();

const body = new THREE.Mesh(
  new THREE.BoxGeometry(0.6, 0.15, 0.6),
  new THREE.MeshStandardMaterial({ color: 0x22d3ee, metalness: 0.3, roughness: 0.5 })
);
drone.add(body);

const armGeo = new THREE.CylinderGeometry(0.03, 0.03, 1.0, 8);
const armMat = new THREE.MeshStandardMaterial({ color: 0x7a809e });
for (const [dx, dz] of [[1, 1], [1, -1], [-1, 1], [-1, -1]]) {
  const arm = new THREE.Mesh(armGeo, armMat);
  arm.rotation.z = Math.PI / 2;
  arm.position.set(dx * 0.35, 0, dz * 0.35);
  arm.rotation.y = Math.atan2(dz, dx);
  drone.add(arm);

  const rotor = new THREE.Mesh(
    new THREE.CylinderGeometry(0.25, 0.25, 0.03, 16),
    new THREE.MeshStandardMaterial({ color: 0x2ed47a, emissive: 0x0a3f20 })
  );
  rotor.position.set(dx * 0.55, 0.08, dz * 0.55);
  drone.add(rotor);
}

const nose = new THREE.Mesh(
  new THREE.ConeGeometry(0.08, 0.2, 16),
  new THREE.MeshStandardMaterial({ color: 0xec4899 })
);
nose.rotation.x = Math.PI / 2;
nose.position.set(0, 0, 0.38);
drone.add(nose);

scene.add(drone);

// ── Trail line ─────────────────────────────────────────────────────

const TRAIL_MAX = 1000;
const trailPositions = new Float32Array(TRAIL_MAX * 3);
const trailGeom = new THREE.BufferGeometry();
trailGeom.setAttribute('position', new THREE.BufferAttribute(trailPositions, 3));
const trailMat = new THREE.LineBasicMaterial({
  color: 0x22d3ee,
  transparent: true,
  opacity: 0.75,
});
const trail = new THREE.Line(trailGeom, trailMat);
scene.add(trail);
let trailCount = 0;
let originEnu = null;

function pushTrail(x, y, z) {
  if (trailCount < TRAIL_MAX) {
    trailPositions[trailCount * 3 + 0] = x;
    trailPositions[trailCount * 3 + 1] = y;
    trailPositions[trailCount * 3 + 2] = z;
    trailCount += 1;
  } else {
    // Shift left by one sample.
    trailPositions.copyWithin(0, 3);
    trailPositions[(TRAIL_MAX - 1) * 3 + 0] = x;
    trailPositions[(TRAIL_MAX - 1) * 3 + 1] = y;
    trailPositions[(TRAIL_MAX - 1) * 3 + 2] = z;
  }
  trailGeom.setDrawRange(0, trailCount);
  trailGeom.attributes.position.needsUpdate = true;
  trailGeom.computeBoundingSphere();
}

// ── Waypoint markers ──────────────────────────────────────────────

const waypointGroup = new THREE.Group();
scene.add(waypointGroup);

// Dashed line connecting waypoints (flight plan path)
let waypointPath = null;

// Raw ENU waypoints — kept so we can re-render once originEnu is known.
let rawWaypoints = null;

function renderWaypoints(waypoints) {
  // Clear previous markers
  while (waypointGroup.children.length) {
    waypointGroup.remove(waypointGroup.children[0]);
  }
  if (waypointPath) {
    scene.remove(waypointPath);
    waypointPath = null;
  }
  if (!waypoints || waypoints.length === 0) return;

  // Use the same origin as the drone (set by the first telemetry sample).
  // Fall back to the first waypoint if telemetry hasn't arrived yet.
  const origin = originEnu || waypoints[0];

  const wpMat = new THREE.MeshStandardMaterial({
    color: 0xfacc15,
    emissive: 0x6b5a00,
    metalness: 0.3,
    roughness: 0.5,
  });
  const poleMat = new THREE.MeshStandardMaterial({
    color: 0xfacc15,
    transparent: true,
    opacity: 0.35,
  });

  const pathPoints = [];

  waypoints.forEach((wp, i) => {
    // ENU → Three.js (same mapping as applySample)
    const [ex, ny, uz] = wp;
    const rx = ex - origin[0];
    const ry = uz - origin[2];
    const rz = -(ny - origin[1]);

    // Sphere marker at waypoint position
    const sphere = new THREE.Mesh(
      new THREE.SphereGeometry(0.4, 16, 12),
      wpMat,
    );
    sphere.position.set(rx, ry, rz);
    waypointGroup.add(sphere);

    // Thin vertical pole from ground to waypoint
    if (ry > 0.2) {
      const poleHeight = ry;
      const pole = new THREE.Mesh(
        new THREE.CylinderGeometry(0.04, 0.04, poleHeight, 6),
        poleMat,
      );
      pole.position.set(rx, poleHeight / 2, rz);
      waypointGroup.add(pole);
    }

    // Small ground ring
    const ring = new THREE.Mesh(
      new THREE.RingGeometry(0.3, 0.5, 16),
      new THREE.MeshStandardMaterial({
        color: 0xfacc15,
        transparent: true,
        opacity: 0.25,
        side: THREE.DoubleSide,
      }),
    );
    ring.rotation.x = -Math.PI / 2;
    ring.position.set(rx, 0.01, rz);
    waypointGroup.add(ring);

    // Label (sprite)
    const canvas = document.createElement('canvas');
    canvas.width = 128;
    canvas.height = 64;
    const ctx = canvas.getContext('2d');
    ctx.fillStyle = '#facc15';
    ctx.font = 'bold 36px monospace';
    ctx.textAlign = 'center';
    ctx.fillText(`WP${i + 1}`, 64, 42);
    const tex = new THREE.CanvasTexture(canvas);
    const spriteMat = new THREE.SpriteMaterial({ map: tex, transparent: true });
    const sprite = new THREE.Sprite(spriteMat);
    sprite.position.set(rx, ry + 1.2, rz);
    sprite.scale.set(2.5, 1.25, 1);
    waypointGroup.add(sprite);

    pathPoints.push(new THREE.Vector3(rx, ry, rz));
  });

  // Dashed line connecting all waypoints
  if (pathPoints.length >= 2) {
    const pathGeom = new THREE.BufferGeometry().setFromPoints(pathPoints);
    const pathMat = new THREE.LineDashedMaterial({
      color: 0xfacc15,
      dashSize: 1.0,
      gapSize: 0.5,
      transparent: true,
      opacity: 0.6,
    });
    waypointPath = new THREE.Line(pathGeom, pathMat);
    waypointPath.computeLineDistances();
    scene.add(waypointPath);
  }
}

// Fetch waypoints from the server
fetch('/api/waypoints')
  .then(r => r.json())
  .then(wps => {
    if (Array.isArray(wps) && wps.length > 0) {
      rawWaypoints = wps;
      renderWaypoints(wps);
    }
  })
  .catch(() => {});  // No waypoints available — that's fine

// ── HUD ────────────────────────────────────────────────────────────

const hud = {
  chip: document.querySelector('.live-status-chip'),
  chipValue: document.getElementById('status'),
  agl: document.getElementById('hud-agl'),
  alt: document.getElementById('hud-alt'),
  speed: document.getElementById('hud-speed'),
  heading: document.getElementById('hud-heading'),
  throttle: document.getElementById('hud-throttle'),
  battV: document.getElementById('hud-batt-v'),
  battPct: document.getElementById('hud-batt-pct'),
  mode: document.getElementById('hud-mode'),
};

function setStatus(state) {
  hud.chip.setAttribute('data-status', state);
  hud.chipValue.textContent = state.toUpperCase();
}
setStatus('connecting');

// ── Sample → scene mapping ────────────────────────────────────────

function applySample(s) {
  const [ex, ny, uz] = s.pos_enu || [0, 0, 0];
  // Recenter to the first valid telemetry fix so the drone remains in-view
  // even if the backend reference geodetic origin differs from the vehicle's
  // absolute location (e.g. SITL at a different latitude/longitude).
  if (originEnu === null) {
    originEnu = [ex, ny, uz];
    // Re-render waypoints now that we know the telemetry origin.
    if (rawWaypoints) renderWaypoints(rawWaypoints);
  }
  const relEx = ex - originEnu[0];
  const relNy = ny - originEnu[1];
  const relUz = uz - originEnu[2];
  // ENU (East, North, Up) → Three.js (X=East, Y=Up, Z=-North) so the
  // default OrbitControls camera sees a familiar orientation.
  const x = relEx;
  const y = relUz;
  const z = -relNy;

  drone.position.set(x, y, z);

  // Euler: roll, pitch, yaw (rad). Three.js expects intrinsic XYZ.
  const [roll, pitch, yaw] = s.euler || [0, 0, 0];
  drone.rotation.set(pitch, -yaw, roll);

  pushTrail(x, y, z);

  // Camera follow — gentle target lerp.
  controls.target.lerp(new THREE.Vector3(x, y, z), 0.05);

  // HUD updates
  hud.agl.textContent = `${relUz.toFixed(1)} m`;
  hud.alt.textContent = `${(s.alt_msl || 0).toFixed(1)} m`;
  const spd = Math.hypot(s.vel_enu?.[0] || 0, s.vel_enu?.[1] || 0, s.vel_enu?.[2] || 0);
  hud.speed.textContent = `${spd.toFixed(1)} m/s`;
  const hdg = ((yaw * 180) / Math.PI + 360) % 360;
  hud.heading.textContent = `${hdg.toFixed(0)}°`;
  hud.throttle.textContent = `${(s.throttle_pct || 0).toFixed(0)}%`;
  hud.battV.textContent = `${(s.battery_voltage_v || 0).toFixed(2)} V`;
  hud.battPct.textContent = `${(s.battery_remaining_pct || 0).toFixed(0)}%`;
  hud.mode.textContent = s.flight_mode || 'UNKNOWN';
}

// ── WebSocket consumer ────────────────────────────────────────────

let ws = null;
let reconnectTimer = null;

function connectWS() {
  if (reconnectTimer) {
    clearTimeout(reconnectTimer);
    reconnectTimer = null;
  }
  const proto = location.protocol === 'https:' ? 'wss:' : 'ws:';
  ws = new WebSocket(`${proto}//${location.host}/ws/telemetry`);

  ws.addEventListener('open', () => setStatus('connected'));

  ws.addEventListener('message', (ev) => {
    let msg;
    try {
      msg = JSON.parse(ev.data);
    } catch (err) {
      return;
    }
    if (msg.type === 'sample' && msg.data) {
      applySample(msg.data);
    } else if (msg.type === 'snapshot' && Array.isArray(msg.data)) {
      for (const s of msg.data) applySample(s);
    }
  });

  const onDrop = () => {
    setStatus('disconnected');
    reconnectTimer = setTimeout(connectWS, 2000);
  };
  ws.addEventListener('close', onDrop);
  ws.addEventListener('error', onDrop);
}

connectWS();

// ── Resize + render loop ──────────────────────────────────────────

window.addEventListener('resize', () => {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
});

function animate() {
  requestAnimationFrame(animate);
  controls.update();
  renderer.render(scene, camera);
}
animate();
