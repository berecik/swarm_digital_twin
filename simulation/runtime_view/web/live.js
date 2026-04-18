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

// ── Multi-drone state ─────────────────────────────────────────────

// Per-drone colour palette (up to 8; wraps for more).
const DRONE_COLORS = [
  0x22d3ee, // cyan
  0x2ed47a, // green
  0xec4899, // pink
  0xf59e0b, // amber
  0x8b5cf6, // violet
  0xef4444, // red
  0x3b82f6, // blue
  0x14b8a6, // teal
];
const TRAIL_MAX = 1000;

// Map<droneId, { group, trail, trailPositions, trailCount, color }>
const drones = new Map();

function createDroneMesh(droneId) {
  const colorIdx = (droneId - 1) % DRONE_COLORS.length;
  const color = DRONE_COLORS[colorIdx];

  const group = new THREE.Group();
  group.name = `drone-${droneId}`;

  const body = new THREE.Mesh(
    new THREE.BoxGeometry(0.6, 0.15, 0.6),
    new THREE.MeshStandardMaterial({ color, metalness: 0.3, roughness: 0.5 })
  );
  group.add(body);

  const armGeo = new THREE.CylinderGeometry(0.03, 0.03, 1.0, 8);
  const armMat = new THREE.MeshStandardMaterial({ color: 0x7a809e });
  for (const [dx, dz] of [[1, 1], [1, -1], [-1, 1], [-1, -1]]) {
    const arm = new THREE.Mesh(armGeo, armMat);
    arm.rotation.z = Math.PI / 2;
    arm.position.set(dx * 0.35, 0, dz * 0.35);
    arm.rotation.y = Math.atan2(dz, dx);
    group.add(arm);

    const rotor = new THREE.Mesh(
      new THREE.CylinderGeometry(0.25, 0.25, 0.03, 16),
      new THREE.MeshStandardMaterial({ color, emissive: color, emissiveIntensity: 0.15 })
    );
    rotor.position.set(dx * 0.55, 0.08, dz * 0.55);
    group.add(rotor);
  }

  const nose = new THREE.Mesh(
    new THREE.ConeGeometry(0.08, 0.2, 16),
    new THREE.MeshStandardMaterial({ color: 0xec4899 })
  );
  nose.rotation.x = Math.PI / 2;
  nose.position.set(0, 0, 0.38);
  group.add(nose);

  // ID label above the drone
  const labelCanvas = document.createElement('canvas');
  labelCanvas.width = 128;
  labelCanvas.height = 64;
  const ctx = labelCanvas.getContext('2d');
  ctx.fillStyle = `#${color.toString(16).padStart(6, '0')}`;
  ctx.font = 'bold 40px monospace';
  ctx.textAlign = 'center';
  ctx.fillText(`D${droneId}`, 64, 46);
  const tex = new THREE.CanvasTexture(labelCanvas);
  const sprite = new THREE.Sprite(
    new THREE.SpriteMaterial({ map: tex, transparent: true })
  );
  sprite.position.set(0, 1.2, 0);
  sprite.scale.set(2.0, 1.0, 1);
  group.add(sprite);

  scene.add(group);

  // Trail
  const trailPositions = new Float32Array(TRAIL_MAX * 3);
  const trailGeom = new THREE.BufferGeometry();
  trailGeom.setAttribute('position', new THREE.BufferAttribute(trailPositions, 3));
  const trailLine = new THREE.Line(
    trailGeom,
    new THREE.LineBasicMaterial({ color, transparent: true, opacity: 0.6 })
  );
  scene.add(trailLine);

  const state = {
    group,
    trail: trailLine,
    trailGeom,
    trailPositions,
    trailCount: 0,
    color,
  };
  drones.set(droneId, state);
  return state;
}

function getDrone(droneId) {
  return drones.get(droneId) || createDroneMesh(droneId);
}

function pushDroneTrail(state, x, y, z) {
  const { trailPositions, trailGeom } = state;
  if (state.trailCount < TRAIL_MAX) {
    trailPositions[state.trailCount * 3 + 0] = x;
    trailPositions[state.trailCount * 3 + 1] = y;
    trailPositions[state.trailCount * 3 + 2] = z;
    state.trailCount += 1;
  } else {
    trailPositions.copyWithin(0, 3);
    trailPositions[(TRAIL_MAX - 1) * 3 + 0] = x;
    trailPositions[(TRAIL_MAX - 1) * 3 + 1] = y;
    trailPositions[(TRAIL_MAX - 1) * 3 + 2] = z;
  }
  trailGeom.setDrawRange(0, state.trailCount);
  trailGeom.attributes.position.needsUpdate = true;
  trailGeom.computeBoundingSphere();
}

// ── Waypoint markers ──────────────────────────────────────────────

const waypointGroup = new THREE.Group();
scene.add(waypointGroup);
let waypointPaths = [];

let originEnu = null;
let rawWaypointsData = null;  // dict or legacy list

function renderAllWaypoints(wpData) {
  // Clear previous
  while (waypointGroup.children.length) {
    waypointGroup.remove(waypointGroup.children[0]);
  }
  for (const p of waypointPaths) scene.remove(p);
  waypointPaths = [];

  if (!wpData) return;

  // Normalise: dict {droneId: [[x,y,z],...]} or legacy list [[x,y,z],...]
  let wpDict;
  if (Array.isArray(wpData)) {
    wpDict = { '1': wpData };
  } else {
    wpDict = wpData;
  }

  const origin = originEnu || [0, 0, 0];

  for (const [droneIdStr, waypoints] of Object.entries(wpDict)) {
    if (!Array.isArray(waypoints) || waypoints.length === 0) continue;
    const droneId = parseInt(droneIdStr, 10);
    const colorIdx = (droneId - 1) % DRONE_COLORS.length;
    const color = DRONE_COLORS[colorIdx];

    const wpMat = new THREE.MeshStandardMaterial({
      color: 0xfacc15,
      emissive: 0x6b5a00,
      metalness: 0.3,
      roughness: 0.5,
    });

    const pathPoints = [];

    waypoints.forEach((wp, i) => {
      const [ex, ny, uz] = wp;
      const rx = ex - origin[0];
      const ry = uz - origin[2];
      const rz = -(ny - origin[1]);

      const sphere = new THREE.Mesh(
        new THREE.SphereGeometry(0.35, 12, 8),
        wpMat,
      );
      sphere.position.set(rx, ry, rz);
      waypointGroup.add(sphere);

      if (ry > 0.2) {
        const pole = new THREE.Mesh(
          new THREE.CylinderGeometry(0.03, 0.03, ry, 6),
          new THREE.MeshStandardMaterial({ color: 0xfacc15, transparent: true, opacity: 0.25 }),
        );
        pole.position.set(rx, ry / 2, rz);
        waypointGroup.add(pole);
      }

      // Label
      const lc = document.createElement('canvas');
      lc.width = 128; lc.height = 64;
      const lctx = lc.getContext('2d');
      lctx.fillStyle = '#facc15';
      lctx.font = 'bold 30px monospace';
      lctx.textAlign = 'center';
      const label = drones.size > 1 ? `D${droneId}W${i+1}` : `WP${i+1}`;
      lctx.fillText(label, 64, 42);
      const tex = new THREE.CanvasTexture(lc);
      const sprite = new THREE.Sprite(
        new THREE.SpriteMaterial({ map: tex, transparent: true })
      );
      sprite.position.set(rx, ry + 1.0, rz);
      sprite.scale.set(2.2, 1.1, 1);
      waypointGroup.add(sprite);

      pathPoints.push(new THREE.Vector3(rx, ry, rz));
    });

    if (pathPoints.length >= 2) {
      const pathGeom = new THREE.BufferGeometry().setFromPoints(pathPoints);
      const pathMat = new THREE.LineDashedMaterial({
        color,
        dashSize: 1.0,
        gapSize: 0.5,
        transparent: true,
        opacity: 0.45,
      });
      const pathLine = new THREE.Line(pathGeom, pathMat);
      pathLine.computeLineDistances();
      scene.add(pathLine);
      waypointPaths.push(pathLine);
    }
  }
}

// Fetch waypoints from the server
fetch('/api/waypoints')
  .then(r => r.json())
  .then(data => {
    if (data && (Array.isArray(data) ? data.length > 0 : Object.keys(data).length > 0)) {
      rawWaypointsData = data;
      renderAllWaypoints(data);
    }
  })
  .catch(() => {});

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
  const droneId = s.drone_id || 1;
  const [ex, ny, uz] = s.pos_enu || [0, 0, 0];

  if (originEnu === null) {
    originEnu = [ex, ny, uz];
    if (rawWaypointsData) renderAllWaypoints(rawWaypointsData);
  }

  const x = ex - originEnu[0];
  const y = uz - originEnu[2];
  const z = -(ny - originEnu[1]);

  const ds = getDrone(droneId);
  ds.group.position.set(x, y, z);

  const [roll, pitch, yaw] = s.euler || [0, 0, 0];
  ds.group.rotation.set(pitch, -yaw, roll);

  pushDroneTrail(ds, x, y, z);

  // Camera follow — lerp toward centroid of all drones.
  if (drones.size === 1) {
    controls.target.lerp(new THREE.Vector3(x, y, z), 0.05);
  } else {
    const centroid = new THREE.Vector3();
    for (const d of drones.values()) centroid.add(d.group.position);
    centroid.divideScalar(drones.size);
    controls.target.lerp(centroid, 0.03);
  }

  // HUD updates (show the latest sample regardless of drone).
  const relUz = uz - originEnu[2];
  hud.agl.textContent = `${relUz.toFixed(1)} m`;
  hud.alt.textContent = `${(s.alt_msl || 0).toFixed(1)} m`;
  const spd = Math.hypot(s.vel_enu?.[0] || 0, s.vel_enu?.[1] || 0, s.vel_enu?.[2] || 0);
  hud.speed.textContent = `${spd.toFixed(1)} m/s`;
  const hdg = ((yaw * 180) / Math.PI + 360) % 360;
  hud.heading.textContent = `${hdg.toFixed(0)}°`;
  hud.throttle.textContent = `${(s.throttle_pct || 0).toFixed(0)}%`;
  hud.battV.textContent = `${(s.battery_voltage_v || 0).toFixed(2)} V`;
  hud.battPct.textContent = `${(s.battery_remaining_pct || 0).toFixed(0)}%`;
  hud.mode.textContent = `${s.flight_mode || 'UNKNOWN'}`;
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
