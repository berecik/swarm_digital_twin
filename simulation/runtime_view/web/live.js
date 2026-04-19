import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

// ── Scene setup ────────────────────────────────────────────────────

const canvas = document.getElementById('viewport');
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x04081a);
scene.fog = new THREE.Fog(0x04081a, 150, 500);

const camera = new THREE.PerspectiveCamera(
  60,
  window.innerWidth / window.innerHeight,
  0.1,
  5000
);
camera.position.set(15, 20, 15);

const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);
renderer.setSize(window.innerWidth, window.innerHeight);

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.08;
controls.target.set(0, 0, 0);
controls.minDistance = 3;
controls.maxDistance = 500;

// ── Lighting ───────────────────────────────────────────────────────

scene.add(new THREE.HemisphereLight(0x8899ff, 0x0c0f22, 0.6));
const dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
dirLight.position.set(30, 40, 20);
scene.add(dirLight);

// ── Ground grid ────────────────────────────────────────────────────

const grid = new THREE.GridHelper(500, 50, 0x2a3360, 0x161b3d);
grid.position.y = 0;
scene.add(grid);

const groundGeo = new THREE.PlaneGeometry(500, 500);
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
// Reusable scratch vectors — avoids per-frame allocation in camera follow.
const _tmpVec = new THREE.Vector3();
const _tmpVec2 = new THREE.Vector3();
let _firstSampleReceived = false;
let _cameraFollowActive = true;

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

  // Scale factor — larger than real life so drones stay visible from distance.
  const S = 1.5;

  const body = new THREE.Mesh(
    new THREE.BoxGeometry(0.6 * S, 0.15 * S, 0.6 * S),
    new THREE.MeshStandardMaterial({
      color, metalness: 0.3, roughness: 0.5,
      emissive: color, emissiveIntensity: 0.1,
    })
  );
  group.add(body);

  const armGeo = new THREE.CylinderGeometry(0.04 * S, 0.04 * S, 1.0 * S, 8);
  const armMat = new THREE.MeshStandardMaterial({ color: 0x7a809e });
  for (const [dx, dz] of [[1, 1], [1, -1], [-1, 1], [-1, -1]]) {
    const arm = new THREE.Mesh(armGeo, armMat);
    arm.rotation.z = Math.PI / 2;
    arm.position.set(dx * 0.35 * S, 0, dz * 0.35 * S);
    arm.rotation.y = Math.atan2(dz, dx);
    group.add(arm);

    const rotor = new THREE.Mesh(
      new THREE.CylinderGeometry(0.3 * S, 0.3 * S, 0.04 * S, 16),
      new THREE.MeshStandardMaterial({ color, emissive: color, emissiveIntensity: 0.3 })
    );
    rotor.position.set(dx * 0.55 * S, 0.1 * S, dz * 0.55 * S);
    group.add(rotor);
  }

  const nose = new THREE.Mesh(
    new THREE.ConeGeometry(0.1 * S, 0.25 * S, 16),
    new THREE.MeshStandardMaterial({ color: 0xec4899, emissive: 0xec4899, emissiveIntensity: 0.2 })
  );
  nose.rotation.x = Math.PI / 2;
  nose.position.set(0, 0, 0.4 * S);
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
  sprite.position.set(0, 1.5 * S, 0);
  sprite.scale.set(3.0, 1.5, 1);
  group.add(sprite);

  scene.add(group);

  // Trail
  const trailPositions = new Float32Array(TRAIL_MAX * 3);
  const trailGeom = new THREE.BufferGeometry();
  trailGeom.setAttribute('position', new THREE.BufferAttribute(trailPositions, 3));
  const trailLine = new THREE.Line(
    trailGeom,
    new THREE.LineBasicMaterial({ color, transparent: true, opacity: 0.7 })
  );
  scene.add(trailLine);

  // Glowing point light on each drone so it's visible from any distance.
  const beacon = new THREE.PointLight(color, 0.8, 50);
  beacon.position.set(0, 0.3, 0);
  group.add(beacon);

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
  // Skip computeBoundingSphere() — the camera follows the drones so
  // trails are always in view; the default infinite sphere suffices.
}

// ── Waypoint markers ──────────────────────────────────────────────

const waypointGroup = new THREE.Group();
scene.add(waypointGroup);
let waypointPaths = [];

let originEnu = null;

/** Convert ENU (East, North, Up) to Three.js (X=East, Y=Up, Z=-North). */
function enuToScene(ex, ny, uz, origin) {
  return {
    x: ex - origin[0],
    y: uz - origin[2],
    z: -(ny - origin[1]),
  };
}
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
      const { x: rx, y: ry, z: rz } = enuToScene(wp[0], wp[1], wp[2], origin);

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

// ── Create default drone immediately so it's visible on page load ────
// The mesh will be repositioned once real telemetry arrives.
createDroneMesh(1);

// Fetch waypoints from the server. When the server publishes a
// multi-drone dict, also pre-create a placeholder mesh for each
// drone_id so swarm flights show all drones at the origin during the
// SITL boot phase (before any telemetry arrives).
function _refreshWaypoints() {
  return fetch('/api/waypoints')
    .then(r => r.json())
    .then(data => {
      if (!data) return;
      const isList = Array.isArray(data);
      if ((isList && data.length === 0) || (!isList && Object.keys(data).length === 0)) {
        return;
      }
      rawWaypointsData = data;
      if (!isList) {
        for (const droneIdStr of Object.keys(data)) {
          const id = parseInt(droneIdStr, 10);
          if (Number.isFinite(id)) getDrone(id);
        }
      }
      renderAllWaypoints(data);
    })
    .catch(() => {});
}
_refreshWaypoints();
// Re-poll once after SITL has had time to publish — cheap insurance for
// launchers that POST waypoints a few seconds after the page loads.
setTimeout(_refreshWaypoints, 4000);

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

  const { x, y, z } = enuToScene(ex, ny, uz, originEnu);

  const ds = getDrone(droneId);
  ds.group.position.set(x, y, z);

  const [roll, pitch, yaw] = s.euler || [0, 0, 0];
  ds.group.rotation.set(pitch, -yaw, roll);

  pushDroneTrail(ds, x, y, z);

  // ── Camera auto-follow ────────────────────────────────────────
  if (!_cameraFollowActive) return; // user disabled via OrbitControls drag

  // Compute the centroid of all visible drones.
  _tmpVec.set(0, 0, 0);
  for (const d of drones.values()) _tmpVec.add(d.group.position);
  _tmpVec.divideScalar(drones.size || 1);

  if (!_firstSampleReceived) {
    // SNAP camera on first data — don't lerp, just jump so the
    // drone is immediately visible.
    _firstSampleReceived = true;
    controls.target.copy(_tmpVec);
    // Position the camera behind and above the centroid.
    camera.position.set(
      _tmpVec.x - 10,
      _tmpVec.y + 15,
      _tmpVec.z + 20,
    );
    controls.update();
  } else if (drones.size === 1) {
    // Single drone: strong follow so it never leaves the screen.
    const dist = controls.target.distanceTo(_tmpVec);
    const lerpFactor = Math.min(0.3, 0.1 + dist * 0.01);
    controls.target.lerp(_tmpVec, lerpFactor);
    // Also move the camera to track, keeping a steady offset.
    _tmpVec2.copy(camera.position).sub(controls.target);
    camera.position.copy(_tmpVec).add(_tmpVec2);
  } else {
    // Multi-drone: follow centroid more gently.
    const dist = controls.target.distanceTo(_tmpVec);
    const lerpFactor = Math.min(0.15, 0.03 + dist * 0.005);
    controls.target.lerp(_tmpVec, lerpFactor);
  }

  // Auto-zoom: if all drones are in a tight cluster, move closer;
  // if spread wide, pull back.
  if (drones.size > 1) {
    let maxSpread = 0;
    for (const d of drones.values()) {
      const dd = d.group.position.distanceTo(_tmpVec);
      if (dd > maxSpread) maxSpread = dd;
    }
    const idealDist = Math.max(15, maxSpread * 2.5);
    const camDist = camera.position.distanceTo(controls.target);
    if (Math.abs(camDist - idealDist) > 5) {
      // Gently adjust distance along the camera→target direction.
      _tmpVec2.copy(camera.position).sub(controls.target).normalize();
      const newDist = camDist + (idealDist - camDist) * 0.02;
      camera.position.copy(controls.target).addScaledVector(_tmpVec2, newDist);
    }
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

// Pause auto-follow when user drags the camera manually.
controls.addEventListener('start', () => { _cameraFollowActive = false; });
// Double-click re-enables auto-follow.
canvas.addEventListener('dblclick', () => {
  _cameraFollowActive = true;
  _firstSampleReceived = false;  // triggers snap on next sample
});

function animate() {
  requestAnimationFrame(animate);
  controls.update();
  renderer.render(scene, camera);
}
animate();
