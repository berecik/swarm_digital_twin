import * as THREE from 'three';
import { OrbitControls } from 'three/addons/OrbitControls.js';

let scene, camera, renderer, controls, drone;
const hud = {
    status: document.getElementById('status'),
    pos: document.getElementById('pos'),
    alt: document.getElementById('alt'),
    yaw: document.getElementById('yaw'),
    mode: document.getElementById('mode'),
    batt: document.getElementById('batt')
};

function init() {
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x222222);

    camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
    camera.position.set(10, 10, 10);

    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(window.innerWidth, window.innerHeight);
    document.getElementById('canvas-container').appendChild(renderer.domElement);

    controls = new OrbitControls(camera, renderer.domElement);

    // Grid helper
    const grid = new THREE.GridHelper(100, 100, 0x444444, 0x888888);
    scene.add(grid);

    // Drone mesh (simple box)
    const geometry = new THREE.BoxGeometry(0.5, 0.2, 0.5);
    const material = new THREE.MeshPhongMaterial({ color: 0x0078d4 });
    drone = new THREE.Mesh(geometry, material);
    scene.add(drone);

    // Light
    const light = new THREE.DirectionalLight(0xffffff, 1);
    light.position.set(5, 10, 7.5);
    scene.add(light);
    scene.add(new THREE.AmbientLight(0x404040));

    window.addEventListener('resize', onWindowResize);

    connectWS();
    animate();
}

function onWindowResize() {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
}

function connectWS() {
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const ws = new WebSocket(`${protocol}//${window.location.host}/ws/telemetry`);

    ws.onopen = () => {
        hud.status.textContent = 'CONNECTED';
        hud.status.style.color = 'lime';
    };

    ws.onmessage = (event) => {
        const msg = JSON.parse(event.data);
        if (msg.type === 'sample') {
            updateDrone(msg.data);
        } else if (msg.type === 'snapshot') {
            if (msg.data.length > 0) {
                updateDrone(msg.data[msg.data.length - 1]);
            }
        }
    };

    ws.onclose = () => {
        hud.status.textContent = 'DISCONNECTED';
        hud.status.style.color = 'red';
        setTimeout(connectWS, 2000);
    };
}

function updateDrone(data) {
    // Update mesh (assuming ENU: X=East, Y=Up, Z=-North)
    // Actually our pos_enu is [X, Y, Z]
    drone.position.set(data.pos_enu[0], data.pos_enu[2], -data.pos_enu[1]);

    // Update orientation (Euler: roll, pitch, yaw)
    drone.rotation.set(data.euler[1], -data.euler[2], -data.euler[0]);

    // Update HUD
    hud.pos.textContent = `${data.pos_enu[0].toFixed(1)}, ${data.pos_enu[1].toFixed(1)}, ${data.pos_enu[2].toFixed(1)}`;
    hud.alt.textContent = `${data.alt_msl.toFixed(1)} m`;
    hud.yaw.textContent = `${(data.euler[2] * 180 / Math.PI).toFixed(0)} deg`;
    hud.mode.textContent = data.flight_mode;
    hud.batt.textContent = `${data.battery_remaining_pct}%`;
}

function animate() {
    requestAnimationFrame(animate);
    controls.update();
    renderer.render(scene, camera);
}

init();
