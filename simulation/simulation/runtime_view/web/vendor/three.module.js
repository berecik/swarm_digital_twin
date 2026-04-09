// Minimal Three.js stub to allow app to load without error in test environment
// In a real environment, the user should replace this with the real three.module.js
export const Scene = class { add() {} };
export const PerspectiveCamera = class { position = { set() {} }; updateProjectionMatrix() {} };
export const WebGLRenderer = class { setSize() {}; render() {}; domElement = document.createElement('div') };
export const GridHelper = class {};
export const BoxGeometry = class {};
export const MeshPhongMaterial = class {};
export const Mesh = class { position = { set() {} }; rotation = { set() {} } };
export const DirectionalLight = class { position = { set() {} } };
export const AmbientLight = class {};
export const Color = class {};
