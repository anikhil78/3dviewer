/**
 * viewer.js — Three.js 3-D point cloud viewer
 *
 * Responsibilities:
 *   • Set up scene, camera, renderer, orbit controls
 *   • Render a point cloud (Points object) with height/distance/RGB coloring
 *   • Add / remove / highlight semi-transparent surface meshes
 *   • Raycasting for surface click selection
 *   • Expose simple API to app.js
 */

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { buildSurfaceGeometry } from './ransac.js';

// Palette for up to 10 surfaces (additional surfaces cycle)
const PALETTE = [
  0x4488ff, 0xff6644, 0x44ff88, 0xffcc44, 0xff44cc,
  0x44ccff, 0x88ff44, 0xff8844, 0xcc44ff, 0x44ffcc,
];

export class PointCloudViewer {
  /**
   * @param {HTMLCanvasElement} canvas
   */
  constructor(canvas) {
    this._canvas     = canvas;
    this._surfaceObjs = [];  // [{ mesh, wireframe, idx }]
    this._clickCb    = null;
    this._downPos    = { x: 0, y: 0 };

    this._initScene();
    this._initEvents();
    this._animate();
  }

  // ─── Scene setup ────────────────────────────────────────────────────────────

  _initScene() {
    const vp = this._canvas.parentElement;

    // Scene
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x08080f);

    // Grid + axes (subtle decoration)
    const grid = new THREE.GridHelper(10, 20, 0x191928, 0x111120);
    this.scene.add(grid);

    const axes = new THREE.AxesHelper(0.5);
    axes.material.opacity   = 0.35;
    axes.material.transparent = true;
    this.scene.add(axes);

    // Camera
    this.camera = new THREE.PerspectiveCamera(
      58,
      vp.clientWidth / vp.clientHeight,
      0.0005,
      5000
    );
    this.camera.position.set(0, 2, 4);

    // Renderer
    this.renderer = new THREE.WebGLRenderer({ canvas: this._canvas, antialias: true });
    this.renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
    this.renderer.setSize(vp.clientWidth, vp.clientHeight);

    // Orbit controls
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping    = true;
    this.controls.dampingFactor    = 0.07;
    this.controls.screenSpacePanning = true;
    this.controls.mouseButtons     = {
      LEFT:   THREE.MOUSE.ROTATE,
      MIDDLE: THREE.MOUSE.DOLLY,
      RIGHT:  THREE.MOUSE.PAN,
    };

    // Lights (only matter for MeshLambertMaterial on surface meshes)
    this.scene.add(new THREE.AmbientLight(0xffffff, 0.65));
    const sun = new THREE.DirectionalLight(0xffffff, 0.55);
    sun.position.set(3, 5, 4);
    this.scene.add(sun);

    // Raycaster
    this._raycaster = new THREE.Raycaster();
    this._mouse = new THREE.Vector2();
  }

  _animate() {
    this._rafId = requestAnimationFrame(() => this._animate());
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  }

  /** Call after the viewport element is resized. */
  onResize() {
    const vp = this._canvas.parentElement;
    this.camera.aspect = vp.clientWidth / vp.clientHeight;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(vp.clientWidth, vp.clientHeight);
  }

  // ─── Point cloud ────────────────────────────────────────────────────────────

  /**
   * Load and display a point cloud.  Replaces any previously displayed cloud.
   * Applies a centering translation so the cloud sits at the scene origin.
   *
   * @param {Float32Array} positions  - flat [x,y,z, …]
   * @param {Float32Array|null} colors - flat [r,g,b, …] in [0,1], or null
   * @param {'height'|'distance'|'rgb'} colorMode
   * @returns {{ center: THREE.Vector3, size: number }}
   *   The centering offset and bounding-box diagonal, useful for surface alignment.
   */
  loadPointCloud(positions, colors, colorMode = 'height') {
    if (this._pointCloud) {
      this.scene.remove(this._pointCloud);
      this._pointCloud.geometry.dispose();
      this._pointCloud.material.dispose();
      this._pointCloud = null;
    }

    this._rawPositions = positions;
    this._rawColors    = colors;

    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(positions.slice(), 3));
    geo.setAttribute('color',    new THREE.BufferAttribute(this._computeColors(colorMode), 3));

    // Center the cloud in the scene
    geo.computeBoundingBox();
    const center = new THREE.Vector3();
    geo.boundingBox.getCenter(center);
    geo.translate(-center.x, -center.y, -center.z);
    this._cloudCenter = center.clone();

    const mat = new THREE.PointsMaterial({
      size:            0.01,
      vertexColors:    true,
      sizeAttenuation: true,
      transparent:     true,
      opacity:         0.85,
      depthWrite:      false,
    });

    this._pointCloud = new THREE.Points(geo, mat);
    this.scene.add(this._pointCloud);

    // Fit camera
    const box  = new THREE.Box3().setFromObject(this._pointCloud);
    const size = box.getSize(new THREE.Vector3()).length();
    const ctr  = box.getCenter(new THREE.Vector3());
    this.controls.target.copy(ctr);
    this.camera.position
      .copy(ctr)
      .addScaledVector(new THREE.Vector3(0.45, 0.65, 1).normalize(), size * 1.6);
    this.controls.update();

    // Scale raycaster threshold to cloud density
    this._raycaster.params.Points.threshold = size * 0.0025;
    this._cloudSize = size;

    return { center, size };
  }

  // ── Display tweaks ───────────────────────────────────────────────────────────

  setPointSize(size) {
    if (this._pointCloud) this._pointCloud.material.size = size * 0.005;
  }

  setPointOpacity(opacity) {
    if (this._pointCloud) this._pointCloud.material.opacity = +opacity;
  }

  setColorMode(mode) {
    if (!this._pointCloud) return;
    const attr = this._pointCloud.geometry.attributes.color;
    attr.array.set(this._computeColors(mode));
    attr.needsUpdate = true;
  }

  setSurfacesVisible(visible) {
    for (const { mesh, wireframe } of this._surfaceObjs) {
      mesh.visible = wireframe.visible = visible;
    }
  }

  // ── Color computation ────────────────────────────────────────────────────────

  _computeColors(mode) {
    const pos = this._rawPositions;
    const n   = pos.length / 3;
    const out = new Float32Array(n * 3);

    if (mode === 'rgb' && this._rawColors) {
      out.set(this._rawColors);
      return out;
    }

    // Find value range
    let lo = Infinity, hi = -Infinity;
    for (let i = 0; i < n; i++) {
      const v = mode === 'height'
        ? pos[i * 3 + 1]
        : Math.hypot(pos[i*3], pos[i*3+1], pos[i*3+2]);
      if (v < lo) lo = v;
      if (v > hi) hi = v;
    }
    const range = hi - lo || 1;

    for (let i = 0; i < n; i++) {
      const v = mode === 'height'
        ? pos[i * 3 + 1]
        : Math.hypot(pos[i*3], pos[i*3+1], pos[i*3+2]);
      const [r, g, b] = turboColor((v - lo) / range);
      out[i*3] = r; out[i*3+1] = g; out[i*3+2] = b;
    }
    return out;
  }

  // ─── Surface meshes ──────────────────────────────────────────────────────────

  /**
   * Add a surface mesh to the scene.
   *
   * @param {SurfaceData} surfaceData  - from ransac.detectSurfaces
   * @param {number} idx               - index (determines palette color)
   * @returns {string} hex color string e.g. '#4488ff'
   */
  addSurface(surfaceData, idx) {
    const geo = buildSurfaceGeometry(surfaceData);
    // NOTE: no centering translation here — RANSAC runs on the already-centered
    // position buffer (loadPointCloud translates in-place), so hull3D coordinates
    // are already in scene space. Applying cloudCenter again would double-shift.

    const color = new THREE.Color(PALETTE[idx % PALETTE.length]);

    const mat = new THREE.MeshLambertMaterial({
      color,
      transparent: true,
      opacity:     0.32,
      side:        THREE.DoubleSide,
      depthWrite:  false,
    });
    const mesh = new THREE.Mesh(geo, mat);
    mesh.userData.surfaceIdx = idx;
    this.scene.add(mesh);

    // Wireframe overlay
    const wfMat = new THREE.LineBasicMaterial({
      color, transparent: true, opacity: 0.55,
    });
    const wireframe = new THREE.LineSegments(
      new THREE.WireframeGeometry(geo),
      wfMat
    );
    this.scene.add(wireframe);

    this._surfaceObjs.push({ mesh, wireframe, idx });

    return '#' + color.getHexString();
  }

  /** Remove all surface meshes from the scene. */
  clearSurfaces() {
    for (const { mesh, wireframe } of this._surfaceObjs) {
      this.scene.remove(mesh);
      mesh.geometry.dispose(); mesh.material.dispose();
      this.scene.remove(wireframe);
      wireframe.geometry.dispose(); wireframe.material.dispose();
    }
    this._surfaceObjs = [];
  }

  /**
   * Visually highlight or un-highlight a surface.
   *
   * @param {number} idx
   * @param {boolean} selected
   */
  selectSurface(idx, selected) {
    const entry = this._surfaceObjs.find(e => e.idx === idx);
    if (!entry) return;
    entry.mesh.material.opacity = selected ? 0.62 : 0.32;
    entry.mesh.material.emissive = new THREE.Color(selected ? 0x111133 : 0x000000);
  }

  // ─── Click handling ──────────────────────────────────────────────────────────

  /**
   * Register a callback fired when the user clicks a surface mesh.
   * @param {function(idx: number): void} cb
   */
  onSurfaceClick(cb) { this._clickCb = cb; }

  _initEvents() {
    window.addEventListener('resize', () => this.onResize());

    this._canvas.addEventListener('mousedown', e => {
      this._downPos = { x: e.clientX, y: e.clientY };
    });

    this._canvas.addEventListener('click', e => {
      // Ignore drags
      const dx = e.clientX - this._downPos.x;
      const dy = e.clientY - this._downPos.y;
      if (Math.hypot(dx, dy) > 5) return;
      if (!this._clickCb || this._surfaceObjs.length === 0) return;

      const rect = this._canvas.getBoundingClientRect();
      this._mouse.set(
        ((e.clientX - rect.left) / rect.width)  *  2 - 1,
        -((e.clientY - rect.top)  / rect.height) *  2 + 1,
      );
      this._raycaster.setFromCamera(this._mouse, this.camera);

      const meshes = this._surfaceObjs.map(e => e.mesh);
      const hits   = this._raycaster.intersectObjects(meshes, false);
      if (hits.length > 0) {
        this._clickCb(hits[0].object.userData.surfaceIdx);
      }
    });
  }
}

// ─── Turbo colormap (compact polynomial approximation) ───────────────────────
// Maps t ∈ [0,1] → [r,g,b] ∈ [0,1]  (blue → cyan → green → yellow → red)

function turboColor(t) {
  t = Math.max(0, Math.min(1, t));
  // Coefficients from google/turbo_colormap
  const r = clamp01(
    0.1357 + t * (4.5974 - t * (42.3277 - t * (130.5887 - t * (150.5799 - t * 58.1312))))
  );
  const g = clamp01(
    0.0914 + t * (2.1856 + t * (4.8052 - t * (14.0557 - t * (4.2652 - t * (-3.4533)))))
  );
  const b = clamp01(
    0.1075 + t * (7.0875 + t * (-31.7152 + t * (81.3862 + t * (-86.5527 + t * 34.0591))))
  );
  return [r, g, b];
}

function clamp01(x) { return Math.max(0, Math.min(1, x)); }
