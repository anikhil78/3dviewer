/**
 * ransac.js — RANSAC-based flat surface detection + convex hull geometry
 *
 * Pipeline per surface:
 *   1. Random-sample 3 points → fit plane
 *   2. Count inliers within distThreshold
 *   3. Keep best plane across `iterations` trials
 *   4. Refine plane via least-squares (PCA of inlier covariance)
 *   5. Re-collect inliers; remove them from the pool
 *   6. Project inliers onto the plane, compute 2-D convex hull
 *   7. Lift hull back to 3-D; fan-triangulate for a mesh
 *
 * Large clouds (> MAX_RANSAC_PTS) are sub-sampled for the detection step
 * only — the final surface geometry still uses the full hull vertices.
 */

import * as THREE from 'three';

const MAX_RANSAC_PTS = 80_000;   // sample ceiling for performance

// ─── Public API ───────────────────────────────────────────────────────────────

/**
 * Detect flat surfaces in a point cloud using iterative RANSAC.
 *
 * @param {Float32Array} positions - Flat [x,y,z, …] array (may be the
 *   already-centered geometry buffer so surface meshes align with the cloud).
 * @param {object} opts
 * @param {number} opts.maxSurfaces
 * @param {number} opts.distThreshold  - Max point-to-plane distance (same units as positions)
 * @param {number} opts.iterations     - RANSAC trials per surface
 * @param {number} opts.minPointsPct   - Minimum inlier fraction (0–1) to accept a plane
 * @param {function} opts.onProgress   - (fraction 0-1, label) callback
 * @returns {Promise<SurfaceData[]>}
 */
export async function detectSurfaces(positions, {
  maxSurfaces   = 5,
  distThreshold = 0.03,
  iterations    = 200,
  minPointsPct  = 0.05,
  onProgress    = () => {},
} = {}) {

  const totalN = positions.length / 3;

  // Sub-sample for RANSAC if needed (only used for plane fitting)
  let ransacPos = positions;
  if (totalN > MAX_RANSAC_PTS) {
    ransacPos = uniformSubsample(positions, MAX_RANSAC_PTS);
  }

  const ransacN   = ransacPos.length / 3;
  const minInliers = Math.max(30, Math.floor(ransacN * minPointsPct));

  // Bit-mask of remaining (un-assigned) points
  const remaining = new Uint8Array(ransacN).fill(1);
  let remainingCount = ransacN;

  const results = [];

  for (let si = 0; si < maxSurfaces; si++) {
    onProgress(si / maxSurfaces, `Fitting surface ${si + 1} of ${maxSurfaces}…`);
    await yieldFrame();   // keep the UI responsive

    if (remainingCount < minInliers) break;

    const activeIdx = collectActive(remaining);
    if (activeIdx.length < minInliers) break;

    // ── RANSAC ──
    const best = runRANSAC(ransacPos, activeIdx, iterations, distThreshold);
    if (!best || best.inliers.length < minInliers) break;

    // ── Least-squares plane refinement ──
    const refined = refinePlane(ransacPos, best.inliers);

    // ── Re-collect inliers against refined plane ──
    const finalInliers = [];
    for (const idx of activeIdx) {
      const x = ransacPos[idx * 3], y = ransacPos[idx * 3 + 1], z = ransacPos[idx * 3 + 2];
      if (Math.abs(refined.nx * x + refined.ny * y + refined.nz * z + refined.d) < distThreshold) {
        finalInliers.push(idx);
      }
    }
    if (finalInliers.length < minInliers) break;

    // Remove inliers from pool
    for (const idx of finalInliers) { remaining[idx] = 0; remainingCount--; }

    // ── Build surface object ──
    const inlierPts = finalInliers.map(i =>
      new THREE.Vector3(ransacPos[i * 3], ransacPos[i * 3 + 1], ransacPos[i * 3 + 2])
    );
    const surface = buildSurface(inlierPts, refined);
    if (surface) results.push(surface);
  }

  return results;
}

/**
 * Build a Three.js BufferGeometry (double-sided, fan-triangulated polygon)
 * from a SurfaceData returned by detectSurfaces.
 *
 * @param {SurfaceData} surface
 * @returns {THREE.BufferGeometry}
 */
export function buildSurfaceGeometry(surface) {
  const { centroid, hull3D } = surface;
  const verts = [];

  // Fan triangulation from centroid
  for (let i = 0; i < hull3D.length; i++) {
    const j = (i + 1) % hull3D.length;
    verts.push(centroid.x, centroid.y, centroid.z);
    verts.push(hull3D[i].x, hull3D[i].y, hull3D[i].z);
    verts.push(hull3D[j].x, hull3D[j].y, hull3D[j].z);
  }

  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(verts), 3));
  geo.computeVertexNormals();
  return geo;
}

// ─── RANSAC core ──────────────────────────────────────────────────────────────

function runRANSAC(positions, activeIdx, iterations, distThreshold) {
  let bestPlane = null, bestInliers = [];
  const n = activeIdx.length;

  for (let iter = 0; iter < iterations; iter++) {
    // Sample 3 distinct indices
    const ia = activeIdx[rndInt(n)];
    let ib = activeIdx[rndInt(n)]; while (ib === ia) ib = activeIdx[rndInt(n)];
    let ic = activeIdx[rndInt(n)]; while (ic === ia || ic === ib) ic = activeIdx[rndInt(n)];

    const ax = positions[ia*3], ay = positions[ia*3+1], az = positions[ia*3+2];
    const bx = positions[ib*3], by = positions[ib*3+1], bz = positions[ib*3+2];
    const cx = positions[ic*3], cy = positions[ic*3+1], cz = positions[ic*3+2];

    // Cross product → normal
    let nx = (by - ay) * (cz - az) - (bz - az) * (cy - ay);
    let ny = (bz - az) * (cx - ax) - (bx - ax) * (cz - az);
    let nz = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
    const len = Math.sqrt(nx*nx + ny*ny + nz*nz);
    if (len < 1e-10) continue;
    nx /= len; ny /= len; nz /= len;
    const d = -(nx * ax + ny * ay + nz * az);

    // Count inliers
    const inliers = [];
    for (const idx of activeIdx) {
      const x = positions[idx*3], y = positions[idx*3+1], z = positions[idx*3+2];
      if (Math.abs(nx*x + ny*y + nz*z + d) < distThreshold) inliers.push(idx);
    }

    if (inliers.length > bestInliers.length) {
      bestInliers = inliers;
      bestPlane   = { nx, ny, nz, d };
    }
  }

  return bestPlane ? { plane: bestPlane, inliers: bestInliers } : null;
}

// ─── Plane refinement via PCA ─────────────────────────────────────────────────

function refinePlane(positions, inliers) {
  // Centroid
  let cx = 0, cy = 0, cz = 0;
  for (const i of inliers) {
    cx += positions[i*3]; cy += positions[i*3+1]; cz += positions[i*3+2];
  }
  cx /= inliers.length; cy /= inliers.length; cz /= inliers.length;

  // 3×3 covariance matrix (upper triangle)
  let xx=0, xy=0, xz=0, yy=0, yz=0, zz=0;
  for (const i of inliers) {
    const dx = positions[i*3] - cx;
    const dy = positions[i*3+1] - cy;
    const dz = positions[i*3+2] - cz;
    xx += dx*dx; xy += dx*dy; xz += dx*dz;
    yy += dy*dy; yz += dy*dz; zz += dz*dz;
  }

  // Smallest eigenvector of cov = surface normal
  const [nx, ny, nz] = smallestEigenvector([
    [xx, xy, xz],
    [xy, yy, yz],
    [xz, yz, zz],
  ]);

  return {
    nx, ny, nz,
    d: -(nx * cx + ny * cy + nz * cz),
    cx, cy, cz,
  };
}

/**
 * Find the smallest eigenvector of a symmetric 3×3 matrix using
 * shift-and-invert power iteration:
 *   largest eigenvector of (trace·I − M)  ≡  smallest eigenvector of M
 */
function smallestEigenvector(m) {
  const trace = m[0][0] + m[1][1] + m[2][2];
  // Initial guess: normalised (1,1,1)
  let v = [0.577350269, 0.577350269, 0.577350269];

  for (let k = 0; k < 40; k++) {
    const w = [
      (trace - m[0][0]) * v[0]  -  m[0][1] * v[1]  -  m[0][2] * v[2],
      -m[1][0] * v[0]  + (trace - m[1][1]) * v[1]  -  m[1][2] * v[2],
      -m[2][0] * v[0]  -  m[2][1] * v[1]  + (trace - m[2][2]) * v[2],
    ];
    const len = Math.sqrt(w[0]*w[0] + w[1]*w[1] + w[2]*w[2]);
    if (len < 1e-12) break;
    v = [w[0] / len, w[1] / len, w[2] / len];
  }
  return v;
}

// ─── Surface geometry building ────────────────────────────────────────────────

/**
 * @typedef {object} SurfaceData
 * @property {object} plane       - { nx, ny, nz, d, cx, cy, cz }
 * @property {THREE.Vector3} normal
 * @property {THREE.Vector3} centroid
 * @property {THREE.Vector3[]} hull3D
 * @property {THREE.Vector3} tangent1
 * @property {THREE.Vector3} tangent2
 * @property {number} area         - m² (same units as input positions)
 * @property {number} pointCount
 */

function buildSurface(inlierPts, plane) {
  if (inlierPts.length < 3) return null;

  const centroid = new THREE.Vector3(plane.cx, plane.cy, plane.cz);
  const normal   = new THREE.Vector3(plane.nx, plane.ny, plane.nz).normalize();

  // Build an orthonormal basis {t1, t2} spanning the plane
  let t1 = new THREE.Vector3(0, 0, 1);
  if (Math.abs(normal.dot(t1)) > 0.85) t1.set(1, 0, 0);
  t1 = new THREE.Vector3().crossVectors(t1, normal).normalize();
  const t2 = new THREE.Vector3().crossVectors(normal, t1).normalize();

  // Project all inliers to 2-D plane coordinates
  const pts2D = inlierPts.map(p => {
    const v = p.clone().sub(centroid);
    return [v.dot(t1), v.dot(t2)];
  });

  // 2-D convex hull → back to 3-D
  const hull2D = convexHull2D(pts2D);
  if (hull2D.length < 3) return null;

  const hull3D = hull2D.map(([u, v]) =>
    centroid.clone().addScaledVector(t1, u).addScaledVector(t2, v)
  );

  // Surface area via fan triangulation from centroid
  let area = 0;
  for (let i = 0; i < hull3D.length; i++) {
    const j = (i + 1) % hull3D.length;
    const e1 = hull3D[i].clone().sub(centroid);
    const e2 = hull3D[j].clone().sub(centroid);
    area += new THREE.Vector3().crossVectors(e1, e2).length() * 0.5;
  }

  return { plane, normal, centroid, hull3D, tangent1: t1, tangent2: t2, area, pointCount: inlierPts.length };
}

// ─── 2-D Convex Hull (Andrew's monotone chain) ───────────────────────────────

function convexHull2D(pts) {
  if (pts.length < 3) return pts.slice();

  // Sort by x then y
  const sorted = pts.slice().sort((a, b) => a[0] - b[0] || a[1] - b[1]);

  const cross = (o, a, b) =>
    (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);

  const lower = [];
  for (const p of sorted) {
    while (lower.length >= 2 && cross(lower.at(-2), lower.at(-1), p) <= 0) lower.pop();
    lower.push(p);
  }
  const upper = [];
  for (let i = sorted.length - 1; i >= 0; i--) {
    const p = sorted[i];
    while (upper.length >= 2 && cross(upper.at(-2), upper.at(-1), p) <= 0) upper.pop();
    upper.push(p);
  }

  lower.pop(); upper.pop();
  return lower.concat(upper);
}

// ─── Utilities ────────────────────────────────────────────────────────────────

// Random reservoir sample — avoids the spatial bias that uniform stride
// introduces when points are stored in scan-line / surface order.
function uniformSubsample(positions, maxPts) {
  const total = positions.length / 3;

  // Fisher-Yates reservoir: pick maxPts distinct indices at random
  const indices = new Int32Array(maxPts);
  for (let i = 0; i < maxPts; i++) indices[i] = i;
  for (let i = maxPts; i < total; i++) {
    const j = Math.floor(Math.random() * (i + 1));
    if (j < maxPts) indices[j] = i;
  }

  const out = new Float32Array(maxPts * 3);
  for (let i = 0; i < maxPts; i++) {
    const src = indices[i] * 3;
    out[i*3]   = positions[src];
    out[i*3+1] = positions[src + 1];
    out[i*3+2] = positions[src + 2];
  }
  return out;
}

function collectActive(mask) {
  const list = [];
  for (let i = 0; i < mask.length; i++) if (mask[i]) list.push(i);
  return list;
}

function rndInt(n) { return Math.floor(Math.random() * n); }
function yieldFrame() { return new Promise(r => setTimeout(r, 0)); }
