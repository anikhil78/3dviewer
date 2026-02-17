/**
 * parsers.js — Point cloud file format parsers
 *
 * Supported formats:
 *   PLY  — ASCII and binary (via Three.js PLYLoader)
 *   XYZ  — whitespace/comma-delimited x y z [r g b] per line
 *   PCD  — PCL ASCII format
 *   JSON — several common layouts (see parseJSON)
 *
 * Every parser resolves with:
 *   {
 *     positions: Float32Array,   // flat [x,y,z, x,y,z, …]
 *     colors:    Float32Array|null,  // flat [r,g,b, …] in [0,1], or null
 *     precomputedSurfaces: Array|null  // if the file already contains surfaces
 *   }
 */

import { PLYLoader } from 'three/addons/loaders/PLYLoader.js';

// ─── Public entry point ────────────────────────────────────────────────────────

/**
 * @param {File} file
 * @returns {Promise<ParseResult>}
 */
export async function parseFile(file) {
  const ext = file.name.split('.').pop().toLowerCase();
  switch (ext) {
    case 'ply':  return parsePLY(file);
    case 'xyz':
    case 'txt':  return parseXYZ(file);
    case 'pcd':  return parsePCD(file);
    case 'json': return parseJSON(file);
    default:
      throw new Error(
        `Unsupported file extension ".${ext}".\nSupported: PLY, XYZ, TXT, PCD, JSON`
      );
  }
}

// ─── PLY ──────────────────────────────────────────────────────────────────────

async function parsePLY(file) {
  return new Promise((resolve, reject) => {
    const loader = new PLYLoader();
    const url = URL.createObjectURL(file);
    loader.load(
      url,
      (geometry) => {
        URL.revokeObjectURL(url);
        const positions = Float32Array.from(geometry.attributes.position.array);
        const colors = geometry.attributes.color
          ? Float32Array.from(geometry.attributes.color.array)
          : null;
        resolve({ positions, colors, precomputedSurfaces: null });
      },
      undefined,
      (err) => {
        URL.revokeObjectURL(url);
        reject(new Error('PLY parse failed: ' + (err.message ?? err)));
      }
    );
  });
}

// ─── XYZ / TXT ────────────────────────────────────────────────────────────────
// Line format:  x y z [r g b]   (space, comma, or semicolon delimited)
// r,g,b may be 0-1 floats OR 0-255 integers — auto-detected.

async function parseXYZ(file) {
  const text = await file.text();
  const lines = text.trim().split(/\r?\n/);
  const positions = [];
  const colorsBuf = [];
  let hasColor = false;

  for (const line of lines) {
    const trimmed = line.trim();
    if (!trimmed || trimmed.startsWith('#') || trimmed.startsWith('//')) continue;
    const p = trimmed.split(/[\s,;]+/);
    if (p.length < 3) continue;
    const x = +p[0], y = +p[1], z = +p[2];
    if (!isFinite(x) || !isFinite(y) || !isFinite(z)) continue;
    positions.push(x, y, z);

    if (p.length >= 6) {
      let r = +p[3], g = +p[4], b = +p[5];
      // Treat values > 1 as 0-255
      if (r > 1 || g > 1 || b > 1) { r /= 255; g /= 255; b /= 255; }
      colorsBuf.push(r, g, b);
      hasColor = true;
    }
  }

  return {
    positions: new Float32Array(positions),
    colors: hasColor ? new Float32Array(colorsBuf) : null,
    precomputedSurfaces: null,
  };
}

// ─── PCD (PCL ASCII) ──────────────────────────────────────────────────────────

async function parsePCD(file) {
  const text = await file.text();
  const lines = text.split(/\r?\n/);

  let fields = [];
  let dataStart = -1;
  let isBinary = false;

  for (let i = 0; i < lines.length; i++) {
    const line = lines[i].trim();
    if (line.startsWith('FIELDS')) {
      fields = line.split(/\s+/).slice(1);
    } else if (line.startsWith('DATA')) {
      isBinary = line.includes('binary');
      dataStart = i + 1;
      break;
    }
  }

  if (dataStart === -1)
    throw new Error('PCD header not found. Only ASCII PCD is supported.');
  if (isBinary)
    throw new Error('Binary PCD is not supported. Please convert to ASCII PCD.');

  const fi = (name) => {
    let idx = fields.indexOf(name);
    if (idx === -1) idx = fields.findIndex(f => f.toLowerCase() === name);
    return idx;
  };
  const xi = fi('x'), yi = fi('y'), zi = fi('z');
  if (xi === -1) throw new Error('PCD file does not contain x, y, z fields.');

  const ri = Math.max(fi('r'), fi('red'));
  const gi = Math.max(fi('g'), fi('green'));
  const bi = Math.max(fi('b'), fi('blue'));
  const hasColor = ri !== -1 && gi !== -1 && bi !== -1;

  const positions = [], colorsBuf = [];
  for (let i = dataStart; i < lines.length; i++) {
    const parts = lines[i].trim().split(/\s+/);
    if (parts.length < Math.max(xi, yi, zi) + 1) continue;
    const x = +parts[xi], y = +parts[yi], z = +parts[zi];
    if (!isFinite(x)) continue;
    positions.push(x, y, z);
    if (hasColor) {
      let r = +parts[ri], g = +parts[gi], b = +parts[bi];
      if (r > 1 || g > 1 || b > 1) { r /= 255; g /= 255; b /= 255; }
      colorsBuf.push(r, g, b);
    }
  }

  return {
    positions: new Float32Array(positions),
    colors: hasColor ? new Float32Array(colorsBuf) : null,
    precomputedSurfaces: null,
  };
}

// ─── JSON ─────────────────────────────────────────────────────────────────────
// Several layouts are supported:
//   Format A: { "positions": [x,y,z,...], "colors": [...], "surfaces": [...] }
//   Format B: { "points": [ {x,y,z}, ... ] }
//   Format C: { "points": [ [x,y,z], ... ] }
//   Format D: root array  [ [x,y,z], ... ] or [ {x,y,z}, ... ]

async function parseJSON(file) {
  const text = await file.text();
  let data;
  try { data = JSON.parse(text); }
  catch (e) { throw new Error('Invalid JSON: ' + e.message); }

  // Format A — flat typed arrays
  if (data.positions && Array.isArray(data.positions)) {
    return {
      positions: new Float32Array(data.positions),
      colors: data.colors ? new Float32Array(data.colors) : null,
      precomputedSurfaces: data.surfaces ?? null,
    };
  }

  // Formats B / C / D — point list
  const pts = data.points ?? data.vertices ?? (Array.isArray(data) ? data : null);
  if (!pts || !Array.isArray(pts)) {
    throw new Error(
      'Cannot locate point data in JSON.\n' +
      'Expected root keys: "positions", "points", "vertices", or a root array.'
    );
  }

  const positions = [], colorsBuf = [];
  let hasColor = false;

  for (const p of pts) {
    if (Array.isArray(p)) {
      if (p.length < 3) continue;
      positions.push(+p[0], +p[1], +p[2]);
      if (p.length >= 6) {
        let r = +p[3], g = +p[4], b = +p[5];
        if (r > 1 || g > 1 || b > 1) { r /= 255; g /= 255; b /= 255; }
        colorsBuf.push(r, g, b); hasColor = true;
      }
    } else if (typeof p === 'object' && p !== null) {
      positions.push(+(p.x ?? 0), +(p.y ?? 0), +(p.z ?? 0));
      if (p.r !== undefined) {
        let r = +p.r, g = +p.g, b = +p.b;
        if (r > 1 || g > 1 || b > 1) { r /= 255; g /= 255; b /= 255; }
        colorsBuf.push(r, g, b); hasColor = true;
      }
    }
  }

  return {
    positions: new Float32Array(positions),
    colors: hasColor ? new Float32Array(colorsBuf) : null,
    precomputedSurfaces: data.surfaces ?? null,
  };
}
