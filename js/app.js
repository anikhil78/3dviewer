/**
 * app.js — Main application orchestrator
 *
 * Ties together: file parsing → point cloud display → surface detection
 *               → surface selection → JSON export
 */

import { parseFile }       from './parsers.js';
import { detectSurfaces }  from './ransac.js';
import { PointCloudViewer } from './viewer.js';

// ─── State ────────────────────────────────────────────────────────────────────

const state = {
  positions:   null,   // Float32Array (original, un-centered)
  colors:      null,   // Float32Array | null
  cloudCenter: null,   // THREE.Vector3 centering offset
  surfaces:    [],     // SurfaceData[] from detectSurfaces
  surfaceColors: [],   // string[] — '#rrggbb' per surface
  selected:    new Set(),
};

let viewer;

// ─── DOM refs ─────────────────────────────────────────────────────────────────

const $ = id => document.getElementById(id);

const els = {
  fileInput:      $('file-input'),
  uploadArea:     $('upload-area'),
  fileInfo:       $('file-info'),

  displaySection: $('display-section'),
  detectSection:  $('detection-section'),
  surfacesSection:$('surfaces-section'),
  exportSection:  $('export-section'),

  pointSize:      $('point-size'),
  pointSizeVal:   $('point-size-val'),
  pointOpacity:   $('point-opacity'),
  pointOpacityVal:$('point-opacity-val'),
  colorMode:      $('color-mode'),
  showSurfaces:   $('show-surfaces'),

  maxSurfaces:    $('max-surfaces'),
  maxSurfacesVal: $('max-surfaces-val'),
  distThreshold:  $('dist-threshold'),
  distThresholdVal:$('dist-threshold-val'),
  iterations:     $('iterations'),
  iterationsVal:  $('iterations-val'),
  minPoints:      $('min-points'),
  minPointsVal:   $('min-points-val'),

  detectBtn:      $('detect-btn'),
  surfaceList:    $('surface-list'),
  selectionCount: $('selection-count'),
  exportBtn:      $('export-btn'),
  selectAllBtn:   $('select-all-btn'),
  selectNoneBtn:  $('select-none-btn'),

  statPoints:     $('stat-points'),
  statSurfaces:   $('stat-surfaces'),
  statSelected:   $('stat-selected'),

  progressContainer: $('progress-container'),
  progressFill:   $('progress-fill'),
  progressLabel:  $('progress-label'),
};

// ─── Init ─────────────────────────────────────────────────────────────────────

viewer = new PointCloudViewer($('canvas'));

viewer.onSurfaceClick(idx => toggleSurface(idx));

// ─── File loading ─────────────────────────────────────────────────────────────

els.uploadArea.addEventListener('click', () => els.fileInput.click());
els.fileInput.addEventListener('change', e => e.target.files[0] && loadFile(e.target.files[0]));

els.uploadArea.addEventListener('dragover', e => {
  e.preventDefault();
  els.uploadArea.classList.add('drag-over');
});
els.uploadArea.addEventListener('dragleave', () => els.uploadArea.classList.remove('drag-over'));
els.uploadArea.addEventListener('drop', e => {
  e.preventDefault();
  els.uploadArea.classList.remove('drag-over');
  if (e.dataTransfer.files[0]) loadFile(e.dataTransfer.files[0]);
});

async function loadFile(file) {
  showProgress(5, `Parsing ${file.name}…`);
  try {
    const result = await parseFile(file);

    state.positions = result.positions;
    state.colors    = result.colors;
    state.surfaces  = [];
    state.surfaceColors = [];
    state.selected.clear();

    showProgress(55, 'Building point cloud…');
    await tick();

    const { center } = viewer.loadPointCloud(
      state.positions,
      state.colors,
      els.colorMode.value
    );
    state.cloudCenter = center;
    viewer.clearSurfaces();

    const n = state.positions.length / 3;
    els.statPoints.textContent   = n.toLocaleString();
    els.statSurfaces.textContent = '—';
    els.statSelected.textContent = '—';
    els.fileInfo.textContent     = `${n.toLocaleString()} points  ·  ${file.name}`;

    // Reveal UI panels
    els.displaySection.hidden  = false;
    els.detectSection.hidden   = false;
    els.surfacesSection.hidden = false;
    els.exportSection.hidden   = false;
    els.detectBtn.disabled     = false;

    // Handle pre-computed surfaces from JSON
    if (result.precomputedSurfaces?.length) {
      showProgress(80, 'Loading pre-computed surfaces…');
      await tick();
      await renderPrecomputedSurfaces(result.precomputedSurfaces);
    }

    renderSurfaceList();
    showProgress(100, 'Ready');
    setTimeout(hideProgress, 700);

  } catch (err) {
    hideProgress();
    alert('Could not load file:\n' + err.message);
    console.error(err);
  }
}

// ─── Surface detection ────────────────────────────────────────────────────────

els.detectBtn.addEventListener('click', runDetection);

async function runDetection() {
  if (!state.positions) return;

  els.detectBtn.disabled   = true;
  els.detectBtn.innerHTML  = '<span class="spinner"></span>Detecting…';
  state.selected.clear();
  viewer.clearSurfaces();
  state.surfaces      = [];
  state.surfaceColors = [];
  renderSurfaceList();

  try {
    // detectSurfaces operates on the CENTERED geometry buffer
    // (that's what the viewer stored after the translate call)
    const centeredPos = viewer._pointCloud.geometry.attributes.position.array;

    state.surfaces = await detectSurfaces(centeredPos, {
      maxSurfaces:   parseInt(els.maxSurfaces.value),
      distThreshold: parseFloat(els.distThreshold.value),
      iterations:    parseInt(els.iterations.value),
      minPointsPct:  parseFloat(els.minPoints.value) / 100,
      onProgress:    (frac, label) => showProgress(frac * 95, label),
    });

    // Add meshes to viewer
    state.surfaceColors = state.surfaces.map((s, i) => viewer.addSurface(s, i));

  } catch (err) {
    console.error('Detection error:', err);
    alert('Surface detection failed:\n' + err.message);
  }

  els.detectBtn.disabled  = false;
  els.detectBtn.textContent = 'Detect Surfaces';
  els.statSurfaces.textContent = state.surfaces.length || '0';
  els.exportBtn.disabled = state.surfaces.length === 0;

  renderSurfaceList();
  updateSelectionUI();
  showProgress(100, `Found ${state.surfaces.length} surface(s)`);
  setTimeout(hideProgress, 700);
}

async function renderPrecomputedSurfaces(rawSurfaces) {
  // Accept the JSON export format we produce ourselves, so the user can
  // reload a previously exported file and see the surfaces again.
  const THREE = await import('three');

  for (let i = 0; i < rawSurfaces.length; i++) {
    const s = rawSurfaces[i];
    if (!s.plane || !s.hull_3d) continue;

    const plane    = s.plane;
    const hull3D   = s.hull_3d.map(([x, y, z]) => new THREE.Vector3(x, y, z));
    const centroid = new THREE.Vector3(...(s.plane.centroid ?? [0, 0, 0]));
    const normal   = new THREE.Vector3(...(s.plane.normal  ?? [0, 1, 0]));
    const t1 = s.tangent1 ? new THREE.Vector3(...s.tangent1) : new THREE.Vector3(1, 0, 0);
    const t2 = s.tangent2 ? new THREE.Vector3(...s.tangent2) : new THREE.Vector3(0, 0, 1);

    const surfData = {
      plane: { nx: plane.normal[0], ny: plane.normal[1], nz: plane.normal[2], d: plane.d,
               cx: plane.centroid[0], cy: plane.centroid[1], cz: plane.centroid[2] },
      normal, centroid, hull3D, tangent1: t1, tangent2: t2,
      area: s.area_m2 ?? 0,
      pointCount: s.point_count ?? 0,
    };

    state.surfaces.push(surfData);
    state.surfaceColors.push(viewer.addSurface(surfData, i));
  }
}

// ─── Display controls ─────────────────────────────────────────────────────────

els.pointSize.addEventListener('input', () => {
  const v = els.pointSize.value;
  els.pointSizeVal.textContent = v;
  viewer.setPointSize(+v);
});

els.pointOpacity.addEventListener('input', () => {
  const v = +els.pointOpacity.value;
  els.pointOpacityVal.textContent = Math.round(v * 100) + '%';
  viewer.setPointOpacity(v);
});

els.colorMode.addEventListener('change', () => {
  viewer.setColorMode(els.colorMode.value);
});

els.showSurfaces.addEventListener('change', () => {
  viewer.setSurfacesVisible(els.showSurfaces.checked);
});

// Slider label live-update
sliderLabel('max-surfaces',   'max-surfaces-val',    v => v);
sliderLabel('dist-threshold', 'dist-threshold-val',  v => parseFloat(v).toFixed(3));
sliderLabel('iterations',     'iterations-val',       v => v);
sliderLabel('min-points',     'min-points-val',       v => v + '%');

function sliderLabel(sliderId, labelId, fmt) {
  const sl = $(sliderId), lb = $(labelId);
  sl.addEventListener('input', () => { lb.textContent = fmt(sl.value); });
}

// ─── Surface selection ────────────────────────────────────────────────────────

function toggleSurface(idx) {
  if (state.selected.has(idx)) {
    state.selected.delete(idx);
    viewer.selectSurface(idx, false);
  } else {
    state.selected.add(idx);
    viewer.selectSurface(idx, true);
  }
  syncListSelection();
  updateSelectionUI();
}

els.selectAllBtn.addEventListener('click', () => {
  state.surfaces.forEach((_, i) => { state.selected.add(i); viewer.selectSurface(i, true); });
  syncListSelection();
  updateSelectionUI();
});

els.selectNoneBtn.addEventListener('click', () => {
  state.selected.forEach(i => viewer.selectSurface(i, false));
  state.selected.clear();
  syncListSelection();
  updateSelectionUI();
});

function syncListSelection() {
  document.querySelectorAll('.surface-item').forEach(el => {
    const i = parseInt(el.dataset.idx);
    const on = state.selected.has(i);
    el.classList.toggle('selected', on);
    el.querySelector('.surface-check').textContent = on ? '✓' : '';
  });
}

function updateSelectionUI() {
  const n = state.selected.size;
  els.statSelected.textContent   = n > 0 ? n : '—';
  els.selectionCount.textContent = n > 0 ? `${n} selected` : '';
  els.exportBtn.disabled         = n === 0;
}

// ─── Surface list rendering ───────────────────────────────────────────────────

function renderSurfaceList() {
  if (state.surfaces.length === 0) {
    els.surfaceList.innerHTML =
      '<p class="empty-state">No surfaces detected yet.<br>Adjust parameters and run detection.</p>';
    return;
  }

  els.surfaceList.innerHTML = state.surfaces.map((s, i) => {
    const selected = state.selected.has(i);
    const areaStr  = s.area < 0.01
      ? (s.area * 1e4).toFixed(1) + ' cm²'
      : s.area.toFixed(3) + ' m²';
    const col = state.surfaceColors[i] ?? '#888888';

    // Normal direction label (most aligned axis)
    const nx = Math.abs(s.plane.nx), ny = Math.abs(s.plane.ny), nz = Math.abs(s.plane.nz);
    const axis = nx > ny && nx > nz ? 'YZ plane' : ny > nz ? 'XZ plane' : 'XY plane';

    return `<div class="surface-item${selected ? ' selected' : ''}" data-idx="${i}">
      <div class="surface-dot" style="background:${col}"></div>
      <div class="surface-info">
        <div class="surface-name">Surface ${i + 1} <span style="color:#404070;font-weight:400">(${axis})</span></div>
        <div class="surface-meta">${s.pointCount.toLocaleString()} pts · ${areaStr}</div>
      </div>
      <div class="surface-check">${selected ? '✓' : ''}</div>
    </div>`;
  }).join('');

  // Attach click listeners to items
  els.surfaceList.querySelectorAll('.surface-item').forEach(el => {
    el.addEventListener('click', () => toggleSurface(parseInt(el.dataset.idx)));
  });
}

// ─── Export ───────────────────────────────────────────────────────────────────

els.exportBtn.addEventListener('click', exportSelected);

function exportSelected() {
  if (state.selected.size === 0) return;

  const payload = {
    // Include metadata for the downstream path-planning / robot code
    exported_at: new Date().toISOString(),
    coordinate_frame: 'point_cloud_local',
    note: 'Centroid / hull coordinates are in the centered point cloud frame. ' +
          'Add cloud_center_offset to recover original sensor coordinates.',
    cloud_center_offset: state.cloudCenter
      ? [state.cloudCenter.x, state.cloudCenter.y, state.cloudCenter.z]
      : [0, 0, 0],
    surfaces: [],
  };

  for (const idx of [...state.selected].sort((a, b) => a - b)) {
    const s = state.surfaces[idx];
    payload.surfaces.push({
      id:          idx,
      label:       `Surface ${idx + 1}`,
      plane: {
        normal:   [s.plane.nx, s.plane.ny, s.plane.nz],
        d:         s.plane.d,
        centroid: [s.plane.cx, s.plane.cy, s.plane.cz],
      },
      // 3-D convex hull vertices (same frame as centroid)
      hull_3d:    s.hull3D.map(p => [p.x, p.y, p.z]),
      // Plane basis vectors (for generating toolpaths)
      tangent1:   [s.tangent1.x, s.tangent1.y, s.tangent1.z],
      tangent2:   [s.tangent2.x, s.tangent2.y, s.tangent2.z],
      area_m2:    s.area,
      point_count: s.pointCount,
    });
  }

  const blob = new Blob([JSON.stringify(payload, null, 2)], { type: 'application/json' });
  const url  = URL.createObjectURL(blob);
  const a    = document.createElement('a');
  a.href     = url;
  a.download = `surfaces_${Date.now()}.json`;
  a.click();
  URL.revokeObjectURL(url);
}

// ─── Progress helpers ─────────────────────────────────────────────────────────

function showProgress(pct, label) {
  els.progressContainer.hidden     = false;
  els.progressFill.style.width     = `${Math.min(100, pct)}%`;
  els.progressLabel.textContent    = label;
}

function hideProgress() {
  els.progressContainer.hidden = true;
  els.progressFill.style.width = '0%';
}

function tick() { return new Promise(r => setTimeout(r, 0)); }
