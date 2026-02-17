#!/usr/bin/env python3
"""
generate_sample.py — Generate synthetic point clouds for testing the viewer.

Produces a PLY file containing several flat surfaces (planes) with noise,
plus scattered background clutter — mimicking what a real depth camera would
see in a room with flat walls/floors.

Usage:
    python generate_sample.py                      # → sample_scene.ply
    python generate_sample.py --out mycloud.ply    # custom filename
    python generate_sample.py --format xyz         # write .xyz instead

Requirements:
    pip install numpy
    pip install open3d     # optional — only for the preview window
"""

import argparse
import struct
import numpy as np
from pathlib import Path


# ─── Scene definition ─────────────────────────────────────────────────────────

SURFACES = [
    # Each entry: (label, normal_xyz, point_on_plane_xyz, width, height, n_points)
    # A roughly room-like scene scaled in metres
    {
        "label":  "Floor",
        "normal": (0, 1, 0),
        "origin": (0, 0, 0),
        "u":      (1, 0, 0),  # local X axis on surface
        "v":      (0, 0, 1),  # local Z axis on surface
        "width":  4.0,
        "height": 5.0,
        "pts":    12_000,
        "color":  (180, 160, 140),
    },
    {
        "label":  "Back wall",
        "normal": (0, 0, -1),
        "origin": (0, 1.5, -2.5),
        "u":      (1, 0, 0),
        "v":      (0, 1, 0),
        "width":  4.0,
        "height": 3.0,
        "pts":    8_000,
        "color":  (200, 200, 220),
    },
    {
        "label":  "Left wall",
        "normal": (1, 0, 0),
        "origin": (-2.0, 1.5, 0),
        "u":      (0, 0, 1),
        "v":      (0, 1, 0),
        "width":  5.0,
        "height": 3.0,
        "pts":    7_000,
        "color":  (210, 195, 195),
    },
    {
        "label":  "Tabletop",
        "normal": (0, 1, 0),
        "origin": (0.5, 0.75, -0.5),
        "u":      (1, 0, 0),
        "v":      (0, 0, 1),
        "width":  1.2,
        "height": 0.8,
        "pts":    3_000,
        "color":  (120, 80, 50),
    },
    {
        "label":  "Tilted shelf",
        "normal": (0.1, 1, 0.05),   # slightly tilted
        "origin": (-1.0, 1.2, -1.5),
        "u":      (1, 0, 0),
        "v":      (0, 0, 1),
        "width":  0.9,
        "height": 0.5,
        "pts":    1_500,
        "color":  (100, 100, 140),
    },
]

NOISE_STD   = 0.008   # metres of Gaussian measurement noise
CLUTTER_PTS = 2_000   # random background points


# ─── Point cloud generation ───────────────────────────────────────────────────

def normalise(v):
    v = np.asarray(v, dtype=float)
    return v / np.linalg.norm(v)


def make_surface_points(surf, rng):
    """Return (N,6) array of [x,y,z,r,g,b] for one surface."""
    origin = np.array(surf["origin"], dtype=float)
    u      = normalise(surf["u"])
    v      = normalise(surf["v"])
    n      = surf["pts"]
    w, h   = surf["width"] / 2, surf["height"] / 2

    # Random positions in surface local frame
    us = rng.uniform(-w, w, n)
    vs = rng.uniform(-h, h, n)

    pts = origin + us[:, None] * u + vs[:, None] * v

    # Add measurement noise perpendicular to the plane
    normal = normalise(surf["normal"])
    noise  = rng.normal(0, NOISE_STD, n)
    pts   += noise[:, None] * normal

    # Per-surface base colour + slight intensity variation
    r0, g0, b0 = surf["color"]
    vary = rng.integers(-20, 20, (n, 3), dtype=int)
    colors = np.clip(np.array([r0, g0, b0]) + vary, 0, 255).astype(np.uint8)

    return np.hstack([pts, colors])


def generate_cloud(seed=42):
    rng = np.random.default_rng(seed)

    parts = [make_surface_points(s, rng) for s in SURFACES]

    # Random clutter (objects, noise hits, etc.)
    clutter_xyz = rng.uniform([-2, 0, -2.5], [2, 3, 2.5], (CLUTTER_PTS, 3))
    clutter_rgb = rng.integers(50, 200, (CLUTTER_PTS, 3), dtype=np.uint8)
    parts.append(np.hstack([clutter_xyz, clutter_rgb]))

    cloud = np.vstack(parts)
    rng.shuffle(cloud)  # mix surfaces so order is not a hint to RANSAC
    return cloud


# ─── Writers ─────────────────────────────────────────────────────────────────

def write_ply_binary(path: Path, cloud: np.ndarray):
    """Write binary PLY (smallest file, fastest to load in Three.js PLYLoader)."""
    n = len(cloud)
    header = (
        "ply\n"
        "format binary_little_endian 1.0\n"
        f"element vertex {n}\n"
        "property float x\n"
        "property float y\n"
        "property float z\n"
        "property uchar red\n"
        "property uchar green\n"
        "property uchar blue\n"
        "end_header\n"
    )
    xyz = cloud[:, :3].astype(np.float32)
    rgb = cloud[:, 3:].astype(np.uint8)

    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        for i in range(n):
            f.write(struct.pack("<fff", *xyz[i]))
            f.write(struct.pack("<BBB", *rgb[i]))

    print(f"Wrote binary PLY  → {path}  ({n:,} points, {path.stat().st_size/1024:.0f} kB)")


def write_ply_ascii(path: Path, cloud: np.ndarray):
    n = len(cloud)
    lines = [
        "ply", "format ascii 1.0",
        f"element vertex {n}",
        "property float x", "property float y", "property float z",
        "property uchar red", "property uchar green", "property uchar blue",
        "end_header",
    ]
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")
        for row in cloud:
            x, y, z = row[:3]
            r, g, b = int(row[3]), int(row[4]), int(row[5])
            f.write(f"{x:.6f} {y:.6f} {z:.6f} {r} {g} {b}\n")

    print(f"Wrote ASCII PLY  → {path}  ({n:,} points)")


def write_xyz(path: Path, cloud: np.ndarray):
    with open(path, "w") as f:
        for row in cloud:
            x, y, z = row[:3]
            r, g, b = int(row[3]), int(row[4]), int(row[5])
            f.write(f"{x:.6f} {y:.6f} {z:.6f} {r} {g} {b}\n")
    print(f"Wrote XYZ  → {path}  ({len(cloud):,} points)")


def write_json(path: Path, cloud: np.ndarray):
    import json
    pts = [
        {"x": float(r[0]), "y": float(r[1]), "z": float(r[2]),
         "r": int(r[3]),   "g": int(r[4]),   "b": int(r[5])}
        for r in cloud
    ]
    with open(path, "w") as f:
        json.dump({"points": pts}, f, separators=(",", ":"))
    print(f"Wrote JSON  → {path}  ({len(cloud):,} points)")


# ─── CLI ──────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out",    default="sample_scene.ply", help="Output file path")
    ap.add_argument("--format", choices=["ply_binary", "ply_ascii", "xyz", "json"],
                    default="ply_binary", help="Output format (default: ply_binary)")
    ap.add_argument("--seed",   type=int, default=42, help="Random seed for reproducibility")
    ap.add_argument("--preview", action="store_true",
                    help="Open interactive preview with Open3D (requires: pip install open3d)")
    args = ap.parse_args()

    print(f"Generating synthetic scene with {sum(s['pts'] for s in SURFACES):,} surface "
          f"points + {CLUTTER_PTS:,} clutter points…")

    cloud = generate_cloud(seed=args.seed)
    out   = Path(args.out)

    writers = {
        "ply_binary": write_ply_binary,
        "ply_ascii":  write_ply_ascii,
        "xyz":        write_xyz,
        "json":       write_json,
    }
    writers[args.format](out, cloud)

    print(f"\nSurfaces in this cloud:")
    for i, s in enumerate(SURFACES):
        print(f"  Surface {i+1}: {s['label']:20s}  normal={s['normal']}  "
              f"size={s['width']}×{s['height']} m  pts={s['pts']:,}")

    if args.preview:
        try:
            import open3d as o3d
            pcd = o3d.io.read_point_cloud(str(out))
            o3d.visualization.draw_geometries([pcd], window_name="Point Cloud Preview")
        except ImportError:
            print("\nOpen3D not installed — skipping preview.  Run: pip install open3d")


if __name__ == "__main__":
    main()
