#!/usr/bin/env python3
"""
Generate low-poly collision proxies for the chassis + flippers.

The CAD-exported body STLs (``dicerox_urdf_v1/meshes/collision``) are far too
heavy for a 100 Hz arm-vs-body clearance check: ``base_link.STL`` alone is
~188k triangles spread over ~2990 disconnected components (the shell, the two
track housings, and ~2900 tiny bolts/standoffs). FCL distance queries scale
with mesh size, and the arm sits right on top of the body, so this mesh is hit
almost every tick.

This script writes lightweight ``.stl`` proxies to ``meshes/collision/`` used by
``arm_teleop``'s ``body_collision`` checker (NOT by the visual digital twin,
which always renders the full-resolution ``meshes/visual`` STLs):

* **chassis** — drop components below ``--min-vol`` m^3 (the bolts), then quadric-
  decimate the remaining structural shell to ``--chassis-target`` triangles.
  Component filtering preserves the concave top deck where the arm mounts (a
  convex hull would fill it in and over-restrict the arm).
* **flippers** — single watertight parts; straight quadric decimation.

A decimated proxy may sit a millimetre or two inside the true surface; that is
compensated at runtime by ``body_collision_stop_dist`` in ``sdls_servo``, not by
inflating the mesh here.

Run once and commit the output:

    python3 scripts/make_body_collision.py

Requires: trimesh, fast-simplification  (pip install trimesh fast-simplification)
"""
from __future__ import annotations

import argparse
import os
import sys

import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
# Default source: the legacy full-res body collision meshes in the reference tree.
DEFAULT_SRC = os.path.normpath(os.path.join(
    PKG, '..', '..', '..', '..', 'reference', 'Arm-TMR-2026', 'src', 'dicerox',
    'dicerox_urdf_v1', 'meshes', 'collision'))
OUT_DIR = os.path.join(PKG, 'meshes', 'collision')

FLIPPERS = ('Flipper1L.stl', 'Flipper2L.stl', 'Flipper3L.stl', 'Flipper4L.stl')
CHASSIS = 'base_link.stl'


def _clean(mesh):
    mesh.merge_vertices()
    mesh.update_faces(mesh.nondegenerate_faces())
    return mesh


def _decimate_chassis(src, min_vol, target):
    m = _clean(trimesh.load(src, force='mesh'))
    comps = m.split(only_watertight=False)
    keep = [c for c in comps if c.bounding_box.volume > min_vol]
    if not keep:
        keep = [m]
    union = _clean(trimesh.util.concatenate(keep))
    before = len(union.faces)
    if before > target:
        union = _clean(union.simplify_quadric_decimation(face_count=target))
    return union, before, len(keep), len(comps)


def _decimate_simple(src, target):
    m = _clean(trimesh.load(src, force='mesh'))
    before = len(m.faces)
    if before > target:
        m = _clean(m.simplify_quadric_decimation(face_count=target))
    return m, before


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--src', default=DEFAULT_SRC, help='source full-res mesh dir')
    ap.add_argument('--out', default=OUT_DIR, help='output low-poly dir')
    ap.add_argument('--chassis-target', type=int, default=5000,
                    help='target triangle count for the chassis shell')
    ap.add_argument('--flipper-target', type=int, default=700,
                    help='target triangle count per flipper')
    ap.add_argument('--min-vol', type=float, default=5e-4,
                    help='drop chassis components below this bbox volume (m^3)')
    args = ap.parse_args()

    if not os.path.isdir(args.src):
        print(f'source dir not found: {args.src}', file=sys.stderr)
        return 1
    os.makedirs(args.out, exist_ok=True)

    chassis, before, kept, total = _decimate_chassis(
        os.path.join(args.src, CHASSIS), args.min_vol, args.chassis_target)
    chassis.export(os.path.join(args.out, CHASSIS))
    print(f'{CHASSIS:16} {before:7d} -> {len(chassis.faces):6d} tris  '
          f'(kept {kept}/{total} components)')

    for name in FLIPPERS:
        m, before = _decimate_simple(os.path.join(args.src, name), args.flipper_target)
        m.export(os.path.join(args.out, name))
        print(f'{name:16} {before:7d} -> {len(m.faces):6d} tris')

    print(f'written to {args.out}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
