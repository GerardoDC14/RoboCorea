#!/usr/bin/env python3
"""
Generate low-poly collision proxies for the arm links.

The CAD-exported collision STLs in ``meshes/collision/`` are full-resolution
(Link6 alone is ~118k triangles). That is fine for rendering but far too heavy
for a 100 Hz self-collision check: FCL distance queries scale with mesh size.
This script decimates each link to a few hundred / thousand triangles with
quadric edge-collapse (shape-preserving, keeps concavity — unlike a convex
hull), writing the result to ``meshes/collision_lowpoly/``.

The decimated proxy may sit a millimetre or two *inside* the true surface; that
is compensated at runtime by ``collision_stop_dist`` (the safety margin in
sdls_servo), not by inflating the mesh here.

Run once and commit the output:

    python3 scripts/make_collision_meshes.py            # default target ~1200 tris
    python3 scripts/make_collision_meshes.py --target 800

Requires: trimesh, fast-simplification  (pip install trimesh fast-simplification)
"""
from __future__ import annotations

import argparse
import os
import sys

import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
SRC_DIR = os.path.join(PKG, 'meshes', 'collision')
OUT_DIR = os.path.join(PKG, 'meshes', 'collision_lowpoly')


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--target', type=int, default=1200,
                    help='target triangle count per mesh (default 1200)')
    ap.add_argument('--src', default=SRC_DIR, help='source mesh dir')
    ap.add_argument('--out', default=OUT_DIR, help='output mesh dir')
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    stls = sorted(f for f in os.listdir(args.src) if f.lower().endswith('.stl'))
    if not stls:
        print(f'no STL files in {args.src}', file=sys.stderr)
        return 1

    total_before = total_after = 0
    for name in stls:
        mesh = trimesh.load(os.path.join(args.src, name), force='mesh')
        mesh.merge_vertices()
        before = len(mesh.faces)

        if before > args.target:
            # quadric decimation (uses fast_simplification under the hood)
            mesh = mesh.simplify_quadric_decimation(face_count=args.target)
        mesh.merge_vertices()
        mesh.update_faces(mesh.nondegenerate_faces())
        after = len(mesh.faces)

        out_path = os.path.join(args.out, name)
        mesh.export(out_path)
        total_before += before
        total_after += after
        print(f'{name:14} {before:7d} -> {after:6d} tris   '
              f'watertight={mesh.is_watertight}')

    print(f'{"TOTAL":14} {total_before:7d} -> {total_after:6d} tris '
          f'({100.0 * total_after / total_before:.1f}% of original)')
    print(f'written to {args.out}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
