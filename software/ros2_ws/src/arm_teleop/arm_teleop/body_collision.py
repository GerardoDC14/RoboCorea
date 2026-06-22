#!/usr/bin/env python3
"""
FCL clearance between the arm links and the robot BODY (chassis + 4 flippers).

``self_collision.py`` answers "is the arm folding into itself?"; this answers
"is the arm about to hit the chassis deck or a raised flipper?". It is the same
FCL-on-low-poly-meshes machinery, but the obstacle set is the body:

* the **chassis** is one fixed mesh, posed in the arm base frame;
* each **flipper** is a mesh whose pose is driven live from ``/encoders/flipper``
  (``set_flipper_angles``), so raising a flipper actually moves the obstacle.

All body part transforms (base_link -> chassis, chassis -> each flipper joint)
are read straight from the **combined URDF** (``arm_description/dicerox_full.urdf``),
so there are no hard-coded offsets — change the URDF mount and this follows.

An allowed-collision matrix (ACM) is sampled once at startup to drop arm-link /
body-part pairs that are effectively always in contact (e.g. the arm mount link
resting on the chassis deck) so they never permanently veto motion — mirroring
what ``self_collision`` does for the arm's own links.

This module is pure geometry + ROS-agnostic. The damping/veto *policy* lives in
``sdls_servo``; this just reports "how far is the arm from the body right now".
The public surface (``distances``/``min_clearance``/``in_collision``) matches
``SelfCollision`` so the servo can treat the two checkers uniformly.
"""
from __future__ import annotations

import os

import numpy as np
from urdf_parser_py.urdf import URDF

try:
    import fcl
    import trimesh
    HAVE_DEPS = True
    _IMPORT_ERROR = ''
except Exception as exc:  # pragma: no cover - depends on host install
    HAVE_DEPS = False
    _IMPORT_ERROR = str(exc)


# ── small URDF transform helpers (kept local so this module is standalone) ──
def _rpy_to_rot(r, p, y):
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


def _origin_to_T(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = _rpy_to_rot(*rpy)
    T[:3, 3] = xyz
    return T


def _axis_rot(axis, angle):
    x, y, z = axis
    K = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    R = np.eye(3) + np.sin(angle) * K + (1.0 - np.cos(angle)) * (K @ K)
    T = np.eye(4)
    T[:3, :3] = R
    return T


def _joint_origin(j):
    xyz = list(j.origin.xyz) if (j.origin and j.origin.xyz) else [0.0, 0.0, 0.0]
    rpy = list(j.origin.rpy) if (j.origin and j.origin.rpy) else [0.0, 0.0, 0.0]
    return _origin_to_T(xyz, rpy)


class BodyCollision:
    """Arm-link vs. body-part clearance via FCL BVH distance queries.

    Parameters
    ----------
    kin : ArmKinematics
        Supplies ``link_poses(q)`` (arm link world transforms in the base frame),
        ``child_links`` and the joint limits used to sample the ACM.
    urdf_string : str
        The **combined** robot URDF (arm + chassis + flippers). Used to read the
        base-frame transforms of the chassis and each flipper joint.
    arm_mesh_dir : str
        Directory of low-poly arm link meshes (``<LinkName>.stl``).
    body_mesh_dir : str
        Directory of low-poly body meshes (chassis + flippers).
    flipper_links : sequence[str]
        Flipper link names, in the SAME order as the angles passed to
        ``set_flipper_angles`` (default the URDF's [fl, fr, rl, rr]).
    chassis_link / chassis_mesh : str
        The chassis link name and its mesh filename (the chassis mesh is
        historically ``base_link.stl`` while the link is ``chassis_base_link``).
    flipper_sample_rad : float
        Half-range (rad) over which flipper angles are sampled when building the
        ACM, so flipper poses that can only collide when raised are kept.
    """

    def __init__(self, kin, urdf_string, arm_mesh_dir, body_mesh_dir,
                 flipper_links=('Flipper1L', 'Flipper2L', 'Flipper3L', 'Flipper4L'),
                 chassis_link='chassis_base_link', chassis_mesh='base_link.stl',
                 acm_samples=1500, acm_always=0.90, near_band=0.08,
                 flipper_sample_rad=np.pi / 2, logger=None):
        if not HAVE_DEPS:
            raise ImportError(
                f'python-fcl + trimesh required for body collision ({_IMPORT_ERROR})')
        for d in (arm_mesh_dir, body_mesh_dir):
            if not d or not os.path.isdir(d):
                raise FileNotFoundError(f'collision mesh dir not found: {d!r}')

        self.kin = kin
        self._log = logger or (lambda _m: None)
        self.near_band = float(near_band)
        self.flipper_sample_rad = float(flipper_sample_rad)
        self._dreq = fcl.DistanceRequest()
        self._creq = fcl.CollisionRequest()

        # ── arm link FCL objects (own copies — not shared with SelfCollision) ──
        self.arm_links = [ln for ln in kin.child_links
                          if os.path.exists(os.path.join(arm_mesh_dir, ln + '.stl'))]
        if not self.arm_links:
            raise FileNotFoundError(f'no arm link meshes in {arm_mesh_dir}')
        self._objs, self._corners = {}, {}
        for ln in self.arm_links:
            self._objs[ln], self._corners[ln] = self._load(
                os.path.join(arm_mesh_dir, ln + '.stl'))

        # ── body FCL objects: chassis (static) + flippers (angle-driven) ──
        self.chassis_link = chassis_link
        self.flipper_links = list(flipper_links)
        self.body_links = [chassis_link] + self.flipper_links
        self._objs[chassis_link], self._corners[chassis_link] = self._load(
            os.path.join(body_mesh_dir, chassis_mesh))
        for ln in self.flipper_links:
            self._objs[ln], self._corners[ln] = self._load(
                os.path.join(body_mesh_dir, ln + '.stl'))

        # ── base-frame transforms of the body parts, from the combined URDF ──
        self._T_base_chassis, self._flip_pre, self._flip_axis, self.flip_joints = \
            self._body_transforms(urdf_string)
        self._n_flip = len(self.flipper_links)
        self._flipper_q = np.zeros(self._n_flip)   # live, set by set_flipper_angles

        # candidate pairs = every arm link against every body part
        all_pairs = [(a, b) for a in self.arm_links for b in self.body_links]
        self.pairs = self._build_acm(all_pairs, acm_samples, acm_always)

    # ── geometry / loading ────────────────────────────────────────────
    @staticmethod
    def _load(path):
        m = trimesh.load(path, force='mesh')
        m.merge_vertices()
        verts = np.asarray(m.vertices, dtype=np.float64)
        faces = np.asarray(m.faces, dtype=np.int64)
        model = fcl.BVHModel()
        model.beginModel(len(verts), len(faces))
        model.addSubModel(verts, faces)
        model.endModel()
        lo, hi = verts.min(axis=0), verts.max(axis=0)
        corners = np.array([[x, y, z] for x in (lo[0], hi[0])
                            for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
        return fcl.CollisionObject(model, fcl.Transform()), corners

    def _body_transforms(self, urdf_string):
        """Compute, from the combined URDF, the body parts' base-frame transforms.

        Returns (T_base_chassis, flip_pre[], flip_axis[], flip_joint_names[])
        where flipper world pose at angle q_k is ``flip_pre[k] @ axis_rot(axis, q_k)``.
        """
        robot = URDF.from_xml_string(urdf_string)
        by_child = {j.child: j for j in robot.joints}

        def world(link):
            T = np.eye(4)
            cur = link
            while cur in by_child:
                j = by_child[cur]
                T = _joint_origin(j) @ T
                cur = j.parent
            return T

        inv_base = np.linalg.inv(world(self.kin.base))
        T_base_chassis = inv_base @ world(self.chassis_link)

        flip_pre, flip_axis, flip_joints = [], [], []
        for ln in self.flipper_links:
            j = by_child.get(ln)
            if j is None:
                raise ValueError(f'flipper link {ln!r} has no joint in the URDF')
            # transform up to (but not including) the flipper joint's rotation
            flip_pre.append(inv_base @ world(j.parent) @ _joint_origin(j))
            axis = np.array(j.axis if j.axis else [1.0, 0.0, 0.0], dtype=float)
            nrm = np.linalg.norm(axis)
            flip_axis.append(axis / nrm if nrm > 0 else np.array([1.0, 0.0, 0.0]))
            flip_joints.append(j.name)
        return T_base_chassis, flip_pre, flip_axis, flip_joints

    # ── live flipper pose ─────────────────────────────────────────────
    def set_flipper_angles(self, angles):
        """Update the flipper obstacle angles (radians), order == flipper_links.

        Cheap + lock-free: stores a fresh array; the actual FCL placement happens
        inside the (serialized) distance/collision queries.
        """
        a = np.asarray(angles, dtype=float).ravel()
        if a.size >= self._n_flip:
            self._flipper_q = a[:self._n_flip].copy()
        elif a.size:
            q = self._flipper_q.copy()
            q[:a.size] = a
            self._flipper_q = q

    def _body_poses(self, flipper_q):
        poses = {self.chassis_link: self._T_base_chassis}
        for k, ln in enumerate(self.flipper_links):
            poses[ln] = self._flip_pre[k] @ _axis_rot(self._flip_axis[k], flipper_q[k])
        return poses

    def _place_one(self, ln, T):
        self._objs[ln].setTransform(fcl.Transform(
            np.ascontiguousarray(T[:3, :3]), np.ascontiguousarray(T[:3, 3])))

    # ── queries (interface mirrors SelfCollision) ─────────────────────
    @staticmethod
    def _aabb(corners, T):
        c = corners @ T[:3, :3].T + T[:3, 3]
        return c.min(axis=0), c.max(axis=0)

    @staticmethod
    def _aabb_gap(a, b):
        amin, amax = a
        bmin, bmax = b
        gap = np.maximum(np.maximum(amin - bmax, bmin - amax), 0.0)
        return float(np.linalg.norm(gap))

    def distances(self, q):
        """Signed clearance (m) for arm/body pairs within the near band.

        ``0.0`` = touching/penetrating. Pairs whose AABBs are farther apart than
        ``near_band`` are skipped (their true clearance can only be larger).
        """
        fq = self._flipper_q
        arm_poses = self.kin.link_poses(q)
        body_poses = self._body_poses(fq)
        poses = {**arm_poses, **body_poses}

        aabb = {ln: self._aabb(self._corners[ln], poses[ln])
                for ln in set(a for p in self.pairs for a in p)}

        out = {}
        placed = set()
        for a, b in self.pairs:
            if self._aabb_gap(aabb[a], aabb[b]) > self.near_band:
                continue
            for ln in (a, b):
                if ln not in placed:
                    self._place_one(ln, poses[ln])
                    placed.add(ln)
            res = fcl.DistanceResult()
            fcl.distance(self._objs[a], self._objs[b], self._dreq, res)
            d = res.min_distance
            out[(a, b)] = 0.0 if d < 0.0 else d
        return out

    def min_clearance(self, q):
        d = self.distances(q)
        return min(d.values()) if d else float('inf')

    def in_collision(self, q, margin=0.0):
        """True if any arm/body pair is closer than ``margin`` (or touching)."""
        if margin > 0.0:
            return self.min_clearance(q) < margin
        poses = {**self.kin.link_poses(q), **self._body_poses(self._flipper_q)}
        for ln in set(a for p in self.pairs for a in p):
            self._place_one(ln, poses[ln])
        return any(self._in_contact(a, b) for a, b in self.pairs)

    def _in_contact(self, a, b):
        res = fcl.CollisionResult()
        return fcl.collide(self._objs[a], self._objs[b], self._creq, res) > 0

    # ── allowed-collision matrix ──────────────────────────────────────
    def _build_acm(self, all_pairs, n_samples, always):
        lo, up = self.kin.lower, self.kin.upper
        rng = np.random.default_rng(0)
        fr = self.flipper_sample_rad

        hits = {p: 0 for p in all_pairs}
        n = max(n_samples, 1)
        arm_links = set(a for p in all_pairs for a in p) & set(self.arm_links)
        body_links = set(b for p in all_pairs for b in p) & set(self.body_links)
        for _ in range(n):
            q = lo + (up - lo) * rng.random(len(lo))
            fq = (rng.random(self._n_flip) * 2.0 - 1.0) * fr
            arm_poses = self.kin.link_poses(q)
            body_poses = self._body_poses(fq)
            for ln in arm_links:
                self._place_one(ln, arm_poses[ln])
            for ln in body_links:
                self._place_one(ln, body_poses[ln])
            for p in all_pairs:
                if self._in_contact(*p):
                    hits[p] += 1

        kept, always_pairs, never_pairs = [], [], []
        for p in all_pairs:
            frac = hits[p] / n
            if frac >= always:
                always_pairs.append(p)       # arm mount permanently on the deck
            elif frac == 0.0:
                never_pairs.append(p)         # geometrically unreachable
            else:
                kept.append(p)
        self._log(
            f'body-collision ACM: {len(kept)} active arm/body pairs (dropped '
            f'{len(always_pairs)} always-touching, {len(never_pairs)} unreachable) '
            f'of {len(all_pairs)} total')
        return kept
