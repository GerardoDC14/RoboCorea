#!/usr/bin/env python3
"""
Closed-form, kinematically-decoupled inverse kinematics for the dicerox 6R arm.

The arm has a **spherical wrist**: the J4/J5/J6 axes intersect at a common point
(the wrist centre), and — after folding the joint-origin rotations — they form a
clean proper-Euler **ZYZ** triplet (axis₁ == axis₃, axis₁ ⟂ axis₂). That lets us
decouple position from orientation (Pieper):

  1. wrist centre  wc = p_target + R_target · w6   (w6 ≈ [0,0,0.189] m, constant)
  2. arm joints J1–J3 place the wrist centre        (3-DOF damped-Newton solve;
       the ~3° frame tilts on J2/J3 stop this being a pristine closed form)
  3. wrist joints J4–J6 set orientation              (closed-form ZYZ extraction,
       two branches → wrist "flip")

Returns *all* valid branches (within joint limits) so a caller can pick the one
nearest a seed deterministically — unlike a single numerical 6-DOF solve. Pure
numpy; reuses ``ArmKinematics`` for FK. If the parsed arm isn't a spherical-wrist
6R, construction raises ``NotSphericalWrist`` and callers fall back to numerical.
"""

from __future__ import annotations

import numpy as np


class NotSphericalWrist(RuntimeError):
    """Raised when the arm geometry doesn't admit the decoupled solver."""


def _axis_rot(axis, angle):
    x, y, z = axis
    K = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return np.eye(3) + np.sin(angle) * K + (1.0 - np.cos(angle)) * (K @ K)


def _axis_rot4(axis, angle):
    T = np.eye(4)
    T[:3, :3] = _axis_rot(axis, angle)
    return T


def _line_common_point(lines):
    """Least-squares point closest to a set of (point, dir) axis lines."""
    M = np.zeros((3, 3))
    b = np.zeros(3)
    for P, d in lines:
        d = d / np.linalg.norm(d)
        Pp = np.eye(3) - np.outer(d, d)
        M += Pp
        b += Pp @ P
    c = np.linalg.solve(M, b)
    res = max(np.linalg.norm((np.eye(3) - np.outer(d / np.linalg.norm(d),
                                                   d / np.linalg.norm(d))) @ (c - P))
              for P, d in lines)
    return c, res


class AnalyticIK:
    def __init__(self, kin, wrist_tol=1e-4):
        if kin.n_act != 6:
            raise NotSphericalWrist(f'need a 6-DOF arm, got {kin.n_act}')
        self.kin = kin
        self.lower = kin.lower
        self.upper = kin.upper

        # Per-actuated-joint fixed transform (folding any fixed joints) + axis.
        self.Borg, self.ax = [], []
        acc = np.eye(4)
        for k, origin in enumerate(kin.origins):
            acc = acc @ origin
            if kin.is_actuated[k]:
                self.Borg.append(acc.copy())
                self.ax.append(np.asarray(kin.axes[k], dtype=float))
                acc = np.eye(4)

        # Folded wrist axes in the Link3 frame: fi expresses joint i's axis after
        # the preceding origin rotations, so R3_6 = R(f4)·R(f5)·R(f6)·C.
        R4 = self.Borg[3][:3, :3]
        R5 = self.Borg[4][:3, :3]
        R6 = self.Borg[5][:3, :3]
        f4 = R4 @ self.ax[3]
        f5 = R4 @ R5 @ self.ax[4]
        f6 = R4 @ R5 @ R6 @ self.ax[5]
        self.C = R4 @ R5 @ R6
        if not np.allclose(f4, f6, atol=1e-3) or abs(float(f4 @ f5)) > 1e-3:
            raise NotSphericalWrist(
                f'wrist is not a proper ZYZ Euler triplet (f4·f5={f4 @ f5:.3g}, '
                f'f4==f6={np.allclose(f4, f6, atol=1e-3)})')
        # Canonical frame B: B·f4 = ẑ, B·f5 = ŷ  ⇒  B·Q·Bᵀ = Rz·Ry·Rz.
        f4 = f4 / np.linalg.norm(f4)
        f5 = f5 / np.linalg.norm(f5)
        self.B = np.vstack([np.cross(f5, f4), f5, f4])

        # Wrist centre in the EE (Link6) frame, from the q=0 axis intersection.
        T, axes, pj = kin.fk(np.zeros(6))
        # axes/pj are *pre-rotation*; reconstruct the three wrist axis world lines.
        lines = [(pj[i], axes[i]) for i in (3, 4, 5)]
        wc0, res = _line_common_point(lines)
        if res > wrist_tol:
            raise NotSphericalWrist(f'wrist axes do not intersect (residual {res*1e3:.2f} mm)')
        Tee0 = T
        self.w6 = np.linalg.inv(Tee0)[:3, :3] @ wc0 + np.linalg.inv(Tee0)[:3, 3]

    # ── position sub-problem (J1–J3 → wrist centre) ───────────────────
    def _wc_and_jac(self, q3):
        q = np.array([q3[0], q3[1], q3[2], 0.0, 0.0, 0.0])
        T, axes, pj = self.kin.fk(q)
        wc = T[:3, 3] + T[:3, :3] @ self.w6
        J = np.zeros((3, 3))
        for i in range(3):
            J[:, i] = np.cross(axes[i], wc - pj[i])
        return wc, J

    def _position_ik(self, wc_target, seed3, iters=80, tol=1e-5, damp=1e-3, max_step=0.3):
        q3 = np.clip(np.asarray(seed3, dtype=float), self.lower[:3], self.upper[:3])
        for _ in range(iters):
            wc, J = self._wc_and_jac(q3)
            e = wc_target - wc
            if np.linalg.norm(e) < tol:
                return q3, True
            dq = J.T @ np.linalg.solve(J @ J.T + (damp ** 2) * np.eye(3), e)
            step = float(np.max(np.abs(dq)))
            if step > max_step:
                dq *= max_step / step
            q3 = np.clip(q3 + dq, self.lower[:3], self.upper[:3])
        wc, _ = self._wc_and_jac(q3)
        return q3, np.linalg.norm(wc_target - wc) < 1e-3

    def _R03(self, q3):
        T = np.eye(4)
        for i in range(3):
            T = T @ self.Borg[i] @ _axis_rot4(self.ax[i], q3[i])
        return T[:3, :3]

    # ── orientation sub-problem (J4–J6 closed-form ZYZ) ───────────────
    def _wrist_solutions(self, q3, R_target, seed_wrist):
        Q = self._R03(q3).T @ R_target @ self.C.T
        Qp = self.B @ Q @ self.B.T   # = Rz(θ4)·Ry(θ5)·Rz(θ6)
        sols = []
        if abs(Qp[0, 2]) < 1e-7 and abs(Qp[1, 2]) < 1e-7:
            # Wrist singularity (sin θ5 ≈ 0): θ4+θ6 fixed; hold θ4 at the seed.
            beta = 0.0 if Qp[2, 2] > 0 else np.pi
            alpha = float(seed_wrist[0])
            s = 1.0 if Qp[2, 2] > 0 else -1.0
            gamma = np.arctan2(Qp[1, 0], Qp[0, 0]) - s * alpha
            sols.append(np.array([alpha, beta, gamma]))
        else:
            for s in (+1.0, -1.0):
                beta = np.arctan2(s * np.hypot(Qp[0, 2], Qp[1, 2]), Qp[2, 2])
                alpha = np.arctan2(s * Qp[1, 2], s * Qp[0, 2])
                gamma = np.arctan2(s * Qp[2, 1], -s * Qp[2, 0])
                sols.append(np.array([alpha, beta, gamma]))
        return sols

    # ── public solve ──────────────────────────────────────────────────
    def solve(self, T_target, seed, position_seeds=None):
        """All valid IK solutions for a target EE transform.

        ``seed`` (6-vector) seeds the position solve and the wrist branch at a
        singularity. ``position_seeds`` optionally adds extra J1–J3 seeds to reach
        other arm branches (shoulder/elbow). Returns a list of 6-vectors, each
        within joint limits; empty if none found.
        """
        seed = np.asarray(seed, dtype=float)
        R_t, p_t = T_target[:3, :3], T_target[:3, 3]
        wc = p_t + R_t @ self.w6

        seeds = [seed[:3]]
        if position_seeds:
            seeds += [np.asarray(s, dtype=float) for s in position_seeds]

        out = []
        seen = []
        for s3 in seeds:
            q3, ok = self._position_ik(wc, s3)
            if not ok:
                continue
            if any(np.allclose(q3, p, atol=1e-4) for p in seen):
                continue
            seen.append(q3)
            for w in self._wrist_solutions(q3, R_t, seed[3:]):
                q = np.concatenate([q3, w])
                if np.all(q >= self.lower - 1e-6) and np.all(q <= self.upper + 1e-6):
                    out.append(np.clip(q, self.lower, self.upper))
        return out

    def solve_nearest(self, T_target, seed, weights=None, position_seeds=None):
        """The valid solution closest to ``seed`` (weighted joint distance), or None."""
        sols = self.solve(T_target, seed, position_seeds=position_seeds)
        if not sols:
            return None
        seed = np.asarray(seed, dtype=float)
        w = np.ones(6) if weights is None else np.asarray(weights, dtype=float)
        return min(sols, key=lambda q: float(np.sum(w * (q - seed) ** 2)))
