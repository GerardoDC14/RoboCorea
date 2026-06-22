#!/usr/bin/env python3
"""
Offline IK + RRT-Connect path planning for the dicerox 6R arm.

Used by ``sdls_servo``'s go_to_pose service: solve IK for a target end-effector
pose (seeded from a reference config so we land on the intended branch), then
plan a collision-free joint-space path from the current config to that goal with
RRT-Connect, validated against the same FCL self-collision checker the servo
uses at runtime. Pure numpy — no MoveIt / OMPL / KDL, consistent with
``arm_kinematics`` and ``sdls_servo``.

Only **self-collision** + joint limits are modelled (there is no environment
map), so a "valid" config is one that is in-limits and not self-colliding. The
planner is the heavy, one-shot path search; the servo's per-tick
``_collision_filter`` remains the runtime safety net during execution.
"""

from __future__ import annotations

import time

import numpy as np


def _log_so3(R: np.ndarray) -> np.ndarray:
    """3x3 rotation matrix -> rotation vector (axis·angle)."""
    cos_th = max(-1.0, min(1.0, (np.trace(R) - 1.0) * 0.5))
    th = np.arccos(cos_th)
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    if th < 1e-9:
        return 0.5 * v
    return (th / (2.0 * np.sin(th))) * v


# ── inverse kinematics ────────────────────────────────────────────────────────

def solve_ik(kin, T_goal, q_seed, lower, upper, *, char_length=0.25,
             pos_tol=1e-3, ori_tol=1e-2, max_iters=200, lam=0.05,
             max_step=0.2):
    """Damped least-squares IK for a target EE transform.

    Iterates dq = Jᵀ(JJᵀ + λ²I)⁻¹ e from ``q_seed`` (so a 6-DOF pose's multiple
    solutions resolve to the branch nearest the seed). Angular rows are weighted
    by ``char_length`` for unit consistency, matching the servo's task weight.

    Returns ``(q, ok)`` — ``ok`` is False if it did not converge within tolerance
    (treat as "unreachable from this seed").
    """
    L = float(char_length)
    q = np.clip(np.asarray(q_seed, dtype=float), lower, upper)
    W = np.diag([1.0, 1.0, 1.0, L, L, L])

    def err(qq):
        T = kin.fk(qq)[0]
        e_pos = T_goal[:3, 3] - T[:3, 3]
        e_ori = _log_so3(T_goal[:3, :3] @ T[:3, :3].T)
        return e_pos, e_ori

    for _ in range(max_iters):
        e_pos, e_ori = err(q)
        if np.linalg.norm(e_pos) < pos_tol and np.linalg.norm(e_ori) < ori_tol:
            return np.clip(q, lower, upper), True
        e = np.concatenate([e_pos, L * e_ori])
        Jw = W @ kin.jacobian(q)
        dq = Jw.T @ np.linalg.solve(Jw @ Jw.T + (lam ** 2) * np.eye(6), e)
        step = float(np.max(np.abs(dq))) if dq.size else 0.0
        if step > max_step:
            dq *= max_step / step
        q = np.clip(q + dq, lower, upper)

    e_pos, e_ori = err(q)
    ok = np.linalg.norm(e_pos) < pos_tol and np.linalg.norm(e_ori) < ori_tol
    return np.clip(q, lower, upper), ok


# ── RRT-Connect ───────────────────────────────────────────────────────────────

def _steer(q_from, q_to, step):
    d = q_to - q_from
    dist = float(np.linalg.norm(d))
    if dist <= step or dist < 1e-12:
        return q_to.copy(), True
    return q_from + d * (step / dist), False


def _segment_valid(a, b, is_valid, res):
    """True if every interpolated config on a→b (resolution ``res`` rad) is valid.

    Skips ``a`` (assumed already valid) and includes ``b``.
    """
    d = b - a
    n = max(1, int(np.ceil(float(np.linalg.norm(d)) / res)))
    for i in range(1, n + 1):
        if not is_valid(a + d * (i / n)):
            return False
    return True


def _backtrace(tree, idx):
    path = []
    while idx != -1:
        path.append(tree['nodes'][idx])
        idx = tree['parent'][idx]
    path.reverse()
    return path


def plan_rrt(q_start, q_goal, is_valid, lower, upper, *, step=0.1, res=0.05,
             max_time=5.0, max_nodes=5000, seed=0):
    """RRT-Connect in joint space. Returns ``(path, message)``.

    ``path`` is a list of joint vectors from ``q_start`` to ``q_goal`` (or None on
    failure). ``is_valid(q) -> bool`` is the in-limits + collision-free predicate.
    A fixed RNG ``seed`` makes a given query repeatable.
    """
    q_start = np.asarray(q_start, dtype=float)
    q_goal = np.asarray(q_goal, dtype=float)
    lower = np.asarray(lower, dtype=float)
    upper = np.asarray(upper, dtype=float)

    if not is_valid(q_start):
        return None, 'start config in collision'
    if not is_valid(q_goal):
        return None, 'goal config in collision'
    if _segment_valid(q_start, q_goal, is_valid, res):
        return [q_start, q_goal], 'direct'

    rng = np.random.default_rng(seed)
    n = len(q_start)
    Ta = {'nodes': [q_start], 'parent': [-1]}
    Tb = {'nodes': [q_goal], 'parent': [-1]}

    def nearest(tree, q):
        arr = np.asarray(tree['nodes'])
        return int(np.argmin(np.sum((arr - q) ** 2, axis=1)))

    def extend(tree, q):
        """Grow ``tree`` one step toward ``q``: ('reached'|'advanced'|'trapped', idx)."""
        i = nearest(tree, q)
        q_near = tree['nodes'][i]
        q_new, reached = _steer(q_near, q, step)
        if not _segment_valid(q_near, q_new, is_valid, res):
            return 'trapped', -1
        tree['nodes'].append(q_new)
        tree['parent'].append(i)
        j = len(tree['nodes']) - 1
        return ('reached' if reached else 'advanced'), j

    def connect(tree, q):
        s, j = 'advanced', -1
        while s == 'advanced':
            s, j = extend(tree, q)
        return s, j

    t0 = time.time()
    while (time.time() - t0) < max_time and \
            (len(Ta['nodes']) + len(Tb['nodes'])) < max_nodes:
        q_rand = lower + (upper - lower) * rng.random(n)
        s, ja = extend(Ta, q_rand)
        if s != 'trapped':
            s2, jb = connect(Tb, Ta['nodes'][ja])
            if s2 == 'reached':
                path = _backtrace(Ta, ja) + _backtrace(Tb, jb)[::-1]
                # trees swap each round, so orient the result to start at q_start
                if np.linalg.norm(path[0] - q_start) > np.linalg.norm(path[-1] - q_start):
                    path = path[::-1]
                return path, 'rrt-connect'
        Ta, Tb = Tb, Ta

    return None, 'planning timeout'


def shortcut(path, is_valid, res, *, iters=100, seed=1):
    """Greedy random shortcutting: collapse waypoints whose direct segment is free."""
    if path is None or len(path) <= 2:
        return path
    rng = np.random.default_rng(seed)
    path = [np.asarray(p, dtype=float) for p in path]
    for _ in range(iters):
        if len(path) <= 2:
            break
        i = int(rng.integers(0, len(path) - 1))
        j = int(rng.integers(i + 1, len(path)))
        if j - i < 2:
            continue
        if _segment_valid(path[i], path[j], is_valid, res):
            path = path[:i + 1] + path[j:]
    return path
