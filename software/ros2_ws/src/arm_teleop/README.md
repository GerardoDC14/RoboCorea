# arm_teleop — singularity-robust servo for the dicerox arm

Real-time Cartesian → joint differential IK for the dicerox 6-DOF arm, built to
stay controllable **through** singularities, where both MoveIt Servo and the
legacy isotropic DLS (`reference/.../damped_servo.py`) misbehave.

## Why a new solver

| Approach | Singularity behaviour | Problem |
|---|---|---|
| Isotropic DLS `Jᵀ(JJᵀ+λ²I)⁻¹` | damps **all** Cartesian axes equally | near a singularity it bends the EE off the commanded direction and slows the healthy axes; triggers on a condition-number *ratio* that fires on merely-extended poses |
| MoveIt Servo | scales the whole twist + **hard-stops** at threshold | the arm **freezes** mid-motion; hysteresis ping-pongs at the boundary |
| **SDLS (this)** | damps **each singular direction by its own σ** | 5 healthy axes track exactly, only the collapsing DOF is bounded; never halts, never explodes |

The dicerox arm is a **non-spherical 6R** (J4→J5 = 0.345 m, J5→J6 = 0.189 m), so
there is no closed-form IK — numerical differential IK is the only option.

## The solver (`sdls_servo.py`)

Each cycle, with the unit-weighted Jacobian `J = U Σ Vᵀ`:

```
dq = Σ_i  f_i (u_i · dx) v_i ,   f_i = σ_i / (σ_i² + λ_i²)
λ_i² = (1 - (σ_i/σ0)²)·λ_max²   for σ_i < σ0,   else 0
```

- `σ_i ≥ σ0` → `f_i = 1/σ_i` (exact tracking).
- `σ_i → 0` → `f_i → 0` (that direction is given up, smoothly and boundedly).

Worst-case joint-velocity gain in the collapsing direction is `~1/(2·λ_max)`.
Angular Jacobian rows are weighted by `char_length` (m) so `σ0` means the same
thing for shoulder/elbow (position) and wrist (orientation) singularities.

State is the **integrated command** (open-loop), seeded from `initial_positions`
— the firmware path is position-command and arm telemetry is too slow to close
the loop on. The 6-joint result is published as `sensor_msgs/JointState` on
**`/joint_states`** (rad), which `esp32_bridge` forwards to the ESP32 as
`MSG_ARM_JOINTS`, and which the GUI digital twin renders.

`arm_kinematics.py` parses the URDF and builds FK + the geometric Jacobian using
each joint's real `<axis>` (Rodrigues), so dicerox's `Joint4`/`Joint6` (`0 0 -1`)
are signed correctly — unlike the legacy class which hard-coded +Z. It also
exposes `link_poses(q)` (world transform per link) for the collision checker.

## Self-collision avoidance (`self_collision.py`)

Beyond joint limits, the servo does **mesh-on-mesh** self-collision avoidance so
the operator can't drive one link into another. It is **on by default** and
degrades gracefully (warns, keeps running without it) if its deps are missing.

How it works:

- **Geometry:** one **low-poly** FCL `BVHModel` per link. The full-res CAD
  collision STLs (Link6 ≈ 118k triangles) are ~20–30× too heavy for a 100 Hz
  query — a single separated `Link6` distance query is ~4.4 ms raw vs ~0.16 ms
  decimated. `scripts/make_collision_meshes.py` decimates `meshes/collision/` →
  `meshes/collision_lowpoly/` (≈6% of the triangles), committed to the repo.
- **ACM:** an allowed-collision matrix is learned once at startup by sampling
  random valid configs — adjacent links (always touching at their joint) and
  unreachable pairs are dropped, leaving only the pairs that can actually clash
  (5 for this arm).
- **Broadphase:** a per-pair world-AABB test skips the expensive BVH query for
  pairs farther apart than the slow band, so a full sweep is ~0.14 ms; the gate
  evaluates `q` and `q+dq` (two sweeps) only while the arm is moving.
- **Policy (look-ahead damper):** a step that drives a pair into the **slow
  band** is scaled down; into the **stop band** it is vetoed. The constraint
  only applies to motion that *reduces* a pair's clearance — **retreating is
  always allowed**, so the arm can never get stuck against itself.

Regenerate the proxies after a URDF/mesh change:

```bash
pip install trimesh fast-simplification          # one-time, for the generator
python3 src/arm_teleop/scripts/make_collision_meshes.py --target 1200
```

Runtime needs `python-fcl` + `trimesh` on the workstation (`pip install
python-fcl trimesh`). Set `collision_check: false` to disable.

## Run

```bash
colcon build --packages-select arm_teleop
source install/setup.bash

# servo only (drive it with your own TwistStamped on /servo_node/delta_twist_cmds)
ros2 launch arm_teleop servo.launch.py

# servo + keyboard (w/s=X a/d=Y r/f=Z u/j=Roll i/k=Pitch o/l=Yaw, m=mode, q=quit)
ros2 launch arm_teleop keyboard.launch.py

# servo + joystick (needs ros-<distro>-joy)
ros2 launch arm_teleop joystick.launch.py
```

On the robot, `esp32_bridge` consumes `/joint_states` → `MSG_ARM_JOINTS`. Its
`joint_names` param must match (`Joint1..Joint6`, now the default).

## Tuning (`config/servo_params.yaml`)

| Param | Effect |
|---|---|
| `sigma_threshold` (σ0) | how early damping engages; ↑ = smoother/earlier give-up |
| `lambda_max` | peak damping; ↑ = safer/softer (lower joint-velocity spikes) |
| `char_length` | wrist→tool distance; balances position vs orientation singularities |
| `max_linear_speed` / `max_rotational_speed` | full-deflection Cartesian speed |
| `max_joint_speed` | hard per-joint velocity ceiling |
| `joint_limit_scale_zone` / `_margin` | soft slow-down band / hard stop-out |
| `smoothing_alpha` | input EMA (1.0 = off) |
| `collision_check` | master enable for mesh self-collision avoidance |
| `collision_slow_dist` / `_stop_dist` | clearance (m) to start damping / veto approach |
| `collision_acm_samples` | configs sampled to learn the allowed-collision matrix |

> **Bench-verify before powered motion:** `initial_positions` must match the
> arm's actual startup pose, and the firmware `*_DIR_*` signs (J4/J5 = −1) must
> map this URDF/MoveIt joint convention to motor motion correctly.
