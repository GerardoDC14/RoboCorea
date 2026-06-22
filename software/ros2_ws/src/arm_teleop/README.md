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

The dicerox arm **does have a spherical wrist** — the J4/J5/J6 axes intersect at
a common point (verified: 0 mm residual at every configuration), and the wrist is
a clean proper-Euler **ZYZ** triplet with the wrist centre 0.189 m back along the
tool axis. So a closed-form, kinematically-decoupled IK *is* available
(`arm_ik.py`: closed-form ZYZ orientation + a 3-DOF wrist-centre position solve;
the ~3° frame tilts on J2/J3 are why the position half isn't pure closed form).
The earlier "non-spherical" note was wrong — it inferred non-intersection from the
large J4→J5 = 0.345 m / J5→J6 = 0.189 m link offsets, but those are *along* the
wrist axes, not perpendicular, so the axes still concur.

A **differential** servo is still the core for real-time teleop (continuous,
singularity-robust, no solution-branch flips); the analytic solver is used for
go-to-pose goals and is available as an opt-in resolved-pose servo mode (with the
differential SDLS step as the singularity/limit fallback) — see below.

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
  decimated. `scripts/make_collision_meshes.py` decimates `meshes/collision_src/`
  → `meshes/collision/` (lowercase `.stl`, ≈6% of the triangles), committed to
  the repo.
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

## Body collision — chassis + flippers (`body_collision.py`)

The servo also avoids the **robot body**: the tracked chassis and the four
articulated flippers, not just the arm's own links. Same FCL look-ahead damper
(and the same `is_valid` gate for RRT planning), with the obstacle set read from
the **combined URDF** (`arm_description/urdf/dicerox_full.urdf`):

- the **chassis** is one fixed low-poly mesh, posed in the arm `base_link` frame
  straight from the URDF mount (no hard-coded offsets);
- each **flipper** is a mesh whose pose is driven **live from `/encoders/flipper`**
  — the servo reads the flipper joint angles back off `/joint_states` (the same
  URDF-convention radians the `flipper_state` bridge publishes, so the collision
  flippers match the digital twin exactly), so **raising a flipper actually
  blocks the arm**.
- an ACM (sampling arm configs × flipper angles) drops pairs that are always in
  contact — e.g. the arm mount resting on the chassis deck — so they never
  permanently veto motion.

Low-poly body proxies live in `arm_description/meshes/collision/`
(`scripts/make_body_collision.py` there decimates the chassis to ~5k tris and
drops the ~2900 bolt components; flippers to ~700). On by default; needs the
combined URDF (with the arm-only URDF it disables itself cleanly). Tunables:
`body_collision`, `body_collision_slow_dist` / `_stop_dist`,
`body_collision_acm_samples`, `flipper_sample_deg`.

## Full-robot digital twin & bring-up

The digital twin (GUI panel / RViz) renders the **whole** robot — arm + chassis +
flippers — from the combined URDF on `/robot_description` and `/joint_states`.
Two publishers feed `/joint_states` and the consumers merge per-joint:

- `servo_node` → the 6 **arm** joints (`Joint1..Joint6`);
- `flipper_state` → the 4 **flipper** joints (`Flipper1J..Flipper4J`), converting
  `/encoders/flipper` (deg, `[fl, fr, rl, rr]`) to radians. Encoder convention
  (zero offset, sign, 0..360 wrap) is bench-tunable via its `angle_offsets_deg`,
  `joint_signs`, `wrap_pm180` params.

The **gui** package's `bringup.launch.py` starts all of it in one shot on the
workstation — robot_state_publisher (combined URDF), the servo (self- **and**
body-collision), `flipper_state`, and the operator GUI (the operator console owns
the bring-up, so it lives in `gui`, not here):

```bash
ros2 launch gui bringup.launch.py                 # GUI + twin + servo
ros2 launch gui bringup.launch.py joystick:=true  # + gamepad teleop
ros2 launch gui bringup.launch.py use_gui:=false use_rviz:=true
```

It does **not** start `esp32_bridge` (that runs on the Jetson); on the
workstation `/encoders/flipper` and friends arrive over DDS from the robot. With
no bridge running the flippers just stay level and the arm still jogs/plans.

## Saved poses + go-to-pose (RRT planning)

The servo can **store named end-effector poses** and **return to them** with a
collision-free, RRT-planned joint motion — driven from the GUI's digital-twin
panel or from the CLI/RViz via services (so it works in the `twin.launch.py`
flow too). The pose library is server-side (`pose_library_path`, default
`~/.config/robocorea_arm/poses.yaml`), so the GUI and CLI share one source.

How a *go* works: re-solve IK for the saved EE pose (seeded from the saved joint
snapshot, so a 6R arm's multiple solutions resolve to the intended branch) →
`arm_planner.plan_rrt` finds a collision-free joint path (self-collision + limits
as the validity check, the **same** FCL checker used at runtime) → shortcut
smoothing → the loop streams the path into `/joint_states`. A **non-zero stick /
twist instantly aborts** an in-progress move (operator override), as does a fault
or `pause_servo`. Planning runs on a worker thread (MultiThreadedExecutor) so the
100 Hz `/joint_states` output never stalls.

Services on `servo_node` (`rescue_interfaces/srv/*`):

| Service | Action |
|---|---|
| `~/save_pose {name}` | snapshot the current EE pose under `name` |
| `~/go_to_pose {name}` *(or `{use_pose: true, pose: …}`)* | plan + move to it |
| `~/delete_pose {name}` | remove a saved pose |
| `~/list_poses` | list names + poses |

Plus `~/ee_pose` (`PoseStamped`, live EE pose, ~20 Hz) and `~/plan_state`
(`String`, latched: `idle` / `planning` / `moving k/n` / `reached` / `aborted …`
/ `unreachable`).

```bash
# software-only twin: RViz + robot_state_publisher + servo (services included)
ros2 launch arm_teleop twin.launch.py
# drive it from a 2nd terminal:  ros2 run arm_teleop keyboard_servo

ros2 service call /servo_node/save_pose  rescue_interfaces/srv/SavePose  "{name: inspect}"
ros2 service call /servo_node/list_poses rescue_interfaces/srv/ListPoses "{}"
ros2 service call /servo_node/go_to_pose rescue_interfaces/srv/GoToPose  "{name: inspect}"
```

Needs `python-fcl` + `trimesh` for collision-aware planning (same dep as the
runtime checker); without them the planner falls back to a limits-only validity
check. RRT tuning (`rrt_step`, `rrt_resolution`, `rrt_max_time`, `rrt_seed`) and
IK tolerances live in `config/servo_params.yaml`.

> **Caveat:** a *go* moves the **physical** arm autonomously (via the bridge). It
> is gated on `respect_fault` and is instantly stick-overridable; saved poses are
> only meaningful relative to the same boot-zero (`initial_positions`).

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

# everything (full-robot twin + servo + flipper bridge + GUI) — see above
ros2 launch gui bringup.launch.py
```

On the robot, `esp32_bridge` consumes `/joint_states` → `MSG_ARM_JOINTS`. It must
ignore the flipper joints and forward only `Joint1..Joint6` (its `joint_names`
param, now the default); `flipper_state` and `servo_node` co-publish on
`/joint_states`.

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
| `body_collision` | master enable for arm-vs-body (chassis + flippers) avoidance |
| `body_collision_slow_dist` / `_stop_dist` | clearance (m) to start damping / veto approach to the body |
| `flipper_sample_deg` | ± flipper range sampled when learning the body ACM |
| `ik_servo` | use the closed-form resolved-pose servo (off = differential SDLS) |
| `ik_servo_sigma_min` / `_max_jump` | when to fall back from IK servo to SDLS (singularity / branch switch) |

> **Bench-verify before powered motion:** `initial_positions` must match the
> arm's actual startup pose, and the firmware `*_DIR_*` signs (J4/J5 = −1) must
> map this URDF/MoveIt joint convention to motor motion correctly.
