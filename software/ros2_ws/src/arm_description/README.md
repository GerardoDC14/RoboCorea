# arm_description — full RoboCorea robot model (arm + chassis + flippers)

The **combined** robot description: the 6-DOF arm bolted onto the tracked chassis
with its four articulated flippers, merged into one URDF. It exists so the
**digital twin** shows the whole robot (not just the floating arm) and so the
arm's **body-collision** avoidance has the chassis + flippers to avoid.

```
arm_description/
├── urdf/dicerox_full.urdf          # the whole robot, one plain URDF
├── meshes/visual/                  # full-res chassis + flipper STLs (twin/RViz)
├── meshes/collision/               # decimated chassis + flipper proxies (FCL, .stl)
├── launch/display.launch.py        # standalone RViz view (joint sliders)
└── scripts/make_body_collision.py  # regenerate the low-poly collision proxies
```

## The model

`dicerox_full.urdf` is hand-merged (plain URDF, no xacro, so launch files can
read it as a string), following the legacy `dicerox_moveit` recipe:

- the **arm** (`world_link → base_link → Link1..Link6`) is reproduced verbatim
  from `arm_teleop/urdf/dicerox_arm.urdf`; the arm link meshes stay in
  `arm_teleop` and are referenced by `package://arm_teleop` URIs (resolved at
  runtime by ament_index — there is deliberately **no** dependency back on
  `arm_teleop`, which would be a cycle since `arm_teleop` depends on this).
- the **chassis + flippers** (`chassis_base_link`, `Flipper1L..Flipper4L` on the
  revolute `Flipper1J..Flipper4J`) come from the legacy `dicerox_urdf_v1`; their
  meshes live here.
- the chassis is bolted under the arm's `world_link` at the same offset the
  legacy MoveIt model used.

Joint names that drive `/joint_states`:

| Joints | Published by |
|---|---|
| `Joint1..Joint6` (arm) | `arm_teleop` `servo_node` |
| `Flipper1J..Flipper4J` | `arm_teleop` `flipper_state` (from `/encoders/flipper`, `[fl, fr, rl, rr]`) |

## Consumers

- **robot_state_publisher** serves this URDF on `/robot_description`; the GUI
  `UrdfViewer` (digital twin) and RViz render it, using the `meshes/visual` STLs.
- **`arm_teleop/sdls_servo`** loads the same URDF for arm kinematics (it only
  traces `base_link → Link6`, so the chassis/flipper joints are ignored there)
  **and** for the base-frame transforms of the chassis + flippers used by its
  `body_collision` checker (which uses the lighter `meshes/collision` proxies).

## Quick look

```bash
colcon build --packages-select arm_description
source install/setup.bash
ros2 launch arm_description display.launch.py     # RViz + sliders for every joint
```

For the operator stack (twin + servo + GUI + live flippers) use
`ros2 launch gui bringup.launch.py`.

## Regenerating the collision proxies

The full-res chassis mesh is ~188k triangles over ~2990 components (mostly
bolts) — far too heavy for a 100 Hz FCL query. Regenerate the low-poly proxies
after a mesh change:

```bash
pip install trimesh fast-simplification     # one-time
python3 src/arm_description/scripts/make_body_collision.py
```

It keeps the structural shell (dropping sub-`--min-vol` components so the concave
top deck where the arm sits is preserved, unlike a convex hull) and decimates to
`--chassis-target` / `--flipper-target` triangles.
