# Test robot: spoon_bot (base + slider + 3-joint arm)

The third test robot, and the first that uses all three supported joint
kinds. The Fusion design is `simple_robot`, and its screenshots are in
[../images/](../images/). The README guide uses spoon_bot as its example.
Units and frames are as in [frames-and-units.md](frames-and-units.md).

## Export `spoon_bot` (2026-11-06)

Exported from real Fusion into `C:/Users/patdu/Desktop/fftest/spoon_bot`:
12 links and 11 joints. xacro 2.1.1 (PyPI) processes the output.

| Link ← component | Fusion joint | URDF type | Parent | Mass [kg] |
|---|---|---|---|---|
| `spoon_bot_base_link` ← `base_link:1` | – | – | – | 0.817 |
| `arm_base_1` | `Rigid 7` | `fixed` | base | 0.368 |
| `dm9095_oak_d_lite_depthai_usb3c_1` | `Rigid 6` | `fixed` | base | 0.157 |
| `stl_19p_3d_1` | `Rigid 5` | `fixed` | base | 0 (no inertial) |
| `wheel_1` … `wheel_4` | `Revolute 1`–`4` | `continuous` | base | 0.043 each |
| `rob_arm_1` | `Slider 8` | `prismatic` | `arm_base_1` | 0.122 |
| `arm_rota_1` | `Revolute 9` | `revolute` | `rob_arm_1` | 0.211 |
| `arm_rota_2` | `Revolute 10` | `revolute` | `arm_rota_1` | 0.211 |
| `arm_spoon_1` | `Revolute 11` | `revolute` | `arm_rota_2` | 0.078 |

- The camera and the lidar are linked designs. The lidar has no material,
  so its mass is 0.
- The lidar got the warning `Mesh frame check uncertain (363.5% bbox
  mismatch)`. The mesh is still in the right place, on top of the base:
  - lidar box in the base frame: x 0.041…0.088 m, z 0.050…0.085 m
  - top of the base: z 0.050 m

  The cause of the mismatch is not known (Q23).
- `base_link:1`, `wheel:1` and the lidar sit at the world origin with an
  identity transform, so they were reported as "world frame". For these
  three components, world and local are the same.
- All four wheel STLs are identical. Copies of one component are written in
  the component's own frame, and the visual origin places each copy.
- The meshes total 9.7 MB; the camera alone is 8.2 MB.

## In ROS 2 Jazzy on Linux (2026-11-06)

- The re-export (with the limit fix) builds with `colcon build` and runs
  with `ros2 launch spoon_bot display.launch.py`.
- `joint_state_publisher_gui` shows 8 sliders: `wheel_1_joint` …
  `wheel_4_joint`, `rob_arm_1_joint`, `arm_rota_1_joint`,
  `arm_rota_2_joint`, `arm_spoon_1_joint`. The fixed joints get none.
- The project owner moved the joints in RViz and reports that it works
  well. Screenshots: [rviz.png](../images/rviz.png),
  [joint_controll.png](../images/joint_controll.png), also in the root
  README.
- `config/joints.yaml` of the re-export: `arm_rota_1_joint` limits
  −1.571 … 3.142 rad, no −2π shift.

## Joint geometry at the export pose (computed from the xacro)

| Joint | Origin in base frame [m] | Axis in base frame |
|---|---|---|
| `wheel_1_joint` (front right) | 0.09 −0.13 0.025 | 0 −1 0 |
| `wheel_2_joint` (front left) | 0.09 0.13 0.025 | 0 1 0 |
| `wheel_3_joint` (rear right) | −0.09 −0.13 0.025 | 0 −1 0 |
| `wheel_4_joint` (rear left) | −0.09 0.13 0.025 | 0 1 0 |
| `rob_arm_1_joint` | −0.04 0 0.188 | 0 0 1 |
| `arm_rota_1_joint` | 0.0025 −0.0325 0.208 | 0 −1 0 |
| `arm_rota_2_joint` | 0.152 −0.0225 0.194 | 0 1 0 |
| `arm_spoon_1_joint` | 0.264 −0.0325 0.293 | 0 −1 0 |

- The wheel origins are at the wheel centres. The wheel box spans
  z −0.010…0.060 m, so the radius is 0.035 m.
- The right wheels turn about −Y and the left wheels about +Y. Setting
  `flip: true` on `wheel_1_joint` and `wheel_3_joint` makes all four turn
  about +Y.
- Forward is +X: the camera is at x 0.10 m.

## Joint values and limits

At export, Fusion showed the slider at −42.051 mm and the arm joints at
−5°, −47° and −49° (screenshot `example_bot.png`). The API returned the
three revolute values as 354.6°, 313.2° and 311.0°, that is, in [0, 2π).

Before the fix of 2026-11-06, this shifted the URDF limits by −2π. For
example, `Revolute 9` came out as −7.76 … −3.05 rad. Corrected values:

| Joint | Fusion limits | Value at export | URDF limits |
|---|---|---|---|
| `Slider 8` | −160 … 0 mm | −42.05 mm | −0.1179 … 0.0421 m |
| `Revolute 9` | −90° … 180° | −5.4° | −84.6° … 185.4° |
| `Revolute 10` | −140° … 140° | −46.8° | −93.2° … 186.8° |
| `Revolute 11` | −90° … 90° | −49.0° | −41.0° … 139.0° |

`Revolute 11` was checked in the Edit Joint dialog
(`docs/images/joint_image_config_1.png`, `joint_image_config_2.png`):
component 1 `arm_spoon:1`, component 2 `arm_rota:2`, Z offset 30 mm, axis Z,
limits −90° … 90°. These match the limits worked back from the export.

The Fusion limits in this table are worked back from the exported values.
They come out as round numbers, which confirms that the values are returned
in [0, 2π).
