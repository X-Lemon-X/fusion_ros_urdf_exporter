# Test robot: sdrac1 (6-axis arm)

The second robot the exporter was tested on, and the first with Fusion
joints. A 6-axis serial arm: `base_link` plus the top-level components
`axis1` … `axis6`, chained by the revolute joints `Rev1` … `Rev6`. Units and
frames as in [frames-and-units.md](frames-and-units.md).

## Export `sdrac1` (2026-11-06)

Exported from real Fusion into `C:/Users/patdu/Desktop/fftest/sdrac1`.
No warnings in the summary. xacro 2.1.1 (PyPI) processes the output.

| Link | Fusion joint | Parent | Mass [kg] | Mesh |
|---|---|---|---|---|
| `sdrac1_base_link` | – | – | 7.266 | world frame, mm |
| `axis1_1` | `Rev1` | `sdrac1_base_link` | 3.727 | local frame, mm |
| `axis2_1` | `Rev2` | `axis1_1` | 5.242 | local frame, mm |
| `axis3_1` | `Rev3` | `axis2_1` | 6.818 | local frame, mm |
| `axis4_1` | `Rev4` | `axis3_1` | 1.455 | local frame, mm |
| `axis5_1` | `Rev5` | `axis4_1` | 0.646 | local frame, mm |
| `axis6_1` | `Rev6` | `axis5_1` | 0.110 | local frame, mm |

- All six joints are `continuous`: the Fusion joints have no limits.
- Every joint has `frame: joint`; the axis in the link frame is
  `[0, 0, 1]`, i.e. the Fusion joint frame's Z is the rotation axis.
- The `base_link` component is at the world origin, so its world and local
  boxes are the same and the detector reported "world frame". The result is
  the same either way.
- Meshes total 105 MB (`axis2_1.stl` and `axis3_1.stl` 28 MB each).

## Forward kinematics at zero (computed from the xacro, 2026-11-06)

Joint origins and axes in the `sdrac1_base_link` frame (m):

| Joint | Origin | Axis |
|---|---|---|
| `axis1_1_joint` | 0.0005 0 −0.0005 | 0 0 −1 |
| `axis2_1_joint` | −0.0315 0 0.0675 | −1 0 0 |
| `axis3_1_joint` | −0.0155 0.0036 0.4425 | 1 0 0 |
| `axis4_1_joint` | 0.0125 0.2536 0.4425 | 0 −1 0 |
| `axis5_1_joint` | 0.0477 0.3786 0.4426 | 1 0 0 |
| `axis6_1_joint` | 0.0125 0.4070 0.4426 | 0 −1 0 |

- This is a plausible 6R arm in its Fusion pose: base yaw, shoulder and
  elbow pitch about X, upper arm 0.375 m along Z, forearm along +Y, wrist
  roll / pitch / roll.
- `axis1_1_joint` points down (−Z): the Fusion joint frame is flipped. Use
  `flip: true` in the override if positive rotation must be
  counter-clockwise seen from above.
- Each mesh lands at its Fusion component pose (all rotations identity;
  child meshes at z −2.5 mm). This follows from the construction
  (`pose_in_parent = inv(T_parent)·T_link`), so it does not test Q21 by
  itself. Q21 is tested by moving the joints in RViz.
