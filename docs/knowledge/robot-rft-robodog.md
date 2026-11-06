# Target robot: rft_robodog

The first robot the exporter is tested on. A 4-wheel skid-steer base with an
OAK-D Lite camera and a Unitree STL-19P lidar. Units and frames as in
[frames-and-units.md](frames-and-units.md).

## Fusion model

- Z is up. The `base_link` origin is at the bottom of the chassis.
- **Forward is −X in Fusion** (the camera looks that way). The REP-103
  option is needed for this robot, see
  [proposal 0001](../proposals/0001-ros-axis-convention.md).

## Component poses relative to `base_link` (m / rad)

| Component | xyz | rpy |
|---|---|---|
| `wheel_left_back:1` | 0.175092 −0.2 0.040096 | 1.5708 1.5708 0 |
| `wheel_right_back:1` | 0.175092 0.2 0.040096 | 1.5708 1.5708 0 |
| `wheel_left_front:1` | −0.175092 −0.2 0.040096 | −1.5708 1.5708 0 |
| `wheel_right_front:1` | −0.175092 0.2 0.040096 | −1.5708 1.5708 0 |
| `OAK_D_LITE_DepthAI:1` | 0.090657 0.023671 0.646387 | 1.7453 0 −1.5708 |
| `STL_19P_3D:1` | 0.253 0 0.115098 | 0 0 1.5708 |

These poses are also used in `tests/fake_design.py`.

## Notes per component

- **Wheels:** origins are exactly at the wheel centres, so they can serve as
  joint frames.
- **OAK-D Lite:** the component origin is **not** the optical centre. It is
  about 41 mm off in Y. The RGB lens centre is at about
  (0.0958, −0.0173, 0.6636).
- **Lidar:** may be mounted upside down (its subcomponents have roll 180°).
  Check the scan direction in RViz (Q7).
- **Wheel radius:** not measured yet. Needed for the `base_footprint` height
  and the drive controller (Q8).
