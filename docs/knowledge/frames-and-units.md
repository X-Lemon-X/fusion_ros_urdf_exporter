# Frames and units

The one reference for units and coordinate frames. Other pages link here.

## Units

- In these docs and in the generated URDF everything is **SI**: lengths in
  m, angles in rad, mass in kg, inertia in kg·m².
- The Fusion API gives values in its own internal units, whatever the
  document units are. See [fusion-api.md](fusion-api.md#units).

| Constant | Value | Use |
|---|---|---|
| `CM_TO_M` | `0.01` | All API lengths (cm) → m |
| `KGCM2_TO_KGM2` | `1e-4` | API inertia (kg·cm²) → kg·m² |

## Frames

| Frame | Definition |
|---|---|
| Fusion world | Root component frame of the design |
| Component frame | Origin and axes of the Fusion component (`occurrence.transform2`) |
| Link frame | The Fusion joint frame when the component has a joint, else the component frame. Rules in [joints.md](joints.md#link-frame) |
| Base link | Component frame of the top-level component `base_link`, named `<pkg>_base_link` |
| Joint origin | Child link frame in the parent link frame: `inv(T_parent_link) * T_link` |

- Axes are kept **1:1 with Fusion**. The output is not rotated to the ROS
  convention (REP-103: X forward, Y left, Z up). If the model's forward
  direction is not +X, the URDF's isn't either. See
  [proposal 0001](../proposals/0001-ros-axis-convention.md).

## Pose class

`Pose` is a rigid transform: `p_parent = R * p_child + t`, with `t` in m.

- `Pose.from_matrix3d(m)` reads the 3×3 rotation and the translation
  (column 3, cm → m) of a Fusion `Matrix3D`.
- `a * b` applies `b` first, then `a`.
- `inverse()` returns `(Rᵀ, −Rᵀ t)`.
- `rpy()` uses the URDF convention `R = Rz(yaw) · Ry(pitch) · Rx(roll)`.
  At gimbal lock (`|cos(pitch)| ≤ 1e-9`) it sets yaw to 0.
- Poses are written with 6 decimals (`fmt_pose`), values below 1e-9 written
  as 0. Masses and inertias use 9 significant digits (`fmt_num`).
