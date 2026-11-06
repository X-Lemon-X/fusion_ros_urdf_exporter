# Joints and kinematic tree (`JointReader`, `KinematicTree`, `fusion_joint.xacro`)

How the exporter turns Fusion joints into URDF joints, and how the user
changes them afterwards. Designed in
[0003](../proposals/0003-joints-from-fusion-xacro.md). Units and frames as in
[frames-and-units.md](frames-and-units.md).

## Reading joints (`JointReader`)

- Reads `rootComponent.joints` and `rootComponent.asBuiltJoints`. Suppressed
  joints (`isSuppressed`) are skipped.
- Per joint: `name`, `occurrenceOne`, `occurrenceTwo`, `jointMotion.jointType`.
- Frames: `geometryOneTransform` / `geometryTwoTransform` for joints,
  `transform` (one frame for both sides) for as-built joints. Read as
  `Matrix3D`, translation cm → m.
- Revolute: `rotationAxisVector`, `rotationValue` (rad), `rotationLimits`.
  Slider: `slideDirectionVector`, `slideValue` (cm → m), `slideLimits`
  (cm → m). A limit counts only if `isMinimumValueEnabled` /
  `isMaximumValueEnabled` is true.
- Every API call is wrapped: a missing value becomes `None` instead of an
  exception.

## Building the tree (`KinematicTree.build`)

1. Each joint end is mapped to its top-level occurrence by walking
   `assemblyContext` up. `None` (root component) maps to the base link.
   Components are matched by occurrence `name`.
2. Skipped: joints whose two ends are in the same top-level component
   (internal), and joints to a hidden component (warning on the visible end).
3. **Component one = child, component two = parent.** If component one is
   `base_link`, the joint is flipped (warning) and the frame of side two is
   used.
4. A component with joints to two different parents → `ExportError` listing
   the joints. Several joints to the same parent → the first non-rigid one
   is used (warning).
5. A component with no parent is attached to `<pkg>_base_link` with a
   `fixed` joint and keeps its component origin as link frame.
6. A cycle → `ExportError` naming the components and joints in the loop.
7. Output order: base link, then breadth-first, siblings sorted by link name.

All validation runs before any file is written.

## Link frame

| Case | Link frame (`frame_source`) |
|---|---|
| Joint with readable geometry | Joint frame on the child side (`geometryOneTransform`; side two when flipped) → `joint` |
| Joint, geometry not readable | Component origin + warning → `origin` |
| `frame: origin` in `joints_override.yaml` | Component origin → `origin` |
| No joint, or the base link | Component origin → `origin` |

The joint origin in URDF is `inv(T_parent_link) · T_link`
(`ComponentInfo.pose_in_parent`).

## Type, axis and limits (`KinematicTree._joint_spec`)

| Fusion | URDF | Limits |
|---|---|---|
| rigid | `fixed` | — |
| revolute, both limits enabled | `revolute` | `lower = min − rotationValue`, `upper = max − rotationValue` |
| revolute, 0 or 1 limit | `continuous` (warning if exactly 1) | — |
| slider, both limits | `prismatic` | `min − slideValue`, `max − slideValue` |
| slider, not both | `prismatic` + warning | `0.0`, `0.0` |
| cylindrical, pin-slot, planar, ball, unknown | `fixed` + warning | — |

- The zero position of every URDF joint is the pose at export time.
- Axis: the Fusion axis vector, normalized and rotated into the link frame
  (`R_linkᵀ · axis`). If it can't be read: `0 0 1` + warning.
- Axis-line check, only for revolute joints whose link frame is the
  component origin: if the axis line (through the origin of the joint frame
  on side two, else side one) is more than `axis_offset_tolerance` (0.1 mm)
  from the link origin, a warning gives the distance in mm.
- Joint name: `<child_link>_joint`.

## Generated files

- `urdf/<pkg>.urdf.xacro`: links, then one
  `<xacro:fusion_joint name parent child xyz rpy/>` per joint. Loads both
  YAML files with `xacro.load_yaml('../config/…')` (path relative to the
  xacro file).
- `urdf/fusion_joint.xacro`: the macro `fusion_joint`. Per key it uses
  `joints_override.yaml` if set, else `joints.yaml`. `flip: true` multiplies
  the axis by −1 and turns `[lower, upper]` into `[−upper, −lower]`. `fixed`
  gets no `<axis>`; `revolute`/`prismatic` get `<limit lower upper effort
  velocity>`; `continuous` gets `<limit effort velocity>`.
- `config/joints.yaml`: per joint `type`, `axis`, `flip: false`, `lower`,
  `upper`, `effort` (`10.0`), `velocity` (`1.0`), with a comment naming the
  parent, the Fusion joint and the frame source. Floats always have a `.`
  (`fmt_yaml_float`), because YAML 1.1 parsers read `1e-05` as a string.
  With no joints the file is `{}`.
- `config/joints_override.yaml`: created once from `OverrideFile.TEMPLATE`
  (comments only), never overwritten or deleted. xacro reads a comments-only
  file as `None`; the macro handles that.

## Override file parsing in the exporter (`OverrideFile`)

The exporter needs only two things from the override file, so it uses a line
parser, not a YAML library:
- top-level keys (`^name:`), to list overrides without a matching joint in
  the summary,
- `frame: joint|origin` under a key, applied on the next export.

Other keys (`type`, `axis`, `flip`, limits, `effort`, `velocity`) are read
by xacro at launch time, so they need no re-export.

## xacro facts (measured with xacro 2.1.1 from PyPI, 2026-11-06)

- `xacro.load_yaml` with a path relative to the current xacro file works.
- `.get()` works on the loaded data.
- **Inside `${…}` a `{` breaks the expression**: xacro ends `${` at the
  first `}`. Use `dict()` instead of `{}`.

## Known limitations (as of 2026-11-06)

- Not run in real Fusion on a model with joints yet: the first real export
  (`ttt1`, 2026-11-06) read none (Q22). The frame space of the joint vectors
  and matrices is assumed to be world (Q21).
- No `mimic`, no closed loops, no joint types beyond the table above.
- `parent`, `child`, `xyz`, `rpy` can't be overridden; change them in Fusion.
