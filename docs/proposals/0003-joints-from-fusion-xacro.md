# 0003 Joints from Fusion, xacro macro and YAML joint config

Status: done · Date: 2026-11-06

Supersedes [0002](0002-real-joints.md). The tool is generic: nothing in it
may depend on one robot. rft_robodog is only the first test model.

## Problem

v0.1 attaches every link to the base link with a `fixed` joint
([exporter-pipeline.md](../knowledge/exporter-pipeline.md#known-limitations-as-of-2026-11-06)).
Real robots need moving joints and parent/child chains (an arm's `link3`
hangs on `link2`, not on the base). At the same time, Fusion joints in
received models are often missing, incomplete or wrong, and fixing them in
Fusion can break the design. The user must be able to correct joints in the
ROS package without touching the Fusion model, and those corrections must
survive a re-export.

## Proposal

### 1. Topology from Fusion joints

1. Collect `rootComponent.joints` and `rootComponent.asBuiltJoints`. Skip
   suppressed joints.
2. Map each joint end (`occurrenceOne`, `occurrenceTwo`) to its **top-level**
   occurrence. A joint whose two ends are in the same top-level component is
   internal to that link's mesh and is skipped.
3. A joint to a hidden (not exported) component is skipped with a warning.
4. **Direction: `occurrenceOne` is the child, `occurrenceTwo` is the
   parent.** In Fusion's joint dialog component 1 is the one that moves onto
   component 2, so component 2 is what component 1 is mounted on.
5. **A visible top-level component with no parent is attached to
   `<pkg>_base_link` with a `fixed` joint.** That covers loose components
   (vendor sensors, cameras, lidars) and the root of an "island" (components
   joined to each other but not to `base_link`).
6. Validation, each with the list of joints involved:
   - a component with two or more parents → error
   - a cycle → error (URDF cannot describe closed loops)
   - `base_link` as a child → see [Q16](../open-questions.md#open)
   - several joints between the same pair → see [Q17](../open-questions.md#open)

### 2. Joint types

| Fusion | URDF |
|---|---|
| rigid | `fixed` |
| revolute with limits | `revolute` |
| revolute without limits | `continuous` (URDF `revolute` requires limits) |
| slider | `prismatic` (limits: [Q13](../open-questions.md#open)) |
| cylindrical, pin-slot, planar, ball | `fixed` + warning |
| no joint | `fixed` to `<pkg>_base_link` |

### 3. Joint geometry

The **link frame** of each child is chosen per link (option B, decided
2026-11-06):

| Case | Link frame |
|---|---|
| Component has a Fusion joint to its parent | **Joint frame**: the joint geometry's origin point and axes (`JointGeometry`, on the parent side; names in [Q21](../open-questions.md#open)) |
| No joint (fixed to `<pkg>_base_link`) | Component origin |
| Joint exists but its geometry can't be read | Component origin + warning |
| `frame: origin` in `joints_override.yaml` | Component origin |

- **Joint origin** = link frame expressed in the parent's link frame:
  `pose_in_parent = inv(T_parent_link_world) · T_link_world`.
- **Mesh and inertia** are expressed relative to the link frame. When the
  link frame is the joint frame, the visual origin and the inertial origin
  get the extra offset `inv(T_link_world) · T_component_world`. The mesh
  code already computes a visual pose, so this is one more composition.
- **Axis:** from the Fusion joint motion (rotation axis for revolute, slide
  direction for slider), transformed into the link frame. In the joint frame
  this is usually `0 0 1`. If no axis can be read, the default is `0 0 1`.
- **Warning** only when the link frame is the component origin and a
  revolute axis line does not pass through it (distance above a tolerance).
  Otherwise the part would rotate about the wrong point in ROS.
- **`frame` is applied by the exporter, not by xacro.** A link's frame also
  changes its children's joint origins, and xacro can't compose transforms.
  So the exporter reads `frame:` from `joints_override.yaml` (simple line
  parser for our own flat format, no YAML library) and it takes effect on
  the next export. All other override keys are read by xacro directly.
- **Zero position = pose at export time.** Fusion limits are measured from
  Fusion's joint zero, so the URDF limits are
  `lower = fusion_min − current_value`, `upper = fusion_max − current_value`
  (rad, or cm → m for sliders).
- Joint name: `<child_link>_joint`. Each child has exactly one parent, so
  the name is unique, and it stays stable while the component name stays
  the same. The YAML files use it as the key.

### 4. Output: one xacro macro + two YAML files

```
urdf/<pkg>.urdf.xacro        robot: includes macro, links, one macro call per joint   (regenerated)
urdf/joint.xacro             the joint macro                                            (regenerated)
config/joints.yaml           per joint: parent, child, type, axis, limits from Fusion (regenerated)
config/joints_override.yaml  user corrections, created once, never overwritten
```

The macro takes `name`, `parent`, `child`, `xyz`, `rpy`, `type`
(`fixed` / `revolute` / `continuous` / `prismatic`), `axis` (default
`0 0 1`), `flip` (multiplies the axis by −1 and turns the limits into
`[−upper, −lower]`), `lower`, `upper`, `effort` and `velocity`. For each
field it uses the value from `joints_override.yaml` if present, else from
`joints.yaml`.

- `parent`, `child`, `xyz` and `rpy` come from the model and are **not**
  overridable. Changing the parent by hand would need recomputed origins.
- One extra override key, `frame: joint | origin` (default `joint`), read by
  the exporter on re-export, see [section 3](#3-joint-geometry). It is the
  escape hatch for a badly placed Fusion joint: no fix in Fusion needed.
- `joints_override.yaml` is created on the first export, empty, with a
  commented example.
- On re-export the script lists override keys that no longer match a joint
  (e.g. a renamed component) in the summary. Reading only the top-level keys
  needs no YAML library.
- Effort and velocity: Fusion has none, see [Q14](../open-questions.md#open).

### 5. Package changes

- `package.xml`: add `exec_depend` on `xacro` and `joint_state_publisher_gui`.
- `setup.py`: install `urdf/*.xacro` and `config/*.yaml`.
- `display.launch.py`: run `xacro` on `<pkg>.urdf.xacro`, start
  `joint_state_publisher_gui` so moving joints can be tried in RViz.
- No plain `.urdf` is written. Fusion's Python has no `xacro`, and
  `xacro <pkg>.urdf.xacro > <pkg>.urdf` gives one when needed.

Package folder, name and re-export rules: [0004](0004-package-folder-and-reexport.md).

## Implementation notes (2026-11-06)

Implemented as described; facts are in [joints.md](../knowledge/joints.md).
Deviations and details settled while coding:
- The link frame is the joint geometry on the **child** side
  (`geometryOneTransform`), not the parent side. It moves with the child, so
  it is a valid link frame at the current pose; when the joint is solved
  both sides coincide anyway. Flipped joints use side two.
- Fusion API names checked against the reference. The joint frame comes from
  `geometryOneTransform`/`geometryTwoTransform` (as-built: `transform`), not
  from `JointGeometry`.
- The axis-line warning only runs when the link frame is the component
  origin; with the joint frame the axis passes through the link origin.
- The macro file is `urdf/fusion_joint.xacro`, macro `fusion_joint`.
  In xacro expressions `dict()` is used instead of `{}` (Q19).
- Joints from the Q13–Q18 defaults are implemented as written there.

## Alternatives

- **All joints `fixed` to `base_link`, types set only by hand.** Rejected:
  chains (arms) need the right parent, and a parent can't be changed by hand
  without recomputing every origin.
- **Option A: link frame always the component origin, only the axis from
  the joint.** Rejected on 2026-11-06 for option B: if the Fusion axis does
  not pass through the component origin, the part rotates about the wrong
  point, and every jointed component's origin would have to be placed by
  hand. With B, `frame: origin` still covers a badly placed joint.
- **Direction from a BFS from `base_link`.** Rejected in favour of
  `occurrenceOne` = child: it matches how joints are made in Fusion, and it
  handles islands without guessing a root.
- **One `joints.yaml` merged on re-export.** Rejected: merging needs a YAML
  parser inside Fusion (PyYAML is probably not there, not checked). Two files
  give the same result without merging.
- **Three macros (fixed, revolute, prismatic).** Rejected: with one macro,
  changing a type means changing one word.
- **Naming or attribute convention for joints** (from 0002). Not needed,
  Fusion joints carry the information.

## Open questions

[Q13](../open-questions.md#open) prismatic without limits ·
[Q14](../open-questions.md#open) effort/velocity defaults ·
[Q15](../open-questions.md#open) joint to the root component ·
[Q16](../open-questions.md#open) `base_link` as a child ·
[Q17](../open-questions.md#open) several joints between one pair ·
[Q19](../open-questions.md#open) xacro lookup in `load_yaml` data ·
[Q21](../open-questions.md#open) Fusion API names.
