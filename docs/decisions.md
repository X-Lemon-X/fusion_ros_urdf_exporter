# Decisions

Newest first. One entry per decision: what, why, and where it is described.

## 2026-11-06 — Joints, xacro output and package folder

- **The tool is generic, for every robot model.** rft_robodog is only the
  first test model. Why: the script will be used on future robots too, so
  nothing may be specific to one model.
- **Read joints from Fusion; a component without a joint is `fixed` to
  `<pkg>_base_link`** (answers Q6). Why: the joints decide the parent/child
  tree, which chains like arms need. Loose components (vendor sensors) just
  go on the base, so they can be dropped into the model and land in the
  right place in TF.
  → [0003](proposals/0003-joints-from-fusion-xacro.md#1-topology-from-fusion-joints)
- **Joint direction: `occurrenceOne` is the child, `occurrenceTwo` the
  parent. A component with no parent (also the root of an "island") is
  `fixed` to `<pkg>_base_link`.** Why: in Fusion component 1 is the one
  mounted onto component 2. This also settles islands without guessing.
  → [0003](proposals/0003-joints-from-fusion-xacro.md#1-topology-from-fusion-joints)
- **Supported: rigid → `fixed`, revolute → `revolute`/`continuous`, slider
  → `prismatic`. Other joint types → `fixed` + warning. Cycles → error.**
  Why: other types are hard to map to URDF; URDF can't describe loops.
  → [0003](proposals/0003-joints-from-fusion-xacro.md#2-joint-types)
- **Link frame: the Fusion joint frame (point + axes) when the component
  has a joint; the component origin when it has none, when the joint
  geometry can't be read, or when `frame: origin` is set in
  `joints_override.yaml`** (option B). Why: with a joint, its frame is the
  real rotation point, so the part always rotates correctly in ROS without
  placing origins by hand. `frame: origin` is the escape hatch for a badly
  placed Fusion joint, so nobody has to repair the model in Fusion.
  → [0003](proposals/0003-joints-from-fusion-xacro.md#3-joint-geometry)
- ~~**Joint origins come from component origins, not from Fusion joint
  geometry.** Why: Fusion joints in received models are often wrong, and
  fixing them in Fusion can break half the design.~~ Superseded on
  2026-11-06 by the option B rule above.
- **One xacro joint macro with all parameters (`type`, `axis` default
  `0 0 1`, `flip`, limits, effort, velocity).** Why: switching a joint's type
  or direction becomes a one-word change.
  → [0003](proposals/0003-joints-from-fusion-xacro.md#4-output-one-xacro-macro--two-yaml-files)
- **Joint settings in YAML: `config/joints.yaml` (generated, always
  overwritten) + `config/joints_override.yaml` (user, created once, never
  overwritten).** Why: a single generated YAML would lose the user's edits
  on re-export, and merging would need a YAML parser inside Fusion.
  → [0003](proposals/0003-joints-from-fusion-xacro.md#4-output-one-xacro-macro--two-yaml-files)
- **xacro is the output; no plain `.urdf` is written** (answers Q11). Why:
  follows from the macro and YAML design; `xacro` turns it into a URDF when
  needed.
- **Root link name is hardcoded as `<pkg>_base_link`; no xacro prefix
  parameter.** Why: the only sensible single convention.
- **The chosen folder is the package folder, and its name is the package
  and robot name. Empty → export; our previous export → regenerate,
  keeping `joints_override.yaml`; anything else → refuse.** Why: the user
  names the robot once, by creating the folder, and re-export must not
  destroy user edits or unrelated files.
  → [0004](proposals/0004-package-folder-and-reexport.md)
- ~~Robot name from a separate input box; output in `<dir>/<robot_name>/`;
  existing folder deleted on overwrite (v0.1).~~ Superseded on 2026-11-06 by
  the folder rule above.

## 2026-11-06 — Project goal and scope

- **Write our own Fusion 360 script instead of using an existing exporter
  or Onshape.** Why: the fusion2urdf add-in and its forks no longer work with
  current Fusion versions. Onshape can export URDF, but its free plan makes
  documents public, which rules it out for company models. Doing it by hand
  (measuring offsets, cm → m, rotations, one mesh export per part) is slow
  and error-prone.
  → [exporter-pipeline.md](knowledge/exporter-pipeline.md)
- **Target ROS 2 only; output an `ament_python` description package.**
  Why: chosen by the project owner at setup. A ready-to-build package means
  `colcon build` + `ros2 launch` works straight after export.
  → [ros2-package.md](knowledge/ros2-package.md)
- **First version (v0.1): URDF with `fixed` joints, STL meshes, inertia,
  display launch and RViz config.** ~~xacro, real joints and collision come
  later.~~ Superseded on 2026-11-06: xacro and real joints are the next step
  ([0003](proposals/0003-joints-from-fusion-xacro.md)); collision still later. Why: fixed joints are enough to test the whole pipeline (poses,
  meshes, inertia) end to end first.
  → [proposals/README.md](proposals/README.md#planned-no-proposal-yet)

## 2026-10-27 — v0.1 design rules

Taken when v0.1 was written (delivered as a patch, on `main` since 2026-11-06).

- **Only visible top-level components become links; subcomponents stay in
  their parent's mesh and inertia. No hardcoded component list.** Why:
  screws, PCBs and other subparts must not become links, and the user
  controls what is exported with the visibility toggle.
  → [exporter-pipeline.md](knowledge/exporter-pipeline.md#preparing-the-model)
- **The root is the top-level component named `base_link`; it becomes
  `<robot_name>_base_link`.** Why: one fixed, explicit root, and the robot
  name prefix keeps link names unique when several robots share a TF tree.
  → [exporter-pipeline.md](knowledge/exporter-pipeline.md#steps)
- **Link frame = Fusion component origin.** Why: the designer sets joint
  frames in Fusion by placing component origins; no extra markup needed.
  → [frames-and-units.md](knowledge/frames-and-units.md#frames)
- **Keep axes 1:1 with Fusion for now.** Why: simplest correct mapping for
  testing the pipeline; the REP-103 option is planned.
  → [proposal 0001](proposals/0001-ros-axis-convention.md)
- **Detect the STL frame (world/local) and unit (mm/cm/m) from bounding
  boxes after export, instead of assuming them.** Why: the frame and unit
  Fusion writes STLs in were not known for sure; measuring them avoids a
  wrong guess putting every mesh in the wrong place.
  → [mesh-export.md](knowledge/mesh-export.md#frame-and-unit-detection)
- **Inertia uses the fusion2urdf convention for `getXYZMomentsOfInertia`.**
  Why: the only known working reference; not verified yet (Q3).
  → [inertia.md](knowledge/inertia.md#computation)
- **Tests run outside Fusion with a fake `adsk` module and a simulated
  robot.** Why: much faster than clicking through Fusion, and the math
  (poses, frames, inertia) can be checked against exact expected values.
  → [development-setup.md](knowledge/development-setup.md#tests)
- **Whitelist `fusion_ros_urdf_exporter.manifest` in `.gitignore`.** Why:
  the Python template ignores `*.manifest`, and without the manifest Fusion
  does not see the script after a clone.
  → [fusion-api.md](knowledge/fusion-api.md#script-packaging)
