# Decisions

Newest first. One entry per decision: what, why, and where it is described.

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
  display launch and RViz config.** xacro, real joints and collision come
  later. Why: fixed joints are enough to test the whole pipeline (poses,
  meshes, inertia) end to end first.
  → [proposals/README.md](proposals/README.md#planned-no-proposal-yet)

## 2026-10-27 — v0.1 design rules

Taken when v0.1 was written (pending patch, see
[development-setup.md](knowledge/development-setup.md#getting-the-code)).

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
