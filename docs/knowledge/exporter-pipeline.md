# Exporter pipeline (`fusion_ros_urdf_exporter.py`)

The exporter is one Fusion 360 **script** (not an add-in that stays loaded).
It runs inside Fusion and turns the open design into a ROS 2 description
package: xacro, STL meshes, inertia, joint config, a display launch file and
an RViz config. Everything is in `fusion_ros_urdf_exporter.py`, split into
classes. It is generic: nothing in it depends on one robot.

Code state: joints from Fusion and the package-folder rules
([0003](../proposals/0003-joints-from-fusion-xacro.md),
[0004](../proposals/0004-package-folder-and-reexport.md)) implemented on
2026-11-06 on top of v0.1.

## Preparing the model

- The design must have a **visible top-level component named `base_link`**.
  It becomes the root link `<pkg>_base_link`.
- Every other **visible top-level** component (light bulb on) becomes one
  link. Hide whatever should not be exported.
- Subcomponents (screws, PCBs, …) are not links. They are part of their
  top-level parent's mesh and inertia.
- Joints between top-level components decide the tree. In each joint,
  **component one is the child** (the part mounted onto component two).
  Components without a joint go on the base link. See [joints.md](joints.md).
- Components without a joint: put the component origin where the link frame
  should be (e.g. a sensor's mounting point).
- Assign materials to all bodies. Mass and inertia come from Fusion's
  physical properties.

## Steps

```
UserPrompt            folder dialog → package folder
PackageFolder.check   name valid? empty / previous export / foreign
OverrideFile          read keys, frame:, mount:, mimic: from config/joints_override.yaml
FusionModelReader     visible top-level occurrences, base_link        (no files written yet)
JointReader           root joints + as-built joints
KinematicTree         parents, link frames, joint types; errors       → joints.md
MotionLinkReader,     motion links → <mimic> on the follower          → joints.md
  MimicResolver
for each link:
  InertiaCalculator   mass, COM, inertia in link frame               → inertia.md
  MeshExporter        STL + frame/unit detection                     → mesh-export.md
XacroBuilder, JointsYamlWriter, RosPackageWriter                     → ros2-package.md
OverrideFile.create_if_missing, PackageFolder.remove_stale, write_marker
summary dialog + Text Commands log (app.log)
```

1. `UserPrompt.ask_package_folder()`: one folder dialog. The chosen folder is
   the package folder; its name is the package and robot name.
2. `PackageFolder.check()`, see [Package folder](#package-folder).
3. Model, joints and tree are read and validated. Any `ExportError` here
   leaves the folder untouched.
4. For each link: inertia, then mesh export. A progress dialog (no cancel
   button) shows the progress.
5. Package files are written. Files from the previous export that were not
   written again are deleted. The marker file is rewritten.
6. The summary lists each link with its Fusion name, mesh frame and unit,
   mass, joint type, parent, joint source, frame source, pose and warnings;
   then the motion links (used or ignored, with the reason), override
   keys without a joint, removed files, and whether the
   override file was created. It goes to a message box and to `app.log`
   (File → View → Show Text Commands).

## Package folder

`PackageFolder` ([0004](../proposals/0004-package-folder-and-reexport.md)):

| Folder | Action |
|---|---|
| name not matching `[a-z][a-z0-9_]*` | `ExportError`, with the sanitized name as a hint |
| empty | export |
| contains `.fusion_urdf_export` | regenerate ("Updated robot …" in the summary) |
| anything else | `ExportError`, nothing written |

- `.fusion_urdf_export` lists every generated file (relative paths, `/`),
  one per line after a `#` comment line. It never lists
  `config/joints_override.yaml`.
- On re-export, listed files that are not written again are deleted, and
  their folder too if it becomes empty. Files the user added are never
  touched.
- There is no confirmation dialog before regenerating.

## Naming

`NameSanitizer.sanitize()`:
- lowercase, `:` → `_`, every character outside `[a-z0-9_]` → `_`
- repeated `_` collapsed, leading and trailing `_` removed
- empty result → `unnamed`; not starting with a letter → `l_` prefix

`UniqueNamer` adds `_2`, `_3`, … on collisions. The base link name is
reserved first. Example: `wheel_left_front:1` → `wheel_left_front_1`.
Joint names are `<link>_joint`.

## Errors

| Exception | Meaning | What the user sees |
|---|---|---|
| `ExportCancelled` | user cancelled the folder dialog | nothing |
| `ExportError` | problem the user can fix (folder, `base_link`, two parents, loop, …) | message box with the text |
| anything else | bug | traceback in a message box and in `app.log` |

## Classes

| Class / function | Responsibility |
|---|---|
| `ExportSettings` | All options, see [Settings](#settings) |
| `mat_mul`, `mat_vec`, `transpose`, `vec_sub`, `dot`, `cross`, `norm`, `normalized` | Plain 3×3 / vector math |
| `clean`, `fmt_pose`, `fmt_num`, `fmt_yaml_float` | Number formatting |
| `Pose` | Rigid transform, see [frames-and-units.md](frames-and-units.md#pose-class) |
| `NameSanitizer`, `UniqueNamer` | ROS-safe, unique link names |
| `InertialInfo`, `FusionJoint`, `FusionMotionLink`, `MimicSpec`, `JointSpec`, `ComponentInfo` | Data collected during export |
| `ExportCancelled`, `ExportError` | Cancel vs. user-fixable error |
| `UserPrompt` | Folder dialog and message box |
| `PackageFolder` | Name check, folder classification, marker, stale files |
| `OverrideFile` | Override template; reads keys, `frame:`, `mount:` and `mimic:` |
| `FusionModelReader` | Find base, collect components and their poses |
| `JointReader`, `KinematicTree` | Fusion joints → tree, see [joints.md](joints.md) |
| `MotionLinkReader`, `MimicResolver` | Fusion motion links → `<mimic>`, see [joints.md](joints.md#mimic-joints-fusion-motion-links) |
| `InertiaCalculator` | Fusion properties → URDF `<inertial>` |
| `StlReader`, `MeshFrameDetector`, `MeshExporter` | STL export and frame/unit detection |
| `JOINT_MACRO`, `XacroBuilder`, `JointsYamlWriter` | xacro and `joints.yaml` text |
| `RosPackageWriter` | Templated package files; records written paths |
| `RobotExporter` | Runs everything, progress dialog, summary |
| `run(_context)` | Fusion entry point and error handling |

## Settings

`ExportSettings` (dataclass, no UI to change it yet):

| Field | Default | Meaning |
|---|---|---|
| `base_component_name` | `"base_link"` | Component used as root |
| `only_visible` | `True` | Skip hidden top-level components |
| `mesh_refinement` | `MeshRefinementMedium` | STL refinement |
| `inertia_accuracy` | `MediumCalculationAccuracy` | Physical properties accuracy |
| `frame_check_tolerance` | `0.05` | Relative bbox mismatch above which mesh detection warns |
| `axis_offset_tolerance` | `1e-4` m | Revolute axis line distance from the link origin above which to warn |
| `default_effort`, `default_velocity` | `10.0`, `1.0` | Written to `joints.yaml` |
| `package_version` | `"0.0.1"` | `package.xml` |
| `maintainer`, `maintainer_email`, `license` | `"todo"`, `"todo@todo.todo"`, `"TODO"` | `package.xml` placeholders |

## Known limitations (as of 2026-11-06)

- No collision geometry.
- Axes are kept 1:1 with Fusion. No REP-103 rotation, see
  [proposal 0001](../proposals/0001-ros-axis-convention.md).
- STL only, so RViz shows no colours.
- Run in real Fusion once (2026-11-06, rft_robodog → `ttt1`, Windows), on
  a model where no joints were read (Q22). Joint reading is only covered by
  the [offline tests](development-setup.md#tests).
- Exports from v0.1 (no marker file) count as foreign folders.
