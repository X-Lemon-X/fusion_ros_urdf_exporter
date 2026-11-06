# Exporter pipeline (`fusion_ros_urdf_exporter.py`)

The exporter is one Fusion 360 **script** (not an add-in that stays loaded).
It runs inside Fusion and turns a prepared design into a ROS 2 description
package: URDF, STL meshes, inertia, a display launch file and an RViz config.
Everything is in `fusion_ros_urdf_exporter.py`, split into classes.

Code state: v0.1, on `main` since 2026-11-06 (with the manifest and
`tests/`). Accepted next steps: [0003](../proposals/0003-joints-from-fusion-xacro.md),
[0004](../proposals/0004-package-folder-and-reexport.md).

## Preparing the model

- The design must have a **visible top-level component named `base_link`**.
  It becomes the root link `<robot_name>_base_link`.
- Every other **visible top-level** component (light bulb on) becomes one
  link. Hide whatever should not be exported.
- Subcomponents (screws, PCBs, …) are not links. They are part of their
  top-level parent's mesh and inertia.
- Each link frame is the **origin of its Fusion component**. Put component
  origins where the joint frames should be.
- Assign materials to all bodies. Mass and inertia come from Fusion's
  physical properties.

## Steps

```
UserPrompt          folder dialog, robot name, overwrite Yes/No
     │
FusionModelReader   root.occurrences (top level only), visible only
                    find "base_link", pose_in_base = inv(T_base) * T_component
     │
for each component:
  InertiaCalculator   getPhysicalProperties → mass, COM, inertia      → inertia.md
  MeshExporter        STL export → bbox → MeshFrameDetector           → mesh-export.md
     │
UrdfBuilder         links (visual + inertial), fixed joints to <robot>_base_link
     │
RosPackageWriter    package files + urdf/<robot>.urdf                 → ros2-package.md
     │
summary dialog + Text Commands log (app.log)
```

1. `UserPrompt.ask_directory()`: folder dialog for the target directory.
2. `UserPrompt.ask_robot_name()`: input box. Default is the sanitized
   document name. An empty name raises `ExportError`.
3. If `<dir>/<robot_name>` exists, a Yes/No box asks: Yes deletes the folder
   (`shutil.rmtree`) and exports again, No stops (`ExportCancelled`).
4. `FusionModelReader.read()` finds `base_link` by **component** name, not
   occurrence name. If more than one exists it uses the first and adds a
   warning. A missing or hidden `base_link` raises `ExportError`.
5. For each component: inertia first, then mesh export. A progress dialog
   (no cancel button) shows the progress.
6. `UrdfBuilder.build()` writes all links, then one `fixed` joint
   `<link>_joint` per non-base link, parent `<robot_name>_base_link`, origin
   = `pose_in_base`.
7. `RosPackageWriter.write()` writes the package files.
8. The summary lists each link with its Fusion name, mesh frame and unit,
   mass, pose and warnings. It goes to a message box and to `app.log`
   (File → View → Show Text Commands).

## Naming

`NameSanitizer.sanitize()`:
- lowercase, `:` → `_`, every character outside `[a-z0-9_]` → `_`
- repeated `_` collapsed, leading and trailing `_` removed
- empty result → `unnamed`; not starting with a letter → `l_` prefix

`UniqueNamer` adds `_2`, `_3`, … on collisions. The base link name is
reserved first. Example: `wheel_left_front:1` → `wheel_left_front_1`.
The same sanitizer is used for the robot name, so the robot name sets the
folder, the ROS package name and the base link name.

## Errors

| Exception | Meaning | What the user sees |
|---|---|---|
| `ExportCancelled` | user cancelled a dialog or chose No | nothing |
| `ExportError` | problem the user can fix in the design | message box with the text |
| anything else | bug | traceback in a message box and in `app.log` |

## Classes

| Class / function | Responsibility |
|---|---|
| `ExportSettings` | All options, see [Settings](#settings) |
| `mat_mul`, `mat_vec`, `transpose`, `clean`, `fmt_pose`, `fmt_num` | Plain 3×3 math and number formatting |
| `Pose` | Rigid transform, see [frames-and-units.md](frames-and-units.md#pose-class) |
| `NameSanitizer`, `UniqueNamer` | ROS-safe, unique link names |
| `InertialInfo`, `ComponentInfo` | Per-component data collected during export |
| `ExportCancelled`, `ExportError` | Cancel vs. user-fixable error |
| `UserPrompt` | All Fusion dialogs |
| `FusionModelReader` | Find base, collect components, compute poses |
| `InertiaCalculator` | Fusion properties → URDF `<inertial>` |
| `StlReader`, `MeshFrameDetector`, `MeshExporter` | STL export and frame/unit detection |
| `UrdfBuilder` | URDF XML (`ElementTree` + `minidom` pretty print) |
| `RosPackageWriter` | Templated package files |
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
| `package_version` | `"0.0.1"` | `package.xml` |
| `maintainer`, `maintainer_email`, `license` | `"todo"`, `"todo@todo.todo"`, `"TODO"` | `package.xml` placeholders |

## Known limitations (as of 2026-11-06)

- All joints are `fixed`, all attached directly to the base link. No chains.
- No collision geometry.
- Axes are kept 1:1 with Fusion. No REP-103 rotation, see
  [proposal 0001](../proposals/0001-ros-axis-convention.md).
- Plain URDF only, no xacro. STL only, so RViz shows no colours.
- Not run in real Fusion yet. Only the [offline tests](development-setup.md#tests)
  have passed.
