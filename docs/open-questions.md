# Open questions

When a question is answered, write the answer next to it (or move it to
`decisions.md`) and update the linked proposal.

## Open

| # | Question | Default assumed in proposals | Affects |
|---|---|---|---|
| Q1 | Do the Fusion dialogs (`createFolderDialog`, `messageBox`, progress dialog) behave as the code assumes? | Yes, as in the fake `adsk`. Check on the first real run. | [exporter-pipeline](knowledge/exporter-pipeline.md#steps) |
| Q2 | Does mesh frame detection work on real Fusion STLs (no "Mesh frame check uncertain" warnings)? | Fusion writes local frame, mm (seen 2026-11-06). Whether the summary showed warnings not yet confirmed. | [mesh-export](knowledge/mesh-export.md) |
| Q3 | What is the sign convention of the off-diagonal terms from `getXYZMomentsOfInertia`? | As in fusion2urdf: used as returned, no sign flip. | [inertia](knowledge/inertia.md) |
| Q7 | Is the lidar on rft_robodog mounted upside down (subcomponents have roll 180°)? | Unknown. Check the scan direction in RViz. | [robodog](knowledge/robot-rft-robodog.md) |
| Q9 | Which ROS 2 distribution(s) must the package support? | Any current ROS 2; nothing distro-specific is used. Builds on Jazzy (2026-11-06); other distros not tested. | [ros2-package](knowledge/ros2-package.md) |
| Q10 | Should the REP-103 axis option be a UI prompt or only a setting? | A setting in `ExportSettings`. | [0001](proposals/0001-ros-axis-convention.md) |
| Q12 | What should the `package.xml` maintainer, email and license be? | Placeholders `todo` / `TODO`, edited by hand after export. | [ros2-package](knowledge/ros2-package.md#files) |
| Q13 | What limits does a `prismatic` joint get when the Fusion slider has none (URDF requires limits)? | `lower = upper = 0` plus a warning; the user sets them in `joints_override.yaml`. | [0003](proposals/0003-joints-from-fusion-xacro.md#2-joint-types) |
| Q14 | Default `effort` and `velocity` (Fusion has none)? | `effort: 10`, `velocity: 1.0` (SI units), editable in the override file. | [0003](proposals/0003-joints-from-fusion-xacro.md#4-output-one-xacro-macro--two-yaml-files) |
| Q15 | A joint to the root component itself (not to an occurrence): what is the parent? | Treat it as `<pkg>_base_link`, keep the joint type. Not verified how the API reports it. | [0003](proposals/0003-joints-from-fusion-xacro.md#1-topology-from-fusion-joints) |
| Q16 | `base_link` is `occurrenceOne` (a child) of some joint: error or flip that joint? | Flip that one joint, with a warning. | [0003](proposals/0003-joints-from-fusion-xacro.md#1-topology-from-fusion-joints) |
| Q17 | Several joints between the same pair of components: what wins? | Use the first non-rigid one, warn; all rigid → one `fixed`. | [0003](proposals/0003-joints-from-fusion-xacro.md#1-topology-from-fusion-joints) |
| Q18 | Folder name is not a valid ROS package name (`[a-z][a-z0-9_]*`): refuse, or offer the sanitized name? | Refuse and show the sanitized name as a hint. | [0004](proposals/0004-package-folder-and-reexport.md) |
| Q20 | Can the user create a new folder inside Fusion's `createFolderDialog`? | Yes (OS folder dialog has a New folder button). Not tested on macOS. | [0004](proposals/0004-package-folder-and-reexport.md) |
| Q21 | In which coordinate space does Fusion return joint axis vectors (`rotationAxisVector`, `slideDirectionVector`) and `geometryOneTransform`/`geometryTwoTransform`/`transform`; do both sides coincide after solving? (API names checked 2026-11-06.) | World (root) coordinates; sides coincide. Check on the first real run with a robot that has joints. | [joints](knowledge/joints.md) |
| Q4 | Are the masses on rft_robodog sane, i.e. do all bodies have materials? | **No, as exported 2026-11-06:** base 229 kg, one wheel 0.263 kg vs 2.065 kg, lidar no mass. Fix materials in Fusion. | [inertia](knowledge/inertia.md), [robodog](knowledge/robot-rft-robodog.md#notes-per-component) |
| Q22 | rft_robodog export `ttt1` read no joints. Does the model have joints between the top-level components, and where are they (root component or inside a subcomponent)? | Unknown. Until joints exist in the root component, all links are fixed to the base. | [joints](knowledge/joints.md), [robodog](knowledge/robot-rft-robodog.md) |

## Answered

| # | Question | Answer |
|---|---|---|
| Q6 | Where do real joints come from: Fusion joints, or a naming/attribute convention? | **Fusion joints; no joint → `fixed` to `<pkg>_base_link`** (2026-11-06). See [decisions](decisions.md#2026-11-06--joints-xacro-output-and-package-folder). |
| Q11 | Should xacro replace plain URDF or be written next to it? | **xacro replaces it; no plain `.urdf` written** (2026-11-06). See [decisions](decisions.md#2026-11-06--joints-xacro-output-and-package-folder). |
| Q19 | Does xacro allow `.get()` on data from `xacro.load_yaml`? | **Yes, but inside `${…}` use `dict()`, not `{}`** (2026-11-06, tested with xacro 2.1.1). See [joints.md](knowledge/joints.md#xacro-facts-measured-with-xacro-211-from-pypi-2026-11-06). |
| Q8 | What is the wheel radius of rft_robodog? | **0.080 m (width 0.040 m)**, from the wheel STL bounding box (2026-11-06). See [robodog](knowledge/robot-rft-robodog.md#notes-per-component). |
| Q5 | Do the meshes appear in the right place in RViz? | **Yes** (2026-11-06, rft_robodog export `ttt1`, `ros2 launch` on ROS 2 Jazzy, Linux): the whole model shows with correct poses. See [ros2-package](knowledge/ros2-package.md#measured-on-ros-2-jazzy-2026-11-06-export-ttt1-linux). |
