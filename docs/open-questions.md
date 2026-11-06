# Open questions

When a question is answered, write the answer next to it (or move it to
`decisions.md`) and update the linked proposal.

## Open

| # | Question | Default assumed in proposals | Affects |
|---|---|---|---|
| Q1 | Do the Fusion dialogs (`createFolderDialog`, `inputBox`, Yes/No `messageBox`) behave as the code assumes? | Yes, as in the fake `adsk`. Check on the first real run. | [exporter-pipeline](knowledge/exporter-pipeline.md#steps) |
| Q2 | Does mesh frame detection work on real Fusion STLs (no "Mesh frame check uncertain" warnings)? | Yes. Check on the first real run. | [mesh-export](knowledge/mesh-export.md) |
| Q3 | What is the sign convention of the off-diagonal terms from `getXYZMomentsOfInertia`? | As in fusion2urdf: used as returned, no sign flip. | [inertia](knowledge/inertia.md) |
| Q4 | Are the masses on rft_robodog sane, i.e. do all bodies have materials? | Unknown. Check the summary on the first real run. | [inertia](knowledge/inertia.md), [robodog](knowledge/robot-rft-robodog.md) |
| Q5 | Do the meshes appear in the right place in RViz? | Yes, the offline tests pass. Check on the first real run. | [mesh-export](knowledge/mesh-export.md), [ros2-package](knowledge/ros2-package.md) |
| Q7 | Is the lidar on rft_robodog mounted upside down (subcomponents have roll 180°)? | Unknown. Check the scan direction in RViz. | [robodog](knowledge/robot-rft-robodog.md) |
| Q8 | What is the wheel radius of rft_robodog? | Unknown. Needed for `base_footprint` height and the controller. | [robodog](knowledge/robot-rft-robodog.md), roadmap item 4 |
| Q9 | Which ROS 2 distribution(s) must the package support? | Any current ROS 2; nothing distro-specific is used. Not tested. | [ros2-package](knowledge/ros2-package.md) |
| Q10 | Should the REP-103 axis option be a UI prompt or only a setting? | A setting in `ExportSettings`. | [0001](proposals/0001-ros-axis-convention.md) |
| Q12 | What should the `package.xml` maintainer, email and license be? | Placeholders `todo` / `TODO`, edited by hand after export. | [ros2-package](knowledge/ros2-package.md#files) |
| Q13 | What limits does a `prismatic` joint get when the Fusion slider has none (URDF requires limits)? | `lower = upper = 0` plus a warning; the user sets them in `joints_override.yaml`. | [0003](proposals/0003-joints-from-fusion-xacro.md#2-joint-types) |
| Q14 | Default `effort` and `velocity` (Fusion has none)? | `effort: 10`, `velocity: 1.0` (SI units), editable in the override file. | [0003](proposals/0003-joints-from-fusion-xacro.md#4-output-one-xacro-macro--two-yaml-files) |
| Q15 | A joint to the root component itself (not to an occurrence): what is the parent? | Treat it as `<pkg>_base_link`, keep the joint type. Not verified how the API reports it. | [0003](proposals/0003-joints-from-fusion-xacro.md#1-topology-from-fusion-joints) |
| Q16 | `base_link` is `occurrenceOne` (a child) of some joint: error or flip that joint? | Flip that one joint, with a warning. | [0003](proposals/0003-joints-from-fusion-xacro.md#1-topology-from-fusion-joints) |
| Q17 | Several joints between the same pair of components: what wins? | Use the first non-rigid one, warn; all rigid → one `fixed`. | [0003](proposals/0003-joints-from-fusion-xacro.md#1-topology-from-fusion-joints) |
| Q18 | Folder name is not a valid ROS package name (`[a-z][a-z0-9_]*`): refuse, or offer the sanitized name? | Refuse and show the sanitized name as a hint. | [0004](proposals/0004-package-folder-and-reexport.md) |
| Q19 | Does xacro allow `override.get(name, {}).get(...)` on data from `xacro.load_yaml`? | Yes. Not tested. | [0003](proposals/0003-joints-from-fusion-xacro.md#4-output-one-xacro-macro--two-yaml-files) |
| Q20 | Can the user create a new folder inside Fusion's `createFolderDialog`? | Yes (OS folder dialog has a New folder button). Not tested on macOS. | [0004](proposals/0004-package-folder-and-reexport.md) |
| Q21 | Exact Fusion API names for joints: `occurrenceOne/Two`, `jointMotion.rotationAxisVector`, `slideDirectionVector`, `rotationLimits`/`slideLimits`, `rotationValue`/`slideValue`, `isSuppressed`, `geometryOrOriginOne/Two` (`JointGeometry` origin and axis vectors); in which frame are the vectors given; do both joint geometries coincide after the joint is solved? | Names as listed, vectors in world coordinates, both sides coincide (use the parent side). Not checked against the API docs. | [0003](proposals/0003-joints-from-fusion-xacro.md#3-joint-geometry) |

## Answered

| # | Question | Answer |
|---|---|---|
| Q6 | Where do real joints come from: Fusion joints, or a naming/attribute convention? | **Fusion joints; no joint → `fixed` to `<pkg>_base_link`** (2026-11-06). See [decisions](decisions.md#2026-11-06--joints-xacro-output-and-package-folder). |
| Q11 | Should xacro replace plain URDF or be written next to it? | **xacro replaces it; no plain `.urdf` written** (2026-11-06). See [decisions](decisions.md#2026-11-06--joints-xacro-output-and-package-folder). |
