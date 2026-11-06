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
| Q6 | Where do real joints come from: Fusion joints, or a naming/attribute convention? | Not decided. All joints `fixed` until then. | [0002](proposals/0002-real-joints.md) |
| Q7 | Is the lidar on rft_robodog mounted upside down (subcomponents have roll 180°)? | Unknown. Check the scan direction in RViz. | [robodog](knowledge/robot-rft-robodog.md) |
| Q8 | What is the wheel radius of rft_robodog? | Unknown. Needed for `base_footprint` height and the controller. | [robodog](knowledge/robot-rft-robodog.md), roadmap item 4 |
| Q9 | Which ROS 2 distribution(s) must the package support? | Any current ROS 2; nothing distro-specific is used. Not tested. | [ros2-package](knowledge/ros2-package.md) |
| Q10 | Should the REP-103 axis option be a UI prompt or only a setting? | A setting in `ExportSettings`. | [0001](proposals/0001-ros-axis-convention.md) |
| Q11 | Should xacro replace plain URDF or be written next to it? | Next to it. | roadmap item 7 |
| Q12 | What should the `package.xml` maintainer, email and license be? | Placeholders `todo` / `TODO`, edited by hand after export. | [ros2-package](knowledge/ros2-package.md#files) |

## Answered

| # | Question | Answer |
|---|---|---|
