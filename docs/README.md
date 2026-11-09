# Knowledge base

Everything we know and decide about **fusion_ros_urdf_exporter**, in plain
Markdown: a Fusion 360 script that turns a prepared design into a
ready-to-build ROS 2 description package (URDF, meshes, inertia, launch
file, RViz config).
Code lives in the repo root; this folder holds explanations, plans and decisions.

## Layout

| Path | What goes there |
|---|---|
| [knowledge/](knowledge/) | Facts: how the code, tools and hardware work. Should stay true until the code changes. |
| [proposals/](proposals/) | Plans and design options that are not built yet. Each has a status. |
| [log.md](log.md) | Project log: meetings and work sessions, newest first. |
| [decisions.md](decisions.md) | Short log of decisions taken, newest first, with the reason. |
| [open-questions.md](open-questions.md) | Things we still have to decide or find out. |
| [images/](images/) | Screenshots used by the root README guide and the robot pages. |
| [sources.md](sources.md) | Where external information came from (documents processed from `inbox/`, links). |

## Pages

Knowledge
- [Exporter pipeline](knowledge/exporter-pipeline.md): overview, steps, package folder, naming, settings, classes
- [Frames and units](knowledge/frames-and-units.md): the unit and frame reference
- [Mesh export](knowledge/mesh-export.md): STL export and world/local + unit detection
- [Joints and kinematic tree](knowledge/joints.md): Fusion joints → URDF, mimic from motion links, xacro macro, YAML config
- [Inertia](knowledge/inertia.md): mass, COM, tensor
- [Generated ROS 2 package](knowledge/ros2-package.md)
- [Fusion 360 API facts](knowledge/fusion-api.md)
- [Development setup](knowledge/development-setup.md): install, VS Code, tests
- [Target robot: rft_robodog](knowledge/robot-rft-robodog.md)
- [Test robot: sdrac1 (6-axis arm)](knowledge/robot-sdrac1.md)
- [Test robot: spoon_bot (all joint kinds)](knowledge/robot-spoon-bot.md)

Proposals
- [0001 ROS axis convention option (REP-103)](proposals/0001-ros-axis-convention.md)
- ~~[0002 Real joints](proposals/0002-real-joints.md)~~ superseded by 0003
- [0003 Joints from Fusion, xacro macro and YAML joint config](proposals/0003-joints-from-fusion-xacro.md)
- [0004 Package folder, name and re-export](proposals/0004-package-folder-and-reexport.md)
- [0005 Mount points: attach separately exported robots](proposals/0005-mount-points.md)
- [0006 Mimic joints from Fusion motion links](proposals/0006-mimic-from-motion-links.md)
- Roadmap items without a proposal yet: [proposals/README.md](proposals/README.md#planned-no-proposal-yet)

## Conventions

- English only.
- One topic per file. Link between pages with relative links.
- Write facts in `knowledge/`, opinions and plans in `proposals/`.
- When a proposal is accepted, set its status to `accepted`, add a line to
  `decisions.md`, and move the facts it establishes into `knowledge/` once
  the code exists.
- Units are SI (m, rad, kg, kg·m²); Fusion API values are in cm. Frames as
  in [frames-and-units.md](knowledge/frames-and-units.md#frames).
- Dates are YYYY-MM-DD.
- External material goes through [`inbox/`](../inbox/README.md).
