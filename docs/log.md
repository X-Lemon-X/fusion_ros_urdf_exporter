# Project log

Chronological record of meetings and work sessions, newest first.
Each entry: what happened, decisions, and next steps. Details live in the
linked pages; this file is the short version.

---

## 2026-11-09 — Mimic tested on simple_bot_v2

**Done**
- The project owner exported spoon_bot v2 (`simple_bot_v2`) with two motion
  links between the wheels and reports that the mimic works. The
  multiplier was negated as expected for the flipped leader `wheel_1_joint`
  → [robot-spoon-bot.md](knowledge/robot-spoon-bot.md#export-simple_bot_v2-mimic-from-motion-links-2026-11-09).

**Next**
- A motion link with Reverse ticked, to close Q25.

## 2026-11-09 — Mimic joints from Fusion motion links (0006)

**Done**
- Checked the Fusion API: motion links are readable since November 2025
  (`Component.motionLinks`) → [fusion-api.md](knowledge/fusion-api.md#motion-links).
- Implemented `MotionLinkReader` and `MimicResolver`:
  - a motion link becomes `mimic` / `multiplier` / `offset` in `joints.yaml`;
  - the macro writes `<mimic>`, with `flip` of both joints applied;
  - chains are collapsed, and unusable links are listed in the summary.

  6 new tests; 37 pass, including real xacro 2.1.1 →
  [0006](proposals/0006-mimic-from-motion-links.md),
  [joints.md](knowledge/joints.md#mimic-joints-fusion-motion-links).
- README: section "Mimic joints (Fusion motion links)".

**Decisions** → [decisions.md](decisions.md#2026-11-09--mimic-joints-from-fusion-motion-links):
`jointOne` leads; values overridable in YAML; flip applied automatically;
chains collapsed.

**Next**
- ~~Export a model with motion links in real Fusion~~ Done on
  `simple_bot_v2`, see the entry above. A link with Reverse ticked is
  still untested (Q25).
- Check mimic support in Isaac Sim and ros2_control (Q26).

## 2026-11-06 — Plain URDF for Isaac Sim

**Done**
- New script `tools/xacro_to_urdf.sh <pkg> [--package] [--collision]`:
  writes `urdf/<pkg>.urdf` with relative mesh paths; `--collision` copies
  each visual mesh as collision → [ros2-package.md](knowledge/ros2-package.md#files).
  README section "Plain URDF (Isaac Sim and other importers)".
- spoon_bot imported in Isaac Sim and stands on the ground.

**Found**
- Isaac Sim imports plain URDF only, no xacro.
- Without collision geometry the robot falls through the ground; the
  exporter writes none.

**Next**
- Proposal for collision geometry in the exporter (roadmap item 6) instead
  of copying visual meshes.

## 2026-11-06 — spoon_bot runs in ROS 2 on Linux

**Done**
- The project owner built and launched the re-exported spoon_bot on ROS 2
  Jazzy (Linux). The joints move as expected in RViz →
  [robot-spoon-bot.md](knowledge/robot-spoon-bot.md#in-ros-2-jazzy-on-linux-2026-11-06).
- Added the RViz and `joint_state_publisher_gui` screenshots to the root
  README, section "Viewing in ROS 2".

## 2026-11-06 — Mount points for modular robots (0005)

**Done**
- Discussed with the project owner what happens to an arm design inserted
  as one component: it would become one rigid link, and its joints would be
  lost without a warning. Decided: export modules on their own and attach
  them through mount links → [decisions](decisions.md#2026-11-06--modular-robots-through-mount-points).
- Implemented `mount: true` in `joints_override.yaml`:
  - an empty link at the inserted design's `base_link`;
  - the joint to paste, shown in the summary.

  xacro properties are now prefixed with the package name. 31 tests pass,
  including one that joins two exported packages in real xacro →
  [0005](proposals/0005-mount-points.md), [joints.md](knowledge/joints.md#mount-points-mount-true).

**Next**
- Real Fusion: insert sdrac1 into spoon_bot, set `mount: true`, export both
  and join them in a xacro (Q24).

## 2026-11-06 — spoon_bot: all joint kinds; limit fix; README guide

**Done**
- Exported spoon_bot from real Fusion: rigid, revolute (with and without
  limits) and slider joints all read; 12 links, 11 joints →
  [robot-spoon-bot.md](knowledge/robot-spoon-bot.md).
- Fixed revolute limits: Fusion returns `rotationValue` in [0, 2π), which
  shifted the limits by −2π. `wrap_angle_into` now moves the value into
  the limit range; new test `test_revolute_value_wrapped_to_limits`
  (27 tests pass) → [joints.md](knowledge/joints.md#type-axis-and-limits-kinematictree_joint_spec).
- Rewrote the root `README.md` as a short guide for preparing a model in
  Fusion, with screenshots in [images/](images/).

**Found**
- The lidar `Mesh frame check uncertain` warning is a false alarm there (Q23).
- Right wheels turn about −Y, left about +Y: flip them in the override.

**Next**
- ~~Re-export spoon_bot with the fix and check the joints in RViz (Q21).~~
  Done, see "spoon_bot runs in ROS 2 on Linux" above. Q21 not yet moved.

## 2026-11-06 — First export with joints (6-axis arm `sdrac1`)

**Done**
- The project owner exported the arm sdrac1 from real Fusion into
  `C:/Users/patdu/Desktop/fftest/sdrac1`. No warnings. All 6 revolute
  joints read, chained `base → axis1 → … → axis6`, all `continuous` (no
  limits in Fusion) → [robot-sdrac1.md](knowledge/robot-sdrac1.md).
- Forward kinematics at zero computed from the xacro: joint origins and
  axes fit a 6R arm; J1 axis points down (flipped Fusion frame).

**Next**
- `ros2 launch sdrac1 display.launch.py`: move each joint with
  `joint_state_publisher_gui` and check that each part turns about its real
  axis (Q21).
- Set joint limits (in Fusion or `joints_override.yaml`, `type: revolute`).

## 2026-11-06 — `ttt1` built on ROS 2 Jazzy; docs check

**Done**
- `colcon build --symlink-install` of `ttt1` on Jazzy (Linux) passes;
  Jazzy's `xacro` + `check_urdf` accept the output (7 links, 6 fixed
  joints) → [ros2-package.md](knowledge/ros2-package.md#measured-on-ros-2-jazzy-2026-11-06-export-ttt1-linux).
- Offline tests on Linux, Python 3.12: 26 pass, 1 skipped (no PyPI xacro).
- Docs check: 0 link/index/status problems. Removed stale "not run / not
  built yet" lines; struck through the superseded v0.1 link frame decision.
- `ros2 launch ttt1 display.launch.py` works; RViz shows the whole model
  with correct poses (answers Q5). The project owner set the joints by hand
  in `config/joints_override.yaml`, because every exported joint was
  `fixed` (Q22).

**Next**
- ~~`ros2 launch ttt1 display.launch.py`; check meshes in RViz (Q5)~~ Done,
  see above. Still open: the lidar scan direction (Q7).
- Find out why no Fusion joints were read (Q22), then re-export and check
  joint axes from Fusion (Q21).

## 2026-11-06 — First run in real Fusion (rft_robodog → `ttt1`)

**Done**
- The project owner ran the exporter in Fusion into
  `C:/Users/patdu/Desktop/fftest/ttt1`. No crash; all package files
  written; the xacro file processes with xacro 2.1.1 (7 links, 6 joints).

**Found**
- Fusion writes STLs in the local frame, in mm → [mesh-export.md](knowledge/mesh-export.md#measured-in-real-fusion-2026-11-06-rft_robodog-export-ttt1).
- No Fusion joints were read; all links fixed to the base (Q22).
- Masses not sane: base 229 kg, one wheel 0.263 kg, lidar no mass (Q4).
- Wheel radius 0.080 m from the STL (answers Q8).
- Meshes total 109 MB; the base mesh alone 81 MB.

**Next**
- Answer Q22 (where are the joints?) and re-export.
- Check the summary in Text Commands for warnings (Q2).
- ~~`colcon build`~~ Done on Jazzy, see the entry above. `ros2 launch` in
  a ROS 2 workspace still to do (Q5, Q9).


## 2026-11-06 — Knowledge base set up

**Done**
- Created the knowledge base in [`docs/`](README.md) and the
  [`inbox/`](../inbox/README.md) for source documents (git-ignored).
- Wrote the knowledge pages from the project notes and checked them against
  the code in the pending v0.1 patch → [README.md](README.md#pages).
- Drafted [0001 ROS axis convention](proposals/0001-ros-axis-convention.md)
  and [0002 Real joints](proposals/0002-real-joints.md). Listed the other
  roadmap items in [proposals/README.md](proposals/README.md#planned-no-proposal-yet).
- Reviewed the target design for the export with the project owner.
  Accepted [0003 Joints from Fusion, xacro + YAML](proposals/0003-joints-from-fusion-xacro.md)
  (supersedes 0002) and [0004 Package folder and re-export](proposals/0004-package-folder-and-reexport.md).
  Added open questions Q13–Q21 for the edge cases.
- Link frame rule changed to option B: Fusion joint frame when a joint
  exists, component origin otherwise; `frame: origin` override as escape
  hatch → [0003](proposals/0003-joints-from-fusion-xacro.md#3-joint-geometry).

**Decisions** → [decisions.md](decisions.md#2026-11-06--project-goal-and-scope):
own Fusion script, ROS 2 `ament_python` package, v0.1 scope with fixed joints.
→ [decisions.md](decisions.md#2026-11-06--joints-xacro-output-and-package-folder):
generic tool; joints read from Fusion (`occurrenceOne` = child), loose
components fixed to `<pkg>_base_link`; one xacro macro; generated
`joints.yaml` + user `joints_override.yaml`; folder name = package name.

**Found**
- ~~The v0.1 code is not on `main`. It only exists as
  `0001-Add-Fusion-360-URDF-ROS-2-package-exporter.patch`. During this
  session an untracked copy of `fusion_ros_urdf_exporter.py` appeared in the
  repo root, identical to the patch. The manifest, `tests/`, README and
  `.gitignore` changes from the patch are still missing.~~ Fixed: the
  owner committed the script and docs as `init`; the rest of the patch
  (manifest, `tests/`, README, `.gitignore`) applied with
  `git apply --exclude=fusion_ros_urdf_exporter.py`. All 9 tests pass.

- Implemented [0004](proposals/0004-package-folder-and-reexport.md) and
  [0003](proposals/0003-joints-from-fusion-xacro.md) in
  `fusion_ros_urdf_exporter.py`; both `done`. New page
  [joints.md](knowledge/joints.md); other knowledge pages updated.
- Tests: 9 → 26, all pass; `test_real_xacro` runs the output through xacro
  2.1.1 → [development-setup.md](knowledge/development-setup.md#tests).

**Found** (implementation)
- xacro ends `${` at the first `}`, so `{}` can't be used in expressions;
  `dict()` works (answers Q19).
- Fusion joints have `geometryOneTransform`/`geometryTwoTransform`: the full
  joint frame as a matrix, simpler than `JointGeometry`. The coordinate
  space is not documented (Q21).
- PyYAML reads `1e-05` as a string, so `joints.yaml` floats are always
  written with a `.`.

**Next**
- ~~Apply the patch (`git am`) and push.~~ Applied on `main` (see Found); not pushed.
- Work continues directly on `main`.
- Run in real Fusion on rft_robodog; answer Q1–Q5, Q7.
- Measure the wheel radius (Q8).
- ~~Check the Fusion joint API names (Q21) and the xacro YAML lookup (Q19)
  before coding 0003.~~ Done: names checked, Q19 answered; Q21 frame space
  still open.
- ~~Implement 0004, then 0003, with offline tests (fake joints in `fake_adsk`).~~
  Done, see Done above.
- Run in real Fusion on a model with joints; check Q21 and the summary
  warnings; `colcon build` + `ros2 launch` in a real ROS 2 workspace (Q9).

## 2026-10-27 — v0.1 exporter written

**Done**
- Exporter script, manifest, README and offline tests (9 tests, fake
  `adsk`), delivered as a patch → [exporter-pipeline.md](knowledge/exporter-pipeline.md).

**Decisions** → [decisions.md](decisions.md#2026-10-27--v01-design-rules).
