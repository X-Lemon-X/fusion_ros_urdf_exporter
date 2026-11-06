# Project log

Chronological record of meetings and work sessions, newest first.
Each entry: what happened, decisions, and next steps. Details live in the
linked pages; this file is the short version.

---

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
