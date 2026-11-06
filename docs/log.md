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

**Decisions** → [decisions.md](decisions.md#2026-11-06--project-goal-and-scope):
own Fusion script, ROS 2 `ament_python` package, v0.1 scope with fixed joints.

**Found**
- The v0.1 code is not on `main`. It only exists as
  `0001-Add-Fusion-360-URDF-ROS-2-package-exporter.patch`. During this
  session an untracked copy of `fusion_ros_urdf_exporter.py` appeared in the
  repo root, identical to the patch. The manifest, `tests/`, README and
  `.gitignore` changes from the patch are still missing.

**Next**
- Apply the patch (`git am`) and push.
- Run in real Fusion on rft_robodog; answer Q1–Q5, Q7.
- Measure the wheel radius (Q8).

## 2026-10-27 — v0.1 exporter written

**Done**
- Exporter script, manifest, README and offline tests (9 tests, fake
  `adsk`), delivered as a patch → [exporter-pipeline.md](knowledge/exporter-pipeline.md).

**Decisions** → [decisions.md](decisions.md#2026-10-27--v01-design-rules).
