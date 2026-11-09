# Proposals

Numbered design notes for things we have not built yet. Number them in order
and don't reuse a number.

Status values: `draft` (being discussed) · `accepted` (we will do it) ·
`done` (implemented, facts moved to `knowledge/`) · `rejected` · `superseded by NNNN`.

| # | Title | Status |
|---|---|---|
| [0001](0001-ros-axis-convention.md) | ROS axis convention option (REP-103) | draft |
| [0002](0002-real-joints.md) | Real joints (continuous, revolute, prismatic) | superseded by 0003 |
| [0003](0003-joints-from-fusion-xacro.md) | Joints from Fusion, xacro macro and YAML joint config | done |
| [0004](0004-package-folder-and-reexport.md) | Package folder, name and re-export | done |
| [0005](0005-mount-points.md) | Mount points: attach separately exported robots | done |
| [0006](0006-mimic-from-motion-links.md) | Mimic joints from Fusion motion links | done |

## Planned, no proposal yet

Roadmap from the project notes (2026-11-06), rough priority. Item 1 is a
task, not a design. Write a numbered proposal before building any of the
others.

1. Test in real Fusion on rft_robodog and answer Q1–Q5 in
   [open-questions.md](../open-questions.md). First run done 2026-11-06
   (export `ttt1`, see [log.md](../log.md)); joints (Q22), materials (Q4)
   and the ROS 2 build (Q5) still to check.
2. ROS axis convention → [0001](0001-ros-axis-convention.md).
3. Real joints → ~~[0002](0002-real-joints.md)~~ [0003](0003-joints-from-fusion-xacro.md);
   package folder and re-export → [0004](0004-package-folder-and-reexport.md).
4. `base_footprint` link on the ground under `base_link` (wheel radius
   0.080 m, Q8 answered).
5. ~~Mount points: empty links for vendor sensors (OAK, lidar) instead of
   their meshes, so vendor URDFs can be attached.~~ Done 2026-11-06 →
   [0005](0005-mount-points.md).
6. Collision geometry: simple primitives (box or cylinder from the bounding
   box), or the visual mesh as an option.
7. ~~xacro output (macro with a `parent` parameter), next to or instead of
   plain URDF (Q11).~~ Covered by [0003](0003-joints-from-fusion-xacro.md#4-output-one-xacro-macro--two-yaml-files).
8. Coloured meshes: OBJ + MTL, or DAE, so RViz shows colours.
9. GitHub Actions CI that runs the tests.

## Template

```markdown
# NNNN Title

Status: draft · Date: YYYY-MM-DD

## Problem
What we need and why.

## Proposal
What we would build. Concrete enough to start coding.

## Alternatives
Other options and why we didn't pick them.

## Open questions
Links to entries in ../open-questions.md.
```
