# 0001 ROS axis convention option (REP-103)

Status: draft · Date: 2026-11-06

## Problem

The exporter keeps axes 1:1 with Fusion
([frames-and-units.md](../knowledge/frames-and-units.md#frames)). ROS
expects X forward, Y left, Z up (REP-103). Navigation, controllers and
vendor sensor URDFs assume that. On rft_robodog forward is −X in Fusion
([robot-rft-robodog.md](../knowledge/robot-rft-robodog.md#fusion-model)),
so the exported robot drives backwards in ROS.

## Proposal

Add an option to rotate the whole output into the ROS convention.

- New setting in `ExportSettings`: the Fusion direction that is "forward"
  and the one that is "up" (for robodog: forward −X, up +Z).
- Build one fixed rotation `R_ros` from them. Apply it to the base link
  frame only: `T_base' = T_base · R_ros`. All `pose_in_base`, visual origins
  and inertias then follow from the existing math, because they are all
  computed relative to the base.
- Child link frames stay at their Fusion component origins (decision
  "link frame = component origin" stays valid). Only the base frame changes.
- Ask in the UI, or keep it a setting for now (Q10).

## Alternatives

- **Re-orient the model in Fusion.** Works without code, but every user has
  to do it by hand, and it moves the company's design files.
- **Add a fixed `base_link` → `base_link_ros` joint.** Simple, but the TF
  tree then has two base frames and tools that expect `base_link` to be
  REP-103 get the wrong one.

## Open questions

- [Q10](../open-questions.md#open): UI prompt or setting?
- Should child link frames also be rotated so their axes follow REP-103
  (e.g. wheel axis along Y)? Probably part of
  [real joints](README.md#planned-no-proposal-yet).
