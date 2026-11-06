# 0002 Real joints (continuous, revolute, prismatic)

Status: superseded by 0003 · Date: 2026-11-06

Superseded on 2026-11-06 by [0003](0003-joints-from-fusion-xacro.md): joints
are read from Fusion (option 1 below), with a YAML override file.

## Problem

v0.1 writes only `fixed` joints, all attached directly to the base link
([exporter-pipeline.md](../knowledge/exporter-pipeline.md#known-limitations-as-of-2026-11-06)).
A drive needs `continuous` wheel joints, and arms need `revolute` or
`prismatic` joints with axis and limits, and parent/child chains.

## Proposal

Not decided yet. Two sources for joint type, axis, limits and parent:

1. **Read Fusion joints** (`rootComponent.joints` / `asBuiltJoints`). Type,
   axis and limits are already in the model.
2. **Naming or attribute convention**: e.g. component name suffix or a
   Fusion attribute (`occurrence.attributes`) that names the joint type,
   axis and parent.

The joint frame stays the child component origin, as now.

## Alternatives

To be filled in when one of the two is chosen.

## Open questions

- [Q6](../open-questions.md#open): Fusion joints or a convention?
