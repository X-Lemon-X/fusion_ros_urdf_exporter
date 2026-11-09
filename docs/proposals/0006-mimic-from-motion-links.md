# 0006 Mimic joints from Fusion motion links

Status: done · Date: 2026-11-09

## Problem

Grippers, linkages and geared mechanisms have joints that move together:
when one finger closes, the other closes too. URDF describes this with
`<mimic joint multiplier offset/>` on the following joint. Until now the
exporter wrote no `<mimic>`, and the user could not add one: the macro
`fusion_joint` builds the whole `<joint>` element and read no `mimic` key.

Fusion models the same thing as a **motion link** (Assemble → Motion Link).
Since November 2025 the Fusion API exposes motion links to scripts
(`Component.motionLinks`, `MotionLink`), so the exporter can read them.

## Proposal

Read the motion links of the root component and turn each usable one into
a `<mimic>` on the following joint. Implemented on 2026-11-09.

1. **Joint one leads, joint two follows.** Joint one is the joint selected
   first in the Motion Link dialog. A Fusion link is symmetric, but URDF
   needs a leader. Decided by the project owner.
2. **Multiplier from the link values.** Fusion says "joint one moves
   `valueOne` while joint two moves `valueTwo`". In SI units:
   `multiplier = (valueTwo · u₂) / (valueOne · u₁)`, negated when
   `isReversed`. `u` is 0.01 for a slider (cm → m) and 1 for a revolute
   joint (rad).
3. **Offset 0.** Every URDF joint is 0 at the export pose
   ([joints.md](../knowledge/joints.md#type-axis-and-limits-kinematictree_joint_spec)), and the
   motion link already holds at that pose. So `q₂ = k · q₁` with no offset.
4. **Written to `joints.yaml`** as `mimic`, `multiplier` and `offset` on the
   follower, so every value can be changed or removed in
   `joints_override.yaml` without exporting again (`mimic: null` removes
   it). This follows the YAML design of
   [0003](0003-joints-from-fusion-xacro.md#4-output-one-xacro-macro--two-yaml-files).
5. **`flip` is applied for you.** `multiplier` and `offset` are stated as
   if neither joint had `flip: true`. The macro multiplies the multiplier
   by the flip sign of both joints, and the offset by the follower's. So
   flipping a joint never breaks the coupling.
6. **Chains are collapsed.** If A drives B and B drives C, C mimics A with
   the product of both multipliers. Not every URDF consumer handles a
   mimic of a mimic.
7. **Unusable links are skipped with a line in the summary:**
   - two motions of one joint (`jointTwo` is null, e.g. a cylindrical joint);
   - a joint that is not exported as a URDF joint (hidden component,
     internal, the unused one of duplicate joints);
   - a joint exported as `fixed` (rigid, or an unsupported type);
   - `valueOne` = 0;
   - a second leader for the same follower (the first link wins);
   - a link that would close a loop.

   Suppressed motion links are skipped silently, like suppressed joints.
8. **Old Fusion versions** without `motionLinks` export as before, with no
   mimic joints.

## Alternatives

- **Only a YAML key, no reading from Fusion.** Simpler, but the user would
  type every ratio again although Fusion already knows it. The YAML key
  exists anyway, as the override.
- **The user picks the leader in a dialog.** Extra UI for a rare case.
  Swapping the leader by hand in the override file is possible (see
  [joints.md](../knowledge/joints.md#mimic-joints-fusion-motion-links)).
- **Keep chains as they are.** It is closer to the Fusion model, but it
  breaks consumers that only follow one level of mimic.
- **The user adjusts the multiplier sign after a `flip`.** It is easy to
  forget, and the gripper then closes one finger and opens the other.

## Open questions

- [Q25](../open-questions.md): do the signs match real Fusion
  (`isReversed`, which value belongs to which joint, and the sign of the
  joint value about the axis)?
- [Q26](../open-questions.md): which consumers handle `<mimic>` (Isaac Sim
  URDF importer, ros2_control)?
