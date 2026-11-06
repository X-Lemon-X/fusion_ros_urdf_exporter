# 0005 Mount points: attach separately exported robots

Status: done · Date: 2026-11-06

## Problem

A robot is often built from modules: a mobile base with an arm, a gripper
or a vendor sensor. Each module can be its own Fusion design, with its own
joints.

The exporter only uses top-level components. So when a whole arm design is
inserted into the robot as one component, it would become **one rigid link**:
- its subcomponents are merged into one mesh and one mass;
- its joints are lost without a warning, because `JointReader` reads only
  the root component's joints, and a joint inside one top-level component
  is skipped as internal.

## Proposal

Export each module on its own, and connect the packages in a top-level
xacro with one fixed joint. This was the project owner's design, accepted
on 2026-11-06.

1. **Export the arm design on its own**, into its own package (`sdrac1`).
   Its root link is `sdrac1_base_link`.
2. **In the robot design, the arm is one top-level component** placed where
   it is mounted. In `joints_override.yaml` the user sets `mount: true` on
   its joint (every non-base link has one, even without a Fusion joint).
3. **On the next export the link becomes empty**: no mesh, no inertial. Only
   the frame and the joint to its parent stay.
4. **The mount link frame is the pose of the `base_link` subcomponent**
   inside the inserted design. This is exactly the frame of
   `sdrac1_base_link` in the arm's own export, so the user's joint needs no
   `<origin>`. Without such a subcomponent the component origin is used, with
   a warning. The Fusion joint frame (option B) is not used for a mount,
   because the snap point can be anywhere on the part.
5. **The user attaches the arm with one joint** in their own xacro:

   ```xml
   <xacro:include filename="$(find spoon_bot)/urdf/spoon_bot.urdf.xacro"/>
   <xacro:include filename="$(find sdrac1)/urdf/sdrac1.urdf.xacro"/>
   <joint name="sdrac1_1_mount" type="fixed">
     <parent link="sdrac1_1"/>
     <child link="sdrac1_base_link"/>
   </joint>
   ```

   The summary and a comment in the xacro print this joint. They guess the
   child name from the inserted component's name (`sdrac1` →
   `sdrac1_base_link`).
6. **Several packages must be includable in one xacro.** The YAML
   properties are named `<pkg>_joints_cfg` / `<pkg>_joints_override`, and
   the macro gets them as parameters `cfg` / `override`. Before this change,
   both packages defined `joints_cfg`; it worked only because xacro expands
   macros in document order.

## Implementation notes (2026-11-06)

Implemented as described:
- `OverrideFile.mounts`
- `FusionModelReader._base_sub_pose` (direct child occurrences only)
- `KinematicTree._make_mount` (frame source `base_link` or `origin`)
- mesh and inertia skipped in `RobotExporter.run`
- `mount_snippet`
- the macro parameters `cfg` / `override`

Tests are in `MountTests`; one of them includes a robot package and a
separately exported arm package in one xacro and runs it through real
xacro. Facts are in
[joints.md](../knowledge/joints.md#mount-points-mount-true).

## Alternatives

- **Expand nested assemblies.** Subcomponents of a top-level component with
  inner joints become links, and joints are read from all components.
  Rejected for now: one big URDF instead of modules, a harder naming
  scheme, and unverified API behaviour (assembly-context proxies). The
  modular way also gives the arm its own reusable package.
- **Hide the visual at xacro time** (no re-export). Rejected: the mount
  frame must change too, and that can only happen at export.

## Open questions

[Q24](../open-questions.md#open) does `transform2` of a child occurrence
give its world pose in real Fusion?
