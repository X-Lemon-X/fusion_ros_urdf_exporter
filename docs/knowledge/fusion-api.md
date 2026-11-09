# Fusion 360 API facts

API behaviour the exporter depends on. Add new API surprises here, with the
conditions they were seen under.

## Units

- **Lengths are always in cm** in the API, whatever the document units are.
- **Inertia is in kg·cm²**. Mass is in kg.
- Conversion constants: [frames-and-units.md](frames-and-units.md#units).

## Occurrences and transforms

- `rootComponent.occurrences` gives the **top level only**.
  `allOccurrences` also includes every nested subcomponent (too much for
  this exporter).
- `occurrence.transform2`: component pose relative to the root, a 4×4
  `Matrix3D` with translation in cm.
- `occurrence.isLightBulbOn`: the visibility toggle in the browser.
- `occurrence.component.name` is the component name; `occurrence.name` adds
  the instance suffix, e.g. `wheel_left_front:1`.
- `occurrence.boundingBox` is in **world** coordinates,
  `occurrence.component.boundingBox` in the component's **own**
  coordinates. The mesh frame detection relies on this, see
  [mesh-export.md](mesh-export.md#frame-and-unit-detection).

## Joints

Checked against the Autodesk API reference on 2026-11-06 (see
[sources.md](../sources.md)); not yet run in real Fusion.

- `Joint`: `occurrenceOne`, `occurrenceTwo`, `jointMotion`, `isSuppressed`,
  `name`, `geometryOneTransform`, `geometryTwoTransform` (joint geometry
  frame as `Matrix3D`, available even when the `JointGeometry` is not),
  `geometryOrOriginOne/Two`, `isFlipped`, `offset`, `angle`.
- `AsBuiltJoint`: `occurrenceOne/Two`, `jointMotion`, `isSuppressed`,
  `transform` (one frame). `geometry` is null for rigid as-built joints.
- `JointTypes`: Rigid 0, Revolute 1, Slider 2, Cylindrical 3, PinSlot 4,
  Planar 5, Ball 6. `jointMotion` is null for inferred joints.
- `RevoluteJointMotion`: `rotationAxis` (X/Y/Z/Custom), `rotationAxisVector`,
  `rotationValue` (rad), `rotationLimits`.
- `SliderJointMotion`: `slideDirection`, `slideDirectionVector`,
  `slideValue` (cm), `slideLimits`.
- `JointLimits`: `isMinimumValueEnabled`, `minimumValue`,
  `isMaximumValueEnabled`, `maximumValue`, in cm or rad.
- `JointGeometry`: `origin`, `primaryAxisVector` (Z), `secondaryAxisVector`
  (X), `thirdAxisVector` (Y).
- **Measured 2026-11-06 (spoon_bot):** `rotationValue` comes back in
  [0, 2π) even when Fusion shows a negative angle (−5.4° → 354.6°), while
  `rotationLimits` are signed (−90° … 180°). `slideValue` is signed
  (−4.205 cm). See [robot-spoon-bot.md](robot-spoon-bot.md#joint-values-and-limits).
- **Not documented:** the coordinate space of the axis vectors and the
  transforms. The exporter assumes world (root) coordinates (Q21).

## Motion links

Checked against the Autodesk API reference on 2026-11-09 (see
[sources.md](../sources.md)); not yet run in real Fusion (Q25).

- New in the **November 2025** Fusion API. Older versions have no
  `motionLinks` attribute.
- `Component.motionLinks` (collection: `count`, `item`, `itemByName`,
  `createInput`, `add`); `Joint.motionLinks` gives the links of one joint.
- `MotionLink`: `name`, `jointOne`, `jointTwo` (`Joint` or `AsBuiltJoint`;
  `jointTwo` is null when both motions come from `jointOne`), `motionOne`,
  `motionTwo` (`JointMotionTypes`), `valueOne`, `valueTwo`
  (`ModelParameter`), `isReversed`, `isSuppressed`, `healthState`,
  `errorOrWarningMessage`, `setMotionData(...)`.
- A rigid joint is never valid in a motion link.
- Values set as a real number are cm or rad. A string can carry units
  (`"60 deg"`). The exporter reads `ModelParameter.value`, which is in the
  same internal units (cm, rad).
- **Not documented:** what `isReversed` reverses, and whether a positive
  value means motion in the joint's positive direction. The exporter
  assumes "joint two moves the other way" and "yes" (Q25).

## STEP imports

After a STEP import, component origins often all sit at the world origin
with the geometry offset inside. Then every pose is 0 and the origins carry
no meaning. Move the origins (or add joint origins) before exporting.

## Script packaging

- A script folder needs `<name>.py` and `<name>.manifest`, and the folder
  name must match the script name, or Fusion does not list it.
- GitHub's Python `.gitignore` template ignores `*.manifest` (a PyInstaller
  rule). The repo's `.gitignore` whitelists
  `!fusion_ros_urdf_exporter.manifest`, otherwise a fresh clone has no
  manifest and Fusion doesn't see the script.
- `adsk.doEvents()` keeps the UI responsive in the export loop.
- `app.log(text)` writes to the Text Commands window.

## Not verified yet (as of 2026-11-06)

- `ui.createFolderDialog` behaves as the code assumes, and lets the user
  create a new folder (Q1, Q20). Only the fake `adsk` has been used so far.
- The coordinate space of joint vectors and transforms (Q21).
- The sign of the off-diagonal terms from `getXYZMomentsOfInertia` (Q3).
- `occurrence.childOccurrences` and their `transform2` in world (root)
  coordinates, used for the mount frame (Q24).
- Motion links: the signs of `isReversed` and of the values, and that
  `valueOne` belongs to `jointOne` (Q25).
