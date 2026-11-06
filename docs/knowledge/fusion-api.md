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

- `ui.createFolderDialog`, `ui.inputBox` (returns `(value, cancelled)`) and
  the Yes/No `ui.messageBox` behave as the code assumes. Only the fake
  `adsk` has been used so far (Q1).
- The sign of the off-diagonal terms from `getXYZMomentsOfInertia` (Q3).
