# Mesh export (`MeshExporter`, `StlReader`, `MeshFrameDetector`)

Each link gets one STL with the geometry of its top-level component,
subcomponents included. The script does not assume which frame or unit
Fusion writes the STL in. It measures that after the export.

## Export

- `design.exportManager.createSTLExportOptions(occurrence, path)`, binary,
  refinement from `ExportSettings.mesh_refinement` (default medium).
- File: `meshes/<link_name>.stl`. URDF reference:
  `package://<pkg>/meshes/<link_name>.stl`.
- If `execute()` returns false or the file is missing → `ExportError`.

## Frame and unit detection

1. `StlReader.bounding_box()` reads the STL's bounding box. It handles binary
   STL (detected by `84 + n·50 == file size`) and ASCII STL (`vertex` lines).
   An STL without vertices → `ExportError`.
2. Two reference boxes, converted cm → m:
   - world: `occurrence.boundingBox`
   - local: `occurrence.component.boundingBox`
3. `MeshFrameDetector.detect()` tries 6 candidates: {world, local} ×
   {mm `0.001`, cm `0.01`, m `1.0`}. For each it scales the STL box and
   sums the absolute differences of the 6 min/max coordinates, divided by
   the largest edge of the reference box. The smallest relative error wins.
4. Result:
   - mesh `scale` = the chosen unit factor (e.g. `0.001 0.001 0.001` for mm)
   - world frame → visual origin = `inv(T_link_world)`
   - local frame → visual origin = `inv(T_link_world) · T_component_world`
     (identity when the link frame is the component origin)
5. If the best relative error is above `frame_check_tolerance` (0.05), the
   link gets the warning "Mesh frame check uncertain (x% bbox mismatch) –
   verify in RViz."

## Known limitations (as of 2026-11-06)

- STL has no colours. RViz shows a single colour per link.
- Detection compares bounding boxes only. If a part's world and local boxes
  are nearly the same (e.g. a symmetric part rotated about its centre),
  both frames score alike and the choice can be wrong. Not seen yet; not
  tested.
- Not checked against real Fusion STLs yet (Q2 in
  [open-questions.md](../open-questions.md)).
