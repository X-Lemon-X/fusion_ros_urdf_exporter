# 0004 Package folder, name and re-export

Status: done · Date: 2026-11-06

## Problem

v0.1 asks for a parent directory and a robot name, creates
`<dir>/<robot_name>/`, and on re-export deletes the whole folder
([exporter-pipeline.md](../knowledge/exporter-pipeline.md#steps)). With
[0003](0003-joints-from-fusion-xacro.md) the package holds a user file
(`config/joints_override.yaml`) that must survive re-export. Asking for a
name separately from the folder is also one step too many.

## Proposal

- **One folder dialog. The chosen folder is the package folder, and its name
  is the package name.** The user creates a new folder in the dialog for a
  new robot.
- The package name sets the `<robot name>`, the ROS package name and the
  root link `<pkg>_base_link` (hardcoded pattern, no xacro prefix parameter).
- The folder name must be a valid ROS package name, `[a-z][a-z0-9_]*`.
  Invalid name: see [Q18](../open-questions.md#open).
- What happens depends on the folder's content:

| Folder content | Action |
|---|---|
| empty | export |
| a previous export of this tool (marker file present) | regenerate the generated files; keep `config/joints_override.yaml` |
| anything else | refuse with a message, write nothing |

- **Marker file** `.fusion_urdf_export`: lists every file the tool wrote.
  On re-export the tool deletes the listed files that it does not write
  again (e.g. the mesh of a removed link). Files the user added to the
  package are not touched.

## Implementation notes (2026-11-06)

Implemented as described in `PackageFolder`; facts in
[exporter-pipeline.md](../knowledge/exporter-pipeline.md#package-folder).
Details: no confirmation dialog before regenerating; an invalid name is
refused with the sanitized name as a hint (Q18 default); a folder emptied by
stale-file removal is deleted; exports from v0.1 (no marker) are foreign.

## Alternatives

- **Parent folder + name prompt (v0.1).** Rejected: one more dialog, and
  the name and the folder can disagree.
- **Delete the whole folder on re-export (v0.1).** Rejected: it would delete
  `joints_override.yaml` and anything the user added.
- **Detect a previous export by `package.xml` only.** Rejected: any ROS
  package has one; the marker is explicit and lists the generated files.

## Open questions

[Q18](../open-questions.md#open) invalid folder name ·
[Q20](../open-questions.md#open) can the Fusion folder dialog create folders.
