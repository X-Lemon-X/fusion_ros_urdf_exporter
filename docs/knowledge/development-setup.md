# Development setup

How to get the code into Fusion, edit it in VS Code, and test it.

## Getting the code

Clone into Fusion's Scripts folder. The folder name must match the script
name (`fusion_ros_urdf_exporter`):

| OS | Scripts folder |
|---|---|
| Windows | `%APPDATA%\Autodesk\Autodesk Fusion 360\API\Scripts\` |
| macOS | `~/Library/Application Support/Autodesk/Autodesk Fusion 360/API/Scripts/` |

```bash
git clone git@github.com:X-Lemon-X/fusion_ros_urdf_exporter.git
```

Or clone anywhere and add it in Fusion: **Utilities → Add-Ins → Scripts and
Add-Ins (Shift+S) → +**.

As of 2026-11-06 `main` has only the initial commit. The v0.1 code is in a
patch that has not been applied yet:

```bash
git am 0001-Add-Fusion-360-URDF-ROS-2-package-exporter.patch
git push
```

## Editing and debugging

- In Fusion's Scripts and Add-Ins dialog select the script → **Edit**.
  Fusion opens VS Code with its own Python interpreter and the `adsk` stubs.
- **Debug** in the same dialog attaches VS Code. Set breakpoints, then run.
- Install the Python extension (`ms-python.python`) if VS Code asks.

## Tests

The tests run **outside Fusion** with any Python 3.8+:

```bash
python -m unittest discover -s tests -v
```

VS Code Testing panel: unittest, start dir `tests`, pattern `test_*.py`.

| File | Content |
|---|---|
| `tests/fake_adsk/adsk/{__init__,core,fusion}.py` | Fake Fusion API: dialogs, enums |
| `tests/fake_design.py` | Box-shaped parts with known geometry and inertia, the robodog poses, a fake STL export manager |
| `tests/test_exporter.py` | 9 tests |

The 9 tests: `test_sanitize`, `test_unique_names`, `test_pose_roundtrip`,
`test_frame_detector` (unit tests); `test_scenarios` (every mesh vertex
lands in the right place, COM and inertia exact for world/local frames in
mm/cm/m), `test_package_files`, `test_overwrite_yes_deletes_old_folder`,
`test_overwrite_no_leaves_folder_untouched`, `test_missing_base_link`
(end-to-end).

## Day-to-day loop

- Edit → **Run** in Fusion → read the summary (also in **Text Commands**:
  File → View → Show Text Commands).
- For logic changes (math, names, URDF), write or extend a test first. That
  loop is much faster than clicking in Fusion.
- Check the result in ROS 2: [ros2-package.md](ros2-package.md#using-it).

## Known limitations (as of 2026-11-06)

- No CI. The tests are run by hand.
