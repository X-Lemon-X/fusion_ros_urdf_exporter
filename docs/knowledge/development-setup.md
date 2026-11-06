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

Development happens directly on `main`.

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

`test_real_xacro` runs the generated package through real xacro. It is
skipped unless the `xacro` package is importable:

```bash
python -m venv .venv && .venv/Scripts/pip install xacro    # Windows
.venv/Scripts/python -m unittest discover -s tests -v
```

Measured 2026-11-06 on Windows 11: 26 tests pass in about 1 s, on Python
3.12.10 (1 skipped) and in a venv with xacro 2.1.1 (none skipped).

| File | Content |
|---|---|
| `tests/fake_adsk/adsk/{__init__,core,fusion}.py` | Fake Fusion API: dialogs, enums incl. `JointTypes` |
| `tests/fake_design.py` | Box-shaped parts with known geometry and inertia, fake joints and as-built joints, the robodog poses and joints, a fake STL export manager |
| `tests/test_exporter.py` | 26 tests |

| Test class | Covers |
|---|---|
| `UnitTests` | names, `Pose`, frame detector, `fmt_yaml_float`, override parser |
| `RobodogTests` | every mesh vertex, COM and inertia exact for world/local × mm/cm/m × moved base, with and without joints; wheel axis and joint frame; package files; real xacro with `flip` and `type` overrides |
| `JointTreeTests` | arm chain (revolute limits shifted by the current value, prismatic in m, cylindrical → fixed), as-built joint, island, flipped base, joint to root, hidden parent, suppressed joint, two parents and loop errors, missing joint geometry |
| `FolderTests` | missing `base_link`, invalid folder name, foreign folder, cancel, re-export (override and user files kept, stale mesh removed, orphan override reported), `frame: origin`, axis-line warning |

## Day-to-day loop

- Edit → **Run** in Fusion → read the summary (also in **Text Commands**:
  File → View → Show Text Commands).
- For logic changes (math, names, URDF), write or extend a test first. That
  loop is much faster than clicking in Fusion.
- Check the result in ROS 2: [ros2-package.md](ros2-package.md#using-it).

## Known limitations (as of 2026-11-06)

- No CI. The tests are run by hand.
