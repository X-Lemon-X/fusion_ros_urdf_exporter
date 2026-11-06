# fusion_ros_urdf_exporter

Fusion 360 script that exports a robot to a **URDF** inside a ready-to-build
**ROS 2 description package** (ament_python), with STL meshes and inertia.

## Install

Clone this repository into Fusion's scripts folder, so the folder name matches
the script name (`fusion_ros_urdf_exporter`):

| OS | Scripts folder |
|---|---|
| Windows | `%APPDATA%\Autodesk\Autodesk Fusion 360\API\Scripts\` |
| macOS | `~/Library/Application Support/Autodesk/Autodesk Fusion 360/API/Scripts/` |

```bash
git clone https://github.com/X-Lemon-X/fusion_ros_urdf_exporter.git
```

Or, in Fusion: **Utilities → Add-Ins → Scripts and Add-Ins (Shift+S) → +** and
point it to the cloned folder. Then select the script and press **Run**.

## Preparing the model

* The design must contain a **visible top-level component named `base_link`**.
  It becomes the root link `<robot_name>_base_link`.
* Every other **visible top-level component** becomes one link. Hide (lightbulb
  off) whatever should not be exported.
* Subcomponents are not separate links; they are part of their top-level
  parent's mesh and inertia.
* Each link frame is the **origin of its component**. Put component origins
  where the joint frames should be (e.g. wheel origin in the wheel centre).
* Assign **materials** to all bodies – masses and inertia come from Fusion's
  physical properties.

## Running

1. Choose the target directory.
2. Enter the robot name. It is sanitized to `lowercase_with_underscores` and used
   for the folder, the ROS package and the base link name.
3. If `<dir>/<robot_name>` already exists you can delete it and export again,
   or stop.

## Output

```
<robot_name>/
  package.xml, setup.py, setup.cfg
  resource/<robot_name>
  <robot_name>/__init__.py
  urdf/<robot_name>.urdf
  meshes/*.stl
  launch/display.launch.py
  rviz/display.rviz
```

* Link names: component names, lowercase, `:` and other symbols → `_`
  (`wheel_left_front:1` → `wheel_left_front_1`); duplicates get `_2`, `_3`…
* Joints: `fixed`, parent `<robot_name>_base_link`, origin = component pose
  relative to `base_link`. Axes are kept exactly as in Fusion.
* Meshes: binary STL, medium refinement, referenced as
  `package://<robot_name>/meshes/<link>.stl`. After export the script reads each
  STL's bounding box to detect whether Fusion wrote it in world or local
  coordinates and in mm / cm / m, and sets the visual origin and scale to match.
* Inertial: mass, centre of mass and inertia tensor about the centre of mass,
  in link axes.
* No collision geometry yet.

The summary dialog (also in **Text Commands**) lists every link with its pose,
mass and warnings (zero mass, invalid inertia, uncertain mesh frame).

## Viewing in ROS 2

```bash
cp -r <robot_name> ~/ros2_ws/src/
cd ~/ros2_ws && colcon build --packages-select <robot_name>
source install/setup.bash
ros2 launch <robot_name> display.launch.py
```

## Code layout

Everything is in `fusion_ros_urdf_exporter.py`, split into classes:

| Class | Responsibility |
|---|---|
| `ExportSettings` | All options (refinement, visibility filter, package metadata) |
| `Pose` | Rigid transforms, inverse, composition, URDF rpy |
| `NameSanitizer`, `UniqueNamer` | ROS-safe, unique link names |
| `UserPrompt` | Folder / name / overwrite dialogs |
| `FusionModelReader` | Finds `base_link`, collects components and their poses |
| `InertiaCalculator` | Fusion physical properties → URDF `<inertial>` |
| `StlReader`, `MeshFrameDetector`, `MeshExporter` | STL export and frame/unit detection |
| `UrdfBuilder` | URDF XML |
| `RosPackageWriter` | ROS 2 package files |
| `RobotExporter` | Orchestrates the export |

## Tests

The tests run outside Fusion with a fake `adsk` module and a simulated design
(box-shaped parts with known geometry and inertia):

```bash
python -m unittest discover -s tests -v
```

They check that every mesh vertex lands in the right place, that centre of mass
and inertia are exact for world/local mesh frames in mm/cm/m, and cover naming,
the overwrite prompt and the generated package files.

## License

MIT
