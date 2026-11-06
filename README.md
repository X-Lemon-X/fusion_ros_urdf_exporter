# fusion_ros_urdf_exporter

Fusion 360 script that exports the open design to a ready-to-build
**ROS 2 description package** (ament_python): xacro with real joints, STL
meshes, inertia, a joint config you can edit, a launch file and an RViz
config. It works for any robot model.

Full documentation: [docs/](docs/README.md).

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
  It becomes the root link `<package>_base_link`.
* Every other **visible top-level component** becomes one link. Hide (light
  bulb off) whatever should not be exported. Subcomponents are part of their
  top-level parent's mesh and inertia.
* **Joints** between top-level components define the tree. In each joint,
  **component 1 is the child** (the part mounted onto component 2).
  Rigid → `fixed`, revolute → `revolute` (with limits) or `continuous`,
  slider → `prismatic`. Other joint types are exported as `fixed`.
* A component **without a joint** is attached to the base link with a fixed
  joint. Good for sensors from vendors: drop them into the model and they
  appear in the right place in TF. Put their component origin where the
  frame should be.
* Assign **materials** to all bodies – masses and inertia come from Fusion's
  physical properties.

## Running

Choose or create a folder in the dialog. **The folder name is the package
name** and must be a valid ROS package name (`my_robot`, not `My-Robot`).

| Folder | What happens |
|---|---|
| empty | the package is exported |
| a previous export of this script | it is regenerated; your `config/joints_override.yaml` and any files you added are kept |
| anything else | the script refuses and writes nothing |

The summary dialog (also in **Text Commands**) lists every link with its
joint, pose, mass and warnings.

## Output

```
<package>/
  package.xml, setup.py, setup.cfg
  resource/<package>
  <package>/__init__.py
  urdf/<package>.urdf.xacro      links + joints (regenerated)
  urdf/fusion_joint.xacro        the joint macro
  config/joints.yaml             joint settings from Fusion (regenerated)
  config/joints_override.yaml    YOUR changes (never overwritten)
  meshes/*.stl
  launch/display.launch.py
  rviz/display.rviz
  .fusion_urdf_export            list of generated files
```

## Changing joints without touching Fusion

Edit `config/joints_override.yaml`. Any key wins over `config/joints.yaml`:

```yaml
wheel_left_front_1_joint:
  type: continuous      # fixed | revolute | continuous | prismatic
  flip: true            # reverse the axis (and the limits)
  axis: [0.0, 0.0, 1.0] # in the child link frame
  lower: -1.0
  upper: 1.0
  effort: 10.0
  velocity: 1.0
  frame: origin         # joint | origin – applied on the next export
```

`frame: origin` uses the component origin as the link frame instead of the
Fusion joint frame. Use it when a joint in Fusion is badly placed.

## Viewing in ROS 2

```bash
cp -r <package> ~/ros2_ws/src/
cd ~/ros2_ws && colcon build --packages-select <package>
source install/setup.bash
ros2 launch <package> display.launch.py
```

The launch file starts `robot_state_publisher`, `joint_state_publisher_gui`
(to move the joints) and RViz.

## Tests

The tests run outside Fusion with a fake `adsk` module and simulated designs:

```bash
python -m unittest discover -s tests -v
```

With `pip install xacro` one more test runs the generated package through
real xacro.

## License

MIT
