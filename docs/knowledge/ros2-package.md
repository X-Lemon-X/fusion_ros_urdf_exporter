# Generated ROS 2 package (`RosPackageWriter`)

The chosen folder becomes an `ament_python` package that builds with
`colcon` and shows the robot in RViz with movable joints. Folder rules:
[exporter-pipeline.md](exporter-pipeline.md#package-folder).

## Layout

```
<pkg>/
  package.xml, setup.py, setup.cfg
  resource/<pkg>                 empty ament index marker
  <pkg>/__init__.py              empty
  urdf/<pkg>.urdf.xacro          links + one fusion_joint call per joint
  urdf/fusion_joint.xacro        the joint macro
  config/joints.yaml             joint settings from Fusion (regenerated)
  config/joints_override.yaml    your changes (created once, never overwritten)
  meshes/*.stl
  launch/display.launch.py
  rviz/display.rviz              Fixed Frame = <pkg>_base_link
  .fusion_urdf_export            list of generated files
```

The xacro and YAML files are described in [joints.md](joints.md#generated-files).

## Files

- Templates are string constants in `RosPackageWriter`, plus `JOINT_MACRO`.
  Placeholders: `@PKG@`, `@BASE@`, `@VERSION@`, `@MAINTAINER@`, `@EMAIL@`,
  `@LICENSE@`.
- Files are written as UTF-8 with `\n` line endings.
  `RosPackageWriter.written` collects their relative paths for the marker.
- `package.xml`: format 3, `exec_depend` on `ament_index_python`, `launch`,
  `launch_ros`, `xacro`, `robot_state_publisher`, `joint_state_publisher_gui`,
  `rviz2`. Maintainer, email and license are the placeholders from
  `ExportSettings` (`todo`, `TODO`).
- `setup.py` installs `urdf/*.xacro`, `config/*.yaml`, `meshes/*.stl`,
  `launch/*.launch.py` and `rviz/*.rviz` into `share/<pkg>/`. The relative
  `../config/` path in the xacro file works because `urdf/` and `config/`
  sit next to each other there too.
- `display.launch.py`: `xacro.process_file(<share>/urdf/<pkg>.urdf.xacro)`
  → `robot_description`, then `robot_state_publisher`,
  `joint_state_publisher_gui` and `rviz2 -d display.rviz`.
- `display.rviz`: Grid, RobotModel (topic `/robot_description`, transient
  local) and TF displays.
- No plain `.urdf` is written. Get one with
  `xacro urdf/<pkg>.urdf.xacro > <pkg>.urdf`.

## Using it

```bash
cp -r <pkg> ~/ros2_ws/src/
cd ~/ros2_ws && colcon build --packages-select <pkg>
source install/setup.bash
ros2 launch <pkg> display.launch.py
```

Changes in `config/joints_override.yaml` (except `frame:`) need only a
rebuild (`colcon build`), not a new export from Fusion.

## Known limitations (as of 2026-11-06)

- Built and launched in one real workspace only (below).
- Only ROS 2 Jazzy has been tried; no distribution is pinned (Q9).

## Measured on ROS 2 Jazzy (2026-11-06, export `ttt1`, Linux)

- `colcon build --symlink-install` in `~/it/fftest` (package `ttt1` copied
  from the Windows export): return code 0 in about 1.2 s.
- `xacro install/ttt1/share/ttt1/urdf/ttt1.urdf.xacro` with Jazzy's xacro
  works; `check_urdf` parses the result: root `ttt1_base_link`, 6 children
  (all `fixed`, because the model had no joints, Q22).
- `ros2 launch ttt1 display.launch.py` starts `robot_state_publisher`,
  `joint_state_publisher_gui` and RViz; all meshes show at the correct
  poses (Q5).
- Joints changed in `config/joints_override.yaml` take effect without a new
  export from Fusion (the override is read by xacro at launch).
