# Generated ROS 2 package (`RosPackageWriter`)

The export folder `<dir>/<robot_name>/` is an `ament_python` package that
builds with `colcon` and shows the robot in RViz.

## Layout

```
<dir>/<robot_name>/
  package.xml, setup.py, setup.cfg
  resource/<robot_name>          empty ament index marker
  <robot_name>/__init__.py       empty
  urdf/<robot_name>.urdf
  meshes/*.stl
  launch/display.launch.py
  rviz/display.rviz              Fixed Frame = <robot_name>_base_link
```

## Files

- Templates are string constants in `RosPackageWriter`. Placeholders:
  `@PKG@`, `@BASE@`, `@VERSION@`, `@MAINTAINER@`, `@EMAIL@`, `@LICENSE@`.
- Files are written as UTF-8 with `\n` line endings.
- `package.xml`: format 3, `exec_depend` on `ament_index_python`, `launch`,
  `launch_ros`, `robot_state_publisher`, `rviz2`. Maintainer, email and
  license are the placeholders from `ExportSettings` (`todo`, `TODO`).
- `setup.py` installs `urdf/*.urdf`, `meshes/*.stl`, `launch/*.launch.py`
  and `rviz/*.rviz` into `share/<robot_name>/`.
- `display.launch.py` reads the URDF file into `robot_description` and starts
  `robot_state_publisher` and `rviz2 -d display.rviz`. There is no
  `joint_state_publisher` (all joints are fixed, so none is needed yet).
- `display.rviz`: Grid, RobotModel (topic `/robot_description`, transient
  local) and TF displays.

## Using it

```bash
cp -r <robot_name> ~/ros2_ws/src/
cd ~/ros2_ws && colcon build --packages-select <robot_name>
source install/setup.bash
ros2 launch <robot_name> display.launch.py
```

## Known limitations (as of 2026-11-06)

- Not built in a real ROS 2 workspace yet. Only the file contents are
  checked by the offline tests.
- No ROS 2 distribution is pinned or tested (Q9 in
  [open-questions.md](../open-questions.md)).
- Once moving joints exist, the launch file will need
  `joint_state_publisher(_gui)`.
