"""Fusion 360 -> URDF + ROS 2 (ament_python) description package.

How it works
------------
* The design must contain a visible top-level component named ``base_link``.
  It becomes the robot root link ``<robot_name>_base_link``.
* Every other *visible top-level* component becomes one link attached to the
  base with a ``fixed`` joint. Subcomponents are not links of their own; they
  are part of their parent's mesh and inertia.
* Each link frame = origin of its Fusion component. Joint origins are the
  component poses relative to ``base_link``. Axes are kept 1:1 with Fusion.
* Meshes are exported as binary STL (medium refinement). After export the
  script reads each STL's bounding box and compares it with Fusion's bounding
  boxes to detect whether Fusion wrote the mesh in world or local coordinates
  and in mm / cm / m. The visual origin and mesh scale are set accordingly.
* Inertia comes from Fusion's physical properties (assign materials to bodies,
  otherwise masses will be wrong).

Output (<dir>/<robot_name>/ is a ROS 2 ament_python package)::

    <robot_name>/
      package.xml, setup.py, setup.cfg
      resource/<robot_name>
      <robot_name>/__init__.py
      urdf/<robot_name>.urdf
      meshes/*.stl
      launch/display.launch.py
      rviz/display.rviz
"""

import math
import os
import re
import shutil
import struct
import traceback
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple
from xml.dom import minidom

import adsk.core
import adsk.fusion

app = adsk.core.Application.get()
ui = app.userInterface

CM_TO_M = 0.01            # Fusion API lengths are always in cm
KGCM2_TO_KGM2 = 1e-4      # Fusion API inertia is in kg*cm^2

Vec3 = List[float]
Mat3 = List[List[float]]


# =============================================================================
# Settings
# =============================================================================
@dataclass
class ExportSettings:
    base_component_name: str = "base_link"
    only_visible: bool = True
    mesh_refinement: int = adsk.fusion.MeshRefinementSettings.MeshRefinementMedium
    inertia_accuracy: int = adsk.fusion.CalculationAccuracy.MediumCalculationAccuracy
    # Relative bbox mismatch above which the mesh frame/unit detection warns
    frame_check_tolerance: float = 0.05
    # package.xml metadata
    package_version: str = "0.0.1"
    maintainer: str = "todo"
    maintainer_email: str = "todo@todo.todo"
    license: str = "TODO"


# =============================================================================
# Small math helpers
# =============================================================================
def mat_mul(a: Mat3, b: Mat3) -> Mat3:
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def mat_vec(a: Mat3, v: Sequence[float]) -> Vec3:
    return [sum(a[i][k] * v[k] for k in range(3)) for i in range(3)]


def transpose(a: Mat3) -> Mat3:
    return [[a[j][i] for j in range(3)] for i in range(3)]


def clean(v: float, eps: float = 1e-12) -> float:
    return 0.0 if abs(v) < eps else v


def fmt_pose(v: float) -> str:
    return f"{clean(v, 1e-9):.6f}"


def fmt_num(v: float) -> str:
    return f"{clean(v):.9g}"


# =============================================================================
# Pose (rigid transform, meters)
# =============================================================================
class Pose:
    """Rigid transform: p_parent = R * p_child + t (t in meters)."""

    def __init__(self, rotation: Optional[Mat3] = None, translation: Optional[Vec3] = None):
        self.R: Mat3 = rotation if rotation is not None else [[1.0, 0.0, 0.0],
                                                             [0.0, 1.0, 0.0],
                                                             [0.0, 0.0, 1.0]]
        self.t: Vec3 = translation if translation is not None else [0.0, 0.0, 0.0]

    @classmethod
    def from_matrix3d(cls, m) -> "Pose":
        rotation = [[m.getCell(i, j) for j in range(3)] for i in range(3)]
        translation = [m.getCell(i, 3) * CM_TO_M for i in range(3)]
        return cls(rotation, translation)

    def inverse(self) -> "Pose":
        rt = transpose(self.R)
        return Pose(rt, [-x for x in mat_vec(rt, self.t)])

    def __mul__(self, other: "Pose") -> "Pose":
        """Composition: (self * other) applies `other` first, then `self`."""
        rotation = mat_mul(self.R, other.R)
        translation = [a + b for a, b in zip(mat_vec(self.R, other.t), self.t)]
        return Pose(rotation, translation)

    def apply(self, p: Sequence[float]) -> Vec3:
        return [a + b for a, b in zip(mat_vec(self.R, p), self.t)]

    def rpy(self) -> Tuple[float, float, float]:
        """URDF convention: R = Rz(yaw) * Ry(pitch) * Rx(roll)."""
        r = self.R
        pitch = math.atan2(-r[2][0], math.hypot(r[0][0], r[1][0]))
        if abs(math.cos(pitch)) > 1e-9:
            roll = math.atan2(r[2][1], r[2][2])
            yaw = math.atan2(r[1][0], r[0][0])
        else:  # gimbal lock
            roll = math.atan2(-r[1][2], r[1][1])
            yaw = 0.0
        return roll, pitch, yaw

    def xyz_str(self) -> str:
        return " ".join(fmt_pose(v) for v in self.t)

    def rpy_str(self) -> str:
        return " ".join(fmt_pose(v) for v in self.rpy())


# =============================================================================
# Names
# =============================================================================
class NameSanitizer:
    """Lowercase letters, digits and '_' only; must start with a letter."""

    @staticmethod
    def sanitize(raw: str) -> str:
        s = raw.strip().lower().replace(":", "_")
        s = re.sub(r"[^a-z0-9_]", "_", s)
        s = re.sub(r"_+", "_", s).strip("_")
        if not s:
            s = "unnamed"
        if not s[0].isalpha():
            s = "l_" + s
        return s


class UniqueNamer:
    """Hands out sanitized names, adding _2, _3 ... on collisions."""

    def __init__(self):
        self._used = set()

    def reserve(self, name: str) -> None:
        self._used.add(name)

    def make(self, raw: str) -> str:
        base = NameSanitizer.sanitize(raw)
        name, i = base, 2
        while name in self._used:
            name = f"{base}_{i}"
            i += 1
        self._used.add(name)
        return name


# =============================================================================
# Data
# =============================================================================
@dataclass
class InertialInfo:
    mass: float                 # kg
    com: Vec3                   # center of mass in link frame [m]
    inertia: Mat3               # about COM, link axes [kg m^2]
    warning: str = ""


@dataclass
class ComponentInfo:
    occurrence: object
    fusion_name: str
    link_name: str
    world_pose: Pose
    is_base: bool = False
    pose_in_base: Pose = field(default_factory=Pose)
    mesh_file: str = ""
    visual_pose: Pose = field(default_factory=Pose)
    mesh_scale: float = 0.001
    mesh_frame: str = ""        # "world" / "local"
    inertial: Optional[InertialInfo] = None
    warnings: List[str] = field(default_factory=list)


class ExportCancelled(Exception):
    """User cancelled; not an error."""


class ExportError(Exception):
    """Problem the user can fix in the design."""


# =============================================================================
# User interaction
# =============================================================================
class UserPrompt:
    def ask_directory(self) -> str:
        dlg = ui.createFolderDialog()
        dlg.title = "Choose where to create the robot package"
        if dlg.showDialog() != adsk.core.DialogResults.DialogOK:
            raise ExportCancelled()
        return dlg.folder

    def ask_robot_name(self, default: str) -> str:
        value, cancelled = ui.inputBox(
            "Robot name (used for the folder, ROS package and links):",
            "Robot name", default)
        if cancelled:
            raise ExportCancelled()
        if not value.strip():
            raise ExportError("Robot name cannot be empty.")
        return NameSanitizer.sanitize(value)

    def confirm_overwrite(self, path: str) -> bool:
        result = ui.messageBox(
            f"Folder already exists:\n{path}\n\n"
            "Yes = delete it completely and export again\n"
            "No = stop",
            "Folder exists",
            adsk.core.MessageBoxButtonTypes.YesNoButtonType,
            adsk.core.MessageBoxIconTypes.WarningIconType)
        return result == adsk.core.DialogResults.DialogYes

    def show(self, text: str, title: str = "URDF exporter") -> None:
        ui.messageBox(text, title)


# =============================================================================
# Reading the Fusion design
# =============================================================================
class FusionModelReader:
    def __init__(self, design, settings: ExportSettings):
        self.design = design
        self.settings = settings

    def _component_name(self, occ) -> str:
        try:
            return occ.component.name
        except Exception:  # pylint:disable=broad-except
            return occ.name.split(":")[0]

    def read(self, robot_name: str) -> List[ComponentInfo]:
        root = self.design.rootComponent
        occs = list(root.occurrences)
        base_name = self.settings.base_component_name

        bases = [o for o in occs if self._component_name(o) == base_name]
        if not bases:
            raise ExportError(f"No top-level component named '{base_name}' found.")
        base_occ = bases[0]
        if self.settings.only_visible and not base_occ.isLightBulbOn:
            raise ExportError(f"Component '{base_name}' is hidden. Make it visible.")

        if self.settings.only_visible:
            occs = [o for o in occs if o.isLightBulbOn]

        namer = UniqueNamer()
        base_link_name = f"{robot_name}_base_link"
        namer.reserve(base_link_name)

        base_world = Pose.from_matrix3d(base_occ.transform2)
        base_inv = base_world.inverse()

        result: List[ComponentInfo] = []
        for occ in occs:
            is_base = occ is base_occ or occ.entityToken == base_occ.entityToken
            world = Pose.from_matrix3d(occ.transform2)
            info = ComponentInfo(
                occurrence=occ,
                fusion_name=occ.name,
                link_name=base_link_name if is_base else namer.make(occ.name),
                world_pose=world,
                is_base=is_base,
                pose_in_base=Pose() if is_base else base_inv * world,
            )
            if len(bases) > 1 and is_base:
                info.warnings.append(f"{len(bases)} components named '{base_name}', used the first.")
            result.append(info)

        result.sort(key=lambda c: (not c.is_base, c.link_name))
        return result


# =============================================================================
# Inertia
# =============================================================================
class InertiaCalculator:
    """Fusion physical properties -> URDF <inertial> in link frame.

    Assumes getXYZMomentsOfInertia returns inertia tensor elements about the
    world origin (same assumption as the fusion2urdf exporter).
    """

    def __init__(self, settings: ExportSettings):
        self.settings = settings

    def compute(self, info: ComponentInfo) -> Optional[InertialInfo]:
        occ = info.occurrence
        try:
            props = occ.getPhysicalProperties(self.settings.inertia_accuracy)
        except Exception:  # pylint:disable=broad-except
            props = occ.physicalProperties

        mass = props.mass
        if mass is None or mass <= 0:
            info.warnings.append("Mass is 0 (no bodies or no material) - <inertial> skipped.")
            return None

        c = props.centerOfMass
        com_world = [c.x * CM_TO_M, c.y * CM_TO_M, c.z * CM_TO_M]

        moments = props.getXYZMomentsOfInertia()
        xx, yy, zz, xy, yz, xz = [v * KGCM2_TO_KGM2 for v in moments[1:7]]
        i_origin = [[xx, xy, xz],
                    [xy, yy, yz],
                    [xz, yz, zz]]

        inertia_link, com_link = self.to_link_frame(mass, com_world, i_origin, info.world_pose)
        inertial = InertialInfo(mass=mass, com=com_link, inertia=inertia_link)
        inertial.warning = self.validate(inertia_link)
        if inertial.warning:
            info.warnings.append(inertial.warning)
        return inertial

    @staticmethod
    def to_link_frame(mass: float, com_world: Vec3, i_origin: Mat3,
                      link_world: Pose) -> Tuple[Mat3, Vec3]:
        # Parallel axis theorem: tensor about COM, still world axes
        x, y, z = com_world
        d = [[y * y + z * z, -x * y, -x * z],
             [-x * y, x * x + z * z, -y * z],
             [-x * z, -y * z, x * x + y * y]]
        i_com_world = [[i_origin[i][j] - mass * d[i][j] for j in range(3)] for i in range(3)]
        # Rotate into link axes: I_link = R^T I R
        r = link_world.R
        i_link = mat_mul(transpose(r), mat_mul(i_com_world, r))
        com_link = link_world.inverse().apply(com_world)
        return i_link, com_link

    @staticmethod
    def validate(i: Mat3) -> str:
        ixx, iyy, izz = i[0][0], i[1][1], i[2][2]
        if min(ixx, iyy, izz) <= 0:
            return "Inertia has non-positive diagonal - check materials."
        tol = 1e-12
        if ixx + iyy < izz - tol or ixx + izz < iyy - tol or iyy + izz < ixx - tol:
            return "Inertia violates triangle inequality - check materials."
        return ""


# =============================================================================
# Mesh export
# =============================================================================
class StlReader:
    @staticmethod
    def bounding_box(path: str) -> Tuple[Vec3, Vec3]:
        with open(path, "rb") as f:
            data = f.read()
        mins = [math.inf] * 3
        maxs = [-math.inf] * 3

        def update(vx, vy, vz):
            for k, v in enumerate((vx, vy, vz)):
                if v < mins[k]:
                    mins[k] = v
                if v > maxs[k]:
                    maxs[k] = v

        is_binary = False
        if len(data) >= 84:
            n = struct.unpack_from("<I", data, 80)[0]
            is_binary = 84 + n * 50 == len(data)

        if is_binary:
            for tri in struct.iter_unpack("<12fH", data[84:84 + n * 50]):
                update(*tri[3:6])
                update(*tri[6:9])
                update(*tri[9:12])
        else:
            for line in data.decode("ascii", errors="ignore").splitlines():
                parts = line.split()
                if len(parts) == 4 and parts[0] == "vertex":
                    update(*(float(p) for p in parts[1:]))

        if mins[0] == math.inf:
            raise ExportError(f"STL has no geometry: {path}")
        return mins, maxs


class MeshFrameDetector:
    """Decides if an exported STL is in world or local coords and its unit."""

    UNITS = {"mm": 0.001, "cm": 0.01, "m": 1.0}

    @classmethod
    def detect(cls, stl_box: Tuple[Vec3, Vec3],
               world_box: Tuple[Vec3, Vec3],
               local_box: Tuple[Vec3, Vec3]) -> Tuple[str, float, float]:
        """Returns (frame, scale, relative_error)."""
        best = ("world", 0.001, math.inf)
        for frame, ref in (("world", world_box), ("local", local_box)):
            size = max(max(ref[1][k] - ref[0][k] for k in range(3)), 1e-9)
            for scale in cls.UNITS.values():
                err = sum(abs(stl_box[0][k] * scale - ref[0][k]) +
                          abs(stl_box[1][k] * scale - ref[1][k]) for k in range(3))
                rel = err / size
                if rel < best[2] - 1e-9:
                    best = (frame, scale, rel)
        return best


class MeshExporter:
    def __init__(self, design, settings: ExportSettings):
        self.design = design
        self.settings = settings

    @staticmethod
    def _box_m(bbox) -> Tuple[Vec3, Vec3]:
        mn, mx = bbox.minPoint, bbox.maxPoint
        return ([mn.x * CM_TO_M, mn.y * CM_TO_M, mn.z * CM_TO_M],
                [mx.x * CM_TO_M, mx.y * CM_TO_M, mx.z * CM_TO_M])

    def export(self, info: ComponentInfo, mesh_dir: str) -> None:
        file_name = f"{info.link_name}.stl"
        path = os.path.join(mesh_dir, file_name)

        em = self.design.exportManager
        opts = em.createSTLExportOptions(info.occurrence, path)
        opts.isBinaryFormat = True
        opts.meshRefinement = self.settings.mesh_refinement
        if not em.execute(opts) or not os.path.isfile(path):
            raise ExportError(f"STL export failed for {info.fusion_name}")
        info.mesh_file = file_name

        stl_box = StlReader.bounding_box(path)
        world_box = self._box_m(info.occurrence.boundingBox)
        local_box = self._box_m(info.occurrence.component.boundingBox)
        frame, scale, rel_err = MeshFrameDetector.detect(stl_box, world_box, local_box)

        info.mesh_frame = frame
        info.mesh_scale = scale
        # world: mesh points are in Fusion world -> bring them into the link frame
        info.visual_pose = info.world_pose.inverse() if frame == "world" else Pose()

        if rel_err > self.settings.frame_check_tolerance:
            info.warnings.append(
                f"Mesh frame check uncertain ({rel_err:.1%} bbox mismatch) - verify in RViz.")


# =============================================================================
# URDF
# =============================================================================
class UrdfBuilder:
    def __init__(self, package_name: str):
        self.package_name = package_name

    def build(self, robot_name: str, components: List[ComponentInfo]) -> str:
        robot = ET.Element("robot", name=robot_name)
        robot.append(ET.Comment(" Generated by the Fusion 360 URDF exporter "))

        base = next(c for c in components if c.is_base)
        for comp in components:
            self._add_link(robot, comp)
        for comp in components:
            if not comp.is_base:
                self._add_fixed_joint(robot, base, comp)

        raw = ET.tostring(robot, encoding="unicode")
        return minidom.parseString(raw).toprettyxml(indent="  ")

    def _add_link(self, robot: ET.Element, comp: ComponentInfo) -> None:
        link = ET.SubElement(robot, "link", name=comp.link_name)

        if comp.mesh_file:
            visual = ET.SubElement(link, "visual")
            ET.SubElement(visual, "origin",
                          xyz=comp.visual_pose.xyz_str(), rpy=comp.visual_pose.rpy_str())
            geometry = ET.SubElement(visual, "geometry")
            s = fmt_num(comp.mesh_scale)
            ET.SubElement(geometry, "mesh",
                          filename=f"package://{self.package_name}/meshes/{comp.mesh_file}",
                          scale=f"{s} {s} {s}")

        if comp.inertial:
            inertial = ET.SubElement(link, "inertial")
            ET.SubElement(inertial, "origin",
                          xyz=" ".join(fmt_pose(v) for v in comp.inertial.com), rpy="0 0 0")
            ET.SubElement(inertial, "mass", value=fmt_num(comp.inertial.mass))
            i = comp.inertial.inertia
            ET.SubElement(inertial, "inertia",
                          ixx=fmt_num(i[0][0]), ixy=fmt_num(i[0][1]), ixz=fmt_num(i[0][2]),
                          iyy=fmt_num(i[1][1]), iyz=fmt_num(i[1][2]), izz=fmt_num(i[2][2]))

    @staticmethod
    def _add_fixed_joint(robot: ET.Element, base: ComponentInfo, comp: ComponentInfo) -> None:
        joint = ET.SubElement(robot, "joint", name=f"{comp.link_name}_joint", type="fixed")
        ET.SubElement(joint, "parent", link=base.link_name)
        ET.SubElement(joint, "child", link=comp.link_name)
        ET.SubElement(joint, "origin",
                      xyz=comp.pose_in_base.xyz_str(), rpy=comp.pose_in_base.rpy_str())


# =============================================================================
# ROS 2 package files
# =============================================================================
class RosPackageWriter:
    PACKAGE_XML = """<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>@PKG@</name>
  <version>@VERSION@</version>
  <description>@PKG@ robot description exported from Fusion 360</description>
  <maintainer email="@EMAIL@">@MAINTAINER@</maintainer>
  <license>@LICENSE@</license>

  <exec_depend>ament_index_python</exec_depend>
  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>rviz2</exec_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
"""

    SETUP_PY = """import os
from glob import glob

from setuptools import setup

package_name = '@PKG@'

setup(
    name=package_name,
    version='@VERSION@',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.stl')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='@MAINTAINER@',
    maintainer_email='@EMAIL@',
    description='@PKG@ robot description exported from Fusion 360',
    license='@LICENSE@',
)
"""

    SETUP_CFG = """[develop]
script_dir=$base/lib/@PKG@
[install]
install_scripts=$base/lib/@PKG@
"""

    LAUNCH = """import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('@PKG@')
    with open(os.path.join(share, 'urdf', '@PKG@.urdf'), 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(share, 'rviz', 'display.rviz')],
        ),
    ])
"""

    RVIZ = """Panels:
  - Class: rviz_common/Displays
    Name: Displays
Visualization Manager:
  Class: ""
  Displays:
    - Class: rviz_default_plugins/Grid
      Enabled: true
      Name: Grid
    - Class: rviz_default_plugins/RobotModel
      Enabled: true
      Name: RobotModel
      Description Source: Topic
      Description Topic:
        Depth: 5
        Durability Policy: Transient Local
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /robot_description
    - Class: rviz_default_plugins/TF
      Enabled: true
      Name: TF
  Enabled: true
  Global Options:
    Fixed Frame: @BASE@
  Name: root
"""

    def __init__(self, package_dir: str, package_name: str, base_link: str,
                 settings: ExportSettings):
        self.dir = package_dir
        self.pkg = package_name
        self.base_link = base_link
        self.settings = settings

    def _render(self, template: str) -> str:
        return (template
                .replace("@PKG@", self.pkg)
                .replace("@BASE@", self.base_link)
                .replace("@VERSION@", self.settings.package_version)
                .replace("@MAINTAINER@", self.settings.maintainer)
                .replace("@EMAIL@", self.settings.maintainer_email)
                .replace("@LICENSE@", self.settings.license))

    def _write(self, rel_path: str, content: str) -> None:
        path = os.path.join(self.dir, rel_path)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", encoding="utf-8", newline="\n") as f:
            f.write(content)

    def write(self, urdf_text: str) -> None:
        self._write("package.xml", self._render(self.PACKAGE_XML))
        self._write("setup.py", self._render(self.SETUP_PY))
        self._write("setup.cfg", self._render(self.SETUP_CFG))
        self._write(os.path.join("resource", self.pkg), "")
        self._write(os.path.join(self.pkg, "__init__.py"), "")
        self._write(os.path.join("launch", "display.launch.py"), self._render(self.LAUNCH))
        self._write(os.path.join("rviz", "display.rviz"), self._render(self.RVIZ))
        self._write(os.path.join("urdf", f"{self.pkg}.urdf"), urdf_text)


# =============================================================================
# Orchestration
# =============================================================================
class RobotExporter:
    def __init__(self, settings: Optional[ExportSettings] = None):
        self.settings = settings or ExportSettings()
        self.prompt = UserPrompt()

    def run(self) -> None:
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            raise ExportError("Open a Fusion design first.")

        target_dir = self.prompt.ask_directory()
        default_name = NameSanitizer.sanitize(app.activeDocument.name)
        robot_name = self.prompt.ask_robot_name(default_name)

        package_dir = os.path.join(target_dir, robot_name)
        if os.path.exists(package_dir):
            if not self.prompt.confirm_overwrite(package_dir):
                raise ExportCancelled()
            shutil.rmtree(package_dir)

        components = FusionModelReader(design, self.settings).read(robot_name)

        mesh_dir = os.path.join(package_dir, "meshes")
        os.makedirs(mesh_dir, exist_ok=True)

        inertia = InertiaCalculator(self.settings)
        meshes = MeshExporter(design, self.settings)

        progress = ui.createProgressDialog()
        progress.isCancelButtonShown = False
        progress.show("URDF exporter", "Exporting %v of %m", 0, len(components))
        try:
            for i, comp in enumerate(components):
                progress.message = f"Exporting {comp.fusion_name} (%v of %m)"
                progress.progressValue = i
                adsk.doEvents()
                comp.inertial = inertia.compute(comp)
                meshes.export(comp, mesh_dir)
            progress.progressValue = len(components)
        finally:
            progress.hide()

        base_link = next(c.link_name for c in components if c.is_base)
        urdf_text = UrdfBuilder(robot_name).build(robot_name, components)
        RosPackageWriter(package_dir, robot_name, base_link, self.settings).write(urdf_text)

        summary = self._summary(package_dir, robot_name, components)
        app.log(summary)
        self.prompt.show(summary)

    @staticmethod
    def _summary(package_dir: str, robot_name: str, components: List[ComponentInfo]) -> str:
        unit = {0.001: "mm", 0.01: "cm", 1.0: "m"}
        lines = [f"Exported robot '{robot_name}' to:", package_dir, ""]
        for c in components:
            mass = f"{c.inertial.mass:.3f} kg" if c.inertial else "no inertia"
            lines.append(f"{c.link_name}  <-  {c.fusion_name}")
            lines.append(f"    mesh: {c.mesh_frame} frame, {unit.get(c.mesh_scale, c.mesh_scale)}, {mass}")
            if not c.is_base:
                lines.append(f"    xyz=\"{c.pose_in_base.xyz_str()}\" rpy=\"{c.pose_in_base.rpy_str()}\"")
            for w in c.warnings:
                lines.append(f"    WARNING: {w}")
        lines += ["", "Test in ROS 2:",
                  f"  colcon build --packages-select {robot_name}",
                  f"  ros2 launch {robot_name} display.launch.py"]
        return "\n".join(lines)


def run(_context: str):
    """Fusion entry point."""
    try:
        RobotExporter().run()
    except ExportCancelled:
        pass
    except ExportError as e:
        ui.messageBox(str(e), "URDF exporter")
    except:  # pylint:disable=bare-except
        app.log(f"Failed:\n{traceback.format_exc()}")
        ui.messageBox(f"Failed:\n{traceback.format_exc()}", "URDF exporter")
