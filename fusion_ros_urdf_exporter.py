"""Fusion 360 -> xacro + ROS 2 (ament_python) description package.

How it works
------------
* Choose (or create) an empty folder. Its name is the ROS package name and
  the robot name. The root link is ``<pkg>_base_link``.
* The design must contain a visible top-level component named ``base_link``.
* Every other *visible top-level* component becomes one link. Subcomponents
  are part of their top-level parent's mesh and inertia.
* Joints are read from Fusion (joints and as-built joints of the root
  component). Component one of a joint is the child, component two the
  parent. A component without a joint is attached to ``<pkg>_base_link``
  with a fixed joint. Rigid -> fixed, revolute -> revolute / continuous,
  slider -> prismatic, anything else -> fixed (with a warning).
* Link frame = the Fusion joint frame when the component has a joint,
  otherwise the component origin.
* Meshes are binary STL. Each STL's frame (world/local) and unit (mm/cm/m)
  is detected from its bounding box.
* Inertia comes from Fusion's physical properties (assign materials!).
* Joint settings live in ``config/joints.yaml`` (generated) and
  ``config/joints_override.yaml`` (yours, never overwritten).

Output (<folder> is a ROS 2 ament_python package)::

    <pkg>/
      package.xml, setup.py, setup.cfg
      resource/<pkg>
      <pkg>/__init__.py
      urdf/<pkg>.urdf.xacro
      urdf/fusion_joint.xacro
      config/joints.yaml
      config/joints_override.yaml
      meshes/*.stl
      launch/display.launch.py
      rviz/display.rviz
      .fusion_urdf_export          (list of generated files)
"""

import math
import os
import re
import struct
import traceback
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Set, Tuple
from xml.dom import minidom

import adsk.core
import adsk.fusion

app = adsk.core.Application.get()
ui = app.userInterface

CM_TO_M = 0.01            # Fusion API lengths are always in cm
KGCM2_TO_KGM2 = 1e-4      # Fusion API inertia is in kg*cm^2

MARKER_FILE = ".fusion_urdf_export"
OVERRIDE_FILE = "config/joints_override.yaml"
XACRO_NS = "http://www.ros.org/wiki/xacro"

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
    # Distance [m] of a revolute axis line from the link origin above which we warn
    axis_offset_tolerance: float = 1e-4
    # Joint defaults (Fusion has no effort / velocity)
    default_effort: float = 10.0
    default_velocity: float = 1.0
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


def vec_sub(a: Sequence[float], b: Sequence[float]) -> Vec3:
    return [x - y for x, y in zip(a, b)]


def dot(a: Sequence[float], b: Sequence[float]) -> float:
    return sum(x * y for x, y in zip(a, b))


def cross(a: Sequence[float], b: Sequence[float]) -> Vec3:
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def norm(a: Sequence[float]) -> float:
    return math.sqrt(dot(a, a))


def normalized(a: Sequence[float]) -> Optional[Vec3]:
    n = norm(a)
    return [x / n for x in a] if n > 1e-12 else None


def clean(v: float, eps: float = 1e-12) -> float:
    return 0.0 if abs(v) < eps else v


def fmt_pose(v: float) -> str:
    return f"{clean(v, 1e-9):.6f}"


def fmt_num(v: float) -> str:
    return f"{clean(v):.9g}"


def wrap_angle_into(value: float, lower: float, upper: float) -> float:
    """Shift `value` by whole turns so it lies in [lower, upper], or as close as possible.

    Fusion returns `rotationValue` in [0, 2*pi) but the limits signed, e.g. -50 deg
    comes back as 310 deg with limits -90..90 deg.
    """
    mid = 0.5 * (lower + upper)
    return value - 2.0 * math.pi * round((value - mid) / (2.0 * math.pi))


def fmt_yaml_float(v: float) -> str:
    """Float that every YAML parser reads as a float (always has a '.')."""
    s = f"{clean(v, 1e-9):.9f}".rstrip("0")
    return s + "0" if s.endswith(".") else s


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
PACKAGE_NAME_RE = re.compile(r"[a-z][a-z0-9_]*")


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
class FusionJoint:
    """One Fusion joint or as-built joint, in SI units and world coordinates."""
    name: str
    kind: str                       # "joint" / "as-built joint"
    fusion_type: Optional[int]      # adsk.fusion.JointTypes value, None if unknown
    occ_one: object                 # occurrence (None = root component)
    occ_two: object
    frame_one: Optional[Pose]       # joint geometry on occurrence one's side
    frame_two: Optional[Pose]       # joint geometry on occurrence two's side
    axis_world: Optional[Vec3] = None
    value: float = 0.0              # current joint value [rad] or [m]
    lower: Optional[float] = None   # Fusion limits [rad] or [m], None = disabled
    upper: Optional[float] = None


@dataclass
class JointSpec:
    """URDF joint between a link and its parent."""
    name: str
    joint_type: str = "fixed"       # fixed / revolute / continuous / prismatic
    axis: Vec3 = field(default_factory=lambda: [0.0, 0.0, 1.0])
    lower: float = 0.0
    upper: float = 0.0
    source: str = "none"            # Fusion joint name, or "none"


@dataclass
class ComponentInfo:
    occurrence: object
    fusion_name: str
    link_name: str
    component_pose: Pose            # component origin in world
    is_base: bool = False
    component_name: str = ""        # Fusion component (design) name, e.g. "sdrac1"
    base_sub_pose: Optional[Pose] = None   # world pose of a 'base_link' subcomponent
    is_mount: bool = False          # empty link where another robot is attached
    link_pose: Pose = field(default_factory=Pose)    # link frame in world
    frame_source: str = "origin"    # "origin" / "joint" / "base_link"
    parent: Optional["ComponentInfo"] = None
    joint: Optional[JointSpec] = None
    mesh_file: str = ""
    visual_pose: Pose = field(default_factory=Pose)
    mesh_scale: float = 0.001
    mesh_frame: str = ""            # "world" / "local"
    inertial: Optional[InertialInfo] = None
    warnings: List[str] = field(default_factory=list)

    @property
    def pose_in_parent(self) -> Pose:
        if self.parent is None:
            return Pose()
        return self.parent.link_pose.inverse() * self.link_pose


class ExportCancelled(Exception):
    """User cancelled; not an error."""


class ExportError(Exception):
    """Problem the user can fix in the design or the folder choice."""


# =============================================================================
# User interaction
# =============================================================================
class UserPrompt:
    def ask_package_folder(self) -> str:
        dlg = ui.createFolderDialog()
        dlg.title = "Choose or create the package folder (its name = package name)"
        if dlg.showDialog() != adsk.core.DialogResults.DialogOK:
            raise ExportCancelled()
        return dlg.folder

    def show(self, text: str, title: str = "URDF exporter") -> None:
        ui.messageBox(text, title)


# =============================================================================
# Package folder (name, empty / previous export / foreign, generated files)
# =============================================================================
class PackageFolder:
    def __init__(self, path: str):
        self.path = os.path.normpath(path)
        self.name = os.path.basename(self.path)
        self.previous_files: List[str] = []
        self.is_previous_export = False

    def check(self) -> None:
        if not PACKAGE_NAME_RE.fullmatch(self.name):
            raise ExportError(
                f"Folder name '{self.name}' is not a valid ROS package name.\n"
                "Use lowercase letters, digits and '_', starting with a letter,\n"
                f"e.g. '{NameSanitizer.sanitize(self.name)}'.")
        if not os.path.isdir(self.path):
            raise ExportError(f"Folder does not exist:\n{self.path}")
        entries = os.listdir(self.path)
        if not entries:
            return
        if MARKER_FILE in entries:
            self.is_previous_export = True
            self.previous_files = self._read_marker()
            return
        raise ExportError(
            f"Folder is not empty and was not created by this exporter:\n{self.path}\n\n"
            "Choose an empty folder (create a new one in the dialog) "
            "or the folder of a previous export.")

    def _read_marker(self) -> List[str]:
        with open(os.path.join(self.path, MARKER_FILE), encoding="utf-8") as f:
            return [line.strip() for line in f if line.strip() and not line.startswith("#")]

    def abs(self, rel: str) -> str:
        return os.path.join(self.path, *rel.split("/"))

    def remove_stale(self, written: Set[str]) -> List[str]:
        """Delete files from the previous export that were not written again."""
        removed = []
        for rel in self.previous_files:
            if rel in written or rel == OVERRIDE_FILE:
                continue
            path = self.abs(rel)
            if os.path.isfile(path):
                os.remove(path)
                removed.append(rel)
                folder = os.path.dirname(path)
                if folder != self.path and os.path.isdir(folder) and not os.listdir(folder):
                    os.rmdir(folder)
        return removed

    def write_marker(self, written: Set[str]) -> None:
        with open(os.path.join(self.path, MARKER_FILE), "w", encoding="utf-8", newline="\n") as f:
            f.write("# Files generated by fusion_ros_urdf_exporter. Do not edit.\n")
            for rel in sorted(written):
                f.write(rel + "\n")


# =============================================================================
# joints_override.yaml (only what the exporter itself needs from it)
# =============================================================================
class OverrideFile:
    """Reads the user's override file with a tiny line parser (no YAML lib).

    Only what is applied at export time is read here: the top-level joint
    names (to report orphans), `frame: joint|origin` and `mount: true|false`
    per joint. Everything else in the file is read by xacro.
    """

    TEMPLATE = """# Your joint settings. Created once by fusion_ros_urdf_exporter and NEVER
# overwritten. Any key set here wins over config/joints.yaml.
#
# Keys per joint (all optional):
#   type:     fixed | revolute | continuous | prismatic
#   axis:     [x, y, z]            in the child link frame
#   flip:     true | false         reverses the axis (and the limits)
#   lower:    -1.0                 [rad] or [m]
#   upper:    1.0                  [rad] or [m]
#   effort:   10.0                 [N m] or [N]
#   velocity: 1.0                  [rad/s] or [m/s]
#   frame:    joint | origin       link frame: Fusion joint frame or component
#                                  origin. Applied on the next export from Fusion.
#   mount:    true | false         empty link (no mesh, no mass) where a robot
#                                  exported on its own is attached, e.g. an arm.
#                                  Frame = its 'base_link' subcomponent.
#                                  Applied on the next export from Fusion.
#
# Example:
# wheel_left_1_joint:
#   type: continuous
#   flip: true
"""

    TOP_KEY = re.compile(r"^([A-Za-z0-9_]+)\s*:\s*(#.*)?$")
    FRAME = re.compile(r"^\s+frame\s*:\s*['\"]?(joint|origin)['\"]?\s*(#.*)?$")
    MOUNT = re.compile(r"^\s+mount\s*:\s*(true|false|True|False|yes|no)\s*(#.*)?$")

    def __init__(self, path: str):
        self.path = path
        self.keys: List[str] = []
        self.frames: Dict[str, str] = {}
        self.mounts: Set[str] = set()
        if os.path.isfile(path):
            self._parse()

    def _parse(self) -> None:
        current = None
        with open(self.path, encoding="utf-8") as f:
            for line in f:
                line = line.rstrip("\n")
                if not line.strip() or line.lstrip().startswith("#"):
                    continue
                top = self.TOP_KEY.match(line)
                if top:
                    current = top.group(1)
                    self.keys.append(current)
                    continue
                frame = self.FRAME.match(line)
                if frame and current:
                    self.frames[current] = frame.group(1)
                mount = self.MOUNT.match(line)
                if mount and current:
                    if mount.group(1).lower() in ("true", "yes"):
                        self.mounts.add(current)
                    else:
                        self.mounts.discard(current)

    def create_if_missing(self) -> bool:
        if os.path.exists(self.path):
            return False
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        with open(self.path, "w", encoding="utf-8", newline="\n") as f:
            f.write(self.TEMPLATE)
        return True


# =============================================================================
# Reading the Fusion design
# =============================================================================
def _top_level(occ):
    """Top-level occurrence that contains `occ` (None stays None = root)."""
    while occ is not None and getattr(occ, "assemblyContext", None) is not None:
        occ = occ.assemblyContext
    return occ


class FusionModelReader:
    def __init__(self, design, settings: ExportSettings):
        self.design = design
        self.settings = settings

    @staticmethod
    def _component_name(occ) -> str:
        try:
            return occ.component.name
        except Exception:  # pylint:disable=broad-except
            return occ.name.split(":")[0]

    @classmethod
    def _base_sub_pose(cls, occ, base_name: str) -> Optional[Pose]:
        """World pose of a direct subcomponent named `base_name`, e.g. the
        base_link of a robot design inserted as one component (mount point)."""
        try:
            for child in occ.childOccurrences:
                if cls._component_name(child) == base_name:
                    return Pose.from_matrix3d(child.transform2)
        except Exception:  # pylint:disable=broad-except
            pass
        return None

    def read(self, package_name: str) -> List[ComponentInfo]:
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
        base_link_name = f"{package_name}_base_link"
        namer.reserve(base_link_name)

        result: List[ComponentInfo] = []
        for occ in occs:
            is_base = occ is base_occ or occ.name == base_occ.name
            pose = Pose.from_matrix3d(occ.transform2)
            info = ComponentInfo(
                occurrence=occ,
                fusion_name=occ.name,
                link_name=base_link_name if is_base else namer.make(occ.name),
                component_pose=pose,
                link_pose=pose,
                is_base=is_base,
                component_name=self._component_name(occ),
                base_sub_pose=None if is_base else self._base_sub_pose(occ, base_name),
            )
            if len(bases) > 1 and is_base:
                info.warnings.append(f"{len(bases)} components named '{base_name}', used the first.")
            result.append(info)
        return result


class JointReader:
    """Reads joints and as-built joints of the root component."""

    def __init__(self, design):
        self.design = design

    def read(self) -> List[FusionJoint]:
        root = self.design.rootComponent
        result: List[FusionJoint] = []
        for kind, attr in (("joint", "joints"), ("as-built joint", "asBuiltJoints")):
            collection = getattr(root, attr, None)
            if collection is None:
                continue
            for j in collection:
                if self._safe(lambda: j.isSuppressed, False):
                    continue
                result.append(self._read_one(j, kind))
        return result

    @staticmethod
    def _safe(fn, default=None):
        try:
            return fn()
        except Exception:  # pylint:disable=broad-except
            return default

    @staticmethod
    def _pose(fn) -> Optional[Pose]:
        try:
            m = fn()
            return Pose.from_matrix3d(m) if m is not None else None
        except Exception:  # pylint:disable=broad-except
            return None

    @staticmethod
    def _limits(limits, scale: float) -> Tuple[Optional[float], Optional[float]]:
        if limits is None:
            return None, None
        lo = limits.minimumValue * scale if limits.isMinimumValueEnabled else None
        hi = limits.maximumValue * scale if limits.isMaximumValueEnabled else None
        return lo, hi

    def _read_one(self, j, kind: str) -> FusionJoint:
        types = adsk.fusion.JointTypes
        motion = self._safe(lambda: j.jointMotion)
        fusion_type = self._safe(lambda: motion.jointType) if motion is not None else None
        if kind == "joint":
            frame_one = self._pose(lambda: j.geometryOneTransform)
            frame_two = self._pose(lambda: j.geometryTwoTransform)
        else:
            frame_one = frame_two = self._pose(lambda: j.transform)

        info = FusionJoint(
            name=self._safe(lambda: j.name, "?"),
            kind=kind,
            fusion_type=fusion_type,
            occ_one=self._safe(lambda: j.occurrenceOne),
            occ_two=self._safe(lambda: j.occurrenceTwo),
            frame_one=frame_one,
            frame_two=frame_two,
        )
        if fusion_type == types.RevoluteJointType:
            v = self._safe(lambda: motion.rotationAxisVector)
            info.axis_world = [v.x, v.y, v.z] if v is not None else None
            info.value = self._safe(lambda: motion.rotationValue, 0.0) or 0.0
            info.lower, info.upper = self._limits(self._safe(lambda: motion.rotationLimits), 1.0)
        elif fusion_type == types.SliderJointType:
            v = self._safe(lambda: motion.slideDirectionVector)
            info.axis_world = [v.x, v.y, v.z] if v is not None else None
            info.value = (self._safe(lambda: motion.slideValue, 0.0) or 0.0) * CM_TO_M
            info.lower, info.upper = self._limits(self._safe(lambda: motion.slideLimits), CM_TO_M)
        return info


# =============================================================================
# Kinematic tree
# =============================================================================
class KinematicTree:
    """Turns Fusion joints into a tree rooted at the base link.

    * component one = child, component two = parent
    * no parent -> fixed to the base link
    * link frame = joint frame (or the component origin, see frame_overrides)
    """

    TYPE_NAMES = {0: "rigid", 1: "revolute", 2: "slider", 3: "cylindrical",
                  4: "pin-slot", 5: "planar", 6: "ball"}

    def __init__(self, settings: ExportSettings):
        self.settings = settings

    def build(self, components: List[ComponentInfo], joints: List[FusionJoint],
              frame_overrides: Dict[str, str],
              mounts: Optional[Set[str]] = None) -> List[ComponentInfo]:
        mounts = mounts or set()
        base = next(c for c in components if c.is_base)
        by_name = {c.fusion_name: c for c in components}

        # child -> list of (parent, joint, child-side frame)
        edges: Dict[str, List[Tuple[ComponentInfo, FusionJoint, Optional[Pose]]]] = {}
        for j in joints:
            edge = self._edge(j, by_name, base)
            if edge:
                child, parent, frame = edge
                edges.setdefault(child.fusion_name, []).append((parent, j, frame))

        for comp in components:
            if comp.is_base:
                continue
            joint_name = f"{comp.link_name}_joint"
            candidates = edges.get(comp.fusion_name, [])
            if not candidates:
                comp.parent = base
                comp.joint = JointSpec(name=joint_name)
                if joint_name in mounts:
                    self._make_mount(comp)
                continue
            parent, j, frame = self._pick(comp, candidates)
            comp.parent = parent
            if joint_name in mounts:
                self._make_mount(comp)
            elif frame_overrides.get(joint_name, "joint") == "joint" and frame is not None:
                comp.link_pose = frame
                comp.frame_source = "joint"
            elif frame is None:
                comp.warnings.append(f"Geometry of joint '{j.name}' not readable - "
                                     "link frame = component origin.")
            comp.joint = self._joint_spec(comp, j)

        self._check_cycles(components, base)
        return self._ordered(components, base)

    def _make_mount(self, comp: ComponentInfo) -> None:
        """Empty link whose frame matches the root link of the robot exported on its own."""
        comp.is_mount = True
        if comp.base_sub_pose is not None:
            comp.link_pose = comp.base_sub_pose
            comp.frame_source = "base_link"
        else:
            comp.link_pose = comp.component_pose
            comp.frame_source = "origin"
            comp.warnings.append(
                f"Mount has no '{self.settings.base_component_name}' subcomponent - "
                "link frame = component origin.")

    def _edge(self, j: FusionJoint, by_name: Dict[str, ComponentInfo], base: ComponentInfo):
        ends = []
        for occ in (j.occ_one, j.occ_two):
            top = _top_level(occ)
            if top is None:
                ends.append(base)              # root component -> base link
                continue
            comp = by_name.get(top.name)
            if comp is None:                   # hidden component
                ends.append(None)
                continue
            ends.append(comp)
        one, two = ends
        if one is None or two is None:
            visible = one or two
            if visible is not None and not visible.is_base:
                visible.warnings.append(f"Joint '{j.name}' goes to a hidden component - ignored.")
            return None
        if one is two:
            return None                        # internal to one link
        if one.is_base:
            two.warnings.append(f"Joint '{j.name}' has base_link as component one - "
                                "direction flipped.")
            return two, one, j.frame_two
        return one, two, j.frame_one

    def _pick(self, comp: ComponentInfo, candidates):
        parents = {p.fusion_name for p, _, _ in candidates}
        if len(parents) > 1:
            listing = ", ".join(f"'{j.name}' -> {p.fusion_name}" for p, j, _ in candidates)
            raise ExportError(
                f"Component '{comp.fusion_name}' has more than one parent: {listing}.\n"
                "In each joint, component one is the child. Remove or suppress "
                "the extra joints, or hide a component.")
        if len(candidates) > 1:
            moving = [c for c in candidates if c[1].fusion_type != adsk.fusion.JointTypes.RigidJointType]
            chosen = moving[0] if moving else candidates[0]
            comp.warnings.append(
                f"{len(candidates)} joints to the same parent - used '{chosen[1].name}'.")
            return chosen
        return candidates[0]

    def _joint_spec(self, comp: ComponentInfo, j: FusionJoint) -> JointSpec:
        types = adsk.fusion.JointTypes
        spec = JointSpec(name=f"{comp.link_name}_joint", source=j.name)
        if j.fusion_type == types.RigidJointType:
            return spec
        if j.fusion_type not in (types.RevoluteJointType, types.SliderJointType):
            kind = self.TYPE_NAMES.get(j.fusion_type, "unknown")
            comp.warnings.append(f"Joint '{j.name}' is {kind} - exported as fixed.")
            return spec

        axis = normalized(j.axis_world) if j.axis_world else None
        if axis is None:
            comp.warnings.append(f"Axis of joint '{j.name}' not readable - using Z.")
            spec.axis = [0.0, 0.0, 1.0]
        else:
            spec.axis = [clean(v, 1e-9) for v in mat_vec(transpose(comp.link_pose.R), axis)]

        lo, up = j.lower, j.upper
        if j.fusion_type == types.RevoluteJointType:
            if lo is not None and up is not None:
                spec.joint_type = "revolute"
                value = wrap_angle_into(j.value, lo, up)
                if not lo - 1e-9 <= value <= up + 1e-9:
                    comp.warnings.append(f"Joint '{j.name}': current angle is outside its "
                                         "limits - check lower/upper.")
                spec.lower, spec.upper = lo - value, up - value
            else:
                spec.joint_type = "continuous"
                if j.lower is not None or j.upper is not None:
                    comp.warnings.append(f"Joint '{j.name}' has only one limit - exported "
                                         "as continuous.")
            if comp.frame_source == "origin" and axis is not None:
                self._check_axis_line(comp, j, axis)
        else:
            spec.joint_type = "prismatic"
            if lo is not None and up is not None:
                spec.lower, spec.upper = lo - j.value, up - j.value
            else:
                comp.warnings.append(f"Slider '{j.name}' has no limits - set lower/upper "
                                     "in joints_override.yaml.")
        return spec

    def _check_axis_line(self, comp: ComponentInfo, j: FusionJoint, axis: Vec3) -> None:
        frame = j.frame_two or j.frame_one
        if frame is None:
            return
        d = norm(cross(vec_sub(comp.link_pose.t, frame.t), axis))
        if d > self.settings.axis_offset_tolerance:
            comp.warnings.append(
                f"Axis of joint '{j.name}' is {d * 1000:.1f} mm from the component origin - "
                "the part will rotate about the wrong point (use frame: joint).")

    @staticmethod
    def _check_cycles(components: List[ComponentInfo], base: ComponentInfo) -> None:
        for comp in components:
            seen = []
            node = comp
            while node is not None and not node.is_base:
                if node in seen:
                    loop = seen[seen.index(node):]
                    names = " -> ".join(f"{c.fusion_name} ('{c.joint.source}')" for c in loop)
                    raise ExportError(f"Joints form a closed loop: {names}.\n"
                                      "URDF needs a tree. Suppress one joint of the loop.")
                seen.append(node)
                node = node.parent

    @staticmethod
    def _ordered(components: List[ComponentInfo], base: ComponentInfo) -> List[ComponentInfo]:
        """Base first, then parents before children, siblings by link name."""
        children: Dict[int, List[ComponentInfo]] = {}
        for c in components:
            if c.parent is not None:
                children.setdefault(id(c.parent), []).append(c)
        ordered, queue = [], [base]
        while queue:
            node = queue.pop(0)
            ordered.append(node)
            queue.extend(sorted(children.get(id(node), []), key=lambda c: c.link_name))
        return ordered


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

        inertia_link, com_link = self.to_link_frame(mass, com_world, i_origin, info.link_pose)
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

    def export(self, info: ComponentInfo, mesh_dir: str) -> str:
        """Exports the STL; returns its file name."""
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
        # Bring mesh points into the link frame
        to_link = info.link_pose.inverse()
        info.visual_pose = to_link if frame == "world" else to_link * info.component_pose

        if rel_err > self.settings.frame_check_tolerance:
            info.warnings.append(
                f"Mesh frame check uncertain ({rel_err:.1%} bbox mismatch) - verify in RViz.")
        return file_name


# =============================================================================
# xacro + joints.yaml
# =============================================================================
JOINT_MACRO = """<?xml version="1.0"?>
<!-- Generated by fusion_ros_urdf_exporter. Overwritten on every export. -->
<!-- One macro for every joint. Values come from config/joints.yaml and can be
     changed per joint in config/joints_override.yaml.
     cfg / override are the loaded YAML files, passed in so that several
     exported packages can be included in one xacro file.
     Note: inside ${} use dict() instead of {} (xacro ends ${ at the first }). -->
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="fusion_joint" params="name parent child xyz rpy cfg override">
    <xacro:property name="jd" value="${cfg[name]}"/>
    <xacro:property name="jo" value="${(override or dict()).get(name) or dict()}"/>
    <xacro:property name="jtype" value="${jo.get('type', jd['type'])}"/>
    <xacro:property name="jaxis" value="${jo.get('axis', jd['axis'])}"/>
    <xacro:property name="js" value="${-1 if jo.get('flip', jd['flip']) else 1}"/>
    <xacro:property name="jlo" value="${float(jo.get('lower', jd['lower']))}"/>
    <xacro:property name="jup" value="${float(jo.get('upper', jd['upper']))}"/>
    <xacro:property name="jeffort" value="${jo.get('effort', jd['effort'])}"/>
    <xacro:property name="jvel" value="${jo.get('velocity', jd['velocity'])}"/>
    <joint name="${name}" type="${jtype}">
      <parent link="${parent}"/>
      <child link="${child}"/>
      <origin xyz="${xyz}" rpy="${rpy}"/>
      <xacro:unless value="${jtype == 'fixed'}">
        <axis xyz="${'%g %g %g' % (js * jaxis[0] + 0.0, js * jaxis[1] + 0.0, js * jaxis[2] + 0.0)}"/>
      </xacro:unless>
      <xacro:if value="${jtype in ('revolute', 'prismatic')}">
        <limit lower="${jlo if js > 0 else -jup}" upper="${jup if js > 0 else -jlo}"
               effort="${jeffort}" velocity="${jvel}"/>
      </xacro:if>
      <xacro:if value="${jtype == 'continuous'}">
        <limit effort="${jeffort}" velocity="${jvel}"/>
      </xacro:if>
    </joint>
  </xacro:macro>
</robot>
"""


def mount_snippet(comp: ComponentInfo) -> List[str]:
    """The joint a user adds to attach the separately exported robot to a mount link."""
    child = f"{NameSanitizer.sanitize(comp.component_name or comp.link_name)}_base_link"
    return [f'<joint name="{comp.link_name}_mount" type="fixed">',
            f'  <parent link="{comp.link_name}"/>',
            f'  <child link="{child}"/>',
            '</joint>']


class XacroBuilder:
    def __init__(self, package_name: str):
        self.package_name = package_name
        ET.register_namespace("xacro", XACRO_NS)

    @staticmethod
    def _x(tag: str) -> str:
        return f"{{{XACRO_NS}}}{tag}"

    def build(self, components: List[ComponentInfo]) -> str:
        robot = ET.Element("robot", name=self.package_name)
        robot.append(ET.Comment(" Generated by fusion_ros_urdf_exporter. Overwritten on every "
                                "export. Change joints in config/joints_override.yaml. "))
        ET.SubElement(robot, self._x("include"), filename="fusion_joint.xacro")
        cfg, override = f"{self.package_name}_joints_cfg", f"{self.package_name}_joints_override"
        ET.SubElement(robot, self._x("property"), name=cfg,
                      value="${xacro.load_yaml('../config/joints.yaml')}")
        ET.SubElement(robot, self._x("property"), name=override,
                      value="${xacro.load_yaml('../config/joints_override.yaml')}")

        for comp in components:
            self._add_link(robot, comp)
        for comp in components:
            if comp.parent is not None:
                p = comp.pose_in_parent
                ET.SubElement(robot, self._x("fusion_joint"), name=comp.joint.name,
                              parent=comp.parent.link_name, child=comp.link_name,
                              xyz=p.xyz_str(), rpy=p.rpy_str(),
                              cfg=f"${{{cfg}}}", override=f"${{{override}}}")

        raw = ET.tostring(robot, encoding="unicode")
        return minidom.parseString(raw).toprettyxml(indent="  ")

    def _add_link(self, robot: ET.Element, comp: ComponentInfo) -> None:
        if comp.is_mount:
            robot.append(ET.Comment(
                f" Mount point for '{comp.component_name}': attach it in your own xacro with\n"
                + "\n".join("     " + line for line in mount_snippet(comp)) + " "))
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


class JointsYamlWriter:
    def __init__(self, settings: ExportSettings):
        self.settings = settings

    def build(self, components: List[ComponentInfo]) -> str:
        f = fmt_yaml_float
        lines = ["# Generated by fusion_ros_urdf_exporter. Overwritten on every export.",
                 "# Do not edit: put your changes in joints_override.yaml.",
                 "# Units: rad, m, N m or N, rad/s or m/s. axis is in the child link frame.",
                 ""]
        for comp in components:
            if comp.parent is None:
                continue
            j = comp.joint
            source = "no joint" if j.source == "none" else f"Fusion joint '{j.source}'"
            lines += [
                f"{j.name}:",
                f"  # {comp.parent.link_name} -> {comp.link_name}, {source}, "
                f"frame: {comp.frame_source}",
                f"  type: {j.joint_type}",
                f"  axis: [{', '.join(f(v) for v in j.axis)}]",
                "  flip: false",
                f"  lower: {f(j.lower)}",
                f"  upper: {f(j.upper)}",
                f"  effort: {f(self.settings.default_effort)}",
                f"  velocity: {f(self.settings.default_velocity)}",
                "",
            ]
        if len(lines) == 4:
            lines.append("{}")      # no joints at all: still a valid (empty) mapping
        return "\n".join(lines)


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
  <exec_depend>xacro</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher_gui</exec_depend>
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
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
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

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('@PKG@')
    robot_description = xacro.process_file(
        os.path.join(share, 'urdf', '@PKG@.urdf.xacro')).toxml()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
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

    def __init__(self, folder: PackageFolder, base_link: str, settings: ExportSettings):
        self.folder = folder
        self.pkg = folder.name
        self.base_link = base_link
        self.settings = settings
        self.written: Set[str] = set()

    def _render(self, template: str) -> str:
        return (template
                .replace("@PKG@", self.pkg)
                .replace("@BASE@", self.base_link)
                .replace("@VERSION@", self.settings.package_version)
                .replace("@MAINTAINER@", self.settings.maintainer)
                .replace("@EMAIL@", self.settings.maintainer_email)
                .replace("@LICENSE@", self.settings.license))

    def write(self, rel_path: str, content: str) -> None:
        path = self.folder.abs(rel_path)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", encoding="utf-8", newline="\n") as f:
            f.write(content)
        self.written.add(rel_path)

    def write_package(self, robot_xacro: str, joints_yaml: str) -> None:
        self.write("package.xml", self._render(self.PACKAGE_XML))
        self.write("setup.py", self._render(self.SETUP_PY))
        self.write("setup.cfg", self._render(self.SETUP_CFG))
        self.write(f"resource/{self.pkg}", "")
        self.write(f"{self.pkg}/__init__.py", "")
        self.write("launch/display.launch.py", self._render(self.LAUNCH))
        self.write("rviz/display.rviz", self._render(self.RVIZ))
        self.write(f"urdf/{self.pkg}.urdf.xacro", robot_xacro)
        self.write("urdf/fusion_joint.xacro", JOINT_MACRO)
        self.write("config/joints.yaml", joints_yaml)


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

        folder = PackageFolder(self.prompt.ask_package_folder())
        folder.check()
        pkg = folder.name
        overrides = OverrideFile(folder.abs(OVERRIDE_FILE))

        # Read and validate everything before writing anything
        components = FusionModelReader(design, self.settings).read(pkg)
        joints = JointReader(design).read()
        components = KinematicTree(self.settings).build(components, joints, overrides.frames,
                                                        overrides.mounts)

        inertia = InertiaCalculator(self.settings)
        meshes = MeshExporter(design, self.settings)
        writer = RosPackageWriter(folder, components[0].link_name, self.settings)
        mesh_dir = folder.abs("meshes")
        os.makedirs(mesh_dir, exist_ok=True)

        progress = ui.createProgressDialog()
        progress.isCancelButtonShown = False
        progress.show("URDF exporter", "Exporting %v of %m", 0, len(components))
        try:
            for i, comp in enumerate(components):
                progress.message = f"Exporting {comp.fusion_name} (%v of %m)"
                progress.progressValue = i
                adsk.doEvents()
                if comp.is_mount:
                    continue                    # empty link: no mesh, no inertia
                comp.inertial = inertia.compute(comp)
                writer.written.add("meshes/" + meshes.export(comp, mesh_dir))
            progress.progressValue = len(components)
        finally:
            progress.hide()

        writer.write_package(XacroBuilder(pkg).build(components),
                             JointsYamlWriter(self.settings).build(components))
        created_override = overrides.create_if_missing()
        removed = folder.remove_stale(writer.written)
        folder.write_marker(writer.written)

        joint_names = {c.joint.name for c in components if c.joint}
        orphans = [k for k in overrides.keys if k not in joint_names]
        summary = self._summary(folder, components, removed, orphans, created_override)
        app.log(summary)
        self.prompt.show(summary)

    @staticmethod
    def _summary(folder: PackageFolder, components: List[ComponentInfo], removed: List[str],
                 orphans: List[str], created_override: bool) -> str:
        unit = {0.001: "mm", 0.01: "cm", 1.0: "m"}
        action = "Updated" if folder.is_previous_export else "Exported"
        lines = [f"{action} robot '{folder.name}' in:", folder.path, ""]
        for c in components:
            mass = f"{c.inertial.mass:.3f} kg" if c.inertial else "no inertia"
            lines.append(f"{c.link_name}  <-  {c.fusion_name}")
            if c.is_mount:
                lines.append("    mount point: no mesh, no inertia. Attach the other robot with:")
                lines += ["      " + s for s in mount_snippet(c)]
            else:
                lines.append(f"    mesh: {c.mesh_frame} frame, "
                             f"{unit.get(c.mesh_scale, c.mesh_scale)}, {mass}")
            if c.parent is not None:
                j = c.joint
                src = "no joint" if j.source == "none" else f"'{j.source}'"
                lines.append(f"    {j.joint_type} to {c.parent.link_name} ({src}, "
                             f"frame: {c.frame_source})")
                p = c.pose_in_parent
                lines.append(f"    xyz=\"{p.xyz_str()}\" rpy=\"{p.rpy_str()}\"")
            for w in c.warnings:
                lines.append(f"    WARNING: {w}")
        if orphans:
            lines += ["", "joints_override.yaml entries without a matching joint: "
                      + ", ".join(orphans)]
        if removed:
            lines += ["", "Removed files from the previous export: " + ", ".join(removed)]
        if created_override:
            lines += ["", f"Created {OVERRIDE_FILE} - edit it to change joints."]
        lines += ["", "Test in ROS 2:",
                  f"  colcon build --packages-select {folder.name}",
                  f"  ros2 launch {folder.name} display.launch.py"]
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
