"""Tests for fusion_ros_urdf_exporter, run outside Fusion with a fake `adsk`.

    python -m unittest discover -s tests -v

If the `xacro` Python package is installed (pip install xacro), the
generated package is also run through real xacro.
"""

import math
import os
import shutil
import struct
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "fake_adsk"))
sys.path.insert(0, os.path.dirname(HERE))

import adsk.core  # noqa: E402  (fake)
import fusion_ros_urdf_exporter as ue  # noqa: E402
from fake_design import (FakeAsBuiltJoint, FakeDesign, FakeExportManager,  # noqa: E402
                         FakeJoint, FakeOccurrence, Pose, box_corners, box_inertia_diag,
                         robodog_joints, robodog_occurrences, rot_rpy)

try:
    import xacro  # noqa: F401
    HAVE_XACRO = True
except ImportError:
    HAVE_XACRO = False

ROBOT = "my_robot"
BASE_LINK = f"{ROBOT}_base_link"
MOVED_BASE = Pose(rot_rpy(0.1, -0.2, 0.7), [0.3, -0.1, 0.05])
X = "{http://www.ros.org/wiki/xacro}"
REV, SLIDER, RIGID, CYL = 1, 2, 0, 3


def parse_origin(el):
    xyz = [float(v) for v in el.get("xyz").split()]
    rpy = [float(v) for v in el.get("rpy").split()]
    return Pose(rot_rpy(*rpy), xyz)


def stl_vertices(path):
    with open(path, "rb") as f:
        data = f.read()
    n = struct.unpack_from("<I", data, 80)[0]
    return [tri[k:k + 3] for tri in struct.iter_unpack("<12fH", data[84:84 + n * 50]) for k in (3, 6, 9)]


def read_joints_yaml(path):
    """Parser for the flat format JointsYamlWriter writes (no PyYAML needed)."""
    result, current = {}, None
    with open(path, encoding="utf-8") as f:
        for line in f:
            if not line.strip() or line.lstrip().startswith("#") or line.strip() == "{}":
                continue
            if not line.startswith(" "):
                current = line.split(":")[0]
                result[current] = {}
                continue
            key, value = [s.strip() for s in line.split(":", 1)]
            if value.startswith("["):
                value = [float(v) for v in value.strip("[]").split(",")]
            elif value in ("true", "false"):
                value = value == "true"
            else:
                try:
                    value = float(value)
                except ValueError:
                    pass
            result[current][key] = value
    return result


def box(name, world, visible=True, dims=(0.1, 0.1, 0.1), mass=1.0, com=(0, 0, 0)):
    return FakeOccurrence(name, world, list(com), list(dims), mass, visible)


class UnitTests(unittest.TestCase):
    def test_sanitize(self):
        s = ue.NameSanitizer.sanitize
        self.assertEqual(s("wheel_left_front:1"), "wheel_left_front_1")
        self.assertEqual(s("OAK D-Lite (v2):3"), "oak_d_lite_v2_3")
        self.assertEqual(s("  123 part"), "l_123_part")
        self.assertEqual(s(":::"), "unnamed")

    def test_unique_names(self):
        namer = ue.UniqueNamer()
        namer.reserve("a_base_link")
        self.assertEqual(namer.make("x:1"), "x_1")
        self.assertEqual(namer.make("X:1"), "x_1_2")
        self.assertEqual(namer.make("x 1"), "x_1_3")
        self.assertEqual(namer.make("a_base_link"), "a_base_link_2")

    def test_pose_roundtrip(self):
        p = Pose(rot_rpy(0.3, -1.1, 2.0), [1.0, -2.0, 0.5])
        q = p * p.inverse()
        for i in range(3):
            self.assertAlmostEqual(q.t[i], 0.0)
            for j in range(3):
                self.assertAlmostEqual(q.R[i][j], 1.0 if i == j else 0.0)
        for got, want in zip(p.rpy(), (0.3, -1.1, 2.0)):
            self.assertAlmostEqual(got, want)

    def test_frame_detector(self):
        world = ([1.0, 2.0, 3.0], [1.2, 2.1, 3.3])
        local = ([-0.1, -0.05, -0.15], [0.1, 0.05, 0.15])
        stl_world_mm = ([1000, 2000, 3000], [1200, 2100, 3300])
        self.assertEqual(ue.MeshFrameDetector.detect(stl_world_mm, world, local)[:2], ("world", 0.001))
        stl_local_cm = ([-10, -5, -15], [10, 5, 15])
        self.assertEqual(ue.MeshFrameDetector.detect(stl_local_cm, world, local)[:2], ("local", 0.01))

    def test_yaml_float(self):
        f = ue.fmt_yaml_float
        self.assertEqual(f(1.0), "1.0")
        self.assertEqual(f(-0.5), "-0.5")
        self.assertEqual(f(1e-5), "0.00001")
        self.assertEqual(f(1e-12), "0.0")

    def test_override_parser(self):
        path = os.path.join(tempfile.mkdtemp(), "o.yaml")
        with open(path, "w") as f:
            f.write(ue.OverrideFile.TEMPLATE)
            f.write("a_joint:\n  flip: true\n  frame: origin  # comment\n"
                    "b_joint:   # x\n  frame: 'joint'\n  type: fixed\n")
        o = ue.OverrideFile(path)
        self.assertEqual(o.keys, ["a_joint", "b_joint"])
        self.assertEqual(o.frames, {"a_joint": "origin", "b_joint": "joint"})
        shutil.rmtree(os.path.dirname(path))


class ExportTestBase(unittest.TestCase):
    def setUp(self):
        self.out = tempfile.mkdtemp()
        self.pkg = os.path.join(self.out, ROBOT)
        os.makedirs(self.pkg)
        self.app = adsk.core.Application.get()
        self.app.userInterface.reset()
        self.app.logs.clear()
        self.app.userInterface.folder = self.pkg

    def tearDown(self):
        shutil.rmtree(self.out, ignore_errors=True)

    def export(self, occs, joints=(), as_built=(), frame="world", unit=0.001, expect_ok=True):
        em = FakeExportManager(frame, unit)
        self.app.activeProduct = FakeDesign(occs, em, joints, as_built)
        self.app.logs.clear()
        self.app.userInterface.messages.clear()
        ue.run("")
        failed = [log for log in self.app.logs if log.startswith("Failed")]
        self.assertFalse(failed, failed and failed[0])
        if expect_ok:
            self.assertTrue(self.app.logs, self.app.userInterface.messages)
        return em

    @property
    def summary(self):
        return self.app.logs[-1]

    @property
    def last_message(self):
        return self.app.userInterface.messages[-1]

    def read_package(self):
        root = ET.parse(os.path.join(self.pkg, "urdf", f"{ROBOT}.urdf.xacro")).getroot()
        links = {link.get("name"): link for link in root.findall("link")}
        joints = {j.get("child"): j for j in root.findall(f"{X}fusion_joint")}
        cfg = read_joints_yaml(os.path.join(self.pkg, "config", "joints.yaml"))
        return links, joints, cfg

    def link_worlds(self, base_world):
        links, joints, _ = self.read_package()
        worlds = {BASE_LINK: base_world}

        def world(name):
            if name not in worlds:
                j = joints[name]
                worlds[name] = world(j.get("parent")) * parse_origin(j)
            return worlds[name]

        for name in links:
            world(name)
        return worlds

    def check_geometry(self, occs, names, base_world):
        """Every mesh vertex on the true box; COM and inertia exact."""
        links, _, _ = self.read_package()
        worlds = self.link_worlds(base_world)
        self.assertEqual(set(links), set(names.values()))
        for occ in occs:
            if occ.name not in names:
                continue
            name = names[occ.name]
            link, link_world = links[name], worlds[name]

            visual = link.find("visual")
            mesh = visual.find("geometry/mesh")
            self.assertEqual(mesh.get("filename"), f"package://{ROBOT}/meshes/{name}.stl")
            scale = float(mesh.get("scale").split()[0])
            visual_pose = parse_origin(visual.find("origin"))
            true_corners = [occ.world.apply(p) for p in box_corners(occ.com_local, occ.dims)]
            for v in stl_vertices(os.path.join(self.pkg, "meshes", f"{name}.stl")):
                placed = link_world.apply(visual_pose.apply([c * scale for c in v]))
                err = min(math.dist(placed, c) for c in true_corners)
                self.assertLess(err, 1e-5, f"{name}: mesh vertex off by {err * 1000:.4f} mm")

            inertial = link.find("inertial")
            com = [float(v) for v in inertial.find("origin").get("xyz").split()]
            self.assertLess(math.dist(link_world.apply(com), occ.world.apply(occ.com_local)), 1e-6)
            self.assertAlmostEqual(float(inertial.find("mass").get("value")), occ.mass)
            inertia = inertial.find("inertia")
            want = box_inertia_diag(occ.mass, occ.dims)
            # Box inertia in link axes: rotate the true tensor (component axes) into link axes
            r = ue.mat_mul(ue.transpose(link_world.R), occ.world.R)
            i_comp = [[want[0], 0, 0], [0, want[1], 0], [0, 0, want[2]]]
            i_link = ue.mat_mul(r, ue.mat_mul(i_comp, ue.transpose(r)))
            for key, (a, b) in {"ixx": (0, 0), "iyy": (1, 1), "izz": (2, 2),
                                "ixy": (0, 1), "ixz": (0, 2), "iyz": (1, 2)}.items():
                # rpy is written with 6 decimals -> relative tolerance
                self.assertAlmostEqual(float(inertia.get(key)), i_link[a][b],
                                       delta=max(want) * 1e-5, msg=f"{name} {key}")


class RobodogTests(ExportTestBase):
    def names(self, occs):
        namer = ue.UniqueNamer()
        namer.reserve(BASE_LINK)
        visible = [o for o in occs if o.isLightBulbOn]
        return {o.name: BASE_LINK if o.name.startswith("base_link") else namer.make(o.name)
                for o in sorted(visible, key=lambda o: o.name.startswith("base_link"), reverse=True)}

    def test_scenarios(self):
        for with_joints in (False, True):
            for frame in ("world", "local"):
                for unit in (0.001, 0.01, 1.0):
                    for moved in (False, True):
                        with self.subTest(joints=with_joints, frame=frame, unit=unit, moved=moved):
                            shutil.rmtree(self.pkg)
                            os.makedirs(self.pkg)
                            base = MOVED_BASE if moved else Pose()
                            occs = robodog_occurrences(base)
                            joints = robodog_joints(occs) if with_joints else []
                            em = self.export(occs, joints, frame=frame, unit=unit)
                            self.assertTrue(all(o.isBinaryFormat and o.meshRefinement == 2
                                                for o in em.calls))
                            self.check_geometry(occs, self.names(occs), base)
                            _, xj, cfg = self.read_package()
                            for child, j in xj.items():
                                self.assertEqual(j.get("parent"), BASE_LINK)
                                wheel = child.startswith("wheel") and not child.endswith("_2")
                                want = "continuous" if (with_joints and wheel) else "fixed"
                                self.assertEqual(cfg[j.get("name")]["type"], want, child)

    def test_wheel_axis_and_frame(self):
        occs = robodog_occurrences(MOVED_BASE)
        self.export(occs, robodog_joints(occs))
        _, xj, cfg = self.read_package()
        worlds = self.link_worlds(MOVED_BASE)
        wheel = next(o for o in occs if o.name == "wheel_left_back:1")
        link = "wheel_left_back_1"
        axis_world = ue.mat_vec(worlds[link].R, cfg[f"{link}_joint"]["axis"])
        want = [wheel.world.R[k][2] for k in range(3)]
        self.assertLess(math.dist(axis_world, want), 1e-6)
        # Link frame = joint frame: rotated 0.4 rad about the wheel axis
        joint_frame = wheel.world * Pose(rot_rpy(0, 0, 0.4), [0, 0, 0])
        for i in range(3):
            for k in range(3):
                self.assertAlmostEqual(worlds[link].R[i][k], joint_frame.R[i][k], places=6)
        self.assertIn("frame: joint", self.summary)

    def test_package_files(self):
        occs = robodog_occurrences(Pose())
        self.export(occs, robodog_joints(occs))
        for rel in ("package.xml", "setup.py", "setup.cfg", f"resource/{ROBOT}",
                    f"{ROBOT}/__init__.py", "launch/display.launch.py", "rviz/display.rviz",
                    f"urdf/{ROBOT}.urdf.xacro", "urdf/fusion_joint.xacro",
                    "config/joints.yaml", "config/joints_override.yaml", ue.MARKER_FILE):
            self.assertTrue(os.path.isfile(os.path.join(self.pkg, rel)), rel)
        for rel in ("setup.py", "launch/display.launch.py"):
            with open(os.path.join(self.pkg, rel), encoding="utf-8") as f:
                compile(f.read(), rel, "exec")
        pkg_xml = ET.parse(os.path.join(self.pkg, "package.xml")).getroot()
        self.assertEqual(pkg_xml.findtext("name"), ROBOT)
        deps = {d.text for d in pkg_xml.findall("exec_depend")}
        self.assertTrue({"xacro", "joint_state_publisher_gui"} <= deps)
        with open(os.path.join(self.pkg, "rviz", "display.rviz")) as f:
            self.assertIn(f"Fixed Frame: {BASE_LINK}", f.read())
        with open(os.path.join(self.pkg, ue.MARKER_FILE)) as f:
            marker = f.read()
        self.assertIn("meshes/wheel_left_back_1.stl", marker)
        self.assertNotIn("joints_override.yaml", marker)

    @unittest.skipUnless(HAVE_XACRO, "xacro not installed (pip install xacro)")
    def test_real_xacro(self):
        occs = robodog_occurrences(Pose())
        self.export(occs, robodog_joints(occs))
        with open(os.path.join(self.pkg, "config", "joints_override.yaml"), "a") as f:
            f.write("wheel_left_back_1_joint:\n  flip: true\n"
                    "wheel_right_back_1_joint:\n  type: fixed\n")
        urdf = xacro.process_file(os.path.join(self.pkg, "urdf", f"{ROBOT}.urdf.xacro")).toxml()
        root = ET.fromstring(urdf)
        joints = {j.get("name"): j for j in root.findall("joint")}
        self.assertEqual(len(root.findall("link")), 8)
        self.assertEqual(joints["wheel_left_front_1_joint"].get("type"), "continuous")
        self.assertEqual(joints["wheel_right_back_1_joint"].get("type"), "fixed")
        self.assertIsNone(joints["wheel_right_back_1_joint"].find("axis"))
        _, _, cfg = self.read_package()
        axis = cfg["wheel_left_back_1_joint"]["axis"]
        got = [float(v) for v in joints["wheel_left_back_1_joint"].find("axis").get("xyz").split()]
        self.assertLess(math.dist(got, [-a for a in axis]), 1e-5)


class JointTreeTests(ExportTestBase):
    def arm(self, value_deg=30.0):
        base = box("base_link:1", Pose())
        l1 = box("link1:1", Pose(rot_rpy(0, 0, math.radians(value_deg)), [0, 0, 0.2]))
        l2 = box("link2:1", l1.world * Pose(translation=[0.3, 0, 0.02]))
        l3 = box("link3:1", l2.world * Pose(translation=[0.1, 0, 0]))
        joints = [
            FakeJoint("Rev1", l1, base, REV, l1.world, [0, 0, 1], math.radians(value_deg),
                      (math.radians(-90), math.radians(90))),
            FakeJoint("Slider1", l2, l1, SLIDER, l2.world, ue.mat_vec(l1.world.R, [1, 0, 0]),
                      2.0, (0.0, 10.0)),     # cm
            FakeJoint("Cyl1", l3, l2, CYL, l3.world, [1, 0, 0]),
        ]
        return [base, l1, l2, l3], joints

    def test_chain_types_and_limits(self):
        occs, joints = self.arm()
        self.export(occs, joints)
        _, xj, cfg = self.read_package()
        self.assertEqual(xj["link1_1"].get("parent"), BASE_LINK)
        self.assertEqual(xj["link2_1"].get("parent"), "link1_1")
        self.assertEqual(xj["link3_1"].get("parent"), "link2_1")
        j1, j2, j3 = cfg["link1_1_joint"], cfg["link2_1_joint"], cfg["link3_1_joint"]
        self.assertEqual(j1["type"], "revolute")
        self.assertAlmostEqual(j1["lower"], math.radians(-120), places=6)
        self.assertAlmostEqual(j1["upper"], math.radians(60), places=6)
        self.assertEqual(j2["type"], "prismatic")
        self.assertAlmostEqual(j2["lower"], -0.02, places=6)
        self.assertAlmostEqual(j2["upper"], 0.08, places=6)
        self.assertLess(math.dist(j2["axis"], [1, 0, 0]), 1e-6)
        self.assertEqual(j3["type"], "fixed")
        self.assertIn("'Cyl1' is cylindrical - exported as fixed", self.summary)
        self.check_geometry(occs, {"base_link:1": BASE_LINK, "link1:1": "link1_1",
                                   "link2:1": "link2_1", "link3:1": "link3_1"}, Pose())

    def test_as_built_joint(self):
        base = box("base_link:1", Pose())
        w = box("wheel:1", Pose(rot_rpy(math.pi / 2, 0, 0), [0, 0.2, 0]))
        joint = FakeAsBuiltJoint("AB1", w, base, REV, w.world, ue.mat_vec(w.world.R, [0, 0, 1]))
        self.export([base, w], as_built=[joint])
        _, _, cfg = self.read_package()
        self.assertEqual(cfg["wheel_1_joint"]["type"], "continuous")
        self.assertLess(math.dist(cfg["wheel_1_joint"]["axis"], [0, 0, 1]), 1e-6)

    def test_island_and_loose_components(self):
        base = box("base_link:1", Pose())
        a = box("a:1", Pose(translation=[1, 0, 0]))
        b = box("b:1", Pose(translation=[1, 1, 0]))
        sensor = box("sensor:1", Pose(translation=[0, 0, 1]))
        self.export([base, a, b, sensor], [FakeJoint("R", a, b, RIGID, a.world)])
        _, xj, _ = self.read_package()
        self.assertEqual(xj["a_1"].get("parent"), "b_1")
        self.assertEqual(xj["b_1"].get("parent"), BASE_LINK)
        self.assertEqual(xj["sensor_1"].get("parent"), BASE_LINK)

    def test_base_as_component_one_is_flipped(self):
        base = box("base_link:1", Pose())
        a = box("a:1", Pose(translation=[1, 0, 0]))
        self.export([base, a], [FakeJoint("R", base, a, REV, a.world, [0, 0, 1])])
        _, xj, _ = self.read_package()
        self.assertEqual(xj["a_1"].get("parent"), BASE_LINK)
        self.assertIn("direction flipped", self.summary)

    def test_joint_to_root_component(self):
        base = box("base_link:1", Pose())
        a = box("a:1", Pose(translation=[1, 0, 0]))
        self.export([base, a], [FakeJoint("R", a, None, REV, a.world, [0, 0, 1])])
        _, xj, cfg = self.read_package()
        self.assertEqual(xj["a_1"].get("parent"), BASE_LINK)
        self.assertEqual(cfg["a_1_joint"]["type"], "continuous")

    def test_hidden_parent_and_suppressed(self):
        base = box("base_link:1", Pose())
        hidden = box("hidden:1", Pose(translation=[0, 1, 0]), visible=False)
        a = box("a:1", Pose(translation=[1, 0, 0]))
        b = box("b:1", Pose(translation=[2, 0, 0]))
        self.export([base, hidden, a, b], [
            FakeJoint("ToHidden", a, hidden, REV, a.world, [0, 0, 1]),
            FakeJoint("Off", b, a, REV, b.world, [0, 0, 1], suppressed=True)])
        _, xj, cfg = self.read_package()
        self.assertEqual(xj["a_1"].get("parent"), BASE_LINK)
        self.assertEqual(cfg["a_1_joint"]["type"], "fixed")
        self.assertEqual(xj["b_1"].get("parent"), BASE_LINK)
        self.assertIn("hidden component - ignored", self.summary)

    def test_two_parents_is_error(self):
        base = box("base_link:1", Pose())
        a, b, c = (box(f"{n}:1", Pose(translation=[i, 0, 0])) for i, n in enumerate("abc", 1))
        self.export([base, a, b, c], [FakeJoint("J1", a, b, RIGID, a.world),
                                      FakeJoint("J2", a, c, RIGID, a.world)], expect_ok=False)
        self.assertIn("more than one parent", self.last_message)
        self.assertEqual(os.listdir(self.pkg), [])

    def test_cycle_is_error(self):
        base = box("base_link:1", Pose())
        a = box("a:1", Pose(translation=[1, 0, 0]))
        b = box("b:1", Pose(translation=[2, 0, 0]))
        self.export([base, a, b], [FakeJoint("J1", a, b, RIGID, a.world),
                                   FakeJoint("J2", b, a, RIGID, b.world)], expect_ok=False)
        self.assertIn("closed loop", self.last_message)
        self.assertEqual(os.listdir(self.pkg), [])

    def test_missing_joint_geometry_falls_back_to_origin(self):
        base = box("base_link:1", Pose())
        a = box("a:1", Pose(translation=[1, 0, 0]))
        self.export([base, a], [FakeJoint("R", a, base, REV, None, [0, 0, 1])])
        self.assertIn("not readable - link frame = component origin", self.summary)
        self.check_geometry([base, a], {"base_link:1": BASE_LINK, "a:1": "a_1"}, Pose())


class FolderTests(ExportTestBase):
    def simple(self, with_extra=True):
        base = box("base_link:1", Pose())
        w = box("wheel:1", Pose(translation=[0, 0.2, 0.05]))
        occs = [base, w]
        joint_frame = w.world * Pose(translation=[0, 0, 0.01])
        joints = [FakeJoint("Rev", w, base, REV, joint_frame, [0, 0, 1])]
        if with_extra:
            occs.append(box("extra:1", Pose(translation=[0, 0, 0.5])))
        return occs, joints

    def test_missing_base_link(self):
        occs = [o for o in robodog_occurrences(Pose()) if not o.name.startswith("base_link")]
        self.export(occs, expect_ok=False)
        self.assertIn("No top-level component named 'base_link'", self.last_message)
        self.assertEqual(os.listdir(self.pkg), [])

    def test_invalid_folder_name(self):
        bad = os.path.join(self.out, "My-Robot")
        os.makedirs(bad)
        self.app.userInterface.folder = bad
        self.export(self.simple()[0], expect_ok=False)
        self.assertIn("not a valid ROS package name", self.last_message)
        self.assertIn("'my_robot'", self.last_message)
        self.assertEqual(os.listdir(bad), [])

    def test_foreign_folder_is_refused(self):
        junk = os.path.join(self.pkg, "notes.txt")
        open(junk, "w").close()
        self.export(self.simple()[0], expect_ok=False)
        self.assertIn("not created by this exporter", self.last_message)
        self.assertEqual(os.listdir(self.pkg), ["notes.txt"])

    def test_cancel(self):
        self.app.userInterface.cancel_folder = True
        self.export(self.simple()[0], expect_ok=False)
        self.assertEqual(self.app.userInterface.messages, [])

    def test_reexport_keeps_override_and_user_files(self):
        occs, joints = self.simple()
        self.export(occs, joints)
        self.assertIn("Created config/joints_override.yaml", self.summary)
        override = os.path.join(self.pkg, "config", "joints_override.yaml")
        with open(override, "a") as f:
            f.write("wheel_1_joint:\n  flip: true\nold_part_joint:\n  type: fixed\n")
        user_file = os.path.join(self.pkg, "launch", "my_own.launch.py")
        open(user_file, "w").close()

        occs, joints = self.simple(with_extra=False)      # extra:1 removed from the model
        self.export(occs, joints)
        self.assertIn("Updated robot", self.summary)
        with open(override) as f:
            self.assertIn("wheel_1_joint:\n  flip: true", f.read())
        self.assertTrue(os.path.isfile(user_file))
        self.assertFalse(os.path.exists(os.path.join(self.pkg, "meshes", "extra_1.stl")))
        self.assertIn("meshes/extra_1.stl", self.summary)
        self.assertIn("without a matching joint: old_part_joint", self.summary)
        self.assertNotIn("Created config/joints_override.yaml", self.summary)

    def test_frame_origin_override(self):
        occs, joints = self.simple(with_extra=False)
        self.export(occs, joints)
        _, xj, _ = self.read_package()
        self.assertAlmostEqual(parse_origin(xj["wheel_1"]).t[2], 0.06)    # joint frame
        with open(os.path.join(self.pkg, "config", "joints_override.yaml"), "a") as f:
            f.write("wheel_1_joint:\n  frame: origin\n")
        self.export(occs, joints)
        _, xj, _ = self.read_package()
        self.assertAlmostEqual(parse_origin(xj["wheel_1"]).t[2], 0.05)    # component origin
        self.assertIn("frame: origin", self.summary)
        self.check_geometry(occs, {"base_link:1": BASE_LINK, "wheel:1": "wheel_1"}, Pose())

    def test_axis_line_warning_with_origin_frame(self):
        base = box("base_link:1", Pose())
        w = box("wheel:1", Pose(translation=[0, 0.2, 0.05]))
        frame = w.world * Pose(translation=[0.01, 0, 0])            # axis 10 mm off
        os.makedirs(os.path.join(self.pkg, "config"))
        with open(os.path.join(self.pkg, ue.MARKER_FILE), "w") as f:
            f.write("# previous export\n")
        with open(os.path.join(self.pkg, "config", "joints_override.yaml"), "w") as f:
            f.write("wheel_1_joint:\n  frame: origin\n")
        self.export([base, w], [FakeJoint("Rev", w, base, REV, frame, [0, 0, 1])])
        self.assertIn("10.0 mm from the component origin", self.summary)


if __name__ == "__main__":
    unittest.main()
