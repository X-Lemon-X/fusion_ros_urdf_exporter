"""Tests for fusion_ros_urdf_exporter, run outside Fusion with a fake `adsk`.

    python -m unittest discover -s tests -v
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
from fake_design import (FakeDesign, FakeExportManager, Pose, box_corners,  # noqa: E402
                         box_inertia_diag, robodog_occurrences, rot_rpy)

ROBOT = "my_robot"
BASE_LINK = f"{ROBOT}_base_link"
MOVED_BASE = Pose(rot_rpy(0.1, -0.2, 0.7), [0.3, -0.1, 0.05])


def parse_origin(el):
    xyz = [float(v) for v in el.get("xyz").split()]
    rpy = [float(v) for v in el.get("rpy").split()]
    return Pose(rot_rpy(*rpy), xyz)


def stl_vertices(path):
    with open(path, "rb") as f:
        data = f.read()
    n = struct.unpack_from("<I", data, 80)[0]
    return [tri[k:k + 3] for tri in struct.iter_unpack("<12fH", data[84:84 + n * 50]) for k in (3, 6, 9)]


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


class ExportTests(unittest.TestCase):
    def setUp(self):
        self.out = tempfile.mkdtemp()
        self.app = adsk.core.Application.get()
        self.app.userInterface.reset()
        self.app.logs.clear()
        self.app.userInterface.folder = self.out
        self.app.userInterface.robot_name = "My Robot"

    def tearDown(self):
        shutil.rmtree(self.out, ignore_errors=True)

    def _export(self, frame="world", unit=0.001, base_world=None):
        base_world = base_world or Pose()
        occs = robodog_occurrences(base_world)
        em = FakeExportManager(frame, unit)
        self.app.activeProduct = FakeDesign(occs, em)
        ue.run("")
        failed = [log for log in self.app.logs if log.startswith("Failed")]
        self.assertFalse(failed, failed and failed[0])
        return occs, em, base_world

    def _check_geometry(self, occs, base_world):
        pkg = os.path.join(self.out, ROBOT)
        root = ET.parse(os.path.join(pkg, "urdf", f"{ROBOT}.urdf")).getroot()
        links = {link.get("name"): link for link in root.findall("link")}
        joints = {j.find("child").get("link"): j for j in root.findall("joint")}

        namer = ue.UniqueNamer()
        namer.reserve(BASE_LINK)
        visible = [o for o in occs if o.isLightBulbOn]
        names = {o.name: BASE_LINK if o.name.startswith("base_link") else namer.make(o.name)
                 for o in sorted(visible, key=lambda o: o.name.startswith("base_link"), reverse=True)}
        self.assertEqual(set(links), set(names.values()))

        for occ in visible:
            name = names[occ.name]
            link = links[name]
            in_base = Pose() if name == BASE_LINK else parse_origin(joints[name].find("origin"))
            if name != BASE_LINK:
                self.assertEqual(joints[name].get("type"), "fixed")
                self.assertEqual(joints[name].find("parent").get("link"), BASE_LINK)
            link_world = base_world * in_base

            # Every mesh vertex must land on a corner of the true box
            visual = link.find("visual")
            mesh = visual.find("geometry/mesh")
            self.assertEqual(mesh.get("filename"), f"package://{ROBOT}/meshes/{name}.stl")
            scale = float(mesh.get("scale").split()[0])
            visual_pose = parse_origin(visual.find("origin"))
            true_corners = [occ.world.apply(p) for p in box_corners(occ.com_local, occ.dims)]
            for v in stl_vertices(os.path.join(pkg, "meshes", f"{name}.stl")):
                placed = link_world.apply(visual_pose.apply([c * scale for c in v]))
                err = min(math.dist(placed, c) for c in true_corners)
                self.assertLess(err, 1e-5, f"{name}: mesh vertex off by {err * 1000:.4f} mm")

            # Centre of mass and inertia about it, in link axes
            inertial = link.find("inertial")
            com = [float(v) for v in inertial.find("origin").get("xyz").split()]
            self.assertLess(math.dist(link_world.apply(com), occ.world.apply(occ.com_local)), 1e-6)
            self.assertAlmostEqual(float(inertial.find("mass").get("value")), occ.mass)
            inertia = inertial.find("inertia")
            want = box_inertia_diag(occ.mass, occ.dims)
            for key, w in zip(("ixx", "iyy", "izz"), want):
                self.assertAlmostEqual(float(inertia.get(key)) / w, 1.0, places=6, msg=f"{name} {key}")
            for key in ("ixy", "ixz", "iyz"):
                self.assertAlmostEqual(float(inertia.get(key)) / max(want), 0.0, places=6)

    def test_scenarios(self):
        for frame in ("world", "local"):
            for unit in (0.001, 0.01, 1.0):
                for moved in (False, True):
                    with self.subTest(frame=frame, unit=unit, moved=moved):
                        shutil.rmtree(os.path.join(self.out, ROBOT), ignore_errors=True)
                        base = MOVED_BASE if moved else Pose()
                        occs, em, base_world = self._export(frame, unit, base)
                        self.assertTrue(all(o.isBinaryFormat and o.meshRefinement == 2 for o in em.calls))
                        self._check_geometry(occs, base_world)

    def test_package_files(self):
        self._export()
        pkg = os.path.join(self.out, ROBOT)
        for rel in ("package.xml", "setup.py", "setup.cfg", f"resource/{ROBOT}",
                    f"{ROBOT}/__init__.py", "launch/display.launch.py", "rviz/display.rviz"):
            self.assertTrue(os.path.isfile(os.path.join(pkg, rel)), rel)
        for rel in ("setup.py", "launch/display.launch.py"):
            with open(os.path.join(pkg, rel), encoding="utf-8") as f:
                compile(f.read(), rel, "exec")
        self.assertEqual(ET.parse(os.path.join(pkg, "package.xml")).getroot().findtext("name"), ROBOT)
        with open(os.path.join(pkg, "rviz", "display.rviz")) as f:
            self.assertIn(f"Fixed Frame: {BASE_LINK}", f.read())

    def test_overwrite_yes_deletes_old_folder(self):
        junk = os.path.join(self.out, ROBOT, "junk.txt")
        os.makedirs(os.path.dirname(junk))
        open(junk, "w").close()
        self._export()
        self.assertFalse(os.path.exists(junk))
        self.assertTrue(os.path.isfile(os.path.join(self.out, ROBOT, "urdf", f"{ROBOT}.urdf")))

    def test_overwrite_no_leaves_folder_untouched(self):
        junk = os.path.join(self.out, ROBOT, "junk.txt")
        os.makedirs(os.path.dirname(junk))
        open(junk, "w").close()
        self.app.userInterface.overwrite = False
        self._export()
        self.assertEqual(os.listdir(os.path.join(self.out, ROBOT)), ["junk.txt"])

    def test_missing_base_link(self):
        occs = [o for o in robodog_occurrences(Pose()) if not o.name.startswith("base_link")]
        self.app.activeProduct = FakeDesign(occs, FakeExportManager())
        ue.run("")
        self.assertIn("No top-level component named 'base_link'", self.app.userInterface.messages[-1])
        self.assertFalse(os.path.exists(os.path.join(self.out, ROBOT)))


if __name__ == "__main__":
    unittest.main()
