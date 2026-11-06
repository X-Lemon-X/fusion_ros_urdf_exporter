"""A fake Fusion design: box-shaped components with known geometry and inertia.

Every component is a solid box (`dims`, centered at `com_local` in the
component's own coordinates). That gives exact expected values for the mesh
placement, centre of mass and inertia the exporter writes to the URDF.
"""

import math
import struct

import fusion_ros_urdf_exporter as ue

Pose = ue.Pose


def rot_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def box_corners(center, dims):
    return [[center[0] + sx * dims[0] / 2,
             center[1] + sy * dims[1] / 2,
             center[2] + sz * dims[2] / 2]
            for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]


def box_inertia_diag(mass, dims):
    a, b, c = dims
    return [mass * (b * b + c * c) / 12, mass * (a * a + c * c) / 12, mass * (a * a + b * b) / 12]


class _Point:
    def __init__(self, x, y, z):
        self.x, self.y, self.z = x, y, z


class _BoundingBox:
    def __init__(self, points_m):
        self.minPoint = _Point(*[min(p[k] for p in points_m) * 100 for k in range(3)])
        self.maxPoint = _Point(*[max(p[k] for p in points_m) * 100 for k in range(3)])


class _Matrix3D:
    def __init__(self, pose):
        self._pose = pose

    def getCell(self, row, col):
        return self._pose.R[row][col] if col < 3 else self._pose.t[row] * 100  # cm


class _PhysicalProperties:
    """Inertia tensor about the world origin in kg*cm^2, like Fusion."""

    def __init__(self, occ):
        self.mass = occ.mass
        com_world = occ.world.apply(occ.com_local)
        self.centerOfMass = _Point(*[v * 100 for v in com_world])
        diag = box_inertia_diag(occ.mass, occ.dims)
        i_local = [[diag[0], 0, 0], [0, diag[1], 0], [0, 0, diag[2]]]
        r = occ.world.R
        i_world = ue.mat_mul(r, ue.mat_mul(i_local, ue.transpose(r)))
        x, y, z = com_world
        d = [[y * y + z * z, -x * y, -x * z],
             [-x * y, x * x + z * z, -y * z],
             [-x * z, -y * z, x * x + y * y]]
        self._i = [[(i_world[i][j] + occ.mass * d[i][j]) * 1e4 for j in range(3)]
                   for i in range(3)]

    def getXYZMomentsOfInertia(self):
        i = self._i
        return (True, i[0][0], i[1][1], i[2][2], i[0][1], i[1][2], i[0][2])


class _Component:
    def __init__(self, occ):
        self._occ = occ
        self.name = occ.name.split(":")[0]

    @property
    def boundingBox(self):
        return _BoundingBox(box_corners(self._occ.com_local, self._occ.dims))


class FakeOccurrence:
    def __init__(self, name, world, com_local, dims, mass, visible=True):
        self.name = name
        self.world = world
        self.com_local = com_local
        self.dims = dims
        self.mass = mass
        self.isLightBulbOn = visible
        self.entityToken = name
        self.component = _Component(self)

    @property
    def transform2(self):
        return _Matrix3D(self.world)

    @property
    def boundingBox(self):
        return _BoundingBox([self.world.apply(p) for p in box_corners(self.com_local, self.dims)])

    def getPhysicalProperties(self, accuracy):
        return _PhysicalProperties(self)


class _StlOptions:
    def __init__(self, occ, path):
        self.occ, self.path = occ, path
        self.isBinaryFormat = False
        self.meshRefinement = None


class FakeExportManager:
    """Writes a 12-triangle box STL in the chosen frame ("world"/"local") and unit."""

    TRIANGLES = [(0, 1, 2), (1, 3, 2), (4, 6, 5), (5, 6, 7), (0, 4, 1), (1, 4, 5),
                 (2, 3, 6), (3, 7, 6), (0, 2, 4), (2, 6, 4), (1, 5, 3), (3, 5, 7)]

    def __init__(self, frame="world", unit=0.001):
        self.frame, self.unit = frame, unit
        self.calls = []

    def createSTLExportOptions(self, occ, path):
        return _StlOptions(occ, path)

    def execute(self, opts):
        self.calls.append(opts)
        pts = box_corners(opts.occ.com_local, opts.occ.dims)
        if self.frame == "world":
            pts = [opts.occ.world.apply(p) for p in pts]
        pts = [[v / self.unit for v in p] for p in pts]
        with open(opts.path, "wb") as f:
            f.write(b"\0" * 80)
            f.write(struct.pack("<I", len(self.TRIANGLES)))
            for i, j, k in self.TRIANGLES:
                f.write(struct.pack("<12fH", 0, 0, 0, *pts[i], *pts[j], *pts[k], 0))
        return True


class FakeDesign:
    def __init__(self, occurrences, export_manager):
        class _Root:
            pass

        self.rootComponent = _Root()
        self.rootComponent.occurrences = occurrences
        self.exportManager = export_manager


def robodog_occurrences(base_world):
    """Poses from the real robodog model, optionally with the whole robot moved."""

    def at(rpy_deg, t):
        return base_world * Pose(rot_rpy(*[math.radians(a) for a in rpy_deg]), t)

    wheel = dict(dims=[0.15, 0.15, 0.04], mass=0.6)
    return [
        FakeOccurrence("wheel_left_back:1", at((90, 90, 0), [0.175092, -0.2, 0.040096]), [0, 0, 0.01], **wheel),
        FakeOccurrence("wheel_right_back:1", at((90, 90, 0), [0.175092, 0.2, 0.040096]), [0, 0, -0.01], **wheel),
        FakeOccurrence("wheel_left_front:1", at((-90, 90, 0), [-0.175092, -0.2, 0.040096]), [0, 0, 0.01], **wheel),
        FakeOccurrence("wheel_right_front:1", at((-90, 90, 0), [-0.175092, 0.2, 0.040096]), [0, 0, 0.01], **wheel),
        FakeOccurrence("base_link:1", base_world, [0.0, 0.0, 0.06], [0.556, 0.532, 0.127], 8.0),
        FakeOccurrence("OAK_D_LITE_DepthAI:1", at((100, 0, -90), [0.090657, 0.023671, 0.646387]),
                       [0.01, 0.0, 0.0], [0.09, 0.02, 0.03], 0.06),
        FakeOccurrence("STL_19P_3D:1", at((0, 0, 90), [0.253, 0, 0.115098]), [0, 0, 0.017], [0.04, 0.04, 0.035], 0.05),
        FakeOccurrence("Hidden Thing:1", at((0, 0, 0), [0, 0, 1]), [0, 0, 0], [0.1, 0.1, 0.1], 1.0, visible=False),
        # Sanitizes to the same name as wheel_left_back:1 -> gets suffix wheel_left_back_1_2
        FakeOccurrence("Wheel Left Back:1",at((0, 0, 0), [0, 0, 0.3]), [0, 0, 0], [0.05, 0.05, 0.05], 0.1),
    ]
