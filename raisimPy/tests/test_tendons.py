"""Run with PYTHONPATH pointing to the built module and RAISIM_ACTIVATION_KEY set."""
import math
import os
from pathlib import Path
import tempfile
import unittest
import numpy as np
import raisimpy as r


class TendonTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        key = os.environ.get("RAISIM_ACTIVATION_KEY")
        if key:
            r.World.setActivationKey(key)

    def setUp(self):
        self.world = r.World()
        self.world.setGravity(np.zeros(3))
        self.world.setTimeStep(0.001)
        self.body = self.world.addSphere(0.02, 1.0)
        self.body.setName("load")
        self.body.setPosition(1.0, 0.0, 0.0)

    def cable(self, name="cable", properties=None):
        path, site = r.Tendon.PathElement, r.Tendon.Site
        return self.world.addSpatialTendon(name, [path.via(site()), path.via(site(self.body))],
            properties if properties is not None else r.Tendon.Properties())

    def test_single_interface_and_copied_properties(self):
        for old in ("addStiffWire", "addCompliantWire", "addCustomWire", "getWire", "getWires"):
            self.assertFalse(hasattr(self.world, old))
        t = self.cable()
        p = t.getProperties()
        p.upperLimit = 0.9
        self.assertTrue(math.isinf(t.getProperties().upperLimit))
        t.setProperties(p)
        self.assertEqual(t.getProperties().upperLimit, 0.9)
        for _ in range(200):
            self.world.integrate()
        t.updateGeometry(True)
        self.assertLessEqual(t.getLength(), 0.9 + 1e-6)
        self.assertEqual(len(t.getVisualSegments()), 1)
        self.assertIs(self.world.getTendon("cable"), t)
        self.assertIs(self.world.getTendons()[0], t)
        t.setName("renamed")
        self.assertIsNone(self.world.getTendon("cable"))
        self.assertIs(self.world.getTendon("renamed"), t)

    def test_compression_spring_force_and_xml(self):
        self.body.setPosition(0.9, 0.0, 0.0)
        p = r.Tendon.Properties()
        p.stiffness = 100.0
        p.springLower, p.springUpper = 1.0, math.inf
        p.color = [0.2, 0.4, 0.6, 1.0]
        t = self.cable(properties=p)
        self.world.integrate()
        self.assertAlmostEqual(self.body.getLinearVelocity()[0], 0.01 / 1.0001, places=10)
        self.assertLess(t.getTension(), 0.0)
        with tempfile.TemporaryDirectory() as folder:
            self.world.exportToXml(folder, "world.xml")
            xml = Path(folder, "world.xml")
            self.assertNotIn("<wire", xml.read_text())
            restored = r.World(str(xml))
            self.assertEqual(restored.getTendon("cable").getProperties().springUpper, math.inf)
            restored.integrate()
        self.body.setPosition(1.1, 0.0, 0.0)
        self.body.setLinearVelocity(np.zeros(3))
        self.world.integrate()
        self.assertAlmostEqual(t.getTension(), 0.0, places=10)

    def test_tension_coupling_and_dependency_removal(self):
        t = self.cable()
        t.setTension(4.0)
        self.world.integrate()
        self.assertAlmostEqual(t.getActuationForce(), -4.0)
        self.assertAlmostEqual(self.body.getLinearVelocity()[0], -0.004)
        path, site = r.Tendon.PathElement, r.Tendon.Site
        other = self.world.addSpatialTendon("other", [path.via(site()), path.via(site(position=[2., 0., 0.]))])
        properties = r.TendonCoupling.Properties()
        properties.coefficients = [0., 2., 0., 0., 0.]
        coupling = self.world.addTendonCoupling("ratio", t, other, properties)
        self.assertIs(coupling.getFirst(), t)
        self.assertEqual(len(self.world.getTendonCouplings()), 1)
        self.world.removeObject(self.body)
        self.assertIsNone(self.world.getTendon("cable"))
        self.assertEqual(len(self.world.getTendonCouplings()), 0)
        self.world.removeTendon(other)
        self.assertEqual(len(self.world.getTendons()), 0)

    def test_heightmap_binding_accepts_current_const_accessor(self):
        # The unified package rebuild also compiles the current const terrain API.
        heights = [0., 0., 0., 0., 0.2, 0., 0., 0., 0.]
        terrain = self.world.addHeightMap(3, 3, 2., 2., 5., 5., heights)
        self.assertEqual(terrain.getHeightMap(), heights)

    def test_path_factories_and_drive_validation(self):
        path, site = r.Tendon.PathElement, r.Tendon.Site
        cylinder = path.cylinder(site(), 0.5, [0., 1., 0.]).withSideSite(site(position=[0., 0., 1.]))
        self.assertTrue(cylinder.hasSideSite)
        self.assertEqual(path.pulley(2.).divisor, 2.)
        self.assertEqual(path.sphere(site(), 0.4).radius, 0.4)
        t = self.cable()
        d = r.Tendon.Drive()
        d.targetLength = 1.0
        d.positionGain = 20.
        t.setDrive(d)
        d.positionGain = -1.
        with self.assertRaises(ValueError):
            t.setDrive(d)
        self.assertEqual(t.getDrive().positionGain, 20.)
        t.setEnabled(False)
        self.assertFalse(t.isEnabled())


if __name__ == "__main__":
    unittest.main()
