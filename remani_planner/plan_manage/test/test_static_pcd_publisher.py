#!/usr/bin/env python3
import importlib.util
import pathlib
import tempfile
import unittest


SCRIPT = pathlib.Path(__file__).parents[1] / "scripts" / "static_pcd_publisher.py"
spec = importlib.util.spec_from_file_location("static_pcd_publisher", SCRIPT)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class StaticPcdPublisherTest(unittest.TestCase):
    def test_reads_xyz_ascii_without_transform(self):
        with tempfile.NamedTemporaryFile(mode="w", suffix=".pcd") as pcd:
            pcd.write("""# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH 2\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS 2\nDATA ascii\n1 2 3\n4 5 6\n""")
            pcd.flush()
            points = module.load_xyz_points(pcd.name)
        self.assertEqual(points, [(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)])

    def test_default_frame_is_world(self):
        self.assertEqual(module.DEFAULT_FRAME, "world")

    def test_voxel_downsample_disabled_returns_all_points(self):
        points = [(0.0, 0.0, 0.0), (0.001, 0.0, 0.0)]
        self.assertEqual(module.voxel_downsample(points, 0.0), points)

    def test_voxel_downsample_merges_into_centroid(self):
        points = [(0.0, 0.0, 0.0), (0.02, 0.0, 0.0), (0.04, 0.0, 0.0),
                  (1.0, 1.0, 1.0)]
        result = module.voxel_downsample(points, 0.1)
        self.assertEqual(result, [(0.02, 0.0, 0.0), (1.0, 1.0, 1.0)])

    def test_voxel_downsample_is_order_preserving_and_deterministic(self):
        points = [(0.05, 0.0, 0.0), (0.0, 0.0, 0.0), (0.2, 0.0, 0.0)]
        self.assertEqual(module.voxel_downsample(points, 0.1),
                         module.voxel_downsample(points, 0.1))

    def test_apply_transform_identity_is_noop(self):
        points = [(1.0, 2.0, 3.0)]
        self.assertEqual(module.apply_transform(points), points)

    def test_apply_transform_translation(self):
        result = module.apply_transform([(1.0, 2.0, 3.0)], translation=(0.5, -0.5, 1.0))
        self.assertEqual(result, [(1.5, 1.5, 4.0)])

    def test_apply_transform_yaw_90(self):
        result = module.apply_transform([(1.0, 0.0, 0.0)], rpy=(0.0, 0.0, 1.5707963267948966))
        self.assertAlmostEqual(result[0][0], 0.0, places=6)
        self.assertAlmostEqual(result[0][1], 1.0, places=6)
        self.assertAlmostEqual(result[0][2], 0.0, places=6)

    def test_vector_param_rejects_wrong_length(self):
        class FakeRospy(object):
            @staticmethod
            def get_param(_name, _default):
                return [1.0, 2.0]

        with self.assertRaises(ValueError):
            module._vector_param(FakeRospy, "~T_world_cloud/translation", [0.0, 0.0, 0.0])


if __name__ == "__main__":
    unittest.main()
