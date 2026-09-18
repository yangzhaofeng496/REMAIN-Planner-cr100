#!/usr/bin/env python3
import pathlib
import unittest

import yaml


ROOT = pathlib.Path(__file__).parents[2]
MM_CONFIG = ROOT / "mm_config" / "src" / "mm_config.cpp"
PARAMS = pathlib.Path(__file__).parents[1] / "config" / "remani_planner_param_manual.yaml"


class Link3Link5SelfCollisionTest(unittest.TestCase):
    def test_link3_link5_pair_is_not_skipped(self):
        source = MM_CONFIG.read_text()
        self.assertNotIn("skipTemporarySelfPair(names[i], names[j])", source)

    def test_self_collision_margin_is_small(self):
        params = yaml.safe_load(PARAMS.read_text())
        self.assertEqual(params["optimization"]["self_safe_margin"], 0.01)


if __name__ == "__main__":
    unittest.main()
