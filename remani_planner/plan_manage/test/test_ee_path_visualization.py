#!/usr/bin/env python3
"""Checks the locomotive RViz layout separates candidate, planned and
measured end-effector visualizations with the documented colors."""
import pathlib
import unittest

import yaml

RVIZ = pathlib.Path(__file__).parents[1] / "launch" / "exp0.rviz"

CANDIDATE_TOPIC = "/remani_planner/cartesian_ik_samples"
PLANNED_TOPIC = "/remani_planner/planned_ee_path"
ACTUAL_TOPIC = "/remani_planner/actual_ee_path"
FRONTEND_TOPIC = "/remani_planner_node/kinoastar/ee_path_nav"


def collect_displays(node):
    displays = []
    for display in node.get("Displays", []):
        displays.append(display)
        displays.extend(collect_displays(display))
    return displays


class EePathVisualizationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.root = yaml.safe_load(RVIZ.read_text())
        cls.manager = cls.root["Visualization Manager"]
        cls.displays = collect_displays(cls.manager)

    def test_fixed_frame_is_world(self):
        self.assertEqual(self.manager["Global Options"]["Fixed Frame"], "world")

    def test_path_topics_are_separate(self):
        topics = [d.get("Topic") for d in self.displays if d.get("Class") == "rviz/Path"]
        self.assertIn(PLANNED_TOPIC, topics)
        self.assertIn(ACTUAL_TOPIC, topics)
        self.assertIn(FRONTEND_TOPIC, topics)

    def test_planned_path_is_blue(self):
        planned = next(d for d in self.displays
                       if d.get("Class") == "rviz/Path" and d.get("Topic") == PLANNED_TOPIC)
        self.assertEqual(planned["Color"].replace(" ", ""), "0;0;255")

    def test_actual_path_is_red(self):
        actual = next(d for d in self.displays
                      if d.get("Class") == "rviz/Path" and d.get("Topic") == ACTUAL_TOPIC)
        self.assertEqual(actual["Color"].replace(" ", ""), "255;0;0")

    def test_candidate_marker_topic_present(self):
        marker_topics = [d.get("Marker Topic") for d in self.displays
                         if d.get("Class") == "rviz/Marker"]
        self.assertIn(CANDIDATE_TOPIC, marker_topics)


if __name__ == "__main__":
    unittest.main()
