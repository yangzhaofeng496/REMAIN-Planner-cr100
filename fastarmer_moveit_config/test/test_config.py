#!/usr/bin/env python3
"""Static checks for the FastArmer MoveIt kinematics configuration."""

import sys
import xml.etree.ElementTree as ET


def main():
    if len(sys.argv) != 3:
        print("usage: test_config.py URDF SRDF", file=sys.stderr)
        return 2

    urdf = ET.parse(sys.argv[1]).getroot()
    srdf = ET.parse(sys.argv[2]).getroot()
    assert urdf.tag == "robot" and urdf.get("name") == "FastArmer"
    links = {node.get("name") for node in urdf.findall("link")}
    joints = {node.get("name") for node in urdf.findall("joint")}
    expected_links = {"base_link", "link1", "link2", "link3", "link4", "link5", "link6"}
    expected_joints = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"}
    assert expected_links <= links, (expected_links - links)
    assert expected_joints <= joints, (expected_joints - joints)

    assert srdf.tag == "robot" and srdf.get("name") == "FastArmer"
    group = next((g for g in srdf.findall("group") if g.get("name") == "arm"), None)
    assert group is not None
    chain = group.find("chain")
    assert chain is not None
    assert chain.get("base_link") == "base_link"
    assert chain.get("tip_link") == "link6"
    state = srdf.find("group_state[@group='arm']")
    assert state is not None
    assert {j.get("name") for j in state.findall("joint")} == expected_joints
    print("FastArmer static configuration checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
