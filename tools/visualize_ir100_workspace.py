#!/usr/bin/env python3
"""Visualize the IR100/CR10 end-effector workspace from its URDF.

The plot is the theoretical kinematic workspace.  Collision filtering against
the locomotive PCD is intentionally optional and is not performed by this
small standalone probe.
"""
import argparse
import subprocess
import xml.etree.ElementTree as ET

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
import PyKDL


def load_model(path):
    if path.endswith(".xacro"):
        xml = subprocess.check_output(["xacro", path], text=True)
    else:
        with open(path, "r", encoding="utf-8") as f:
            xml = f.read()
    return ET.fromstring(xml)


def _vec(text, default):
    return [float(x) for x in (text or default).split()]


def make_chain(root, tip_name="Link6"):
    links = {x.attrib["name"] for x in root.findall("link")}
    joints = {}
    children = set()
    for node in root.findall("joint"):
        parent = node.find("parent").attrib["link"]
        child = node.find("child").attrib["link"]
        origin = node.find("origin")
        xyz = _vec(origin.attrib.get("xyz") if origin is not None else None, "0 0 0")
        rpy = _vec(origin.attrib.get("rpy") if origin is not None else None, "0 0 0")
        axis_node = node.find("axis")
        axis = _vec(axis_node.attrib.get("xyz") if axis_node is not None else None, "1 0 0")
        limit = node.find("limit")
        lower = float(limit.attrib.get("lower", -np.pi)) if limit is not None else -np.pi
        upper = float(limit.attrib.get("upper", np.pi)) if limit is not None else np.pi
        joints[child] = (node, parent, xyz, rpy, axis, lower, upper)
        children.add(child)
    root_link = next(iter(links - children))
    tip = tip_name if tip_name in links else next(link for link in links if link not in {x[1] for x in joints.values()})
    chain_data = []
    current = tip
    while current != root_link:
        item = joints[current]
        chain_data.append(item)
        current = item[1]
    chain = PyKDL.Chain()
    chain_data.reverse()
    limits = []
    for node, parent, xyz, rpy, axis, lower, upper in chain_data:
        joint_type = node.attrib.get("type", "revolute")
        if joint_type == "fixed":
            joint = PyKDL.Joint(node.attrib["name"])
        elif joint_type == "prismatic":
            joint = PyKDL.Joint(node.attrib["name"], PyKDL.Vector(0, 0, 0), PyKDL.Vector(*axis), PyKDL.Joint.TransAxis)
        else:
            joint = PyKDL.Joint(node.attrib["name"], PyKDL.Vector(0, 0, 0), PyKDL.Vector(*axis), PyKDL.Joint.RotAxis)
        frame = PyKDL.Frame(PyKDL.Rotation.RPY(*rpy), PyKDL.Vector(*xyz))
        chain.addSegment(PyKDL.Segment(current, joint, frame))
        if joint_type != "fixed":
            limits.append((lower, upper))
        current = node.find("parent").attrib["link"]
    return chain, limits


def fk_workspace(root, samples, seed, tip_name):
    chain, limits = make_chain(root, tip_name)
    rng = np.random.default_rng(seed)
    q = PyKDL.JntArray(chain.getNrOfJoints())
    solver = PyKDL.ChainFkSolverPos_recursive(chain)
    points = []
    for _ in range(samples):
        for i, (lower, upper) in enumerate(limits):
            q[i] = rng.uniform(lower, upper)
        frame = PyKDL.Frame()
        if solver.JntToCart(q, frame) < 0:
            continue
        points.append([frame.p.x(), frame.p.y(), frame.p.z()])
    return np.asarray(points)


def zero_pose_skeleton(root, tip_name):
    chain, limits = make_chain(root, tip_name)
    q = PyKDL.JntArray(chain.getNrOfJoints())
    for i in range(chain.getNrOfJoints()):
        q[i] = 0.0
    solver = PyKDL.ChainFkSolverPos_recursive(chain)
    points = [[0.0, 0.0, 0.0]]
    for i in range(1, chain.getNrOfSegments() + 1):
        frame = PyKDL.Frame()
        if solver.JntToCart(q, frame, i) >= 0:
            points.append([frame.p.x(), frame.p.y(), frame.p.z()])
    return np.asarray(points)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", default="local_robot/ir100_description/urdf/ir100_robot.xacro")
    parser.add_argument("--samples", type=int, default=50000)
    parser.add_argument("--tip", default="Link6")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--output", default="/tmp/ir100_workspace.png")
    args = parser.parse_args()

    robot = load_model(args.urdf)
    points = fk_workspace(robot, args.samples, args.seed, args.tip)
    skeleton = zero_pose_skeleton(robot, args.tip)
    if len(points) == 0:
        raise RuntimeError("no valid FK samples")
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")
    ax.scatter(points[:, 0], points[:, 1], points[:, 2], s=1, alpha=0.18, c=points[:, 2], cmap="viridis")
    ax.plot(skeleton[:, 0], skeleton[:, 1], skeleton[:, 2], "-o", color="black", linewidth=3,
            markersize=4, label="URDF zero pose")
    ax.scatter([0], [0], [0], c="red", s=55, label="URDF base origin")
    ax.text(0.02, 0.02, 0.02, "O (URDF base)", color="red")
    ax.scatter([skeleton[-1, 0]], [skeleton[-1, 1]], [skeleton[-1, 2]],
               c="black", s=45, label=args.tip)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title("IR100 theoretical end-effector workspace")
    ax.legend()
    plt.tight_layout()
    plt.savefig(args.output, dpi=180)
    print("samples={} output={}".format(len(points), args.output))
    plt.show()


if __name__ == "__main__":
    main()
