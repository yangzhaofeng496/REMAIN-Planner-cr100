# FastArmer MoveIt Kinematics Integration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a standalone ROS1 Noetic MoveIt package exposing FastArmer FK and IK.

**Architecture:** Reuse the existing FastArmer URDF and meshes through `robot_description`. Configure an `arm` MoveIt group and KDL kinematics plugin, then provide a small roscpp demo node with FK/IK modes; leave the existing planner untouched.

**Tech Stack:** ROS1 Noetic, catkin, MoveIt, KDL kinematics, C++11, GTest.

**Spec:** `docs/superpowers/specs/2026-09-16-fastarmer-moveit-kinematics-design.md`

## Global Constraints

- Use existing `FastArmer.urdf` and `arm_gripper_link`.
- Keep MoveIt integration standalone; do not modify existing planner collision logic.
- IK output must be checked by FK with a documented pose-error tolerance.

### Task 1: Package and MoveIt configuration

**Files:**
- Create: `fastarmer_moveit_config/package.xml`
- Create: `fastarmer_moveit_config/CMakeLists.txt`
- Create: `fastarmer_moveit_config/config/kinematics.yaml`
- Create: `fastarmer_moveit_config/config/fastarmer.srdf`
- Create: `fastarmer_moveit_config/launch/kinematics_demo.launch`

- [ ] Add package dependencies for `roscpp`, `rospy`, `urdf`, `moveit_core`, `moveit_ros_planning`, `kdl_kinematics_plugin`, and `joint_state_publisher`.
- [ ] Define the six-joint `arm` group from `base_link` to `arm_gripper_link` in SRDF.
- [ ] Configure `KDLKinematicsPlugin` with `base_link`, `arm_gripper_link`, and a 5-second timeout.
- [ ] Load URDF into `robot_description` from the existing package resource and load SRDF/kinematics parameters.

### Task 2: FK/IK demo node

**Files:**
- Create: `fastarmer_moveit_config/src/kinematics_demo.cpp`
- Modify: `fastarmer_moveit_config/CMakeLists.txt`

- [ ] Write a node that loads `RobotModel`, creates `RobotState`, and validates the `arm` joint model group.
- [ ] Implement FK using `RobotState::setJointGroupPositions` and `getGlobalLinkTransform`.
- [ ] Implement IK using `RobotState::setFromIK` with a target `Eigen::Isometry3d`.
- [ ] Parse six joint values and target pose parameters, print results, and return nonzero on invalid input or IK failure.
- [ ] For IK, recompute FK and reject solutions whose translation error exceeds `1e-4 m` or rotation error exceeds `1e-3 rad`.

### Task 3: Tests and documentation

**Files:**
- Create: `fastarmer_moveit_config/test/test_config.py`
- Create: `fastarmer_moveit_config/README.md`

- [ ] Test the URDF/SRDF text for expected links, joints, group name, and end-effector.
- [ ] Add CTest invocation for the static configuration test.
- [ ] Document FK/IK launch commands, parameter formats, and expected output.
- [ ] Run XML/YAML validation, package discovery, build, and FK/IK smoke checks where MoveIt is installed.
