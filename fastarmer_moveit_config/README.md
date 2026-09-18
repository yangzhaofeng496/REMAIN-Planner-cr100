# FastArmer MoveIt kinematics

This package exposes the existing FastArmer URDF to ROS1 MoveIt for forward
and inverse kinematics. The planning group is `arm`, with `base_link` as its
base and `arm_gripper_link` as its terminal link (the URDF has no `arm_gripper_link`).

## Start

Source the workspace, then run:

```bash
roslaunch fastarmer_moveit_config kinematics_demo.launch
```

The launch file loads `robot_description`, the semantic description, and the
KDL solver configuration. Start the demo node in another terminal.

## Forward kinematics

`joint_values` is an array of six joint values in radians, ordered
`joint1` through `joint6`:

```bash
rosrun fastarmer_moveit_config kinematics_demo_node \
  _mode:=fk _joint_values:=[0, 0, 0, 0, 0, 0]
```

The node prints the `arm_gripper_link` position in metres and roll/pitch/yaw in radians.

## Inverse kinematics

`target_pose` is `[x, y, z, roll, pitch, yaw]`, with metres and radians:

```bash
rosrun fastarmer_moveit_config kinematics_demo_node \
  _mode:=ik _joint_values:=[0, 0, 0, 0, 0, 0] \
  _target_pose:=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```

IK prints a six-joint solution, the achieved `arm_gripper_link` pose, and round-trip
errors. It exits nonzero if IK fails or the error exceeds `1e-4 m` or
`1e-3 rad`. The initial joint array supplies the IK seed.

## Static test

## Interactive planning in RViz

Start the MoveIt planning scene and RViz MotionPlanning panel:

```bash
roslaunch fastarmer_moveit_config interactive_planning.launch
```

Select the `arm` group in the MotionPlanning panel. The end-effector marker
is attached to `arm_gripper_link`; drag it to set a goal and press **Plan**. Trajectory
execution is deliberately disabled because no real controller is configured.

When configured with testing enabled, CTest runs the XML checks:

```bash
catkin_make --cmake-args -DCATKIN_ENABLE_TESTING=ON
ctest --test-dir build --output-on-failure -R fastarmer_moveit_config_static
```
