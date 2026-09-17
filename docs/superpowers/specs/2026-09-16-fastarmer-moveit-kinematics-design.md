# FastArmer MoveIt Kinematics Integration Design

## Goal

Add a standalone ROS1 Noetic MoveIt configuration for the existing FastArmer URDF so users can compute forward kinematics and inverse kinematics without changing the current planner.

## Scope

- Reuse `remani_planner/mm_config/meshes/FastArmer/FastArmer.urdf` and its meshes.
- Create a MoveIt package with the `arm` planning group from `joint1` through `joint6` and end-effector link `link6` (the actual terminal link in the existing URDF).
- Provide launch files and a small command-line node for FK and IK demonstrations.
- Validate URDF loading, FK output, IK convergence, and IK round-trip pose error.
- Defer MoveIt-based collision replacement; the existing planner collision implementation remains unchanged.

## Architecture

The new package will own only MoveIt configuration and kinematics examples. The URDF will be exposed through `robot_description`, while MoveIt will construct a `RobotModel`/`RobotState` for FK and a `PlanningScene`-independent `KinematicsBase` solver for IK. The default solver will be KDL, configured through `kinematics.yaml`; callers can later switch plugins without changing the example node API.

## Interfaces

- Launch: `roslaunch fastarmer_moveit_config kinematics_demo.launch`
- Demo node parameters: `~mode` (`fk` or `ik`), `~joint_values`, and `~target_pose`.
- Output: end-effector pose for FK, or a six-joint solution plus achieved pose/error for IK.

## Testing

- Unit-level configuration checks verify the six expected joints, base link, and end-effector link.
- A launch-time smoke test loads the model and runs FK.
- IK is accepted only when the solver returns success and the achieved pose error is within a documented tolerance.

## Non-goals

- No change to the existing `remani_planner` collision detector.
- No automatic MoveIt RViz setup or trajectory execution controller in the first version.
