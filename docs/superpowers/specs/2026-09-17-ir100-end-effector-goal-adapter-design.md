# IR100 End-Effector Goal Adapter Design

## Goal

Add a decoupled ROS1 node that accepts an IR100 end-effector XYZ/Pose target and hands it to the existing REMAIN-Planner pipeline, which remains responsible for coupled mobile-base and arm planning and execution.

## Scope

The first milestone targets an empty scene. It keeps joint limits, base limits, arm self-collision, and arm/base collision, while disabling dependence on a PCD corridor. The adapter must not publish motor commands or duplicate the planner/controller.

## Interfaces

- Input: `geometry_msgs/PoseStamped` on `~target_topic` (default `/ir100/end_effector_target`), with XYZ required and orientation optional through a configurable default orientation.
- Trigger: `std_srvs/Trigger` on `~clear_target` to clear the latched target.
- Planner goal: `geometry_msgs/PoseStamped` on a configurable planner-facing topic (default `/move_base_simple/goal`) only after explicit target validation and projection to the planner's supported base-goal interface.
- Status: `diagnostic_msgs/DiagnosticArray` on `~status` and latched `std_msgs/String` on `~state`.

The adapter will expose the original end-effector target and its derived planner goal separately so consumers can replace the planner-facing bridge later without changing the input node.

## Responsibility boundaries

1. `ir100_goal_adapter`: input validation, frame normalization, target latching, status and planner-goal adaptation.
2. `remani_planner`: coupled base/arm search, IK candidates, collision checks, interpolation, optimization, and `/planning/trajectory` publication.
3. `mm_controller`: execution of `/planning/trajectory` and hardware/simulator feedback.

## Empty-scene launch

Add an adapter launch that includes the existing empty-scene planner launch and starts the adapter with explicit topic parameters. No code path will publish `/MMctrl/car_cmd` directly from the adapter.

## Failure behavior

Invalid frames or non-finite coordinates publish `INVALID_TARGET`; planner rejection or timeout is reported as `PLANNER_REJECTED`; missing planner subscribers is reported as `PLANNER_UNAVAILABLE`. No stale target is resent after a clear request.

## Verification

- Unit tests cover finite/range validation, frame preservation, target latching, and planner-goal publication.
- Catkin build succeeds for the adapter and existing planner packages.
- In the empty scene, publishing one target produces one planner goal and a `/planning/trajectory` message with the combined base/arm dimension; the adapter itself produces no motor command.
