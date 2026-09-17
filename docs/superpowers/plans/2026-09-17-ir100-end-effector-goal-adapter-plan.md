# IR100 End-Effector Goal Adapter Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans (recommended). Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a decoupled ROS1 end-effector target node and empty-scene launch path that feeds the existing REMAIN-Planner without taking over planning or motor control.

**Architecture:** A small `ir100_goal_adapter` package validates and latches `PoseStamped` targets, publishes a planner-facing goal, and reports status. REMAIN-Planner and `mm_controller` remain unchanged in responsibility and communicate through their existing ROS topics.

**Tech Stack:** ROS Noetic, catkin, C++14, `geometry_msgs`, `diagnostic_msgs`, `std_srvs`, `std_msgs`.

**Spec:** `docs/superpowers/specs/2026-09-17-ir100-end-effector-goal-adapter-design.md`

## Global Constraints

- The adapter never publishes `/MMctrl/car_cmd`.
- Empty-scene validation disables PCD corridor dependence.
- Target input and planner output are separate topics.
- Invalid targets must not reach the planner goal topic.

---

### Task 1: Add the package skeleton and validation tests

**Files:**
- Create: `remani_planner/ir100_goal_adapter/package.xml`
- Create: `remani_planner/ir100_goal_adapter/CMakeLists.txt`
- Create: `remani_planner/ir100_goal_adapter/include/ir100_goal_adapter/target_validator.hpp`
- Create: `remani_planner/ir100_goal_adapter/test/target_validator_test.cpp`

**Interfaces:**
- `bool validateTarget(const geometry_msgs::PoseStamped&, const std::string&, double, std::string*)` rejects wrong/empty frame, non-finite XYZ, and XYZ outside configured bounds.

- [ ] Write a failing gtest for valid target acceptance and invalid frame/non-finite rejection.
- [ ] Run the focused test and confirm it fails because the validator is absent.
- [ ] Implement the validator and package dependencies.
- [ ] Run the focused test and confirm it passes.

### Task 2: Implement the ROS adapter node

**Files:**
- Create: `remani_planner/ir100_goal_adapter/src/ir100_goal_adapter_node.cpp`
- Modify: `remani_planner/ir100_goal_adapter/CMakeLists.txt`

**Interfaces:**
- Subscribe: `~target_topic` (`geometry_msgs/PoseStamped`).
- Publish: `~planner_goal_topic` (`geometry_msgs/PoseStamped`), `~status` (`diagnostic_msgs/DiagnosticArray`), `~state` (`std_msgs/String`, latched).
- Service: `~clear_target` (`std_srvs/Trigger`).

- [ ] Add the node with parameterized input/output topics and bounds.
- [ ] Normalize only the configured accepted frame; preserve the target pose in the adapter topic.
- [ ] Publish `RECEIVED`/`INVALID_TARGET`/`PLANNER_UNAVAILABLE` states and publish the planner goal only when valid.
- [ ] Build the package and run a roscore smoke test.

### Task 3: Add empty-scene launch and documentation

**Files:**
- Create: `remani_planner/ir100_goal_adapter/launch/empty_scene_goal_planning.launch`
- Modify: `README.md`
- Modify: `remani_planner/plan_manage/launch/exp0_ir100_cr10_manual.launch` only if launch arguments are required by the existing planner.

- [ ] Include the existing empty-scene planner launch without changing controller ownership.
- [ ] Start `ir100_goal_adapter` with explicit target and planner topics.
- [ ] Document one `rostopic pub` command for an XYZ target and expected status/topics.

### Task 4: Build and end-to-end empty-scene verification

**Files:**
- Verify all files from Tasks 1-3.

- [ ] Build the workspace with `catkin_make` in the project Docker environment.
- [ ] Start the empty-scene launch and verify adapter subscriptions/publications.
- [ ] Publish one finite target and verify exactly one planner goal.
- [ ] Verify `/planning/trajectory` contains a combined base/arm trajectory when the existing planner accepts the target.
- [ ] Verify the adapter has no `/MMctrl/car_cmd` publisher and record any planner-side rejection separately.
