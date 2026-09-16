# Coupled IK Waypoint Trajectory Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the locomotive planner execute a collision-checked joint trajectory that passes through the selected Cartesian end-effector samples after IK conversion and interpolation.

**Architecture:** Build a time-layered set of arm joint candidates, where each candidate is generated from a Cartesian end-effector sample and accepted only after full coupled collision checking. Connect candidates in adjacent base-trajectory layers with interpolated joint states and collision checks; preserve accepted joint waypoints as hard constraints when constructing the backend trajectory. Publish candidate, planned, and actual end-effector paths separately.

**Tech Stack:** ROS Noetic, C++14, Eigen, KDL IK, PCL/URDF collision meshes, catkin, RViz.

**Spec:** Chat-approved requirement: each base trajectory point gets Cartesian arm sampling; every IK result must pass full collision checking; consecutive joint solutions are interpolated and checked; the executed trajectory must preserve the IK joint waypoints.

## Global Constraints

- The fixed locomotive PCD scene uses `/home/hyf/REMAIN-Planner-cr100/scansdibu_xyzRemove_filtered_10cm.pcd`.
- All collision acceptance checks use `MMConfig::checkcollision(..., false, coll_type)`.
- Candidate samples, optimized trajectory, and measured trajectory must use the `world` frame.
- A trajectory is successful only if final high-precision safety passes and execution reaches `WAIT_TARGET` without collision emergency stop.
- Do not remove the existing general-scene fallback; make locomotive behavior configurable.

---

### Task 1: Add a testable layered waypoint data model

**Files:**
- Modify: `remani_planner/path_searching/include/path_searching/sample_mani_RRT.h`
- Modify: `remani_planner/path_searching/src/sample_mani_RRT.cpp`
- Test: `remani_planner/path_searching/test/layered_waypoint_test.cpp`
- Modify: `remani_planner/path_searching/CMakeLists.txt`

**Interfaces:**
- Add `struct LayeredManiWaypoint { int layer; Eigen::Vector3d ee_world; Eigen::VectorXd joint_state; };`.
- Add a pure helper `bool interpolateJointSegment(const Eigen::VectorXd& q0, const Eigen::VectorXd& q1, int samples, std::vector<Eigen::VectorXd>& out);`.
- The helper must include both endpoints, reject mismatched dimensions or fewer than two samples, and produce linear joint interpolation.

- [ ] **Step 1: Write the failing test** asserting endpoint preservation, sample count, and midpoint interpolation.
- [ ] **Step 2: Run the test and verify it fails** because the helper is not implemented.
- [ ] **Step 3: Implement the helper** without ROS dependencies beyond the existing Eigen types.
- [ ] **Step 4: Run the focused test and the existing package build.**
- [ ] **Step 5: Commit** with message `test: define layered joint waypoint interpolation`.

### Task 2: Enforce full collision checking for every IK candidate

**Files:**
- Modify: `remani_planner/path_searching/src/sample_mani_RRT.cpp: initNode/getSampleNode`
- Modify: `remani_planner/mm_config/src/mm_config.cpp` only if collision diagnostics need a stable minimum-distance result
- Test: `remani_planner/path_searching/test/ik_candidate_acceptance_test.cpp`

**Interfaces:**
- `initNode()` remains the single node acceptance gate.
- It must call `checkcollision(car_state_list_[idx], q, false, collision_type)`, not only `checkManicollision()`.
- IK success must never directly insert a node; collision failure increments a per-type counter and causes resampling.

- [ ] **Step 1: Add a regression test** with a fake collision checker where IK succeeds but arm-cloud collision is true; assert that the candidate is rejected.
- [ ] **Step 2: Run the test and verify the expected failure.**
- [ ] **Step 3: Route every Cartesian IK result through `initNode()` and record `collision_type` counters.**
- [ ] **Step 4: Verify no direct `solveEndEffectorIK()` success path inserts a node without `initNode()`.**
- [ ] **Step 5: Build `path_searching` and commit** as `fix: require full collision gate for IK nodes`.

### Task 3: Generate candidates for each base trajectory layer

**Files:**
- Modify: `remani_planner/path_searching/include/path_searching/sample_mani_RRT.h`
- Modify: `remani_planner/path_searching/src/sample_mani_RRT.cpp`
- Modify: `remani_planner/plan_manage/config/remani_planner_param_manual.yaml`

**Interfaces:**
- Add parameters `search/cartesian_samples_per_layer`, `search/cartesian_sample_radius_xy`, `search/cartesian_sample_z_min`, `search/cartesian_sample_z_max`, and `search/enable_shared_posture_fast_path`.
- Add `std::vector<std::vector<ManiPathNodePtr>> layer_candidates_`.
- Add `bool sampleLayerCandidates(int layer, const Eigen::VectorXd& seed, std::vector<ManiPathNodePtr>& candidates)`.
- The method samples around the end-effector pose obtained from the layer-specific seed, runs IK, then calls the full `initNode()` gate.

- [ ] **Step 1: Add a deterministic unit test** proving a layer produces only full-collision-free candidates.
- [ ] **Step 2: Run it red.**
- [ ] **Step 3: Implement the per-layer candidate generator** with bounded retry count and explicit logs: `layer`, `ik_success`, `ik_failure`, `collision_type`, and accepted count.
- [ ] **Step 4: Disable the shared-posture and stationary-arm shortcuts for the fixed locomotive YAML.**
- [ ] **Step 5: Build and verify logs show multiple layers with accepted candidates.**
- [ ] **Step 6: Commit** as `feat: sample coupled arm candidates per base layer`.

### Task 4: Connect adjacent layers with hard joint interpolation

**Files:**
- Modify: `remani_planner/path_searching/src/sample_mani_RRT.cpp`
- Modify: `remani_planner/path_searching/include/path_searching/sample_mani_RRT.h`
- Test: `remani_planner/path_searching/test/layer_connection_test.cpp`

**Interfaces:**
- Add `bool connectLayerCandidates(int layer, const ManiPathNodePtr& from, const ManiPathNodePtr& to);`.
- Use the existing `t_list_` duration and joint velocity limits to choose interpolation density.
- Every interpolated state must call `checkcollision()` with the corresponding interpolated base pose from `car_state_list_check_`.
- Reject edges on collision, velocity violation, dimension mismatch, or missing candidate layers.

- [ ] **Step 1: Write tests** for a safe edge and an edge whose midpoint collides although endpoints are safe.
- [ ] **Step 2: Run tests red.**
- [ ] **Step 3: Implement edge interpolation and full collision checking.**
- [ ] **Step 4: Replace any connection path that checks only endpoints.**
- [ ] **Step 5: Verify the resulting path contains one joint waypoint per selected layer and all inserted intermediate states.**
- [ ] **Step 6: Commit** as `feat: connect IK candidates with collision-checked interpolation`.

### Task 5: Preserve joint waypoints through backend optimization

**Files:**
- Modify: `remani_planner/traj_opt/include/optimizer/poly_traj_optimizer.hpp`
- Modify: `remani_planner/traj_opt/src/poly_traj_optimizer.cpp`
- Modify: `remani_planner/plan_manage/src/planner_manager.cpp`
- Test: `remani_planner/traj_opt/test/hard_waypoint_test.cpp`

**Interfaces:**
- Add an explicit backend option `optimize/preserve_frontend_joint_waypoints`.
- Pass the accepted frontend joint path and its segment times into the optimizer.
- Build polynomial pieces so each frontend waypoint is a junction/constraint; do not replace all frontend points with only endpoint control points.
- After optimization, evaluate `traj.getPos(waypoint_time)` and require joint error below `1e-3` for every hard waypoint.

- [ ] **Step 1: Write a failing test** where an optimizer would otherwise move an interior joint waypoint; assert the hard-waypoint error bound.
- [ ] **Step 2: Run the test red.**
- [ ] **Step 3: Add waypoint-time bookkeeping and equality constraints at every preserved waypoint.**
- [ ] **Step 4: Keep velocity/acceleration optimization between waypoints, but reject or retry if equality constraints cannot be satisfied.**
- [ ] **Step 5: Add the post-optimization waypoint validation before publishing a trajectory.**
- [ ] **Step 6: Build `traj_opt` and `remani_planner`, then commit** as `feat: preserve joint IK waypoints in backend trajectory`.

### Task 6: Publish candidate, planned, and actual end-effector paths

**Files:**
- Modify: `remani_planner/path_searching/src/sample_mani_RRT.cpp`
- Modify: `remani_planner/plan_manage/include/plan_manage/remani_replan_fsm.h`
- Modify: `remani_planner/plan_manage/src/remani_replan_fsm.cpp`
- Modify: `remani_planner/plan_manage/src/planner_manager.cpp`
- Modify: the RViz config used by `exp0_ir100_cr10_pcd.launch`

**Interfaces:**
- Publish `/remani_planner/cartesian_ik_samples` for candidates.
- Publish `/remani_planner/planned_ee_path` as `nav_msgs/Path` from the final polynomial trajectory.
- Publish `/remani_planner/actual_ee_path` as `nav_msgs/Path`, computing FK from the live joint state `mm_state_pos_.tail(manipulator_dim_)` and transforming with the live base pose.
- Use distinct colors: green candidates, blue planned path, red actual path.

- [ ] **Step 1: Add a visualization test** for frame id, color/topic separation, and FK point publication.
- [ ] **Step 2: Run it red.**
- [ ] **Step 3: Publish planned samples at 50–100 Hz-equivalent trajectory resolution.**
- [ ] **Step 4: Append actual FK samples from the odometry/joint-state callback at the existing timer rate.**
- [ ] **Step 5: Add throttled logs of planned-vs-actual EE distance and joint distance.**
- [ ] **Step 6: Build and commit** as `feat: visualize planned and measured end-effector paths`.

### Task 7: Fix static PCD map timing and execution validation

**Files:**
- Modify: `remani_planner/plan_manage/launch/exp0_ir100_cr10_pcd.launch`
- Modify: `remani_planner/plan_env/include/plan_env/grid_map.h`
- Modify: `remani_planner/plan_env/src/grid_map.cpp`
- Modify: `remani_planner/plan_manage/src/remani_replan_fsm.cpp`

- [ ] **Step 1: Add a map-ready accessor** such as `bool isGlobalMapReady() const` backed by the received PCD cloud state.
- [ ] **Step 2: Prevent goal planning until the static global map is ready.**
- [ ] **Step 3: Set `global_map_refresh=false` for the fixed static PCD launch so the ESDF does not change during execution.**
- [ ] **Step 4: Log map readiness before accepting the goal.**
- [ ] **Step 5: Build and commit** as `fix: stabilize static PCD map before planning`.

### Task 8: End-to-end verification with the saved goal

**Files:**
- Verify: `remani_planner/plan_manage/config/pcd_test_goal.yaml`
- Verify: `remani_planner/plan_manage/launch/exp0_ir100_cr10_pcd.launch`

- [ ] **Step 1: Build in the Docker ROS workspace.**

```bash
docker exec remani_pcd_demo bash -lc 'cd /opt/remani_ws && source /opt/ros/noetic/setup.bash && catkin_make -j2'
```

- [ ] **Step 2: Restart the PCD container and wait for `/map_generator/global_cloud`.**
- [ ] **Step 3: Publish the saved goal.**

```bash
docker exec remani_pcd_demo bash -lc 'source /opt/ros/noetic/setup.bash && rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped "{header: {frame_id: world}, pose: {position: {x: 2.878, y: -3.411, z: 0.0}, orientation: {z: -0.7142136581, w: 0.6999277467}}}"'
```

- [ ] **Step 4: Verify logs contain** per-layer candidate counts, full collision rejection counts, hard-waypoint validation success, final high-precision safety success, and `reach goal`.
- [ ] **Step 5: Verify logs contain no** `CollisionWatch COLLISION`, `EMERGENCY_STOP`, or planned/actual EE divergence beyond configured tolerance.
- [ ] **Step 6: Inspect RViz:** red actual path must track blue planned path; blue path must pass the hard IK waypoint markers within tolerance.
- [ ] **Step 7: If verification fails, preserve the complete log and classify the failure as waypoint constraint, collision model, coordinate transform, or controller tracking before changing code.**

