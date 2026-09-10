# URDF Collision Mesh Planning Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace hand-authored collision points with sampled URDF collision meshes while preserving the existing ESDF planner interface.

**Architecture:** Add a small mesh-loading/sampling component to `mm_config`. `MMConfig` owns per-link local samples, transforms them through its existing FK, and feeds the resulting points to the current ESDF and self-collision checks. RViz receives the same transformed samples through a diagnostic MarkerArray.

**Tech Stack:** ROS Noetic, C++, URDF parser, Assimp/resource retrieval already available to RViz/URDF tooling, Eigen, catkin, Python unit tests for pure sampling helpers.

**Spec:** `docs/superpowers/specs/2026-09-09-urdf-collision-mesh-design.md`

## Global Constraints

- `collision_model_source: urdf_mesh` must not silently fall back to the legacy model.
- `collision_mesh_sample_resolution: 0.03` is the default.
- Planning continues to use the existing ESDF and `MMConfig::checkcollision()` API.
- Coordinates remain in the existing `world`/link TF convention.

### Task 1: Define testable mesh sampling and configuration behavior

**Files:**
- Create: `tests/test_urdf_collision_mesh.py`
- Modify: `remani_planner/plan_manage/config/mm_param_ir100_cr10.yaml`

**Interfaces:**
- Tests define the expected deterministic surface sampler behavior and configuration defaults.

- [ ] **Step 1: Write the failing tests** for deterministic triangle sampling, empty-mesh rejection, and the configured `0.03` resolution.
- [ ] **Step 2: Run the tests** and verify they fail because the mesh sampling helper/configuration is absent.
- [ ] **Step 3: Add only the configuration keys** and a pure helper API used by the C++ implementation.
- [ ] **Step 4: Run the tests** and verify the sampling/configuration cases pass.
- [ ] **Step 5: Commit** the tests and configuration.

### Task 2: Load and sample URDF collision meshes

**Files:**
- Create: `remani_planner/mm_config/include/mm_config/urdf_collision_model.hpp`
- Create: `remani_planner/mm_config/src/urdf_collision_model.cpp`
- Modify: `remani_planner/mm_config/CMakeLists.txt`
- Test: `tests/test_urdf_collision_mesh.py`

**Interfaces:**
- `UrdfCollisionModel::load(const std::string& robot_description, double resolution)` returns `bool` and an error string.
- `UrdfCollisionModel::linkSamples(const std::string& link) const` returns local `std::vector<Eigen::Vector3d>` samples.
- `UrdfCollisionModel::linkNames() const` returns loaded collision-link names.

- [ ] **Step 1: Extend the failing tests** to require non-empty samples for all CR10 collision links and failure for unresolved mesh resources.
- [ ] **Step 2: Run tests** and verify the new cases fail.
- [ ] **Step 3: Implement URDF parsing, `package://` resolution, mesh loading, and deterministic triangle surface sampling at the configured spacing.
- [ ] **Step 4: Link the component in `mm_config` and run the focused tests/build.**
- [ ] **Step 5: Commit** the mesh loader and tests.

### Task 3: Replace MMConfig collision checks and publish diagnostics

**Files:**
- Modify: `remani_planner/mm_config/include/mm_config/mm_config.hpp`
- Modify: `remani_planner/mm_config/src/mm_config.cpp`
- Modify: `remani_planner/mm_config/CMakeLists.txt`
- Test: `tests/test_urdf_collision_mesh.py`

**Interfaces:**
- `MMConfig::loadCollisionModel(ros::NodeHandle&)` initializes the model once.
- `MMConfig::getTransformedCollisionSamples(...)` returns per-link world samples.
- Existing `checkcollision(...)` signatures remain unchanged.

- [ ] **Step 1: Add failing tests** for translated/rotated sample coordinates, obstacle collision, and self-collision link separation.
- [ ] **Step 2: Run tests** and verify they fail against the legacy point model.
- [ ] **Step 3: Implement initialization from `robot_description`, FK transformation for each link, continuous link sampling, and ESDF checks using mesh samples.
- [ ] **Step 4: Preserve car/manipulator collision categories and publish a `visualization_msgs/MarkerArray` diagnostic topic using the same transformed samples.
- [ ] **Step 5: Run focused tests and `catkin_make --pkg mm_config remani_planner`.
- [ ] **Step 6: Commit** the collision integration.

### Task 4: Runtime verification in the empty-map bar scenario

**Files:**
- Modify: `remani_planner/plan_manage/launch/exp0.rviz`
- Create: `tools/verify_urdf_collision_runtime.py`

**Interfaces:**
- The verifier subscribes to the diagnostic MarkerArray and TF, and reports frame/link alignment and collision status.

- [ ] **Step 1: Add a failing runtime check** requiring all collision links to have current `world` samples.
- [ ] **Step 2: Run it against the current launch** and record the expected failure from the legacy model.
- [ ] **Step 3: Enable the diagnostic display and implement the runtime checker.
- [ ] **Step 4: Start `run_ir100_cr10_empty_map.sh` without rebuilding, move the base through clear and intersecting poses, and verify SAFE/COLLISION transitions.
- [ ] **Step 5: Capture RViz evidence and run the full focused test suite.
- [ ] **Step 6: Commit** the runtime verifier and RViz configuration.
