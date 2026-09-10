# URDF Collision Mesh Planning Design

## Goal

Make mobile-manipulator obstacle and self-collision checks use the same URDF `<collision>` mesh geometry that RViz displays, sampled at a configurable resolution.

## Scope

- Read the existing `robot_description` URDF and resolve `package://` mesh paths.
- Load collision meshes for the CR10 arm and the mobile base.
- Sample mesh surfaces into local collision points at a configurable resolution.
- Transform those points with the existing `MMConfig` FK into `world` for ESDF checks.
- Keep the current ESDF interface and planner APIs unchanged.
- Replace the hand-authored manipulator point tables for obstacle and self-collision checks.
- Publish the sampled collision points for RViz diagnostics.

## Non-goals

- No FCL/Bullet dependency.
- No change to the planner state vector or trajectory optimizer interfaces.
- No automatic inference of visual-only geometry.

## Configuration

```yaml
collision_model_source: urdf_mesh
collision_mesh_sample_resolution: 0.03
collision_mesh_use_volume: false
```

The implementation must fail clearly if a required collision mesh cannot be resolved or parsed; it must not silently fall back to the old model when `urdf_mesh` is selected.

## Coordinate convention

Each mesh is sampled in its URDF link-local frame. The existing `MMConfig` joint transforms and fixed base transform are the sole transform chain used by planning. RViz and the planner therefore share `world`, `base_link`, `arm_world_link`, `arm_base_link`, `Link1`–`Link6`, and `arm_gripper_link` conventions.

## Acceptance criteria

1. A loaded CR10 collision mesh produces non-empty samples for every collision link.
2. A translated/rotated robot transforms those samples by the same FK used by `checkcollision()`.
3. A bar intersecting a sampled mesh is reported as an obstacle collision; the same pose clear of the mesh is safe.
4. Self-collision checks use distinct link sample sets and report link-link intersections.
5. RViz can display the sampled collision points and the URDF collision mesh at the same pose.
6. Existing planner packages compile without a full workspace rebuild.
