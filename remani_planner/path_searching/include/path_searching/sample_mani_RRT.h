#ifndef _SAMPLE_MANI_H_
#define _SAMPLE_MANI_H_

#include <ros/console.h>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>
#include <map>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <time.h>
#include <random>
#include <cstdint>
#include <array>
#include <functional>
#include "plan_env/grid_map.h"
#include "mm_config/mm_config.hpp"
#include "path_searching/rrt.h"
#include <fstream>
#include <visualization_msgs/MarkerArray.h>

namespace mani_sample {

  // One accepted arm configuration attached to a specific mobile-base
  // trajectory layer.  layer indexes car_state_list_; ee_world is the
  // Cartesian end-effector sample (world frame) that produced it through IK.
  struct LayeredManiWaypoint {
    int layer{0};
    Eigen::Vector3d ee_world{Eigen::Vector3d::Zero()};
    Eigen::VectorXd joint_state;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  // Pure linear interpolation between two joint configurations.  Always
  // includes both endpoints, returns exactly `samples` configurations, and
  // rejects mismatched dimensions or fewer than two requested samples.
  bool interpolateJointSegment(const Eigen::VectorXd &q0,
                               const Eigen::VectorXd &q1,
                               int samples,
                               std::vector<Eigen::VectorXd> &out);

  // Coupled collision checker: returns true when the given coupled base/arm
  // state is in collision and reports the collision type
  // (0 car, 1 arm, 2 arm-car, 3 arm-arm).
  using ManiCollisionFn = std::function<bool(const Eigen::Vector3d &car_state,
                                             const Eigen::VectorXd &joint_state,
                                             int &collision_type)>;

  // Single acceptance gate for a Cartesian IK candidate.  Returns true only
  // when IK succeeded and the complete coupled collision gate is clear.
  // Collision failures are tallied per collision type; IK failures are
  // tallied separately.  This never inserts a node: callers must resample
  // whenever it returns false.
  bool acceptIkCandidate(bool ik_ok,
                         const Eigen::Vector3d &car_state,
                         const Eigen::VectorXd &joint_state,
                         const ManiCollisionFn &collision,
                         int &collision_type,
                         std::array<size_t, 4> *collision_type_counts,
                         size_t *ik_failure_count);

  // Inverse-kinematics solver for a Cartesian end-effector target expressed
  // in the mobile-base frame.
  using ManiIkFn = std::function<bool(const Eigen::Vector3d &target_position,
                                      const Eigen::Matrix3d &target_rotation,
                                      const Eigen::VectorXd &seed,
                                      Eigen::VectorXd &solution)>;

  // One IK-converted arm configuration plus the Cartesian sample that
  // produced it.
  struct LayerIkCandidate {
    Eigen::Vector3d ee_position{Eigen::Vector3d::Zero()};
    Eigen::VectorXd joint_state;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  struct LayerIkStats {
    int ik_success{0};
    int ik_failure{0};
    int out_of_limits{0};
    int accepted{0};
  };

  // Deterministically sample `samples_per_layer` Cartesian end-effector
  // targets around `center` inside an axis-aligned box, convert each through
  // IK and keep the solutions inside the joint limits.  No ROS dependency
  // beyond Eigen, so it is unit-testable.
  bool sampleLayerIkCandidates(const Eigen::Vector3d &center,
                               const Eigen::Matrix3d &rotation,
                               const Eigen::VectorXd &ik_seed,
                               int manipulator_dof,
                               const Eigen::VectorXd &min_joint,
                               const Eigen::VectorXd &max_joint,
                               int samples_per_layer,
                               double radius_xy,
                               double z_min,
                               double z_max,
                               uint32_t rng_seed,
                               const ManiIkFn &ik,
                               std::vector<LayerIkCandidate> &out,
                               LayerIkStats &stats);

  // Mobile-base pose as a function of the normalized fraction along a joint
  // transition (0 = from-layer, 1 = to-layer).
  using ManiBasePoseFn = std::function<Eigen::Vector3d(double fraction)>;

  // Validate a joint transition between two adjacent base layers.  The
  // interpolation density is derived from the joint displacement, the
  // transition duration and the joint velocity limit.  Every interpolated
  // state is checked with `collision` using the mobile-base pose returned by
  // `base_pose_at`.  Returns false on dimension mismatch, velocity
  // violation, or any collision.
  bool checkJointTransition(const Eigen::VectorXd &q_from,
                            const Eigen::VectorXd &q_to,
                            double duration,
                            double max_joint_vel,
                            int min_samples,
                            const ManiBasePoseFn &base_pose_at,
                            const ManiCollisionFn &collision,
                            int &collision_type);

  class ManiPathNode{
    public:
    enum NODE_STATE
    {
      EXPAND,
      NOT_EXPAND,
      IN_TREE,
      IN_ANTI_TREE,
      COLLISION
    };

    int index;
    Eigen::VectorXd state;
    ManiPathNode* parent;
    double g_score;
    NODE_STATE node_state;
    std::map<string, ManiPathNode*> children;

    ManiPathNode(){
      parent = nullptr;
      node_state = NOT_EXPAND;
    } 
    ManiPathNode(int dof){
      parent = nullptr;
      node_state = NOT_EXPAND;
      state = Eigen::VectorXd::Zero(dof);
    }
    ~ManiPathNode(){};
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
  typedef ManiPathNode* ManiPathNodePtr;

  class SampleMani{

    private:
    ManiPathNodePtr end_node_;
    std::map<string, ManiPathNodePtr> node_pool_;
    std::vector<Eigen::Vector3d> car_state_list_;
    std::vector<Eigen::Vector3d> car_state_list_check_;
    std::vector<double> t_list_;

    
    bool have_path_;
    Eigen::VectorXd min_joint_pos_, max_joint_pos_;
    double max_joint_vel_, max_joint_acc_;
    double self_safe_margin_, safe_margin_mani_;
    double mobile_base_check_radius_;
    int manipulator_dof_, mobile_base_dof_, traj_dim_;
    double mani_thickness_;
    double angle_res_;
    int max_index_, tree_max_index_, tree_min_index_;
    int tree_count_, anti_tree_count_;
    int check_num_;
    double goal_rate_;
    double guided_sample_rate_{0.5};
    double local_sample_rate_{0.35};
    double local_sample_std_{0.35};
    Eigen::VectorXd sample_start_state_, sample_goal_state_;
    int max_loop_num_;
    bool enable_mani_oneshot_{true};
    int max_oneshot_calls_{0};
    int oneshot_stride_{1};
    int oneshot_max_jump_layers_{0}; // <=0 preserves unlimited ancestor search
    int oneshot_calls_{0};
    double max_mani_search_time_;
    // When enabled, allow the legacy single-posture shortcut.  The
    // locomotive PCD scene disables it so every base trajectory layer gets
    // its own IK candidates and inter-layer connections.
    bool enable_shared_posture_fast_path_{true};
    // Per-layer Cartesian IK candidate sampling parameters for the fixed
    // locomotive scene.
    int cartesian_samples_per_layer_{64};
    double cartesian_sample_radius_xy_{0.32};
    double cartesian_sample_z_min_{1.15};
    double cartesian_sample_z_max_{1.80};
    // Candidates accepted per base trajectory layer (indexed like
    // car_state_list_).  Filled lazily by sampleLayerCandidates().
    std::vector<std::vector<ManiPathNodePtr>> layer_candidates_;
    size_t collision_check_calls_{0};
    size_t edge_interpolation_checks_{0};
    size_t nodes_created_{0};
    // Per-type IK candidate collision rejections and IK failures.  Reported
    // by sampleLayerCandidates() so a failing layer can be diagnosed.
    std::array<size_t, 4> collision_type_counts_{{0, 0, 0, 0}};
    size_t ik_failure_count_{0};
    Eigen::Matrix3d phi_; // state transit matrix
    Eigen::Matrix4d T_q_0_;
    std::vector<Eigen::Matrix4Xd> manipulator_link_pts_;
    std::mt19937 goal_gen_;
    std::mt19937 state_gen_;
    std::mt19937 node_gen_;
    ros::Publisher cartesian_sample_marker_pub_;
    int cartesian_marker_id_{0};

    bool checkcollision(const ManiPathNodePtr& cur_state, const ManiPathNodePtr& next_state);
    int doubleIdx2int(double idx);
    Eigen::VectorXd idx2coord(const Eigen::VectorXi &s);
    Eigen::VectorXi coord2idx(const Eigen::VectorXd &s);
    double calAngleErr(double angle1, double angle2);
    bool feasibleCheck(ManiPathNodePtr &x1, ManiPathNodePtr &x2);
    double estimateHeuristic(ManiPathNodePtr &x1, ManiPathNodePtr &x2);
    string calculateValue(int &idx, const Eigen::VectorXd &state);
    string calculateValue(ManiPathNodePtr &x);
    ManiPathNodePtr initNode(int idx, const Eigen::VectorXd &s);
    ManiPathNodePtr getSampleNode();
    // Greedy layer-by-layer construction using sampleLayerCandidates() and
    // connectLayerCandidates().  Returns one coupled state per accepted layer.
    bool buildLayeredJointPath(const Eigen::VectorXd &start_state,
                               const Eigen::VectorXd &end_state,
                               std::vector<Eigen::VectorXd> &path,
                               std::vector<double> &yaw_list);
    ManiPathNodePtr getNearestNode(ManiPathNodePtr &x, bool dir);
    ManiPathNodePtr extendNode(ManiPathNodePtr &q_near, ManiPathNodePtr &q_rand, bool dir);
    void oneShot(ManiPathNodePtr &q);
    void trajShot(ManiPathNodePtr &q);
    void allShot(ManiPathNodePtr &q);
    void linkNode(ManiPathNodePtr &parent, ManiPathNodePtr &child);
    void expandGscore(ManiPathNodePtr &p);
    void adjustTree(ManiPathNodePtr &q_new, bool dir);
    void organizeTree();
    void organizeTree(ManiPathNodePtr &q);
    void clearSubTree(ManiPathNodePtr &q);
    void publishCartesianSample(const Eigen::Vector3d &p_base,
                                const Eigen::Vector3d &car_state,
                                int status);
    void mergeTrees(const ManiPathNodePtr &q1, const ManiPathNodePtr &q2);
    bool fullStateRepair();

    public:
    // Sample Cartesian end-effector targets for a base trajectory layer,
    // convert them through IK and accept each solution only after the full
    // coupled collision gate in initNode().  Accepted nodes are appended to
    // `candidates` and cached in layer_candidates_[layer].  Returns true when
    // at least one candidate is collision free.
    bool sampleLayerCandidates(int layer, const Eigen::VectorXd &seed,
                               std::vector<ManiPathNodePtr> &candidates);
    // Validate a joint-space edge between two already accepted layer
    // candidates.  Rejects on collision, velocity violation, dimension
    // mismatch, or missing candidate layers.
    bool connectLayerCandidates(int layer, const ManiPathNodePtr &from,
                                const ManiPathNodePtr &to);
    remani_planner::RrtPlanning::Ptr rrt_plan_;
    std::shared_ptr<remani_planner::MMConfig> mm_config_;
    SampleMani():
    goal_gen_(std::random_device{}()),
    state_gen_(std::random_device{}()),
    node_gen_(std::random_device{}())
    {};
    ~SampleMani(){
      for(auto it = node_pool_.begin(); it != node_pool_.end(); ++it){
        delete it->second;
      }
      node_pool_.clear();
    }

    bool search(const Eigen::VectorXd &start_state, const Eigen::VectorXd &end_state);
    void setParam(ros::NodeHandle& nh, const std::shared_ptr<remani_planner::MMConfig> &mm_config);
    void init(const std::vector<Eigen::Vector3d> &car_state_list, const std::vector<Eigen::Vector3d> &car_state_list_check, const std::vector<double> &t_list);
    void reset();
    bool getTraj(std::vector<Eigen::VectorXd> &traj);
    int getTreeNum(){return tree_count_;}
    double getCost();
    bool sampleManiSearch(const bool astar_succ, const Eigen::VectorXd &start_state, const Eigen::VectorXd &end_state,
                    const std::vector<Eigen::Vector3d> &car_state_list, const std::vector<Eigen::Vector3d> &car_state_list_check, 
                    const std::vector<double> &t_list, const std::vector<int> &singul_container, const int start_singul,// size = t_list.size()
                    std::vector<std::vector<Eigen::VectorXd>> &simple_path_container, std::vector<int> &singul_container_new,
                    std::vector<std::vector<double>> &yaw_list_container, std::vector<Eigen::VectorXd> &t_list_container);
    typedef shared_ptr<SampleMani> Ptr;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
};

#endif
