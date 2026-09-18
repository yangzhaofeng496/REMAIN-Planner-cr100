#include <plan_manage/remani_replan_fsm.h>

namespace remani_planner
{
  REMANIReplanFSM::~REMANIReplanFSM(){}
  void REMANIReplanFSM::init(ros::NodeHandle &nh)
  {
    exec_state_ = FSM_EXEC_STATE::INIT;
    have_target_ = false;
    have_odom_ = false;
    have_recv_pre_agent_ = false;
    flag_escape_emergency_ = true;
    try_plan_after_emergency_ = false;
    flag_relan_astar_ = false;
    have_local_traj_ = false;
    replan_fail_time_ = 0;
    consecutive_generation_failures_ = 0;

    /*  fsm param  */
    nh.param("fsm/target_type", target_type_, -1);
    nh.param("fsm/thresh_replan_time", replan_thresh_, -1.0);
    nh.param("fsm/thresh_no_replan_meter", no_replan_thresh_, -1.0);
    nh.param("fsm/planning_horizon", planning_horizen_, -1.0);
    nh.param("fsm/emergency_time", emergency_time_, 1.0);
    nh.param("fsm/fail_safe", enable_fail_safe_, true);
    nh.param("fsm/replan_trajectory_time", replan_trajectory_time_, 0.0);
    nh.param("fsm/time_for_gripper", time_for_gripper_, -1.0);
    nh.param("fsm/global_plan", global_plan_, false);
    if(global_plan_) planning_horizen_ = 1.0e3;

    nh.param("mm/mobile_base_dof", mobile_base_dim_, -1);
    nh.param("mm/manipulator_dof", manipulator_dim_, -1);
    nh.param("mm/mobile_base_non_singul_vel", mobile_base_non_singul_vel_, -1.0);

    // 3D end-effector goal via /clicked_point.
    nh.param("fsm/ee_goal_reach_xy_min", ee_goal_reach_xy_min_, 0.25);
    nh.param("fsm/ee_goal_reach_xy_max", ee_goal_reach_xy_max_, 1.10);
    nh.param("fsm/ee_goal_standoff", ee_goal_standoff_, 0.75);
    nh.param("fsm/ee_goal_clearance", ee_goal_clearance_, 0.05);
    nh.param("fsm/ee_goal_z_min", ee_goal_z_min_, 0.15);
    nh.param("fsm/ee_goal_z_max", ee_goal_z_max_, 2.00);
    nh.param("fsm/ee_goal_ik_samples", ee_goal_ik_samples_, 30);
    

    traj_dim_ = mobile_base_dim_ + manipulator_dim_;

    mm_state_pos_ = Eigen::VectorXd::Zero(traj_dim_);
    mm_state_vel_ = Eigen::VectorXd::Zero(traj_dim_);
    mm_state_acc_ = Eigen::VectorXd::Zero(traj_dim_);

    gripper_flag_ = true;

    start_pos_.resize(traj_dim_);
    start_vel_.resize(traj_dim_);
    start_acc_.resize(traj_dim_);
    start_jer_.resize(traj_dim_);

    nh.param("fsm/waypoint_num", waypoint_num_, -1);
    wpt_id_ = 0;

    waypoints_.clear();
    waypoints_yaw_.clear();
    Eigen::VectorXd wp = Eigen::VectorXd::Zero(traj_dim_);
    double yaw_temp;
    bool gripper_close;
    for (int i = 0; i < waypoint_num_; i++){
      nh.param("fsm/waypoint" + to_string(i) + "_yaw", yaw_temp, -1.0);
      waypoints_yaw_.push_back(yaw_temp * M_PI / 180.0);

      nh.param("fsm/waypoint" + to_string(i) + "_gripper_close", gripper_close, true);
      waypoint_gripper_close_.push_back(gripper_close);

      std::vector<double> waypoints_temp;
      nh.getParam("fsm/waypoint" + to_string(i), waypoints_temp);
      for(unsigned int j = 0; j < waypoints_temp.size(); j++){
        wp(j) = waypoints_temp[j];
        if((int)j >= mobile_base_dim_) wp(j) = wp(j) * M_PI / 180.0;
      }
      waypoints_.push_back(wp);
    }
    
    init_time_list_.clear();
    opt_time_list_.clear();
    total_time_list_.clear();

    rcv_gripper_state_ = false;
    gripper_state_ = false;
    map_state_ = 0;

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(nh));
    planner_manager_.reset(new MMPlannerManager);
    planner_manager_->initPlanModules(nh, visualization_);
    /* callback */
    exec_timer_ = nh.createTimer(ros::Duration(0.01), &REMANIReplanFSM::execFSMCallback, this);
    safety_timer_ = nh.createTimer(ros::Duration(0.01), &REMANIReplanFSM::checkCollisionCallback, this);
    // Always-on collision watch for the measured state (also useful while
    // manually driving the base/arm).  Publishes -1 when clear, otherwise the
    // collision type: 0 car-env, 1 arm-env, 2 arm-car, 3 arm-arm.
    collision_type_pub_ = nh.advertise<std_msgs::Int32>("collision_type", 1, true);
    collision_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("collision_markers", 1, true);
    gray_model_pub_ = nh.advertise<visualization_msgs::MarkerArray>("gray_robot_model", 1, true);
    watch_timer_ = nh.createTimer(ros::Duration(0.1), &REMANIReplanFSM::collisionWatchCallback, this);
    recovery_timer_ = nh.createTimer(ros::Duration(0.05), &REMANIReplanFSM::recoveryCallback, this);
    recovery_timer_.stop();
    recovery_joint_pub_ = nh.advertise<control_msgs::JointTrajectoryControllerState>(
        "/mm_controller_node/recovery_joint_cmd", 1);
    recovery_car_pub_ = nh.advertise<geometry_msgs::Twist>(
        "/mm_controller_node/recovery_car_cmd", 1);

    odom_sub_ = nh.subscribe("odom_world", 1, &REMANIReplanFSM::mmCarOdomCallback, this);
    joint_state_sub_ = nh.subscribe("joint_state", 1, &REMANIReplanFSM::mmManiOdomCallback, this);
    gripper_state_sub_ = nh.subscribe("gripper_state", 1, &REMANIReplanFSM::gripperCallback, this);

    poly_traj_pub_ = nh.advertise<quadrotor_msgs::PolynomialTraj>("planning/trajectory", 10);
    data_disp_pub_ = nh.advertise<traj_utils::DataDisp>("planning/data_display", 100);

    gripper_cmd_pub_ = nh.advertise<std_msgs::Bool>("gripper_cmd", 100);
    map_state_pub_ = nh.advertise<std_msgs::Int32>("/map_generator/map_state", 100);

    start_pub_ = nh.advertise<std_msgs::Bool>("planning/start", 1);
    reached_pub_ = nh.advertise<std_msgs::Bool>("planning/finish", 1);
    actual_ee_path_pub_ =
        nh.advertise<nav_msgs::Path>("/remani_planner/actual_ee_path", 1, true);
    resetEePath(actual_ee_path_, "world", ros::Time::now());
    waypoint_sub_ = nh.subscribe("/move_base_simple/goal", 1, &REMANIReplanFSM::waypointCallback, this);
    ee_goal_sub_ = nh.subscribe("/clicked_point", 1, &REMANIReplanFSM::eeGoalCallback, this);
    ee_goal_marker_pub_ = nh.advertise<visualization_msgs::Marker>("ee_goal_marker", 1, true);
    
  }

  void REMANIReplanFSM::execFSMCallback(const ros::TimerEvent &e)
  {
    exec_timer_.stop(); // To avoid blockage

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100){
      fsm_num = 0;
      // printFSMExecState();
    }

    switch (exec_state_){
    case INIT:
    {
      if (!have_odom_){
        goto force_return; // return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_)
        goto force_return; // return;
      else{
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case GEN_NEW_TRAJ:
    {
      if(try_plan_after_emergency_){
        std::cout << "emergency stop mm pos: " << mm_state_pos_.transpose() << std::endl;
        std::cout << "emergency stop mm vel: " << mm_state_vel_.transpose() << std::endl;
        std::cout << "emergency stop mm acc: " << mm_state_acc_.transpose() << std::endl;
        std::cout << "emergency stop mm yaw: " << mm_car_yaw_ << std::endl;
      }
      // std::cout << "gen new traj 1\n";
      have_local_traj_ = false;
      bool success = planFromGlobalTraj(1);
      // std::cout << "gen new traj 2\n";
      if (success){
        consecutive_generation_failures_ = 0;
        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
        try_plan_after_emergency_ = false;
      }
      else
      {
        ++consecutive_generation_failures_;
        ROS_WARN("[FSM] trajectory generation failed (%d/3); target remains pending",
                 consecutive_generation_failures_);
        if (consecutive_generation_failures_ >= 3)
        {
          ROS_ERROR("[FSM] planning failed after 3 attempts; returning to WAIT_TARGET");
          have_target_ = false;
          have_new_target_ = false;
          have_local_traj_ = false;
          consecutive_generation_failures_ = 0;
          changeFSMExecState(WAIT_TARGET, "FSM");
        }
        else
        {
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        }
      }
      break;
    }

    case REPLAN_TRAJ:
    {
      
      if(planFromLocalTraj(flag_relan_astar_)){
        replan_fail_time_ = 0;
        flag_relan_astar_ = false;
        if((ros::Time::now() - t_last_Astar_ ).toSec() > 1.0){
          std::cout << "cal front end next time" << std::endl;
          flag_relan_astar_ = true;
          t_last_Astar_ = ros::Time::now();
        }
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else{
        replan_fail_time_++;
        flag_relan_astar_ = true;
        t_last_Astar_ = ros::Time::now();
        if(replan_fail_time_ >= 20){
          replan_fail_time_ = 0;
          ROS_ERROR("[FSM]:REPLAN fail over 20 times!!!");
          changeFSMExecState(WAIT_TARGET, "FSM");
        }
        else{
          changeFSMExecState(REPLAN_TRAJ, "FSM");
        }
      }
      break;
    }

    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      SingulTrajData *info = &planner_manager_->traj_container_.singul_traj_data;
      // LocalTrajData *info = &planner_manager_->traj_container_.local_traj;
      double t_cur = ros::Time::now().toSec() - info->start_time;
      bool need_to_plan_next = ((t_cur - info->duration) > time_for_gripper_);
      bool need_to_gripper = (t_cur > info->duration + 0.01);
      t_cur = min(info->duration, t_cur);

      Eigen::VectorXd pos = info->getPos(t_cur);
      // A direct global safety-detour segment does not populate the local
      // target state.  It is nevertheless complete when its own endpoint is
      // reached, so do not subtract vectors with an uninitialised dimension.
      bool touch_the_goal = (local_target_pt_.size() == end_pt_.size() &&
                             (local_target_pt_ - end_pt_).norm() < 1e-2);
      bool close_to_no_replan_thresh = ((end_pt_ - pos).head(2).norm() < no_replan_thresh_);

      if(target_type_ == TARGET_TYPE::PRESET_TARGET && close_to_no_replan_thresh){
        if((wpt_id_ < waypoint_num_ - 1) && need_to_plan_next){
          ++wpt_id_;
          planNextWaypoint(waypoints_[wpt_id_], waypoints_yaw_[wpt_id_]);
          gripper_flag_ = true;
        }else if(need_to_gripper && gripper_flag_){
          ++map_state_;
          std_msgs::Bool gripper_cmd;
          gripper_cmd.data = waypoint_gripper_close_[wpt_id_]; // true: close gripper; false: open
          gripper_cmd_pub_.publish(gripper_cmd);

          std::string gripper_cmd_str = waypoint_gripper_close_[wpt_id_] ? "close gripper" : "open gripper";
          ROS_INFO(gripper_cmd_str.c_str());

          std_msgs::Int32 map_state;
          map_state.data = map_state_;
          // map_state_pub_.publish(map_state);

          // planner_manager_->grid_map_->md_.has_cloud_ = false;

          gripper_flag_ = false;
        }
        
      }else if(t_cur > info->duration - 1e-2 && touch_the_goal){
        
        if(target_type_ != TARGET_TYPE::PRESET_TARGET && wpt_id_ >= waypoint_num_ - 1){
          have_target_ = false;
          have_trigger_ = false;
          /* The navigation task completed */
          std::cout << "reach goal\n";
          changeFSMExecState(WAIT_TARGET, "FSM");

          std_msgs::Bool msg;
          msg.data = true;
          reached_pub_.publish(msg);
          goto force_return;
        }
        
      }else if(!close_to_no_replan_thresh && t_cur > replan_thresh_ && (!global_plan_)){
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EMERGENCY_STOP:
    {
      if(flag_escape_emergency_){ // Avoiding repeated calls
        callEmergencyStop(mm_state_pos_, mm_car_yaw_, mm_car_singul_);
      }
      else{
        if(enable_fail_safe_ && mm_state_vel_.head(2).norm() < 0.1){
          try_plan_after_emergency_ = true;
          have_local_traj_ = false;
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        }
      }

      flag_escape_emergency_ = false;

      break;
    }
    }

    data_disp_.header.stamp = ros::Time::now();
    data_disp_pub_.publish(data_disp_);

  force_return:;
    exec_timer_.start();
  }

  void REMANIReplanFSM::checkCollisionCallback(const ros::TimerEvent &e){
    SingulTrajData *info = &planner_manager_->traj_container_.singul_traj_data;
    auto map = planner_manager_->grid_map_;

    if (exec_state_ == WAIT_TARGET || info->traj_id <= 0)
      return;
    /* ---------- check lost of depth ---------- */
    if (map->getOdomDepthTimeout()){
      ROS_ERROR("Depth Lost! EMERGENCY_STOP");
      enable_fail_safe_ = false;
      changeFSMExecState(EMERGENCY_STOP, "SAFETY");
    }
    // std::cout << "check 3" << std::endl;
    /* ---------- check trajectory ---------- */
    constexpr double time_step = 0.01;
    double t_cur = ros::Time::now().toSec() - info->start_time;
    Eigen::VectorXd p_cur = info->getPos(t_cur);
    double t_1_2 = info->duration * 1 / 2;
    double t_2_3 = info->duration * 2 / 3;
    double t_temp;
    bool occ = false;
    // std::cout << "check 4" << std::endl;
    int coll_type;
    for (double t = t_cur; t < info->duration; t += time_step){
      // If t_cur < t_1_2, only the first 2/3 partition of the trajectory is considered valid and will get checked.
      if (t_cur < t_1_2 && t >= t_2_3)
        break;
        
      if (planner_manager_->ploy_traj_opt_->checkCollision(*info, t, coll_type)){
        if(coll_type == 0){
          ROS_WARN("car collision at relative time %f!", t / info->duration);
        }else if (coll_type == 1){
          ROS_WARN("mani collision at relative time %f!", t / info->duration);
        }else if (coll_type == 2){
          ROS_WARN("car-mani collision at relative time %f!", t / info->duration);
        }else if (coll_type == 3){
          ROS_WARN("mani-mani collision at relative time %f!", t / info->duration);
        }
        
        t_temp = t;
        occ = true;
        break;
      }
    }

    if (occ){
      /* Handle the collided case immediately */
      ROS_INFO("Try to replan a safe trajectory");
      if (planFromLocalTraj(false)){ // Make a chance
        ROS_INFO("Plan success when detect collision.");
        changeFSMExecState(EXEC_TRAJ, "SAFETY");
        return;
      }else{
        // if(planFromLocalTraj(true))
        // {
        //   ROS_INFO("Plan success when detect collision.");
        //   changeFSMExecState(EXEC_TRAJ, "SAFETY");
        //   return;
        // }
        if (t_temp - t_cur < emergency_time_){ // 1.0s of emergency time
          ROS_WARN("Emergency stop! time=%f", t_temp - t_cur);
          changeFSMExecState(EMERGENCY_STOP, "SAFETY");
        }else{
          ROS_WARN("current traj in collision, replan.");
          if(planFromLocalTraj(true))
          {
            ROS_INFO("Plan success when detect collision.");
            changeFSMExecState(EXEC_TRAJ, "SAFETY");
            return;
          }
          changeFSMExecState(REPLAN_TRAJ, "SAFETY");
        }
        return;
      }
    }
  }

  void REMANIReplanFSM::collisionWatchCallback(const ros::TimerEvent &e){
    if (!have_odom_ || mm_state_pos_.size() < traj_dim_)
      return;
    Eigen::Vector3d car_state(mm_state_pos_(0), mm_state_pos_(1), mm_car_yaw_);
    Eigen::VectorXd mani_state = mm_state_pos_.tail(manipulator_dim_);
    int coll_type = -1;
    bool coll = planner_manager_->mm_config_->checkcollision(car_state, mani_state, false, coll_type);
    std_msgs::Int32 msg;
    msg.data = coll ? coll_type : -1;
    collision_type_pub_.publish(msg);
    visualization_msgs::MarkerArray collision_markers;
    planner_manager_->mm_config_->getSelfCollisionMarkers(car_state, mani_state, collision_markers);
    collision_marker_pub_.publish(collision_markers);
    visualization_msgs::MarkerArray gray_model;
    planner_manager_->mm_config_->getMMMarkerArray(gray_model, "gray_robot_model", 0, 1.0,
                                                    car_state, mani_state, true);
    gray_model.markers.push_back(planner_manager_->mm_config_->getArmGripperMarker(car_state, mani_state));
    gray_model_pub_.publish(gray_model);
    if (coll && coll_type != last_collision_type_){
      ROS_WARN("[CollisionWatch] COLLISION type=%d (0 car-env, 1 arm-env, 2 arm-car, 3 arm-arm)", coll_type);
      ROS_WARN_STREAM("[CollisionWatch] actual car=(" << car_state.transpose()
                      << ") arm=" << mani_state.transpose());
      SingulTrajData *traj = &planner_manager_->traj_container_.singul_traj_data;
      if (traj->traj_id > 0) {
        const double t = std::max(0.0, ros::Time::now().toSec() - traj->start_time);
        const Eigen::VectorXd planned = traj->getPos(std::min(t, traj->duration));
        ROS_WARN_STREAM("[CollisionWatch] planned t=" << t
                        << " car=(" << planned.head(3).transpose()
                        << ") arm=" << planned.tail(manipulator_dim_).transpose()
                        << " state_error=" << (mani_state - planned.tail(manipulator_dim_)).norm());
      }
    } else if (!coll && last_collision_type_ != -1){
      ROS_INFO("[CollisionWatch] clear");
    }
    last_collision_type_ = coll ? coll_type : -1;
  }

  bool REMANIReplanFSM::planNextWaypoint(const Eigen::VectorXd next_wp, const double next_yaw)
  {
    // The fixed PCD scene must not start planning before the static global
    // cloud has populated the occupancy buffer; otherwise the first plan is
    // built against an empty map and fails mid-execution.
    if(planner_manager_ && planner_manager_->grid_map_ &&
       planner_manager_->grid_map_->usesGlobalMap() &&
       !planner_manager_->grid_map_->isGlobalMapReady()){
      ROS_WARN("[FSM] rejecting goal: static global map is not ready yet");
      return false;
    }
    ROS_INFO("[FSM] accepting goal: uses_global_map=%s global_map_ready=%s",
             (planner_manager_ && planner_manager_->grid_map_ &&
              planner_manager_->grid_map_->usesGlobalMap()) ? "true" : "false",
             (planner_manager_ && planner_manager_->grid_map_ &&
              planner_manager_->grid_map_->isGlobalMapReady()) ? "true" : "false");

    std::vector<Eigen::VectorXd> one_pt_wps;
    one_pt_wps.push_back(next_wp);
    bool success = planner_manager_->planGlobalTrajWaypoints(
        mm_state_pos_, mm_car_yaw_, Eigen::VectorXd::Zero(traj_dim_), Eigen::VectorXd::Zero(traj_dim_),
        one_pt_wps, next_yaw, Eigen::VectorXd::Zero(traj_dim_), Eigen::VectorXd::Zero(traj_dim_));

    // visualization_->displayGoalPoint(next_wp, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {
      end_pt_ = next_wp;
      end_yaw_ = next_yaw;
      have_local_traj_ = false;
      start_singul_ = 0;

      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->traj_container_.global_traj.duration / step_size_t);
      vector<Eigen::Vector2d> global_traj(i_end);
      for (int i = 0; i < i_end; i++){
        global_traj[i] = planner_manager_->traj_container_.global_traj.traj.getPos(i * step_size_t).head(mobile_base_dim_);
      }

      have_target_ = true;
      have_new_target_ = true;

      /*** FSM ***/
      if (exec_state_ != WAIT_TARGET)
      {
        // Do not block inside the goal callback waiting for EXEC_TRAJ.  The
        // callback runs on the ROS spinner thread; blocking here can prevent
        // the FSM timer from advancing and leaves a received goal without a
        // planning attempt.  A new goal preempts the current state and is
        // consumed by the normal FSM loop immediately.
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      }

      // visualization_->displayGoalPoint(final_goal_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGoalPoint(end_pt_.head(2), Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalTraj(global_traj, 0.05, 0);
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }

    return success;
  }

  // manual waypoint
  void REMANIReplanFSM::waypointCallback(const geometry_msgs::PoseStamped::ConstPtr &msg){
    
    if (target_type_ == TARGET_TYPE::PRESET_TARGET){
      have_trigger_ = true;
      cout << "Triggered! traget type: " << target_type_ << endl;

      std_msgs::Bool flag_msg;
      flag_msg.data = true;
      planner_manager_->global_start_time_ = ros::Time::now();
      planner_manager_->start_flag_ = true;
      start_pub_.publish(flag_msg);
      wpt_id_ = 0;
      planNextWaypoint(waypoints_[wpt_id_], waypoints_yaw_[wpt_id_]);
      return;
    }

    if(msg->pose.position.z < -0.1)
      return;
    cout << "Triggered! traget type: " << target_type_ << endl;
    // trigger_ = true;
    init_state_ = mm_state_pos_;
    end_pt_ = Eigen::VectorXd::Zero(traj_dim_);
    
    if(target_type_ == TARGET_TYPE::MANUAL_TARGET){
      end_pt_(0) = msg->pose.position.x;
      end_pt_(1) = msg->pose.position.y;
      end_yaw_ = tf::getYaw(msg->pose.orientation);
      ROS_WARN("[FSM] new goal: x=%.3f y=%.3f yaw=%.3f (start x=%.3f y=%.3f yaw=%.3f)",
               end_pt_(0), end_pt_(1), end_yaw_,
               mm_state_pos_(0), mm_state_pos_(1), mm_car_yaw_);
      // A 2D Nav Goal only specifies the mobile base.  Keep the measured arm
      // posture when it is already collision-free at the goal (this also keeps
      // the cheap stationary-arm path); only fold to a sampled feasible
      // posture when the measured one would collide there.
      Eigen::VectorXd goal_mani = mm_state_pos_.tail(manipulator_dim_);
      Eigen::Vector3d goal_car(end_pt_(0), end_pt_(1), end_yaw_);
      int goal_coll_type = -1;
      if (planner_manager_->mm_config_->checkcollision(goal_car, goal_mani, false, goal_coll_type)) {
        ROS_WARN("[FSM] measured arm posture collides at goal (type=%d); sampling a folded posture",
                 goal_coll_type);
        if (!planner_manager_->mm_config_->sampleFeasibleManiState(goal_car, goal_mani)) {
          ROS_WARN("[FSM] no feasible goal arm posture found; keeping measured posture");
        }
      }
      end_pt_.tail(manipulator_dim_) = goal_mani;
    }else{
      ROS_ERROR("wrong target type: %d", target_type_);
      return;
    }

    planNextWaypoint(end_pt_, end_yaw_);
    return;
  }

  // RViz Publish Point -> /clicked_point.  Treat the clicked world point as the
  // desired arm end-effector position, choose a mobile-base goal that brings
  // the point into the arm workspace, solve IK (position primary, orientation
  // sampled) and hand the resulting joint goal to the normal planning pipeline.
  void REMANIReplanFSM::eeGoalCallback(const geometry_msgs::PointStamped::ConstPtr &msg)
  {
    if (!msg->header.frame_id.empty() && msg->header.frame_id != "world")
    {
      ROS_WARN("[FSM] /clicked_point frame '%s' != world; ignoring",
               msg->header.frame_id.c_str());
      return;
    }

    const Eigen::Vector3d target(msg->point.x, msg->point.y, msg->point.z);
    ROS_WARN("[FSM] ee-goal clicked: (%.3f %.3f %.3f)", target.x(), target.y(), target.z());
    bool preempted = false;

    if (exec_state_ == EXEC_TRAJ || have_local_traj_) {
      ROS_WARN("[FSM] preempting previous trajectory for newest ee-goal");
      callEmergencyStop(mm_state_pos_, mm_car_yaw_, mm_car_singul_);
      have_local_traj_ = false;
      have_target_ = false;
      have_new_target_ = true;
      pending_ee_goal_ = target;
      preempt_settle_active_ = true;
      preempt_settle_deadline_ = ros::Time::now() + ros::Duration(0.20);
      preempted = true;
      changeFSMExecState(WAIT_TARGET, "NEW_EE_GOAL");
    }

    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "ee_goal";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = target.x();
    marker.pose.position.y = target.y();
    marker.pose.position.z = target.z();
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.12;
    marker.color.r = 1.0;
    marker.color.g = 0.2;
    marker.color.b = 0.2;
    marker.color.a = 1.0;
    ee_goal_marker_pub_.publish(marker);

    // Let the controller consume ACTION_ABORT and publish one or two fresh
    // measured states before constructing the replacement trajectory.
    if (preempted) {
      recovery_timer_.start();
      ROS_WARN("[FSM] waiting for controller settle before replanning newest ee-goal");
      return;
    }

    int current_collision = -1;
    const Eigen::Vector3d current_car(mm_state_pos_(0), mm_state_pos_(1), mm_car_yaw_);
    const Eigen::VectorXd current_arm = mm_state_pos_.tail(manipulator_dim_);
    if (planner_manager_->mm_config_->checkcollision(current_car, current_arm, false,
                                                      current_collision)) {
      Eigen::VectorXd recovery_q;
      bool recovery_found = false;
      // Search a small coupled base/arm neighborhood.  A colliding arm can
      // be unrecoverable at the current base pose even when a nearby base
      // pose has a valid self-collision-free arm posture.
      const double offsets[] = {0.0, 0.25, -0.25, 0.50, -0.50};
      const double yaw_offsets[] = {0.0, 0.35, -0.35, 0.70, -0.70};
      for (double dx : offsets) {
        for (double dy : offsets) {
          for (double dyaw : yaw_offsets) {
            Eigen::Vector3d candidate_car = current_car;
            candidate_car.x() += dx;
            candidate_car.y() += dy;
            candidate_car.z() += dyaw;
            if (planner_manager_->mm_config_->sampleFeasibleManiState(
                    candidate_car, recovery_q, 40)) {
              recovery_car_goal_ = candidate_car;
              recovery_found = true;
              break;
            }
          }
          if (recovery_found) break;
        }
        if (recovery_found) break;
      }
      if (!recovery_found) {
        ROS_ERROR("[Recovery] no safe arm posture found; refusing ee-goal");
        return;
      }
      pending_ee_goal_ = target;
      recovery_joint_goal_ = recovery_q;
      recovery_active_ = true;
      recovery_deadline_ = ros::Time::now() + ros::Duration(5.0);
      recovery_timer_.start();
      ROS_WARN("[Recovery] current collision type=%d; moving arm to a safe posture before replanning",
               current_collision);
      return;
    }

    planToEeGoal(target);
  }

  void REMANIReplanFSM::recoveryCallback(const ros::TimerEvent &)
  {
    if (preempt_settle_active_) {
      if (ros::Time::now() < preempt_settle_deadline_) return;
      preempt_settle_active_ = false;
      recovery_timer_.stop();
      if (have_odom_) {
        ROS_WARN("[FSM] controller settled; replanning from measured state");
        planToEeGoal(pending_ee_goal_);
      }
      return;
    }
    if (!recovery_active_) return;
    if (!have_odom_ || recovery_joint_goal_.size() != manipulator_dim_) return;

    const Eigen::Vector3d car(mm_state_pos_(0), mm_state_pos_(1), mm_car_yaw_);
    int collision_type = -1;
    const bool still_colliding = planner_manager_->mm_config_->checkcollision(
        car, mm_state_pos_.tail(manipulator_dim_), false, collision_type);
    if (!still_colliding) {
      recovery_active_ = false;
      recovery_timer_.stop();
      ROS_WARN("[Recovery] arm is SAFE; replanning pending ee-goal (%.3f %.3f %.3f)",
               pending_ee_goal_.x(), pending_ee_goal_.y(), pending_ee_goal_.z());
      planToEeGoal(pending_ee_goal_);
      return;
    }
    if (ros::Time::now() > recovery_deadline_) {
      recovery_active_ = false;
      recovery_timer_.stop();
      ROS_ERROR("[Recovery] failed to leave self-collision within 5 seconds (type=%d)",
                collision_type);
      return;
    }

    geometry_msgs::Twist car_cmd;
    car_cmd.linear.x = recovery_car_goal_.x();
    car_cmd.linear.y = recovery_car_goal_.y();
    car_cmd.linear.z = recovery_car_goal_.z();
    recovery_car_pub_.publish(car_cmd);

    control_msgs::JointTrajectoryControllerState cmd;
    cmd.joint_names.resize(manipulator_dim_);
    cmd.desired.positions.resize(manipulator_dim_);
    cmd.desired.velocities.assign(manipulator_dim_, 0.0);
    cmd.desired.effort.assign(manipulator_dim_, 0.0);
    for (int i = 0; i < manipulator_dim_; ++i) {
      cmd.joint_names[i] = "joint" + std::to_string(i + 1);
      cmd.desired.positions[i] = recovery_joint_goal_(i);
    }
    recovery_joint_pub_.publish(cmd);
  }

  bool REMANIReplanFSM::planToEeGoal(const Eigen::Vector3d &target_world)
  {
    const ros::WallTime goal_start = ros::WallTime::now();
    if (target_type_ != TARGET_TYPE::MANUAL_TARGET)
    {
      ROS_WARN("[FSM] ee-goal ignored: target_type is not manual");
      return false;
    }
    if (!have_odom_ || !planner_manager_ || !planner_manager_->mm_config_)
    {
      ROS_WARN("[FSM] ee-goal rejected: odom/planner not ready");
      return false;
    }
    if (mm_state_pos_.size() < traj_dim_)
    {
      ROS_WARN("[FSM] ee-goal rejected: state not ready");
      return false;
    }
    if (planner_manager_->grid_map_ && planner_manager_->grid_map_->usesGlobalMap() &&
        !planner_manager_->grid_map_->isGlobalMapReady())
    {
      ROS_WARN("[FSM] ee-goal rejected: static global map is not ready yet");
      return false;
    }

    const Eigen::Vector2d base_xy(mm_state_pos_(0), mm_state_pos_(1));
    const double base_yaw = mm_car_yaw_;
    const Eigen::Vector2d target_xy(target_world.x(), target_world.y());

    const Eigen::Vector2d to_target = target_xy - base_xy;
    const double dist = to_target.norm();
    Eigen::Vector2d dir;
    if (dist > 1e-3)
      dir = to_target / dist;
    else
      dir = Eigen::Vector2d(std::cos(base_yaw), std::sin(base_yaw));

    if (target_world.z() < ee_goal_z_min_ || target_world.z() > ee_goal_z_max_)
    {
      ROS_ERROR("[FSM] ee-goal z=%.3f outside [%.2f, %.2f]",
                target_world.z(), ee_goal_z_min_, ee_goal_z_max_);
      return false;
    }

    // Standoff candidates: place the base so the point sits at the preferred
    // arm reach.  Deliberately avoid a zero-motion base goal: the coupled
    // sampler needs at least a short mobile-base path to reconfigure the arm,
    // so a purely stationary base makes the arm-only search return NO_PATH.
    std::vector<double> standoffs;
    const double kMinBaseMove = 0.10;
    const double offsets[] = {0.0, 0.15, -0.15, 0.30, -0.30, 0.45, -0.45};
    for (double o : offsets)
    {
      double s = ee_goal_standoff_ + o;
      s = std::max(ee_goal_reach_xy_min_, std::min(ee_goal_reach_xy_max_, s));
      if (std::abs(s - dist) < kMinBaseMove)
        continue;
      bool dup = false;
      for (double e : standoffs)
        if (std::abs(e - s) < 1e-3)
          dup = true;
      if (!dup)
        standoffs.push_back(s);
    }
    // Last resort: keep the current base position when the point is in reach.
    if (dist >= ee_goal_reach_xy_min_ && dist <= ee_goal_reach_xy_max_)
    {
      bool dup = false;
      for (double e : standoffs)
        if (std::abs(e - dist) < 1e-3)
          dup = true;
      if (!dup)
        standoffs.push_back(dist);
    }

    // Try to reach the clicked point exactly first; only pull the IK target
    // back toward the base when the exact point has no collision-free IK (for
    // points picked on an obstacle surface).
    std::vector<double> clearances;
    clearances.push_back(0.0);
    if (ee_goal_clearance_ > 1e-6)
      clearances.push_back(ee_goal_clearance_);

    const Eigen::VectorXd ik_seed = mm_state_pos_.tail(manipulator_dim_);

    for (double clearance : clearances)
    {
      const Eigen::Vector3d reach_world =
          target_world - Eigen::Vector3d(clearance * dir.x(), clearance * dir.y(), 0.0);
      for (double standoff : standoffs)
      {
        const Eigen::Vector2d goal_xy = target_xy - standoff * dir;
        const Eigen::Vector2d goal_to_target = target_xy - goal_xy;
        double goal_yaw = base_yaw;
        if (goal_to_target.norm() > 1e-3)
          goal_yaw = std::atan2(goal_to_target.y(), goal_to_target.x());
        const Eigen::Vector3d car_state(goal_xy.x(), goal_xy.y(), goal_yaw);

        Eigen::Matrix4d T_car;
        planner_manager_->mm_config_->CarState2T(car_state, T_car);
        const Eigen::Vector3d p_base =
            T_car.block<3, 3>(0, 0).transpose() *
            (reach_world - T_car.block<3, 1>(0, 3));

        // LM is a local solver: try the measured posture first, then a set of
        // collision-free sampled postures at this base goal as alternative seeds.
        std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd>> seeds;
        // Try independently sampled collision-free postures first. The
        // measured posture is a useful fallback, but preferring it can force
        // the subsequent coupled trajectory through a Link3-Link5 self-
        // collision even when another IK branch is available.
        for (int k = 0; k < ee_goal_ik_samples_; ++k)
        {
          Eigen::VectorXd s;
          if (planner_manager_->mm_config_->sampleFeasibleManiState(car_state, s, 40))
            seeds.push_back(s);
        }
        seeds.push_back(ik_seed);

        for (const auto &s : seeds)
        {
          const ros::WallTime arm_ik_start = ros::WallTime::now();
          Eigen::VectorXd q;
          if (!planner_manager_->mm_config_->solveEndEffectorPositionIK(p_base, s, q))
            continue;
          if (q.size() != manipulator_dim_)
            continue;
          // LM only minimises a weighted task error, so verify the achieved
          // end-effector position before accepting the joint goal.
          Eigen::Matrix4d T_ee;
          if (!planner_manager_->computeUrdfEeTransform(q, T_ee))
            continue;
          const Eigen::Vector3d ee_center =
              T_ee.block<3, 1>(0, 3) +
              T_ee.block<3, 3>(0, 0) *
              planner_manager_->mm_config_->getEndEffectorCenterOffset();
          if ((ee_center - p_base).norm() > 2e-3)
            continue;
          int coll_type = -1;
          if (planner_manager_->mm_config_->checkcollision(car_state, q, false, coll_type))
          {
            ROS_INFO("[FSM] ee-goal IK candidate rejected (collision type=%d)", coll_type);
            continue;
          }

          end_pt_ = Eigen::VectorXd::Zero(traj_dim_);
          end_pt_(0) = car_state(0);
          end_pt_(1) = car_state(1);
          end_pt_.tail(manipulator_dim_) = q;
          end_yaw_ = car_state(2);
          ROS_WARN("[FSM] ee-goal solved: target=(%.3f %.3f %.3f) base=(%.3f %.3f %.3f) "
                   "standoff=%.3f clearance=%.3f",
                   target_world.x(), target_world.y(), target_world.z(),
                   car_state(0), car_state(1), car_state(2),
                   standoff, clearance);
          ROS_INFO("[PLAN_TIME] base_ik_select_ms=%.3f arm_ik_ms=%.3f",
                   (ros::WallTime::now() - goal_start).toSec() * 1000.0,
                   (ros::WallTime::now() - arm_ik_start).toSec() * 1000.0);
          return planNextWaypoint(end_pt_, end_yaw_);
        }
      }
    }

    ROS_ERROR("[FSM] ee-goal: no reachable collision-free IK for target=(%.3f %.3f %.3f)",
              target_world.x(), target_world.y(), target_world.z());
    return false;
  }

  void REMANIReplanFSM::mmCarOdomCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    // std::cout << "odom: " << mm_state_pos_.transpose() << "\n";
    mm_state_pos_(0) = msg->pose.pose.position.x;
    mm_state_pos_(1) = msg->pose.pose.position.y;
    mm_car_yaw_ = tf::getYaw(msg->pose.pose.orientation);

    mm_car_orient_.w() = msg->pose.pose.orientation.w;
    mm_car_orient_.x() = msg->pose.pose.orientation.x;
    mm_car_orient_.y() = msg->pose.pose.orientation.y;
    mm_car_orient_.z() = msg->pose.pose.orientation.z;

    mm_state_vel_(0) = msg->twist.twist.linear.x;
    mm_state_vel_(1) = msg->twist.twist.linear.y;
    if(mm_state_vel_.head(2).norm() < mobile_base_non_singul_vel_){
      mm_state_vel_(0) = mobile_base_non_singul_vel_ * cos(mm_car_yaw_);
      mm_state_vel_(1) = mobile_base_non_singul_vel_ * sin(mm_car_yaw_);
      mm_car_singul_ = 0;
    }else{
      Eigen::Vector2d car_head(cos(mm_car_yaw_), sin(mm_car_yaw_));
      mm_car_singul_ = 1 ? car_head.dot(mm_state_vel_.head(2)) >= 0 : -1;
    }

    mm_car_yaw_rate_ = msg->twist.twist.angular.z;

    have_odom_ = true;
  }

  void REMANIReplanFSM::mmManiOdomCallback(const sensor_msgs::JointStateConstPtr &msg){
    for(int i = 0; i < manipulator_dim_; ++i){
      mm_state_pos_(mobile_base_dim_ + i) = msg->position[i];
      mm_state_vel_(mobile_base_dim_ + i) = msg->velocity[i];
      mm_state_acc_(mobile_base_dim_ + i) = msg->effort[i];
    }
    if(!planner_manager_ || !have_odom_ || mm_state_pos_.size() < traj_dim_){
      return;
    }
    const ros::Time now = ros::Time::now();
    // Append measured end-effector samples at the joint-state callback rate,
    // throttled to ~20 Hz.
    if(last_actual_ee_time_.isZero() || (now - last_actual_ee_time_).toSec() >= 0.05){
      Eigen::Matrix4d T_ee;
      if(planner_manager_->computeUrdfEeTransform(mm_state_pos_.tail(manipulator_dim_), T_ee)){
        const Eigen::Vector3d car_state(mm_state_pos_(0), mm_state_pos_(1), mm_car_yaw_);
        Eigen::Matrix4d T_car;
        planner_manager_->mm_config_->CarState2T(car_state, T_car);
        appendEePose(actual_ee_path_, T_car * T_ee, now);
        actual_ee_path_.header.stamp = now;
        actual_ee_path_pub_.publish(actual_ee_path_);
      }
      last_actual_ee_time_ = now;
    }

    // Throttled planned-vs-actual tracking diagnostics.
    SingulTrajData &traj = planner_manager_->traj_container_.singul_traj_data;
    if(traj.traj_id > 0){
      const double t = now.toSec() - traj.start_time;
      if(t >= 0.0 && t <= traj.duration){
        const Eigen::VectorXd planned = traj.getPos(t);
        const double joint_dist =
            (mm_state_pos_.tail(manipulator_dim_) -
             planned.tail(manipulator_dim_)).norm();
        double ee_dist = -1.0;
        double car_xy_dist = -1.0;
        double car_yaw_dist = -1.0;
        Eigen::Matrix4d T_planned_ee;
        if(planner_manager_->computeUrdfEeTransform(planned.tail(manipulator_dim_), T_planned_ee)){
          Eigen::Matrix4d T_planned_car;
          const Eigen::VectorXd planned_vel = traj.getVel(t);
          const int planned_singul = traj.getSingul(t);
          const double planned_yaw = std::atan2(planned_singul * planned_vel(1),
                                                planned_singul * planned_vel(0));
          const Eigen::Vector3d planned_car(planned(0), planned(1), planned_yaw);
          car_xy_dist = (planned_car.head<2>() - mm_state_pos_.head<2>()).norm();
          car_yaw_dist = std::atan2(std::sin(planned_yaw - mm_car_yaw_),
                                    std::cos(planned_yaw - mm_car_yaw_));
          planner_manager_->mm_config_->CarState2T(planned_car, T_planned_car);
          Eigen::Matrix4d T_actual_ee;
          if(planner_manager_->computeUrdfEeTransform(mm_state_pos_.tail(manipulator_dim_), T_actual_ee)){
            const Eigen::Vector3d actual_car(mm_state_pos_(0), mm_state_pos_(1), mm_car_yaw_);
            Eigen::Matrix4d T_actual_car;
            planner_manager_->mm_config_->CarState2T(actual_car, T_actual_car);
            ee_dist = ((T_actual_car * T_actual_ee).block<3, 1>(0, 3) -
                       (T_planned_car * T_planned_ee).block<3, 1>(0, 3)).norm();
          }
        }
        ROS_INFO_THROTTLE(1.0,
                          "[EETrack] t=%.2f joint_dist=%.4f ee_dist=%.4f car_xy=%.4f car_yaw=%.4f",
                          t, joint_dist, ee_dist, car_xy_dist, car_yaw_dist);
      }
    }
  }

  void REMANIReplanFSM::gripperCallback(const std_msgs::Bool::ConstPtr &msg){
    if(gripper_state_ != msg->data || (!rcv_gripper_state_)){
      rcv_gripper_state_ = true;
      gripper_state_ = msg->data;
      planner_manager_->mm_config_->setGripperPoint(gripper_state_);
    }
  }

  void REMANIReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call){
    if (new_state == exec_state_)
      continously_called_times_++;
    else
      continously_called_times_ = 1;

    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  void REMANIReplanFSM::printFSMExecState(){
    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
    static int last_printed_state = -1, dot_nums = 0;

    if (exec_state_ != last_printed_state)
      dot_nums = 0;
    else
      dot_nums++;

    cout << "\r[FSM]: state: " + state_str[int(exec_state_)];

    last_printed_state = exec_state_;

    // some warnings
    if (!have_odom_)
    {
      cout << ", waiting for odom";
    }
    if (!have_target_)
    {
      cout << ", waiting for target";
    }
    if (!have_trigger_)
    {
      cout << ", waiting for trigger";
    }
    if (planner_manager_->pp_.drone_id >= 1 && !have_recv_pre_agent_)
    {
      cout << ", haven't receive traj from previous drone";
    }

    cout << string(dot_nums, '.') << endl;

    fflush(stdout);
  }

  std::pair<int, REMANIReplanFSM::FSM_EXEC_STATE> REMANIReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
  }

  void REMANIReplanFSM::sendPolyTrajROSMsg(){
    auto data = &planner_manager_->traj_container_.singul_traj_data;
    // Start a fresh measured end-effector trace for each new trajectory.
    const ros::Time execution_stamp = ros::Time::now();
    resetEePath(actual_ee_path_, "world", execution_stamp);
    last_actual_ee_time_ = ros::Time(0);
    // Publish the empty reset immediately.  Otherwise RViz keeps the old
    // latched red path visible until the next joint-state callback and can
    // connect samples from two different trajectories into a false jump.
    actual_ee_path_pub_.publish(actual_ee_path_);
    // Send one complete trajectory atomically.  Sending one ROS message per
    // piece lets a replan interleave with the previous trajectory and causes
    // the controller to append pieces from different trajectory versions.
    quadrotor_msgs::PolynomialTraj msg;
    msg.trajectory_id = 1;
    msg.header.stamp = ros::Time(data->start_time);
    msg.action = msg.ACTION_ADD;
    msg.singul = data->singul_traj.empty() ? 1 : data->singul_traj.front().singul;
    for(unsigned int i = 0; i < data->singul_traj.size(); ++i){
      int piece_num = data->singul_traj[i].traj.getPieceNum();
      for (int j = 0; j < piece_num; ++j)
      {
        quadrotor_msgs::PolynomialMatrix piece;
        piece.num_dim = data->singul_traj[i].traj.getPiece(j).getDim();
        piece.num_order = data->singul_traj[i].traj.getPiece(j).getDegree();
        piece.duration = data->singul_traj[i].traj.getPiece(j).getDuration();
        auto cMat = data->singul_traj[i].traj.getPiece(j).getCoeffMat();
        piece.data.assign(cMat.data(),cMat.data() + cMat.rows()*cMat.cols());
        msg.trajectory.emplace_back(piece);
      }
    }
    if (!data->singul_traj.empty()) {
      const auto &last = data->singul_traj.back();
      ROS_WARN_STREAM("[Planner] send trajectory pieces=" << msg.trajectory.size()
                      << " end=" << last.traj.getPos(last.traj.getTotalDuration()).transpose());
    }
    poly_traj_pub_.publish(msg);

  }

  bool REMANIReplanFSM::planFromGlobalTraj(const int trial_times /*= 1*/){
    start_pos_ = mm_state_pos_;
    start_vel_ = mm_state_vel_;
    start_acc_.setZero();
    start_jer_.setZero();
    start_yaw_ = mm_car_yaw_;
    start_singul_ = mm_car_singul_;
    bool flag_random_poly_init;
    if(timesOfConsecutiveStateCalls().first == 1) flag_random_poly_init = false;
    else flag_random_poly_init = true;
    for(int i = 0; i < trial_times; i++){
      if(callReboundReplan(true, flag_random_poly_init)){
        return true;
      }
    }
    return false;
  }

  bool REMANIReplanFSM::planFromLocalTraj(bool flag_use_poly_init){
    SingulTrajData *info = &planner_manager_->traj_container_.singul_traj_data;
    double t_cur = ros::Time::now().toSec() - info->start_time + replan_trajectory_time_;
    t_cur = min(info->duration, t_cur);

    start_pos_     = info->getPos(t_cur);
    start_vel_    = info->getVel(t_cur);
    start_acc_    = info->getAcc(t_cur);
    start_jer_   = info->getJer(t_cur);
    start_singul_ = info->getSingul(t_cur);
    if(start_vel_.norm() >= mobile_base_non_singul_vel_) start_yaw_ = atan2(start_singul_ * start_vel_(1), start_singul_ * start_vel_(0));
    else start_yaw_ = mm_car_yaw_;

    bool success = callReboundReplan(flag_use_poly_init, false);
    if (!success){
      for (int i = 0; i < 1; i++){
        success = callReboundReplan(true, true);
        if (success)
          break;
      }
      if (!success)
      {
        return false;
      }
    }

    return true;
  }

  bool REMANIReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj){
    bool reach_horizon;
    planner_manager_->getLocalTarget(
        planning_horizen_, start_pos_, start_yaw_, end_pt_, end_yaw_,
        local_target_pt_, local_target_vel_, local_target_acc_, reach_horizon);
    bool local_target_gripper;
    // wpt_id_ is only assigned for preset targets.  Manual goals leave it at
    // its default, so guard the waypoint lookup instead of indexing an
    // uninitialized value into waypoint_gripper_close_.
    if(!reach_horizon && wpt_id_ >= 0 &&
       wpt_id_ < static_cast<int>(waypoint_gripper_close_.size())){
      local_target_gripper = waypoint_gripper_close_[wpt_id_];
    }else{
      local_target_gripper = gripper_state_;
    }
    local_target_acc_.setZero();
    double local_target_yaw = atan2(local_target_vel_(1), local_target_vel_(0)); // global traj is foreward, no need to take singul into account
    local_target_vel_.setZero();
    local_target_vel_.head(2) = mobile_base_non_singul_vel_ * Eigen::Vector2d(cos(local_target_yaw), sin(local_target_yaw));

    Eigen::VectorXd desired_start_pt, desired_start_vel, desired_start_acc, desired_start_jerk;
    int desired_start_singul;
    double desired_start_yaw;
    double desired_start_time, start_time_dura;
    
    if(have_local_traj_)
    {
      desired_start_time = ros::Time::now().toSec() + replan_trajectory_time_;
      start_time_dura = desired_start_time - planner_manager_->traj_container_.singul_traj_data.start_time;
      start_time_dura = min(start_time_dura, planner_manager_->traj_container_.singul_traj_data.duration);
      
      desired_start_pt = planner_manager_->traj_container_.singul_traj_data.getPos(start_time_dura);
      desired_start_vel = planner_manager_->traj_container_.singul_traj_data.getVel(start_time_dura);
      if(desired_start_vel.head(2).norm() < mobile_base_non_singul_vel_){
        desired_start_vel(0) = start_singul_ * mobile_base_non_singul_vel_ * cos(start_yaw_);
        desired_start_vel(1) = start_singul_ * mobile_base_non_singul_vel_ * sin(start_yaw_);
      }
      desired_start_singul = planner_manager_->traj_container_.singul_traj_data.getSingul(start_time_dura);
      desired_start_acc = planner_manager_->traj_container_.singul_traj_data.getAcc(start_time_dura);
      desired_start_jerk = planner_manager_->traj_container_.singul_traj_data.getJer(start_time_dura);
      desired_start_yaw = atan2(desired_start_singul * desired_start_vel(1), desired_start_singul * desired_start_vel(0));
    }else{
      desired_start_time = ros::Time::now().toSec();
      desired_start_pt = start_pos_;
      desired_start_vel = start_vel_;
      if(desired_start_vel.head(2).norm() < mobile_base_non_singul_vel_){
        desired_start_vel(0) = start_singul_ * mobile_base_non_singul_vel_ * cos(start_yaw_);
        desired_start_vel(1) = start_singul_ * mobile_base_non_singul_vel_ * sin(start_yaw_);
      }
      desired_start_acc = start_acc_;
      desired_start_jerk = start_jer_;
      desired_start_yaw = start_yaw_;
      desired_start_singul = start_singul_;
    }
    // std::cout << "desired_start_singul: " << desired_start_singul << std::endl;
    double init_time, opt_time;
    
    bool plan_success = planner_manager_->reboundReplan(
        desired_start_pt, desired_start_vel, desired_start_acc,desired_start_jerk, desired_start_yaw, desired_start_singul, gripper_state_,
        desired_start_time, local_target_pt_, local_target_vel_, local_target_acc_, local_target_yaw, local_target_gripper,
        (have_new_target_ || flag_use_poly_init),
        flag_randomPolyTraj, have_local_traj_, init_time, opt_time,
        // start_pos_ is a predicted replan state when a local trajectory is
        // active.  RViz must start at the measured robot state instead.
        mm_state_pos_, mm_car_yaw_);
    ROS_INFO("[PLAN_TIME] remain_frontend_ms=%.3f remain_optimizer_ms=%.3f remain_total_ms=%.3f success=%s",
             init_time, opt_time, init_time + opt_time, plan_success ? "true" : "false");
    have_new_target_ = false;

    if (plan_success){
      init_time_list_.push_back(init_time);
      opt_time_list_.push_back(opt_time);
      total_time_list_.push_back(init_time + opt_time);
      sendPolyTrajROSMsg();
      have_local_traj_ = true;

      // vis local traj
      int i_end = floor(planner_manager_->traj_container_.singul_traj_data.duration / 0.02);
      std::vector<Eigen::Vector2d> local_path_list;
      Eigen::Vector2d local_traj_pt;
      for(int i = 0; i < i_end; ++i){
        local_traj_pt = planner_manager_->traj_container_.singul_traj_data.getPos(i * 0.02).head(2);
        local_path_list.push_back(local_traj_pt);
      }
      visualization_->displayGlobalTraj(local_path_list, 0.05, 0);
      planner_manager_->ploy_traj_opt_->displayBackEndMesh(planner_manager_->traj_container_.singul_traj_data, false, gripper_state_);
    }

    return plan_success;
  }

  bool REMANIReplanFSM::callEmergencyStop(Eigen::VectorXd stop_pos, double stop_yaw, const int singul){
    std::cout << "\033[31mcall EmergencyStop\033[0m" << std::endl;
    planner_manager_->EmergencyStop(stop_pos, stop_yaw, singul);
    quadrotor_msgs::PolynomialTraj msg;
    msg.action = quadrotor_msgs::PolynomialTraj::ACTION_ABORT;
    poly_traj_pub_.publish(msg);

    return true;
  }

} // namespace remani_planner
