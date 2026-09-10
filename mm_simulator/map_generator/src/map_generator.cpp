#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <iostream>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Eigen>
#include <random>
#include <std_msgs/Int32.h>
#include <array>


using namespace std; 

pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeLocalMap;
random_device rd;
default_random_engine eng;
uniform_real_distribution<double> rand_x;
uniform_real_distribution<double> rand_y;
uniform_real_distribution<double> rand_z;

ros::Publisher _all_map_pub;

int _obs_num, _float_obs_num;
int _map_type;
double _x_size, _y_size, _z_size;
double _x_l, _x_h, _y_l, _y_h;
double _resolution, _pub_rate;
double _min_dist;
bool _add_test_bar = false;
bool _test_bar_only = false;
double _test_bar_x = -0.6, _test_bar_y = 0.0;
double _test_bar_length = 2.0, _test_bar_width = 0.40;
double _test_bar_bottom = 0.80, _test_bar_height = 1.00;

bool _map_ok = false;
bool _has_odom = false;

uniform_real_distribution<double> rand_z_;
uniform_real_distribution<double> rand_box_x_;
uniform_real_distribution<double> rand_box_y_;
uniform_real_distribution<double> rand_box_z_;

sensor_msgs::PointCloud2 globalMap_pcd;
pcl::PointCloud<pcl::PointXYZ> cloudMap;

struct OccupiedSphere {
  Eigen::Vector3d center;
  double radius;
};

double _mobile_base_length = 0.0;
double _mobile_base_width = 0.0;
double _mobile_base_height = 0.0;
double _mobile_base_check_radius = 0.0;
double _manipulator_thickness = 0.0;
int _manipulator_dof = 0;
bool _use_fast_armer = true;
double _init_yaw = 0.0;
Eigen::VectorXd _manipulator_config;
Eigen::VectorXd _init_joint_state;
Eigen::Matrix4d _base_to_mani = Eigen::Matrix4d::Identity();
vector<Eigen::Matrix4Xd> _manipulator_link_pts;
vector<OccupiedSphere> _initial_occupied_spheres;

Eigen::Matrix3d euler2rotation(double r, double p, double y) {
  Eigen::AngleAxisd roll_angle(r, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitch_angle(p, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yaw_angle(y, Eigen::Vector3d::UnitZ());
  return (yaw_angle * pitch_angle * roll_angle).toRotationMatrix();
}

void setLinkPoint() {
  _manipulator_link_pts.clear();
  Eigen::Matrix4Xd link_pts;
  if (_use_fast_armer) {
    for (int i = 0; i < _manipulator_dof; ++i) {
      switch (i) {
        case 0:
          link_pts.resize(4, 4);
          link_pts.col(0) = Eigen::Vector4d(0, 0, 0, 1);
          link_pts.col(1) = Eigen::Vector4d(0, 0, 0.05, 1);
          link_pts.col(2) = Eigen::Vector4d(0, 0, -0.05, 1);
          link_pts.col(3) = Eigen::Vector4d(0, -0.05, 0, 1);
          break;
        case 1:
          link_pts.resize(4, 6);
          link_pts.col(0) = Eigen::Vector4d(0, 0, 0.0, 1);
          link_pts.col(1) = Eigen::Vector4d(0.07, 0, 0.0, 1);
          link_pts.col(2) = Eigen::Vector4d(0.14, 0, 0.0, 1);
          link_pts.col(3) = Eigen::Vector4d(0.21, 0, 0.0, 1);
          link_pts.col(4) = Eigen::Vector4d(0.28, 0, 0.0, 1);
          link_pts.col(5) = Eigen::Vector4d(0.35, 0, 0.0, 1);
          break;
        case 2:
          link_pts.resize(4, 1);
          link_pts.col(0) = Eigen::Vector4d(0, 0, 0, 1);
          break;
        case 3:
          link_pts.resize(4, 5);
          link_pts.col(0) = Eigen::Vector4d(0.0, 0.07, 0.0, 1);
          link_pts.col(1) = Eigen::Vector4d(0.0, 0.14, 0.0, 1);
          link_pts.col(2) = Eigen::Vector4d(0.0, 0.21, 0.0, 1);
          link_pts.col(3) = Eigen::Vector4d(0.0, 0.28, 0.0, 1);
          link_pts.col(4) = Eigen::Vector4d(0.0, 0.35, 0.0, 1);
          break;
        case 4:
          link_pts.resize(4, 1);
          link_pts.col(0) = Eigen::Vector4d(0, 0, 0, 1);
          break;
        case 5:
          link_pts.resize(4, 7);
          link_pts.col(0) = Eigen::Vector4d(0, 0, -0.10, 1);
          link_pts.col(1) = Eigen::Vector4d(0, 0.03, -0.10, 1);
          link_pts.col(2) = Eigen::Vector4d(0, -0.03, -0.10, 1);
          link_pts.col(3) = Eigen::Vector4d(0, 0.05, -0.05, 1);
          link_pts.col(4) = Eigen::Vector4d(0, -0.05, -0.05, 1);
          link_pts.col(5) = Eigen::Vector4d(0, 0.06, 0.0, 1);
          link_pts.col(6) = Eigen::Vector4d(0, -0.06, 0.0, 1);
          break;
        default:
          break;
      }
      _manipulator_link_pts.push_back(link_pts);
    }
  } else {
    for (int i = 0; i < _manipulator_dof; ++i) {
      switch (i) {
        case 0:
          link_pts.resize(4, 2);
          link_pts.col(0) = Eigen::Vector4d(0, 0, 0, 1);
          link_pts.col(1) = Eigen::Vector4d(0, 0.05, 0, 1);
          break;
        case 1:
          link_pts.resize(4, 5);
          link_pts.col(0) = Eigen::Vector4d(0, 0, 0.14, 1);
          link_pts.col(1) = Eigen::Vector4d(-0.1, 0, 0.14, 1);
          link_pts.col(2) = Eigen::Vector4d(-0.2, 0, 0.14, 1);
          link_pts.col(3) = Eigen::Vector4d(-0.3, 0, 0.14, 1);
          link_pts.col(4) = Eigen::Vector4d(-0.4, 0, 0.14, 1);
          break;
        case 2:
          link_pts.resize(4, 5);
          link_pts.col(0) = Eigen::Vector4d(0, 0, 0, 1);
          link_pts.col(1) = Eigen::Vector4d(-0.1, 0, 0, 1);
          link_pts.col(2) = Eigen::Vector4d(-0.2, 0, 0, 1);
          link_pts.col(3) = Eigen::Vector4d(-0.3, 0, 0, 1);
          link_pts.col(4) = Eigen::Vector4d(-0.4, 0, 0, 1);
          break;
        default:
          link_pts.resize(4, 1);
          link_pts.col(0) = Eigen::Vector4d(0, 0, 0, 1);
          break;
      }
      _manipulator_link_pts.push_back(link_pts);
    }
  }
}

void getAJointTran(int joint_num, double theta, Eigen::Matrix4d &T) {
  const double sinTheta = sin(theta);
  const double cosTheta = cos(theta);
  const double linkLength = _manipulator_config(joint_num);
  T = Eigen::Matrix4d::Identity();
  if (_use_fast_armer) {
    switch (joint_num) {
      case 0:
        T(0, 0) = cosTheta; T(0, 2) = sinTheta;
        T(1, 0) = sinTheta; T(1, 1) = 0; T(1, 2) = -cosTheta;
        T(2, 1) = 1; T(2, 2) = 0; T(2, 3) = linkLength;
        break;
      case 1:
        T(0, 0) = cosTheta; T(0, 1) = -sinTheta; T(0, 3) = -linkLength * cosTheta;
        T(1, 0) = sinTheta; T(1, 1) = cosTheta;  T(1, 3) = -linkLength * sinTheta;
        break;
      case 2:
        T(0, 0) = -sinTheta; T(0, 2) = cosTheta; T(0, 3) = -linkLength * sinTheta;
        T(1, 0) = cosTheta;  T(1, 1) = 0.0;      T(1, 2) = sinTheta; T(1, 3) = linkLength * cosTheta;
        T(2, 1) = 1.0;       T(2, 2) = 0.0;
        break;
      case 3:
        T(0, 0) = cosTheta; T(0, 2) = -sinTheta;
        T(1, 0) = sinTheta; T(1, 1) = 0; T(1, 2) = cosTheta;
        T(2, 1) = -1; T(2, 2) = 0; T(2, 3) = linkLength;
        break;
      case 4:
        T(0, 0) = sinTheta;  T(0, 2) = cosTheta;
        T(1, 0) = -cosTheta; T(1, 1) = 0; T(1, 2) = sinTheta;
        T(2, 1) = -1; T(2, 2) = 0;
        break;
      case 5:
        T(0, 0) = cosTheta; T(0, 1) = -sinTheta;
        T(1, 0) = sinTheta; T(1, 1) = cosTheta;
        T(2, 3) = linkLength;
        break;
      default:
        break;
    }
  } else {
    if (joint_num == 0 || joint_num == 3) {
      T(0, 0) = cosTheta; T(0, 2) = -sinTheta;
      T(1, 0) = sinTheta; T(1, 1) = 0; T(1, 2) = cosTheta;
      T(2, 1) = -1; T(2, 2) = 0; T(2, 3) = linkLength;
    } else if (joint_num == 4) {
      T(0, 0) = cosTheta; T(0, 2) = sinTheta;
      T(1, 0) = sinTheta; T(1, 1) = 0; T(1, 2) = -cosTheta;
      T(2, 1) = 1; T(2, 2) = 0; T(2, 3) = linkLength;
    } else if (joint_num == 1 || joint_num == 2) {
      T(0, 0) = cosTheta; T(0, 1) = -sinTheta; T(0, 3) = linkLength * cosTheta;
      T(1, 0) = sinTheta; T(1, 1) = cosTheta;  T(1, 3) = linkLength * sinTheta;
    } else if (joint_num == 5) {
      T(0, 0) = cosTheta; T(0, 1) = -sinTheta;
      T(1, 0) = sinTheta; T(1, 1) = cosTheta;
      T(2, 3) = linkLength;
    }
  }
}

void appendBaseOccupiedSpheres(const Eigen::Vector3d& car_state) {
  Eigen::Matrix2d R;
  R << cos(car_state(2)), -sin(car_state(2)),
       sin(car_state(2)),  cos(car_state(2));
  const Eigen::Vector2d corner1 = car_state.head(2) + R * Eigen::Vector2d(_mobile_base_length / 2 - _mobile_base_check_radius,  _mobile_base_width / 2 - _mobile_base_check_radius);
  const Eigen::Vector2d corner2 = car_state.head(2) + R * Eigen::Vector2d(_mobile_base_length / 2 - _mobile_base_check_radius, -_mobile_base_width / 2 + _mobile_base_check_radius);
  const Eigen::Vector2d corner3 = car_state.head(2) + R * Eigen::Vector2d(-_mobile_base_length / 2 + _mobile_base_check_radius, -_mobile_base_width / 2 + _mobile_base_check_radius);
  const Eigen::Vector2d corner4 = car_state.head(2) + R * Eigen::Vector2d(-_mobile_base_length / 2 + _mobile_base_check_radius,  _mobile_base_width / 2 - _mobile_base_check_radius);
  const array<Eigen::Vector2d, 4> corners{corner1, corner2, corner3, corner4};
  const array<double, 4> norms{
      (corner2 - corner1).norm(),
      (corner3 - corner2).norm(),
      (corner4 - corner3).norm(),
      (corner1 - corner4).norm()};

  for (double height = _mobile_base_check_radius; height < _mobile_base_height; height += _mobile_base_check_radius) {
    for (const auto& corner : corners) {
      _initial_occupied_spheres.push_back({Eigen::Vector3d(corner.x(), corner.y(), height), _mobile_base_check_radius});
    }
    for (double dl = _mobile_base_check_radius; dl < norms[0]; dl += _mobile_base_check_radius) {
      const Eigen::Vector2d pt = dl / norms[0] * (corner2 - corner1) + corner1;
      _initial_occupied_spheres.push_back({Eigen::Vector3d(pt.x(), pt.y(), height), _mobile_base_check_radius});
    }
    for (double dl = _mobile_base_check_radius; dl < norms[1]; dl += _mobile_base_check_radius) {
      const Eigen::Vector2d pt = dl / norms[1] * (corner3 - corner2) + corner2;
      _initial_occupied_spheres.push_back({Eigen::Vector3d(pt.x(), pt.y(), height), _mobile_base_check_radius});
    }
    for (double dl = _mobile_base_check_radius; dl < norms[2]; dl += _mobile_base_check_radius) {
      const Eigen::Vector2d pt = dl / norms[2] * (corner4 - corner3) + corner3;
      _initial_occupied_spheres.push_back({Eigen::Vector3d(pt.x(), pt.y(), height), _mobile_base_check_radius});
    }
    for (double dl = _mobile_base_check_radius; dl < norms[3]; dl += _mobile_base_check_radius) {
      const Eigen::Vector2d pt = dl / norms[3] * (corner1 - corner4) + corner4;
      _initial_occupied_spheres.push_back({Eigen::Vector3d(pt.x(), pt.y(), height), _mobile_base_check_radius});
    }
  }
}

void initRobotOccupiedSpheres(ros::NodeHandle& n) {
  _initial_occupied_spheres.clear();

  n.param("mm/mobile_base_length", _mobile_base_length, 0.0);
  n.param("mm/mobile_base_width", _mobile_base_width, 0.0);
  n.param("mm/mobile_base_height", _mobile_base_height, 0.0);
  n.param("mm/mobile_base_check_radius", _mobile_base_check_radius, 0.0);
  n.param("mm/manipulator_thickness", _manipulator_thickness, 0.0);
  n.param("mm/manipulator_dof", _manipulator_dof, 0);
  n.param("mm/use_fast_armer", _use_fast_armer, true);
  vector<double> manipulator_config;
  vector<double> base_mani_fixed_joint_xyz_ypr{0.03, -0.02, _mobile_base_height, 0.0, 0.0, 0.0};
  vector<double> init_state;
  n.getParam("mm/manipulator_config", manipulator_config);
  n.param<vector<double>>("mm/base_mani_fixed_joint_xyz_ypr", base_mani_fixed_joint_xyz_ypr, base_mani_fixed_joint_xyz_ypr);
  n.getParam("fsm/init_state", init_state);
  n.param("fsm/init_yaw", _init_yaw, 0.0);

  if (_manipulator_dof <= 0 || init_state.size() < static_cast<size_t>(2 + _manipulator_dof) || manipulator_config.size() < static_cast<size_t>(_manipulator_dof)) {
    return;
  }

  _manipulator_config.resize(_manipulator_dof);
  _init_joint_state.resize(_manipulator_dof);
  for (int i = 0; i < _manipulator_dof; ++i) {
    _manipulator_config(i) = manipulator_config[i];
    _init_joint_state(i) = init_state[i + 2] * M_PI / 180.0;
  }
  _init_yaw = _init_yaw / 180.0 * M_PI;

  _base_to_mani = Eigen::Matrix4d::Identity();
  _base_to_mani.block(0, 0, 3, 3) = euler2rotation(base_mani_fixed_joint_xyz_ypr[5], base_mani_fixed_joint_xyz_ypr[4], base_mani_fixed_joint_xyz_ypr[3]);
  _base_to_mani(0, 3) = base_mani_fixed_joint_xyz_ypr[0];
  _base_to_mani(1, 3) = base_mani_fixed_joint_xyz_ypr[1];
  _base_to_mani(2, 3) = base_mani_fixed_joint_xyz_ypr[2];

  setLinkPoint();

  const Eigen::Vector3d car_state(init_state[0], init_state[1], _init_yaw);
  appendBaseOccupiedSpheres(car_state);

  Eigen::Matrix4d world_from_base = Eigen::Matrix4d::Identity();
  world_from_base(0, 0) = cos(_init_yaw);
  world_from_base(0, 1) = -sin(_init_yaw);
  world_from_base(1, 0) = sin(_init_yaw);
  world_from_base(1, 1) = cos(_init_yaw);
  world_from_base(0, 3) = init_state[0];
  world_from_base(1, 3) = init_state[1];

  Eigen::Matrix4d T_now = world_from_base * _base_to_mani;
  for (int i = 0; i < _manipulator_dof; ++i) {
    Eigen::Matrix4d T_joint;
    getAJointTran(i, _init_joint_state(i), T_joint);
    T_now = T_now * T_joint;
    const int pts_size = _manipulator_link_pts[i].cols();
    for (int j = 0; j < pts_size; ++j) {
      const Eigen::Vector3d pt = (T_now * _manipulator_link_pts[i].col(j)).head(3);
      _initial_occupied_spheres.push_back({pt, _manipulator_thickness});
    }
  }
}

bool boxCollidesWithInitialRobot(const Eigen::Vector3d& center, const Eigen::Vector3d& box_size) {
  for (const auto& sphere : _initial_occupied_spheres) {
    const Eigen::Vector3d delta = (sphere.center - center).cwiseAbs();
    if (delta.x() <= box_size.x() / 2.0 + sphere.radius &&
        delta.y() <= box_size.y() / 2.0 + sphere.radius &&
        delta.z() <= box_size.z() / 2.0 + sphere.radius) {
      return true;
    }
  }
  return false;
}

void GenerateWall(double x_l, double x_h, 
                  double y_l, double y_h, 
                  double z_l, double z_h, 
                  pcl::PointCloud<pcl::PointXYZ>& cloudMap){
  int x_num, y_num, z_num;
  double resolution = _resolution / 2.0;
  x_num = ceil((x_h - x_l)/resolution);
  y_num = ceil((y_h - y_l)/resolution);
  z_num = ceil((z_h - z_l)/resolution);
  pcl::PointXYZ pt;
  for (int i=0; i<x_num; i++)
    for (int j=0; j<y_num; j++)
      for (int k=0; k<z_num; k++){
        pt.x = x_l + i * resolution;
        pt.y = y_l + j * resolution;
        pt.z = z_l + k * resolution;
        cloudMap.points.push_back(pt);
      }
}

void GenerateBox(Eigen::Vector3d pos, Eigen::Vector3d box_size, pcl::PointCloud<pcl::PointXYZ>& cloudMap){
  GenerateWall(pos(0) - box_size(0) / 2, pos(0) + box_size(0) / 2, 
               pos(1) - box_size(1) / 2, pos(1) + box_size(1) / 2,
               pos(2) - box_size(2) / 2, pos(2) + box_size(2) / 2, 
               cloudMap);
}

void GenerateWall(Eigen::Vector3d pos, double theta, Eigen::Vector3d wall_size, pcl::PointCloud<pcl::PointXYZ>& cloudMap){
  Eigen::Vector2d dirx(cos(theta), sin(theta));
  Eigen::Vector2d diry(cos(theta + M_PI_2), sin(theta + M_PI_2));
  double resolution = _resolution / 3.0;
  int x_num = ceil(wall_size(0) / resolution);
  int y_num = ceil(wall_size(1) / resolution);
  int z_num = ceil(wall_size(2) / resolution);
  Eigen::Vector2d posx;
  Eigen::Vector2d posy;
  pcl::PointXYZ pt;
  for (int i=0; i<x_num; i++){
    posx = pos.head(2) + (double)i * resolution * dirx;
    pt.x = posx(0);
    for (int j=0; j<y_num; j++){
      posy = posx + (double)j * resolution * diry;
      pt.y = posy(1);
      for (int k=0; k<z_num; k++){
        pt.z = pos(2) + (double)k * resolution;
        cloudMap.points.push_back(pt);
      }
    }
  }
}

void GenerateBridge(){
  pcl::PointXYZ pt_random;
  vector<Eigen::Vector2d> obs_position, obs_size;
  double x, y;
  Eigen::Vector3d bridge_size, box_size;
  Eigen::Vector2d bridge_pos;
  bridge_size << 0.8, 1.8, 0.4;
  box_size << 0.2, 0.2, 0.2;
  int bridge_num = 1;

  rand_x = uniform_real_distribution<double>(_x_l * 0.75, _x_h * 0.75);
  rand_y = uniform_real_distribution<double>(_y_l, _y_h);
  rand_z_ = uniform_real_distribution<double>(-0.1, 0.1);

  rand_box_x_ = uniform_real_distribution<double>(0.1, 0.8);
  rand_box_y_ = uniform_real_distribution<double>(0.1, 0.8);
  rand_box_z_ = uniform_real_distribution<double>(0.8, 3.0);

  // generate wall
  GenerateWall(_x_l, _x_h, _y_l, _y_l + _resolution, 0.0, 0.2, cloudMap);
  GenerateWall(_x_l, _x_h, _y_h - _resolution, _y_h, 0.0, 0.2, cloudMap);
  GenerateWall(_x_l, _x_l + _resolution, _y_l, _y_h, 0.0, 0.2, cloudMap);
  GenerateWall(_x_h - _resolution, _x_h, _y_l, _y_h, 0.0, 0.2, cloudMap);

  GenerateWall(-_resolution, _resolution, _y_l, -0.75, 0.0, 0.2, cloudMap);
  GenerateWall(-_resolution, _resolution, 0.75, _y_h, 0.0, 0.2, cloudMap);

  std::vector<Eigen::Vector2d> bridge_pos_list;
  std::vector<Eigen::Vector3d> bridge_size_list;
  for(int i = 0; i < bridge_num; ++i){
    switch(i){
      case 0 :{
        bridge_pos << 0.0, 0.0;
        bridge_size << 0.6, 1.5, 0.7;
        break;
      }
    }
    bridge_pos_list.push_back(bridge_pos);
    bridge_size_list.push_back(bridge_size);
  }

  for (int i = 0; i < bridge_num && ros::ok(); i++){
    x = bridge_pos_list[i](0);
    y = bridge_pos_list[i](1);
    bridge_size = bridge_size_list[i];

    obs_position.push_back(Eigen::Vector2d(x, y));

    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;

    auto pos = Eigen::Vector2d(x, y);
      // board
    GenerateWall(pos(0) - bridge_size(0) / 2.0, pos(0) + bridge_size(0) / 2.0,
                pos(1) - bridge_size(1) / 2.0, pos(1) + bridge_size(1) / 2.0,
                bridge_size(2), bridge_size(2) + 1e-3 + 2 * _resolution,
                cloudMap);
  // feet
    std::vector<Eigen::Vector2d> corner_list;
    Eigen::Vector2d corner;
    corner << pos(0) - bridge_size(0) / 2, pos(1) - bridge_size(1) / 2;
    corner_list.push_back(corner);
    corner << pos(0) - bridge_size(0) / 2, pos(1) + bridge_size(1) / 2;
    corner_list.push_back(corner);
    corner << pos(0) + bridge_size(0) / 2, pos(1) - bridge_size(1) / 2;
    corner_list.push_back(corner);
    corner << pos(0) + bridge_size(0) / 2, pos(1) + bridge_size(1) / 2;
    corner_list.push_back(corner);

    GenerateWall(pos(0) - bridge_size(0) / 2, pos(0) + bridge_size(0) / 2, 
                pos(1) - bridge_size(1) / 2, pos(1) - bridge_size(1) / 2 + 2 * _resolution,
                0.0, bridge_size(2), cloudMap);
    GenerateWall(pos(0) - bridge_size(0) / 2, pos(0) + bridge_size(0) / 2 , 
                pos(1) + bridge_size(1) / 2 - 2 * _resolution, pos(1) + bridge_size(1) / 2,
                0.0, bridge_size(2), cloudMap);
  }

  GenerateWall(_x_l, _x_h, _y_l, _y_l + _resolution, 0.0, 0.2, cloudMap);
  GenerateWall(_x_l, _x_h, _y_h - _resolution, _y_h, 0.0, 0.2, cloudMap);
  GenerateWall(_x_l, _x_l + _resolution, _y_l, _y_h, 0.0, 0.2, cloudMap);
  GenerateWall(_x_h - _resolution, _x_h, _y_l, _y_h, 0.0, 0.2, cloudMap);

  GenerateBox(Eigen::Vector3d(_x_l + _resolution, _y_l + _resolution, 0.9), Eigen::Vector3d(0.1, 0.1, 1.8), cloudMap);
  GenerateBox(Eigen::Vector3d(_x_l + _resolution, _y_h - _resolution, 0.9), Eigen::Vector3d(0.1, 0.1, 1.8), cloudMap);
  GenerateBox(Eigen::Vector3d(_x_h - _resolution, _y_l + _resolution, 0.9), Eigen::Vector3d(0.1, 0.1, 1.8), cloudMap);
  GenerateBox(Eigen::Vector3d(_x_h - _resolution, _y_h - _resolution, 0.9), Eigen::Vector3d(0.1, 0.1, 1.8), cloudMap);

  cloudMap.width = cloudMap.points.size();
  cloudMap.height = 1;
  cloudMap.is_dense = true;


  kdtreeLocalMap.setInputCloud(cloudMap.makeShared());
  _map_ok = true;
  ROS_WARN("Finished generate Bridge Map ");
}

void GenerateCuboids(){
    pcl::PointXYZ pt_random;
  vector<Eigen::Vector2d> obs_position, obs_size;
  double x, y, z;
  Eigen::Vector3d bridge_size, box_size;
  Eigen::Vector2d bridge_pos;
  bridge_size << 0.8, 1.8, 0.4;
  box_size << 0.2, 0.2, 0.2;
  int bridge_num = 0;

  if (_test_bar_only) {
    GenerateBox(Eigen::Vector3d(_test_bar_x, _test_bar_y,
                                _test_bar_bottom + _test_bar_height / 2.0),
                Eigen::Vector3d(_test_bar_width, _test_bar_length,
                                _test_bar_height), cloudMap);
    cloudMap.width = cloudMap.points.size();
    cloudMap.height = 1;
    cloudMap.is_dense = true;
    kdtreeLocalMap.setInputCloud(cloudMap.makeShared());
    _map_ok = true;
    ROS_WARN("Generated test-bar-only map");
    return;
  }

  std::vector<Eigen::Vector2d> bridge_pos_list;
  std::vector<Eigen::Vector3d> bridge_size_list;
  for(int i = 0; i < bridge_num; ++i){
    switch(i){
      case 0 :{
        bridge_pos << 4.7, 2.2;
        bridge_size << 0.8, 1.8, 0.4;
        break;
      }
    }
    bridge_pos_list.push_back(bridge_pos);
    bridge_size_list.push_back(bridge_size);
  }


  rand_x = uniform_real_distribution<double>(_x_l * 0.75, _x_h * 0.75);
  rand_y = uniform_real_distribution<double>(_y_l, _y_h);
  rand_z_ = uniform_real_distribution<double>(-0.1, 0.1);

  auto rand_float_x = uniform_real_distribution<double>(_x_l * 0.6, _x_h * 0.6);
  auto rand_float_y = uniform_real_distribution<double>(_y_l, _y_h);
  auto rand_float_z = uniform_real_distribution<double>(0.6, 1.1);


  rand_box_x_ = uniform_real_distribution<double>(0.1, 0.8);
  rand_box_y_ = uniform_real_distribution<double>(0.1, 0.8);
  rand_box_z_ = uniform_real_distribution<double>(0.8, 3.0);

  auto rand_float_box_x = uniform_real_distribution<double>(0.3, 0.6);
  auto rand_float_box_y = uniform_real_distribution<double>(0.3, 0.6);
  auto rand_float_box_z = uniform_real_distribution<double>(0.3, 0.6);

  // generate wall
  GenerateWall(_x_l, _x_h, _y_l, _y_l + _resolution, 0.0, 0.2, cloudMap);
  GenerateWall(_x_l, _x_h, _y_h - _resolution, _y_h, 0.0, 0.2, cloudMap);
  GenerateWall(_x_l, _x_l + _resolution, _y_l, _y_h, 0.0, 0.2, cloudMap);
  GenerateWall(_x_h - _resolution, _x_h, _y_l, _y_h, 0.0, 0.2, cloudMap);

  // Fixed horizontal bar for arm-obstacle avoidance testing.
  if (_add_test_bar) {
    GenerateBox(Eigen::Vector3d(_test_bar_x, _test_bar_y,
                                _test_bar_bottom + _test_bar_height / 2.0),
                Eigen::Vector3d(_test_bar_width, _test_bar_length,
                                _test_bar_height), cloudMap);
  }

  // generate random box
  obs_position.clear();
  obs_size.clear();
  for (int i = 0; i < _obs_num && ros::ok(); ++i){
    x = rand_x(eng);
    y = rand_y(eng);
    z = rand_z_(eng);
    double size_x = rand_box_x_(eng);
    double size_y = rand_box_y_(eng);
    double size_z = rand_box_z_(eng);

    box_size << size_x, size_y, size_z;

    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;
    z = floor(z / _resolution) * _resolution + _resolution / 2.0;

    bool flag_continue = false;
    if (boxCollidesWithInitialRobot(Eigen::Vector3d(x, y, z), box_size)) {
      i--;
      continue;
    }
    for (auto p : obs_position)
      if ((Eigen::Vector2d(x, y) - p).norm() < _min_dist /*metres*/)
      {
        i--;
        flag_continue = true;
        break;
      }
    if (flag_continue)
      continue;

    obs_position.push_back(Eigen::Vector2d(x, y));
    obs_size.push_back(Eigen::Vector2d(size_x, size_y));

    GenerateBox(Eigen::Vector3d(x, y, z), box_size, cloudMap);
  }

  obs_position.clear();
  obs_size.clear();
  for (int i = 0; i < _float_obs_num && ros::ok(); ++i){
    x = rand_float_x(eng);
    y = rand_float_y(eng);
    z = rand_float_z(eng);
    double size_x = rand_float_box_x(eng);
    double size_y = rand_float_box_y(eng);
    double size_z = rand_float_box_z(eng);

    box_size << size_x, size_y, size_z;

    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;
    z = floor(z / _resolution) * _resolution + _resolution / 2.0;

    bool flag_continue = false;
    if (boxCollidesWithInitialRobot(Eigen::Vector3d(x, y, z), box_size)) {
      i--;
      continue;
    }
    for (auto p : obs_position)
      if ((Eigen::Vector2d(x, y) - p).norm() < 1.0 /*metres*/)
      {
        i--;
        flag_continue = true;
        break;
      }
    if (flag_continue)
      continue;

    obs_position.push_back(Eigen::Vector2d(x, y));
    obs_size.push_back(Eigen::Vector2d(size_x, size_y));

    GenerateBox(Eigen::Vector3d(x, y, z), box_size, cloudMap);
  }

  GenerateBox(Eigen::Vector3d(_x_l + _resolution, _y_l + _resolution, 0.9), Eigen::Vector3d(0.1, 0.1, 1.8), cloudMap);
  GenerateBox(Eigen::Vector3d(_x_l + _resolution, _y_h - _resolution, 0.9), Eigen::Vector3d(0.1, 0.1, 1.8), cloudMap);
  GenerateBox(Eigen::Vector3d(_x_h - _resolution, _y_l + _resolution, 0.9), Eigen::Vector3d(0.1, 0.1, 1.8), cloudMap);
  GenerateBox(Eigen::Vector3d(_x_h - _resolution, _y_h - _resolution, 0.9), Eigen::Vector3d(0.1, 0.1, 1.8), cloudMap);

  cloudMap.width = cloudMap.points.size();
  cloudMap.height = 1;
  cloudMap.is_dense = true;


  kdtreeLocalMap.setInputCloud(cloudMap.makeShared());
  _map_ok = true;
  ROS_WARN("Finished generate Cuboids Map ");
}

void pubPoints(){
  while (ros::ok())
  {
    ros::spinOnce();
    if (_map_ok)
      break;
  }
  pcl::toROSMsg(cloudMap, globalMap_pcd);
  globalMap_pcd.header.frame_id = "world";
  _all_map_pub.publish(globalMap_pcd);
}

int main(int argc, char **argv){
  
  ros::init(argc, argv, "random_map_sensing");
  ros::NodeHandle n("~");

  _all_map_pub = n.advertise<sensor_msgs::PointCloud2>("/map_generator/global_cloud", 1);

  n.param("map/x_size", _x_size, 50.0);
  n.param("map/y_size", _y_size, 50.0);
  n.param("map/z_size", _z_size, 5.0);
  n.param("map/obs_num", _obs_num, 30);
  n.param("map/float_obs_num", _float_obs_num, 30);
  n.param("map/map_type", _map_type, 0);
  n.param("map/resolution", _resolution, 0.1);
  n.param("map/add_test_bar", _add_test_bar, false);
  n.param("map/test_bar_only", _test_bar_only, false);
  n.param("map/test_bar_x", _test_bar_x, -0.6);
  n.param("map/test_bar_y", _test_bar_y, 0.0);
  n.param("map/test_bar_length", _test_bar_length, 2.0);
  n.param("map/test_bar_width", _test_bar_width, 0.40);
  n.param("map/test_bar_bottom", _test_bar_bottom, 0.80);
  n.param("map/test_bar_height", _test_bar_height, 1.00);

  int seed = 0;
  n.param("map/seed", seed, 0);
  if(seed == 0){seed = rd();} 
  eng.seed(seed);

  n.param("pub_rate", _pub_rate, 10.0);
  n.param("min_distance", _min_dist, 1.0);
  initRobotOccupiedSpheres(n);

  _x_l = -_x_size / 2.0;
  _x_h = +_x_size / 2.0;

  _y_l = -_y_size / 2.0;
  _y_h = +_y_size / 2.0;

  _obs_num = min(_obs_num, (int)_x_size * 10);
  
  if(_map_type == 0){
    GenerateCuboids();
  }else if(_map_type == 1){
    GenerateBridge();
  }

  ros::Rate loop_rate(_pub_rate);
  while (ros::ok()){
    pubPoints();   
    ros::spinOnce();
    loop_rate.sleep();
  }

}
