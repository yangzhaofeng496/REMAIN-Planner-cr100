#pragma once

#include <Eigen/Eigen>
#include <map>
#include <string>
#include <vector>

namespace remani_planner {

class UrdfCollisionModel {
 public:
  struct Sphere { Eigen::Vector3d center; double radius; };
  bool load(const std::string& robot_description, double resolution,
            std::string& error, const std::string& sphere_config_path = "");
  bool reloadSphereConfigIfChanged(std::string& error);
  const std::vector<Eigen::Vector3d>& linkSamples(const std::string& link) const;
  const std::vector<Sphere>& linkSpheres(const std::string& link) const;
  std::vector<std::string> linkNames() const;

 private:
  std::map<std::string, std::vector<Eigen::Vector3d>> samples_;
  std::map<std::string, std::vector<Sphere>> spheres_;
  std::vector<Eigen::Vector3d> empty_;
  std::vector<Sphere> empty_spheres_;
  std::string description_;
  std::string sphere_config_path_;
  double resolution_ = 0.03;
  long long sphere_config_mtime_ = -1;
};

}  // namespace remani_planner
