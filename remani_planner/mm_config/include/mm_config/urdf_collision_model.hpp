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
            std::string& error);
  const std::vector<Eigen::Vector3d>& linkSamples(const std::string& link) const;
  const std::vector<Sphere>& linkSpheres(const std::string& link) const;
  std::vector<std::string> linkNames() const;

 private:
  std::map<std::string, std::vector<Eigen::Vector3d>> samples_;
  std::map<std::string, std::vector<Sphere>> spheres_;
  std::vector<Eigen::Vector3d> empty_;
  std::vector<Sphere> empty_spheres_;
};

}  // namespace remani_planner
