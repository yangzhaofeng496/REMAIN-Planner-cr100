#include "mm_config/urdf_collision_model.hpp"

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <ros/package.h>
#include <urdf/model.h>

#include <cmath>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <sys/stat.h>

namespace remani_planner {
namespace {

constexpr size_t kMaxPlanningSamplesPerLink = 800;

bool resolveMesh(const std::string& uri, std::string& path) {
  const std::string prefix = "package://";
  if (uri.compare(0, prefix.size(), prefix) == 0) {
    const std::string rest = uri.substr(prefix.size());
    const size_t slash = rest.find('/');
    if (slash == std::string::npos) return false;
    const std::string pkg = rest.substr(0, slash);
    const std::string base = ros::package::getPath(pkg);
    if (base.empty()) return false;
    path = base + "/" + rest.substr(slash + 1);
    return true;
  }
  path = uri.compare(0, 7, "file://") == 0 ? uri.substr(7) : uri;
  return true;
}

void addTriangleSamples(const aiVector3D& a, const aiVector3D& b,
                        const aiVector3D& c, double resolution,
                        std::vector<Eigen::Vector3d>& out) {
  const Eigen::Vector3d pa(a.x, a.y, a.z), pb(b.x, b.y, b.z), pc(c.x, c.y, c.z);
  const double longest = std::max({(pa-pb).norm(), (pb-pc).norm(), (pc-pa).norm()});
  const int n = std::max(1, static_cast<int>(std::ceil(longest / resolution)));
  for (int i = 0; i <= n; ++i) {
    for (int j = 0; j <= n - i; ++j) {
      const double u = static_cast<double>(i) / n;
      const double v = static_cast<double>(j) / n;
      out.push_back((1.0-u-v)*pa + u*pb + v*pc);
    }
  }
}

struct SphereConfig { int count = 3; double scale = 1.0; };

bool readSphereConfig(const std::string& path,
                      std::map<std::string, SphereConfig>& config,
                      std::string& error) {
  std::ifstream in(path);
  if (!in) { error = "cannot open sphere config: " + path; return false; }
  std::string line, link;
  while (std::getline(in, line)) {
    const size_t hash = line.find('#');
    if (hash != std::string::npos) line.resize(hash);
    const size_t first = line.find_first_not_of(" \t");
    if (first == std::string::npos) continue;
    const size_t colon = line.find(':', first);
    if (colon == std::string::npos) continue;
    const std::string key = line.substr(first, colon - first);
    const std::string value = line.substr(colon + 1);
    if (first == 2 && key != "links") link = key;
    else if (first >= 4 && !link.empty()) {
      std::istringstream parser(value);
      if (key == "count") parser >> config[link].count;
      else if (key == "scale") parser >> config[link].scale;
    }
  }
  for (const auto& item : config) {
    if (item.second.count < 1 || item.second.count > 32 ||
        !(item.second.scale > 0.0) || !std::isfinite(item.second.scale)) {
      error = "invalid sphere config for " + item.first;
      return false;
    }
  }
  return true;
}

long long fileMtime(const std::string& path) {
  struct stat st;
  if (stat(path.c_str(), &st) != 0) return -1;
  return static_cast<long long>(st.st_mtim.tv_sec) * 1000000000LL + st.st_mtim.tv_nsec;
}

}  // namespace

bool UrdfCollisionModel::load(const std::string& description, double resolution,
                               std::string& error, const std::string& sphere_config_path) {
  description_ = description;
  resolution_ = resolution;
  sphere_config_path_ = sphere_config_path;
  sphere_config_mtime_ = fileMtime(sphere_config_path_);
  std::map<std::string, SphereConfig> sphere_config;
  if (!sphere_config_path_.empty() && !readSphereConfig(sphere_config_path_, sphere_config, error))
    return false;
  samples_.clear();
  spheres_.clear();
  if (resolution <= 0.0) { error = "collision mesh resolution must be positive"; return false; }
  urdf::Model model;
  if (!model.initString(description)) { error = "robot_description is not a valid URDF"; return false; }
  for (const auto& item : model.links_) {
    const std::string& link_name = item.first;
    const urdf::LinkConstSharedPtr& link = item.second;
    if (!link->collision || !link->collision->geometry ||
        link->collision->geometry->type != urdf::Geometry::MESH) continue;
    const auto mesh = std::dynamic_pointer_cast<urdf::Mesh>(link->collision->geometry);
    std::string path;
    if (!resolveMesh(mesh->filename, path)) { error = "cannot resolve mesh for " + link_name + ": " + mesh->filename; return false; }
    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(path, aiProcess_Triangulate | aiProcess_JoinIdenticalVertices);
    if (!scene || !scene->HasMeshes()) { error = "cannot load mesh for " + link_name + ": " + path; return false; }
    auto& out = samples_[link_name];
    const Eigen::Matrix3d collision_rotation = Eigen::Quaterniond(
        link->collision->origin.rotation.w,
        link->collision->origin.rotation.x,
        link->collision->origin.rotation.y,
        link->collision->origin.rotation.z).normalized().toRotationMatrix();
    const Eigen::Vector3d collision_translation(
        link->collision->origin.position.x,
        link->collision->origin.position.y,
        link->collision->origin.position.z);
    const size_t before = out.size();
    for (unsigned int mi = 0; mi < scene->mNumMeshes; ++mi) {
      const aiMesh* m = scene->mMeshes[mi];
      for (unsigned int fi = 0; fi < m->mNumFaces; ++fi) {
        const aiFace& f = m->mFaces[fi];
        if (f.mNumIndices != 3) continue;
        addTriangleSamples(m->mVertices[f.mIndices[0]], m->mVertices[f.mIndices[1]],
                           m->mVertices[f.mIndices[2]], resolution, out);
      }
    }
    for (size_t i = before; i < out.size(); ++i)
      out[i] = collision_rotation * out[i] + collision_translation;
    if (out.empty()) { error = "collision mesh has no triangles for " + link_name; return false; }
    // Collision checks compare samples from two links.  Bound the per-link
    // cloud so mesh complexity cannot turn each planner query into an
    // effectively quadratic multi-million-point operation.
    if (out.size() > kMaxPlanningSamplesPerLink) {
      std::vector<Eigen::Vector3d> reduced;
      reduced.reserve(kMaxPlanningSamplesPerLink);
      const double step = static_cast<double>(out.size()) /
                          static_cast<double>(kMaxPlanningSamplesPerLink);
      for (size_t i = 0; i < kMaxPlanningSamplesPerLink; ++i)
        reduced.push_back(out[static_cast<size_t>(i * step)]);
      out.swap(reduced);
    }
    // Compact broad-phase representation for optimization: spheres along the
    // link's longest local axis.  Link2 and Link3 use four spheres so their
    // displayed inflated models match the four collision spheres requested.
    // Mesh samples remain authoritative for the final safety check.
    Eigen::Vector3d lo = out.front(), hi = out.front();
    for (const auto &p : out) { lo = lo.cwiseMin(p); hi = hi.cwiseMax(p); }
    const Eigen::Vector3d ext = hi - lo;
    int axis = 0; if (ext.y() > ext.x() && ext.y() >= ext.z()) axis = 1;
    else if (ext.z() > ext.x() && ext.z() > ext.y()) axis = 2;
    const double along = ext(axis);
    auto &ss = spheres_[link_name];
    const auto config_it = sphere_config.find(link_name);
    const SphereConfig config = config_it == sphere_config.end() ? SphereConfig() : config_it->second;
    const int sphere_count = config.count;
    for (int si = 0; si < sphere_count; ++si) {
      const double u = (si + 0.5) / sphere_count;
      Eigen::Vector3d c = lo + 0.5 * ext;
      c(axis) = lo(axis) + u * along;
      const double low = lo(axis) + (static_cast<double>(si) / sphere_count) * along;
      const double high = lo(axis) + (static_cast<double>(si + 1) / sphere_count) * along;
      double radius = 0.0;
      for (const auto &p : out) {
        if (p(axis) >= low && (si == sphere_count - 1 || p(axis) < high))
          radius = std::max(radius, (p - c).norm());
      }
      // The URDF mesh samples already describe the link surface.  Keep only
      // a small discretization margin; a larger padding creates false
      // self-collisions between non-adjacent links (notably Link3/Link5).
      double sphere_radius = std::max(0.025, radius + 0.005);
      sphere_radius *= config.scale;
      ss.push_back({c, sphere_radius});
    }
  }
  if (samples_.empty()) { error = "robot_description contains no mesh collision geometry"; return false; }
  return true;
}

bool UrdfCollisionModel::reloadSphereConfigIfChanged(std::string& error) {
  if (sphere_config_path_.empty()) return false;
  const long long current_mtime = fileMtime(sphere_config_path_);
  if (current_mtime < 0 || current_mtime == sphere_config_mtime_) return false;
  UrdfCollisionModel candidate;
  if (!candidate.load(description_, resolution_, error, sphere_config_path_)) return false;
  *this = std::move(candidate);
  return true;
}

const std::vector<Eigen::Vector3d>& UrdfCollisionModel::linkSamples(const std::string& link) const {
  auto it = samples_.find(link);
  return it == samples_.end() ? empty_ : it->second;
}

const std::vector<UrdfCollisionModel::Sphere>& UrdfCollisionModel::linkSpheres(const std::string& link) const {
  auto it = spheres_.find(link);
  return it == spheres_.end() ? empty_spheres_ : it->second;
}

std::vector<std::string> UrdfCollisionModel::linkNames() const {
  std::vector<std::string> names;
  for (const auto& item : samples_) names.push_back(item.first);
  return names;
}

}  // namespace remani_planner
