#include <glim/util/pose_file_loader.hpp>

#include <fstream>
#include <sstream>
#include <algorithm>
#include <spdlog/spdlog.h>

namespace glim {

Eigen::Isometry3d ExternalPose::pose() const {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = position;
  T.linear() = orientation.normalized().toRotationMatrix();
  return T;
}

bool PoseFileLoader::load(const std::string& pose_file_path) {
  poses_.clear();

  std::ifstream ifs(pose_file_path);
  if (!ifs.is_open()) {
    spdlog::error("Failed to open pose file: {}", pose_file_path);
    return false;
  }

  std::string line;
  int line_count = 0;
  int valid_count = 0;

  while (std::getline(ifs, line)) {
    line_count++;

    // Skip empty lines and comments
    if (line.empty() || line[0] == '#') {
      continue;
    }

    std::istringstream iss(line);
    ExternalPose pose;

    double tx, ty, tz, qw, qx, qy, qz, lidar_ts, img_ts;

    // FAST-LIVO2 format: tx ty tz qw qx qy qz lidar_ts img_ts
    if (!(iss >> tx >> ty >> tz >> qw >> qx >> qy >> qz >> lidar_ts >> img_ts)) {
      spdlog::warn("Failed to parse line {}: {}", line_count, line);
      continue;
    }

    pose.position = Eigen::Vector3d(tx, ty, tz);
    pose.orientation = Eigen::Quaterniond(qw, qx, qy, qz);
    pose.lidar_timestamp = lidar_ts;
    pose.image_timestamp = img_ts;

    // Validate quaternion
    if (std::abs(pose.orientation.norm() - 1.0) > 0.01) {
      spdlog::warn("Line {}: Quaternion not normalized (norm={}), normalizing...",
                   line_count, pose.orientation.norm());
      pose.orientation.normalize();
    }

    poses_.push_back(pose);
    valid_count++;
  }

  ifs.close();

  if (poses_.empty()) {
    spdlog::error("No valid poses loaded from {}", pose_file_path);
    return false;
  }

  // Sort poses by timestamp
  std::sort(poses_.begin(), poses_.end(),
            [](const ExternalPose& a, const ExternalPose& b) {
              return a.lidar_timestamp < b.lidar_timestamp;
            });

  spdlog::info("Loaded {} poses from {} (time range: {:.6f} - {:.6f})",
               valid_count, pose_file_path,
               poses_.front().lidar_timestamp,
               poses_.back().lidar_timestamp);

  return true;
}

double PoseFileLoader::min_timestamp() const {
  if (poses_.empty()) return 0.0;
  return poses_.front().lidar_timestamp;
}

double PoseFileLoader::max_timestamp() const {
  if (poses_.empty()) return 0.0;
  return poses_.back().lidar_timestamp;
}

}  // namespace glim
