#include <glim/util/pose_interpolator.hpp>

#include <cmath>
#include <algorithm>
#include <spdlog/spdlog.h>

namespace glim {

PoseInterpolator::PoseInterpolator(const std::vector<ExternalPose>& poses)
    : poses_(poses), min_time_(0.0), max_time_(0.0) {
  if (!poses_.empty()) {
    min_time_ = poses_.front().lidar_timestamp;
    max_time_ = poses_.back().lidar_timestamp;
  }
}

int PoseInterpolator::find_lower_bound(double timestamp) const {
  if (poses_.empty() || timestamp < poses_.front().lidar_timestamp) {
    return -1;
  }

  // Binary search for the largest timestamp <= query timestamp
  int left = 0;
  int right = static_cast<int>(poses_.size()) - 1;

  while (left < right) {
    int mid = left + (right - left + 1) / 2;
    if (poses_[mid].lidar_timestamp <= timestamp) {
      left = mid;
    } else {
      right = mid - 1;
    }
  }

  return left;
}

Eigen::Isometry3d PoseInterpolator::interpolate(double timestamp) const {
  if (poses_.empty()) {
    spdlog::warn("PoseInterpolator: No poses loaded, returning identity");
    return Eigen::Isometry3d::Identity();
  }

  // Handle boundary cases
  if (timestamp <= min_time_) {
    return poses_.front().pose();
  }
  if (timestamp >= max_time_) {
    return poses_.back().pose();
  }

  // Find the two poses surrounding the timestamp
  int idx = find_lower_bound(timestamp);
  if (idx < 0) {
    return poses_.front().pose();
  }
  if (idx >= static_cast<int>(poses_.size()) - 1) {
    return poses_.back().pose();
  }

  const ExternalPose& p0 = poses_[idx];
  const ExternalPose& p1 = poses_[idx + 1];

  // Compute interpolation factor
  double dt = p1.lidar_timestamp - p0.lidar_timestamp;
  if (dt < 1e-9) {
    return p0.pose();
  }

  double t = (timestamp - p0.lidar_timestamp) / dt;
  t = std::clamp(t, 0.0, 1.0);

  // Interpolate position (linear)
  Eigen::Vector3d position = (1.0 - t) * p0.position + t * p1.position;

  // Interpolate orientation (SLERP)
  Eigen::Quaterniond orientation = p0.orientation.slerp(t, p1.orientation);

  // Construct interpolated pose
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = position;
  T.linear() = orientation.toRotationMatrix();

  return T;
}

std::optional<ExternalPose> PoseInterpolator::find_nearest(double timestamp, double tolerance_sec) const {
  if (poses_.empty()) {
    return std::nullopt;
  }

  int idx = find_lower_bound(timestamp);

  double min_diff = std::numeric_limits<double>::max();
  int best_idx = -1;

  // Check the pose at idx
  if (idx >= 0 && idx < static_cast<int>(poses_.size())) {
    double diff = std::abs(poses_[idx].lidar_timestamp - timestamp);
    if (diff < min_diff) {
      min_diff = diff;
      best_idx = idx;
    }
  }

  // Check the pose at idx + 1
  if (idx + 1 >= 0 && idx + 1 < static_cast<int>(poses_.size())) {
    double diff = std::abs(poses_[idx + 1].lidar_timestamp - timestamp);
    if (diff < min_diff) {
      min_diff = diff;
      best_idx = idx + 1;
    }
  }

  if (best_idx >= 0 && min_diff <= tolerance_sec) {
    return poses_[best_idx];
  }

  return std::nullopt;
}

bool PoseInterpolator::in_range(double timestamp) const {
  return !poses_.empty() && timestamp >= min_time_ && timestamp <= max_time_;
}

std::pair<double, double> PoseInterpolator::time_range() const {
  return {min_time_, max_time_};
}

Eigen::Vector3d PoseInterpolator::estimate_velocity(double timestamp, double dt) const {
  if (poses_.empty() || dt <= 0) {
    return Eigen::Vector3d::Zero();
  }

  // Use central difference when possible
  double t0 = timestamp - dt / 2.0;
  double t1 = timestamp + dt / 2.0;

  // Clamp to available range
  t0 = std::max(t0, min_time_);
  t1 = std::min(t1, max_time_);

  if (t1 - t0 < 1e-9) {
    return Eigen::Vector3d::Zero();
  }

  Eigen::Isometry3d pose0 = interpolate(t0);
  Eigen::Isometry3d pose1 = interpolate(t1);

  Eigen::Vector3d velocity = (pose1.translation() - pose0.translation()) / (t1 - t0);

  return velocity;
}

}  // namespace glim
