#pragma once

#include <vector>
#include <optional>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <glim/util/pose_file_loader.hpp>

namespace glim {

/**
 * @brief Interpolator for external poses with timestamp synchronization
 *
 * Provides pose interpolation using SLERP for rotation and linear interpolation for translation.
 */
class PoseInterpolator {
public:
  /**
   * @brief Construct a pose interpolator
   * @param poses  Vector of external poses (must be sorted by timestamp)
   */
  explicit PoseInterpolator(const std::vector<ExternalPose>& poses);
  ~PoseInterpolator() = default;

  /**
   * @brief Get interpolated pose at exact timestamp
   * @param timestamp  Query timestamp
   * @return Interpolated pose, or identity if out of range
   */
  Eigen::Isometry3d interpolate(double timestamp) const;

  /**
   * @brief Find nearest pose within tolerance
   * @param timestamp      Query timestamp
   * @param tolerance_sec  Maximum time difference (seconds)
   * @return Nearest pose if found within tolerance
   */
  std::optional<ExternalPose> find_nearest(double timestamp, double tolerance_sec = 0.1) const;

  /**
   * @brief Check if timestamp is within the pose range
   * @param timestamp  Query timestamp
   * @return true if timestamp is within [min_time, max_time]
   */
  bool in_range(double timestamp) const;

  /**
   * @brief Get the time range of loaded poses
   * @return pair of (min_time, max_time)
   */
  std::pair<double, double> time_range() const;

  /**
   * @brief Estimate velocity at given timestamp using finite differences
   * @param timestamp  Query timestamp
   * @param dt         Time delta for finite difference (default 0.01s)
   * @return Velocity in world frame (linear velocity only)
   */
  Eigen::Vector3d estimate_velocity(double timestamp, double dt = 0.01) const;

private:
  /**
   * @brief Find the index of the pose just before the given timestamp
   * @param timestamp  Query timestamp
   * @return Index of the pose before timestamp, or -1 if before first pose
   */
  int find_lower_bound(double timestamp) const;

  std::vector<ExternalPose> poses_;
  double min_time_;
  double max_time_;
};

}  // namespace glim
