#pragma once

#include <string>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace glim {

/**
 * @brief External pose data loaded from a pose file (e.g., FAST-LIVO2 pose.txt)
 */
struct ExternalPose {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double lidar_timestamp;         ///< LiDAR timestamp (Unix epoch seconds)
  double image_timestamp;         ///< Image timestamp (Unix epoch seconds)
  Eigen::Vector3d position;       ///< Position (x, y, z) in world frame
  Eigen::Quaterniond orientation; ///< Orientation (w, x, y, z) in world frame

  /**
   * @brief Get the pose as an Isometry3d transformation
   * @return Eigen::Isometry3d  T_world_sensor
   */
  Eigen::Isometry3d pose() const;
};

/**
 * @brief Loader for external pose files
 *
 * Supports FAST-LIVO2 pose.txt format:
 *   tx ty tz qw qx qy qz lidar_ts img_ts
 */
class PoseFileLoader {
public:
  PoseFileLoader() = default;
  ~PoseFileLoader() = default;

  /**
   * @brief Load poses from a file
   * @param pose_file_path  Path to the pose file
   * @return true if loading succeeded
   */
  bool load(const std::string& pose_file_path);

  /**
   * @brief Get the number of loaded poses
   */
  size_t size() const { return poses_.size(); }

  /**
   * @brief Check if poses are loaded
   */
  bool empty() const { return poses_.empty(); }

  /**
   * @brief Get all poses
   */
  const std::vector<ExternalPose>& poses() const { return poses_; }

  /**
   * @brief Get minimum timestamp
   */
  double min_timestamp() const;

  /**
   * @brief Get maximum timestamp
   */
  double max_timestamp() const;

private:
  std::vector<ExternalPose> poses_;
};

}  // namespace glim
