#pragma once

#include <memory>
#include <deque>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <glim/odometry/odometry_estimation_base.hpp>
#include <glim/util/pose_file_loader.hpp>
#include <glim/util/pose_interpolator.hpp>

namespace glim {

class CloudDeskewing;

/**
 * @brief Parameters for external odometry estimation
 */
struct ExternalOdometryEstimationParams {
  ExternalOdometryEstimationParams();
  ~ExternalOdometryEstimationParams();

  std::string pose_file_path;       ///< Path to external pose file (e.g., FAST-LIVO2 pose.txt)
  double timestamp_tolerance;       ///< Maximum time difference for pose matching (seconds)
  bool estimate_velocity;           ///< Estimate velocity from pose differences
  double velocity_dt;               ///< Time delta for velocity estimation (seconds)

  Eigen::Isometry3d T_lidar_imu;    ///< LiDAR-IMU transformation
};

/**
 * @brief Odometry estimation using external pose source (e.g., FAST-LIVO2)
 *
 * This module replaces GLIM's internal odometry estimation with poses from
 * an external source. It reads pre-computed poses from a file and synchronizes
 * them with incoming point cloud frames based on timestamps.
 *
 * Use case: Run FAST-LIVO2 offline to generate odometry, then use GLIM's
 * mapping pipeline (SubMapping + GlobalMapping) for pose graph optimization.
 */
class ExternalOdometryEstimation : public OdometryEstimationBase {
public:
  ExternalOdometryEstimation();
  explicit ExternalOdometryEstimation(const ExternalOdometryEstimationParams& params);
  ~ExternalOdometryEstimation() override;

  /**
   * @brief Load external poses from file
   * @param pose_file_path  Path to pose file
   * @return true if loading succeeded
   */
  bool load_poses(const std::string& pose_file_path);

  /**
   * @brief This odometry module does not require IMU data
   */
  bool requires_imu() const override { return false; }

  /**
   * @brief Insert an IMU measurement (stored for optional velocity estimation)
   */
  void insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) override;

  /**
   * @brief Insert a point cloud frame and return the estimation result
   * @param frame                Preprocessed point cloud frame
   * @param marginalized_frames  [out] Frames that have been marginalized (not used here)
   * @return Estimation frame with external pose
   */
  EstimationFrame::ConstPtr insert_frame(
    const PreprocessedFrame::Ptr& frame,
    std::vector<EstimationFrame::ConstPtr>& marginalized_frames) override;

  /**
   * @brief Get remaining frames at the end of sequence
   */
  std::vector<EstimationFrame::ConstPtr> get_remaining_frames() override;

private:
  std::unique_ptr<ExternalOdometryEstimationParams> params_;
  std::unique_ptr<PoseFileLoader> pose_loader_;
  std::unique_ptr<PoseInterpolator> pose_interpolator_;
  std::unique_ptr<CloudDeskewing> deskewing_;

  // Frame management
  int64_t frame_id_;
  std::deque<EstimationFrame::Ptr> frames_;

  // IMU buffer for velocity estimation (optional)
  struct IMUData {
    double stamp;
    Eigen::Vector3d linear_acc;
    Eigen::Vector3d angular_vel;
  };
  std::deque<IMUData> imu_buffer_;

  // First frame timestamp for time offset handling
  bool first_frame_received_;
  double first_frame_stamp_;
  double first_pose_stamp_;
};

}  // namespace glim
