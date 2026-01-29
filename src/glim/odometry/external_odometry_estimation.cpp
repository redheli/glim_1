#include <glim/odometry/external_odometry_estimation.hpp>

#include <spdlog/spdlog.h>

#include <gtsam_points/types/point_cloud_cpu.hpp>

#include <glim/util/config.hpp>
#include <glim/common/cloud_deskewing.hpp>
#include <glim/odometry/callbacks.hpp>

namespace glim {

using Callbacks = OdometryEstimationCallbacks;

ExternalOdometryEstimationParams::ExternalOdometryEstimationParams() {
  // Load from config file
  Config config(GlobalConfig::get_config_path("config_odometry"));

  pose_file_path = config.param<std::string>("odometry_estimation", "pose_file_path", "");
  timestamp_tolerance = config.param<double>("odometry_estimation", "timestamp_tolerance", 0.1);
  estimate_velocity = config.param<bool>("odometry_estimation", "estimate_velocity", true);
  velocity_dt = config.param<double>("odometry_estimation", "velocity_dt", 0.02);

  // Load T_lidar_imu from sensors config
  Config sensors_config(GlobalConfig::get_config_path("config_sensors"));
  auto T_lidar_imu_values = sensors_config.param<std::vector<double>>("sensors", "T_lidar_imu", {0, 0, 0, 0, 0, 0, 1});

  if (T_lidar_imu_values.size() == 7) {
    // TUM format: [x, y, z, qx, qy, qz, qw]
    Eigen::Vector3d trans(T_lidar_imu_values[0], T_lidar_imu_values[1], T_lidar_imu_values[2]);
    Eigen::Quaterniond quat(T_lidar_imu_values[6], T_lidar_imu_values[3], T_lidar_imu_values[4], T_lidar_imu_values[5]);
    T_lidar_imu = Eigen::Isometry3d::Identity();
    T_lidar_imu.translation() = trans;
    T_lidar_imu.linear() = quat.normalized().toRotationMatrix();
  } else {
    T_lidar_imu = Eigen::Isometry3d::Identity();
  }
}

ExternalOdometryEstimationParams::~ExternalOdometryEstimationParams() {}

ExternalOdometryEstimation::ExternalOdometryEstimation()
    : ExternalOdometryEstimation(ExternalOdometryEstimationParams()) {}

ExternalOdometryEstimation::ExternalOdometryEstimation(const ExternalOdometryEstimationParams& params)
    : params_(std::make_unique<ExternalOdometryEstimationParams>(params)),
      pose_loader_(std::make_unique<PoseFileLoader>()),
      frame_id_(0),
      first_frame_received_(false),
      first_frame_stamp_(0.0),
      first_pose_stamp_(0.0) {

  // Load poses if path is provided
  if (!params_->pose_file_path.empty()) {
    load_poses(params_->pose_file_path);
  }

  // Create deskewing module
  deskewing_ = std::make_unique<CloudDeskewing>();

  spdlog::info("ExternalOdometryEstimation initialized");
  spdlog::info("  pose_file_path: {}", params_->pose_file_path);
  spdlog::info("  timestamp_tolerance: {} sec", params_->timestamp_tolerance);
  spdlog::info("  estimate_velocity: {}", params_->estimate_velocity);
  spdlog::info("  deskewing: enabled");
}

ExternalOdometryEstimation::~ExternalOdometryEstimation() {}

bool ExternalOdometryEstimation::load_poses(const std::string& pose_file_path) {
  if (!pose_loader_->load(pose_file_path)) {
    spdlog::error("Failed to load poses from {}", pose_file_path);
    return false;
  }

  pose_interpolator_ = std::make_unique<PoseInterpolator>(pose_loader_->poses());

  auto [min_t, max_t] = pose_interpolator_->time_range();
  spdlog::info("Pose interpolator ready: {} poses, time range [{:.6f}, {:.6f}]",
               pose_loader_->size(), min_t, max_t);

  return true;
}

void ExternalOdometryEstimation::insert_imu(
    const double stamp,
    const Eigen::Vector3d& linear_acc,
    const Eigen::Vector3d& angular_vel) {

  // Trigger callback
  Callbacks::on_insert_imu(stamp, linear_acc, angular_vel);

  // Store IMU data for optional velocity estimation
  IMUData imu_data;
  imu_data.stamp = stamp;
  imu_data.linear_acc = linear_acc;
  imu_data.angular_vel = angular_vel;
  imu_buffer_.push_back(imu_data);

  // Keep buffer size reasonable
  while (imu_buffer_.size() > 1000) {
    imu_buffer_.pop_front();
  }
}

EstimationFrame::ConstPtr ExternalOdometryEstimation::insert_frame(
    const PreprocessedFrame::Ptr& frame,
    std::vector<EstimationFrame::ConstPtr>& marginalized_frames) {

  // Trigger callback
  Callbacks::on_insert_frame(frame);

  if (!pose_interpolator_) {
    spdlog::error("No poses loaded! Cannot process frame.");
    return nullptr;
  }

  // Use frame timestamps directly -- rosbag and pose.txt share the same time domain.
  // FAST-LIVO2 skips initial frames during initialization, so early frames may
  // fall before the pose range and will use the boundary pose.
  double query_stamp = frame->stamp;
  double time_offset = 0.0;

  if (!first_frame_received_) {
    first_frame_received_ = true;
    first_frame_stamp_ = frame->stamp;

    auto [min_pose_t, max_pose_t] = pose_interpolator_->time_range();
    first_pose_stamp_ = min_pose_t;

    spdlog::info("First frame stamp: {:.6f}, First pose stamp: {:.6f}",
                 first_frame_stamp_, first_pose_stamp_);
    spdlog::info("Using direct timestamp matching (no offset)");
  }

  // Check if timestamp is in range
  if (!pose_interpolator_->in_range(query_stamp)) {
    spdlog::warn("Frame timestamp {:.6f} (query: {:.6f}) out of pose range, using boundary pose",
                 frame->stamp, query_stamp);
  }

  // Get interpolated pose
  Eigen::Isometry3d T_world_lidar = pose_interpolator_->interpolate(query_stamp);

  // Create estimation frame
  auto est_frame = std::make_shared<EstimationFrame>();
  est_frame->id = frame_id_++;
  est_frame->stamp = frame->stamp;

  // Set poses
  est_frame->T_lidar_imu = params_->T_lidar_imu;
  est_frame->T_world_lidar = T_world_lidar;
  est_frame->T_world_imu = T_world_lidar * params_->T_lidar_imu;

  // Estimate velocity
  if (params_->estimate_velocity) {
    est_frame->v_world_imu = pose_interpolator_->estimate_velocity(query_stamp, params_->velocity_dt);
  } else {
    est_frame->v_world_imu = Eigen::Vector3d::Zero();
  }

  // Set IMU bias to zero (not estimated from external odometry)
  est_frame->imu_bias.setZero();

  // Store raw frame
  est_frame->raw_frame = frame;

  // Set frame_id to IMU so sub_mapping and global_mapping can use IMU-frame factors
  est_frame->frame_id = FrameID::IMU;

  // Deskew point cloud using interpolated poses over the scan duration
  std::vector<Eigen::Vector4d> deskewed_points;
  if (!frame->times.empty() && frame->times.size() == frame->points.size()) {
    // Generate a trajectory of poses covering the scan duration for deskewing
    const double scan_duration = frame->scan_end_time - frame->stamp;
    const int num_steps = 10;
    std::vector<double> imu_times(num_steps + 1);
    std::vector<Eigen::Isometry3d> imu_poses(num_steps + 1);

    for (int i = 0; i <= num_steps; i++) {
      double t = frame->stamp + scan_duration * i / num_steps;
      double qt = t + time_offset;
      imu_times[i] = scan_duration * i / num_steps;  // relative to scan start
      // Pose at this time: T_world_imu
      Eigen::Isometry3d T_world_lidar_t = pose_interpolator_->interpolate(qt);
      imu_poses[i] = T_world_lidar_t * params_->T_lidar_imu;
    }

    Eigen::Isometry3d T_imu_lidar = params_->T_lidar_imu.inverse();
    deskewed_points = deskewing_->deskew(T_imu_lidar, imu_times, imu_poses, 0.0, frame->times, frame->points);
  } else {
    deskewed_points = frame->points;
  }

  // Convert deskewed points to gtsam_points::PointCloud
  auto points = std::make_shared<gtsam_points::PointCloudCPU>(deskewed_points);
  if (!frame->intensities.empty()) {
    points->add_intensities(frame->intensities);
  }
  est_frame->frame = points;

  // Store frame
  frames_.push_back(est_frame);

  // Trigger callbacks
  Callbacks::on_new_frame(est_frame);
  Callbacks::on_update_new_frame(est_frame);
  std::vector<EstimationFrame::ConstPtr> updated_frames = {est_frame};
  Callbacks::on_update_frames(updated_frames);

  // Marginalize old frames (keep a sliding window)
  const size_t max_frames = 10;
  while (frames_.size() > max_frames) {
    marginalized_frames.push_back(frames_.front());
    frames_.pop_front();
  }

  if (!marginalized_frames.empty()) {
    Callbacks::on_marginalized_frames(marginalized_frames);
  }

  spdlog::debug("Frame {}: stamp={:.6f}, pos=({:.3f}, {:.3f}, {:.3f})",
                est_frame->id, est_frame->stamp,
                T_world_lidar.translation().x(),
                T_world_lidar.translation().y(),
                T_world_lidar.translation().z());

  return est_frame;
}

std::vector<EstimationFrame::ConstPtr> ExternalOdometryEstimation::get_remaining_frames() {
  std::vector<EstimationFrame::ConstPtr> remaining;
  for (const auto& frame : frames_) {
    remaining.push_back(frame);
  }
  frames_.clear();
  return remaining;
}

}  // namespace glim
