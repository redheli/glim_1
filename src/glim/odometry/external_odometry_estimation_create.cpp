#include <glim/odometry/external_odometry_estimation.hpp>

extern "C" glim::OdometryEstimationBase* create_odometry_estimation_module() {
  glim::ExternalOdometryEstimationParams params;
  return new glim::ExternalOdometryEstimation(params);
}
