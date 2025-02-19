#include "hydrobatic_localization/gtsam_graph.h"
#include <cmath>

GtsamGraph::GtsamGraph() : current_index_(0) {}

void GtsamGraph::initGraphAndState(const Rot3& initial_rot, const Point3& initial_position) {
  Pose3 prior_pose(initial_rot, initial_position);
  Vector3 prior_velocity = Vector3::Zero();
  imuBias::ConstantBias prior_imu_bias, prior_sbg_bias;

  auto pose_noise = noiseModel::Isotropic::Sigma(6, 1e-3);
  auto velocity_noise = noiseModel::Isotropic::Sigma(3, 1e-3);
  auto bias_noise = noiseModel::Isotropic::Sigma(6, 1e-3);

  // Add prior factors
  graph_.addPrior<Pose3>(X(0), prior_pose, pose_noise);
  graph_.addPrior<Vector3>(V(0), prior_velocity, velocity_noise);
  graph_.addPrior<imuBias::ConstantBias>(B(0), prior_imu_bias, bias_noise);
  graph_.addPrior<imuBias::ConstantBias>(B2(0), prior_sbg_bias, bias_noise);

  // Insert initial estimates
  initial_estimate_.insert(X(0), prior_pose);
  initial_estimate_.insert(V(0), prior_velocity);
  initial_estimate_.insert(B(0), prior_imu_bias);
  initial_estimate_.insert(B2(0), prior_sbg_bias);

  // Save the initial state.
  previous_state_ = NavState(prior_pose, prior_velocity);
  current_imu_bias_ = prior_imu_bias;
  current_sbg_bias_ = prior_sbg_bias;
  current_index_ = 0;
}

NavState GtsamGraph::addImuFactor(const PreintegratedCombinedMeasurements& pim,
                                  const NavState& previous_state,
                                  const imuBias::ConstantBias& IMU_bias) {
  // int next_index = current_index_ + 1;
  CombinedImuFactor imu_factor(
    X(current_index_), V(current_index_),
    X(current_index_+1),     V(current_index_+1),
    B(current_index_), B(current_index_+1),
    pim
  );
  graph_.add(imu_factor);

  // Predict state using the preintegrated measurements.
  NavState predicted_state = pim.predict(previous_state, IMU_bias);

  // Insert the predicted state into the initial estimate.
  initial_estimate_.insert(X(current_index_+1), predicted_state.pose());
  initial_estimate_.insert(V(current_index_+1), predicted_state.v());
  initial_estimate_.insert(B(current_index_+1), IMU_bias);

  // Update the current index.
  // current_index_ = next_index;
  return predicted_state;
}

 
NavState GtsamGraph::addSbgFactor(const PreintegratedCombinedMeasurements& pim,
                        const NavState& previous_state,
                        const imuBias::ConstantBias& SBG_bias) {

    CombinedImuFactor imu_factor(
    X(current_index_), V(current_index_),
    X(current_index_+1), V(current_index_+1),
    B2(current_index_), B2(current_index_+1),
    pim
  );

  graph_.add(imu_factor);
  NavState predicted_state = pim.predict(previous_state, SBG_bias);

  // initial_estimate_.insert(X(current_index_+1), predicted_state.pose());
  // initial_estimate_.insert(V(current_index_+1), predicted_state.v());
  initial_estimate_.insert(B2(current_index_+1), SBG_bias);
  return predicted_state;
    }


void GtsamGraph::addDvlFactor(const Vector3& dvl_velocity) {
  auto dvl_noise = noiseModel::Diagonal::Sigmas((Vector(3) << 0.01, 0.01, 0.1).finished());
  graph_.add(PriorFactor<Vector3>(V(current_index_+1), dvl_velocity, dvl_noise));
}

void GtsamGraph::addGpsFactor(const Point3& gps_point) {
  auto gps_noise = noiseModel::Diagonal::Sigmas((Vector(3) << 0.5, 0.5, 1).finished());
  graph_.add(GPSFactor(X(current_index_+1), gps_point, gps_noise));
}

void GtsamGraph::addBarometerFactor(double depth_measurement) {
  auto barometer_noise = noiseModel::Diagonal::Sigmas((Vector(1) << 0.01).finished());
  graph_.add(BarometerFactor(X(current_index_+1), depth_measurement, barometer_noise));
}

void GtsamGraph::optimize() {
  current_index_++;
  LevenbergMarquardtParams params;
  LevenbergMarquardtOptimizer optimizer(graph_, initial_estimate_, params);
  Values result = optimizer.optimize();

  current_imu_bias_ = result.at<imuBias::ConstantBias>(B(current_index_));
  current_sbg_bias_ = result.at<imuBias::ConstantBias>(B2(current_index_));
  previous_state_ = NavState(result.at<Pose3>(X(current_index_)), result.at<Vector3>(V(current_index_)));

}

NavState GtsamGraph::getCurrentState() const {
  return previous_state_;
}

imuBias::ConstantBias GtsamGraph::getCurrentImuBias() const {
  return current_imu_bias_;
}

imuBias::ConstantBias GtsamGraph::getCurrentSbgBias() const {
  return current_sbg_bias_ ;
}

int GtsamGraph::getCurrentIndex() const {
  return current_index_;
}

std::shared_ptr<PreintegratedCombinedMeasurements::Params> GtsamGraph::getImuParams() {
  double accel_noise_sigma = 2.0e-6;
  double gyro_noise_sigma = 5.0e-6;
  double accel_bias_rw_sigma = 1e-3;
  double gyro_bias_rw_sigma = 1e-4;
  Matrix33 measured_acc_cov = I_3x3 * pow(accel_noise_sigma, 2);
  Matrix33 measured_omega_cov = I_3x3 * pow(gyro_noise_sigma, 2);
  Matrix33 integration_error_cov = I_3x3 * 1e-8;
  Matrix33 bias_acc_cov = I_3x3 * pow(accel_bias_rw_sigma, 2);
  Matrix33 bias_omega_cov = I_3x3 * pow(gyro_bias_rw_sigma, 2);
  Matrix66 bias_acc_omega_init = I_6x6 * 1e-5;
  auto params = PreintegratedCombinedMeasurements::Params::MakeSharedD(9.81);
  params->accelerometerCovariance = measured_acc_cov;
  params->integrationCovariance = integration_error_cov;
  params->gyroscopeCovariance = measured_omega_cov;
  params->biasAccCovariance = bias_acc_cov;
  params->biasOmegaCovariance = bias_omega_cov;
  params->biasAccOmegaInt = bias_acc_omega_init;
  params->body_P_sensor = Pose3(Rot3(), Point3(0.24, 0.0, -0.036));
  return params;
}

std::shared_ptr<PreintegratedCombinedMeasurements::Params> GtsamGraph::getSbgParams() {
  double accel_noise_sigma = 2.0e-6;
  double gyro_noise_sigma = 5.0e-6;
  double accel_bias_rw_sigma = 1e-3;
  double gyro_bias_rw_sigma = 1e-4;
  Matrix33 measured_acc_cov = I_3x3 * pow(accel_noise_sigma, 2);
  Matrix33 measured_omega_cov = I_3x3 * pow(gyro_noise_sigma, 2);
  Matrix33 integration_error_cov = I_3x3 * 1e-8;
  Matrix33 bias_acc_cov = I_3x3 * pow(accel_bias_rw_sigma, 2);
  Matrix33 bias_omega_cov = I_3x3 * pow(gyro_bias_rw_sigma, 2);
  Matrix66 bias_acc_omega_init = I_6x6 * 1e-5;
  auto params = PreintegratedCombinedMeasurements::Params::MakeSharedD(9.81);
  params->accelerometerCovariance = measured_acc_cov;
  params->integrationCovariance = integration_error_cov;
  params->gyroscopeCovariance = measured_omega_cov;
  params->biasAccCovariance = bias_acc_cov;
  params->biasOmegaCovariance = bias_omega_cov;
  params->biasAccOmegaInt = bias_acc_omega_init;
  params->body_P_sensor = Pose3(Rot3(), Point3(0.46, 0.0, 0.002));
  return params;
}
