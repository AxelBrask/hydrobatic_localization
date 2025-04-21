#include "hydrobatic_localization/gtsam_graph.h"
#include <cmath>

GtsamGraph::GtsamGraph(InferenceStrategy strategy) : current_index_(0), inference_strategy_(strategy) {

    // Initialize the IMU preintegrator using the parameters from the GTSAM side.
  imuBias::ConstantBias prior_bias;
  auto imu_param = getImuParams();
  imu_preintegrated_ = std::make_shared<PreintegratedCombinedMeasurements>(imu_param, prior_bias);

  // Initialize the SBG preintegrator using the parameters from the GTSAM side.
  imuBias::ConstantBias prior_sbg_bias;
  auto sbg_param = getSbgParams();
  sbg_preintegrated_ = std::make_shared<PreintegratedCombinedMeasurements>(sbg_param, prior_sbg_bias);
  
  if(strategy == InferenceStrategy::ISAM2){
    std::cout << "Using ISAM2" << std::endl;
    gtsam::ISAM2Params params;
    params.relinearizeThreshold = 0.01;
    params.relinearizeSkip = 1;
    params.enablePartialRelinearizationCheck = true;
        params.print("ISAM2");

    isam_ = std::make_shared<gtsam::ISAM2>(params);
  }
  else if(strategy == InferenceStrategy::FixedLagSmoothing){
    std::cout << "Using FixedLagSmoothing" << std::endl;
    gtsam::ISAM2Params params;
    params.relinearizeThreshold = 0.01;
    params.relinearizeSkip = 1;
    params.findUnusedFactorSlots = true;
    params.print("FixedLagSmoother");

    smootherLag = 10.0;
    fixed_lag_smoother_ = std::make_shared<gtsam::IncrementalFixedLagSmoother>(smootherLag, params);

  }
  else if (strategy == InferenceStrategy::EKF) {
    std::cout << "Using EKF" << std::endl;
    gtsam::ISAM2Params params;
    params.relinearizeThreshold = 0.01;
    params.relinearizeSkip = 1;
    params.findUnusedFactorSlots = true;
    smootherLag = 1.0;
    params.print("EKF");
    fixed_lag_smoother_ = std::make_shared<gtsam::IncrementalFixedLagSmoother>(smootherLag, params);

  }

  else if (strategy == InferenceStrategy::FullSmoothing) {
    std::cout << "Using FullSmoothing" << std::endl;
  }

  else {
    throw std::invalid_argument("Invalid inference strategy");
  }


}


void GtsamGraph::initGraphAndState(const Rot3& initial_rot, const Point3& initial_position) {
  Pose3 prior_pose(initial_rot, initial_position);
  Vector3 prior_velocity = Vector3::Zero();
  imuBias::ConstantBias prior_imu_bias, prior_sbg_bias;

  auto pose_noise = noiseModel::Isotropic::Sigma(6, 1e-8);
  auto velocity_noise = noiseModel::Isotropic::Sigma(3, 1e-8);
  auto bias_noise = noiseModel::Isotropic::Sigma(6, 1e-3);

  // Add prior factors
  graph_.addPrior<Pose3>(X(0), prior_pose, pose_noise);
  graph_.addPrior<Vector3>(V(0), prior_velocity, velocity_noise);
  graph_.addPrior<imuBias::ConstantBias>(B(0), prior_imu_bias, bias_noise);
  // graph_.addPrior<imuBias::ConstantBias>(B2(0), prior_sbg_bias, bias_noise); 

  // Insert initial estimates
  initial_estimate_.insert(X(0), prior_pose);
  initial_estimate_.insert(V(0), prior_velocity);
  initial_estimate_.insert(B(0), prior_imu_bias);
  // initial_estimate_.insert(B2(0), prior_sbg_bias);

  // Save the initial state.
  previous_state_ = NavState(prior_pose, prior_velocity);
  current_imu_bias_ = prior_imu_bias;
  current_sbg_bias_ = prior_sbg_bias;
  current_index_ = 0;
}

void GtsamGraph::integrateImuMeasurement(const Vector3& acc, const Vector3& gyro, const double dt){
  imu_preintegrated_->integrateMeasurement(acc, gyro, dt);
  }

void GtsamGraph::integrateSbgMeasurement(const Vector3& acc, const Vector3& gyro, const double dt){
  sbg_preintegrated_->integrateMeasurement(acc, gyro, dt);
  }

NavState GtsamGraph::addImuFactor() {
  // int next_index = current_index_ + 1;
   PreintegratedCombinedMeasurements pim = *imu_preintegrated_;
  CombinedImuFactor imu_factor(
    X(current_index_), V(current_index_),
    X(current_index_+1),     V(current_index_+1),
    B(current_index_), B(current_index_+1),
    pim
  );
  graph_.add(imu_factor);

  // Predict state using the preintegrated measurements.
  NavState predicted_state = imu_preintegrated_->predict(previous_state_, current_imu_bias_);

  // Insert the predicted state into the initial estimate.
  initial_estimate_.insert(X(current_index_+1), predicted_state.pose());
  initial_estimate_.insert(V(current_index_+1), predicted_state.v());
  initial_estimate_.insert(B(current_index_+1), current_imu_bias_);


  // Update the current index.
  // current_index_ = next_index; 
  return predicted_state;
}


NavState GtsamGraph::addSbgFactor() {
    PreintegratedCombinedMeasurements pim = *sbg_preintegrated_;
    CombinedImuFactor imu_factor(
    X(current_index_), V(current_index_),
    X(current_index_+1), V(current_index_+1),
    B2(current_index_), B2(current_index_+1),
    pim
  );

  graph_.add(imu_factor);
  NavState predicted_state = sbg_preintegrated_->predict(previous_state_, current_sbg_bias_);

  // initial_estimate_.insert(X(current_index_+1), predicted_state.pose());
  // initial_estimate_.insert(V(current_index_+1), predicted_state.v());
  initial_estimate_.insert(B2(current_index_+1), current_sbg_bias_);

  return predicted_state;
    }

/*
 TODO: add all extrinsics to a seperate class/struct
*/


void GtsamGraph::addMotionModelFactor(const double start_time, const double end_time,
 const std::shared_ptr<const PreintegratedMotionModel>& pmm, const Vector3& gyro){

  auto motionModelNoise = noiseModel::Isotropic::Sigma(9, 0.05);
  // auto motionModelNoise = noiseModel::Diagonal::Sigmas((Vector(9) <<
  graph_.add(SamMotionModelFactor(X(current_index_), X(current_index_+1), V(current_index_), V(current_index_+1),
                                  motionModelNoise, start_time, end_time, *pmm, gyro));
}
void GtsamGraph::addDvlFactor(const Vector3& dvl_velocity, const Vector3& gyro) {
  auto dvl_noise = noiseModel::Diagonal::Sigmas((Vector(3) << 0.01, 0.01, 0.05).finished());
  Vector3 base_link_to_dvl_offset(0.573 ,0.0 ,-0.063); 
  Rot3 base_link_dvl_rotation = Rot3::Identity();
  graph_.add(DvlFactor(X(current_index_+1),V(current_index_+1),B(current_index_+1),
   dvl_velocity, gyro, base_link_to_dvl_offset, base_link_dvl_rotation, dvl_noise));
}

void GtsamGraph::addGpsFactor(const Point3& gps_point) {
  auto gps_noise = noiseModel::Diagonal::Sigmas((Vector(3) << 0.1, 0.1, 0.1).finished());
  Point3 base_to_gps_offset(0.528 ,0.0, 0.071);
  graph_.add(GPSFactorArm(X(current_index_+1), gps_point, base_to_gps_offset, gps_noise));
}


void GtsamGraph::addBarometerFactor(double depth_measurement) {
  auto barometer_noise = noiseModel::Diagonal::Sigmas((Vector(1) << 0.01).finished());
  Vector3 base_to_pressure_offset(-0.503, 0.025, 0.057);
  graph_.add(BarometerFactor(X(current_index_+1), depth_measurement, base_to_pressure_offset, barometer_noise));
}

void GtsamGraph::optimize() {
  current_index_++;


  if(inference_strategy_ ==InferenceStrategy::FullSmoothing){
    std::cout << "Full Smoothing optmizer" << std::endl;
    // optimizeFullSmoothing();

  LevenbergMarquardtParams params;
  // params.setVerbosity("ERROR");
  LevenbergMarquardtOptimizer optimizer(graph_, initial_estimate_, params);
  Values result = optimizer.optimize();
  // result.print("Final Result:\n");

  current_imu_bias_ = result.at<imuBias::ConstantBias>(B(current_index_));
  // current_sbg_bias_ = result.at<imuBias::ConstantBias>(B2(current_index_));
  previous_state_ = NavState(result.at<Pose3>(X(current_index_)), result.at<Vector3>(V(current_index_)));

  imu_preintegrated_->resetIntegrationAndSetBias(current_imu_bias_);
  sbg_preintegrated_->resetIntegrationAndSetBias(current_sbg_bias_);
  }
  else if(inference_strategy_ == InferenceStrategy::FixedLagSmoothing || inference_strategy_ == InferenceStrategy::EKF){
      double t = static_cast<double>(current_index_);
      std::cout<< "timestamp: " << t << std::endl;
      for (auto const& kv : initial_estimate_) {
      smoother_timestamp_map_[kv.key] = t;
    }
    std::cout << "Fixed Lag Smoothing optmizer" << std::endl;
    
    fixed_lag_smoother_->update(graph_, initial_estimate_,smoother_timestamp_map_);
        for (auto it = smoother_timestamp_map_.begin(); it != smoother_timestamp_map_.end();) {
      if (it->second < t - smootherLag)
        it = smoother_timestamp_map_.erase(it);
      else
        ++it;
    }
    Values result = fixed_lag_smoother_->calculateEstimate();
    current_imu_bias_ = result.at<imuBias::ConstantBias>(B(current_index_));
    // current_sbg_bias_ = result.at<imuBias::ConstantBias>(B2(current_index_));
    previous_state_ = NavState(result.at<Pose3>(X(current_index_)), result.at<Vector3>(V(current_index_)));
    //log the current size of the graph
    std::cout << "# factors kept by smoother: "
              << fixed_lag_smoother_->getFactors().size() << '\n';
    std::cout << "# variables in window    : "
              << fixed_lag_smoother_->getISAM2().getLinearizationPoint().size() << '\n';
    graph_.resize(0);
    // smoother_timestamp_map_.clear();
    initial_estimate_.clear();
    imu_preintegrated_->resetIntegrationAndSetBias(current_imu_bias_);
    sbg_preintegrated_->resetIntegrationAndSetBias(current_sbg_bias_);
  } 


  else if(inference_strategy_ == InferenceStrategy::ISAM2){
    std::cout << "ISAM2 optmizer" << std::endl;
    isam_->update(graph_, initial_estimate_);

    Values result = isam_->calculateEstimate();
    current_imu_bias_ = result.at<imuBias::ConstantBias>(B(current_index_));
    // current_sbg_bias_ = result.at<imuBias::ConstantBias>(B2(current_index_));
    previous_state_ = NavState(result.at<Pose3>(X(current_index_)), result.at<Vector3>(V(current_index_)));
    graph_.resize(0);
    initial_estimate_.clear();
    imu_preintegrated_->resetIntegrationAndSetBias(current_imu_bias_);
    sbg_preintegrated_->resetIntegrationAndSetBias(current_sbg_bias_);

  }

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
  double accel_noise_sigma = 2.0e-4;
  double gyro_noise_sigma = 5.0e-4;
  double accel_bias_rw_sigma = 1e-3;
  double gyro_bias_rw_sigma = 1e-3;
  Matrix33 measured_acc_cov = I_3x3 * pow(accel_noise_sigma, 2);
  Matrix33 measured_omega_cov = I_3x3 * pow(gyro_noise_sigma, 2);
  Matrix33 integration_error_cov = I_3x3 * 1e-5;
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
  double accel_noise_sigma = 2.0e-4;
  double gyro_noise_sigma = 5.0e-4;
  double accel_bias_rw_sigma = 1e-3;
  double gyro_bias_rw_sigma = 1e-3;
  Matrix33 measured_acc_cov = I_3x3 * pow(accel_noise_sigma, 2);
  Matrix33 measured_omega_cov = I_3x3 * pow(gyro_noise_sigma, 2);
  Matrix33 integration_error_cov = I_3x3 * 1e-5;
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
