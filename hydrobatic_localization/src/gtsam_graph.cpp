#include "hydrobatic_localization/gtsam_graph.h"
#include <cmath>
#include <gtsam/constrained/NonlinearConstraint.h>

GtsamGraph::GtsamGraph(InferenceStrategy strategy,const std::string& config_file) : current_index_(0),
            inference_strategy_(strategy),config_(Config::load(config_file)) {
  //Initialize the imu and sbg preintegrators with the noise models and sensor offsets           
  imuBias::ConstantBias prior_bias;
  imu_preintegrated_ = std::make_shared<PreintegratedCombinedMeasurements>(
      makeParams(config_.imu, config_.extrinsics.imu_sensor_offset), prior_bias);

  imuBias::ConstantBias prior_sbg_bias;
  sbg_preintegrated_ = std::make_shared<PreintegratedCombinedMeasurements>(
     makeParams(config_.sbg, config_.extrinsics.sbg_sensor_offset), prior_sbg_bias);

  // Define the inference strategy
  if(strategy == InferenceStrategy::ISAM2)
  {
    std::cout << "Using ISAM2" << std::endl;
    gtsam::ISAM2Params params;
    params.relinearizeThreshold = 0.1;
    params.relinearizeSkip = 1;
    params.enablePartialRelinearizationCheck = true;
    params.print("ISAM2");

    isam_ = std::make_shared<gtsam::ISAM2>(params);
  }

  else if(strategy == InferenceStrategy::FixedLagSmoothing)
  {
    std::cout << "Using FixedLagSmoothing" << std::endl;
    gtsam::ISAM2Params params;
    params.relinearizeThreshold = 0.01;
    params.relinearizeSkip = 1;
    params.findUnusedFactorSlots = true;
    params.print("FixedLagSmoother");

    smootherLag = 100;
    fixed_lag_smoother_ = std::make_shared<gtsam::IncrementalFixedLagSmoother>(smootherLag, params);

  }

  else if (strategy == InferenceStrategy::EKF) 
  {
    std::cout << "Using EKF" << std::endl;
    gtsam::ISAM2Params params;
    params.relinearizeThreshold = 0.01;
    params.relinearizeSkip = 1;
    params.findUnusedFactorSlots = true;
    smootherLag = 1.0;
    params.print("EKF");
    fixed_lag_smoother_ = std::make_shared<gtsam::IncrementalFixedLagSmoother>(smootherLag, params);

  }

  else if (strategy == InferenceStrategy::FullSmoothing) 
  {
    std::cout << "Using FullSmoothing" << std::endl;
  }

  else 
  {
    throw std::invalid_argument("Invalid inference strategy");
  }


}


void GtsamGraph::initGraphAndState(const Rot3& initial_rot, const Point3& initial_position, const Vector3& initial_velocity ) 
{
  Pose3 prior_pose(initial_rot, initial_position);
  Vector3 prior_velocity = initial_velocity;
  imuBias::ConstantBias prior_imu_bias, prior_sbg_bias;

  auto pose_noise = noiseModel::Diagonal::Sigmas(config_.noise_models.prior.pose_sigma);
  auto velocity_noise = noiseModel::Diagonal::Sigmas(config_.noise_models.prior.velocity_sigma);
  auto bias_noise = noiseModel::Diagonal::Sigmas(config_.noise_models.prior.bias_sigma);


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

void GtsamGraph::integrateImuMeasurement(const Vector3& acc, const Vector3& gyro, const double dt)
{
  imu_preintegrated_->integrateMeasurement(acc, gyro, dt);
}

void GtsamGraph::integrateSbgMeasurement(const Vector3& acc, const Vector3& gyro, const double dt)
{
  sbg_preintegrated_->integrateMeasurement(acc, gyro, dt);
}

// Used for adding ground truth velocity factors to the graph, e.g. for simulation or testing purposes or when DVL is bad.
void GtsamGraph::addGtVelocityFactor(const Vector3& velocity) {
  auto velocity_noise = noiseModel::Diagonal::Sigmas(config_.noise_models.dvl_sigma);
  graph_.add(BodyVelocityFactor(X(current_index_+1), V(current_index_+1), velocity, velocity_noise));
}

// Used for adding ground truth pose factors to the graph, e.g. for simulation or testing purposes.
void GtsamGraph::addGtPoseFactor(const Pose3& pose) {
  
  gtsam::Vector6 sigmas;
  sigmas << 0.01, 0.01, 0.01,
            0.001, 0.001, 0.001;
  auto pose_noise = noiseModel::Diagonal::Sigmas(sigmas);
  graph_.addPrior<Pose3>(X(current_index_+1), pose, pose_noise);
}

// Used for adding ground truth prior factors to the graph, e.g. for simulation or testing purposes.
void GtsamGraph::addGtPriorFactor(const Pose3& pose, const Vector3& velocity){
  auto pose_noise = noiseModel::Diagonal::Sigmas(config_.noise_models.prior.pose_sigma);
  auto velocity_noise = noiseModel::Diagonal::Sigmas(config_.noise_models.dvl_sigma);
  auto bias_noise = noiseModel::Diagonal::Sigmas(config_.noise_models.prior.bias_sigma);
  graph_.addPrior<Pose3>(X(current_index_+1), pose, pose_noise);
  graph_.add(BodyVelocityFactor(X(current_index_+1), V(current_index_+1),velocity,  velocity_noise));
}

NavState GtsamGraph::addImuFactor() 
{
  PreintegratedCombinedMeasurements pim = *imu_preintegrated_;
  CombinedImuFactor imu_factor(
    X(current_index_), V(current_index_),
    X(current_index_+1),     V(current_index_+1),
    B(current_index_), B(current_index_+1),
    pim
  );
  graph_.add(imu_factor);

  imu_prediction_state_ = imu_preintegrated_->predict(previous_state_, current_imu_bias_);

  return imu_prediction_state_;
}


NavState GtsamGraph::addSbgFactor()   
{
    PreintegratedCombinedMeasurements pim = *sbg_preintegrated_;
    CombinedImuFactor imu_factor(
    X(current_index_), V(current_index_),
    X(current_index_+1), V(current_index_+1),
    B2(current_index_), B2(current_index_+1),
    pim
  );

  graph_.add(imu_factor);
  sbg_prediction_state_ = sbg_preintegrated_->predict(previous_state_, current_sbg_bias_);

  
  return sbg_prediction_state_;
}

void GtsamGraph::addSbgOrientationFactor(const Rot3& orientation) 
{
  gtsam::Vector6 sigmas;
  sigmas << 0.0001, 0.0001, 0.1, // small rotation noise
            10000, 10000, 10000; // no rotation noise, but large translation noise
  auto orientation_noise = noiseModel::Diagonal::Sigmas(sigmas);  
  //add prior on Pose3 but only constrain the rotation
  graph_.addPrior<Pose3>(X(current_index_+1), Pose3(orientation, previous_state_.pose().translation()), orientation_noise);
  // graph_.addPrior<Pose3>(X(current_index_+1), orientation, orientation_noise)
}
void GtsamGraph::addMotionModelFactor(const double start_time, const double end_time,
 const std::shared_ptr<const PreintegratedMotionModel>& pmm, const Vector3& gyro,NavState& new_state) 
{

  auto motionModelNoise = noiseModel::Diagonal::Sigmas(config_.noise_models.motion_model_sigma);
  graph_.add(SamMotionModelFactor(X(current_index_), X(current_index_+1), V(current_index_), V(current_index_+1),
                                  motionModelNoise, start_time, end_time, *pmm, gyro));
  motion_model_prediction_state_ =pmm->getMotionModelPredictionState();

}

void GtsamGraph::addDvlFactor(const Vector3& dvl_velocity, const Vector3& gyro, const Vector3& dvl_velocity_covariance, const bool& use_sensor_covariance)
{
  Vector3 sigmas = use_sensor_covariance ? dvl_velocity_covariance.cwiseSqrt() : config_.noise_models.dvl_sigma;
  auto dvl_noise = noiseModel::Diagonal::Sigmas(sigmas);
  Vector3 base_link_to_dvl_offset = config_.extrinsics.dvl_sensor_offset;

  Rot3 base_link_dvl_rotation = Rot3::Identity();
  graph_.add(DvlFactor(X(current_index_+1),V(current_index_+1),B(current_index_+1),
   dvl_velocity, gyro, base_link_to_dvl_offset, base_link_dvl_rotation, dvl_noise));
}

void GtsamGraph::addGpsFactor(const Point3& gps_point, const Vector3& gps_variances, const bool& use_sensor_covariance) 
{
  Vector3 gps_sigma = use_sensor_covariance ? gps_variances.cwiseSqrt() : config_.noise_models.gps_sigma;
  auto gps_noise = noiseModel::Diagonal::Sigmas(gps_sigma);
  Point3 base_to_gps_offset(
  config_.extrinsics.gps_sensor_offset.x(),
  config_.extrinsics.gps_sensor_offset.y(),
  config_.extrinsics.gps_sensor_offset.z());
  graph_.add(GPSFactorArm(X(current_index_+1), gps_point, base_to_gps_offset, gps_noise));
}


void GtsamGraph::addBarometerFactor(double depth_measurement)
{
  auto barometer_noise = noiseModel::Isotropic::Sigma(1, config_.noise_models.barometer_sigma);
  Vector3 base_to_pressure_offset = config_.extrinsics.baro_sensor_offset;
  graph_.add(BarometerFactor(X(current_index_+1), depth_measurement, base_to_pressure_offset, barometer_noise));
}

void GtsamGraph::addInitialEstimate()
{

  if(current_index_ == 0)
  {
    // If this is the first iteration, we need to insert the initial estimate
    initial_estimate_.insert(X(current_index_+1), initial_estimate_.at<Pose3>(X(0)));
    initial_estimate_.insert(V(current_index_+1), initial_estimate_.at<Vector3>(V(0)));
    initial_estimate_.insert(B(current_index_+1), current_imu_bias_);
    // initial_estimate_.insert(B2(current_index_+1), current_sbg_bias_);
  }
  else
  {
    // If this is not the first iteration, we need to insert the initial estimate for the next index
  initial_estimate_.insert( X(current_index_+1),results_.at<Pose3>(X(current_index_)) );
  initial_estimate_.insert( V(current_index_+1), results_.at<Vector3>(V(current_index_)) );
  initial_estimate_.insert( B(current_index_+1), current_imu_bias_ );
  // initial_estimate_.insert(B2(current_index_+1), current_sbg_bias_);
  }
}

void GtsamGraph::optimize() {
  addInitialEstimate();
  


  if(inference_strategy_ ==InferenceStrategy::FullSmoothing)
  {
    LevenbergMarquardtParams params;
    LevenbergMarquardtOptimizer optimizer(graph_, initial_estimate_, params);
    results_ = optimizer.optimize();

    current_imu_bias_ = results_.at<imuBias::ConstantBias>(B(current_index_+1));
    current_sbg_bias_ = results_.at<imuBias::ConstantBias>(B2(current_index_+1));
    previous_state_ = NavState(results_.at<Pose3>(X(current_index_+1)), results_.at<Vector3>(V(current_index_+1)));

    imu_preintegrated_->resetIntegrationAndSetBias(current_imu_bias_);
    sbg_preintegrated_->resetIntegrationAndSetBias(current_sbg_bias_);
  }
  else if(inference_strategy_ == InferenceStrategy::FixedLagSmoothing || inference_strategy_ == InferenceStrategy::EKF)
  {
      double t = static_cast<double>(current_index_+1);
      for (auto const& kv : initial_estimate_) {
       smoother_timestamp_map_[kv.key] = t;
    }
      
      fixed_lag_smoother_->update(graph_, initial_estimate_,smoother_timestamp_map_);
          for (auto it = smoother_timestamp_map_.begin(); it != smoother_timestamp_map_.end();) {
        if (it->second < t - smootherLag)
          it = smoother_timestamp_map_.erase(it);
        else
          ++it;
    }
       results_ = fixed_lag_smoother_->calculateEstimate();
      current_imu_bias_ = results_.at<imuBias::ConstantBias>(B(current_index_+1));
      // current_sbg_bias_ = results_.at<imuBias::ConstantBias>(B2(current_index_+1));
      previous_state_ = NavState(results_.at<Pose3>(X(current_index_+1)), results_.at<Vector3>(V(current_index_+1)));
      graph_.resize(0);
      initial_estimate_.clear();
      imu_preintegrated_->resetIntegrationAndSetBias(current_imu_bias_);
      // sbg_preintegrated_->resetIntegrationAndSetBias(current_sbg_bias_);
  } 


  else if(inference_strategy_ == InferenceStrategy::ISAM2)
  {
    // std::cout << "ISAM2 optmizer" << std::endl;
    isam_->update(graph_, initial_estimate_);

    results_ = isam_->calculateEstimate();
    current_imu_bias_ = results_.at<imuBias::ConstantBias>(B(current_index_+1));
    // current_sbg_bias_ = results_.at<imuBias::ConstantBias>(B2(current_index_+1));
    previous_state_ = NavState(results_.at<Pose3>(X(current_index_+1)), results_.at<Vector3>(V(current_index_+1)));
    graph_.resize(0);
    initial_estimate_.clear();
    imu_preintegrated_->resetIntegrationAndSetBias(current_imu_bias_);
    // sbg_preintegrated_->resetIntegrationAndSetBias(current_sbg_bias_);

  }
  current_index_++;
}


std::shared_ptr<PreintegratedCombinedMeasurements::Params>
GtsamGraph::makeParams(const NoiseConfig& n, const Vector3& sensor_offset) {
  Matrix33 measured_acc_cov    = I_3x3 * std::pow(n.accel_noise_sigma,    2);
  Matrix33 measured_omega_cov  = I_3x3 * std::pow(n.gyro_noise_sigma,     2);
  Matrix33 integration_error   = I_3x3 * n.integration_error_cov;
  Matrix33 bias_acc_cov        = I_3x3 * std::pow(n.accel_bias_rw_sigma,  2);
  Matrix33 bias_omega_cov      = I_3x3 * std::pow(n.gyro_bias_rw_sigma,   2);
  Matrix66 bias_acc_omega_init = I_6x6 * n.bias_acc_omega_init;

  auto params = PreintegratedCombinedMeasurements::Params::MakeSharedD(n.gravity);
  params->accelerometerCovariance = measured_acc_cov;
  params->gyroscopeCovariance     = measured_omega_cov;
  params->integrationCovariance   = integration_error;
  params->biasAccCovariance       = bias_acc_cov;
  params->biasOmegaCovariance     = bias_omega_cov;
  params->biasAccOmegaInt         = bias_acc_omega_init;
  params->body_P_sensor = Pose3(
    Rot3::Identity(),
    Point3(sensor_offset.x(), sensor_offset.y(), sensor_offset.z())
  );

  return params;
}