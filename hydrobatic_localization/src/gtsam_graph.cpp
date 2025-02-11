#include "gtsam_graph.h"

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/ISAM2Params.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <iostream>

GtsamGraph::GtsamGraph()
    : current_index_(0),
      last_time_(0.0)
{
    // Construct the shared PreintegratedCombinedMeasurements with default bias
    imuBias::ConstantBias prior_bias; 
    imu_preintegrated_ = std::make_shared<PreintegratedCombinedMeasurements>(
        makeImuParams(), prior_bias
    );

    // Initialize bias with zero (or near-zero) by default
    current_bias_ = prior_bias;
}

void GtsamGraph::initializeGraph(const gtsam::Rot3 &initial_rot, 
                                 const gtsam::Point3 &initial_pos)
{
    std::cout << "[GtsamGraph] Initializing graph/state.\n";

    // Create a prior pose, velocity, bias
    gtsam::Pose3 prior_pose(initial_rot, initial_pos);
    gtsam::Vector3 prior_velocity(0.0, 0.0, 0.0);
    imuBias::ConstantBias prior_bias(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)); 

    // Noise models
    auto pose_noise     = noiseModel::Isotropic::Sigma(6, 1e-3);
    auto velocity_noise = noiseModel::Isotropic::Sigma(3, 1e-3);
    auto bias_noise     = noiseModel::Isotropic::Sigma(6, 1e-3);

    // Add priors
    graph_.addPrior(X(0), prior_pose, pose_noise);
    graph_.addPrior(V(0), prior_velocity, velocity_noise);
    graph_.addPrior(B(0), prior_bias, bias_noise);

    // Populate initial estimates
    initial_estimate_.insert(X(0), prior_pose);
    initial_estimate_.insert(V(0), prior_velocity);
    initial_estimate_.insert(B(0), prior_bias);

    // Store as previous state
    previous_state_ = NavState(prior_pose, prior_velocity);
    current_bias_   = prior_bias;

    // current_index_ remains 0
    // If you wish, you can run a first optimization here:
    // LevenbergMarquardtOptimizer optimizer(graph_, initial_estimate_);
    // optimizer.optimize();
}

void GtsamGraph::integrateIMUMeasurement(const gtsam::Vector3 &acc,
                                         const gtsam::Vector3 &gyro,
                                         double delta_t)
{
    // Integrate the IMU reading into imu_preintegrated_
    imu_preintegrated_->integrateMeasurement(acc, gyro, delta_t);
}

void GtsamGraph::keyframeUpdate(bool new_dvl_measurement, 
                                const gtsam::Vector3 &dvl_body_frame_vel)
{
    // Build the CombinedImuFactor
    int next_index = current_index_ + 1;

    // Make a local copy of the preintegrated data
    PreintegratedCombinedMeasurements pim = *imu_preintegrated_;
    CombinedImuFactor imu_factor(
        X(current_index_), V(current_index_),
        X(next_index),     V(next_index),
        B(current_index_), B(next_index),
        pim
    );
    graph_.add(imu_factor);

    // Predict the next state using the preintegrator
    gtsam::NavState predicted_state = 
        imu_preintegrated_->predict(previous_state_, current_bias_);

    // Insert predicted state into initial estimate
    initial_estimate_.insert(X(next_index), predicted_state.pose());
    initial_estimate_.insert(V(next_index), predicted_state.v());
    initial_estimate_.insert(B(next_index), current_bias_);

    // If there's a new DVL measurement, add a prior on velocity:
    if(new_dvl_measurement)
    {
        // Example: use a small noise since DVL is presumably good
        auto dvl_noise = noiseModel::Isotropic::Sigma(3, 0.00001);
        // Transform the DVL velocity from body to the world if needed.
        // In many cases you might do this with the current pose's rotation:
        // But for simplicity, assume the measurement is in the body frame 
        // and we want a prior in the world frame. 
        // This snippet is just an example; adapt to your reference frames.
        
        // E.g. world-frame velocity:
        gtsam::Vector3 dvl_world_vel = previous_state_.pose().rotation() * dvl_body_frame_vel;
        graph_.add(PriorFactor<gtsam::Vector3>(V(next_index), dvl_world_vel, dvl_noise));

        // std::cout << "[GtsamGraph] DVL Factor added: " 
        //           << dvl_world_vel.transpose() << std::endl;
    }

    // Now do a batch optimization (or you could do iSAM)
    LevenbergMarquardtParams params;
    LevenbergMarquardtOptimizer optimizer(graph_, initial_estimate_, params);
    Values result = optimizer.optimize();

    // Retrieve the latest states
    auto latest_bias = result.at<imuBias::ConstantBias>(B(next_index));
    std::cout << "[GtsamGraph] Latest bias:" << std::endl;
    std::cout << "  Accelerometer bias: " << latest_bias.accelerometer().transpose() << std::endl;
    std::cout << "  Gyroscope bias:     " << latest_bias.gyroscope().transpose() << std::endl;
    gtsam::NavState latest_state(
        result.at<Pose3>(X(next_index)), 
        result.at<gtsam::Vector3>(V(next_index))
    );

    // Update our stored state
    previous_state_ = latest_state;
    current_bias_   = latest_bias;
    current_index_  = next_index;

    // Reset the integration with the new bias
    resetIntegrationWithCurrentBias();

    // (Optional) If you want to clear or keep the graph_ and initial_estimate_,
    // that depends on your approach. Often in iSAM2 we don't clear these.
    // For standard factor-graph approach, you typically keep them growing 
    // or selectively remove older factors.
    // 
    // Example of clearing them if you want a sliding window approach:
    // graph_.resize(0);
    // initial_estimate_.clear();

    // std::cout << "[GtsamGraph] Keyframe " << next_index
    //           << " -> Pose: " << latest_state.pose().translation().transpose()
    //           << ", Velocity: " << latest_state.v().transpose() << std::endl;
}

void GtsamGraph::resetIntegrationWithCurrentBias()
{
    imu_preintegrated_->resetIntegrationAndSetBias(current_bias_);
}

gtsam::Pose3 GtsamGraph::getCurrentPose() const
{
    return previous_state_.pose();
}

gtsam::Vector3 GtsamGraph::getCurrentVelocity() const
{
    return previous_state_.v();
}

gtsam::imuBias::ConstantBias GtsamGraph::getCurrentBias() const
{
    return current_bias_;
}

std::shared_ptr<PreintegratedCombinedMeasurements::Params> GtsamGraph::makeImuParams()
{
    double accel_noise_sigma     = 2.0e-3;
    double gyro_noise_sigma      = 5.0e-5;
    double accel_bias_rw_sigma   = 1e-3;
    double gyro_bias_rw_sigma    = 1e-5;

    gtsam::Matrix33 measured_acc_cov    = I_3x3 * pow(accel_noise_sigma, 2);
    gtsam::Matrix33 measured_omega_cov  = I_3x3 * pow(gyro_noise_sigma, 2);
    gtsam::Matrix33 integration_error_cov = I_3x3 * 1e-8;
    gtsam::Matrix33 bias_acc_cov        = I_3x3 * pow(accel_bias_rw_sigma, 2);
    gtsam::Matrix33 bias_omega_cov      = I_3x3 * pow(gyro_bias_rw_sigma, 2);
    gtsam::Matrix66 bias_acc_omega_init = I_6x6 * 1e-8;

    auto params = PreintegratedCombinedMeasurements::Params::MakeSharedD(0.0);
    params->accelerometerCovariance = measured_acc_cov;
    params->integrationCovariance   = integration_error_cov;
    params->gyroscopeCovariance     = measured_omega_cov;
    params->biasAccCovariance       = bias_acc_cov;
    params->biasOmegaCovariance     = bias_omega_cov;
    params->biasAccOmegaInt         = bias_acc_omega_init;

    // Example offset from body to IMU
    params->body_P_sensor = Pose3(Rot3(), Point3(0.24, 0.0, -0.036));
    return params;
}

gtsam::Rot3 GtsamGraph::averageRotations(const std::vector<gtsam::Rot3>& rotations)
{
    gtsam::Vector3 sum_log = gtsam::Vector3::Zero();
    for (const auto& rot : rotations) {
        sum_log += gtsam::Rot3::Logmap(rot);
    }
    gtsam::Vector3 avg_log = sum_log / static_cast<double>(rotations.size());
    return gtsam::Rot3::Expmap(avg_log);
}
