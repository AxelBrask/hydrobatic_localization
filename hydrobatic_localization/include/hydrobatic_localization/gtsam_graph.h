#ifndef GTSAM_GRAPH_HPP
#define GTSAM_GRAPH_HPP

// GTSAM Includes
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>

// Standard Includes
#include <memory>
#include <vector>

//
// Shorthand for symbols X, V, B
//
using namespace gtsam;
using symbol_shorthand::X; // Pose
using symbol_shorthand::V; // Velocity
using symbol_shorthand::B; // Bias

/**
 * @brief Class responsible for managing the GTSAM factor graph, 
 *        states, biases, and optimization.
 */
class GtsamGraph
{
public:
    GtsamGraph();
    
    /**
     * @brief Initialize the graph with a prior pose, velocity, and bias.
     * 
     * @param initial_rot     The initial rotation (orientation).
     * @param initial_pos     The initial position.
     */
    void initializeGraph(const gtsam::Rot3 &initial_rot, 
                         const gtsam::Point3 &initial_pos);

    /**
     * @brief Integrate a single IMU measurement (accel + gyro).
     * 
     * @param acc        Acceleration measurement.
     * @param gyro       Gyro measurement.
     * @param delta_t    Timestep between last IMU measurement and current.
     */
    void integrateIMUMeasurement(const gtsam::Vector3 &acc,
                                 const gtsam::Vector3 &gyro,
                                 double delta_t);

    /**
     * @brief Called at "keyframe" intervals to add the combined IMU factor,
     *        optionally add other sensor factors (e.g., DVL), and optimize.
     * 
     * @param new_dvl_measurement If there's a new DVL velocity measurement.
     * @param dvl_body_frame_vel  The DVL velocity in the DVL/body frame.
     *                            (Set only if new_dvl_measurement == true)
     */
    void keyframeUpdate(bool new_dvl_measurement, 
                        const gtsam::Vector3 &dvl_body_frame_vel);

    /**
     * @brief Reset the IMU preintegration with the latest bias.
     */
    void resetIntegrationWithCurrentBias();

    /**
     * @brief Returns the latest estimated Pose.
     */
    gtsam::Pose3 getCurrentPose() const;

    /**
     * @brief Returns the latest estimated Velocity.
     */
    gtsam::Vector3 getCurrentVelocity() const;

    /**
     * @brief Returns the current bias (accel + gyro).
     */
    gtsam::imuBias::ConstantBias getCurrentBias() const;

    /**
     * @brief Utility to average a set of rotations (for IMU initialization).
     */
    static gtsam::Rot3 averageRotations(const std::vector<gtsam::Rot3>& rotations);

private:
    /**
     * @brief Create and return IMU preintegration parameters (noise, etc.).
     */
    std::shared_ptr<PreintegratedCombinedMeasurements::Params> makeImuParams();

private:
    // Graph & Estimation
    gtsam::NonlinearFactorGraph graph_;
    gtsam::Values               initial_estimate_;

    // Instead of iSAM2, this example uses a single batch optimizer each time:
    // std::shared_ptr<gtsam::ISAM2> isam_;

    // IMU Preintegration
    std::shared_ptr<gtsam::PreintegratedCombinedMeasurements> imu_preintegrated_;

    // Current index for X, V, B in factor graph
    int current_index_;

    // State tracking
    gtsam::NavState              previous_state_;
    gtsam::imuBias::ConstantBias current_bias_;

    double last_time_;
};

#endif // GTSAM_GRAPH_HPP
