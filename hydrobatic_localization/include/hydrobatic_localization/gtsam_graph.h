#ifndef GTSAMGRAPH_H
#define GTSAMGRAPH_H

#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/dataset.h>
#include <hydrobatic_localization/BarometerFactor.h>
#include <hydrobatic_localization/DvlFactor.h>
#include <vector>

using namespace gtsam;
using symbol_shorthand::X; // Pose
using symbol_shorthand::V; // Velocity
using symbol_shorthand::B; // Bias

//Defining shorthand for sbg bias
namespace gtsam {
namespace symbol_shorthand {

inline gtsam::Key B2(size_t i) {
   return gtsam::Symbol('s', i);
}

} // namespace symbol_shorthand
} // namespace gtsam

using symbol_shorthand::B2; // Bias for SBG
class GtsamGraph {
public:
  GtsamGraph();

  /**
   * @brief Initialize the factor graph and the state with prior factors
   * @param initial_rot: the initial rotation of the vehicle
   * @param initial_position: the initial position of the vehicle in the odom frame
   */
  void initGraphAndState(const Rot3& initial_rot, const Point3& initial_position);


  /**
   * @brief Add an IMU factor (from the preintegrator) and insert a predicted state.
   * @return the predicted state as a NavState
   */
  NavState addImuFactor();

  /**
   * @brief Add an SBG factor (from the preintegrator) and insert a predicted state.
   * @return the predicted state as a NavState
   */
  NavState addSbgFactor();

  /**
   * @brief Add a DVL factor to the factor graph
   * @param dvl_velocity: the velocity measurement from the DVL
   * @param gyro: the angular velocity measurement from the IMU
   */
  void addDvlFactor(const Vector3& dvl_velocity, const Vector3& gyro);

  /**
   * @brief Add a GPS factor to the factor graph
   * @param gps_point: the relative GPS measurment to the navigation frame
   */
  void addGpsFactor(const Point3& gps_point);

  /**
   * @brief Add a barometer factor to the factor graph
   * @param depth_measurement: the depth measurement from the barometer in meters, negative dowmwards
   */
  void addBarometerFactor(double depth_measurement);

  /**
   * @brief Optimize the factor graph, increments the index and updates the state and bias of both the IMU and SBG.
   */
  void optimize();

  /**
   * @brief Get the current state of the vehicle
   * @return the current state as a NavState
   */
  NavState getCurrentState() const;

  /**
   * @brief Get the current IMU bias
   * @return the current IMU bias as a ConstantBias
   */
  imuBias::ConstantBias getCurrentImuBias() const;

  /**
   * @brief Get the current SBG bias
   * @return the current SBG bias as a ConstantBias
   */
  imuBias::ConstantBias getCurrentSbgBias() const;

  /**
   * @brief Get the current index of the factor graph
   * @return the current index as an integer
   */
  int getCurrentIndex() const;

  std::shared_ptr<PreintegratedCombinedMeasurements::Params> getImuParams();
  std::shared_ptr<PreintegratedCombinedMeasurements::Params> getSbgParams();

  /**
   * @brief Integrate the IMU measurements to the preintegrator
   * @param acc: the acceleration measurement
   * @param gyro: the angular velocity measurement
   * @param dt: the time step
   */
  void integrateImuMeasurement(const Vector3& acc, const Vector3& gyro, const double dt);

  /**
   * @brief Integrate the SBG measurements to the preintegrator
   * @param acc: the acceleration measurement
   * @param gyro: the angular velocity measurement
   * @param dt: the time step
   */
  void integrateSbgMeasurement(const Vector3& acc, const Vector3& gyro, const double dt);

private:
  NonlinearFactorGraph graph_;
  Values initial_estimate_;
  int current_index_;
  NavState previous_state_;
  imuBias::ConstantBias current_imu_bias_;
  imuBias::ConstantBias current_sbg_bias_;

    // Preintegrated IMU measurements (GTSAM)
  std::shared_ptr<PreintegratedCombinedMeasurements> imu_preintegrated_;
  //Preintegrated SBG measurements (GTSAM)
  std::shared_ptr<PreintegratedCombinedMeasurements> sbg_preintegrated_;
};

#endif // GTSAMGRAPH_H
