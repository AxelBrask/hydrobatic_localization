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

  // Initialize the factor graph with prior factors.
  void initGraphAndState(const Rot3& initial_rot, const Point3& initial_position);

  // Add an IMU factor (from the preintegrator) and insert a predicted state.
  // Returns the predicted state.

NavState addImuFactor(
                        const NavState& previous_state,
                        const imuBias::ConstantBias& current_bias);

  // Add an SBG factor (from the preintegrator) and insert a predicted state.
  // Returns the predicted state.
NavState addSbgFactor(
                        const NavState& previous_state,
                        const imuBias::ConstantBias& current_bias);

  // Add additional factors when new measurements are available.
  void addDvlFactor(const Vector3& dvl_velocity, const Vector3& gyro);
  void addGpsFactor(const Point3& gps_point);
  void addBarometerFactor(double depth_measurement);

  // Optimize the factor graph to update the state.
  void optimize();

  // Get the updated state and bias.
  NavState getCurrentState() const;
  imuBias::ConstantBias getCurrentImuBias() const;
  imuBias::ConstantBias getCurrentSbgBias() const;

  // Get the current keyframe index.
  int getCurrentIndex() const;

  std::shared_ptr<PreintegratedCombinedMeasurements::Params> getImuParams();
  std::shared_ptr<PreintegratedCombinedMeasurements::Params> getSbgParams();

  // Imu and Sbg integrators
  void integrateImuMeasurement(const Vector3& acc, const Vector3& gyro, const double dt);
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
