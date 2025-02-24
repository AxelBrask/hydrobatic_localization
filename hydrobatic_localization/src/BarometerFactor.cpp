#include <hydrobatic_localization/BarometerFactor.h>

namespace gtsam {

Vector BarometerFactor::evaluateError(const Pose3 &pose,
                                      gtsam::OptionalMatrixType H) const {
  // double W_B_depht = pose.translation().z();
  Point3 r = pose.rotation().rotate(base_to_pressure_offset_);
  Point3 sensor_position = pose.translation() + r;
  double expected_depth = sensor_position.z();
  // std::cout << "Expected Pressure: " << expectedPressure << "  MeasurredPressure: "<<measuredPressure_<< std::endl;
  double error = expected_depth - measuredDepth_;
  // std::cout << "Error: " << error << std::endl; 
  if (H) {
    // Calculate the Jacobians.
    // 
    *H = (Matrix(1, 6) << 0, -r.y(), -r.x(), 0, 0, 1).finished();
  }

  return (Vector1(error));
}

}  // namespace gtsam