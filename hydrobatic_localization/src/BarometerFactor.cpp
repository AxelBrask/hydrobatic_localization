#include <hydrobatic_localization/BarometerFactor.h>

namespace gtsam {

Vector BarometerFactor::evaluateError(const Pose3 &pose,
                                      gtsam::OptionalMatrixType H) const {
    Matrix t_H; 
  if (H) {

    *H = (Matrix(1, 6) << 0, 0, 0, 0, 0, 1).finished();
    // std::cout << "Jacobian: " << *H << std::endl;  // Use *H to print the matrix.
  }
  double expectedPressure = pose.translation().z();
  std::cout << "Expected Pressure: " << expectedPressure << "  MeasurredPressure: "<<measuredPressure_<< std::endl;
  double error = expectedPressure - measuredPressure_;
  std::cout << "Error: " << error << std::endl;
  return (Vector1(error));
}

}  // namespace gtsam