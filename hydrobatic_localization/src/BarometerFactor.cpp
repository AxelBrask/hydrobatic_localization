#include <hydrobatic_localization/BarometerFactor.h>

namespace gtsam {

Vector BarometerFactor::evaluateError(const Pose3 &pose,
                                      gtsam::OptionalMatrixType H) const {
  Point3 r = pose.rotation().rotate(base_to_pressure_offset_);
  Point3 sensor_position = pose.translation() + r;
  double expected_depth = sensor_position.z();
  double error = expected_depth - measuredDepth_;
  if (H) {
    // Calculate the Jacobian.
    //jociabain with translation
    Matrix H1_tranlation = pose.rotation().matrix().row(2);
    //jacobian with respect to rotation
    Matrix H1_rotation = -(pose.rotation().matrix()*skewSymmetric(base_to_pressure_offset_)).row(2);
    *H = (Matrix(1, 6) << H1_rotation, H1_tranlation).finished();

  }

  return (Vector1(error));
}

}  // namespace gtsam