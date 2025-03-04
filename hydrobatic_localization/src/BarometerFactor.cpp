#include <hydrobatic_localization/BarometerFactor.h>

namespace gtsam {

Vector BarometerFactor::evaluateError(const Pose3 &pose,
                                      gtsam::OptionalMatrixType H) const {
  // double W_B_depht = pose.translation().z();
  Point3 r = pose.rotation().rotate(base_to_pressure_offset_);
  Point3 sensor_position = pose.translation() + r;
  double expected_depth = sensor_position.z();
  double error = expected_depth - measuredDepth_;
  // std::cout << "Error: " << error << std::endl; 
  if (H) {
    // Calculate the Jacobians.
    // 
      // *H = (Matrix(1, 6) << 0, 0, 0 , 0, 0, 1).finished();
    //jociabain with translation
    Matrix H1_tranlation = pose.rotation().matrix().row(2);
    //jacobian with respect to rotation
    Matrix H1_rotation = -(pose.rotation().matrix()*skewSymmetric(base_to_pressure_offset_)).row(2);
    *H = (Matrix(1, 6) << H1_rotation, H1_tranlation).finished();
    // std::cout << "Jacobian: " << *H << std::endl;

  }

  return (Vector1(error));
}

}  // namespace gtsam