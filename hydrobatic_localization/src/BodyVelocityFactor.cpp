#include <hydrobatic_localization/BodyVelocityFactor.h>

namespace gtsam {

Vector BodyVelocityFactor::evaluateError(const Pose3 &pose, const Vector3 &velocity,
                    gtsam::OptionalMatrixType H1, gtsam::OptionalMatrixType H2) const {
    // Rotation matrix from baselink to world frame
    Rot3 R_world_base_link = pose.rotation();
    Vector3 velocity_in_body = R_world_base_link.matrix().transpose() * velocity;
    // error
    Vector3 error = velocity_in_body - velocity_measurement_;
    //Jacobian with respect to pose
    if (H1) {

      // Jacobian with respect to translation is zero matrix
      Matrix H1_translation = Matrix::Zero(3,3);
      // Jacobian with respect to rotation
      Matrix H1_rotation = skewSymmetric(velocity_in_body);
      // Matrix H1_rotation = pose.rotation().matrix().transpose()* skewSymmetric(estimated_velocity);
      Eigen::Matrix<double,3,6> H1_mat;
      H1_mat.block<3,3>(0,0) = H1_rotation;
      H1_mat.block<3,3>(0,3) = H1_translation;
      *H1 = H1_mat;
    }

    //Jacobian with respect to velocity
    if(H2){
      *H2 = R_world_base_link.matrix().transpose();
    }

  return (Vector(error));
}

}  // namespace gtsam