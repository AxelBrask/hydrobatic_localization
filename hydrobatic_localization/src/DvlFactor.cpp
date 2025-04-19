#include <hydrobatic_localization/DvlFactor.h>

namespace gtsam {

Vector DvlFactor::evaluateError(const Pose3 &pose, const Vector3 &estimated_velocity, const imuBias::ConstantBias &bias_gyro,
                    gtsam::OptionalMatrixType H1, gtsam::OptionalMatrixType H2, gtsam::OptionalMatrixType H3) const {

    // velocity of the dvl frame without the angular velocity contribution
    Vector3 baslinkVel = base_link_dvl_rotations_.matrix().transpose()*pose.rotation().matrix().transpose()*estimated_velocity;
    // angular velocity contribution
    Vector3 angular_velocity_contribution = base_link_dvl_rotations_*(base_link_gyro_measurement_ - bias_gyro.gyroscope()).cross(base_to_dvl_offset_);
    // expected velocity of the dvl frame
    Vector3 expected_velocity = baslinkVel + angular_velocity_contribution;

    //Rotation matrix from baselink to world frame
    Rot3 R_world_base_link = pose.rotation();

    // error
    Vector3 error = expected_velocity - dvl_velocity_measurement_;

    //Jacobian with respect to pose
    if (H1) {

      // Jacobian with respect to translation is zero matrix
      Matrix H1_translation = Matrix::Zero(3,3);
      // Jacobian with respect to rotation
      Matrix H1_rotation = base_link_dvl_rotations_.matrix().transpose()*R_world_base_link.transpose().matrix()*skewSymmetric(estimated_velocity)*R_world_base_link.matrix();
      // Matrix H1_rotation = pose.rotation().matrix().transpose()* skewSymmetric(estimated_velocity);
      Eigen::Matrix<double,3,6> H1_mat;
      H1_mat.block<3,3>(0,0) = H1_rotation;
      H1_mat.block<3,3>(0,3) = H1_translation;
      *H1 = H1_mat;
    }

    //Jacobian with respect to velocity
    if(H2){
      *H2 = base_link_dvl_rotations_.matrix()*R_world_base_link.transpose().matrix(); 
    }

    //Jacobian with respect to gyro bias
    if(H3){
        // Create a 3x6 zero matrix (accelerometer and gyro bias)
        Eigen::Matrix<double, 3, 6> H3_mat = Eigen::Matrix<double, 3, 6>::Zero();
        // Fill in the last 3 columns with the jacobian with repsect to gyro bias
        H3_mat.block<3,3>(0,3) = base_link_dvl_rotations_.matrix() * skewSymmetric(base_to_dvl_offset_);
//skewSymmetric(base_to_dvl_offset_) * base_link_dvl_rotations_.matrix();
        *H3 = H3_mat;
    }
  return (Vector(error));
}

}  // namespace gtsam