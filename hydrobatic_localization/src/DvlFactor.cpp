#include <hydrobatic_localization/DvlFactor.h>

namespace gtsam {

Vector DvlFactor::evaluateError(const Pose3 &pose, const Vector3 &estimated_velocity, const imuBias::ConstantBias &bias_gyro,
                    gtsam::OptionalMatrixType H1, gtsam::OptionalMatrixType H2, gtsam::OptionalMatrixType H3) const {

    //Rotation matrix from world to base_link
    Rot3 R_world_base_link = pose.rotation();
    // velocity of the dvl frame without the angular velocity contribution
    Vector3  vel_world_dvl = base_link_dvl_rotations_.matrix()*(R_world_base_link.transpose().matrix()*estimated_velocity);
    // angular velocity contribution
    Vector3 angualr_velocity_contribution = base_to_dvl_offset_.cross(
      base_link_dvl_rotations_*(base_link_gyro_measurement_-bias_gyro.gyroscope()));
    // expected velocity of the dvl frame
    Vector3 expected_velocity = vel_world_dvl + angualr_velocity_contribution;
    // error
    Vector3 error = expected_velocity - dvl_velocity_measurement_;
    // std::cout << "Expected Velocity: " << expected_velocity.transpose() << std::endl;
    // std::cout << "Measured Velocity: " << dvl_velocity_measurement_.transpose() << std::endl;
    std::cout << "Error:             " << error.transpose() << std::endl;
    // std::cout << "--------------------\n" << std::endl;

    //Jacobian with respect to pose
    if (H1) {

      // Jacobian with respect to translation is zero matrix
      Matrix H1_translation = Matrix::Zero(3,3);
     // Jacobian with respect to rotation
      Matrix H1_rotation = base_link_dvl_rotations_.matrix()*R_world_base_link.transpose().matrix()*skewSymmetric(estimated_velocity)*R_world_base_link.matrix();
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
        H3_mat.block<3,3>(0,3) = -skewSymmetric(base_to_dvl_offset_) * base_link_dvl_rotations_.matrix();
        *H3 = H3_mat;
    }
  return (Vector(error));
}

}  // namespace gtsam