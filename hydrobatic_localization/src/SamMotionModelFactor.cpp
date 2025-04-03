#include <hydrobatic_localization/SamMotionModelFactor.h>

namespace gtsam {

Eigen::VectorXd PreintegratedMotionModel::predict(const Eigen::VectorXd& state, const double start_time, const double end_time) {
  if (control_list_.empty()) {  // control_list_ is a std::vector instead of a deque
            return state;
      }
      Eigen::VectorXd integratedState = state;
      double currentTime = start_time;
      size_t idx = 0;  // Index to track current control

      // Skip controls that occurred before the start_time, if necessary
      while (idx < control_list_.size() && control_list_[idx].timestamp < currentTime) {
          idx++;
      }

      // If there is a gap between start_time and the first control, integrate using the first control.
      if (idx < control_list_.size() && control_list_[idx].timestamp > currentTime) {
          double dt = prev_control_.timestamp - currentTime;
          integratedState = sam_motion_model_.integrateState(integratedState, prev_control_.u, dt);
          currentTime = control_list_[idx].timestamp;
      }

      // Integrate over the control sequence until reaching end_time
      for (; idx < control_list_.size() && control_list_[idx].timestamp <= end_time; idx++) {
          double dt = (idx == 0) ? control_list_[idx].timestamp - currentTime :
                                  control_list_[idx].timestamp - currentTime;
          integratedState = sam_motion_model_.integrateState(integratedState, control_list_[idx].u, dt);
          currentTime = control_list_[idx].timestamp;
      }

      // Integrate from the last control to end_time, if necessary
      double dt = end_time - currentTime;
      if (dt > 0) {
          // Use the last available control input
          integratedState = sam_motion_model_.integrateState(integratedState, control_list_.back().u, dt);
          prev_control_ = control_list_.back();
      }
      // std::stringstream ss;
      // ss << "Updated state: " << integratedState.transpose();
      // RCLCPP_INFO(logger, "%s", ss.str().c_str());
      return integratedState;


}
Eigen::VectorXd PreintegratedMotionModel::stateToVector(const gtsam::NavState& state, const gtsam::Vector3 gyro) {

        // Translation and rotation this needs to be converted from ENU to NED
        Eigen::Matrix3d R_e2n;       
        R_e2n << 0, 1, 0,
                1, 0, 0,
                0, 0, -1;
        
        // Convert the translation: from ENU to Ned
        Eigen::VectorXd t_enu = state.pose().translation();
        Eigen::VectorXd t_ned(3);
        t_ned << t_enu.y(), t_enu.x(), -t_enu.z();
        
        // Get the ENU rotation matrix from the nav state
        Eigen::Matrix3d R_enu = state.pose().rotation().matrix();
        // Convert it to NED by applying the transformation on both sides
        Eigen::Matrix3d R_ned = R_e2n * R_enu * R_e2n.transpose();
        // Convert the rotation matrix to a quaternion (in NED)
        Eigen::Quaterniond q_ned(R_ned);
        
        // Fill the state vector: first three elements are translation, 
        // next four are the quaternion (w, x, y, z)
        Eigen::VectorXd eta(7);
        eta << t_ned(0), t_ned(1), t_ned(2),
        q_ned.w(), q_ned.x(), q_ned.y(), q_ned.z();
        // Linear and angular velocity
        Eigen::VectorXd nu(6);
        //Velcoity in the base link frame
        Eigen::VectorXd base_link_velocity = state.pose().rotation().transpose().matrix()*state.velocity();
        nu << base_link_velocity.x(), base_link_velocity.y(), base_link_velocity.z(), gyro.x(), gyro.y(), gyro.z();
        Eigen::VectorXd state_vector(eta.size() + nu.size());
        state_vector << eta, nu;
        return state_vector;
}
void PreintegratedMotionModel::controlToList(const Eigen::VectorXd& u, const double& timestamp, const bool& isThrusterVector) {
          // add the first input to the qeue
          std::cout << "Control input: " << u.transpose() << std::endl;
          if(control_list_.empty()){
                  controlSequence new_control;
                  new_control.u = Eigen::VectorXd::Zero(6); // Initialize with zeros
                  if (isThrusterVector) {
                    new_control.u.segment<2>(0) = u;
                  }
                  else {
                    new_control.u.segment<2>(0) = u.segment<2>(0);
                    new_control.u.segment<2>(4) = u.segment<2>(2);
                  }
                  new_control.timestamp = timestamp;
                  // std::stringstream ss;
                  // ss << "New control added: " << new_control.u.transpose() << ", timestamp: " << new_control.timestamp;
                  // RCLCPP_INFO(logger, "%s", ss.str().c_str());
                  control_list_.push_back(new_control);
                  return;
          }
          //Get the latest control in the queue
          controlSequence latest_control = control_list_.back();

          // Update the control input, if the latest control is a thruster vector, update only element 2 and 3
          if(isThrusterVector){
                  latest_control.u.segment<2>(2) = u;

          }
          // If the latest control is a feedback control, update the first 2 elements and the last 2 elements
          else{
                  latest_control.u.segment<2>(0) = u.segment<2>(0);
                  latest_control.u.segment<2>(4) = u.segment<2>(2);
          }
          latest_control.timestamp = timestamp;
          // std::stringstream ss;
          // ss << "Updated latest control: " << latest_control.u.transpose() << ", timestamp: " << latest_control.timestamp;
          // RCLCPP_INFO(logger, "%s", ss.str().c_str());
          // RCLCPP_INFO(logger, "Control queue size: %d", control_queue_.size());
          control_list_.push_back(latest_control);

  }




Vector SamMotionModelFactor::evaluateError(const Pose3 &pose1, const Pose3& pose2,const Vector3 &velocity1, const Vector3& velocity2,
                    gtsam::OptionalMatrixType H1 , gtsam::OptionalMatrixType H2 , gtsam::OptionalMatrixType H3 , gtsam::OptionalMatrixType H4) const {

    // Jacobians, let A = Tij^-1 Ti^-1 Tj
    Matrix6 H_i, H_j; // will store ∂(T_i^-1 T_j)/∂T_i and ∂(T_i^-1 T_j)/∂T_j
    Matrix6 H_Tij_TijInverse;// will store ∂(T_ij^-1)/∂T_ij
    Matrix6 H_A_logA; // will store ∂(logA)/∂A
    Matrix6 H_TijInverse_A; // will store ∂(A)/∂T_ij^-1
    Matrix6 H_xi_Tij; // will store ∂(T_ij)/∂xi
    Matrix6 H_TiInvTj_A; // will store ∂(A)/∂T_i^-1 T_j
    Matrix6 H_x_i_Tj; // will store ∂(T_j)/∂x_i
    Matrix6 H_Tj_TiInvTj; // will store ∂(T_i^-1 T_j)/∂T_j
    // Predicted state from the motion model , measured state
    Pose3 Tj_hat = Pose3(Rot3::Quaternion(predictedModelState_(3), predictedModelState_(4), predictedModelState_(5), predictedModelState_(6)),
                         Point3(predictedModelState_(0), predictedModelState_(1), predictedModelState_(2)));
    Pose3 Ti = pose1; // Estimated state
    Pose3 Tj = pose2; // Estimated state
    Pose3 Tij_hat = Ti.between(Tj_hat, &H_i, &H_j);
    Pose3 T_ij_inverse = Tij_hat.inverse(&H_Tij_TijInverse);
    Pose3 A = T_ij_inverse.compose(Ti.inverse().compose(Tj));
    Vector6 logA = Pose3::Logmap(A, &H_A_logA);
    Pose3 T_A = T_ij_inverse.compose(Ti.inverse().compose(Tj), &H_TijInverse_A);
    Pose3 T_ij_new = Ti.retract(Pose3::Logmap(Tij_hat), &H_xi_Tij);
    H_TiInvTj_A = T_ij_inverse.AdjointMap();  
    H_Tj_TiInvTj = Ti.inverse().AdjointMap();
    Vector6 v = Pose3::Logmap(Tj_hat);
    // Vector6 v = Vector6::Zero();

    Pose3 Tj_new = Tj.retract(v, &H_x_i_Tj);


    Vector6 pose_error = Pose3::Logmap(Tij_hat.inverse().compose(Ti.inverse().compose(Tj)));







    Vector3 predicted_velocity = Vector3(predictedModelState_(7), predictedModelState_(8), predictedModelState_(9));
    Vector velocity_error = velocity2 - predicted_velocity;
    Vector error(9);
    error << pose_error, velocity_error;

    //Jacobian with respect to pose1
    if(H1) {
      // the jacobian w.r.t the translation is the identity matrix
        *H1 = Matrix::Zero(9, 6);
        H1->block(3,3,3,3) = Matrix3::Identity();
        // the jacobian w.r.t to rotation we use the numerical solver
        Matrix H_rot = gtsam::numericalDerivative11<Vector, Pose3>( 
        [this, pose2, velocity1, velocity2](const Pose3& p1){
          return this->evaluateError(p1, pose2, velocity1, velocity2);},pose1);

        H1->block<3,3>(0,0) = H_rot.block(3, 3, 3, 3);
        std::cout << "H_rot: " << H_rot << std::endl;
      }
    //Jacobian with respect to pose2
    if(H2){
      //Since pose2 is the estimated state we are comparing to, the jacobian is identity matrix
      *H2 = Matrix::Zero(9, 6);
        Matrix H2_pose = gtsam::numericalDerivative11<Vector, Pose3>( 
        [this, pose1, velocity1, velocity2](const Pose3& p2){
          return this->evaluateError(pose1, p2, velocity1, velocity2);},pose2);
        H2->block<6,6>(0,0) = H2_pose.block(0, 0, 6,6);
      Matrix6 H2_ana = H_A_logA * H_TiInvTj_A * H_Tj_TiInvTj*H_x_i_Tj;
      std::cout << "H2_ana: " << H2_ana << std::endl;
      std::cout << "H2_pose: " << H2_pose << std::endl;
    }

      // 
      // // *H2 = Matrix::Zero(9, 6);
      // rotation part
      // // H2->block<3,3>(0,3) = Matrix::Identity(3,3);
      // translation part
      // // H2->block<3,3>(3,0) = Matrix3::Identity(3,3);

    
   //Jacobian with respect to velocity1
    if(H3){
      
      *H3 = Matrix::Zero(9, 3);
    }
    //Jacobian with respect to velocity2
    if(H4){
      *H4 = Matrix::Zero(9, 3);
      H4->block<3,3>(6,0) = Matrix3::Identity();
      // Matrix H4_vel = gtsam::numericalDerivative11<Vector, Vector3>( 
      //   [this, pose1, pose2, velocity1](const Vector3& v2){
      //     return this->evaluateError(pose1, pose2, velocity1, v2);},velocity2);
      //   H4->block<3,3>(6,0) = H4_vel.block(6, 0, 3, 3);
      //   std::cout << "H4_vel: " << H4_vel << std::endl;
      // H4->block<3,3>(6,0) = Matrix3::Identity();
    }

  return error;
}

}  // namespace gtsam