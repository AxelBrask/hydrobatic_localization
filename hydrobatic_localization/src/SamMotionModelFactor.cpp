#include <hydrobatic_localization/SamMotionModelFactor.h>

namespace gtsam {



NavState PreintegratedMotionModel::predict(const NavState& state,const Vector3& gyro,  const double start_time, const double end_time) {
  
      // Convert the input NavState to a state vector using the provided gyro measurement.
      Eigen::VectorXd vectorState(19);
      vectorState.head(13) = stateToVector(state, gyro);     
      vectorState.tail(6) = prev_control_.u;  
      
      // If there are no control inputs, return the input state.
      if (control_list_.empty()) {  
          return state;
      }

      // std::cout << "start time: " << start_time << ", end time: " << end_time << std::endl;
      //print the control list
      for (const auto& control : control_list_) {
      }
      Eigen::VectorXd integratedState = vectorState;
      // std::cout << " Size of integrated state: " << integratedState.size() << std::endl;
      double currentTime = start_time;
      size_t idx = 0;  // Index to track current control

      // Integrate from start_time to the first control input if there's a gap.
      if (idx < control_list_.size() && control_list_[0].timestamp > currentTime) {
          double dt = control_list_[0].timestamp - currentTime;
          integratedState = sam_motion_model_->integrateState(integratedState, prev_control_.u, dt);
          currentTime = control_list_[0].timestamp;

      }

      // Integrate over the control sequence until reaching end_time.
      for (; idx < control_list_.size()-1 && control_list_[idx+1].timestamp <= end_time; idx++) {
          // std::cout << "Integrating between control inputs" << std::endl;
          double dt = control_list_[idx+1].timestamp - currentTime;
          integratedState = sam_motion_model_->integrateState(integratedState, control_list_[idx].u, dt);
          currentTime = control_list_[idx+1].timestamp;
          // std::cout << "Control used: " << control_list_[idx].u.transpose() 
                    // << ", timestamp: " << control_list_[idx].timestamp << " dt: " << dt << std::endl;
          // std::cout<< "integrated state: " << integratedState.transpose() << std::endl;

      }

      // Integrate from the last control to end_time if necessary.
      double dt = end_time - control_list_[idx].timestamp;
      if (dt > 0) {
          // std::cout << "Integrating from last control input to end_time" << std::endl;
          integratedState = sam_motion_model_->integrateState(integratedState, control_list_.back().u, dt);
          // std::cout << "Control used: " << control_list_.back().u.transpose() 
                    // << ", timestamp: " << control_list_.back().timestamp << " dt: " << dt << std::endl;
                    // std::cout<< "integrated state: " << integratedState.transpose() << std::endl;
          // std::cout << "Final integrated state: " << integratedState.transpose() << std::endl;
      }

      // Convert the integrated state vector back to a NavState.
      NavState integratedNavState = vectorToState(integratedState, state);
      // deltaPose_
      deltaPose_ = state.pose().between(integratedNavState.pose());
      deltaVel_ = integratedNavState.velocity() - state.velocity();
    return integratedNavState;


}

Eigen::VectorXd PreintegratedMotionModel::stateToVector(const gtsam::NavState& state, const gtsam::Vector3 gyro) const {

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

NavState PreintegratedMotionModel::vectorToState(const Eigen::VectorXd& state_vector, const NavState& pose1 ) const{
  // Convert the translation and rotation from NED to ENU
    Eigen::Matrix3d R_n2e;
    R_n2e << 0, 1, 0,
            1, 0, 0,
            0, 0, -1;

    // Convert the translation: from NED to ENU
    Eigen::Vector3d t_ned = state_vector.head<3>();  
    Eigen::Vector3d t_enu;
    t_enu << t_ned(1), t_ned(0), -t_ned(2);

    // Get the NED rotation matrix from the nav state
    Eigen::Quaterniond q_ned(state_vector(3), state_vector(4), state_vector(5), state_vector(6));
    q_ned.normalize();  // Ensure the quaternion is normalized

    // Convert it to ENU by applying the transformation on both sides
    Eigen::Matrix3d R_enu = R_n2e * q_ned.toRotationMatrix() * R_n2e.transpose();

    // Convert the rotation matrix to a quaternion (in ENU)
    Eigen::Quaterniond q_enu(R_enu);

    // Linear velocity
    Vector3 nu = state_vector.segment<3>(7);  
    //rotate the linear velocity from base link frame to ENU
    Vector3 world_velocity = pose1.rotation().matrix()*nu;
    gtsam::Rot3 rot_enu = gtsam::Rot3(R_enu);  
    gtsam::Point3 trans_enu(t_enu);         
    // Create the NavState object
    NavState state(Pose3(rot_enu, trans_enu), world_velocity);
  return state;
}

void PreintegratedMotionModel::controlToList(const Eigen::VectorXd& u, const double& timestamp, const bool& isThrusterVector) {
          // add the first input to the qeue
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
          control_list_.push_back(latest_control);

  }




Vector SamMotionModelFactor::evaluateError(const Pose3 &pose1, const Pose3& pose2,const Vector3 &velocity1,
           const Vector3& velocity2,gtsam::OptionalMatrixType H1 ,
            gtsam::OptionalMatrixType H2 , gtsam::OptionalMatrixType H3 , gtsam::OptionalMatrixType H4) const {

    Pose3 Ti = pose1; 
    Pose3 Tj = pose2; 



    Vector6 pose_error = Pose3::Logmap(PPM_.getDeltaPose().inverse().compose(Ti.inverse().compose(Tj)));
    Vector3 vel_error = velocity2 - (velocity1 + PPM_.getDeltaVel());

    Vector error(9);
    error << pose_error, vel_error;

    //Jacobian with respect to pose1
    if(H1) {
        *H1 = Matrix::Zero(9, 6);
        // the jacobian w.r.t to rotation we use the numerical solver
        Matrix H_rot = gtsam::numericalDerivative11<Vector, Pose3>( 
        [this, pose2, velocity1, velocity2](const Pose3& p1){
          return this->evaluateError(p1, pose2, velocity1, velocity2);},pose1);

        H1->block<9,6>(0,0) = H_rot.block(0, 0, 9, 6);
        // std::cout << "H_rot: " << H_rot << std::endl;
      }
    //Jacobian with respect to pose2
    if(H2){
      *H2 = Matrix::Zero(9, 6);
      // Matrix6 H2_ana = H_A_logA * H_TiInvTj_A * H_Tj_TiInvTj*H_x_i_Tj;
        Matrix H2_pose = gtsam::numericalDerivative11<Vector, Pose3>( 
        [this, pose1, velocity1, velocity2](const Pose3& p2){
          return this->evaluateError(pose1, p2, velocity1, velocity2);},pose2);
        // H2->block<6,6>(0,0) = H2_ana.block(0, 0, 6,6);
        H2->block<9,6>(0,0) = H2_pose.block(0, 0, 9,6);

    }


   //Jacobian with respect to velocity1
    if(H3){
      *H3 = Matrix::Zero(9, 3);
        Matrix H3_num = gtsam::numericalDerivative11<Vector, Vector3>( 
        [this, pose1,pose2, velocity2 ](const Vector3& v1){
          return this->evaluateError(pose1, pose2, v1, velocity2);},velocity1);
        H3->block<9,3>(0,0) = H3_num.block(0, 0, 9, 3);
    }
    //Jacobian with respect to velocity2
    if(H4){
      *H4 = Matrix::Zero(9, 3);
      H4->block<3,3>(6,0) = Matrix3::Identity();

    }

  return error;
}

}  // namespace gtsam