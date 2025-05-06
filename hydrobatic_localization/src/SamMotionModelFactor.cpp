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

      std::cout << "start time: " << start_time << ", end time: " << end_time << std::endl;
      //print the control list
      Eigen::VectorXd integratedState = vectorState;
      // std::cout << " Size of integrated state: " << integratedState.size() << std::endl;
      double currentTime = start_time;
      size_t idx = 0;  // Index to track current control

      // Integrate from start_time to the first control input if there's a gap.
      if (idx < control_list_.size() && control_list_[0].timestamp > currentTime) {
          double dt = control_list_[0].timestamp - currentTime;
          // std::cout << "Integrating from start_time to first control input" << std::endl;
          //  std::cout << "Control used: " << prev_control_.u.transpose() 
                    // << ", timestamp: " << prev_control_.timestamp << " dt: " << dt << std::endl;
          integratedState = sam_motion_model_->integrateState(integratedState, prev_control_.u, dt);
          currentTime = control_list_[0].timestamp;
          // std::cout<< "integrated state: " << integratedState.transpose() << std::endl;
// 
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
Eigen::VectorXd PreintegratedMotionModel::stateToVector(
    const gtsam::NavState& state,
    const gtsam::Vector3 gyro) const
{
    Eigen::Matrix3d T;
    T << 0, 1,  0,
         1, 0,  0,
         0, 0, -1;
    Eigen::Matrix3d B ;
    B << 1, 0, 0,
         0, -1,  0,
         0, 0,  -1;

    // ENU to NED translation
    Eigen::Vector3d te = state.pose().translation();
    Eigen::Vector3d tn = T * te;  // [ y_e, x_e, -z_e ]

    // ENU to NED orientation
    Eigen::Matrix3d Re = state.pose().rotation().matrix();
    Eigen::Matrix3d Rn = T * Re ;  
    Eigen::Quaterniond qn(Rn);
    qn.normalize();

    Eigen::Vector3d v_b_enu = state.pose().rotation().matrix().transpose()
                              * state.velocity();
    Eigen::Vector3d un = B * v_b_enu; 

    Eigen::Vector3d gn = B* gyro;

    Eigen::VectorXd eta(7), nu(6), x(13);
    eta << tn.x(), tn.y(), tn.z(),
           qn.w(), qn.x(), qn.y(), qn.z();
    nu  << un.x(), un.y(), un.z(),
           gn.x(), gn.y(), gn.z();
    x << eta, nu;
    return x;
}

NavState PreintegratedMotionModel::vectorToState(
    const Eigen::VectorXd& xv,
    const NavState& /*unused*/) const
{
    Eigen::Matrix3d T;
    T << 0, 1,  0,
         1, 0,  0,
         0, 0, -1;
        Eigen::Matrix3d B ;
    B << 1, 0, 0,
         0, -1,  0,
         0, 0,  -1;
    //  NED state
    Eigen::Vector3d tn    = xv.head<3>();
    Eigen::Quaterniond qn(xv[3], xv[4], xv[5], xv[6]);
    qn.normalize();
    Eigen::Vector3d un    = xv.segment<3>(7);

    //  NED to ENU translation
    Eigen::Vector3d te = T * tn;  // [ y_n, x_n, -z_n ]

    // NED to ENU orientation 
    Eigen::Matrix3d Rbn_n = qn.toRotationMatrix();
    Eigen::Matrix3d Rbn_e = T * Rbn_n ;  
    gtsam::Pose3 pose_e{ gtsam::Rot3(Rbn_e), gtsam::Point3(te) };

  //velocity in ENU
    Eigen::Vector3d v_b_enu = B * un;          
    Eigen::Vector3d ve      = Rbn_e * v_b_enu; 
                                         
    return NavState(pose_e, gtsam::Vector3(ve));
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




Vector SamMotionModelFactor::evaluateError(
    const Pose3 &pose1, const Pose3& pose2,
    const Vector3 &velocity1, const Vector3& velocity2,
    gtsam::OptionalMatrixType H1, gtsam::OptionalMatrixType H2,
    gtsam::OptionalMatrixType H3, gtsam::OptionalMatrixType H4) const
{
  Vector error(9);
  // 1) Compute the true “nominal” error:
  Pose3 Ti = pose1;
  Pose3 Tj = pose2;
  Vector6 pose_err = Pose3::Logmap(PPM_.getDeltaPose()
                    .inverse()
                    .compose(Ti.inverse().compose(Tj)));
  Vector3 vel_err  = velocity2 - (velocity1 + PPM_.getDeltaVel());


  if(nominal) {
    nom_Ti        = pose1;
    nom_Tj        = pose2;
    nom_velocity1 = velocity1;
    nom_velocity2 = velocity2;
    nominal_error_.resize(9);
    nominal_error_ << pose_err, vel_err;
    // 2) On *first* call: compute & stash *all* jacobians:
    stored_H1_ = gtsam::numericalDerivative11<Vector,Pose3>(
      [this,pose2,velocity1,velocity2](const Pose3& p1){
        return this->rawError(p1, pose2, velocity1, velocity2);
      }, pose1);

    stored_H2_ = gtsam::numericalDerivative11<Vector,Pose3>(
      [this,pose1,velocity1,velocity2](const Pose3& p2){
        return this->rawError(pose1, p2, velocity1, velocity2);
      }, pose2);

    stored_H3_ = gtsam::numericalDerivative11<Vector,Vector3>(
      [this,pose1,pose2,velocity2](const Vector3& v1){
        return this->rawError(pose1, pose2, v1, velocity2);
      }, velocity1);

    // we know ∂err/∂v2 is trivial:
    stored_H4_.setZero(9,3);
    stored_H4_.block<3,3>(6,0) = Matrix3::Identity();

    // flip the flag, and return the raw nominal error:
    nominal = false;
    
    error << nominal_error_;
  }
  else{
  // 3) On subsequent calls: build your Taylor‐approximation:
  Vector err = nominal_error_
             + stored_H1_ * Pose3::Logmap(nom_Ti.inverse().compose(Ti))
             + stored_H2_ * Pose3::Logmap(nom_Tj.inverse().compose(Tj))
             + stored_H3_ * (velocity1 - nom_velocity1)
             + stored_H4_ * (velocity2 - nom_velocity2);
  error << err;
  }
  // 4) Now, if the solver *did* request H1…H4, just hand out your cached blocks:
  if(H1) *H1  = gtsam::numericalDerivative11<Vector,Pose3>(
      [this,pose2,velocity1,velocity2](const Pose3& p1){
        return this->rawError(p1, pose2, velocity1, velocity2);
      }, pose1);


    
  if(H2) *H2 = gtsam::numericalDerivative11<Vector,Pose3>(
      [this,pose1,velocity1,velocity2](const Pose3& p2){
        return this->rawError(pose1, p2, velocity1, velocity2);
      }, pose2);;
  if(H3) *H3 = gtsam::numericalDerivative11<Vector,Vector3>(
      [this,pose1,pose2,velocity2](const Vector3& v1){
        return this->rawError(pose1, pose2, v1, velocity2);
      }, velocity1);
  if(H4) *H4 = stored_H4_;
  
  return error;
}





}  // namespace gtsam
