#include <hydrobatic_localization/SamMotionModelFactor.h>
#include <iomanip> 
namespace gtsam {



NavState PreintegratedMotionModel::predict(const NavState& state,const Vector3& gyro,
    const double start_time, const double end_time, const Eigen::MatrixXd& Sigma0) {

      // Convert the input NavState to a state vector using the provided gyro measurement.
      Eigen::VectorXd vectorState(19);
      vectorState.head(13) = stateToVector(state, gyro);     
      vectorState.tail(6) = prev_integrated_control_.u;  
      //if vectorState is only zeros takt the first control input
      if(vectorState.tail(6).isZero(6)){
          vectorState.tail(6) = prev_control_.u;
      }
      if (control_list_.empty()) {  
          return state;
      }

      Eigen::VectorXd integratedState = propagateStateVector(vectorState, start_time, end_time);

      // Convert the integrated state vector back to a NavState.
      NavState integratedNavState = vectorToState(integratedState, state);
      // deltaPose_
      deltaPose_ = state.pose().between(integratedNavState.pose());
      deltaVel_ = integratedNavState.velocity() - state.velocity();
      prev_integrated_control_.u = integratedState.tail(6);
      motion_model_prediction_state_ = integratedNavState; // Store the predicted state for later use
    return integratedNavState;


}

Eigen::VectorXd PreintegratedMotionModel::propagateStateVector(const Eigen::VectorXd& x,
        double t0, double t1)
        {
        Eigen::VectorXd integratedState = x;

        double currentTime = t0;
        size_t idx = 0;  // Index to track current control

        // Integrate from start_time to the first control input if there's a gap.
        if (idx < control_list_.size() && control_list_[0].timestamp > currentTime) {
            double dt = control_list_[0].timestamp - currentTime;
            integratedState = sam_motion_model_->integrateState(integratedState, prev_control_.u, dt);
            currentTime = control_list_[0].timestamp;
        }

        // Integrate over the control sequence until reaching end_time.
        for (; idx < control_list_.size()-1 && control_list_[idx+1].timestamp <= t1; idx++) {
            double dt = control_list_[idx+1].timestamp - currentTime;
            integratedState = sam_motion_model_->integrateState(integratedState, control_list_[idx].u, dt);
            currentTime = control_list_[idx+1].timestamp;
        }

        // Integrate from the last control to end_time if necessary.
        double dt = t1 - control_list_[idx].timestamp;

        if (dt > 0) {
            integratedState = sam_motion_model_->integrateState(integratedState, control_list_.back().u, dt);
        }
        return integratedState;
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
    Eigen::Matrix3d Rn = T * Re *B;  
    Eigen::Quaterniond qn(Rn);
    qn.normalize();

    Eigen::Vector3d v_b_enu = state.pose().rotation().matrix().transpose()
                              * state.velocity();
    Eigen::Vector3d un = B * v_b_enu; 

    Eigen::Vector3d gn = B* (gyro);

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
    Eigen::Matrix3d Rbn_e = T * Rbn_n * B;  
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
                  new_control.u = prev_control_.u; // Initialize with zeros
                  if (isThrusterVector) {
                    new_control.u.segment<2>(2) = u;
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
  const double tol = 1e-9;
  bool changed =
    !pose1.equals(nom_Ti, tol) ||
    !pose2.equals(nom_Tj, tol) ||
    !velocity1.isApprox(nom_velocity1, tol) ||
    !velocity2.isApprox(nom_velocity2, tol);

  if (changed) {
    // Recompute and cache everything exactly once
    nom_Ti        = pose1;
    nom_Tj        = pose2;
    nom_velocity1 = velocity1;
    nom_velocity2 = velocity2;

    Vector6 pose_err = Pose3::Logmap(
      PPM_.getDeltaPose().inverse()
        .compose(pose1.inverse().compose(pose2)));
    Vector3 vel_err  = velocity2 - (velocity1 + PPM_.getDeltaVel());
    nominal_error_.resize(9);
    nominal_error_ << pose_err, vel_err;

    // Compute and cache the Jacobians once here
    stored_H1_ = gtsam::numericalDerivative11<Vector,Pose3>(
      [this,pose2,velocity1,velocity2](auto&& p1){
        return this->rawError(p1, pose2, velocity1, velocity2);
      }, pose1);
    stored_H2_ = gtsam::numericalDerivative11<Vector,Pose3>(
      [this,pose1,velocity1,velocity2](auto&& p2){
        return this->rawError(pose1, p2, velocity1, velocity2);
      }, pose2);
    stored_H3_ = gtsam::numericalDerivative11<Vector,Vector3>(
      [this,pose1,pose2,velocity2](auto&& v1){
        return this->rawError(pose1, pose2, v1, velocity2);
      }, velocity1);
    stored_H4_.setZero(9,3);
    stored_H4_.block<3,3>(6,0) = Matrix3::Identity();
  }

  // Form the actual 1st‐order error approximation
  Vector error = nominal_error_  
             + stored_H1_ * Pose3::Logmap(nom_Ti.inverse().compose(pose1))
             + stored_H2_ * Pose3::Logmap(nom_Tj.inverse().compose(pose2))
             + stored_H3_ * (velocity1 - nom_velocity1)
             + stored_H4_ * (velocity2 - nom_velocity2);

  // Only assign jacobians from the cached matrices
  if (H1) *H1 = stored_H1_;
  if (H2) *H2 = stored_H2_;
  if (H3) *H3 = stored_H3_;
  if (H4) *H4 = stored_H4_;

  return error;
}





}  // namespace gtsam
