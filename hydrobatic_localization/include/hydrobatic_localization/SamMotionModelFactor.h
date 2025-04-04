#ifndef HYDROBATIC_LOCALIZATION_SAMMOTIONMODELFACTOR_H
#define HYDROBATIC_LOCALIZATION_SAMMOTIONMODELFACTOR_H

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
#include <boost/optional.hpp>
#include <memory>
#include <boost/optional/optional_io.hpp>
#include <gtsam/navigation/ImuBias.h>
#include <iostream>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/Matrix.h>
#include <hydrobatic_localization/SamMotionModel.h>
#include <rclcpp/rclcpp.hpp>
namespace gtsam {

class PreintegratedMotionModel
{
  private:
  //list of the motion model inputs:
  struct controlSequence {
      double timestamp;
      Eigen::VectorXd u;};
  std::vector<controlSequence> control_list_;
  SamMotionModelWrapper sam_motion_model_;
  controlSequence prev_control_;
  double dt_;

  public:
  /**
   * @brief Constructor for the PreintegratedMotionModel
   * @param dt: time step for the motion model
   */
  PreintegratedMotionModel(double dt): dt_(dt) ,sam_motion_model_(dt) {
    prev_control_.u = Eigen::VectorXd::Zero(6);
  }
  /**
   * @brief Resets the control list and sets the pre control
   */
  void resetIntegration() {
      prev_control_ = control_list_.back();
      control_list_.clear();

  }
  /**
   * @brief Predicts the next state from the current state and the control inputs
   * @param state: the current state
   * @return NavState: the predicted state
   */
  NavState predict(const NavState& state,const Vector3& gyro,  const double start_time, const double end_time) const;

  /**
   * @brief Function to add control inputs to the control seqeuence
   * @param u: the control input is either [4x1] or [2x1] depending if fb topic or command topic
   * @param timestamp: the timestamp of the message, this will be used to calculate the integration time
   * @param isThrusterVector: if true, u needs to be [2x1] and is a thruster vector cmd message, if false [4x1] and a fb control
   */
  void controlToList(const Eigen::VectorXd& u, const double& timestamp, const bool& isThrusterVector) ;
  
  /**     
   * @brief Function to convert the gtsam::NavState and gyro measurements to eta and nu for the motion model
   * @param state: the current NavState state from the optimizer in the navigation frame ENU
   * @param gyro: the raw gyro measurements from the IMU
   * @return Eigen::VectorXd: the state vector, eta and nu in NED. The linear and angluar velocities will tbe transformed to the base link frame 
   */
  Eigen::VectorXd stateToVector(const gtsam::NavState& state, const gtsam::Vector3 gyro) const;

  /** 
  * @brief Function to convert the state vector to a gtsam::NavState, converting from NED to ENU
  * @param state_vector: the state vector in NED
  * @return NavState: the gtsam::NavState object in NED
  */
  NavState vectorToState(const Eigen::VectorXd& state_vector) const ;

  std::vector<controlSequence> getControlList() const {
      return control_list_;
  }
  
  Eigen::VectorXd getPrevControl() const {
      return prev_control_.u;
  }

};

class SamMotionModelFactor : public NoiseModelFactor4<Pose3, Pose3, Vector3,Vector3> {
 private:
  PreintegratedMotionModel PPM_;
  double start_time_;
  double end_time_;
  Vector3 gyro_;
 public:
  using Base = NoiseModelFactor4<Pose3, Pose3, Vector3, Vector3>; //might need to include the bias for the gyro
  // SamMotionModelFactor(){};
  virtual ~SamMotionModelFactor() {}  
  /**
   * @brief Constructor
   * @param poseKey1 the previous key of the robot pose
   * @param velKey1 the previous key of the robot velocity
   * @param poseKey2 the current key of the robot pose
   * @param velKey2 the current key of the robot velocity
   * @param model the noise model for the factor
   * @param start_time the start time of the integration
   * @param end_time the end time of the integration
   * @param pmm the preintegrated motion model
   */
  SamMotionModelFactor(Key poseKey1,Key poseKey2, Key velKey1,Key velKey2 , const SharedNoiseModel& model, const double& start_time, const double& end_time,const PreintegratedMotionModel& pmm, const Vector3& gyro):
  Base(model, poseKey1, poseKey2, velKey1, velKey2), start_time_(start_time), end_time_(end_time), PPM_(pmm), gyro_(gyro) {}
       
   /**
    * @brief Evaluate the error
    * @param pose1 the previous pose to evaluate the error
    * @param pose2 the current pose to evaluate the error
    * @param velocity1: the previous estimated velocity in the navigation frame
    * @param velocity2: the current estimated velocity in the navigation frame
    * @return returns the residual
    */
  Vector evaluateError(const Pose3 &pose1, const Pose3& pose2,const Vector3 &velocity1, const Vector3& velocity2,
                    gtsam::OptionalMatrixType H1 = OptionalNone, gtsam::OptionalMatrixType H2 = OptionalNone, gtsam::OptionalMatrixType H3 = OptionalNone, gtsam::OptionalMatrixType H4 = OptionalNone) const override;


};

}  // namespace gtsam
#endif  // HYDROBATIC_LOCALIZATION_SAMMOTIONMODELFACTOR_H