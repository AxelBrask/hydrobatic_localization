#ifndef HYDROBATIC_LOCALIZATION_SAMMOTIONMODELFACTOR_H
#define HYDROBATIC_LOCALIZATION_SAMMOTIONMODELFACTOR_H

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
#include <boost/optional.hpp>s
#include <memory>
#include <boost/optional/optional_io.hpp>
#include <gtsam/navigation/ImuBias.h>
#include <iostream>

namespace gtsam {

class SamMotionModelFactor : public NoiseModelFactor2<Pose3, Vector3> {
 private:

 public:
  using Base = NoiseModelFactor2<Pose3, Vector3>; //might need to include the bias for the gyro
  SamMotionModelFactor(){};
  virtual ~SamMotionModelFactor() {}  
  /**
   * @brief Constructor
   * @param poseKey the key of the robot pose
   * @param velKey the key of the robot velocity
   * @param model the noise model for the factor
   */
  SamMotionModelFactor(Key poseKey, Key velKey , const SharedNoiseModel& model, const double& integration_time, )
      : Base(model, poseKey, velKey){

      }
       
   /**
    * @brief Evaluate the error
    * @param pose the pose to evaluate the error
    * @param velocity: the estimated velocity in the navigation frame
    * @return returns the residual
    */
  Vector evaluateError(const Pose3 &pose, const Vector3 &velocity,
                    gtsam::OptionalMatrixType H1 = OptionalNone, gtsam::OptionalMatrixType H2 = OptionalNone) const override;


};

}  // namespace gtsam
#endif  // HYDROBATIC_LOCALIZATION_SAMMOTIONMODELFACTOR_H