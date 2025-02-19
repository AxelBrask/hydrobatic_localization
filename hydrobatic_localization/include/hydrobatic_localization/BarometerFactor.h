#ifndef HYDROBATIC_LOCALIZATION_BAROMETERFACTOR_H
#define HYDROBATIC_LOCALIZATION_BAROMETERFACTOR_H

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
#include <boost/optional.hpp>
#include <memory>
#include <boost/optional/optional_io.hpp>
#include <iostream>
namespace gtsam {

class BarometerFactor : public NoiseModelFactor1<Pose3> {
 private:
  double measuredPressure_;

 public:
  using Base = NoiseModelFactor1<Pose3>;
  BarometerFactor(){};
  virtual ~BarometerFactor() {}  
  // Constructor.
  BarometerFactor(Key poseKey, double measuredPressure, const SharedNoiseModel& model)
      : Base(model, poseKey), measuredPressure_(measuredPressure) {}

  // Override evaluateError with the exact signature.
  Vector evaluateError(const Pose3 &pose,
                       gtsam::OptionalMatrixType H ) const override;


};

}  // namespace gtsam
#endif  // HYDROBATIC_LOCALIZATION_BAROMETERFACTOR_H