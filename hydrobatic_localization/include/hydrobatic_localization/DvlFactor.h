#ifndef HYDROBATIC_LOCALIZATION_DVLFACTOR_H
#define HYDROBATIC_LOCALIZATION_DVLFACTOR_H

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
#include <boost/optional.hpp>
#include <memory>
#include <boost/optional/optional_io.hpp>
#include <gtsam/navigation/ImuBias.h>
#include <iostream>
namespace gtsam {

class DvlFactor : public NoiseModelFactor3<Pose3, Vector3, imuBias::ConstantBias> {
 private:
    Vector3 dvl_velocity_measurement_;
    Vector3 base_link_gyro_measurement_;
    Vector3 base_to_dvl_offset_;
    Rot3 base_link_dvl_rotations_;

 public:
  using Base = NoiseModelFactor3<Pose3, Vector3, imuBias::ConstantBias>;
  DvlFactor(){};
  virtual ~DvlFactor() {}  
  // Constructor.
  DvlFactor(Key poseKey, Key velKey, Key bgyroKey, const Vector3 dvl_velocity_mesurment, const Vector3 base_link_gyro_measurement,
   const Vector3 base_to_dvl_offset,const Rot3 base_link_dvl_rotations, const SharedNoiseModel& model)
      : Base(model, poseKey, velKey, bgyroKey), dvl_velocity_measurement_(dvl_velocity_mesurment), base_link_gyro_measurement_(base_link_gyro_measurement),
       base_to_dvl_offset_(base_to_dvl_offset), base_link_dvl_rotations_(base_link_dvl_rotations) {}

  Vector evaluateError(const Pose3 &pose, const Vector3 &estimated_velocity, const imuBias::ConstantBias &bias_gyro,
                    gtsam::OptionalMatrixType H1, gtsam::OptionalMatrixType H2, gtsam::OptionalMatrixType H3) const override;


};

}  // namespace gtsam
#endif  // HYDROBATIC_LOCALIZATION_DVLFACTOR_H