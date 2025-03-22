#include <hydrobatic_localization/SamMotionModelFactor.h>

namespace gtsam {

Vector SamMotionModelFactor::evaluateError(const Pose3 &pose, const Vector3 &velocity,
                    gtsam::OptionalMatrixType H1, gtsam::OptionalMatrixType H2) const {


    //Jacobian with respect to pose
    if(H1) {

    }

    //Jacobian with respect to velocity
    if(H2){
    }

  return ();
}

}  // namespace gtsam