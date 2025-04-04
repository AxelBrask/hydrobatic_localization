#include <hydrobatic_localization/SamMotionModelFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <CppUnitLite/TestHarness.h>

#include <Eigen/Dense>

using namespace gtsam;
using namespace symbol_shorthand;

int main() {
  py::scoped_interpreter guard{};
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}

TEST(SamMotionModelFactor, EvaluateErrorAndJacobians) {

  Eigen::VectorXd predictedState(10);
  predictedState << 1.0, 2.0, 3.0,        // predicted position
                    0.9239, 0.0, 0.3827, 0.0, // predicted rotation (quaternion: w,x,y,z)
                    0.5, 0.0, 0.0;        // predicted velocity

  double delta_t = 0.1;
  PreintegratedMotionModel pmm(delta_t);
  double startIntegrationTime = 0.0;
  double endIntegrationTime = 0.3;


  Eigen::VectorXd fbControl(4);
  fbControl << 50.2875, 50.3017, 0.0, 0.0; // Feedback control input
  double timestamp = 0.0; // Timestamp of the control input
  pmm.controlToList(fbControl, timestamp, false);

  Eigen::VectorXd thrusterControl(2);
  thrusterControl << -0.06  ,  -0.1; // Thruster control input
  double timestamp2 = 0.2; // Timestamp of the control input
  pmm.controlToList(thrusterControl, timestamp2, true);


// create the factor
  auto noise = noiseModel::Unit::Create(9);
  SamMotionModelFactor factor(X(1), X(2), V(1), V(2),
                              noise,startIntegrationTime, endIntegrationTime, pmm, Vector3(0.0, 0.0, 0.0));

  // Create the poses and velocities for the factor
  Pose3 pose1(Rot3::Identity(), Point3(0.0, 0.0, 0.0));
  Pose3 pose2(Rot3::RzRyRx(0.1, 0.2, 0.3), Point3(1.5, 2.5, 3.5));
  Vector3 velocity1(0.5, 0.1, -0.1);  // could be arbitrary
  NavState state1(pose1, velocity1);


  NavState predictedState1 = pmm.predict(state1,Vector3(0.0,0.0,0.0), startIntegrationTime, endIntegrationTime);
  Vector3 velocity2(0.7, 0.0, 0.0);     // so that velocity error = (0.7,0,0)-(0.5,0,0) = (0.2,0,0)

  std::cout << "Predicted ENU translation state: " << predictedState1.pose().translation().transpose() << std::endl;
  std::cout << "Predicted ENU rotation state: " << predictedState1.pose().rotation().toQuaternion().coeffs().transpose() << std::endl;

  // Compute the expected error
  Vector6 poseError = Pose3::Logmap(predictedState1.pose().inverse().compose(pose2));// since pose1 is identity
  // Velocity error: velocity2 - predicted_velocity = (0.7,0,0) - (0.5,0,0) = (0.2,0,0)
  Vector3 expectedVelocity(0.2, 0.0, 0.0);
  Vector expectedError(9);
  expectedError <<  poseError, expectedVelocity;

  // Evaluate error using the factor.
  Vector error = factor.evaluateError(pose1, pose2, velocity1, velocity2);
  EXPECT(assert_equal(expectedError, error, 1e-4));

  // Insert the variables into a Values container.
  Values values;
  values.insert(X(1), pose1);
  values.insert(X(2), pose2);
  values.insert(V(1), velocity1);
  values.insert(V(2), velocity2);

  // Check that the Jacobians computed by the factor are correct.
  // The tolerance is set to 1e-5 for both error and Jacobian comparisons.
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-5, 1e-5);
}
