#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/linear/NoiseModel.h>
#include <hydrobatic_localization/BarometerFactor.h>
#include <gtsam/nonlinear/factorTesting.h>
//CPP Unit Test
#include <CppUnitLite/TestHarness.h>
#include <cmath>

using namespace gtsam;
using namespace symbol_shorthand;

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
TEST(NonlinearFactor, BarometerFactor) {
  // Create a factor with:
  // - pose key X(1)
  // - a base_llink to pressure offset corresponding to sam
  // - a measured depth of -8
  // - a estimated base link depth of -10
  // - error = (-10+0.057) - (-8) = -1.943 since we have not rotation

  BarometerFactor factor(X(1), -8.0, Vector3(-0.503, 0.025, 0.057),
                         noiseModel::Unit::Create(1));

  // Define a test pose: identity rotation and translation (0,0,10)
  Pose3 pose(Rot3::Identity(), Point3(0.0, 0.0, -10.0));
  Vector error = factor.evaluateError(pose);
  EXPECT_DOUBLES_EQUAL(-1.943, error(0), 1e-9);

  // Create a Values container and insert the pose.
  Values values;
  values.insert(X(1), pose);
  
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-5, 1e-5);
}

TEST(NonlinearFactor1, BarometerFactor) {
  // Create a factor with:

  Vector3 base_to_pressure_offset(-0.503, 0.025, 0.057);
  BarometerFactor factor(X(1), -8.0, base_to_pressure_offset,
                         noiseModel::Unit::Create(1));

  Pose3 pose(Rot3::Rx(M_PI/4), Point3(0.0, 0.0, -10.0));
  double rotated_offset = base_to_pressure_offset.y()*sin(M_PI/4) + base_to_pressure_offset.z()*cos(M_PI/4);
  double expected_depth = pose.translation().z() + rotated_offset;
  double expected_error = expected_depth - (-8.0);
  Vector error2 = factor.evaluateError(pose);
  EXPECT_DOUBLES_EQUAL(expected_error, error2(0), 1e-9);

  // Create a Values container and insert the pose.
  Values values;
  values.insert(X(1), pose);
  
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-5, 1e-5);
}
