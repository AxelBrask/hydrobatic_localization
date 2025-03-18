#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/linear/NoiseModel.h>
#include <hydrobatic_localization/DvlFactor.h>
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
TEST(NonlinearFactor, DvlFactor) {
  // Create a factor with:
  // - pose key X(1)
  // - a base_llink to pressure offset corresponding to sam
  // - a measured depth of -8
  // - a estimated base link depth of -10
  // - error = (-10+0.057) - (-8) = -1.943 since we have not rotation

  DvlFactor factor(X(1),V(1),B(1), Vector3(1,0,0), Vector3(0,0,1), Vector3(0.573 ,0.0 ,-0.063),Rot3::Identity(),
                         noiseModel::Unit::Create(3));

Pose3 pose(Rot3::Identity(), Point3(0.0, 0.0, 0.0)); 
Vector3 expected_velocity = Vector3(1.0, 0.2, 0.0);
imuBias::ConstantBias bias_gyro = imuBias::ConstantBias(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0));
  Vector error = factor.evaluateError(pose,expected_velocity,bias_gyro);
    EXPECT_DOUBLES_EQUAL(0.0, error(0), 1e-9);
    EXPECT_DOUBLES_EQUAL(0.773, error(1), 1e-9);
    EXPECT_DOUBLES_EQUAL(0.0, error(2), 1e-9);
    




  // Create a Values container and insert the pose.
  Values values;
  values.insert(X(1), pose);
  values.insert(V(1), expected_velocity);
values.insert(B(1), bias_gyro);
  
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-5, 1e-5);
}

TEST(NonlinearFactor1, DvlFactor) {
//world to base link rotation +90 degree around z axis
Rot3 world_to_base_link = Rot3(0.0,-1.0,0,
                                1.0,0.0,0.0,
                                0.0,0.0,1.0);
DvlFactor factor(X(1),V(1),B(1), Vector3(1,0,0), Vector3(0,0,1), Vector3(0.573 ,0.0 ,-0.063),Rot3::Identity(),
                         noiseModel::Unit::Create(3));

Pose3 pose(Rot3 (0.0,-1.0,0,
                                1.0,0.0,0.0,
                                0.0,0.0,1.0), Point3(0.0, 0.0, 0.0)); 
Vector3 expected_velocity = Vector3(1.0, 0.2, 0.0);
imuBias::ConstantBias bias_gyro = imuBias::ConstantBias(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0));
  Vector error = factor.evaluateError(pose,expected_velocity,bias_gyro);
    EXPECT_DOUBLES_EQUAL(-0.8, error(0), 1e-9);
    EXPECT_DOUBLES_EQUAL(-0.427, error(1), 1e-9);
    EXPECT_DOUBLES_EQUAL(0.0, error(2), 1e-9);
    




  // Create a Values container and insert the pose.
  Values values;
  values.insert(X(1), pose);
  values.insert(V(1), expected_velocity);
  values.insert(B(1), bias_gyro);
  
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-5, 1e-5);
}
