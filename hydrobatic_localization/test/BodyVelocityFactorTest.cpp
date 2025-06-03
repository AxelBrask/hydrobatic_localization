#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/linear/NoiseModel.h>
#include <hydrobatic_localization/BodyVelocityFactor.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <CppUnitLite/TestHarness.h>
#include <cmath>

using namespace gtsam;
using namespace symbol_shorthand;

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}

TEST(NonlinearFactor, BodyVelocityFactorIdentity) {
  Vector3 v_body_meas(1.0, 2.0, 3.0);

  auto noise = noiseModel::Unit::Create(3);
  BodyVelocityFactor factor(X(1), V(1), v_body_meas, noise);

  Pose3 pose(Rot3::Identity(), Point3(0.0, 0.0, 0.0));
  Vector3 v_world(1.0, 2.0, 3.0);

  Vector error = factor.evaluateError(pose, v_world);
  EXPECT(assert_equal(Vector3::Zero(), error));

  Values values;
  values.insert(X(1), pose);
  values.insert(V(1), v_world);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-5, 1e-5);
}

TEST(NonlinearFactor1, BodyVelocityFactorYaw90) {
  Vector3 v_world(1.0, 0.0, 0.0);
  Vector3 v_body_meas(0.0, -1.0, 0.0);

  auto noise = noiseModel::Unit::Create(3);
  BodyVelocityFactor factor(X(1), V(2), v_body_meas, noise);

  Pose3 pose(Rot3::Rz(M_PI_2), Point3(0.0, 0.0, 0.0));

  Vector error = factor.evaluateError(pose, v_world);
  EXPECT(assert_equal(Vector3::Zero(), error));

  Values values;
  values.insert(X(1), pose);
  values.insert(V(2), v_world);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-5, 1e-5);
}
