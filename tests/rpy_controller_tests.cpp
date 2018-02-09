#include <gtest/gtest.h>

#include <gazebo_aerial_manipulation_plugin/rpycontroller.h>

using Vector3 = gazebo::math::Vector3;
using Time = gazebo::common::Time;
using Quaternion = gazebo::math::Quaternion;

class RpyControllerTests : public ::testing::Test {
protected:
  Vector3 p_gains_;
  Vector3 i_gains_;
  Vector3 d_gains_;
  double max_torque_;
  RpyController controller_;
public:
  RpyControllerTests(): p_gains_(5.0, 5.0, 5.0)
  , i_gains_(Vector3::Zero)
  , d_gains_(5.0, 5.0, 5.0)
  , max_torque_(5.0)
  , controller_(p_gains_, i_gains_, d_gains_, max_torque_) {}
};

TEST_F(RpyControllerTests, Constructor) {
  SUCCEED();
}

TEST_F(RpyControllerTests, testConverged) {
  Vector3 rpy_command(1.0, 1.0, 1.0);
  Quaternion orientation(rpy_command);
  Vector3 omega(0,0,0);
  // dt = 0, updates the internal rpy command but the output torque is 0
  controller_.update(rpy_command, orientation, omega, 0.0);
  Vector3 body_torque = controller_.update(rpy_command, orientation, omega, 0.01);
  ASSERT_EQ(body_torque.x, 0);
  ASSERT_EQ(body_torque.y, 0);
  ASSERT_EQ(body_torque.z, 0);
}

TEST_F(RpyControllerTests, testLimits) {
  Vector3 rpy_command(0.0, 0.0, 0.0);
  Quaternion orientation(Vector3::Zero);
  Vector3 omega(0,0,0);
  Vector3 body_torque = controller_.update(rpy_command, orientation, omega, 0.01);
  rpy_command = Vector3(2,2,2);
  body_torque = controller_.update(rpy_command, orientation, omega, 0.02);
  ASSERT_EQ(body_torque.x, max_torque_);
  ASSERT_EQ(body_torque.y, max_torque_);
  ASSERT_EQ(body_torque.z, max_torque_);
}

TEST_F(RpyControllerTests, testdT) {
  Vector3 rpy_command(1.0, 1.0, 1.0);
  Quaternion orientation(Vector3::Zero);
  Vector3 omega(0,0,0);
  Vector3 body_torque = controller_.update(rpy_command, orientation, omega, 0.01);
  Vector3 updated_body_torque = controller_.update(rpy_command, orientation, omega, 0.01001);
  ASSERT_EQ(body_torque.x, updated_body_torque.x);
  ASSERT_EQ(body_torque.y, updated_body_torque.y);
  ASSERT_EQ(body_torque.z, updated_body_torque.z);
  updated_body_torque = controller_.update(rpy_command, orientation, omega, 2.0);
  ASSERT_EQ(updated_body_torque.x, 0);
  ASSERT_EQ(updated_body_torque.y, 0);
  ASSERT_EQ(updated_body_torque.z, 0);
}

TEST_F(RpyControllerTests, testRunning) {
  Vector3 rpy_command(-0.5,0.5,1);
  Vector3 rpy(0,0,0);
  Vector3 omega(Vector3::Zero);
  gazebo::common::Time time(0);
  double dt = 0.01;
  for(int i = 0; i < 1000; i++) {
    time = time + dt;
    Quaternion orientation(rpy);
    Vector3 body_torque = controller_.update(rpy_command, orientation, omega, time);
    rpy =  rpy + controller_.omegaToRpydot(rpy, omega)*dt;
    omega = omega + body_torque*dt;
  }
  ASSERT_NEAR(rpy.x, rpy_command.x, 1e-2);
  ASSERT_NEAR(rpy.y, rpy_command.y, 1e-2);
  ASSERT_NEAR(rpy.z, rpy_command.z, 1e-2);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
