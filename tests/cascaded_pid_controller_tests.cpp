#include <gtest/gtest.h>

#include <gazebo_aerial_manipulation_plugin/cascaded_pid_controller.h>

using Time = gazebo::common::Time;

class CascadedPIDControllerControllerTests : public ::testing::Test {
protected:
  double p_gain_;
  double i_gain_;
  double d_gain_;
  double max_command_;
  double max_velocity_command_;
  double max_i_error_;
  CascadedPIDController controller_;
public:
  CascadedPIDControllerControllerTests(): p_gain_(5.0)
  , i_gain_(0.01)
  , d_gain_(5.0)
  , max_command_(10.0)
  , max_velocity_command_(1.0)
  , max_i_error_(0.2)
  , controller_(p_gain_, i_gain_, d_gain_, max_command_, max_velocity_command_, max_i_error_) {}
};

TEST_F(CascadedPIDControllerControllerTests, Constructor) {
  SUCCEED();
}

TEST_F(CascadedPIDControllerControllerTests, testConverged) {
  double out = controller_.update(1.0, 1.0, 0.0, 0.0);
  ASSERT_EQ(out, 0);
  controller_.reset();
  out = controller_.update(1.0, 1.0, 0.5, 0.0);
  ASSERT_NE(out, 0);
}

TEST_F(CascadedPIDControllerControllerTests, testLimitCommand) {
  double out = controller_.update(0.0, 5.0, 5.0, 0.0);
  ASSERT_EQ(out, -max_command_);
  out = controller_.update(0.0, -5.0, -5.0, 0.0);
  ASSERT_EQ(out, max_command_);
}

TEST_F(CascadedPIDControllerControllerTests, testLimitVelocity) {
  double command = controller_.update(0.0, 5.0, 0.0, 0.0);
  double expected_force = -d_gain_*(max_velocity_command_);
  ASSERT_EQ(expected_force, command);
}
TEST_F(CascadedPIDControllerControllerTests, testMaxIntegrator) {
  CascadedPIDController controller(p_gain_, 1.0, d_gain_, max_command_, max_velocity_command_, max_i_error_);
  double goal = 0.0;
  double p = 0.1;
  double v = 0.0;
  double command = controller.update(goal, p, v, 0.9);
  double expected_force = -d_gain_*(v + p_gain_*(p - goal)) + max_i_error_;
  ASSERT_EQ(expected_force, command);
}
TEST_F(CascadedPIDControllerControllerTests, testRunning) {
  double goal = 5.0;
  double p = 0.0;
  double v = 0.0; 
  double dt = 0.01;
  double command;
  double time = 0.0;
  for(int i = 0; i < 1000; i++) {
    time = time + dt;
    command = controller_.update(goal, p, v, time);
    v = v + command*dt;
    p = p + v*dt;
  }
  ASSERT_NEAR(p, goal, 1e-3);
  ASSERT_NEAR(v, 0.0, 1e-4);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
