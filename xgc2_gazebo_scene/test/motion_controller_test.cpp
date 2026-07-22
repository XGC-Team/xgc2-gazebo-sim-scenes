#include "xgc2_gazebo_scene/motion_controller.hpp"

#include <gtest/gtest.h>

#include <string>

namespace xgc2_gazebo_scene {
namespace {

TEST(MotionControllerTest, ConstantTwistUsesAbsoluteSimulationTime) {
    MotionConfiguration configuration;
    configuration.mode = MotionMode::kConstantTwist;
    configuration.linear_velocity.Set(2.0, -1.0, 0.5);
    configuration.angular_velocity.Set(0.0, 0.0, 1.0);
    MotionController controller;
    std::string error;
    ASSERT_TRUE(controller.Configure(configuration, ignition::math::Pose3d(1, 2, 3, 0, 0, 0), 10.0, &error)) << error;

    const MotionSample sample = controller.Sample(12.0);
    EXPECT_NEAR(sample.pose.Pos().X(), 5.0, 1e-9);
    EXPECT_NEAR(sample.pose.Pos().Y(), 0.0, 1e-9);
    EXPECT_NEAR(sample.pose.Pos().Z(), 4.0, 1e-9);
    EXPECT_NEAR(sample.pose.Rot().Yaw(), 2.0, 1e-9);
    EXPECT_EQ(controller.modeName(), "constant_twist");
}

TEST(MotionControllerTest, PingPongReversesAtBothEndpoints) {
    MotionConfiguration configuration;
    configuration.mode = MotionMode::kPingPong;
    configuration.waypoints = {{0, 0, 0}, {4, 0, 0}};
    configuration.speed = 2.0;
    MotionController controller;
    std::string error;
    ASSERT_TRUE(controller.Configure(configuration, ignition::math::Pose3d::Zero, 5.0, &error)) << error;

    EXPECT_NEAR(controller.Sample(6.0).pose.Pos().X(), 2.0, 1e-9);
    EXPECT_NEAR(controller.Sample(7.0).pose.Pos().X(), 4.0, 1e-9);
    EXPECT_NEAR(controller.Sample(8.0).pose.Pos().X(), 2.0, 1e-9);
    EXPECT_NEAR(controller.Sample(9.0).pose.Pos().X(), 0.0, 1e-9);
    EXPECT_LT(controller.Sample(8.0).linear_velocity.X(), 0.0);
}

TEST(MotionControllerTest, RejectsDegeneratePingPong) {
    MotionConfiguration configuration;
    configuration.mode = MotionMode::kPingPong;
    configuration.waypoints = {{1, 1, 1}, {1, 1, 1}};
    configuration.speed = 1.0;
    MotionController controller;
    std::string error;
    EXPECT_FALSE(controller.Configure(configuration, ignition::math::Pose3d::Zero, 0.0, &error));
    EXPECT_FALSE(error.empty());
}

} // namespace
} // namespace xgc2_gazebo_scene

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
