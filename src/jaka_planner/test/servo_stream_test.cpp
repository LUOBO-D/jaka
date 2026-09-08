#include "jaka_planner/servo_stream.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>

namespace
{
trajectory_msgs::msg::JointTrajectory linearTrajectory(double end_position, double duration)
{
  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
  trajectory.points.resize(2);
  trajectory.points[0].positions.assign(6, 0.0);
  trajectory.points[1].positions.assign(6, 0.0);
  trajectory.points[1].positions[0] = end_position;
  const auto total_nanoseconds = static_cast<std::int64_t>(std::llround(duration * 1e9));
  trajectory.points[1].time_from_start.sec = static_cast<int32_t>(total_nanoseconds / 1000000000);
  trajectory.points[1].time_from_start.nanosec =
    static_cast<uint32_t>(total_nanoseconds % 1000000000);
  return trajectory;
}
}  // namespace

TEST(ServoStream, ResamplesAtEightMilliseconds)
{
  const auto trajectory = linearTrajectory(0.01, 0.016);
  jaka_planner::ServoStreamConfig config;
  std::vector<jaka_planner::ServoSample> samples;
  std::string error;
  ASSERT_TRUE(jaka_planner::resampleTrajectory(trajectory, config, samples, error)) << error;
  ASSERT_EQ(samples.size(), 3U);
  EXPECT_NEAR(samples[0][0], 0.0, 1e-12);
  EXPECT_NEAR(samples[1][0], 0.005, 1e-12);
  EXPECT_NEAR(samples[2][0], 0.01, 1e-12);
}

TEST(ServoStream, RejectsCommandsAboveRobotVelocityLimit)
{
  const auto trajectory = linearTrajectory(0.1, 0.008);
  jaka_planner::ServoStreamConfig config;
  std::vector<jaka_planner::ServoSample> samples;
  std::string error;
  EXPECT_FALSE(jaka_planner::resampleTrajectory(trajectory, config, samples, error));
  EXPECT_NE(error.find("velocity limit"), std::string::npos);
}

TEST(ServoStream, RequiresTrajectoryToStartAtZero)
{
  auto trajectory = linearTrajectory(0.01, 0.016);
  trajectory.points[0].time_from_start.nanosec = 1000000;
  jaka_planner::ServoStreamConfig config;
  std::vector<jaka_planner::ServoSample> samples;
  std::string error;
  EXPECT_FALSE(jaka_planner::resampleTrajectory(trajectory, config, samples, error));
}
