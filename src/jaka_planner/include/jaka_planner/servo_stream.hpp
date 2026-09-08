#pragma once

#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include <array>
#include <string>
#include <vector>

namespace jaka_planner
{
constexpr std::size_t kServoJointCount = 6;
using ServoSample = std::array<double, kServoJointCount>;

struct ServoStreamConfig
{
  double period_seconds{0.008};
  double maximum_joint_velocity{3.14159265358979323846};
};

bool resampleTrajectory(
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  const ServoStreamConfig & config,
  std::vector<ServoSample> & samples,
  std::string & error);

bool validateCommandVelocity(
  const ServoSample & previous,
  const ServoSample & current,
  const ServoStreamConfig & config,
  std::string & error);
}  // namespace jaka_planner
