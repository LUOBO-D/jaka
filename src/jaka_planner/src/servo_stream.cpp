#include "jaka_planner/servo_stream.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace jaka_planner
{
namespace
{
double seconds(const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<double>(duration.sec) + static_cast<double>(duration.nanosec) * 1e-9;
}
}  // namespace

bool validateCommandVelocity(
  const ServoSample & previous,
  const ServoSample & current,
  const ServoStreamConfig & config,
  std::string & error)
{
  if (config.period_seconds <= 0.0 || config.maximum_joint_velocity <= 0.0) {
    error = "servo stream period and velocity limit must be positive";
    return false;
  }
  for (std::size_t joint = 0; joint < kServoJointCount; ++joint) {
    const double velocity = std::abs(current[joint] - previous[joint]) / config.period_seconds;
    if (!std::isfinite(velocity) || velocity > config.maximum_joint_velocity + 1e-9) {
      error = "resampled command exceeds the joint velocity limit at joint_" +
        std::to_string(joint + 1);
      return false;
    }
  }
  return true;
}

bool resampleTrajectory(
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  const ServoStreamConfig & config,
  std::vector<ServoSample> & samples,
  std::string & error)
{
  samples.clear();
  if (config.period_seconds <= 0.0 || config.maximum_joint_velocity <= 0.0) {
    error = "servo stream configuration must be positive";
    return false;
  }
  if (trajectory.points.size() < 2 ||
    std::abs(seconds(trajectory.points.front().time_from_start)) > 1e-9)
  {
    error = "trajectory must have at least two points and start at time zero";
    return false;
  }
  for (const auto & point : trajectory.points) {
    if (point.positions.size() != kServoJointCount) {
      error = "trajectory point does not contain six positions";
      return false;
    }
  }

  const double duration = seconds(trajectory.points.back().time_from_start);
  if (!std::isfinite(duration) || duration <= 0.0) {
    error = "trajectory duration must be positive";
    return false;
  }
  const std::size_t intervals = static_cast<std::size_t>(
    std::ceil(duration / config.period_seconds));
  samples.reserve(intervals + 1);

  std::size_t segment = 0;
  for (std::size_t sample_index = 0; sample_index <= intervals; ++sample_index) {
    const double sample_time = std::min(
      static_cast<double>(sample_index) * config.period_seconds, duration);
    while (segment + 1 < trajectory.points.size() - 1 &&
      sample_time > seconds(trajectory.points[segment + 1].time_from_start))
    {
      ++segment;
    }
    const auto & from = trajectory.points[segment];
    const auto & to = trajectory.points[segment + 1];
    const double from_time = seconds(from.time_from_start);
    const double segment_duration = seconds(to.time_from_start) - from_time;
    if (segment_duration <= 0.0) {
      error = "trajectory timestamps must be strictly increasing";
      return false;
    }
    const double ratio = std::clamp((sample_time - from_time) / segment_duration, 0.0, 1.0);
    const bool has_velocities = from.velocities.size() == kServoJointCount &&
      to.velocities.size() == kServoJointCount;
    ServoSample sample{};
    for (std::size_t joint = 0; joint < kServoJointCount; ++joint) {
      if (has_velocities) {
        const double ratio2 = ratio * ratio;
        const double ratio3 = ratio2 * ratio;
        sample[joint] =
          (2.0 * ratio3 - 3.0 * ratio2 + 1.0) * from.positions[joint] +
          (ratio3 - 2.0 * ratio2 + ratio) * segment_duration * from.velocities[joint] +
          (-2.0 * ratio3 + 3.0 * ratio2) * to.positions[joint] +
          (ratio3 - ratio2) * segment_duration * to.velocities[joint];
      } else {
        sample[joint] = from.positions[joint] + ratio * (to.positions[joint] - from.positions[joint]);
      }
      if (!std::isfinite(sample[joint])) {
        error = "trajectory interpolation produced a non-finite position";
        return false;
      }
    }
    if (!samples.empty() && !validateCommandVelocity(samples.back(), sample, config, error)) {
      samples.clear();
      return false;
    }
    samples.push_back(sample);
  }
  return true;
}
}  // namespace jaka_planner
