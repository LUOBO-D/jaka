#include "jaka_planner/trajectory_selector.hpp"

#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace jaka_planner
{
std::vector<double> toRadians(const std::vector<double> & degrees)
{
  std::vector<double> radians;
  radians.reserve(degrees.size());
  for (const double value : degrees) {
    radians.push_back(value * kDegreesToRadians);
  }
  return radians;
}

double computeJointTravel(const std::vector<std::vector<double>> & positions)
{
  double travel = 0.0;
  for (std::size_t point = 1; point < positions.size(); ++point) {
    if (positions[point - 1].size() != positions[point].size()) {
      throw std::invalid_argument("Trajectory points have inconsistent dimensions");
    }
    for (std::size_t joint = 0; joint < positions[point].size(); ++joint) {
      travel += std::abs(positions[point][joint] - positions[point - 1][joint]);
    }
  }
  return travel;
}

double computeScore(
  double duration, double joint_travel, double minimum_singular_value,
  const SelectionConfig & config)
{
  if (minimum_singular_value <= 0.0) {
    return -std::numeric_limits<double>::infinity();
  }
  return -(
    config.time_weight * duration + config.joint_travel_weight * joint_travel +
    config.singularity_weight / minimum_singular_value);
}

int interpolationSteps(
  const std::vector<double> & from, const std::vector<double> & to, double max_step)
{
  if (from.size() != to.size() || max_step <= 0.0) {
    throw std::invalid_argument("Invalid interpolation input");
  }
  double maximum_delta = 0.0;
  for (std::size_t joint = 0; joint < from.size(); ++joint) {
    maximum_delta = std::max(maximum_delta, std::abs(to[joint] - from[joint]));
  }
  return std::max(1, static_cast<int>(std::ceil(maximum_delta / max_step)));
}

bool passesSingularityLimits(
  double minimum_singular_value, double maximum_condition_number,
  const SelectionConfig & config)
{
  return std::isfinite(minimum_singular_value) && std::isfinite(maximum_condition_number) &&
         minimum_singular_value >= config.minimum_singular_value &&
         maximum_condition_number <= config.maximum_condition_number;
}

bool validSelectionConfig(const SelectionConfig & config)
{
  return config.candidate_count >= 1 && config.candidate_count <= 100 &&
         config.max_attempts >= config.candidate_count && config.max_attempts <= 1000 &&
         config.time_weight >= 0.0 && config.joint_travel_weight >= 0.0 &&
         config.singularity_weight >= 0.0 &&
         (config.time_weight > 0.0 || config.joint_travel_weight > 0.0 ||
         config.singularity_weight > 0.0) && config.minimum_singular_value > 0.0 &&
         config.maximum_condition_number > 1.0 && config.singularity_check_step > 0.0;
}

namespace
{
bool evaluateTrajectory(
  Candidate & candidate, const moveit::core::RobotState & reference_state,
  const moveit::core::JointModelGroup * joint_model_group, const SelectionConfig & config)
{
  const auto & trajectory_points = candidate.plan.trajectory_.joint_trajectory.points;
  if (trajectory_points.empty()) {
    return false;
  }

  std::vector<std::vector<double>> positions;
  positions.reserve(trajectory_points.size());
  for (const auto & point : trajectory_points) {
    if (point.positions.size() != joint_model_group->getVariableCount()) {
      return false;
    }
    positions.push_back(point.positions);
  }

  double minimum_singular_value = std::numeric_limits<double>::infinity();
  double maximum_condition_number = 0.0;
  auto state = reference_state;
  auto check_position = [&](const std::vector<double> & position) {
      state.setJointGroupPositions(joint_model_group, position);
      state.update();
      const Eigen::MatrixXd jacobian = state.getJacobian(joint_model_group);
      if (jacobian.rows() == 0 || jacobian.cols() == 0) {
        return false;
      }
      const Eigen::JacobiSVD<Eigen::MatrixXd> svd(
        jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
      const auto & values = svd.singularValues();
      if (values.size() == 0) {
        return false;
      }
      const double point_minimum = values.minCoeff();
      const double point_maximum = values.maxCoeff();
      const double condition = point_minimum > std::numeric_limits<double>::epsilon() ?
        point_maximum / point_minimum : std::numeric_limits<double>::infinity();
      minimum_singular_value = std::min(minimum_singular_value, point_minimum);
      maximum_condition_number = std::max(maximum_condition_number, condition);
      return true;
    };

  if (!check_position(positions.front())) {
    return false;
  }
  for (std::size_t point = 1; point < positions.size(); ++point) {
    const int steps = interpolationSteps(
      positions[point - 1], positions[point], config.singularity_check_step);
    std::vector<double> interpolated(positions[point].size());
    for (int step = 1; step <= steps; ++step) {
      const double ratio = static_cast<double>(step) / static_cast<double>(steps);
      for (std::size_t joint = 0; joint < interpolated.size(); ++joint) {
        interpolated[joint] = positions[point - 1][joint] +
          ratio * (positions[point][joint] - positions[point - 1][joint]);
      }
      if (!check_position(interpolated)) {
        return false;
      }
    }
  }

  if (!passesSingularityLimits(minimum_singular_value, maximum_condition_number, config)) {
    return false;
  }
  const auto duration = trajectory_points.back().time_from_start;
  candidate.duration_seconds =
    static_cast<double>(duration.sec) + static_cast<double>(duration.nanosec) * 1e-9;
  candidate.joint_travel_radians = computeJointTravel(positions);
  candidate.minimum_singular_value = minimum_singular_value;
  candidate.maximum_condition_number = maximum_condition_number;
  candidate.score = computeScore(
    candidate.duration_seconds, candidate.joint_travel_radians,
    candidate.minimum_singular_value, config);
  return std::isfinite(candidate.score);
}
}  // namespace

std::optional<Candidate> selectBestTrajectory(
  moveit::planning_interface::MoveGroupInterface & move_group,
  const moveit::core::RobotState & from_state,
  const moveit::core::JointModelGroup * joint_model_group,
  const std::vector<double> & target_radians,
  const SelectionConfig & config,
  const std::string & label,
  const rclcpp::Logger & logger)
{
  if (!validSelectionConfig(config)) {
    throw std::invalid_argument("Candidate configuration cannot produce the requested count");
  }
  if (!move_group.setJointValueTarget(target_radians)) {
    RCLCPP_ERROR(logger, "%s: MoveIt rejected the target", label.c_str());
    return std::nullopt;
  }

  std::vector<Candidate> candidates;
  candidates.reserve(config.candidate_count);
  for (int attempt = 1;
    attempt <= config.max_attempts &&
    static_cast<int>(candidates.size()) < config.candidate_count && rclcpp::ok(); ++attempt)
  {
    move_group.setStartState(from_state);
    Candidate candidate;
    candidate.attempt_number = attempt;
    const auto result = move_group.plan(candidate.plan);
    if (result != moveit::core::MoveItErrorCode::SUCCESS ||
      !evaluateTrajectory(candidate, from_state, joint_model_group, config))
    {
      RCLCPP_WARN(logger, "%s attempt %d rejected", label.c_str(), attempt);
      continue;
    }
    RCLCPP_INFO(logger,
      "%s candidate %zu/%d (attempt %d): time=%.3f s, travel=%.3f rad, "
      "min_sigma=%.6f, max_condition=%.3f, score=%.6f",
      label.c_str(), candidates.size() + 1, config.candidate_count, attempt,
      candidate.duration_seconds, candidate.joint_travel_radians,
      candidate.minimum_singular_value, candidate.maximum_condition_number, candidate.score);
    candidates.push_back(std::move(candidate));
  }

  if (static_cast<int>(candidates.size()) != config.candidate_count) {
    RCLCPP_ERROR(logger, "%s: only %zu/%d valid trajectories after %d attempts",
      label.c_str(), candidates.size(), config.candidate_count, config.max_attempts);
    return std::nullopt;
  }
  const auto best = std::max_element(
    candidates.begin(), candidates.end(),
    [](const Candidate & left, const Candidate & right) {return left.score < right.score;});
  RCLCPP_INFO(logger, "%s selected attempt %d with score %.6f",
    label.c_str(), best->attempt_number, best->score);
  return *best;
}
}  // namespace jaka_planner
