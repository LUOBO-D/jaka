#pragma once

#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/robot_state/robot_state.h"
#include "rclcpp/logger.hpp"

#include <optional>
#include <string>
#include <vector>

namespace jaka_planner
{
constexpr double kDegreesToRadians = 3.14159265358979323846 / 180.0;

struct SelectionConfig
{
  int candidate_count{10};
  int max_attempts{30};
  double time_weight{1.0};
  double joint_travel_weight{1.0};
  double singularity_weight{0.1};
  double minimum_singular_value{0.01};
  double maximum_condition_number{200.0};
  double singularity_check_step{1.0 * kDegreesToRadians};
};

struct Candidate
{
  int attempt_number{0};
  double duration_seconds{0.0};
  double joint_travel_radians{0.0};
  double minimum_singular_value{0.0};
  double maximum_condition_number{0.0};
  double score{0.0};
  moveit::planning_interface::MoveGroupInterface::Plan plan;
};

std::vector<double> toRadians(const std::vector<double> & degrees);
double computeJointTravel(const std::vector<std::vector<double>> & positions);
double computeScore(
  double duration, double joint_travel, double minimum_singular_value,
  const SelectionConfig & config);
int interpolationSteps(const std::vector<double> & from, const std::vector<double> & to, double max_step);
bool passesSingularityLimits(double minimum_singular_value, double maximum_condition_number,
  const SelectionConfig & config);
bool validSelectionConfig(const SelectionConfig & config);

std::optional<Candidate> selectBestTrajectory(
  moveit::planning_interface::MoveGroupInterface & move_group,
  const moveit::core::RobotState & from_state,
  const moveit::core::JointModelGroup * joint_model_group,
  const std::vector<double> & target_radians,
  const SelectionConfig & config,
  const std::string & label,
  const rclcpp::Logger & logger);
}  // namespace jaka_planner
