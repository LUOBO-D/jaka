#include "jaka_planner/trajectory_selector.hpp"

#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"

#include <array>
#include <algorithm>
#include <cmath>
#include <csignal>
#include <string>
#include <thread>
#include <vector>

namespace
{
constexpr std::array<double, 6> kStartDegrees = {
  -179.753, 90.057, -90.199, 90.196, 91.724, -64.680};
constexpr std::array<double, 6> kEndDegrees = {
  89.816, 109.950, -132.277, 201.880, 94.806, -74.132};

void sigintHandler(int)
{
  rclcpp::shutdown();
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::signal(SIGINT, sigintHandler);
  auto node = rclcpp::Node::make_shared("jaka_end_to_start_planner");

  const auto execute = node->declare_parameter<bool>("execute", false);
  const auto planning_time = node->declare_parameter<double>("planning_time", 5.0);
  const auto velocity_scaling = node->declare_parameter<double>("velocity_scaling", 0.05);
  const auto acceleration_scaling = node->declare_parameter<double>("acceleration_scaling", 0.05);
  const auto planner_id = node->declare_parameter<std::string>("planner_id", "");
  const auto tolerance = node->declare_parameter<double>("execution_start_tolerance_degrees", 0.5);
  const auto start_degrees = node->declare_parameter<std::vector<double>>(
    "start_degrees", std::vector<double>(kStartDegrees.begin(), kStartDegrees.end()));
  const auto end_degrees = node->declare_parameter<std::vector<double>>(
    "end_degrees", std::vector<double>(kEndDegrees.begin(), kEndDegrees.end()));

  jaka_planner::SelectionConfig config;
  config.candidate_count = static_cast<int>(node->declare_parameter<int>("candidate_count", 10));
  config.max_attempts = static_cast<int>(node->declare_parameter<int>("max_candidate_attempts", 30));
  config.time_weight = node->declare_parameter<double>("time_weight", 1.0);
  config.joint_travel_weight = node->declare_parameter<double>("joint_travel_weight", 1.0);
  config.singularity_weight = node->declare_parameter<double>("singularity_weight", 0.1);
  config.minimum_singular_value = node->declare_parameter<double>("minimum_singular_value", 0.01);
  config.maximum_condition_number =
    node->declare_parameter<double>("maximum_condition_number", 200.0);
  config.singularity_check_step =
    node->declare_parameter<double>("singularity_check_step_degrees", 1.0) *
    jaka_planner::kDegreesToRadians;

  if (start_degrees.size() != 6 || end_degrees.size() != 6 ||
    !jaka_planner::validSelectionConfig(config) ||
    planning_time <= 0.0 || velocity_scaling <= 0.0 || velocity_scaling > 1.0 ||
    acceleration_scaling <= 0.0 || acceleration_scaling > 1.0 || tolerance < 0.0)
  {
    RCLCPP_ERROR(node->get_logger(), "Invalid planning parameter");
    rclcpp::shutdown();
    return 2;
  }

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() {executor.spin();});
  int exit_code = 1;
  try {
    moveit::planning_interface::MoveGroupInterface move_group(node, "jaka_s5");
    move_group.setPlanningTime(planning_time);
    move_group.setNumPlanningAttempts(1);
    move_group.setMaxVelocityScalingFactor(velocity_scaling);
    move_group.setMaxAccelerationScalingFactor(acceleration_scaling);
    if (!planner_id.empty()) {
      move_group.setPlannerId(planner_id);
    }

    const auto current = move_group.getCurrentState(10.0);
    const auto start = jaka_planner::toRadians(start_degrees);
    const auto end = jaka_planner::toRadians(end_degrees);
    const auto * group = current ? current->getJointModelGroup("jaka_s5") : nullptr;
    if (!current || !group || group->getVariableCount() != 6) {
      RCLCPP_ERROR(node->get_logger(), "MoveIt did not provide a valid S5 state");
    } else {
      auto end_state = *current;
      end_state.setJointGroupPositions(group, end);
      end_state.update();
      const auto best = jaka_planner::selectBestTrajectory(
        move_group, end_state, group, start, config, "end_to_start", node->get_logger());
      if (best && !execute) {
        RCLCPP_INFO(node->get_logger(), "Plan-only mode; selected trajectory was not executed");
        exit_code = 0;
      } else if (best) {
        std::vector<double> actual;
        current->copyJointGroupPositions(group, actual);
        double maximum_error = 0.0;
        for (std::size_t joint = 0; joint < end.size(); ++joint) {
          maximum_error = std::max(maximum_error, std::abs(actual[joint] - end[joint]));
        }
        if (maximum_error / jaka_planner::kDegreesToRadians > tolerance) {
          RCLCPP_ERROR(node->get_logger(), "Execution refused because robot is not at end");
        } else if (move_group.execute(best->plan) == moveit::core::MoveItErrorCode::SUCCESS) {
          exit_code = 0;
        }
      }
    }
  } catch (const std::exception & error) {
    RCLCPP_ERROR(node->get_logger(), "Planning failed: %s", error.what());
  }

  rclcpp::shutdown();
  if (spinner.joinable()) {
    spinner.join();
  }
  return exit_code;
}
