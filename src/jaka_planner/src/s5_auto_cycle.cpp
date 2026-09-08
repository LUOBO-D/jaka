#include "jaka_planner/trajectory_selector.hpp"

#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
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

std::int64_t steadyNowNanoseconds()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

bool serverHealthy(const std::atomic_bool & ready, const std::atomic<std::int64_t> & last_update)
{
  constexpr std::int64_t kHeartbeatTimeoutNanoseconds = 1000000000;
  const auto update = last_update.load();
  return ready.load() && update > 0 &&
         steadyNowNanoseconds() - update <= kHeartbeatTimeoutNanoseconds;
}

bool waitForReady(
  const std::atomic_bool & ready, const std::atomic<std::int64_t> & last_update,
  double timeout_seconds, const rclcpp::Logger & logger)
{
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration<double>(timeout_seconds);
  while (rclcpp::ok() && !serverHealthy(ready, last_update) &&
    std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  if (!serverHealthy(ready, last_update)) {
    RCLCPP_ERROR(logger, "moveit_server did not become ready within %.1f seconds", timeout_seconds);
    return false;
  }
  return true;
}

bool stateNearTarget(
  moveit::planning_interface::MoveGroupInterface & move_group,
  const moveit::core::JointModelGroup * joint_model_group,
  const std::vector<double> & target, double tolerance_degrees,
  const std::string & label, const rclcpp::Logger & logger)
{
  const auto state = move_group.getCurrentState(2.0);
  if (!state) {
    RCLCPP_ERROR(logger, "%s: current robot state is unavailable", label.c_str());
    return false;
  }
  std::vector<double> actual;
  state->copyJointGroupPositions(joint_model_group, actual);
  if (actual.size() != target.size()) {
    RCLCPP_ERROR(logger, "%s: current state does not contain six joints", label.c_str());
    return false;
  }
  double maximum_error = 0.0;
  for (std::size_t joint = 0; joint < target.size(); ++joint) {
    maximum_error = std::max(maximum_error, std::abs(actual[joint] - target[joint]));
  }
  const double maximum_error_degrees = maximum_error / jaka_planner::kDegreesToRadians;
  if (maximum_error_degrees > tolerance_degrees) {
    RCLCPP_ERROR(logger, "%s: maximum joint error %.3f deg exceeds %.3f deg",
      label.c_str(), maximum_error_degrees, tolerance_degrees);
    return false;
  }
  return true;
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::signal(SIGINT, sigintHandler);
  auto node = rclcpp::Node::make_shared("s5_auto_cycle");

  const auto candidate_count = node->declare_parameter<int>("candidate_count", 10);
  const auto max_attempts = node->declare_parameter<int>("max_candidate_attempts", 30);
  const auto dwell_seconds = node->declare_parameter<double>("dwell_seconds", 5.0);
  const auto planning_time = node->declare_parameter<double>("planning_time", 5.0);
  const auto velocity_scaling = node->declare_parameter<double>("velocity_scaling", 0.02);
  const auto acceleration_scaling = node->declare_parameter<double>("acceleration_scaling", 0.02);
  const auto ready_timeout = node->declare_parameter<double>("ready_timeout", 60.0);
  const auto position_tolerance = node->declare_parameter<double>("position_tolerance_degrees", 0.5);
  const auto planner_id = node->declare_parameter<std::string>("planner_id", "");
  const auto start_degrees = node->declare_parameter<std::vector<double>>(
    "start_degrees", std::vector<double>(kStartDegrees.begin(), kStartDegrees.end()));
  const auto end_degrees = node->declare_parameter<std::vector<double>>(
    "end_degrees", std::vector<double>(kEndDegrees.begin(), kEndDegrees.end()));

  jaka_planner::SelectionConfig selection;
  selection.candidate_count = static_cast<int>(candidate_count);
  selection.max_attempts = static_cast<int>(max_attempts);
  selection.time_weight = node->declare_parameter<double>("time_weight", 1.0);
  selection.joint_travel_weight = node->declare_parameter<double>("joint_travel_weight", 1.0);
  selection.singularity_weight = node->declare_parameter<double>("singularity_weight", 0.1);
  selection.minimum_singular_value = node->declare_parameter<double>("minimum_singular_value", 0.01);
  selection.maximum_condition_number =
    node->declare_parameter<double>("maximum_condition_number", 200.0);
  selection.singularity_check_step =
    node->declare_parameter<double>("singularity_check_step_degrees", 1.0) *
    jaka_planner::kDegreesToRadians;

  if (start_degrees.size() != 6 || end_degrees.size() != 6 ||
    !jaka_planner::validSelectionConfig(selection) ||
    dwell_seconds < 0.0 || planning_time <= 0.0 || ready_timeout <= 0.0 ||
    position_tolerance < 0.0 || velocity_scaling <= 0.0 || velocity_scaling > 1.0 ||
    acceleration_scaling <= 0.0 || acceleration_scaling > 1.0)
  {
    RCLCPP_ERROR(node->get_logger(), "Invalid mission parameter");
    rclcpp::shutdown();
    return 2;
  }

  std::atomic_bool server_ready{false};
  std::atomic<std::int64_t> ready_last_update{0};
  rclcpp::QoS ready_qos(1);
  ready_qos.reliable().transient_local();
  const auto ready_subscription = node->create_subscription<std_msgs::msg::Bool>(
    "/moveit_server/ready", ready_qos,
    [&server_ready, &ready_last_update](const std_msgs::msg::Bool::SharedPtr message) {
      server_ready.store(message->data);
      ready_last_update.store(steadyNowNanoseconds());
    });
  (void)ready_subscription;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() {executor.spin();});
  int exit_code = 1;

  try {
    if (waitForReady(server_ready, ready_last_update, ready_timeout, node->get_logger())) {
      moveit::planning_interface::MoveGroupInterface move_group(node, "jaka_s5");
      move_group.setPlanningTime(planning_time);
      move_group.setNumPlanningAttempts(1);
      move_group.setMaxVelocityScalingFactor(velocity_scaling);
      move_group.setMaxAccelerationScalingFactor(acceleration_scaling);
      if (!planner_id.empty()) {
        move_group.setPlannerId(planner_id);
      }

      const auto current_state = move_group.getCurrentState(10.0);
      const auto start = jaka_planner::toRadians(start_degrees);
      const auto end = jaka_planner::toRadians(end_degrees);
      const auto * group = current_state ? current_state->getJointModelGroup("jaka_s5") : nullptr;
      if (!current_state || !group || group->getVariableCount() != 6) {
        RCLCPP_ERROR(node->get_logger(), "MoveIt did not provide a valid S5 state");
      } else if (!stateNearTarget(
          move_group, group, end, position_tolerance, "Initial end check", node->get_logger()))
      {
        RCLCPP_ERROR(node->get_logger(), "Mission aborted before planning: robot must be at end");
      } else {
        auto end_state = *current_state;
        end_state.setJointGroupPositions(group, end);
        end_state.update();
        auto start_state = *current_state;
        start_state.setJointGroupPositions(group, start);
        start_state.update();
        if (!end_state.satisfiesBounds(group) || !start_state.satisfiesBounds(group)) {
          RCLCPP_ERROR(node->get_logger(), "Configured start or end violates S5 joint bounds");
        } else {
          const auto outbound = jaka_planner::selectBestTrajectory(
            move_group, end_state, group, start, selection, "end_to_start", node->get_logger());
          const auto inbound = outbound ? jaka_planner::selectBestTrajectory(
            move_group, start_state, group, end, selection, "start_to_end", node->get_logger()) :
            std::nullopt;

          if (!outbound || !inbound) {
            RCLCPP_ERROR(node->get_logger(), "Mission aborted before motion: both legs need 10 valid candidates");
          } else if (!serverHealthy(server_ready, ready_last_update) || !stateNearTarget(
              move_group, group, end, position_tolerance, "Pre-execution end check", node->get_logger()))
          {
            RCLCPP_ERROR(node->get_logger(), "Mission aborted before motion: server or end state changed");
          } else if (move_group.execute(outbound->plan) != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(node->get_logger(), "end_to_start execution failed");
          } else if (!stateNearTarget(
              move_group, group, start, position_tolerance, "Start arrival check", node->get_logger()))
          {
            RCLCPP_ERROR(node->get_logger(), "Robot did not reach start; return motion suppressed");
          } else {
            RCLCPP_INFO(node->get_logger(), "At start; dwelling for %.3f seconds", dwell_seconds);
            const auto dwell_deadline = std::chrono::steady_clock::now() +
              std::chrono::duration<double>(dwell_seconds);
            while (rclcpp::ok() && serverHealthy(server_ready, ready_last_update) &&
              std::chrono::steady_clock::now() < dwell_deadline)
            {
              std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
            if (!rclcpp::ok() || !serverHealthy(server_ready, ready_last_update)) {
              RCLCPP_ERROR(node->get_logger(), "Server fault during dwell; return motion suppressed");
            } else if (!stateNearTarget(
                move_group, group, start, position_tolerance, "Pre-return start check", node->get_logger()))
            {
              RCLCPP_ERROR(node->get_logger(), "Robot moved during dwell; return motion suppressed");
            } else if (move_group.execute(inbound->plan) != moveit::core::MoveItErrorCode::SUCCESS) {
              RCLCPP_ERROR(node->get_logger(), "start_to_end execution failed");
            } else if (!stateNearTarget(
                move_group, group, end, position_tolerance, "Final end check", node->get_logger()))
            {
              RCLCPP_ERROR(node->get_logger(), "Robot did not finish within the end tolerance");
            } else {
              RCLCPP_INFO(node->get_logger(), "S5 automatic cycle completed; robot remains enabled");
              exit_code = 0;
            }
          }
        }
      }
    }
  } catch (const std::exception & error) {
    RCLCPP_ERROR(node->get_logger(), "Automatic cycle failed: %s", error.what());
  }

  rclcpp::shutdown();
  if (spinner.joinable()) {
    spinner.join();
  }
  return exit_code;
}
