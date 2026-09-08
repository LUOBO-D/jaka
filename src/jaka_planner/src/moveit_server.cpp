#include "jaka_planner/JAKAZuRobot.h"
#include "jaka_planner/jktypes.h"
#include "jaka_planner/servo_stream.hpp"

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace
{
using FollowTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandle = rclcpp_action::ServerGoalHandle<FollowTrajectory>;

JAKAZuRobot robot;
std::mutex sdk_mutex;
std::atomic_bool robot_ready{false};
std::atomic_bool goal_active{false};
std::thread goal_thread;
constexpr double kRadiansToDegrees = 180.0 / 3.14159265358979323846;
constexpr std::array<const char *, 6> kJointNames = {
  "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

double seconds(const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<double>(duration.sec) + static_cast<double>(duration.nanosec) * 1e-9;
}

std::shared_ptr<FollowTrajectory::Result> result(int code, const std::string & message)
{
  auto value = std::make_shared<FollowTrajectory::Result>();
  value->error_code = code;
  value->error_string = message;
  return value;
}

bool readRobotStatus(RobotStatus & status)
{
  std::lock_guard<std::mutex> lock(sdk_mutex);
  if (robot.get_robot_status(&status) != 0) {
    return false;
  }
  return status.is_socket_connect == 1 && status.powered_on == 1 && status.enabled == 1 &&
         status.errcode == 0 && status.emergency_stop == 0 && status.protective_stop == 0;
}

bool waitForStatus(bool require_power, bool require_enabled, std::chrono::seconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    RobotStatus status{};
    {
      std::lock_guard<std::mutex> lock(sdk_mutex);
      if (robot.get_robot_status(&status) == 0 && status.is_socket_connect == 1 &&
        status.emergency_stop == 0 && status.protective_stop == 0 && status.errcode == 0 &&
        (!require_power || status.powered_on == 1) && (!require_enabled || status.enabled == 1))
      {
        return true;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
  return false;
}

bool validateTrajectory(const trajectory_msgs::msg::JointTrajectory & trajectory, std::string & error)
{
  if (trajectory.joint_names.size() != kJointNames.size()) {
    error = "trajectory must contain exactly six joint names";
    return false;
  }
  for (std::size_t index = 0; index < kJointNames.size(); ++index) {
    if (trajectory.joint_names[index] != kJointNames[index]) {
      error = "trajectory joint names must be ordered joint_1 through joint_6";
      return false;
    }
  }
  if (trajectory.points.size() < 2) {
    error = "trajectory must contain at least two points";
    return false;
  }
  double previous_time = -1.0;
  for (const auto & point : trajectory.points) {
    if (point.positions.size() != kJointNames.size()) {
      error = "every trajectory point must contain six positions";
      return false;
    }
    const double point_time = seconds(point.time_from_start);
    if (!std::isfinite(point_time) || point_time < 0.0 || point_time <= previous_time) {
      error = "trajectory time_from_start values must be finite and strictly increasing";
      return false;
    }
    if (!std::all_of(point.positions.begin(), point.positions.end(),
      [](double value) {return std::isfinite(value);}))
    {
      error = "trajectory contains a non-finite position";
      return false;
    }
    previous_time = point_time;
  }
  return true;
}

void stopServoMotion()
{
  std::lock_guard<std::mutex> lock(sdk_mutex);
  robot.motion_abort();
  robot.servo_move_enable(false);
}

bool targetReached(const JointValue & target, double tolerance_degrees)
{
  JointValue actual{};
  std::lock_guard<std::mutex> lock(sdk_mutex);
  if (robot.get_joint_position(&actual) != 0) {
    return false;
  }
  for (std::size_t joint = 0; joint < kJointNames.size(); ++joint) {
    if (std::abs(actual.jVal[joint] - target.jVal[joint]) * kRadiansToDegrees > tolerance_degrees) {
      return false;
    }
  }
  return true;
}

void executeGoal(
  const std::shared_ptr<GoalHandle> goal_handle, double arrival_timeout_margin,
  double maximum_send_lateness)
{
  const auto clear_active = []() {goal_active.store(false);};
  const auto & trajectory = goal_handle->get_goal()->trajectory;
  jaka_planner::ServoStreamConfig stream_config;
  std::vector<jaka_planner::ServoSample> samples;
  std::string stream_error;
  if (!jaka_planner::resampleTrajectory(trajectory, stream_config, samples, stream_error)) {
    goal_handle->abort(result(FollowTrajectory::Result::INVALID_GOAL, stream_error));
    clear_active();
    return;
  }

  JointValue actual{};
  {
    std::lock_guard<std::mutex> lock(sdk_mutex);
    if (robot.get_joint_position(&actual) != 0) {
      goal_handle->abort(result(FollowTrajectory::Result::INVALID_GOAL, "failed to read initial joints"));
      clear_active();
      return;
    }
  }
  jaka_planner::ServoSample actual_sample{};
  for (std::size_t joint = 0; joint < kJointNames.size(); ++joint) {
    actual_sample[joint] = actual.jVal[joint];
  }
  if (!jaka_planner::validateCommandVelocity(
      actual_sample, samples.front(), stream_config, stream_error))
  {
    goal_handle->abort(result(FollowTrajectory::Result::INVALID_GOAL, "unsafe initial command: " + stream_error));
    clear_active();
    return;
  }

  {
    std::lock_guard<std::mutex> lock(sdk_mutex);
    const int sdk_result = robot.servo_move_enable(true);
    if (sdk_result != 0) {
      goal_handle->abort(result(FollowTrajectory::Result::INVALID_GOAL, "failed to enable servo mode"));
      clear_active();
      return;
    }
  }

  JointValue target{};
  const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(stream_config.period_seconds));
  const auto allowed_lateness = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(maximum_send_lateness));
  auto send_deadline = std::chrono::steady_clock::now();
  for (std::size_t sample_index = 0; sample_index < samples.size(); ++sample_index) {
    if (sample_index > 0) {
      send_deadline += period;
      std::this_thread::sleep_until(send_deadline);
    }
    if (!rclcpp::ok()) {
      stopServoMotion();
      clear_active();
      return;
    }
    if (goal_handle->is_canceling()) {
      stopServoMotion();
      goal_handle->canceled(result(FollowTrajectory::Result::SUCCESSFUL, "trajectory canceled"));
      clear_active();
      return;
    }
    if (std::chrono::steady_clock::now() > send_deadline + allowed_lateness) {
      stopServoMotion();
      goal_handle->abort(result(
        FollowTrajectory::Result::PATH_TOLERANCE_VIOLATED, "8 ms servo send deadline missed"));
      clear_active();
      return;
    }

    for (std::size_t joint = 0; joint < kJointNames.size(); ++joint) {
      target.jVal[joint] = samples[sample_index][joint];
    }
    int sdk_result = 0;
    {
      std::lock_guard<std::mutex> lock(sdk_mutex);
      sdk_result = robot.servo_j(&target, MoveMode::ABS, 1);
    }
    if (sdk_result != 0) {
      stopServoMotion();
      goal_handle->abort(result(FollowTrajectory::Result::PATH_TOLERANCE_VIOLATED, "servo_j failed"));
      clear_active();
      return;
    }
    if (sample_index % 12 == 0) {
      RobotStatus status{};
      if (!readRobotStatus(status)) {
        robot_ready.store(false);
        stopServoMotion();
        goal_handle->abort(result(
          FollowTrajectory::Result::PATH_TOLERANCE_VIOLATED,
          "robot fault during servo streaming"));
        clear_active();
        return;
      }
    }
  }

  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration<double>(arrival_timeout_margin);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    if (goal_handle->is_canceling()) {
      stopServoMotion();
      goal_handle->canceled(result(FollowTrajectory::Result::SUCCESSFUL, "trajectory canceled"));
      clear_active();
      return;
    }
    RobotStatus status{};
    if (!readRobotStatus(status)) {
      robot_ready.store(false);
      stopServoMotion();
      goal_handle->abort(result(FollowTrajectory::Result::PATH_TOLERANCE_VIOLATED, "robot fault while waiting for target"));
      clear_active();
      return;
    }
    if (targetReached(target, 0.2)) {
      std::lock_guard<std::mutex> lock(sdk_mutex);
      robot.servo_move_enable(false);
      goal_handle->succeed(result(FollowTrajectory::Result::SUCCESSFUL, "target reached"));
      clear_active();
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  stopServoMotion();
  goal_handle->abort(result(FollowTrajectory::Result::GOAL_TOLERANCE_VIOLATED, "target arrival timeout"));
  clear_active();
}

void sigintHandler(int)
{
  rclcpp::shutdown();
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::signal(SIGINT, sigintHandler);
  auto node = rclcpp::Node::make_shared("moveit_server");
  const auto robot_ip = node->declare_parameter<std::string>("ip", "10.5.5.100");
  const auto robot_model = node->declare_parameter<std::string>("model", "s5");
  const auto arrival_timeout_margin = node->declare_parameter<double>("arrival_timeout_margin", 10.0);
  const auto maximum_send_lateness = node->declare_parameter<double>("maximum_send_lateness", 0.008);
  if (robot_model != "s5" || arrival_timeout_margin <= 0.0 || maximum_send_lateness <= 0.0) {
    RCLCPP_ERROR(node->get_logger(), "This hardened server requires model:=s5 and a positive timeout margin");
    rclcpp::shutdown();
    return 2;
  }

  {
    std::lock_guard<std::mutex> lock(sdk_mutex);
    if (robot.login_in(robot_ip.c_str(), false) != 0) {
      RCLCPP_ERROR(node->get_logger(), "Failed to connect to S5 at %s", robot_ip.c_str());
      rclcpp::shutdown();
      return 1;
    }
    robot.servo_move_enable(false);
    robot.servo_move_use_joint_LPF(0.5);
    if (robot.power_on() != 0) {
      RCLCPP_ERROR(node->get_logger(), "S5 power-on command failed");
      robot.login_out();
      rclcpp::shutdown();
      return 1;
    }
  }
  if (!waitForStatus(true, false, std::chrono::seconds(15))) {
    RCLCPP_ERROR(node->get_logger(), "S5 did not reach powered-on state");
    {
      std::lock_guard<std::mutex> lock(sdk_mutex);
      robot.login_out();
    }
    rclcpp::shutdown();
    return 1;
  }
  {
    std::lock_guard<std::mutex> lock(sdk_mutex);
    if (robot.enable_robot() != 0) {
      RCLCPP_ERROR(node->get_logger(), "S5 enable command failed");
      robot.login_out();
      rclcpp::shutdown();
      return 1;
    }
  }
  if (!waitForStatus(true, true, std::chrono::seconds(10))) {
    RCLCPP_ERROR(node->get_logger(), "S5 did not reach enabled state");
    {
      std::lock_guard<std::mutex> lock(sdk_mutex);
      robot.login_out();
    }
    rclcpp::shutdown();
    return 1;
  }
  robot_ready.store(true);

  const auto joint_publisher = node->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
  rclcpp::QoS ready_qos(1);
  ready_qos.reliable().transient_local();
  const auto ready_publisher = node->create_publisher<std_msgs::msg::Bool>("/moveit_server/ready", ready_qos);

  const auto action_server = rclcpp_action::create_server<FollowTrajectory>(
    node, "/jaka_s5_controller/follow_joint_trajectory",
    [](const rclcpp_action::GoalUUID &, const std::shared_ptr<const FollowTrajectory::Goal> goal) {
      std::string error;
      if (!robot_ready.load() || goal_active.load() || !validateTrajectory(goal->trajectory, error)) {
        RCLCPP_ERROR(rclcpp::get_logger("moveit_server"), "Goal rejected: %s",
          error.empty() ? "robot not ready or another goal is active" : error.c_str());
        return rclcpp_action::GoalResponse::REJECT;
      }
      goal_active.store(true);
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [](const std::shared_ptr<GoalHandle>) {
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [arrival_timeout_margin, maximum_send_lateness](const std::shared_ptr<GoalHandle> goal_handle) {
      if (goal_thread.joinable()) {
        goal_thread.join();
      }
      goal_thread = std::thread(
        executeGoal, goal_handle, arrival_timeout_margin, maximum_send_lateness);
    });
  (void)action_server;
  RCLCPP_INFO(node->get_logger(), "S5 connected, powered, enabled, and ready");

  rclcpp::Rate rate(125.0);
  int status_counter = 0;
  while (rclcpp::ok()) {
    if (!goal_active.load()) {
      JointValue joints{};
      bool joint_state_ok = false;
      {
        std::lock_guard<std::mutex> lock(sdk_mutex);
        joint_state_ok = robot.get_joint_position(&joints) == 0;
      }
      if (joint_state_ok) {
        sensor_msgs::msg::JointState message;
        message.header.stamp = node->now();
        for (std::size_t joint = 0; joint < kJointNames.size(); ++joint) {
          message.name.emplace_back(kJointNames[joint]);
          message.position.push_back(joints.jVal[joint]);
        }
        joint_publisher->publish(message);
      }
      if (++status_counter >= 12) {
        RobotStatus status{};
        robot_ready.store(readRobotStatus(status));
        status_counter = 0;
      }
    }
    std_msgs::msg::Bool ready_message;
    ready_message.data = robot_ready.load();
    ready_publisher->publish(ready_message);
    rclcpp::spin_some(node);
    rate.sleep();
  }

  robot_ready.store(false);
  stopServoMotion();
  if (goal_thread.joinable()) {
    goal_thread.join();
  }
  {
    std::lock_guard<std::mutex> lock(sdk_mutex);
    robot.login_out();
  }
  rclcpp::shutdown();
  return 0;
}
