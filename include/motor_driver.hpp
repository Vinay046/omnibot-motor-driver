#pragma once

#include <array>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include <chrono>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

// Omnibot messages
#include "omnibot_msgs/msg/wheel_vel_cmd.hpp"
#include "omnibot_msgs/msg/motor_state.hpp"
#include "omnibot_msgs/msg/motor_states.hpp"

// Omnibot services
#include "omnibot_msgs/srv/torque_enable.hpp"
#include "omnibot_msgs/srv/set_operating_mode.hpp"
#include "omnibot_msgs/srv/set_int16.hpp"

namespace omnibot::motor
{

class MotorDriver : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit MotorDriver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~MotorDriver() override = default;

protected:
  // Lifecycle transitions
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_error(const rclcpp_lifecycle::State & state) override;

private:
  // --- Callbacks (no hardware code here) ---
  void wheel_cmd_cb(const omnibot_msgs::msg::WheelVelCmd::ConstSharedPtr msg);
  void publish_states();

  // Service handlers (fill in with device logic later)
  void srv_torque_enable_cb(
    const std::shared_ptr<omnibot_msgs::srv::TorqueEnable::Request> request,
    std::shared_ptr<omnibot_msgs::srv::TorqueEnable::Response> response);

  void srv_set_mode_cb(
    const std::shared_ptr<omnibot_msgs::srv::SetOperatingMode::Request> request,
    std::shared_ptr<omnibot_msgs::srv::SetOperatingMode::Response> response);

  void srv_set_int16_cb(
    const std::shared_ptr<omnibot_msgs::srv::SetInt16::Request> request,
    std::shared_ptr<omnibot_msgs::srv::SetInt16::Response> response);

  // --- ROS interfaces ---
  rclcpp::Subscription<omnibot_msgs::msg::WheelVelCmd>::SharedPtr sub_wheel_cmd_;

  // Publish aggregated states for all motors
  rclcpp_lifecycle::LifecyclePublisher<omnibot_msgs::msg::MotorStates>::SharedPtr pub_motor_states_;

  // Optional: single-motor state publisher (handy for quick tests)
  rclcpp_lifecycle::LifecyclePublisher<omnibot_msgs::msg::MotorState>::SharedPtr pub_motor_state_single_;

  rclcpp::Service<omnibot_msgs::srv::TorqueEnable>::SharedPtr srv_torque_enable_;
  rclcpp::Service<omnibot_msgs::srv::SetOperatingMode>::SharedPtr srv_set_mode_;
  rclcpp::Service<omnibot_msgs::srv::SetInt16>::SharedPtr srv_set_int16_;

  rclcpp::TimerBase::SharedPtr timer_publish_;

  // --- Parameters / state (SDK-independent placeholders) ---
  int32_t num_motors_{3};
  double publish_rate_hz_{100.0};         // “near realtime” publish loop
  std::vector<std::string> motor_names_;  // ["motor_0","motor_1","motor_2"]

  // Last commanded wheel velocities (you’ll map these to your HW later)
  std::vector<double> last_cmd_vel_rps_;  // size == num_motors_

  // Simple placeholder state cache to publish without hardware
  omnibot_msgs::msg::MotorStates cached_states_;

  // QoS profiles
  rclcpp::QoS qos_cmd_{rclcpp::KeepLast(1)};     // low-latency commands
  rclcpp::QoS qos_state_{rclcpp::KeepLast(10)};  // state streaming

  // Helper to (re)build caches when num_motors_ changes
  void init_caches_();
};

}  // namespace omnibot::motor
