#include "motor_driver.hpp"

int main ()

{

  std::cout << "Hello World!" << std::endl;
  return 0;
}#include "motor_driver.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace omnibot::motor
{

MotorDriver::MotorDriver(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("motor_driver", options)
{
  // --- Declare parameters (editable at runtime if needed) ---
  this->declare_parameter<int>("num_motors", num_motors_);
  this->declare_parameter<double>("publish_rate_hz", publish_rate_hz_);
  this->declare_parameter<std::vector<std::string>>("motor_names", std::vector<std::string>{});
  // You can declare serial ports, IDs, etc., later when you add SDK code.

  // Set QoS for low latency
  qos_cmd_.best_effort().durability_volatile();
  qos_state_.reliable().durability_volatile();

  RCLCPP_INFO(get_logger(), "MotorDriver constructed (lifecycle).");
}

CallbackReturn MotorDriver::on_configure(const rclcpp_lifecycle::State &)
{
  // Read params
  num_motors_ = this->get_parameter("num_motors").as_int();
  publish_rate_hz_ = this->get_parameter("publish_rate_hz").as_double();
  motor_names_ = this->get_parameter("motor_names").as_string_array();

  if (num_motors_ <= 0) {
    RCLCPP_ERROR(get_logger(), "num_motors must be > 0");
    return CallbackReturn::FAILURE;
  }

  if (motor_names_.empty()) {
    motor_names_.resize(static_cast<size_t>(num_motors_));
    for (int i = 0; i < num_motors_; ++i) {
      motor_names_[static_cast<size_t>(i)] = "motor_" + std::to_string(i);
    }
  }

  init_caches_();

  // --- Subscribers ---
  sub_wheel_cmd_ = this->create_subscription<omnibot_msgs::msg::WheelVelCmd>(
    "~/wheel_vel_cmd", qos_cmd_,
    std::bind(&MotorDriver::wheel_cmd_cb, this, std::placeholders::_1));

  // --- Lifecycle Publishers ---
  pub_motor_states_ = this->create_publisher<omnibot_msgs::msg::MotorStates>(
    "~/motor_states", qos_state_);

  pub_motor_state_single_ = this->create_publisher<omnibot_msgs::msg::MotorState>(
    "~/motor_state", qos_state_);

  // --- Services (servers) ---
  srv_torque_enable_ = this->create_service<omnibot_msgs::srv::TorqueEnable>(
    "~/torque_enable",
    std::bind(&MotorDriver::srv_torque_enable_cb, this,
              std::placeholders::_1, std::placeholders::_2));

  srv_set_mode_ = this->create_service<omnibot_msgs::srv::SetOperatingMode>(
    "~/set_operating_mode",
    std::bind(&MotorDriver::srv_set_mode_cb, this,
              std::placeholders::_1, std::placeholders::_2));

  srv_set_int16_ = this->create_service<omnibot_msgs::srv::SetInt16>(
    "~/set_int16",
    std::bind(&MotorDriver::srv_set_int16_cb, this,
              std::placeholders::_1, std::placeholders::_2));

  // --- Timer (created now; enabled when activated) ---
  if (publish_rate_hz_ <= 0.0) {
    publish_rate_hz_ = 100.0;
  }
  auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
  timer_publish_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(period),
    std::bind(&MotorDriver::publish_states, this));
  timer_publish_->cancel();  // enable on activate

  RCLCPP_INFO(get_logger(), "Configured with %d motors, publish @ %.1f Hz.",
              num_motors_, publish_rate_hz_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn MotorDriver::on_activate(const rclcpp_lifecycle::State &)
{
  pub_motor_states_->on_activate();
  pub_motor_state_single_->on_activate();

  // Start periodic publishing
  if (timer_publish_) {
    timer_publish_->reset();
  }

  RCLCPP_INFO(get_logger(), "Activated.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn MotorDriver::on_deactivate(const rclcpp_lifecycle::State &)
{
  // Stop periodic publishing
  if (timer_publish_) {
    timer_publish_->cancel();
  }

  pub_motor_states_->on_deactivate();
  pub_motor_state_single_->on_deactivate();

  RCLCPP_INFO(get_logger(), "Deactivated.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn MotorDriver::on_cleanup(const rclcpp_lifecycle::State &)
{
  // Release interfaces
  timer_publish_.reset();
  sub_wheel_cmd_.reset();
  pub_motor_states_.reset();
  pub_motor_state_single_.reset();
  srv_torque_enable_.reset();
  srv_set_mode_.reset();
  srv_set_int16_.reset();

  last_cmd_vel_rps_.clear();
  cached_states_ = omnibot_msgs::msg::MotorStates();

  RCLCPP_INFO(get_logger(), "Cleaned up.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn MotorDriver::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutdown.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn MotorDriver::on_error(const rclcpp_lifecycle::State &)
{
  RCLCPP_ERROR(get_logger(), "Error transition entered.");
  return CallbackReturn::SUCCESS;
}

// -------------------- Callbacks --------------------

void MotorDriver::wheel_cmd_cb(const omnibot_msgs::msg::WheelVelCmd::ConstSharedPtr msg)
{
  if (!msg) return;

  // WheelVelCmd.msg has: builtin_interfaces/Time stamp, float64[3] velocity_rad_s
  const auto & v = msg->velocity_rad_s;  // fixed-size array< double, 3 >
  const size_t n = std::min(static_cast<size_t>(num_motors_), v.size());
  for (size_t i = 0; i < n; ++i) {
    last_cmd_vel_rps_[i] = v[i];
  }
}

void MotorDriver::publish_states()
{
  if (!pub_motor_states_->is_activated()) {
    return;
  }

  // Ensure size and fill placeholder state
  if (static_cast<int>(cached_states_.motors.size()) != num_motors_) {
    cached_states_.motors.resize(static_cast<size_t>(num_motors_));
  }

  for (int i = 0; i < num_motors_; ++i) {
    auto & s = cached_states_.motors[static_cast<size_t>(i)];
    s.id = static_cast<uint8_t>(i);
    s.velocity_rad_s = last_cmd_vel_rps_[static_cast<size_t>(i)];  // reflect command as a stub
    // leave other fields default-initialized for now
  }

  pub_motor_states_->publish(cached_states_);

  // Optionally publish the first motor as single
  if (!cached_states_.motors.empty() && pub_motor_state_single_->is_activated()) {
    pub_motor_state_single_->publish(cached_states_.motors.front());
  }
}

// -------------------- Services --------------------

void MotorDriver::srv_torque_enable_cb(
  const std::shared_ptr<omnibot_msgs::srv::TorqueEnable::Request> request,
  std::shared_ptr<omnibot_msgs::srv::TorqueEnable::Response> response)
{
  (void)request;  // TODO: implement hardware torque enable/disable
  response->success = true;
  response->message = "Torque request accepted (stub).";
}

void MotorDriver::srv_set_mode_cb(
  const std::shared_ptr<omnibot_msgs::srv::SetOperatingMode::Request> request,
  std::shared_ptr<omnibot_msgs::srv::SetOperatingMode::Response> response)
{
  (void)request;  // TODO: implement operating mode change
  response->success = true;
  response->message = "Operating mode set (stub).";
}

void MotorDriver::srv_set_int16_cb(
  const std::shared_ptr<omnibot_msgs::srv::SetInt16::Request> request,
  std::shared_ptr<omnibot_msgs::srv::SetInt16::Response> response)
{
  (void)request;  // TODO: implement simple int16 setter (e.g., accel, PWM limit)
  response->success = true;
  response->message = "SetInt16 applied (stub).";
}

// -------------------- Helpers --------------------

void MotorDriver::init_caches_()
{
  last_cmd_vel_rps_.assign(static_cast<size_t>(num_motors_), 0.0);
  cached_states_.motors.clear();
  cached_states_.motors.resize(static_cast<size_t>(num_motors_));
}

}  // namespace omnibot::motor

// -------------- main() (optional) --------------
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<omnibot::motor::MotorDriver>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
