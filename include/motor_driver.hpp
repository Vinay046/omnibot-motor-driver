#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>

#include <omnibot_msgs/msg/wheel_vel_cmd.hpp>
#include <omnibot_msgs/msg/motor_state.hpp>
#include <omnibot_msgs/msg/motor_states.hpp>
#include <omnibot_msgs/srv/torque_enable.hpp>
#include <omnibot_msgs/srv/set_operating_mode.hpp>
#include <omnibot_msgs/srv/set_int16.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

namespace omnibot_motor_driver {

/**
 * @brief Low-jitter backend for DYNAMIXEL XM430-W210 (Protocol 2.0).
 * Talks over a serial adapter (e.g., U2D2) and supports:
 *  - Velocity control: sync write to Goal Velocity
 *  - Readback: current, velocity, position, voltage, temperature
 *  - Services: torque enable, operating mode, goal current
 */
class HardwareBackend {
public:
  struct MotorIO {
    uint8_t id{0};
    // commanded:
    double vel_cmd_rad_s{0.0};
    // measured:
    double position_rad{0.0};
    double velocity_rad_s{0.0};
    double current_a{0.0};
    double voltage_v{0.0};
    double temperature_c{0.0};
    uint8_t hw_error{0};
    bool moving{false};
  };

  HardwareBackend() = default;
  ~HardwareBackend();

  bool configure(const std::string& device, int baudrate,
                 const std::vector<uint8_t>& motor_ids);

  bool start(double loop_hz);
  void stop();

  // Called by the control thread every period
  bool step(std::chrono::steady_clock::time_point now);

  // Thread-safe setter
  void set_velocity_commands(const std::vector<double>& vels);

  // Services mapping
  bool torque_enable(uint8_t id, bool enable, std::string& msg);
  bool set_operating_mode(uint8_t id, uint8_t mode, std::string& msg);
  bool set_int16(uint8_t id, int16_t value, std::string& msg); // Goal Current

  // Copy of last readback for publisher
  std::vector<MotorIO> readback();

private:
  // ======= Constants (XM430-W210; Protocol 2.0) =======
  static constexpr uint16_t ADDR_OPERATING_MODE   = 11;   // 1B
  static constexpr uint16_t ADDR_TORQUE_ENABLE    = 64;   // 1B
  static constexpr uint16_t ADDR_GOAL_CURRENT     = 102;  // 2B
  static constexpr uint16_t ADDR_GOAL_VELOCITY    = 104;  // 4B
  static constexpr uint16_t ADDR_PRESENT_CURRENT  = 126;  // 2B
  static constexpr uint16_t ADDR_PRESENT_VELOCITY = 128;  // 4B
  static constexpr uint16_t ADDR_PRESENT_POSITION = 132;  // 4B
  static constexpr uint16_t ADDR_PRESENT_VOLTAGE  = 144;  // 2B
  static constexpr uint16_t ADDR_PRESENT_TEMP     = 146;  // 1B

  // Units
  static constexpr double VELOCITY_UNIT_RPM_PER_LSB = 0.229;    // rpm / LSB
  static constexpr double CURRENT_UNIT_AMP_PER_LSB  = 0.00269;  // A / LSB
  static constexpr double VOLT_UNIT_V_PER_LSB       = 0.1;      // V / LSB
  static constexpr double TICKS_PER_REV             = 4096.0;

  // RT niceness
  bool rt_raise_priority();

  // Serial
  bool hw_open(const std::string& device, int baudrate);
  void hw_close();
  bool write_bytes(const uint8_t* data, size_t n);
  bool read_bytes(uint8_t* out, size_t n, int timeout_us = 2000);

  // Protocol helpers
  static uint16_t compute_crc(const std::vector<uint8_t>& pkt);
  bool txrx(uint8_t id, uint8_t inst, const std::vector<uint8_t>& params,
            std::vector<uint8_t>* status_params, int expect_len);
  bool dxl_write1(uint8_t id, uint16_t addr, uint8_t val);
  bool dxl_write2(uint8_t id, uint16_t addr, uint16_t val);
  bool dxl_write4(uint8_t id, uint16_t addr, int32_t val);
  bool dxl_read(uint8_t id, uint16_t addr, uint16_t len, std::vector<uint8_t>& out);

  bool dxl_sync_write_goal_velocity(const std::vector<double>& rad_s);
  static int32_t rad_s_to_dxl(double rad_s);
  static double dxl_to_rad_s(int32_t vel_units);

  void read_all_states();

private:
  int fd_{-1};
  std::string dev_;
  int baud_{1000000};

  std::thread loop_thread_;
  std::atomic<bool> running_{false};
  double loop_period_s_{0.001}; // 1 kHz
  std::vector<uint8_t> ids_;
  std::vector<MotorIO> state_;
  std::mutex cmd_mtx_;
  std::vector<double> vel_cmd_latest_;
};

// ==================== ROS 2 Node ====================

class MotorDriverNode : public rclcpp::Node {
public:
  explicit MotorDriverNode(const rclcpp::NodeOptions& opts);

private:
  // ROS interfaces
  rclcpp::Subscription<omnibot_msgs::msg::WheelVelCmd>::SharedPtr sub_cmd_;
  rclcpp::Publisher<omnibot_msgs::msg::MotorStates>::SharedPtr pub_states_;
  rclcpp::Service<omnibot_msgs::srv::TorqueEnable>::SharedPtr srv_torque_;
  rclcpp::Service<omnibot_msgs::srv::SetOperatingMode>::SharedPtr srv_mode_;
  rclcpp::Service<omnibot_msgs::srv::SetInt16>::SharedPtr srv_int16_;
  rclcpp::TimerBase::SharedPtr pub_timer_;

  // Params
  std::string device_;
  int baud_{1000000};
  std::vector<int64_t> motor_ids_param_;
  double loop_hz_{1000.0};
  double cmd_timeout_s_{0.1};

  // State
  HardwareBackend hw_;
  rclcpp::Time last_cmd_time_;

  // Helpers
  void on_cmd(const omnibot_msgs::msg::WheelVelCmd::SharedPtr msg);
  void publish_states();
};

} // namespace omnibot_motor_driver
