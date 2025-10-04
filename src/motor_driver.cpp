#include "motor_driver.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <limits>
#include <pthread.h>
#include <stdexcept>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

using namespace std::chrono_literals;

namespace omnibot_motor_driver {

// =================== HardwareBackend ===================

HardwareBackend::~HardwareBackend() { stop(); }

bool HardwareBackend::configure(const std::string& device, int baudrate,
                                const std::vector<uint8_t>& motor_ids) {
  ids_ = motor_ids;
  state_.assign(ids_.size(), {});
  for (size_t i = 0; i < ids_.size(); ++i) state_[i].id = ids_[i];
  vel_cmd_latest_.assign(ids_.size(), 0.0);
  return hw_open(device, baudrate);
}

bool HardwareBackend::start(double loop_hz) {
  if (running_) return true;
  loop_period_s_ = 1.0 / std::max(1.0, loop_hz);
  running_ = true;
  loop_thread_ = std::thread([this]() {
    rt_raise_priority();
    auto next = std::chrono::steady_clock::now();
    // read at ~1 kHz or same cadence as loop
    size_t read_div = std::max<size_t>(1, static_cast<size_t>(std::round(1000.0 * loop_period_s_)));
    size_t k = 0;
    while (running_) {
      next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(loop_period_s_));
      step(std::chrono::steady_clock::now());
      if ((k++ % read_div) == 0) read_all_states();
      std::this_thread::sleep_until(next);
    }
  });
  return true;
}

void HardwareBackend::stop() {
  if (!running_) return;
  running_ = false;
  if (loop_thread_.joinable()) loop_thread_.join();
  hw_close();
}

bool HardwareBackend::step(std::chrono::steady_clock::time_point /*now*/) {
  std::vector<double> cmds;
  {
    std::lock_guard<std::mutex> lk(cmd_mtx_);
    cmds = vel_cmd_latest_;
  }
  return dxl_sync_write_goal_velocity(cmds);
}

void HardwareBackend::set_velocity_commands(const std::vector<double>& vels) {
  std::lock_guard<std::mutex> lk(cmd_mtx_);
  size_t n = std::min(vels.size(), vel_cmd_latest_.size());
  for (size_t i = 0; i < n; ++i) vel_cmd_latest_[i] = vels[i];
}

bool HardwareBackend::torque_enable(uint8_t id, bool enable, std::string& msg) {
  bool ok = dxl_write1(id, ADDR_TORQUE_ENABLE, static_cast<uint8_t>(enable ? 1 : 0));
  msg = ok ? "OK" : "WRITE_FAIL";
  return ok;
}

bool HardwareBackend::set_operating_mode(uint8_t id, uint8_t mode, std::string& msg) {
  bool ok = dxl_write1(id, ADDR_OPERATING_MODE, mode);
  msg = ok ? "OK" : "WRITE_FAIL";
  return ok;
}

bool HardwareBackend::set_int16(uint8_t id, int16_t value, std::string& msg) {
  bool ok = dxl_write2(id, ADDR_GOAL_CURRENT, static_cast<uint16_t>(value));
  msg = ok ? "OK (Goal Current)" : "WRITE_FAIL";
  return ok;
}

std::vector<HardwareBackend::MotorIO> HardwareBackend::readback() {
  return state_;
}

bool HardwareBackend::rt_raise_priority() {
  sched_param sch; sch.sched_priority = 60;
  pthread_setschedparam(pthread_self(), SCHED_FIFO, &sch); // best-effort
  return true;
}

// ---------------- Serial I/O ----------------

static speed_t to_baud(int baudrate) {
  switch (baudrate) {
    case 57600:   return B57600;
    case 115200:  return B115200;
#ifdef B1000000
    case 1000000: return B1000000;
#endif
#ifdef B2000000
    case 2000000: return B2000000;
#endif
#ifdef B3000000
    case 3000000: return B3000000;
#endif
    default:      return B115200;
  }
}

bool HardwareBackend::hw_open(const std::string& device, int baudrate) {
  dev_ = device; baud_ = baudrate;
  fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) return false;

  termios tio{};
  if (tcgetattr(fd_, &tio) != 0) return false;
  cfmakeraw(&tio);
  tio.c_cflag |= (CLOCAL | CREAD);
  tio.c_cflag &= ~CSTOPB;
  tio.c_cflag &= ~PARENB;
  tio.c_cflag &= ~CSIZE;
  tio.c_cflag |= CS8;
  tio.c_cc[VTIME] = 0;
  tio.c_cc[VMIN] = 0;

  speed_t sp = to_baud(baudrate);
  cfsetispeed(&tio, sp);
  cfsetospeed(&tio, sp);
  if (tcsetattr(fd_, TCSANOW, &tio) != 0) return false;

  tcflush(fd_, TCIOFLUSH);
  return true;
}

void HardwareBackend::hw_close() {
  if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

bool HardwareBackend::write_bytes(const uint8_t* data, size_t n) {
  size_t off = 0;
  while (off < n) {
    ssize_t w = ::write(fd_, data + off, n - off);
    if (w < 0) {
      if (errno == EAGAIN) continue;
      return false;
    }
    off += static_cast<size_t>(w);
  }
  return true;
}

bool HardwareBackend::read_bytes(uint8_t* out, size_t n, int timeout_us) {
  size_t off = 0;
  auto start = std::chrono::steady_clock::now();
  while (off < n) {
    ssize_t r = ::read(fd_, out + off, n - off);
    if (r < 0) {
      if (errno == EAGAIN) {
        auto dt = std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::steady_clock::now() - start).count();
        if (dt > timeout_us) return false;
        continue;
      }
      return false;
    } else if (r == 0) {
      auto dt = std::chrono::duration_cast<std::chrono::microseconds>(
                  std::chrono::steady_clock::now() - start).count();
      if (dt > timeout_us) return false;
      continue;
    }
    off += static_cast<size_t>(r);
  }
  return true;
}

// ---------------- Protocol helpers ----------------

// Simple CRC-16/IBM bitwise (fast enough here; swap for table if you like)
static uint16_t crc_update(uint16_t crc, uint8_t data) {
  crc ^= data;
  for (int i = 0; i < 8; ++i)
    crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
  return crc;
}

uint16_t HardwareBackend::compute_crc(const std::vector<uint8_t>& pkt) {
  uint16_t crc = 0;
  for (auto b : pkt) crc = crc_update(crc, b);
  return crc;
}

bool HardwareBackend::txrx(uint8_t id, uint8_t inst, const std::vector<uint8_t>& params,
                           std::vector<uint8_t>* status_params, int expect_len) {
  std::vector<uint8_t> pkt;
  pkt.reserve(10 + params.size());
  // Header
  pkt.push_back(0xFF); pkt.push_back(0xFF); pkt.push_back(0xFD); pkt.push_back(0x00);
  pkt.push_back(id);
  uint16_t length = static_cast<uint16_t>(params.size() + 3); // INST + params + CRC(2) excluded here, but spec wants +3
  pkt.push_back(uint8_t(length & 0xFF));
  pkt.push_back(uint8_t((length >> 8) & 0xFF));
  pkt.push_back(inst);
  pkt.insert(pkt.end(), params.begin(), params.end());
  uint16_t crc = compute_crc(pkt);
  pkt.push_back(uint8_t(crc & 0xFF));
  pkt.push_back(uint8_t((crc >> 8) & 0xFF));

  if (!write_bytes(pkt.data(), pkt.size())) return false;

  if (!status_params) return true; // e.g., Sync Write

  // Status header (7 bytes)
  uint8_t hdr[7];
  if (!read_bytes(hdr, 7)) return false;
  if (!(hdr[0]==0xFF && hdr[1]==0xFF && hdr[2]==0xFD && hdr[3]==0x00)) return false;
  uint16_t slen = uint16_t(hdr[5]) | (uint16_t(hdr[6])<<8);
  if (slen < 4) return false;

  std::vector<uint8_t> rest(slen);
  if (!read_bytes(rest.data(), rest.size())) return false;
  // rest[0] = error
  size_t param_bytes = slen - 4 - 1; // -CRC2 -ERROR1
  if (expect_len && int(param_bytes) != expect_len)
    param_bytes = std::min(param_bytes, static_cast<size_t>(expect_len));
  status_params->assign(rest.begin() + 1, rest.begin() + 1 + param_bytes);
  return true;
}

bool HardwareBackend::dxl_write1(uint8_t id, uint16_t addr, uint8_t val) {
  std::vector<uint8_t> p = {
    uint8_t(addr & 0xFF), uint8_t((addr>>8)&0xFF),
    val
  };
  return txrx(id, 0x03 /*WRITE*/, p, nullptr, 0);
}

bool HardwareBackend::dxl_write2(uint8_t id, uint16_t addr, uint16_t val) {
  std::vector<uint8_t> p = {
    uint8_t(addr & 0xFF), uint8_t((addr>>8)&0xFF),
    uint8_t(val & 0xFF), uint8_t((val>>8)&0xFF)
  };
  return txrx(id, 0x03, p, nullptr, 0);
}

bool HardwareBackend::dxl_write4(uint8_t id, uint16_t addr, int32_t val) {
  std::vector<uint8_t> p = {
    uint8_t(addr & 0xFF), uint8_t((addr>>8)&0xFF),
    uint8_t(val & 0xFF), uint8_t((val>>8)&0xFF),
    uint8_t((val>>16) & 0xFF), uint8_t((val>>24) & 0xFF)
  };
  return txrx(id, 0x03, p, nullptr, 0);
}

bool HardwareBackend::dxl_read(uint8_t id, uint16_t addr, uint16_t len, std::vector<uint8_t>& out) {
  std::vector<uint8_t> p = {
    uint8_t(addr & 0xFF), uint8_t((addr>>8)&0xFF),
    uint8_t(len & 0xFF),  uint8_t((len>>8)&0xFF)
  };
  out.clear();
  return txrx(id, 0x02 /*READ*/, p, &out, len);
}

bool HardwareBackend::dxl_sync_write_goal_velocity(const std::vector<double>& rad_s) {
  if (ids_.empty()) return true;
  const uint16_t addr = ADDR_GOAL_VELOCITY;
  const uint16_t data_len = 4;

  std::vector<uint8_t> params;
  // [ADDR_L, ADDR_H, LEN_L, LEN_H]
  params.push_back(uint8_t(addr & 0xFF));
  params.push_back(uint8_t((addr >> 8) & 0xFF));
  params.push_back(uint8_t(data_len & 0xFF));
  params.push_back(uint8_t((data_len >> 8) & 0xFF));

  // Then repeating [ID][data_len bytes]
  for (size_t i = 0; i < ids_.size(); ++i) {
    int32_t vel_units = rad_s_to_dxl(rad_s[i]);
    params.push_back(ids_[i]);
    params.push_back(uint8_t(vel_units & 0xFF));
    params.push_back(uint8_t((vel_units >> 8) & 0xFF));
    params.push_back(uint8_t((vel_units >> 16) & 0xFF));
    params.push_back(uint8_t((vel_units >> 24) & 0xFF));
  }

  return txrx(0xFE /*broadcast*/, 0x83 /*SYNC_WRITE*/, params, nullptr, 0);
}

int32_t HardwareBackend::rad_s_to_dxl(double rad_s) {
  double rpm = rad_s * 60.0 / (2.0 * M_PI);
  double lsb = rpm / VELOCITY_UNIT_RPM_PER_LSB;
  long v = std::lround(lsb);
  if (v > std::numeric_limits<int32_t>::max()) v = std::numeric_limits<int32_t>::max();
  if (v < std::numeric_limits<int32_t>::min()) v = std::numeric_limits<int32_t>::min();
  return static_cast<int32_t>(v);
}

double HardwareBackend::dxl_to_rad_s(int32_t vel_units) {
  double rpm = double(vel_units) * VELOCITY_UNIT_RPM_PER_LSB;
  return rpm * (2.0 * M_PI) / 60.0;
}

void HardwareBackend::read_all_states() {
  for (size_t i = 0; i < ids_.size(); ++i) {
    auto& s = state_[i];
    std::vector<uint8_t> buf;

    if (dxl_read(ids_[i], ADDR_PRESENT_CURRENT, 2, buf) && buf.size()==2) {
      int16_t raw = int16_t(buf[0] | (buf[1]<<8));
      s.current_a = raw * CURRENT_UNIT_AMP_PER_LSB;
    }
    if (dxl_read(ids_[i], ADDR_PRESENT_VELOCITY, 4, buf) && buf.size()==4) {
      int32_t raw = int32_t(buf[0] | (buf[1]<<8) | (buf[2]<<16) | (buf[3]<<24));
      s.velocity_rad_s = dxl_to_rad_s(raw);
    }
    if (dxl_read(ids_[i], ADDR_PRESENT_POSITION, 4, buf) && buf.size()==4) {
      int32_t raw = int32_t(buf[0] | (buf[1]<<8) | (buf[2]<<16) | (buf[3]<<24));
      s.position_rad = (double(raw) * 2.0 * M_PI) / TICKS_PER_REV;
    }
    if (dxl_read(ids_[i], ADDR_PRESENT_VOLTAGE, 2, buf) && buf.size()==2) {
      uint16_t raw = uint16_t(buf[0] | (buf[1]<<8));
      s.voltage_v = raw * VOLT_UNIT_V_PER_LSB;
    }
    if (dxl_read(ids_[i], ADDR_PRESENT_TEMP, 1, buf) && buf.size()==1) {
      s.temperature_c = buf[0];
    }
    s.hw_error = 0;
    s.moving = std::fabs(s.velocity_rad_s) > 1e-3;
  }
}

// =================== MotorDriverNode ===================

MotorDriverNode::MotorDriverNode(const rclcpp::NodeOptions& opts)
: Node("motor_driver", opts)
{
  // --- Parameters ---
  device_ = this->declare_parameter<std::string>("device", "/dev/ttyUSB0");
  baud_   = this->declare_parameter<int>("baudrate", 1000000);
  motor_ids_param_ = this->declare_parameter<std::vector<int64_t>>("motor_ids", {1,2,3});
  loop_hz_         = this->declare_parameter<double>("loop_hz", 1000.0);
  cmd_timeout_s_   = this->declare_parameter<double>("cmd_timeout_s", 0.1);

  std::vector<uint8_t> ids; ids.reserve(motor_ids_param_.size());
  for (auto v : motor_ids_param_) ids.push_back(static_cast<uint8_t>(v));

  if (!hw_.configure(device_, baud_, ids)) {
    RCLCPP_FATAL(get_logger(), "Failed to configure DYNAMIXEL backend");
    throw std::runtime_error("HW config failed");
  }
  hw_.start(loop_hz_);

  // QoS: low latency on cmd, SensorData on states
  auto qos_cmd = rclcpp::QoS(1).best_effort().durability_volatile().reliability_best_effort();
  auto qos_state = rclcpp::SensorDataQoS();

  sub_cmd_ = this->create_subscription<omnibot_msgs::msg::WheelVelCmd>(
    "wheel_vel_cmd", qos_cmd,
    std::bind(&MotorDriverNode::on_cmd, this, std::placeholders::_1));

  pub_states_ = this->create_publisher<omnibot_msgs::msg::MotorStates>(
    "motor_states", qos_state);

  pub_timer_ = this->create_wall_timer(5ms, std::bind(&MotorDriverNode::publish_states, this));

  srv_torque_ = this->create_service<omnibot_msgs::srv::TorqueEnable>(
    "torque_enable",
    [this](const auto req, auto res){
      std::string msg;
      bool ok = hw_.torque_enable(req->id, req->enable, msg);
      res->success = ok; res->message = msg;
    });

  srv_mode_ = this->create_service<omnibot_msgs::srv::SetOperatingMode>(
    "set_operating_mode",
    [this](const auto req, auto res){
      std::string msg;
      bool ok = hw_.set_operating_mode(req->id, req->mode, msg);
      res->success = ok; res->message = msg;
    });

  srv_int16_ = this->create_service<omnibot_msgs::srv::SetInt16>(
    "set_int16",
    [this](const auto req, auto res){
      std::string msg;
      bool ok = hw_.set_int16(req->id, req->value, msg);
      res->success = ok; res->message = msg;
    });

  last_cmd_time_ = this->now();
  RCLCPP_INFO(get_logger(), "motor_driver ready. loop=%.1f Hz, timeout=%.3f s",
              loop_hz_, cmd_timeout_s_);
}

void MotorDriverNode::on_cmd(const omnibot_msgs::msg::WheelVelCmd::SharedPtr msg) {
  std::vector<double> vels(msg->velocity_rad_s.begin(), msg->velocity_rad_s.end());
  hw_.set_velocity_commands(vels);
  last_cmd_time_ = msg->stamp; // respect sender time
}

void MotorDriverNode::publish_states() {
  // Failsafe: zero if stale
  if ((this->now() - last_cmd_time_).seconds() > cmd_timeout_s_) {
    hw_.set_velocity_commands(std::vector<double>(motor_ids_param_.size(), 0.0));
  }

  auto out = omnibot_msgs::msg::MotorStates();
  out.header.stamp = this->now();
  out.header.frame_id = "base_link";

  auto rb = hw_.readback();
  out.motors.resize(rb.size());
  for (size_t i = 0; i < rb.size(); ++i) {
    const auto &s = rb[i];
    auto &d = out.motors[i];
    d.id = s.id;
    d.position_rad = s.position_rad;
    d.velocity_rad_s = s.velocity_rad_s;
    d.current_a = s.current_a;
    d.voltage_v = s.voltage_v;
    d.temperature_c = s.temperature_c;
    d.hw_error = s.hw_error;
    d.moving = s.moving;
  }
  pub_states_->publish(std::move(out));
}

} // namespace omnibot_motor_driver
