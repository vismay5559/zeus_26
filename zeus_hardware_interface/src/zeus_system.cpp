#include "zeus_hardware_interface/zeus_system.hpp"
#include "zeus_hardware_interface/encoder_utils.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <pluginlib/class_list_macros.hpp>
#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <fstream>

namespace zeus_hardware_interface
{

namespace
{
constexpr double kTwoPi = 6.28318530717958647692;
constexpr std::size_t kDefaultCan0JointCount = 5;
constexpr std::size_t kMaxCanFramesPerRead = 128;

bool contains_token(const std::string & text, const std::string & token)
{
  return text.find(token) != std::string::npos;
}

float read_float32_le(const uint8_t * data)
{
  float value = 0.0f;
  std::memcpy(&value, data, sizeof(value));
  return value;
}

// Write a string to a sysfs file (returns true on success).
bool sysfs_write(const std::string & path, const std::string & value)
{
  std::ofstream file(path);
  if (!file.is_open()) {
    return false;
  }
  file << value;
  return file.good();
}
}  // namespace

hardware_interface::CallbackReturn ZeusSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  can0_name_ = info_.hardware_parameters.count("can0") ?
    info_.hardware_parameters.at("can0") : "can0";
  can1_name_ = info_.hardware_parameters.count("can1") ?
    info_.hardware_parameters.at("can1") : "can1";

  command_only_mode_ = info_.hardware_parameters.count("command_only_mode") &&
    info_.hardware_parameters.at("command_only_mode") == "true";

  imu_sensor_name_ = info_.hardware_parameters.count("imu_sensor_name") ?
    info_.hardware_parameters.at("imu_sensor_name") : "imu";
  imu_uart_device_ = info_.hardware_parameters.count("imu_uart_device") ?
    info_.hardware_parameters.at("imu_uart_device") : "/dev/ttyAMA0";
  if (info_.hardware_parameters.count("imu_uart_baud_rate")) {
    imu_uart_baud_rate_ = static_cast<uint32_t>(std::stoul(
      info_.hardware_parameters.at("imu_uart_baud_rate")));
  }
  if (info_.hardware_parameters.count("imu_reset_gpio")) {
    imu_reset_gpio_ = std::stoi(info_.hardware_parameters.at("imu_reset_gpio"));
  }
  if (info_.hardware_parameters.count("imu_report_interval_us")) {
    imu_report_interval_us_ = static_cast<uint32_t>(std::stoul(
      info_.hardware_parameters.at("imu_report_interval_us")));
  }

  encoder_spi_device_ = info_.hardware_parameters.count("encoder_spi_device") ?
    info_.hardware_parameters.at("encoder_spi_device") : "/dev/spidev0.1";

  if (info_.hardware_parameters.count("encoder_spi_speed_hz")) {
    encoder_spi_speed_hz_ = static_cast<uint32_t>(std::stoul(
      info_.hardware_parameters.at("encoder_spi_speed_hz")));
  }

  if (info_.hardware_parameters.count("encoder_spi_mode")) {
    encoder_spi_mode_ = static_cast<uint8_t>(std::stoul(
      info_.hardware_parameters.at("encoder_spi_mode")));
  }

  if (info_.hardware_parameters.count("num_daisy_encoders")) {
    num_daisy_encoders_ = static_cast<std::size_t>(std::stoul(
      info_.hardware_parameters.at("num_daisy_encoders")));
  }

  const std::size_t joint_count = info_.joints.size();
  hw_states_.assign(joint_count, 0.0);
  hw_commands_.assign(joint_count, std::numeric_limits<double>::quiet_NaN());
  odrive_load_encoder_positions_.assign(joint_count, 0.0);
  odrive_torque_estimates_.assign(joint_count, 0.0);
  current_interpolated_targets_.assign(joint_count, std::numeric_limits<double>::quiet_NaN());
  prev_hw_commands_.assign(joint_count, std::numeric_limits<double>::quiet_NaN());
  spi_positions_buffer_.assign(joint_count, 0.0);

  // Pre-allocate zero-copy SPI buffers (0xFF triggers angle read)
  spi_tx_buffer_.assign(num_daisy_encoders_ * 2, 0xFF);
  spi_rx_buffer_.assign(num_daisy_encoders_ * 2, 0x00);

  joint_node_ids_.resize(joint_count);
  joint_uses_can0_.resize(joint_count);

  for (std::size_t i = 0; i < joint_count; ++i) {
    const auto & joint = info_.joints[i];
    joint_node_ids_[i] = joint.parameters.count("node_id") ?
      static_cast<uint32_t>(std::stoul(joint.parameters.at("node_id"))) :
      static_cast<uint32_t>(i + 1);

    if (joint.parameters.count("can_bus")) {
      joint_uses_can0_[i] = (joint.parameters.at("can_bus") != "can1");
    } else {
      joint_uses_can0_[i] = (i < kDefaultCan0JointCount);
    }
  }

  encoder_joint_map_.resize(num_daisy_encoders_);
  for (std::size_t i = 0; i < num_daisy_encoders_; ++i) {
    encoder_joint_map_[i] = i < joint_count ? i : 0;
  }

  if (info_.hardware_parameters.count("encoder_to_joint_map")) {
    const auto & map_str = info_.hardware_parameters.at("encoder_to_joint_map");
    encoder_joint_map_.clear();
    std::size_t pos = 0;
    std::size_t comma_pos;
    while ((comma_pos = map_str.find(',', pos)) != std::string::npos) {
      encoder_joint_map_.push_back(std::stoul(map_str.substr(pos, comma_pos - pos)));
      pos = comma_pos + 1;
    }
    encoder_joint_map_.push_back(std::stoul(map_str.substr(pos)));
  }

  // Bounds validation
  if (encoder_joint_map_.size() != num_daisy_encoders_) {
    RCLCPP_ERROR(rclcpp::get_logger("ZeusSystemHardware"), "encoder_to_joint_map length mismatch.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (std::size_t mapped_joint : encoder_joint_map_) {
    if (mapped_joint >= joint_count) {
      RCLCPP_ERROR(rclcpp::get_logger("ZeusSystemHardware"), "encoder_to_joint_map contains out-of-bounds joint index.");
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  sensor_states_.clear();
  sensor_state_indices_.clear();
  gpio_switches_.clear();

  for (const auto & sensor : info_.sensors) {
    for (const auto & state_interface : sensor.state_interfaces) {
      sensor_state_indices_.push_back(
        SensorStateIndex{sensor.name, state_interface.name, sensor_states_.size()});
      sensor_states_.push_back(0.0);
    }

    const bool looks_like_imu =
      sensor.name == imu_sensor_name_ ||
      has_sensor_state(sensor.name, "orientation.x") ||
      has_sensor_state(sensor.name, "linear_acceleration.x") ||
      has_sensor_state(sensor.name, "angular_velocity.x");
    if (looks_like_imu) {
      imu_sensor_name_ = sensor.name;
      has_imu_sensor_ = true;
    }

    // Mechanical switch: identified by "contact" state interface or explicit gpio_pin param.
    const bool has_gpio_pin = sensor.parameters.count("gpio_pin") > 0;
    const bool looks_like_switch =
      has_sensor_state(sensor.name, "contact") || has_gpio_pin ||
      contains_token(sensor.name, "switch");
    if (looks_like_switch && !looks_like_imu) {
      GpioSwitch sw;
      sw.sensor_name = sensor.name;
      sw.gpio_pin = has_gpio_pin ?
        std::stoi(sensor.parameters.at("gpio_pin")) : -1;
      sw.active_low = sensor.parameters.count("active_low") ?
        (sensor.parameters.at("active_low") == "true") : false;
      if (sw.gpio_pin < 0) {
        RCLCPP_WARN(rclcpp::get_logger("ZeusSystemHardware"),
          "Switch sensor '%s' has no gpio_pin parameter; contact reads will be skipped.",
          sensor.name.c_str());
      }
      gpio_switches_.push_back(sw);
    }
  }

  if (has_sensor_state(imu_sensor_name_, "orientation.w")) {
    set_sensor_state(imu_sensor_name_, "orientation.w", 1.0);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ZeusSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  const bool needs_can0 = std::any_of(
    joint_uses_can0_.begin(), joint_uses_can0_.end(), [](bool uses_can0) { return uses_can0; });
  const bool needs_can1 = std::any_of(
    joint_uses_can0_.begin(), joint_uses_can0_.end(), [](bool uses_can0) { return !uses_can0; });

  if (needs_can0 && !can0_driver_.open_port(can0_name_)) {
    RCLCPP_ERROR(rclcpp::get_logger("ZeusSystemHardware"), "Failed to open CAN interface %s", can0_name_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (needs_can1 && !can1_driver_.open_port(can1_name_)) {
    RCLCPP_ERROR(rclcpp::get_logger("ZeusSystemHardware"), "Failed to open CAN interface %s", can1_name_.c_str());
    can0_driver_.close_port();
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (command_only_mode_) {
    RCLCPP_WARN(
      rclcpp::get_logger("ZeusSystemHardware"),
      "Command-only mode enabled: SPI encoder reads are bypassed and states mirror commands.");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  encoder_spi_fd_ = encoder_utils::open_as5048a_spi(
    encoder_spi_device_, encoder_spi_mode_, encoder_spi_speed_hz_);

  if (encoder_spi_fd_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("ZeusSystemHardware"), "Failed to open encoder SPI device %s", encoder_spi_device_.c_str());
    can1_driver_.close_port();
    can0_driver_.close_port();
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (has_imu_sensor_ && !imu_reader_.open_device(
      imu_uart_device_, imu_uart_baud_rate_, imu_reset_gpio_, imu_report_interval_us_)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ZeusSystemHardware"),
      "Failed to open BNO085 IMU UART device %s", imu_uart_device_.c_str());
    close_encoder_spi();
    can1_driver_.close_port();
    can0_driver_.close_port();
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Export and configure GPIO pins for mechanical switches via Linux sysfs.
  for (const auto & sw : gpio_switches_) {
    if (sw.gpio_pin < 0) {
      continue;
    }

    const std::string pin_str = std::to_string(sw.gpio_pin);
    const std::string gpio_dir = "/sys/class/gpio/gpio" + pin_str;

    // Export pin (ignore errors if already exported)
    sysfs_write("/sys/class/gpio/export", pin_str);

    // Configure as input
    if (!sysfs_write(gpio_dir + "/direction", "in")) {
      RCLCPP_WARN(rclcpp::get_logger("ZeusSystemHardware"),
        "Could not set GPIO %d direction to 'in'; switch '%s' may not read correctly.",
        sw.gpio_pin, sw.sensor_name.c_str());
    }

    // Open value file for low-latency reads at 1 kHz (lseek + read avoids re-open overhead)
    const std::string value_path = gpio_dir + "/value";
    const int fd = ::open(value_path.c_str(), O_RDONLY);
    if (fd < 0) {
      RCLCPP_WARN(rclcpp::get_logger("ZeusSystemHardware"),
        "Could not open GPIO %d value file; switch '%s' reads will be skipped.",
        sw.gpio_pin, sw.sensor_name.c_str());
    } else {
      gpio_fds_[sw.gpio_pin] = fd;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("ZeusSystemHardware"),
    "Successfully configured hardware: CAN0=%s, CAN1=%s, Encoder SPI=%s, IMU UART=%s, GPIO switches=%zu",
    can0_name_.c_str(), can1_name_.c_str(), encoder_spi_device_.c_str(),
    has_imu_sensor_ ? imu_uart_device_.c_str() : "disabled",
    gpio_switches_.size());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ZeusSystemHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Close GPIO file descriptors and unexport pins
  for (const auto & [pin, fd] : gpio_fds_) {
    if (fd >= 0) {
      ::close(fd);
    }
    sysfs_write("/sys/class/gpio/unexport", std::to_string(pin));
  }
  gpio_fds_.clear();

  imu_reader_.close_device();
  close_encoder_spi();
  can1_driver_.close_port();
  can0_driver_.close_port();

  RCLCPP_INFO(rclcpp::get_logger("ZeusSystemHardware"), "Hardware cleaned up successfully");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ZeusSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  std::size_t total_joint_si = 0;
  for (const auto & joint : info_.joints) {
    total_joint_si += joint.state_interfaces.size();
  }
  state_interfaces.reserve(total_joint_si + sensor_state_indices_.size());

  // Export only the state interfaces that are declared in the URDF for each joint.
  // SEA joints declare after_spring_angle + load_encoder_position + torque_estimate.
  // QDD joints declare only load_encoder_position + torque_estimate.
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    for (const auto & si : info_.joints[i].state_interfaces) {
      if (si.name == "after_spring_angle") {
        state_interfaces.emplace_back(
          info_.joints[i].name, "after_spring_angle", &hw_states_[i]);
      } else if (si.name == "load_encoder_position") {
        state_interfaces.emplace_back(
          info_.joints[i].name, "load_encoder_position", &odrive_load_encoder_positions_[i]);
      } else if (si.name == "torque_estimate") {
        state_interfaces.emplace_back(
          info_.joints[i].name, "torque_estimate", &odrive_torque_estimates_[i]);
      }
    }
  }

  for (const auto & sensor_state : sensor_state_indices_) {
    state_interfaces.emplace_back(
      sensor_state.sensor_name, sensor_state.interface_name,
      &sensor_states_[sensor_state.value_index]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ZeusSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(info_.joints.size());

  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(
      info_.joints[i].name, "target_actuator_angle", &hw_commands_[i]);
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn ZeusSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (encoder_spi_fd_ < 0) {
    if (command_only_mode_) {
      RCLCPP_INFO(rclcpp::get_logger("ZeusSystemHardware"), "Hardware activated in command-only mode");
      return hardware_interface::CallbackReturn::SUCCESS;
    }

    RCLCPP_ERROR(rclcpp::get_logger("ZeusSystemHardware"), "Encoder SPI not open; on_configure may have failed");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("ZeusSystemHardware"), "Hardware activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ZeusSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ZeusSystemHardware"), "Hardware deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ZeusSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Always drain CAN telemetry frames so load_encoder_position and torque_estimate
  // are populated even when running in command_only_mode (no SPI encoders connected).
  read_odrive_can_telemetry();

  if (command_only_mode_) {
    for (std::size_t i = 0; i < hw_states_.size(); ++i) {
      if (!std::isnan(hw_commands_[i])) {
        hw_states_[i] = hw_commands_[i];
      }
    }
    return hardware_interface::return_type::OK;
  }

  if (!read_as5048a_encoder_chain(spi_positions_buffer_)) {
    return hardware_interface::return_type::ERROR;
  }

  const std::size_t joint_count = std::min(hw_states_.size(), spi_positions_buffer_.size());
  for (std::size_t i = 0; i < joint_count; ++i) {
    hw_states_[i] =
      (LOW_PASS_ALPHA * spi_positions_buffer_[i]) + ((1.0 - LOW_PASS_ALPHA) * hw_states_[i]);
  }

  read_odrive_can_telemetry();

  if (has_imu_sensor_) {
    imu_reader_.read_sample(last_imu_sample_);
    apply_imu_sample(last_imu_sample_);
  }

  if (!gpio_switches_.empty()) {
    read_gpio_switches();
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ZeusSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    if (std::isnan(hw_commands_[i])) {
      continue;  // forward_command_controller publishes NaN until the first command arrives
    }

    // The commander already linearly interpolates the 22 Hz gait waypoints to a
    // smooth 1 kHz stream. Pass the command straight through — no extra filter.
    // Velocity feedforward = position change between consecutive 1 kHz commands
    // (Δpos / 1ms = rev/s). This tells the ODrive how fast to move right now,
    // eliminating phase-lag without needing higher position gains.
    float vel_ff = 0.0f;
    if (!std::isnan(prev_hw_commands_[i])) {
      vel_ff = static_cast<float>((hw_commands_[i] - prev_hw_commands_[i]) * 1000.0);
    }
    prev_hw_commands_[i] = hw_commands_[i];

    auto & can_driver = use_can0_for_joint(i) ? can0_driver_ : can1_driver_;
    const uint32_t node_id = node_id_for_joint(i);

    can_driver.send_position_target(node_id, static_cast<float>(hw_commands_[i]), vel_ff);
  }

  return hardware_interface::return_type::OK;
}

void ZeusSystemHardware::close_encoder_spi()
{
  encoder_utils::close_as5048a_spi(encoder_spi_fd_);
  encoder_spi_fd_ = -1;
}

bool ZeusSystemHardware::read_as5048a_encoder_chain(std::vector<double> & positions)
{
  if (encoder_spi_fd_ < 0 || num_daisy_encoders_ == 0) {
    return false;
  }

  std::vector<uint16_t> raw_words;
  if (!encoder_utils::read_as5048a_daisy_chain(
      encoder_spi_fd_, num_daisy_encoders_, spi_tx_buffer_, spi_rx_buffer_, raw_words)) {
    return false;
  }

  // Preserve state by default; overwrite only valid measurements.
  for (std::size_t i = 0; i < num_daisy_encoders_ && i < raw_words.size(); ++i) {
    encoder_utils::AS5048AWord word;
    word.raw = raw_words[i];

    // Frame Validation & Parity Checking
    if (word.error() || word.mag_too_weak() || word.mag_too_strong() ||
        !encoder_utils::validate_as5048a_parity(word)) {
      continue; // Corrupted frame: hold last known good value
    }

    const uint16_t raw_angle = word.angle();
    const double angle_rad = encoder_utils::as5048a_raw_to_radians(raw_angle);

    std::size_t physical_encoder_idx = (num_daisy_encoders_ - 1) - i;

    if (physical_encoder_idx < encoder_joint_map_.size()) {
      const std::size_t joint_idx = encoder_joint_map_[physical_encoder_idx];
      positions[joint_idx] = angle_rad;
    }
  }

  return true;
}

bool ZeusSystemHardware::use_can0_for_joint(std::size_t joint_index) const
{
  return joint_uses_can0_.at(joint_index);
}

uint32_t ZeusSystemHardware::node_id_for_joint(std::size_t joint_index) const
{
  return joint_node_ids_.at(joint_index);
}

void ZeusSystemHardware::read_odrive_can_telemetry()
{
  drain_odrive_can_telemetry(can0_driver_, true);
  drain_odrive_can_telemetry(can1_driver_, false);
}

void ZeusSystemHardware::drain_odrive_can_telemetry(
  zeus_can_interface::SocketCANTransceiver & can_driver, bool uses_can0)
{
  struct canfd_frame frame;
  std::size_t frames_read = 0;
  while (frames_read < kMaxCanFramesPerRead && can_driver.read_frame(frame)) {
    handle_odrive_can_frame(frame, uses_can0);
    ++frames_read;
  }
}

void ZeusSystemHardware::handle_odrive_can_frame(const struct canfd_frame & frame, bool uses_can0)
{
  if ((frame.can_id & CAN_RTR_FLAG) != 0 || frame.len < 8) {
    return;
  }

  const uint32_t standard_id = frame.can_id & CAN_SFF_MASK;
  const uint32_t node_id = standard_id >> 5;
  const uint32_t command_id = standard_id & 0x1F;
  const std::size_t joint_index = joint_index_for_can_node(node_id, uses_can0);
  if (joint_index >= info_.joints.size()) {
    return;
  }

  if (command_id == zeus_can_interface::ODESC_CMD_GET_ENCODER_ESTIMATES) {
    odrive_load_encoder_positions_[joint_index] = read_float32_le(&frame.data[0]);
  } else if (command_id == zeus_can_interface::ODESC_CMD_GET_TORQUES) {
    odrive_torque_estimates_[joint_index] = read_float32_le(&frame.data[4]);
  }
}

std::size_t ZeusSystemHardware::joint_index_for_can_node(uint32_t node_id, bool uses_can0) const
{
  for (std::size_t i = 0; i < joint_node_ids_.size(); ++i) {
    if (joint_node_ids_[i] == node_id && joint_uses_can0_[i] == uses_can0) {
      return i;
    }
  }

  return info_.joints.size();
}

bool ZeusSystemHardware::has_sensor_state(
  const std::string & sensor_name, const std::string & interface_name) const
{
  return std::any_of(
    sensor_state_indices_.begin(), sensor_state_indices_.end(),
    [&](const SensorStateIndex & sensor_state) {
      return sensor_state.sensor_name == sensor_name &&
             sensor_state.interface_name == interface_name;
    });
}

void ZeusSystemHardware::set_sensor_state(
  const std::string & sensor_name, const std::string & interface_name, double value)
{
  for (const auto & sensor_state : sensor_state_indices_) {
    if (sensor_state.sensor_name == sensor_name &&
        sensor_state.interface_name == interface_name) {
      sensor_states_[sensor_state.value_index] = value;
      return;
    }
  }
}

void ZeusSystemHardware::apply_imu_sample(const sensor_utils::BNO085Sample & sample)
{
  set_sensor_state(imu_sensor_name_, "orientation.x", sample.orientation_xyzw[0]);
  set_sensor_state(imu_sensor_name_, "orientation.y", sample.orientation_xyzw[1]);
  set_sensor_state(imu_sensor_name_, "orientation.z", sample.orientation_xyzw[2]);
  set_sensor_state(imu_sensor_name_, "orientation.w", sample.orientation_xyzw[3]);

  set_sensor_state(imu_sensor_name_, "linear_acceleration.x", sample.linear_acceleration[0]);
  set_sensor_state(imu_sensor_name_, "linear_acceleration.y", sample.linear_acceleration[1]);
  set_sensor_state(imu_sensor_name_, "linear_acceleration.z", sample.linear_acceleration[2]);

  set_sensor_state(imu_sensor_name_, "angular_velocity.x", sample.angular_velocity[0]);
  set_sensor_state(imu_sensor_name_, "angular_velocity.y", sample.angular_velocity[1]);
  set_sensor_state(imu_sensor_name_, "angular_velocity.z", sample.angular_velocity[2]);
}

void ZeusSystemHardware::read_gpio_switches()
{
  char buf[2];
  for (const auto & sw : gpio_switches_) {
    if (sw.gpio_pin < 0) {
      continue;
    }

    auto it = gpio_fds_.find(sw.gpio_pin);
    if (it == gpio_fds_.end() || it->second < 0) {
      continue;
    }

    // Seek to start of the sysfs value file and read a single ASCII character ('0' or '1').
    ::lseek(it->second, 0, SEEK_SET);
    const ssize_t n = ::read(it->second, buf, 1);
    if (n != 1) {
      continue;
    }

    // Parse the character: '1' → pin HIGH, '0' → pin LOW.
    bool pin_high = (buf[0] == '1');
    // Apply active_low inversion: active_low means contact when pin is LOW.
    double contact = (sw.active_low ? !pin_high : pin_high) ? 1.0 : 0.0;
    set_sensor_state(sw.sensor_name, "contact", contact);
  }
}

}  // namespace zeus_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  zeus_hardware_interface::ZeusSystemHardware, hardware_interface::SystemInterface)
