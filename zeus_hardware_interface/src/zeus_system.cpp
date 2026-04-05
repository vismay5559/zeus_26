#include "zeus_hardware_interface/zeus_system.hpp"
#include "zeus_hardware_interface/encoder_utils.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <linux/spi/spidev.h>
#include <pluginlib/class_list_macros.hpp>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>

namespace zeus_hardware_interface
{

namespace
{
constexpr double kTwoPi = 6.28318530717958647692;
constexpr std::size_t kDefaultCan0JointCount = 5;
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
  hw_commands_.assign(joint_count, 0.0);
  current_interpolated_targets_.assign(joint_count, 0.0);
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

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ZeusSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!can0_driver_.open_port(can0_name_)) {
    RCLCPP_ERROR(rclcpp::get_logger("ZeusSystemHardware"), "Failed to open CAN interface %s", can0_name_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!can1_driver_.open_port(can1_name_)) {
    RCLCPP_ERROR(rclcpp::get_logger("ZeusSystemHardware"), "Failed to open CAN interface %s", can1_name_.c_str());
    can0_driver_.close_port();
    return hardware_interface::CallbackReturn::ERROR;
  }

  encoder_spi_fd_ = encoder_utils::open_as5048a_spi(
    encoder_spi_device_, encoder_spi_mode_, encoder_spi_speed_hz_);
  
  if (encoder_spi_fd_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("ZeusSystemHardware"), "Failed to open encoder SPI device %s", encoder_spi_device_.c_str());
    can1_driver_.close_port();
    can0_driver_.close_port();
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("ZeusSystemHardware"),
    "Successfully configured hardware: CAN0=%s, CAN1=%s, Encoder SPI=%s",
    can0_name_.c_str(), can1_name_.c_str(), encoder_spi_device_.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ZeusSystemHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  close_encoder_spi();
  can1_driver_.close_port();
  can0_driver_.close_port();

  RCLCPP_INFO(rclcpp::get_logger("ZeusSystemHardware"), "Hardware cleaned up successfully");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ZeusSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(info_.joints.size());

  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(
      info_.joints[i].name, "after_spring_angle", &hw_states_[i]);
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
  if (!read_as5048a_encoder_chain(spi_positions_buffer_)) {
    return hardware_interface::return_type::ERROR;
  }

  const std::size_t joint_count = std::min(hw_states_.size(), spi_positions_buffer_.size());
  for (std::size_t i = 0; i < joint_count; ++i) {
    hw_states_[i] =
      (LOW_PASS_ALPHA * spi_positions_buffer_[i]) + ((1.0 - LOW_PASS_ALPHA) * hw_states_[i]);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ZeusSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    const double step_size = (hw_commands_[i] - current_interpolated_targets_[i]) * 0.25;
    current_interpolated_targets_[i] += step_size;

    auto & can_driver = use_can0_for_joint(i) ? can0_driver_ : can1_driver_;
    const uint32_t node_id = node_id_for_joint(i);
    
    can_driver.send_position_target(node_id, static_cast<float>(current_interpolated_targets_[i]));
    // Soft error handling: Missed frames on a non-blocking socket are ignored to maintain 1kHz continuity.
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

}  // namespace zeus_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  zeus_hardware_interface::ZeusSystemHardware, hardware_interface::SystemInterface)