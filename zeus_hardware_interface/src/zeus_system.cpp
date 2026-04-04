#include "zeus_hardware_interface/zeus_system.hpp"

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
  spi_device_ = info_.hardware_parameters.count("spi_device") ?
    info_.hardware_parameters.at("spi_device") : "/dev/spidev0.0";

  if (info_.hardware_parameters.count("spi_speed_hz")) {
    spi_speed_hz_ = static_cast<uint32_t>(std::stoul(info_.hardware_parameters.at("spi_speed_hz")));
  }
  if (info_.hardware_parameters.count("spi_mode")) {
    spi_mode_ = static_cast<uint8_t>(std::stoul(info_.hardware_parameters.at("spi_mode")));
  }
  if (info_.hardware_parameters.count("spi_bits_per_word")) {
    spi_bits_per_word_ = static_cast<uint8_t>(
      std::stoul(info_.hardware_parameters.at("spi_bits_per_word")));
  }
  if (info_.hardware_parameters.count("spi_word_size_bytes")) {
    spi_word_size_bytes_ = static_cast<uint8_t>(
      std::stoul(info_.hardware_parameters.at("spi_word_size_bytes")));
  }
  if (info_.hardware_parameters.count("spi_angle_bits")) {
    spi_angle_bits_ = static_cast<uint8_t>(std::stoul(info_.hardware_parameters.at("spi_angle_bits")));
  }
  if (info_.hardware_parameters.count("spi_angle_lsb_shift")) {
    spi_angle_lsb_shift_ = static_cast<uint8_t>(
      std::stoul(info_.hardware_parameters.at("spi_angle_lsb_shift")));
  }
  if (info_.hardware_parameters.count("spi_tx_word")) {
    spi_tx_word_ = static_cast<uint16_t>(std::stoul(info_.hardware_parameters.at("spi_tx_word"), nullptr, 0));
  }
  if (info_.hardware_parameters.count("spi_reverse_encoder_order")) {
    const auto & value = info_.hardware_parameters.at("spi_reverse_encoder_order");
    spi_reverse_encoder_order_ = (value == "true" || value == "1");
  }

  const std::size_t joint_count = info_.joints.size();
  hw_states_.assign(joint_count, 0.0);
  hw_commands_.assign(joint_count, 0.0);
  current_interpolated_targets_.assign(joint_count, 0.0);
  spi_positions_buffer_.assign(joint_count, 0.0);
  spi_tx_buffer_.assign(joint_count * spi_word_size_bytes_, 0);
  spi_rx_buffer_.assign(joint_count * spi_word_size_bytes_, 0);
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
  if (!can0_driver_.open_port(can0_name_)) {
    RCLCPP_ERROR(rclcpp::get_logger("ZeusSystemHardware"),
      "Failed to open CAN interface %s", can0_name_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!can1_driver_.open_port(can1_name_)) {
    RCLCPP_ERROR(rclcpp::get_logger("ZeusSystemHardware"),
      "Failed to open CAN interface %s", can1_name_.c_str());
    can0_driver_.close_port();
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!open_spi_port()) {
    RCLCPP_ERROR(rclcpp::get_logger("ZeusSystemHardware"),
      "Failed to open SPI encoder chain on %s", spi_device_.c_str());
    can1_driver_.close_port();
    can0_driver_.close_port();
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ZeusSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  close_spi_port();
  can1_driver_.close_port();
  can0_driver_.close_port();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ZeusSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!read_spi_encoder_positions(spi_positions_buffer_)) {
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
  bool write_ok = true;

  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    const double step_size = (hw_commands_[i] - current_interpolated_targets_[i]) * 0.25;
    current_interpolated_targets_[i] += step_size;

    auto & can_driver = use_can0_for_joint(i) ? can0_driver_ : can1_driver_;
    const uint32_t node_id = node_id_for_joint(i);
    if (!can_driver.send_position_target(
        node_id, static_cast<float>(current_interpolated_targets_[i]))) {
      write_ok = false;
    }
  }

  return write_ok ? hardware_interface::return_type::OK : hardware_interface::return_type::ERROR;
}

bool ZeusSystemHardware::open_spi_port()
{
  close_spi_port();

  spi_fd_ = open(spi_device_.c_str(), O_RDWR);
  if (spi_fd_ < 0) {
    return false;
  }

  if (ioctl(spi_fd_, SPI_IOC_WR_MODE, &spi_mode_) < 0 ||
      ioctl(spi_fd_, SPI_IOC_RD_MODE, &spi_mode_) < 0 ||
      ioctl(spi_fd_, SPI_IOC_WR_BITS_PER_WORD, &spi_bits_per_word_) < 0 ||
      ioctl(spi_fd_, SPI_IOC_RD_BITS_PER_WORD, &spi_bits_per_word_) < 0 ||
      ioctl(spi_fd_, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed_hz_) < 0 ||
      ioctl(spi_fd_, SPI_IOC_RD_MAX_SPEED_HZ, &spi_speed_hz_) < 0) {
    close_spi_port();
    return false;
  }

  return true;
}

void ZeusSystemHardware::close_spi_port()
{
  if (spi_fd_ >= 0) {
    close(spi_fd_);
    spi_fd_ = -1;
  }
}

bool ZeusSystemHardware::read_spi_encoder_positions(std::vector<double> & positions)
{
  if (spi_fd_ < 0 || spi_word_size_bytes_ == 0 || spi_angle_bits_ == 0 || spi_angle_bits_ > 16) {
    return false;
  }

  const std::size_t encoder_count = info_.joints.size();
  const std::size_t transfer_size = encoder_count * spi_word_size_bytes_;
  if (spi_tx_buffer_.size() != transfer_size) {
    spi_tx_buffer_.assign(transfer_size, 0);
  }
  if (spi_rx_buffer_.size() != transfer_size) {
    spi_rx_buffer_.assign(transfer_size, 0);
  }

  std::fill(spi_tx_buffer_.begin(), spi_tx_buffer_.end(), 0);
  std::fill(spi_rx_buffer_.begin(), spi_rx_buffer_.end(), 0);

  for (std::size_t offset = 0; offset < transfer_size; offset += spi_word_size_bytes_) {
    if (spi_word_size_bytes_ >= 1) {
      spi_tx_buffer_[offset] = static_cast<uint8_t>((spi_tx_word_ >> 8) & 0xFF);
    }
    if (spi_word_size_bytes_ >= 2) {
      spi_tx_buffer_[offset + 1] = static_cast<uint8_t>(spi_tx_word_ & 0xFF);
    }
  }

  struct spi_ioc_transfer transfer;
  std::memset(&transfer, 0, sizeof(transfer));
  transfer.tx_buf = reinterpret_cast<unsigned long>(spi_tx_buffer_.data());
  transfer.rx_buf = reinterpret_cast<unsigned long>(spi_rx_buffer_.data());
  transfer.len = static_cast<uint32_t>(transfer_size);
  transfer.speed_hz = spi_speed_hz_;
  transfer.bits_per_word = spi_bits_per_word_;

  if (ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &transfer) < 0) {
    return false;
  }

  const uint16_t angle_mask = static_cast<uint16_t>((1u << spi_angle_bits_) - 1u);
  const double counts_per_turn = static_cast<double>(1u << spi_angle_bits_);
  positions.assign(encoder_count, 0.0);

  for (std::size_t i = 0; i < encoder_count; ++i) {
    const std::size_t source_index = spi_reverse_encoder_order_ ? (encoder_count - 1U - i) : i;
    const std::size_t offset = source_index * spi_word_size_bytes_;

    uint16_t raw_word = 0;
    if (spi_word_size_bytes_ >= 1) {
      raw_word = static_cast<uint16_t>(spi_rx_buffer_[offset]) << 8;
    }
    if (spi_word_size_bytes_ >= 2) {
      raw_word |= static_cast<uint16_t>(spi_rx_buffer_[offset + 1]);
    }

    raw_word = static_cast<uint16_t>(raw_word >> spi_angle_lsb_shift_);
    const uint16_t raw_angle = static_cast<uint16_t>(raw_word & angle_mask);
    positions[i] = (static_cast<double>(raw_angle) / counts_per_turn) * kTwoPi;
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
