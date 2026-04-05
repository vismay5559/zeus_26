#ifndef ZEUS_HARDWARE_INTERFACE__ZEUS_SYSTEM_HPP_
#define ZEUS_HARDWARE_INTERFACE__ZEUS_SYSTEM_HPP_

#include <cstdint>
#include <string>
#include <vector>
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "zeus_can_interface/socketcan.hpp"

namespace zeus_hardware_interface
{

class ZeusSystemHardware : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void close_encoder_spi();
  bool read_as5048a_encoder_chain(std::vector<double> & positions);
  bool use_can0_for_joint(std::size_t joint_index) const;
  uint32_t node_id_for_joint(std::size_t joint_index) const;

  zeus_can_interface::SocketCANTransceiver can0_driver_;
  zeus_can_interface::SocketCANTransceiver can1_driver_;

  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;
  std::vector<double> current_interpolated_targets_;
  std::vector<double> spi_positions_buffer_;
  std::vector<uint8_t> spi_tx_buffer_;
  std::vector<uint8_t> spi_rx_buffer_;
  std::vector<uint32_t> joint_node_ids_;
  std::vector<bool> joint_uses_can0_;

  std::string can0_name_ = "can0";
  std::string can1_name_ = "can1";
  
  // AS5048A Encoder daisy-chain SPI configuration
  std::string encoder_spi_device_ = "/dev/spidev0.1";
  int encoder_spi_fd_ = -1;
  uint32_t encoder_spi_speed_hz_ = 1000000;
  uint8_t encoder_spi_mode_ = 1;
  uint8_t encoder_spi_bits_per_word_ = 8;
  std::size_t num_daisy_encoders_ = 10;
  std::vector<std::size_t> encoder_joint_map_;  // Maps encoder position to joint index

  static constexpr double LOW_PASS_ALPHA = 0.2;
  static constexpr std::size_t AS5048A_WORD_SIZE_BYTES = 2;
  static constexpr std::size_t AS5048A_ANGLE_BITS = 13;  // 13-bit angle (0-8192)
  static constexpr uint16_t AS5048A_ANGLE_MASK = 0x1FFF;  // 13-bit mask
  static constexpr uint16_t AS5048A_PARITY_BIT = 0x8000;  // Bit 15
  static constexpr uint16_t AS5048A_ERROR_BIT = 0x0100;   // Bit 8
};

} // namespace zeus_hardware_interface

#endif // ZEUS_HARDWARE_INTERFACE__ZEUS_SYSTEM_HPP_
