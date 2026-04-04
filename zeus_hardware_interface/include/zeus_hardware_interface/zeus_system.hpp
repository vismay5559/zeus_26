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
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  bool open_spi_port();
  void close_spi_port();
  bool read_spi_encoder_positions(std::vector<double> & positions);
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
  std::string spi_device_ = "/dev/spidev0.0";
  int spi_fd_ = -1;
  uint32_t spi_speed_hz_ = 1000000;
  uint8_t spi_mode_ = 1;
  uint8_t spi_bits_per_word_ = 8;
  uint8_t spi_word_size_bytes_ = 2;
  uint8_t spi_angle_bits_ = 14;
  uint8_t spi_angle_lsb_shift_ = 0;
  uint16_t spi_tx_word_ = 0x0000;
  bool spi_reverse_encoder_order_ = false;

  static constexpr double LOW_PASS_ALPHA = 0.2;
};

} // namespace zeus_hardware_interface

#endif // ZEUS_HARDWARE_INTERFACE__ZEUS_SYSTEM_HPP_
