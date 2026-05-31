#ifndef ZEUS_HARDWARE_INTERFACE__ZEUS_SYSTEM_HPP_
#define ZEUS_HARDWARE_INTERFACE__ZEUS_SYSTEM_HPP_

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "zeus_can_interface/socketcan.hpp"
#include "zeus_hardware_interface/sensor_utils.hpp"

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
  struct SensorStateIndex
  {
    std::string sensor_name;
    std::string interface_name;
    std::size_t value_index = 0;
  };

  // Mechanical switch connected directly to a Raspberry Pi GPIO pin (Linux sysfs interface).
  struct GpioSwitch
  {
    std::string sensor_name;
    int gpio_pin = -1;
    bool active_low = false;  // true → LOW signal means contact detected
  };

  void close_encoder_spi();
  bool read_as5048a_encoder_chain(std::vector<double> & positions);
  bool use_can0_for_joint(std::size_t joint_index) const;
  uint32_t node_id_for_joint(std::size_t joint_index) const;
  void read_odrive_can_telemetry();
  void drain_odrive_can_telemetry(zeus_can_interface::SocketCANTransceiver & can_driver, bool uses_can0);
  void handle_odrive_can_frame(const struct can_frame & frame, bool uses_can0);
  std::size_t joint_index_for_can_node(uint32_t node_id, bool uses_can0) const;
  bool has_sensor_state(const std::string & sensor_name, const std::string & interface_name) const;
  void set_sensor_state(
    const std::string & sensor_name, const std::string & interface_name, double value);
  void apply_imu_sample(const sensor_utils::BNO085Sample & sample);
  void read_gpio_switches();

  zeus_can_interface::SocketCANTransceiver can0_driver_;
  zeus_can_interface::SocketCANTransceiver can1_driver_;

  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;
  std::vector<double> odrive_load_encoder_positions_;
  std::vector<double> odrive_torque_estimates_;
  std::vector<double> current_interpolated_targets_;
  std::vector<double> spi_positions_buffer_;
  std::vector<uint32_t> joint_node_ids_;
  std::vector<bool> joint_uses_can0_;
  std::vector<double> sensor_states_;
  std::vector<SensorStateIndex> sensor_state_indices_;
  std::vector<GpioSwitch> gpio_switches_;
  // Open file descriptors for /sys/class/gpio/gpioN/value (one per switch, keyed by pin number)
  std::unordered_map<int, int> gpio_fds_;

  std::string can0_name_ = "can0";
  std::string can1_name_ = "can1";
  bool command_only_mode_ = false;
  bool has_imu_sensor_ = false;
  std::string imu_sensor_name_ = "imu";
  sensor_utils::BNO085Sample last_imu_sample_;
  sensor_utils::BNO085SpiReader imu_reader_;

  std::string imu_spi_device_ = "/dev/spidev0.0";
  uint32_t imu_spi_speed_hz_ = 1000000;
  int imu_interrupt_gpio_ = -1;
  int imu_reset_gpio_ = -1;
  uint32_t imu_report_interval_us_ = 2500;

  // AS5048A Encoder daisy-chain SPI configuration
  std::string encoder_spi_device_ = "/dev/spidev0.2";
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
