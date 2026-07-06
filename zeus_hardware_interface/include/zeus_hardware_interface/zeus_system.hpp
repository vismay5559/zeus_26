#ifndef ZEUS_HARDWARE_INTERFACE__ZEUS_SYSTEM_HPP_
#define ZEUS_HARDWARE_INTERFACE__ZEUS_SYSTEM_HPP_

#include <chrono>
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
  void handle_odrive_can_frame(const struct canfd_frame & frame, bool uses_can0);
  void handle_heartbeat_frame(std::size_t joint_index, const struct canfd_frame & frame);
  void check_actuator_faults();
  void reset_actuator_fault_state();
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
  std::vector<double> prev_hw_commands_;   // one cycle old, for velocity feedforward
  std::vector<double> odrive_load_encoder_positions_;
  std::vector<double> odrive_torque_estimates_;
  std::vector<double> current_interpolated_targets_;

  // --- Actuator fault tracking (heartbeat parsing + CAN staleness + TX failure watchdog) ---
  std::vector<double> odrive_axis_error_;         // decoded from Heartbeat (0x001), raw ODrive error bitmask
                                                   // (internal only — not exported; folded into the
                                                   // kFaultBitAxisError bit of actuator_fault, full value logged)
  std::vector<double> odrive_axis_state_;         // decoded from Heartbeat (0x001), ODrive axis state enum
  std::vector<double> joint_fault_states_;        // exported "actuator_fault" state interface: reason bitmask,
                                                   // 0.0 = healthy, see kFaultBit* below for bit meanings
  std::vector<bool> joint_command_gated_;         // true -> write() withholds new position commands this joint
  std::vector<uint32_t> tx_consecutive_failures_; // consecutive failed send_position_target() calls
  std::vector<std::chrono::steady_clock::time_point> last_heartbeat_rx_;
  std::vector<std::chrono::steady_clock::time_point> last_encoder_rx_;
  std::vector<std::chrono::steady_clock::time_point> last_torque_rx_;

  // Whole-robot latch: set the instant ANY joint reports a nonzero actuator_fault.
  // Once true, write() withholds position commands to ALL joints, not just the faulted
  // one. Deliberately does not auto-clear — a fault serious enough to warrant a full
  // stop should not silently resume unsupervised. Cleared only by re-activating the
  // hardware component (on_activate()), i.e. an operator-driven restart.
  bool global_estop_latched_ = false;
  std::vector<double> spi_positions_buffer_;
  std::vector<uint8_t> spi_tx_buffer_;  // pre-allocated encoder SPI TX frame (0xFF × 2N bytes)
  std::vector<uint8_t> spi_rx_buffer_;  // pre-allocated encoder SPI RX frame
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
  sensor_utils::BNO085UartReader imu_reader_;

  // BNO085 UART configuration (GPIO14/15 → /dev/ttyAMA0, 3 Mbaud)
  std::string imu_uart_device_ = "/dev/ttyAMA0";
  uint32_t imu_uart_baud_rate_ = 3000000;
  int imu_reset_gpio_ = -1;
  uint32_t imu_report_interval_us_ = 2500;

  // AS5048A encoder daisy-chain — SPI3 (/dev/spidev3.0, CAN-FD HAT owns SPI0+SPI1)
  std::string encoder_spi_device_ = "/dev/spidev3.0";
  int encoder_spi_fd_ = -1;
  uint32_t encoder_spi_speed_hz_ = 1000000;
  uint8_t encoder_spi_mode_ = 1;
  uint8_t encoder_spi_bits_per_word_ = 8;
  std::size_t num_daisy_encoders_ = 4;   // 4 SEA joints: left/right hip_pitch and knee_pitch
  std::vector<std::size_t> encoder_joint_map_;

  static constexpr double LOW_PASS_ALPHA = 0.2;
  static constexpr std::size_t AS5048A_WORD_SIZE_BYTES = 2;
  static constexpr std::size_t AS5048A_ANGLE_BITS = 13;
  static constexpr uint16_t AS5048A_ANGLE_MASK = 0x1FFF;
  static constexpr uint16_t AS5048A_PARITY_BIT = 0x8000;
  static constexpr uint16_t AS5048A_ERROR_BIT  = 0x0100;

  // Each ODrive must be configured (via odrivetool) to send Get_Heartbeat every 10 ms:
  //   odrv0.axis0.config.can.heartbeat_rate_ms = 10
  // Get_Encoder_Estimates / Get_Torques broadcast at 1 kHz per the existing ODrive CAN config.
  // Timeouts below allow a few missed broadcasts (CAN/SPI scheduling jitter) before latching a fault.
  static constexpr double kHeartbeatTimeoutMs = 30.0;
  static constexpr double kCanRxTimeoutMs = 5.0;
  static constexpr uint32_t kTxFailThreshold = 3;

  // Bit meanings for the "actuator_fault" state interface reason bitmask.
  static constexpr uint32_t kFaultBitHeartbeatStale = 0x1;
  static constexpr uint32_t kFaultBitTelemetryStale = 0x2;
  static constexpr uint32_t kFaultBitAxisError      = 0x4;
  static constexpr uint32_t kFaultBitTxFailure       = 0x8;
};

} // namespace zeus_hardware_interface

#endif // ZEUS_HARDWARE_INTERFACE__ZEUS_SYSTEM_HPP_
