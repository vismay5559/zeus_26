#ifndef ZEUS_HARDWARE_INTERFACE__SENSOR_UTILS_HPP_
#define ZEUS_HARDWARE_INTERFACE__SENSOR_UTILS_HPP_

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace zeus_hardware_interface::sensor_utils
{

struct BNO085Sample
{
  std::array<double, 4> orientation_xyzw{0.0, 0.0, 0.0, 1.0};
  std::array<double, 3> linear_acceleration{0.0, 0.0, 0.0};
  std::array<double, 3> angular_velocity{0.0, 0.0, 0.0};
  bool has_orientation = false;
  bool has_linear_acceleration = false;
  bool has_angular_velocity = false;
};

// BNO085 driver using SHTP over hardware UART (GPIO14/15 on RPi 5).
// The SHTP packet framing is identical to the SPI variant; only the
// transport layer changes: write() for TX, non-blocking read() for RX,
// and a streaming accumulation buffer to handle partial receives.
class BNO085UartReader
{
public:
  // Open /dev/ttyAMAx at baud_rate, optionally hard-reset via reset_gpio,
  // then enable rotation-vector, linear-acceleration, and gyroscope reports
  // at the requested report_interval_us.
  bool open_device(
    const std::string & device, uint32_t baud_rate,
    int reset_gpio, uint32_t report_interval_us);
  void close_device();
  bool is_open() const;

  // Drain the UART FIFO and parse any complete SHTP packets into sample.
  // Returns true if at least one sensor value was updated.
  bool read_sample(BNO085Sample & sample);

private:
  bool configure_uart(const std::string & device, uint32_t baud_rate);
  bool configure_gpio(int gpio, const std::string & direction);
  bool write_gpio(int gpio, int value);
  void hard_reset();
  bool send_shtp_packet(uint8_t channel, const std::vector<uint8_t> & payload);
  void enable_report(uint8_t report_id, uint32_t report_interval_us);
  bool pump_rx();
  bool extract_packet(std::vector<uint8_t> & packet);
  bool parse_packet(const std::vector<uint8_t> & packet, BNO085Sample & sample);
  bool parse_report(const uint8_t * report, std::size_t report_size, BNO085Sample & sample);

  int fd_ = -1;
  int reset_gpio_ = -1;
  std::array<uint8_t, 6> sequence_numbers_{0, 0, 0, 0, 0, 0};
  std::vector<uint8_t> rx_buf_;  // accumulation buffer for partial UART packets
};

}  // namespace zeus_hardware_interface::sensor_utils

#endif  // ZEUS_HARDWARE_INTERFACE__SENSOR_UTILS_HPP_
