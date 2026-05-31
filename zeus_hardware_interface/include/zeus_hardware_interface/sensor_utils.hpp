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

class BNO085SpiReader
{
public:
  bool open_device(
    const std::string & device, uint32_t speed_hz, int interrupt_gpio,
    int reset_gpio, uint32_t report_interval_us);
  void close_device();
  bool is_open() const;
  bool read_sample(BNO085Sample & sample);

private:
  bool configure_spi(const std::string & device, uint8_t spi_mode, uint32_t speed_hz);
  bool configure_gpio(int gpio, const std::string & direction);
  bool write_gpio(int gpio, int value);
  bool read_gpio(int gpio, int & value) const;
  bool data_ready() const;
  bool wait_for_interrupt(int timeout_ms) const;
  bool transfer(const std::vector<uint8_t> & tx, std::vector<uint8_t> & rx);
  bool read_packet(std::vector<uint8_t> & packet);
  bool send_packet(uint8_t channel, const std::vector<uint8_t> & payload);
  void hard_reset();
  void enable_report(uint8_t report_id, uint32_t report_interval_us);
  bool parse_packet(const std::vector<uint8_t> & packet, BNO085Sample & sample);
  bool parse_report(const uint8_t * report, std::size_t report_size, BNO085Sample & sample);

  int fd_ = -1;
  uint32_t speed_hz_ = 1000000;
  int interrupt_gpio_ = -1;
  int reset_gpio_ = -1;
  std::array<uint8_t, 6> sequence_numbers_{0, 0, 0, 0, 0, 0};
};

class MCP3008Reader
{
public:
  bool open_device(const std::string & device, uint32_t speed_hz);
  void close_device();
  bool is_open() const;
  bool read_channel(uint8_t channel, uint16_t & value);

private:
  int fd_ = -1;
  uint32_t speed_hz_ = 1000000;
};

}  // namespace zeus_hardware_interface::sensor_utils

#endif  // ZEUS_HARDWARE_INTERFACE__SENSOR_UTILS_HPP_
