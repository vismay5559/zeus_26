#include "zeus_hardware_interface/sensor_utils.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <fstream>
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <string>
#include <sys/ioctl.h>
#include <thread>
#include <unistd.h>

namespace zeus_hardware_interface::sensor_utils
{

namespace
{
constexpr uint8_t kBnoChannelControl = 2;
constexpr uint8_t kBnoChannelInputReports = 3;
constexpr uint8_t kBnoChannelWakeInputReports = 4;

constexpr uint8_t kSetFeatureCommand = 0xFD;
constexpr uint8_t kBaseTimestampReport = 0xFB;
constexpr uint8_t kTimestampRebaseReport = 0xFA;
constexpr uint8_t kGetFeatureResponse = 0xFC;

constexpr uint8_t kReportGyroscope = 0x02;
constexpr uint8_t kReportLinearAcceleration = 0x04;
constexpr uint8_t kReportRotationVector = 0x05;

constexpr double kQuatScalar = 1.0 / 16384.0;  // Q14
constexpr double kAccelScalar = 1.0 / 256.0;   // Q8, m/s^2
constexpr double kGyroScalar = 1.0 / 512.0;    // Q9, rad/s

bool write_text_file(const std::string & path, const std::string & value)
{
  std::ofstream stream(path);
  if (!stream.is_open()) {
    return false;
  }
  stream << value;
  return stream.good();
}

bool read_text_file(const std::string & path, std::string & value)
{
  std::ifstream stream(path);
  if (!stream.is_open()) {
    return false;
  }
  stream >> value;
  return stream.good();
}

void put_u32_le(std::vector<uint8_t> & buffer, std::size_t offset, uint32_t value)
{
  buffer[offset] = static_cast<uint8_t>(value & 0xFF);
  buffer[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
  buffer[offset + 2] = static_cast<uint8_t>((value >> 16) & 0xFF);
  buffer[offset + 3] = static_cast<uint8_t>((value >> 24) & 0xFF);
}

int16_t read_i16_le(const uint8_t * data, std::size_t offset)
{
  const uint16_t raw =
    static_cast<uint16_t>(data[offset]) |
    (static_cast<uint16_t>(data[offset + 1]) << 8);
  return static_cast<int16_t>(raw);
}

std::size_t bno_report_length(uint8_t report_id)
{
  switch (report_id) {
    case kReportGyroscope:
    case kReportLinearAcceleration:
      return 10;
    case kReportRotationVector:
      return 14;
    case kBaseTimestampReport:
    case kTimestampRebaseReport:
      return 5;
    case kGetFeatureResponse:
      return 17;
    default:
      return 0;
  }
}

bool configure_spi_fd(int fd, uint8_t spi_mode, uint8_t bits_per_word, uint32_t speed_hz)
{
  return ioctl(fd, SPI_IOC_WR_MODE, &spi_mode) >= 0 &&
         ioctl(fd, SPI_IOC_RD_MODE, &spi_mode) >= 0 &&
         ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word) >= 0 &&
         ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits_per_word) >= 0 &&
         ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz) >= 0 &&
         ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed_hz) >= 0;
}
}  // namespace

bool BNO085SpiReader::open_device(
  const std::string & device, uint32_t speed_hz, int interrupt_gpio,
  int reset_gpio, uint32_t report_interval_us)
{
  close_device();
  interrupt_gpio_ = interrupt_gpio;
  reset_gpio_ = reset_gpio;
  speed_hz_ = speed_hz;
  sequence_numbers_.fill(0);

  if (!configure_spi(device, SPI_MODE_3, speed_hz_)) {
    close_device();
    return false;
  }

  if (interrupt_gpio_ >= 0 && !configure_gpio(interrupt_gpio_, "in")) {
    close_device();
    return false;
  }

  if (reset_gpio_ >= 0 && !configure_gpio(reset_gpio_, "out")) {
    close_device();
    return false;
  }

  hard_reset();

  std::vector<uint8_t> packet;
  for (int i = 0; i < 3; ++i) {
    read_packet(packet);
  }

  enable_report(kReportRotationVector, report_interval_us);
  enable_report(kReportLinearAcceleration, report_interval_us);
  enable_report(kReportGyroscope, report_interval_us);
  return true;
}

void BNO085SpiReader::close_device()
{
  if (fd_ >= 0) {
    close(fd_);
    fd_ = -1;
  }
}

bool BNO085SpiReader::is_open() const
{
  return fd_ >= 0;
}

bool BNO085SpiReader::configure_spi(
  const std::string & device, uint8_t spi_mode, uint32_t speed_hz)
{
  fd_ = open(device.c_str(), O_RDWR);
  if (fd_ < 0) {
    return false;
  }

  const uint8_t bits_per_word = 8;
  return configure_spi_fd(fd_, spi_mode, bits_per_word, speed_hz);
}

bool BNO085SpiReader::configure_gpio(int gpio, const std::string & direction)
{
  const std::string gpio_number = std::to_string(gpio);
  write_text_file("/sys/class/gpio/export", gpio_number);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  return write_text_file("/sys/class/gpio/gpio" + gpio_number + "/direction", direction);
}

bool BNO085SpiReader::write_gpio(int gpio, int value)
{
  return write_text_file(
    "/sys/class/gpio/gpio" + std::to_string(gpio) + "/value", std::to_string(value));
}

bool BNO085SpiReader::read_gpio(int gpio, int & value) const
{
  std::string text;
  if (!read_text_file("/sys/class/gpio/gpio" + std::to_string(gpio) + "/value", text)) {
    return false;
  }
  value = text == "0" ? 0 : 1;
  return true;
}

bool BNO085SpiReader::data_ready() const
{
  if (interrupt_gpio_ < 0) {
    return true;
  }

  int value = 1;
  return read_gpio(interrupt_gpio_, value) && value == 0;
}

bool BNO085SpiReader::wait_for_interrupt(int timeout_ms) const
{
  if (interrupt_gpio_ < 0) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    return true;
  }

  const auto start = std::chrono::steady_clock::now();
  while (std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - start).count() < timeout_ms) {
    if (data_ready()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return false;
}

bool BNO085SpiReader::transfer(const std::vector<uint8_t> & tx, std::vector<uint8_t> & rx)
{
  if (fd_ < 0 || tx.empty()) {
    return false;
  }

  rx.assign(tx.size(), 0);
  struct spi_ioc_transfer transfer;
  std::memset(&transfer, 0, sizeof(transfer));
  transfer.tx_buf = reinterpret_cast<unsigned long>(tx.data());
  transfer.rx_buf = reinterpret_cast<unsigned long>(rx.data());
  transfer.len = static_cast<uint32_t>(tx.size());
  transfer.speed_hz = speed_hz_;
  transfer.bits_per_word = 8;

  return ioctl(fd_, SPI_IOC_MESSAGE(1), &transfer) >= 0;
}

bool BNO085SpiReader::read_packet(std::vector<uint8_t> & packet)
{
  if (!wait_for_interrupt(1)) {
    return false;
  }

  std::vector<uint8_t> header_tx(4, 0x00);
  std::vector<uint8_t> header_rx;
  if (!transfer(header_tx, header_rx) || header_rx.size() < 4) {
    return false;
  }

  const uint16_t packet_length =
    static_cast<uint16_t>(header_rx[0]) |
    (static_cast<uint16_t>(header_rx[1] & 0x7F) << 8);
  if (packet_length < 4 || packet_length > 512) {
    return false;
  }

  std::vector<uint8_t> packet_tx(packet_length, 0x00);
  if (!wait_for_interrupt(1) || !transfer(packet_tx, packet)) {
    return false;
  }

  return packet.size() >= packet_length;
}

bool BNO085SpiReader::send_packet(uint8_t channel, const std::vector<uint8_t> & payload)
{
  if (channel >= sequence_numbers_.size()) {
    return false;
  }

  const uint16_t packet_length = static_cast<uint16_t>(payload.size() + 4);
  std::vector<uint8_t> tx(packet_length, 0);
  tx[0] = static_cast<uint8_t>(packet_length & 0xFF);
  tx[1] = static_cast<uint8_t>((packet_length >> 8) & 0x7F);
  tx[2] = channel;
  tx[3] = sequence_numbers_[channel];
  std::copy(payload.begin(), payload.end(), tx.begin() + 4);

  std::vector<uint8_t> rx;
  if (!wait_for_interrupt(3) || !transfer(tx, rx)) {
    return false;
  }

  sequence_numbers_[channel] = static_cast<uint8_t>(sequence_numbers_[channel] + 1);
  return true;
}

void BNO085SpiReader::hard_reset()
{
  if (reset_gpio_ < 0) {
    return;
  }

  write_gpio(reset_gpio_, 1);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  write_gpio(reset_gpio_, 0);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  write_gpio(reset_gpio_, 1);
  wait_for_interrupt(3000);
}

void BNO085SpiReader::enable_report(uint8_t report_id, uint32_t report_interval_us)
{
  std::vector<uint8_t> payload(17, 0);
  payload[0] = kSetFeatureCommand;
  payload[1] = report_id;
  put_u32_le(payload, 5, report_interval_us);
  send_packet(kBnoChannelControl, payload);
}

bool BNO085SpiReader::read_sample(BNO085Sample & sample)
{
  if (!is_open() || !data_ready()) {
    return false;
  }

  bool updated = false;
  std::vector<uint8_t> packet;
  for (int i = 0; i < 8 && data_ready(); ++i) {
    if (!read_packet(packet)) {
      break;
    }
    updated = parse_packet(packet, sample) || updated;
  }

  return updated;
}

bool BNO085SpiReader::parse_packet(const std::vector<uint8_t> & packet, BNO085Sample & sample)
{
  if (packet.size() < 5) {
    return false;
  }

  const uint16_t packet_length =
    static_cast<uint16_t>(packet[0]) |
    (static_cast<uint16_t>(packet[1] & 0x7F) << 8);
  if (packet_length < 4 || packet_length > packet.size()) {
    return false;
  }

  const uint8_t channel = packet[2];
  if (channel != kBnoChannelInputReports && channel != kBnoChannelWakeInputReports &&
      channel != kBnoChannelControl) {
    return false;
  }

  bool updated = false;
  std::size_t offset = 4;
  while (offset < packet_length) {
    const uint8_t report_id = packet[offset];
    const std::size_t report_size = bno_report_length(report_id);
    if (report_size == 0 || offset + report_size > packet_length) {
      return updated;
    }
    updated = parse_report(packet.data() + offset, report_size, sample) || updated;
    offset += report_size;
  }

  return updated;
}

bool BNO085SpiReader::parse_report(
  const uint8_t * report, std::size_t report_size, BNO085Sample & sample)
{
  if (report_size < 10) {
    return false;
  }

  switch (report[0]) {
    case kReportRotationVector:
      if (report_size >= 12) {
        sample.orientation_xyzw[0] = read_i16_le(report, 4) * kQuatScalar;
        sample.orientation_xyzw[1] = read_i16_le(report, 6) * kQuatScalar;
        sample.orientation_xyzw[2] = read_i16_le(report, 8) * kQuatScalar;
        sample.orientation_xyzw[3] = read_i16_le(report, 10) * kQuatScalar;
        sample.has_orientation = true;
        return true;
      }
      break;
    case kReportLinearAcceleration:
      sample.linear_acceleration[0] = read_i16_le(report, 4) * kAccelScalar;
      sample.linear_acceleration[1] = read_i16_le(report, 6) * kAccelScalar;
      sample.linear_acceleration[2] = read_i16_le(report, 8) * kAccelScalar;
      sample.has_linear_acceleration = true;
      return true;
    case kReportGyroscope:
      sample.angular_velocity[0] = read_i16_le(report, 4) * kGyroScalar;
      sample.angular_velocity[1] = read_i16_le(report, 6) * kGyroScalar;
      sample.angular_velocity[2] = read_i16_le(report, 8) * kGyroScalar;
      sample.has_angular_velocity = true;
      return true;
    default:
      break;
  }

  return false;
}

bool MCP3008Reader::open_device(const std::string & device, uint32_t speed_hz)
{
  close_device();
  fd_ = open(device.c_str(), O_RDWR);
  if (fd_ < 0) {
    return false;
  }

  speed_hz_ = speed_hz;
  const uint8_t mode = SPI_MODE_0;
  const uint8_t bits_per_word = 8;
  if (!configure_spi_fd(fd_, mode, bits_per_word, speed_hz_)) {
    close_device();
    return false;
  }

  return true;
}

void MCP3008Reader::close_device()
{
  if (fd_ >= 0) {
    close(fd_);
    fd_ = -1;
  }
}

bool MCP3008Reader::is_open() const
{
  return fd_ >= 0;
}

bool MCP3008Reader::read_channel(uint8_t channel, uint16_t & value)
{
  if (fd_ < 0 || channel > 7) {
    return false;
  }

  std::array<uint8_t, 3> tx{0x01, static_cast<uint8_t>((0x08 | channel) << 4), 0x00};
  std::array<uint8_t, 3> rx{0x00, 0x00, 0x00};
  struct spi_ioc_transfer transfer;
  std::memset(&transfer, 0, sizeof(transfer));
  transfer.tx_buf = reinterpret_cast<unsigned long>(tx.data());
  transfer.rx_buf = reinterpret_cast<unsigned long>(rx.data());
  transfer.len = static_cast<uint32_t>(tx.size());
  transfer.speed_hz = speed_hz_;
  transfer.bits_per_word = 8;

  if (ioctl(fd_, SPI_IOC_MESSAGE(1), &transfer) < 0) {
    return false;
  }

  value = static_cast<uint16_t>(((rx[1] & 0x03) << 8) | rx[2]);
  return true;
}

}  // namespace zeus_hardware_interface::sensor_utils
