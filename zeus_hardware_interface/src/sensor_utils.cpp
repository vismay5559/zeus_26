#include "zeus_hardware_interface/sensor_utils.hpp"

#include <chrono>
#include <cstring>
#include <fstream>
#include <fcntl.h>
#include <string>
#include <termios.h>
#include <thread>
#include <unistd.h>

namespace zeus_hardware_interface::sensor_utils
{

namespace
{
// SHTP channel numbers
constexpr uint8_t kBnoChannelControl        = 2;
constexpr uint8_t kBnoChannelInputReports   = 3;
constexpr uint8_t kBnoChannelWakeInputReports = 4;

// SHTP command / report IDs
constexpr uint8_t kSetFeatureCommand        = 0xFD;
constexpr uint8_t kBaseTimestampReport      = 0xFB;
constexpr uint8_t kTimestampRebaseReport    = 0xFA;
constexpr uint8_t kGetFeatureResponse       = 0xFC;
constexpr uint8_t kReportGyroscope          = 0x02;
constexpr uint8_t kReportLinearAcceleration = 0x04;
constexpr uint8_t kReportRotationVector     = 0x05;

// Fixed-point scale factors (matching BNO085 datasheet)
constexpr double kQuatScalar  = 1.0 / 16384.0;  // Q14
constexpr double kAccelScalar = 1.0 / 256.0;    // Q8,  m/s²
constexpr double kGyroScalar  = 1.0 / 512.0;    // Q9,  rad/s

bool write_text_file(const std::string & path, const std::string & value)
{
  std::ofstream stream(path);
  if (!stream.is_open()) return false;
  stream << value;
  return stream.good();
}

void put_u32_le(std::vector<uint8_t> & buf, std::size_t offset, uint32_t value)
{
  buf[offset]     = static_cast<uint8_t>(value & 0xFF);
  buf[offset + 1] = static_cast<uint8_t>((value >>  8) & 0xFF);
  buf[offset + 2] = static_cast<uint8_t>((value >> 16) & 0xFF);
  buf[offset + 3] = static_cast<uint8_t>((value >> 24) & 0xFF);
}

int16_t read_i16_le(const uint8_t * data, std::size_t offset)
{
  return static_cast<int16_t>(
    static_cast<uint16_t>(data[offset]) |
    (static_cast<uint16_t>(data[offset + 1]) << 8));
}

std::size_t bno_report_length(uint8_t id)
{
  switch (id) {
    case kReportGyroscope:
    case kReportLinearAcceleration: return 10;
    case kReportRotationVector:     return 14;
    case kBaseTimestampReport:
    case kTimestampRebaseReport:    return 5;
    case kGetFeatureResponse:       return 17;
    default:                        return 0;
  }
}

// Map a numeric baud rate to the POSIX B* constant.
speed_t to_posix_baud(uint32_t baud)
{
  switch (baud) {
    case 9600:    return B9600;
    case 19200:   return B19200;
    case 38400:   return B38400;
    case 57600:   return B57600;
    case 115200:  return B115200;
    case 230400:  return B230400;
    case 460800:  return B460800;
    case 500000:  return B500000;
    case 576000:  return B576000;
    case 921600:  return B921600;
    case 1000000: return B1000000;
    case 1152000: return B1152000;
    case 1500000: return B1500000;
    case 2000000: return B2000000;
    case 2500000: return B2500000;
    case 3000000: return B3000000;
    case 3500000: return B3500000;
    case 4000000: return B4000000;
    default:      return B3000000;
  }
}
}  // namespace

// ---------------------------------------------------------------------------
// BNO085UartReader — SHTP framing over hardware UART
// ---------------------------------------------------------------------------

bool BNO085UartReader::open_device(
  const std::string & device, uint32_t baud_rate,
  int reset_gpio, uint32_t report_interval_us)
{
  close_device();
  reset_gpio_ = reset_gpio;
  sequence_numbers_.fill(0);
  rx_buf_.clear();

  if (!configure_uart(device, baud_rate)) {
    return false;
  }

  if (reset_gpio_ >= 0) {
    configure_gpio(reset_gpio_, "out");
    hard_reset();
  }

  // Let the BNO085 send its startup packets, then discard them.
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  pump_rx();
  rx_buf_.clear();

  enable_report(kReportRotationVector,     report_interval_us);
  enable_report(kReportLinearAcceleration, report_interval_us);
  enable_report(kReportGyroscope,          report_interval_us);
  return true;
}

void BNO085UartReader::close_device()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool BNO085UartReader::is_open() const { return fd_ >= 0; }

bool BNO085UartReader::configure_uart(const std::string & device, uint32_t baud_rate)
{
  fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) return false;

  struct termios tio;
  if (tcgetattr(fd_, &tio) < 0) { close_device(); return false; }

  cfmakeraw(&tio);
  const speed_t spd = to_posix_baud(baud_rate);
  cfsetispeed(&tio, spd);
  cfsetospeed(&tio, spd);
  tio.c_cflag |=  (CLOCAL | CREAD);
  tio.c_cflag &= ~CRTSCTS;

  if (tcsetattr(fd_, TCSANOW, &tio) < 0) { close_device(); return false; }
  tcflush(fd_, TCIOFLUSH);
  return true;
}

bool BNO085UartReader::configure_gpio(int gpio, const std::string & direction)
{
  const std::string pin = std::to_string(gpio);
  write_text_file("/sys/class/gpio/export", pin);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  return write_text_file("/sys/class/gpio/gpio" + pin + "/direction", direction);
}

bool BNO085UartReader::write_gpio(int gpio, int value)
{
  return write_text_file(
    "/sys/class/gpio/gpio" + std::to_string(gpio) + "/value",
    std::to_string(value));
}

void BNO085UartReader::hard_reset()
{
  write_gpio(reset_gpio_, 0);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  write_gpio(reset_gpio_, 1);
  // BNO085 datasheet: allow 300 ms for firmware boot after reset
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
}

bool BNO085UartReader::send_shtp_packet(uint8_t channel, const std::vector<uint8_t> & payload)
{
  if (fd_ < 0 || channel >= sequence_numbers_.size()) return false;

  const uint16_t len = static_cast<uint16_t>(payload.size() + 4);
  std::vector<uint8_t> frame;
  frame.reserve(len);
  frame.push_back(static_cast<uint8_t>(len & 0xFF));
  frame.push_back(static_cast<uint8_t>((len >> 8) & 0x7F));
  frame.push_back(channel);
  frame.push_back(sequence_numbers_[channel]++);
  frame.insert(frame.end(), payload.begin(), payload.end());

  const ssize_t written = ::write(fd_, frame.data(), frame.size());
  return written == static_cast<ssize_t>(frame.size());
}

void BNO085UartReader::enable_report(uint8_t report_id, uint32_t report_interval_us)
{
  std::vector<uint8_t> payload(17, 0);
  payload[0] = kSetFeatureCommand;
  payload[1] = report_id;
  put_u32_le(payload, 5, report_interval_us);
  send_shtp_packet(kBnoChannelControl, payload);
}

// Drain whatever bytes are currently waiting in the UART hardware FIFO.
bool BNO085UartReader::pump_rx()
{
  uint8_t tmp[256];
  bool got_any = false;
  ssize_t n;
  while ((n = ::read(fd_, tmp, sizeof(tmp))) > 0) {
    rx_buf_.insert(rx_buf_.end(), tmp, tmp + n);
    got_any = true;
  }
  return got_any;
}

// If rx_buf_ contains a complete SHTP packet, pop it into 'packet'.
// On framing error, discard one byte and return false so the caller
// can call pump_rx() again on the next cycle.
bool BNO085UartReader::extract_packet(std::vector<uint8_t> & packet)
{
  if (rx_buf_.size() < 4) return false;

  const uint16_t pkt_len =
    static_cast<uint16_t>(rx_buf_[0]) |
    (static_cast<uint16_t>(rx_buf_[1] & 0x7F) << 8);

  if (pkt_len < 4 || pkt_len > 512) {
    rx_buf_.erase(rx_buf_.begin());  // discard one byte and resync
    return false;
  }

  if (rx_buf_.size() < pkt_len) return false;

  packet.assign(rx_buf_.begin(), rx_buf_.begin() + pkt_len);
  rx_buf_.erase(rx_buf_.begin(), rx_buf_.begin() + pkt_len);
  return true;
}

bool BNO085UartReader::read_sample(BNO085Sample & sample)
{
  if (!is_open()) return false;
  pump_rx();

  bool updated = false;
  std::vector<uint8_t> packet;
  while (extract_packet(packet)) {
    updated = parse_packet(packet, sample) || updated;
  }
  return updated;
}

bool BNO085UartReader::parse_packet(const std::vector<uint8_t> & packet, BNO085Sample & sample)
{
  if (packet.size() < 5) return false;

  const uint16_t pkt_len =
    static_cast<uint16_t>(packet[0]) |
    (static_cast<uint16_t>(packet[1] & 0x7F) << 8);
  if (pkt_len < 4 || pkt_len > packet.size()) return false;

  const uint8_t channel = packet[2];
  if (channel != kBnoChannelInputReports &&
      channel != kBnoChannelWakeInputReports &&
      channel != kBnoChannelControl) {
    return false;
  }

  bool updated = false;
  std::size_t offset = 4;
  while (offset < pkt_len) {
    const uint8_t report_id  = packet[offset];
    const std::size_t rlen   = bno_report_length(report_id);
    if (rlen == 0 || offset + rlen > pkt_len) return updated;
    updated = parse_report(packet.data() + offset, rlen, sample) || updated;
    offset += rlen;
  }
  return updated;
}

bool BNO085UartReader::parse_report(
  const uint8_t * report, std::size_t report_size, BNO085Sample & sample)
{
  if (report_size < 10) return false;

  switch (report[0]) {
    case kReportRotationVector:
      if (report_size >= 12) {
        sample.orientation_xyzw[0] = read_i16_le(report,  4) * kQuatScalar;
        sample.orientation_xyzw[1] = read_i16_le(report,  6) * kQuatScalar;
        sample.orientation_xyzw[2] = read_i16_le(report,  8) * kQuatScalar;
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

}  // namespace zeus_hardware_interface::sensor_utils
