#include "zeus_hardware_interface/encoder_utils.hpp"

#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace zeus_hardware_interface::encoder_utils
{

namespace
{
constexpr double kTwoPi = 6.28318530717958647692;
constexpr double kAS5048AAngleMax = 8192.0;  // 2^13
}  // namespace

int open_as5048a_spi(const std::string & device, uint8_t spi_mode, uint32_t speed_hz)
{
  int fd = open(device.c_str(), O_RDWR);
  if (fd < 0) {
    return -1;
  }

  // Configure SPI mode
  if (ioctl(fd, SPI_IOC_WR_MODE, &spi_mode) < 0 ||
      ioctl(fd, SPI_IOC_RD_MODE, &spi_mode) < 0) {
    close(fd);
    return -1;
  }

  // Configure bits per word (8 bits)
  uint8_t bits_per_word = 8;
  if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word) < 0 ||
      ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits_per_word) < 0) {
    close(fd);
    return -1;
  }

  // Configure SPI clock speed
  if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz) < 0 ||
      ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed_hz) < 0) {
    close(fd);
    return -1;
  }

  return fd;
}

void close_as5048a_spi(int fd)
{
  if (fd >= 0) {
    close(fd);
  }
}

bool read_as5048a_daisy_chain(
  int fd, 
  std::size_t num_encoders, 
  std::vector<uint8_t> & tx_buf, 
  std::vector<uint8_t> & rx_buf, 
  std::vector<uint16_t> & raw_words)
{
  if (fd < 0 || num_encoders == 0) {
    return false;
  }

  const std::size_t transfer_size = num_encoders * 2;

  struct spi_ioc_transfer transfer;
  std::memset(&transfer, 0, sizeof(transfer));
  transfer.tx_buf = reinterpret_cast<unsigned long>(tx_buf.data());
  transfer.rx_buf = reinterpret_cast<unsigned long>(rx_buf.data());
  transfer.len = static_cast<uint32_t>(transfer_size);
  transfer.bits_per_word = 8;

  if (ioctl(fd, SPI_IOC_MESSAGE(1), &transfer) < 0) {
    return false;
  }

  raw_words.assign(num_encoders, 0);
  for (std::size_t i = 0; i < num_encoders; ++i) {
    const std::size_t offset = i * 2;
    raw_words[i] = (static_cast<uint16_t>(rx_buf[offset]) << 8) | rx_buf[offset + 1];
  }

  return true;
}

double as5048a_raw_to_radians(uint16_t raw_angle)
{
  // Ensure we're only using the 13-bit angle value
  raw_angle &= 0x1FFF;
  // Convert 13-bit count (0-8192) to radians (0-2π)
  return (static_cast<double>(raw_angle) / kAS5048AAngleMax) * kTwoPi;
}

bool validate_as5048a_parity(const AS5048AWord & word)
{
  // AS5048A uses odd parity on all 16 bits
  // If parity is correct, XOR of all bits should result in 1
  uint16_t parity_check = word.raw;
  parity_check ^= parity_check >> 8;   // XOR high and low bytes
  parity_check ^= parity_check >> 4;   // Fold to 4 bits
  parity_check ^= parity_check >> 2;   // Fold to 2 bits
  parity_check ^= parity_check >> 1;   // Fold to 1 bit
  
  // For odd parity, result should be 1
  return (parity_check & 1) == 1;
}

}  // namespace zeus_hardware_interface::encoder_utils
