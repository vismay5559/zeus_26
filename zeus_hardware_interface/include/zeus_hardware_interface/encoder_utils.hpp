#ifndef ZEUS_HARDWARE_INTERFACE__ENCODER_UTILS_HPP_
#define ZEUS_HARDWARE_INTERFACE__ENCODER_UTILS_HPP_

#include <cstdint>
#include <vector>
#include <string>

namespace zeus_hardware_interface::encoder_utils
{

/**
 * @brief AS5048A 16-bit word structure
 * Bit 15: Parity (odd)
 * Bits 14-13: Mag_too_strong, Mag_too_weak
 * Bits [12:0]: 13-bit angle (0–8192 = 0–360°)
 * Bit 8: Error flag
 */
struct AS5048AWord
{
  uint16_t raw = 0;

  /// Extract 13-bit angle value (bits 12:0)
  uint16_t angle() const { return raw & 0x1FFF; }

  /// Check parity bit (bit 15)
  bool parity() const { return (raw & 0x8000) != 0; }

  /// Check error flag (bit 8)
  bool error() const { return (raw & 0x0100) != 0; }

  /// Check magnetic field warning (bits 14:13)
  bool mag_too_strong() const { return (raw & 0x4000) != 0; }
  bool mag_too_weak() const { return (raw & 0x2000) != 0; }
};

/**
 * @brief Open SPI device for AS5048A encoder daisy-chain
 * @param device Path to SPI device (e.g., "/dev/spidev0.1")
 * @param spi_mode SPI mode (0-3, typically 1 for AS5048A)
 * @param speed_hz SPI clock speed in Hz
 * @return File descriptor on success, negative value on failure
 */
int open_as5048a_spi(const std::string & device, uint8_t spi_mode, uint32_t speed_hz);

/**
 * @brief Close SPI device
 * @param fd File descriptor returned by open_as5048a_spi()
 */
void close_as5048a_spi(int fd);

/**
 * @brief Read all encoders from AS5048A daisy-chain in one SPI burst
 * 
 * @param fd SPI file descriptor
 * @param num_encoders Number of encoders in the chain (typically 10)
 * @param raw_words Output vector of raw 16-bit words (size = num_encoders)
 * @return true on success, false on SPI error
 * 
 * Protocol:
 * - Hold CS low, clock out 2*num_encoders bytes on MOSI (dummy data)
 * - Simultaneous clock in 2*num_encoders bytes on MISO (encoder positions)
 * - CS returns high, latching all position reads
 */
/**
 * @brief Read all encoders from AS5048A daisy-chain in one SPI burst (Zero Allocation)
 */
bool read_as5048a_daisy_chain(
  int fd, 
  std::size_t num_encoders, 
  std::vector<uint8_t> & tx_buf, 
  std::vector<uint8_t> & rx_buf, 
  std::vector<uint16_t> & raw_words);

/**
 * @brief Convert AS5048A raw word to radians
 * 
 * @param raw_word 13-bit angle value (0–8192)
 * @return Angle in radians (0 to 2π)
 */
double as5048a_raw_to_radians(uint16_t raw_angle);

/**
 * @brief Validate parity of AS5048A word (odd parity)
 * 
 * @param word AS5048AWord structure
 * @return true if parity is correct, false otherwise
 */
bool validate_as5048a_parity(const AS5048AWord & word);

}  // namespace zeus_hardware_interface::encoder_utils

#endif  // ZEUS_HARDWARE_INTERFACE__ENCODER_UTILS_HPP_
