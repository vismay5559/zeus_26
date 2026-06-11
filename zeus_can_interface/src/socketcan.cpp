#include "zeus_can_interface/socketcan.hpp"
#include <cerrno>
#include <cstring>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <net/if.h>

namespace zeus_can_interface
{

SocketCANTransceiver::SocketCANTransceiver() : socket_fd_(-1), is_open_(false) {}

SocketCANTransceiver::~SocketCANTransceiver()
{
    close_port();
}

bool SocketCANTransceiver::open_port(const std::string& interface_name)
{
    struct sockaddr_can addr;
    struct ifreq ifr;

    // 1. Create the raw CAN socket
    socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0) {
        std::cerr << "Error: Could not open CAN socket." << std::endl;
        return false;
    }

    // 2. Identify the network interface (e.g., "can0")
    strncpy(ifr.ifr_name, interface_name.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';
    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
        std::cerr << "Error: Could not find interface " << interface_name << std::endl;
        close_port();
        return false;
    }

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    // 3. Bind the socket to the interface
    if (bind(socket_fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        std::cerr << "Error: Could not bind CAN socket to " << interface_name << std::endl;
        close_port();
        return false;
    }

    // 4. Enable CAN FD frames so the socket can send/receive canfd_frame at 5 Mbps data phase.
    // Without this, FD frames from the ODrive are silently dropped by the kernel.
    int enable_canfd = 1;
    if (setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd)) < 0) {
        std::cerr << "ERROR: CAN_RAW_FD_FRAMES failed on " << interface_name
                  << ": errno=" << errno << " (" << std::strerror(errno) << ")" << std::endl;
        close_port();
        return false;
    }
    std::cerr << "[socketcan] CAN FD frames enabled on " << interface_name << std::endl;

    // 5. CRITICAL FOR 1 kHz: Set the socket to non-blocking mode.
    // If the bus is full, the write() command will instantly return instead of freezing the loop.
    int flags = fcntl(socket_fd_, F_GETFL, 0);
    fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);

    is_open_ = true;
    return true;
}

void SocketCANTransceiver::close_port()
{
    if (is_open_ && socket_fd_ >= 0) {
        close(socket_fd_);
        socket_fd_ = -1;
        is_open_ = false;
    }
}

// Memory-safe bit packing to avoid Strict Aliasing rule violations
void SocketCANTransceiver::pack_float(float value, uint8_t* buffer, int start_index)
{
    std::memcpy(&buffer[start_index], &value, sizeof(float));
}

void SocketCANTransceiver::pack_int16(int16_t value, uint8_t* buffer, int start_index)
{
    std::memcpy(&buffer[start_index], &value, sizeof(int16_t));
}

bool SocketCANTransceiver::send_position_target(uint32_t node_id, float position, float vel_ff, float torque_ff)
{
    if (!is_open_) return false;

    // TX as classic CAN (1 Mbps). The MCP2518FD SPI driver deadlocks under sustained
    // CAN-FD 5 Mbps TX on Pi due to SPI interrupt storms from retransmissions.
    // Classic CAN is identical in content for 8-byte frames; the ODrive accepts it
    // regardless of CAN-FD mode. RX (ODrive→Pi) still uses CAN-FD via read_frame().
    struct can_frame frame;
    std::memset(&frame, 0, sizeof(frame));
    frame.can_id  = (node_id << 5) | ODESC_CMD_SET_INPUT_POS;
    frame.can_dlc = 8;

    int16_t vel_int    = static_cast<int16_t>(vel_ff    * 1000.0f);
    int16_t torque_int = static_cast<int16_t>(torque_ff * 1000.0f);

    pack_float(position, frame.data, 0);
    pack_int16(vel_int,    frame.data, 4);
    pack_int16(torque_int, frame.data, 6);

    int bytes_sent = write(socket_fd_, &frame, sizeof(struct can_frame));
    if (bytes_sent != static_cast<int>(sizeof(struct can_frame))) {
        static int err_count = 0;
        if (++err_count <= 5) {
            std::cerr << "[socketcan] send_position_target FAILED: bytes_sent=" << bytes_sent
                      << " errno=" << errno << " (" << std::strerror(errno) << ")"
                      << " [error " << err_count << " of first 5]" << std::endl;
        }
        return false;
    }
    return true;
}

bool SocketCANTransceiver::send_axis_state(uint32_t node_id, uint32_t state)
{
    if (!is_open_) return false;

    struct can_frame frame;
    std::memset(&frame, 0, sizeof(frame));
    frame.can_id  = (node_id << 5) | ODESC_CMD_SET_AXIS_STATE;
    frame.can_dlc = 8;
    std::memcpy(frame.data, &state, sizeof(state));  // uint32_t LE

    int bytes_sent = write(socket_fd_, &frame, sizeof(struct can_frame));
    return (bytes_sent == sizeof(struct can_frame));
}

bool SocketCANTransceiver::read_frame(struct canfd_frame& frame)
{
    if (!is_open_) return false;

    // Non-blocking: returns -1 immediately if no frame is waiting.
    // Accepts both classic CAN (CAN_MTU=16) and CAN-FD (CANFD_MTU=72) frames.
    std::memset(&frame, 0, sizeof(frame));
    int bytes_read = read(socket_fd_, &frame, sizeof(struct canfd_frame));
    return (bytes_read == CANFD_MTU || bytes_read == CAN_MTU);
}

} // namespace zeus_can_interface