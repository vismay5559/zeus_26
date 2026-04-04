#include "zeus_can_interface/socketcan.hpp"
#include <iostream>
#include <cstring>
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

    // 4. CRITICAL FOR 1 kHz: Set the socket to non-blocking mode.
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

    struct can_frame frame;
    
    // ODrive CAN Math: Shift the Node ID left by 5 bits and append the Command ID
    frame.can_id = (node_id << 5) | ODESC_CMD_SET_INPUT_POS;
    frame.can_dlc = 8; // We are sending 8 bytes of data

    // The ODESC Set_Input_Pos command expects:
    // Bytes 0-3: Position (Float32)
    // Bytes 4-5: Velocity Feedforward (Int16) -> Factor: 0.001
    // Bytes 6-7: Torque Feedforward (Int16) -> Factor: 0.001

    int16_t vel_int = static_cast<int16_t>(vel_ff * 1000.0f);
    int16_t torque_int = static_cast<int16_t>(torque_ff * 1000.0f);

    pack_float(position, frame.data, 0);
    pack_int16(vel_int, frame.data, 4);
    pack_int16(torque_int, frame.data, 6);

    // Send the frame over the Linux socket
    int bytes_sent = write(socket_fd_, &frame, sizeof(struct can_frame));
    
    return (bytes_sent == sizeof(struct can_frame));
}

bool SocketCANTransceiver::read_frame(struct can_frame& frame)
{
    if (!is_open_) return false;

    // Because the socket is non-blocking, this instantly returns -1 if no message is waiting.
    int bytes_read = read(socket_fd_, &frame, sizeof(struct can_frame));
    
    return (bytes_read == sizeof(struct can_frame));
}

} // namespace zeus_can_interface