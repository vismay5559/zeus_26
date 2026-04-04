#ifndef ZEUS_CAN_INTERFACE__SOCKETCAN_HPP_
#define ZEUS_CAN_INTERFACE__SOCKETCAN_HPP_

#include <string>
#include <vector>
#include <cstdint>
#include <linux/can.h>
#include <linux/can/raw.h>

namespace zeus_can_interface
{

// ODrive / ODESC V4.2 CAN Command IDs
constexpr uint32_t ODESC_CMD_GET_ENCODER_ESTIMATES = 0x009;
constexpr uint32_t ODESC_CMD_SET_INPUT_POS = 0x00C;
constexpr uint32_t ODESC_CMD_SET_AXIS_STATE = 0x007;

class SocketCANTransceiver
{
public:
    SocketCANTransceiver();
    ~SocketCANTransceiver();

    // Initializes the SocketCAN interface (e.g., "can0")
    bool open_port(const std::string& interface_name);
    void close_port();

    // The 1 kHz target sender. 
    // Uses the ODESC 'Set_Input_Pos' command which works perfectly with INPUT_MODE_POS_FILTER
    bool send_position_target(uint32_t node_id, float position, float vel_ff = 0.0f, float torque_ff = 0.0f);

    // Clears the incoming buffer to read telemetry (Non-blocking)
    bool read_frame(struct can_frame& frame);

private:
    int socket_fd_;
    bool is_open_;

    // Helper function to safely copy float bits into uint8_t arrays
    void pack_float(float value, uint8_t* buffer, int start_index);
    void pack_int16(int16_t value, uint8_t* buffer, int start_index);
};

} // namespace zeus_can_interface

#endif // ZEUS_CAN_INTERFACE__SOCKETCAN_HPP_