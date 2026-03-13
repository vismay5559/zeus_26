#ifndef ZEUS_CONTROL_INTERFACE_HPP
#define ZEUS_CONTROL_INTERFACE_HPP

#include <vector>
#include <string>
#include <memory>
#include <linux/can.h>
#include "zeus_can_interface/zeus_can_interface.hpp"

namespace zeus_hardware {

// ODrive CAN Command IDs (The lower 5 bits)
enum ODriveCommandId : uint32_t {
    kHeartbeat           = 0x001, 
    kClearErrors         = 0x003,
    kSetAxisState        = 0x007,
    kGetEncoderEstimates = 0x009,
    kSetControllerMode   = 0x00B,
    kSetInputPos         = 0x00C,
    kSetInputVel         = 0x00D,
    kSetInputTorque      = 0x00E
};

// ODrive Axis States
enum ODriveAxisState : uint32_t {
    AXIS_STATE_IDLE                  = 1,
    AXIS_STATE_CLOSED_LOOP_CONTROL   = 8
};

// Data structures to hold the ROS 2 values
struct JointState {
    double position = 0.0; 
    double velocity = 0.0; 
    double effort = 0.0;   
};

struct JointCommand {
    double position = 0.0;
    double velocity = 0.0;
    double effort = 0.0;
};

// Represents a single physical entity (Either a Motor on can0, or a Spring on can1)
class ZeusJoint {
public:
    ZeusJoint(std::string name, uint32_t node_id, bool motor) 
        : joint_name(name), can_node_id(node_id), is_motor(motor) {}

    std::string joint_name;
    uint32_t can_node_id;
    bool is_motor; 

    JointState state;
    JointCommand command;

    uint32_t axis_error = 0;
    uint8_t axis_state = 0;

    bool pos_input_enabled = false;
    bool vel_input_enabled = false;
    bool effort_input_enabled = false;
};

// The main translator class
class ZeusControlInterface {
public:
    ZeusControlInterface();
    ~ZeusControlInterface();

    bool init(const std::string& muscle_iface, const std::string& sense_iface);
    void add_joint(const std::string& name, uint32_t node_id);
    ZeusJoint* get_joint(const std::string& name);

    void read();
    void write();

    void enable_motor(const std::string& joint_name);
    void disable_motor(const std::string& joint_name);

private:
    void on_can_frame_received(const std::string& iface_name, const can_frame& frame);

    void send_odrive_position(const ZeusJoint& joint);
    void send_odrive_velocity(const ZeusJoint& joint);
    void send_odrive_torque(const ZeusJoint& joint);

    std::unique_ptr<ZeusCanInterface> can_bridge_;
    std::vector<ZeusJoint> joints_;
};

} // namespace zeus_hardware

#endif // ZEUS_CONTROL_INTERFACE_HPP