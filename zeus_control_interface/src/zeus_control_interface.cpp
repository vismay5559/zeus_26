#include "zeus_control_interface/zeus_control_interface.hpp"
#include <iostream>
#include <cstring>
#include <cmath>

namespace zeus_hardware {

ZeusControlInterface::ZeusControlInterface() {
    can_bridge_ = std::make_unique<ZeusCanInterface>();
}

ZeusControlInterface::~ZeusControlInterface() {}

bool ZeusControlInterface::init(const std::string& muscle_iface, const std::string& sense_iface) {
    auto callback = [this](const std::string& iface, const can_frame& frame) {
        this->on_can_frame_received(iface, frame);
    };
    return can_bridge_->init(muscle_iface, sense_iface, callback);
}

void ZeusControlInterface::add_joint(const std::string& name, uint32_t node_id) {
    bool is_motor = (name.find("motor") != std::string::npos);
    joints_.emplace_back(name, node_id, is_motor);
    std::cout << "[ZeusControlInterface] Registered " << (is_motor ? "MOTOR" : "SPRING") 
              << " joint: " << name << " (Node " << node_id << ")" << std::endl;
}

ZeusJoint* ZeusControlInterface::get_joint(const std::string& name) {
    for (auto& joint : joints_) {
        if (joint.joint_name == name) return &joint;
    }
    return nullptr;
}

void ZeusControlInterface::read() {
    if (can_bridge_) can_bridge_->read_all_nonblocking();
}

void ZeusControlInterface::write() {
    for (const auto& joint : joints_) {
        if (joint.is_motor) {
            if (joint.pos_input_enabled) send_odrive_position(joint);
            else if (joint.vel_input_enabled) send_odrive_velocity(joint);
            else if (joint.effort_input_enabled) send_odrive_torque(joint);
        }
    }
}

void ZeusControlInterface::on_can_frame_received(const std::string& iface_name, const can_frame& frame) {
    uint32_t node_id = (frame.can_id >> 5) & 0x3F;
    uint32_t cmd_id  = frame.can_id & 0x1F;

    if (iface_name == "can0") {
        ZeusJoint* target_motor = nullptr;
        for (auto& joint : joints_) {
            if (joint.is_motor && joint.can_node_id == node_id) {
                target_motor = &joint; break;
            }
        }
        if (!target_motor) return;

        if (cmd_id == kHeartbeat && frame.can_dlc >= 8) {
            std::memcpy(&target_motor->axis_error, &frame.data[0], 4);
            std::memcpy(&target_motor->axis_state, &frame.data[4], 1);
        }
        else if (cmd_id == kGetEncoderEstimates && frame.can_dlc >= 8) {
            float pos_turns, vel_turns_s;
            std::memcpy(&pos_turns, &frame.data[0], 4);
            std::memcpy(&vel_turns_s, &frame.data[4], 4);
            
            target_motor->state.position = pos_turns * (2.0 * M_PI);
            target_motor->state.velocity = vel_turns_s * (2.0 * M_PI);
        }
    } 
    else if (iface_name == "can1") {
        ZeusJoint* target_spring = nullptr;
        for (auto& joint : joints_) {
            if (!joint.is_motor && joint.can_node_id == node_id) {
                target_spring = &joint; break;
            }
        }
        if (!target_spring) return;

        if (frame.can_dlc >= 8) {
            float esp32_pos_rads, esp32_vel_rads;
            std::memcpy(&esp32_pos_rads, &frame.data[0], 4);
            std::memcpy(&esp32_vel_rads, &frame.data[4], 4);
            
            target_spring->state.position = esp32_pos_rads;
            target_spring->state.velocity = esp32_vel_rads;
        }
    }
}

void ZeusControlInterface::send_odrive_position(const ZeusJoint& joint) {
    can_frame frame = {};
    frame.can_id = (joint.can_node_id << 5) | kSetInputPos;
    frame.can_dlc = 8;

    float pos_revs = joint.command.position / (2.0 * M_PI);
    float vel_ff = joint.command.velocity / (2.0 * M_PI); 
    float torque_ff = joint.command.effort;               

    int16_t vel_ff_int = static_cast<int16_t>(vel_ff * 1000.0f);
    int16_t torque_ff_int = static_cast<int16_t>(torque_ff * 1000.0f);

    std::memcpy(&frame.data[0], &pos_revs, 4);
    std::memcpy(&frame.data[4], &vel_ff_int, 2);
    std::memcpy(&frame.data[6], &torque_ff_int, 2);

    can_bridge_->send_muscle_frame(frame);
}

void ZeusControlInterface::send_odrive_velocity(const ZeusJoint& joint) {
    can_frame frame = {};
    frame.can_id = (joint.can_node_id << 5) | kSetInputVel;
    frame.can_dlc = 8;

    float vel_revs = joint.command.velocity / (2.0 * M_PI);
    float torque_ff = joint.command.effort;

    std::memcpy(&frame.data[0], &vel_revs, 4);
    std::memcpy(&frame.data[4], &torque_ff, 4);

    can_bridge_->send_muscle_frame(frame);
}

void ZeusControlInterface::send_odrive_torque(const ZeusJoint& joint) {
    can_frame frame = {};
    frame.can_id = (joint.can_node_id << 5) | kSetInputTorque;
    frame.can_dlc = 4;

    float torque = joint.command.effort;
    std::memcpy(&frame.data[0], &torque, 4);

    can_bridge_->send_muscle_frame(frame);
}

void ZeusControlInterface::enable_motor(const std::string& joint_name) {
    ZeusJoint* joint = get_joint(joint_name);
    if (!joint || !joint->is_motor) return;

    can_frame clear_msg = {};
    clear_msg.can_id = (joint->can_node_id << 5) | kClearErrors;
    clear_msg.can_dlc = 8;
    std::memset(&clear_msg.data[0], 0, 8);
    can_bridge_->send_muscle_frame(clear_msg);

    can_frame state_msg = {};
    state_msg.can_id = (joint->can_node_id << 5) | kSetAxisState;
    state_msg.can_dlc = 4;
    uint32_t closed_loop = AXIS_STATE_CLOSED_LOOP_CONTROL;
    std::memcpy(&state_msg.data[0], &closed_loop, 4);
    can_bridge_->send_muscle_frame(state_msg);

    std::cout << "[Zeus Hardware] Activated ODrive for " << joint_name << std::endl;
}

void ZeusControlInterface::disable_motor(const std::string& joint_name) {
    ZeusJoint* joint = get_joint(joint_name);
    if (!joint || !joint->is_motor) return;

    can_frame state_msg = {};
    state_msg.can_id = (joint->can_node_id << 5) | kSetAxisState;
    state_msg.can_dlc = 4;
    uint32_t idle = AXIS_STATE_IDLE;
    std::memcpy(&state_msg.data[0], &idle, 4);
    can_bridge_->send_muscle_frame(state_msg);

    std::cout << "[Zeus Hardware] Deactivated ODrive for " << joint_name << std::endl;
}

} // namespace zeus_hardware