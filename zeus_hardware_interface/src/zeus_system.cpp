#include "zeus_hardware_interface/zeus_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace zeus_hardware {

hardware_interface::CallbackReturn ZeusHardwareInterface::on_init(const hardware_interface::HardwareInfo& info) {
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    muscle_can_name_ = info_.hardware_parameters.count("muscle_can") ? info_.hardware_parameters.at("muscle_can") : "can0";
    sense_can_name_  = info_.hardware_parameters.count("sense_can")  ? info_.hardware_parameters.at("sense_can")  : "can1";

    for (const auto& joint : info_.joints) {
        uint32_t node_id = 0;
        if (joint.parameters.count("node_id")) {
            node_id = std::stoi(joint.parameters.at("node_id"));
        }
        control_interface_.add_joint(joint.name, node_id);
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ZeusHardwareInterface::on_configure(const rclcpp_lifecycle::State&) {
    RCLCPP_INFO(rclcpp::get_logger("ZeusHardwareInterface"), "Configuring dual CAN buses for SEA...");
    
    if (!control_interface_.init(muscle_can_name_, sense_can_name_)) {
        RCLCPP_ERROR(rclcpp::get_logger("ZeusHardwareInterface"), "Failed to bind to CAN sockets.");
        return hardware_interface::CallbackReturn::ERROR;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ZeusHardwareInterface::on_activate(const rclcpp_lifecycle::State&) {
    RCLCPP_INFO(rclcpp::get_logger("ZeusHardwareInterface"), "Activating SEA hardware interfaces...");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ZeusHardwareInterface::on_deactivate(const rclcpp_lifecycle::State&) {
    RCLCPP_INFO(rclcpp::get_logger("ZeusHardwareInterface"), "Deactivating SEA hardware interfaces...");
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ZeusHardwareInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (const auto& joint_info : info_.joints) {
        ZeusJoint* joint = control_interface_.get_joint(joint_info.name);
        if (!joint) continue;

        for (const auto& interface : joint_info.state_interfaces) {
            if (interface.name == hardware_interface::HW_IF_POSITION) {
                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    joint_info.name, hardware_interface::HW_IF_POSITION, &joint->state.position));
            }
            else if (interface.name == hardware_interface::HW_IF_VELOCITY) {
                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    joint_info.name, hardware_interface::HW_IF_VELOCITY, &joint->state.velocity));
            }
        }
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ZeusHardwareInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (const auto& joint_info : info_.joints) {
        ZeusJoint* joint = control_interface_.get_joint(joint_info.name);
        if (!joint) continue;

        for (const auto& interface : joint_info.command_interfaces) {
            if (interface.name == hardware_interface::HW_IF_POSITION) {
                command_interfaces.emplace_back(hardware_interface::CommandInterface(
                    joint_info.name, hardware_interface::HW_IF_POSITION, &joint->command.position));
            }
        }
    }

    return command_interfaces;
}

hardware_interface::return_type ZeusHardwareInterface::perform_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces) 
{
    for (const auto& joint_info : info_.joints) {
        ZeusJoint* joint = control_interface_.get_joint(joint_info.name);
        if (!joint) continue;

        bool mode_switched_off = false;
        bool mode_switched_on = false;

        for (const std::string& key : stop_interfaces) {
            if (key == joint->joint_name + "/" + hardware_interface::HW_IF_POSITION) {
                joint->pos_input_enabled = false;
                mode_switched_off = true;
            }
        }

        for (const std::string& key : start_interfaces) {
            if (key == joint->joint_name + "/" + hardware_interface::HW_IF_POSITION) {
                joint->pos_input_enabled = true;
                mode_switched_on = true;
            }
        }

        if (mode_switched_on && joint->is_motor) {
            control_interface_.enable_motor(joint->joint_name);
        } else if (mode_switched_off && joint->is_motor && !joint->pos_input_enabled) {
            control_interface_.disable_motor(joint->joint_name);
        }
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type ZeusHardwareInterface::read(const rclcpp::Time&, const rclcpp::Duration&) {
    control_interface_.read();
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type ZeusHardwareInterface::write(const rclcpp::Time&, const rclcpp::Duration&) {
    control_interface_.write();
    return hardware_interface::return_type::OK;
}

} // namespace zeus_hardware

PLUGINLIB_EXPORT_CLASS(zeus_hardware::ZeusHardwareInterface, hardware_interface::SystemInterface)