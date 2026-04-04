#ifndef ZEUS_HARDWARE_INTERFACE_HPP
#define ZEUS_HARDWARE_INTERFACE_HPP

#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

// Bring in the translator layer we built
#include "zeus_control_interface/zeus_control_interface.hpp"

namespace zeus_hardware {

class ZeusHardwareInterface final : public hardware_interface::SystemInterface {
public:
    // A helpful ROS 2 macro that automatically creates smart pointer aliases for this class
    RCLCPP_SHARED_PTR_DEFINITIONS(ZeusHardwareInterface)

    // --- LIFECYCLE MANAGEMENT ---
    // These functions map directly to the states of the ROS 2 controller manager
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

    // --- INTERFACE EXPORTS ---
    // These tell ROS 2 exactly what variables it is allowed to read from and write to
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    // --- MODE SWITCHING ---
    // Handles changing between Position, Velocity, and Effort control on the fly
    hardware_interface::return_type perform_command_mode_switch(
        const std::vector<std::string>& start_interfaces,
        const std::vector<std::string>& stop_interfaces
    ) override;

    // --- THE 1000 HZ LOOP ---
    hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    // Our custom translation engine that does all the CAN packing/unpacking
    ZeusControlInterface control_interface_;
    
    // The OS network names (e.g., "can0" and "can1") pulled from the URDF
    std::string muscle_can_name_;
    std::string sense_can_name_;
};

} // namespace zeus_hardware

#endif // ZEUS_HARDWARE_INTERFACE_HPP