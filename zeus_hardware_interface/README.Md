# zeus_hardware_interface

ROS 2 control hardware plugin for the Zeus robot.

This package bridges ROS 2 control to real hardware:

- SocketCAN commands to ODrive S1 motor drivers
- SocketCAN telemetry from ODrive S1 drivers (encoder estimates, torque, heartbeat)
- AS5048A after-spring encoder SPI reads (SEA joints only)
- BNO085 IMU UART (SHTP) reads
- Mechanical foot contact switches via GPIO sysfs

## Interfaces

Command interfaces:

```text
joint_0/target_actuator_angle
...
joint_9/target_actuator_angle
```

State interfaces:

```text
joint_0/after_spring_angle       (SEA joints only: 0, 2, 5, 7)
joint_0/load_encoder_position
joint_0/torque_estimate
joint_0/actuator_fault
...
joint_9/load_encoder_position
joint_9/torque_estimate
joint_9/actuator_fault

imu/orientation.x
imu/orientation.y
imu/orientation.z
imu/orientation.w
imu/linear_acceleration.x
imu/linear_acceleration.y
imu/linear_acceleration.z
imu/angular_velocity.x
imu/angular_velocity.y
imu/angular_velocity.z

left_toe_switch/contact
left_heel_switch/contact
right_toe_switch/contact
right_heel_switch/contact
```

`actuator_fault` is a reason bitmask (0 = healthy) driven by heartbeat/CAN staleness detection —
see the main [README.md](../README.md#actuator-fault-detection--heartbeat--can-staleness-watchdog).

Total:

```text
10 command interfaces
48 state interfaces
```

## Timing Targets

```text
write() Set_Input_Pos commands: 1000 Hz
read() AS5048A encoder chain: 1000 Hz
read() GPIO contact switch reads: 1000 Hz
read() ODrive CAN telemetry drain (encoder + torque + heartbeat): 1000 Hz
BNO085 UART report interval: 400 Hz
```

Each ODrive should be configured (via `odrivetool`) to publish:

```text
axis.config.can.encoder_rate_ms = 1
axis.config.can.torques_rate_ms = 1
axis.config.can.heartbeat_rate_ms = 10
```

## Key Files

```text
include/zeus_hardware_interface/zeus_system.hpp
src/zeus_system.cpp
```

Main ROS 2 control hardware plugin — CAN telemetry/command handling, actuator fault
watchdog, GPIO switch reads.

```text
include/zeus_hardware_interface/encoder_utils.hpp
src/encoder_utils.cpp
```

AS5048A encoder SPI helpers.

```text
include/zeus_hardware_interface/sensor_utils.hpp
src/sensor_utils.cpp
```

BNO085 IMU UART (SHTP) helpers.
