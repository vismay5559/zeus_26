# zeus_hardware_interface

ROS 2 control hardware plugin for the Zeus robot.

This package bridges ROS 2 control to real hardware:

- SocketCAN commands to ODrive / ODESC motor drivers
- SocketCAN telemetry from ODrive S1 drivers
- AS5048A after-spring encoder SPI reads
- BNO085 IMU SPI reads
- MCP3008 FSR ADC SPI reads

## Interfaces

Command interfaces:

```text
joint_0/target_actuator_angle
...
joint_9/target_actuator_angle
```

State interfaces:

```text
joint_0/after_spring_angle
joint_0/load_encoder_position
joint_0/torque_estimate
...
joint_9/after_spring_angle
joint_9/load_encoder_position
joint_9/torque_estimate

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

left_toe_fsr/force
left_heel_fsr/force
right_toe_fsr/force
right_heel_fsr/force
```

Total:

```text
10 command interfaces
44 state interfaces
```

## Timing Targets

```text
write() Set_Input_Pos commands: 1000 Hz
read() AS5048A encoder chain: 1000 Hz
read() MCP3008 FSR channels: 1000 Hz
read() ODrive CAN telemetry drain: 1000 Hz
BNO085 report interval: 400 Hz
```

ODrive S1 should be configured to publish:

```text
encoder_msg_rate_ms = 1
torques_msg_rate_ms = 1
```

## Key Files

```text
include/zeus_hardware_interface/zeus_system.hpp
src/zeus_system.cpp
```

Main ROS 2 control hardware plugin.

```text
include/zeus_hardware_interface/encoder_utils.hpp
src/encoder_utils.cpp
```

AS5048A encoder SPI helpers.

```text
include/zeus_hardware_interface/sensor_utils.hpp
src/sensor_utils.cpp
```

BNO085 and MCP3008 helpers.
