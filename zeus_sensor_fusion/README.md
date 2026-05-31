# Zeus Sensor Fusion

This package is the future place for robot state estimation.

Current inputs:
- IMU data from `imu_sensor_broadcaster`
- encoder states from `joint_state_broadcaster`
- FSR values from `fsr_state_broadcaster`

Current outputs:
- `/zeus/estimated_height`
- `/zeus/estimated_linear_velocity`

The current math is intentionally minimal. Real height estimation should use the leg geometry and encoder angles, and real velocity estimation should fuse IMU acceleration with encoder/contact constraints instead of directly integrating acceleration forever.
