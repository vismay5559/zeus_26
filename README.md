# Zeus Workspace

ROS 2 workspace for the Zeus biped running on a Raspberry Pi 5.

Current hardware direction in this repo:
- 10 series-elastic actuators commanded over CAN
- 2 CAN buses: joints `1-5` on `can0`, joints `6-10` on `can1`
- 10 daisy-chained AS5048A absolute after-spring encoders read over SPI
- Gazebo and real-hardware packages live in the same workspace

## Workspace Layout

This is the current repository structure, including files and folders that are present but still empty.

```text
nexus_final_ws/
├── LICENSE
├── README.md
├── zeus/
│   ├── CMakeLists.txt
│   ├── LICENSE
│   ├── README.md
│   └── package.xml
├── zeus_bringup/
│   ├── CMakeLists.txt
│   ├── LICENSE
│   ├── README.md
│   ├── config/
│   │   ├── fastdds_shm.xml
│   │   └── zeus_controllers.yaml
│   ├── launch/
│   │   ├── hardware.launch.py
│   │   └── sim.launch.py
│   └── package.xml
├── zeus_can_interface/
│   ├── CMakeLists.txt
│   ├── LICENSE
│   ├── README.md
│   ├── include/
│   │   └── zeus_can_interface/
│   │       └── socketcan.hpp
│   ├── package.xml
│   └── src/
│       └── socketcan.cpp
├── zeus_control_interface/
│   ├── LICENSE
│   ├── README.md
│   ├── package.xml
│   ├── resource/
│   │   └── zeus_control_interface
│   ├── setup.cfg
│   ├── setup.py
│   └── zeus_control_interface/
│       ├── __init__.py
│       └── rl_policy_node.py
├── zeus_description/
│   ├── CMakeLists.txt
│   ├── LICENSE
│   ├── README.md
│   ├── launch/
│   │   └── display.launch.py
│   ├── meshes/
│   ├── package.xml
│   ├── rviz/
│   └── urdf/
│       ├── zeus_ros2_control.xacro
│       └── zeus_urdf.xacro
├── zeus_gazebo/
│   ├── CMakeLists.txt
│   ├── LICENSE
│   ├── README.md
│   ├── launch/
│   ├── models/
│   ├── package.xml
│   └── worlds/
└── zeus_hardware_interface/
    ├── CMakeLists.txt
    ├── LICENSE
    ├── README.Md
    ├── include/
    │   └── zeus_hardware_interface/
    │       ├── encoder_utils.hpp
    │       └── zeus_system.hpp
    ├── package.xml
    ├── src/
    │   ├── encoder_utils.cpp
    │   └── zeus_system.cpp
    └── zeus_hardware_interface.xml
```


## Package Roles

### `zeus`
Metapackage that depends on the other `zeus_*` packages.

### `zeus_bringup`
Place for launch files and controller config.

Current contents:
- [zeus_controllers.yaml](/home/vismay/nexus_final_ws/zeus_bringup/config/zeus_controllers.yaml): sets `controller_manager` update rate to `1000 Hz`, exposes `after_spring_angle` through `joint_state_broadcaster`, and accepts `target_actuator_angle` through a forward command controller
- [fastdds_shm.xml](/home/vismay/nexus_final_ws/zeus_bringup/config/fastdds_shm.xml): Fast DDS profile that forces shared-memory-only transport so the Python RL node and C++ ROS 2 nodes can communicate through SHM on the same machine
- [hardware.launch.py](/home/vismay/nexus_final_ws/zeus_bringup/launch/hardware.launch.py): sets `FASTRTPS_DEFAULT_PROFILES_FILE` to the shared-memory profile, launches `ros2_control_node`, and launches `zeus_control_interface/rl_policy_node`
- [sim.launch.py](/home/vismay/nexus_final_ws/zeus_bringup/launch/sim.launch.py): currently empty

### `zeus_can_interface`
Low-level CAN transport package.

Main files:
- [socketcan.hpp](/home/vismay/nexus_final_ws/zeus_can_interface/include/zeus_can_interface/socketcan.hpp)
- [socketcan.cpp](/home/vismay/nexus_final_ws/zeus_can_interface/src/socketcan.cpp)

What it currently does:
- opens a Linux SocketCAN raw socket
- binds to a specific interface such as `can0` or `can1`
- packs and sends ODESC `Set_Input_Pos` frames
- exposes a non-blocking `read_frame()` helper for future CAN telemetry

### `zeus_control_interface`
Python-side control package.

Current state:
- package scaffolding is present
- [rl_policy_node.py](/home/vismay/nexus_final_ws/zeus_control_interface/zeus_control_interface/rl_policy_node.py) is currently empty
- the top-level hardware launch already reserves a launch slot for this node

### `zeus_description`
Robot description package.

Main files:
- [zeus_ros2_control.xacro](/home/vismay/nexus_final_ws/zeus_description/urdf/zeus_ros2_control.xacro): current hardware interface configuration
- [zeus_urdf.xacro](/home/vismay/nexus_final_ws/zeus_description/urdf/zeus_urdf.xacro): currently empty

Current empty paths:
- [display.launch.py](/home/vismay/nexus_final_ws/zeus_description/launch/display.launch.py)
- `meshes/`
- `rviz/`

### `zeus_gazebo`
Simulation package intended for Gazebo integration.

Current state:
- package scaffolding is present
- `launch/`, `models/`, and `worlds/` currently exist but are empty

### `zeus_hardware_interface`
ROS 2 hardware plugin that bridges ROS control to CAN and SPI.

Main files:
- [zeus_system.hpp](/home/vismay/nexus_final_ws/zeus_hardware_interface/include/zeus_hardware_interface/zeus_system.hpp)
- [zeus_system.cpp](/home/vismay/nexus_final_ws/zeus_hardware_interface/src/zeus_system.cpp)
- [encoder_utils.hpp](/home/vismay/nexus_final_ws/zeus_hardware_interface/include/zeus_hardware_interface/encoder_utils.hpp)
- [encoder_utils.cpp](/home/vismay/nexus_final_ws/zeus_hardware_interface/src/encoder_utils.cpp)
- [zeus_hardware_interface.xml](/home/vismay/nexus_final_ws/zeus_hardware_interface/zeus_hardware_interface.xml)

What it currently does:
- exports `target_actuator_angle` as the command interface
- exports `after_spring_angle` as the state interface
- opens both `can0` and `can1`
- routes joints `0-4` to `can0` and joints `5-9` to `can1` by configuration
- reads the AS5048A encoder daisy chain over SPI
- maps encoder readings back to joint states
- low-pass filters the after-spring angle before publishing it to ROS 2

## Current Hardware Flow

The intended real-hardware control path in this workspace is:

1. `hardware.launch.py` exports `FASTRTPS_DEFAULT_PROFILES_FILE` pointing to [fastdds_shm.xml](/home/vismay/nexus_final_ws/zeus_bringup/config/fastdds_shm.xml).
2. Fast DDS is configured to use shared memory only for nodes launched in that process tree.
3. The Python RL side and the C++ ROS 2 side exchange command/state data through local SHM transport.
4. A ROS 2 controller writes `target_actuator_angle`.
5. `zeus_hardware_interface` receives that command.
6. The command is smoothed/interpolated inside the hardware interface.
7. The target is sent over CAN using `zeus_can_interface`.
8. ODESC receives `Set_Input_Pos` commands on either `can0` or `can1`.
9. SPI reads the 10 after-spring encoders.
10. The filtered encoder angle is published as `after_spring_angle`.

## Fast DDS Shared Memory

The workspace now includes [fastdds_shm.xml](/home/vismay/nexus_final_ws/zeus_bringup/config/fastdds_shm.xml), which defines a Fast DDS participant profile named `shm_only_profile`.

What it currently does:
- disables builtin transports
- enables only SHM transport
- sets `maxMessageSize` to `65000`

This is meant for low-latency communication between:
- the Python RL process in `zeus_control_interface`
- the C++ ROS 2 hardware/control side

The SHM profile is applied from [hardware.launch.py](/home/vismay/nexus_final_ws/zeus_bringup/launch/hardware.launch.py) by setting the `FASTRTPS_DEFAULT_PROFILES_FILE` environment variable before launching nodes.

## Current `ros2_control` Configuration

The hardware description in [zeus.ros2_control.xacro](/home/vismay/nexus_final_ws/zeus_description/urdf/zeus.ros2_control.xacro) currently sets:

- `can0` for joints `joint_0` to `joint_4`
- `can1` for joints `joint_5` to `joint_9`
- `node_id` `1..5` on each CAN bus
- SPI device `/dev/spidev1.0`
- SPI speed `1000000`
- SPI mode `1`
- `10` daisy-chained encoders
- direct encoder-to-joint map `0,1,2,3,4,5,6,7,8,9`

## Build

Typical workspace build:

```bash
colcon build
source install/setup.bash
```

Focused rebuild for the hardware path:

```bash
colcon build --packages-select zeus_can_interface zeus_hardware_interface
source install/setup.bash
```

## Current Gaps

A few important source files are still placeholders:
- [zeus_bringup/launch/sim.launch.py](/home/vismay/nexus_final_ws/zeus_bringup/launch/sim.launch.py)
- [zeus_description/launch/display.launch.py](/home/vismay/nexus_final_ws/zeus_description/launch/display.launch.py)
- [zeus_description/urdf/zeus.urdf.xacro](/home/vismay/nexus_final_ws/zeus_description/urdf/zeus.urdf.xacro)
- [zeus_control_interface/zeus_control_interface/rl_policy_node.py](/home/vismay/nexus_final_ws/zeus_control_interface/zeus_control_interface/rl_policy_node.py)

Also note that [hardware.launch.py](/home/vismay/nexus_final_ws/zeus_bringup/launch/hardware.launch.py) is no longer empty, but it still contains a placeholder comment where the full `ros2_control_node` parameters should be added.

So right now the strongest implemented part of the repo is the hardware-side CAN/SPI stack, the ROS 2 hardware plugin around it, and the shared-memory bringup direction for Python/C++ communication.
