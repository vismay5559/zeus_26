ROS code base to be run on rpi-model 5 on custom made 13 DOF biped with 10 custom series elastic actuators using odrive-s1 and brushless motors from cubemars.


The repository structure -

zeus_26/
|
├── .gitignore
├── README.md               <-- Main documentation for the whole workspace
├── LICENSE                 <-- Global workspace license (e.g., Apache 2.0 or MIT)
│
├── zeus/                   <-- THE METAPACKAGE (Installs the whole ecosystem)
│   ├── CMakeLists.txt
│   ├── package.xml         <-- Contains dependencies for all other zeus_* packages
│   ├── README.md
│   └── LICENSE
│
├── zeus_bringup/           <-- EVERYONE shares this
│   ├── launch/
│   │   ├── hardware.launch.py   (Your launch file)
│   │   └── sim.launch.py        (Vedant's launch file)
│   ├── config/
│   │   └── zeus_controllers.yaml
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── README.md
│   └── LICENSE
│
├── zeus_description/       <-- SURYANSHU'S sandbox
│   ├── meshes/             <-- He drops his CAD exports (.stl or .dae) here
│   ├── urdf/
│   │   ├── zeus.urdf.xacro <-- The physical blueprint
│   │   └── zeus.ros2_control.xacro <-- Hardware/Sim toggles
|   ├── launch/ 
|   |   └── dispay.launch.py
|   ├── rviz/ 
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── README.md
│   └── LICENSE
│
├── zeus_gazebo/            <-- VEDANT'S sandbox
│   ├── worlds/             <-- Training environments (stairs, flat ground)
│   ├── models/
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── README.md
│   └── LICENSE
│
├── zeus_can_interface/     <-- YOUR sandbox (Standalone Library)
│   ├── include/zeus_can_interface/
│   │   └── socketcan.hpp
│   ├── src/
│   │   └── socketcan.cpp
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── README.md
│   └── LICENSE
│
├── zeus_hardware_interface/<-- YOUR sandbox (ROS 2 Plugin)
│   ├── include/zeus_hardware_interface/
│   │   └── zeus_system.hpp
│   ├── src/
│   │   └── zeus_system.cpp
│   ├── zeus_hardware_interface.xml
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── README.md
│   └── LICENSE
│
└── zeus_control_interface/ <-- AI Sandbox (Python)
    ├── zeus_control_interface/
    │   └── rl_policy_node.py
    ├── setup.py
    ├── package.xml
    ├── README.md
    ├── CMakeLists.txt
    └── LICENSE