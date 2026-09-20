cmake_minimum_required(VERSION 3.8)
project(zeus_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# `ros2 run zeus_bringup zeus check`, or put lib/zeus_bringup on PATH and just
# `zeus check`.
install(
  PROGRAMS scripts/zeus
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
