cmake_minimum_required(VERSION 3.8)
project(zeus_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ==============================================================================
# 1. Find Dependencies
# ==============================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(zeus_control_interface REQUIRED)
find_package(zeus_can_interface REQUIRED)

# ==============================================================================
# 2. Build the Library
# ==============================================================================
# ==============================================================================
# 2. Build the Library
# ==============================================================================
add_library(${PROJECT_NAME} SHARED
  src/zeus_hardware_interface.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE include)

# USE THIS INSTEAD OF target_link_libraries
# This ensures that headers like hardware_info.hpp are found
ament_target_dependencies(${PROJECT_NAME}
  zeus_control_interface
  zeus_can_interface
  rclcpp
  hardware_interface
  pluginlib
  rclcpp_lifecycle
  std_msgs
)

# ==============================================================================
# 3. Export the Plugin (The "Menu")
# ==============================================================================
# Tell pluginlib that this package provides a plugin via the XML file
pluginlib_export_plugin_description_file(
  hardware_interface zeus_hardware_interface.xml
)

# ==============================================================================
# 4. Install Targets
# ==============================================================================
# Install the compiled library (.so file) so ROS can load it
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install the XML file so it appears in the share folder
install(
  FILES zeus_hardware_interface.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install the header files (good practice)
install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_dependencies(pluginlib)

# ==============================================================================
# 5. Testing & Packaging
# ==============================================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()