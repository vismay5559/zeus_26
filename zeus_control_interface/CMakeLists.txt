cmake_minimum_required(VERSION 3.8)
project(zeus_control_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. FIND DEPENDENCIES
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(zeus_can_interface REQUIRED)  # <--- CRITICAL: Must find the package

# 2. CREATE LIBRARY
add_library(${PROJECT_NAME} SHARED 
src/robot.cpp)

# 3. INCLUDE DIRECTORIES
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 4. LINK LIBRARIES & HEADERS
# This function ensures the compiler gets the "-I" flag for masterboard
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  zeus_can_interface  # <--- CRITICAL: Must link the package
)

# 5. INSTALLATION (So zeus_hardware_interface can find THIS package later)
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# 6. EXPORT
ament_export_targets(export_${PROJECT_NAME})
ament_export_include_directories(include)
ament_export_dependencies(rclcpp)
ament_export_dependencies(zeus_can_interface) # Export this so downstream packages know about it too

ament_package()