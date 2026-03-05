cmake_minimum_required(VERSION 3.16)
project(lc3_hw_interface)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# Find libmodbus
find_library(MODBUS_LIBRARY
  NAMES modbus libmodbus
)

find_path(MODBUS_INCLUDE_DIR
  modbus/modbus.h
)

if(NOT MODBUS_LIBRARY)
  message(FATAL_ERROR "libmodbus not found. Install with: sudo apt install libmodbus-dev")
endif()

message(STATUS "Found libmodbus: ${MODBUS_LIBRARY}")

# Build LC3 hardware interface library
add_library(
  lc3_hw_interface
  SHARED
  src/lc3_hardware_interface.cpp
)

target_compile_features(lc3_hw_interface PUBLIC cxx_std_17)

target_include_directories(lc3_hw_interface PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/lc3_hw_interface>
  ${MODBUS_INCLUDE_DIR}
)

ament_target_dependencies(
  lc3_hw_interface PUBLIC
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

target_compile_definitions(lc3_hw_interface PRIVATE "LC3_HW_INTERFACE_BUILDING_DLL")

target_link_libraries(lc3_hw_interface PUBLIC ${MODBUS_LIBRARY})

# Export hardware plugin
pluginlib_export_plugin_description_file(hardware_interface lc3_hw_interface_plugin.xml)

# Install
install(
  DIRECTORY include/
  DESTINATION include/lc3_hw_interface
)

install(
  DIRECTORY urdf launch config scripts
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS lc3_hw_interface
  EXPORT export_lc3_hw_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## EXPORTS
ament_export_targets(export_lc3_hw_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)

ament_package()
