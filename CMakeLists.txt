cmake_minimum_required(VERSION 3.16)
project(feetech_sts_hardware LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set the same behavior for windows as it is on linux
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## Add FTServo_Linux library
add_subdirectory(src/SCServo)

## COMPILE
add_library(
  feetech_sts_hardware
  SHARED
  src/feetech_sts_models.cpp
  src/feetech_sts_hardware.cpp
)
target_compile_features(feetech_sts_hardware PUBLIC cxx_std_20)
target_include_directories(feetech_sts_hardware PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/SCServo/src>
)
ament_target_dependencies(
  feetech_sts_hardware PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Link FTServo_Linux library
target_link_libraries(feetech_sts_hardware PRIVATE SCServo)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface feetech_sts_hardware.xml)

# INSTALL
install(TARGETS feetech_sts_hardware
  EXPORT export_feetech_sts_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## EXPORTS
ament_export_targets(export_feetech_sts_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
