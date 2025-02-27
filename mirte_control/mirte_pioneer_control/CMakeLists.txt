cmake_minimum_required(VERSION 3.16)
project(mirte_control LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS hardware_interface pluginlib rclcpp
                                 rclcpp_lifecycle mirte_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# COMPILE
add_library(mirte_control SHARED hardware/mirte_pioneer_system.cpp)
target_compile_features(mirte_control PUBLIC cxx_std_17)
target_include_directories(
  mirte_control PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
                       $<INSTALL_INTERFACE:include/mirte_control>)
ament_target_dependencies(mirte_control PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Causes the visibility macros to use dllexport rather than dllimport, which is
# appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "MIRTE_CONTROL_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface mirte_control.xml)

# INSTALL
install(DIRECTORY hardware/include/ DESTINATION include/mirte_control)
install(DIRECTORY description/launch description/ros2_control description/urdf
                  description/rviz DESTINATION share/mirte_control)
install(DIRECTORY bringup/launch bringup/config DESTINATION share/mirte_control)
install(
  TARGETS mirte_control
  EXPORT export_mirte_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

# EXPORTS
ament_export_targets(export_mirte_control HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
