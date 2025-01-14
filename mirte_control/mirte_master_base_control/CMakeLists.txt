cmake_minimum_required(VERSION 3.9)
project(mirte_base_control)
add_compile_options(-g)

set(CMAKE_CXX_STANDARD 20)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mirte_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
    mirte_msgs
    std_srvs
    std_msgs
    control_toolbox
    yaml_cpp_vendor)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(hardware/include/ ${catkin_INCLUDE_DIRS})
# COMPILE
add_library(${PROJECT_NAME} SHARED hardware/mirte_base_control.cpp
                                   hardware/params.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
         $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
ament_target_dependencies(${PROJECT_NAME} PUBLIC
                          ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Causes the visibility macros to use dllexport rather than dllimport, which is
# appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME}
                           PRIVATE "${PROJECT_NAME}_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface ${PROJECT_NAME}.xml)

# INSTALL
install(DIRECTORY hardware/include/ DESTINATION include/${PROJECT_NAME})
# install( DIRECTORY description/launch description/ros2_control
# description/urdf description/rviz DESTINATION share/${PROJECT_NAME} )
install(DIRECTORY bringup/launch bringup/config bringup/urdf
        DESTINATION share/${PROJECT_NAME})
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

# EXPORTS
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
