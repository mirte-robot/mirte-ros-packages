cmake_minimum_required(VERSION 3.9)
project(mirte_telemetrix_cpp)

# Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++20)
set(CMAKE_CXX_STANDARD 20)
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native")

include(CheckIncludeFileCXX)

set(MIRTE_EXTRA_LIBS "")

# Use libgpiod for Mirte Master LED power indicator.
check_include_file_cxx(gpiod.hpp HAVE_GPIOD_HPP)
if(HAVE_GPIOD_HPP)
  list(APPEND MIRTE_EXTRA_LIBS gpiodcxx)
  add_compile_definitions(WITH_GPIO)
  add_compile_options(-lgpiodcxx)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# for iwyu, use iwyu_tool.py -p . -j 8
if(CMAKE_EXPORT_COMPILE_COMMANDS)
  set(CMAKE_CXX_STANDARD_INCLUDE_DIRECTORIES
      ${CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES})
endif()

# to time compilation time. Will gnerate json files you can open in
# chrome://tracing SET (CMAKE_CXX_FLAGS_INIT                "-ftime-trace")

# Find catkin macros and libraries if COMPONENTS list like find_package(catkin
# REQUIRED COMPONENTS xyz) is used, also find other catkin packages
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcpputils REQUIRED)
find_package(mirte_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(color_util REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(backward_ros REQUIRED)

find_package(OpenCV REQUIRED COMPONENTS core imgcodecs)
find_package(Boost REQUIRED COMPONENTS system thread)
# ##############################################################################
# Build ##
# ##############################################################################

# Specify additional locations of header files Your package locations should be
# listed before other locations
add_subdirectory(libs/tmx-cpp)
include_directories(
  include/ libs/tmx-cpp/include/ libs/tmx-cpp/libs/async_serial/include/
  ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

# include(CheckIPOSupported) check_ipo_supported(RESULT supported OUTPUT error)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wswitch-enum -Wall -Werror
# -Wno-sign-compare") # -Wall -Wextra -Wpedantic -Werror") set(NO_CLANG_TIDY
# FALSE CACHE BOOL "Disable clang-tidy") if(NOT NO_CLANG_TIDY)
# set(CMAKE_CXX_CLANG_TIDY clang-tidy-14; -format-style='file';
# -header-filter=${CMAKE_CURRENT_SOURCE_DIR}; )
# set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# endif()

add_executable(
  ${PROJECT_NAME}_node
  src/util.cpp
  src/mirte-telemetrix.cpp
  src/mirte-board.cpp
  src/boards/atmega328.cpp
  src/boards/pico.cpp
  src/boards/pcb.cpp
  src/mirte-actuators.cpp
  src/mirte-modules.cpp
  src/mirte-sensors.cpp
  src/device.cpp
  src/gpio_pin.cpp
  src/actuators/servo_base.cpp
  src/actuators/servo/servo.cpp
  src/actuators/motor.cpp
  src/actuators/motor/ddp_motor.cpp
  src/actuators/motor/dp_motor.cpp
  src/actuators/motor/pp_motor.cpp
  src/parsers/parsers.cpp
  src/parsers/device_data.cpp
  src/parsers/actuators/motor_data.cpp
  src/parsers/actuators/servo_data.cpp
  src/parsers/sensors/base_sensor_data.cpp
  src/parsers/sensors/encoder_data.cpp
  src/parsers/sensors/intensity_data.cpp
  src/parsers/sensors/sonar_data.cpp
  src/parsers/sensors/keypad_data.cpp
  src/parsers/modules/adxl345_data.cpp
  src/parsers/modules/module_data.cpp
  src/parsers/modules/i2c_module_data.cpp
  src/parsers/modules/ina226_data.cpp
  src/parsers/modules/hiwonder_data.cpp
  src/parsers/modules/hiwonder/hiwonder_servo_data.cpp
  src/parsers/modules/pca_data.cpp
  src/parsers/modules/pca/pca_motor_data.cpp
  src/parsers/modules/pca/pca_servo_data.cpp
  src/parsers/modules/ssd1306_data.cpp
  src/parsers/modules/mpu9250_data.cpp
  src/parsers/modules/veml6040_data.cpp
  src/sensors/base_sensor.cpp
  src/sensors/encoder_monitor.cpp
  src/sensors/intensity_monitor.cpp
  src/sensors/keypad_monitor.cpp
  src/sensors/sonar_monitor.cpp
  src/modules/adxl345_module.cpp
  src/modules/base_module.cpp
  src/modules/hiwonder_module.cpp
  src/modules/hiwonder/hiwonder_servo.cpp
  src/modules/ina226_module.cpp
  src/modules/mpu9250_module.cpp
  src/modules/pca_module.cpp
  src/modules/pca/pca_motor.cpp
  src/modules/pca/pca_servo.cpp
  src/modules/ssd1306_module.cpp
  src/modules/veml6040_module.cpp)
ament_target_dependencies(
  ${PROJECT_NAME}_node
  std_msgs
  std_srvs
  mirte_msgs
  rclcpp
  rcpputils
  sensor_msgs
  color_util
  cv_bridge # DEPENDS
  backward_ros
  # system_lib
)

if(0) # extra strict errors
  target_compile_options(
    ${PROJECT_NAME}_node
    PRIVATE -Wall
            -Wextra
            -Wpedantic
            -Wconversion
            -Wno-sign-conversion
            -Wdouble-promotion
            -Wno-unused-parameter
            -Wno-float-conversion
            -Wno-conversion
            -Wno-double-promotion
            -Wno-sign-compare
            -Wno-unused-function
            -Werror
            -Wuninitialized)
endif()

# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node Boost::system Boost::thread tmx_cpp
                      ${MIRTE_EXTRA_LIBS} ${OpenCV_LIBS})

# if(supported) if(CMAKE_BUILD_TYPE STREQUAL "Release") message(STATUS "IPO LTO
# enabled") set_property(TARGET ${PROJECT_NAME}_node PROPERTY
# INTERPROCEDURAL_OPTIMIZATION TRUE) endif() else() message(STATUS "IPO  / LTO
# not supported: <${error}>") endif()

install(
  TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})

# EXPORTS ament_export_targets( ${PROJECT_NAME}_node HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
# install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)
install(PROGRAMS scripts/default_screen.sh DESTINATION share/${PROJECT_NAME}/scripts/default_screen.sh)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_clang_format_CONFIG_FILE ${PROJECT_SOURCE_DIR}/.clang-format)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
