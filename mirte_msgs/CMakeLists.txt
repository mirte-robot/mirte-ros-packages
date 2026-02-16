cmake_minimum_required(VERSION 3.8)
project(mirte_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(msg_files
    "msg/ColorHSL.msg"
    "msg/ColorHSLStamped.msg"
    "msg/ColorRGBAStamped.msg"
    "msg/ColorRGBW.msg"
    "msg/ColorRGBWStamped.msg"
    "msg/Encoder.msg"
    "msg/Intensity.msg"
    "msg/IntensityDigital.msg"
    "msg/Keypad.msg"
    "msg/ServoPosition.msg"
    "msg/SetSpeedNamed.msg")

set(srv_files
    "srv/GetAnalogPinValue.srv"
    "srv/GetBoardCharacteristics.srv"
    "srv/GetColorHSL.srv"
    "srv/GetColorRGBA.srv"
    "srv/GetColorRGBW.srv"
    "srv/GetDigitalPinValue.srv"
    "srv/GetEncoder.srv"
    "srv/GetImu.srv"
    "srv/GetIntensity.srv"
    "srv/GetIntensityDigital.srv"
    "srv/GetKeypad.srv"
    "srv/GetRange.srv"
    "srv/GetServoRange.srv"
    "srv/SetDigitalPinValue.srv"
    "srv/SetLEDValue.srv"
    "srv/SetMotorSpeed.srv"
    "srv/SetOLEDFile.srv"
    "srv/SetOLEDImage.srv"
    "srv/SetOLEDImageLegacy.srv"
    "srv/SetOLEDText.srv"
    "srv/SetPWMPinValue.srv"
    "srv/SetServoAngle.srv"
    "srv/SetServoAngleWithSpeed.srv"
    "srv/SetSpeedMultiple.srv"
    "srv/GetServoOffset.srv"
    "srv/SetServoOffset.srv"
    "srv/AddHiwonderServo.srv"
    )

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES
  builtin_interfaces
  std_msgs
  sensor_msgs)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
