#include <algorithm>
#include <functional>
#include <numbers>

#include <rclcpp/callback_group.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tmx_cpp/tmx.hpp>

#include <mirte_telemetrix_cpp/actuators/servo_base.hpp>

#include <mirte_msgs/srv/set_servo_angle.hpp>
#include <mirte_msgs/srv/set_servo_pwm.hpp>
/* The Servo callback group can be reentrant (Parrallel), since the second
 * callback does not influence the hardware. */
ServoBase::ServoBase(NodeData node_data, std::vector<pin_t> pins,
                     ServoData servo_data,
                     rclcpp::CallbackGroupType callback_group_type)
    : TelemetrixDevice(node_data, pins, (DeviceData)servo_data,
                       callback_group_type),
      data(servo_data) {
  if (data.pin_mode == "motor") {
    this->setup_motor();
  } else if (data.pin_mode == "pwm") {
    this->setup_pwm();
  } else {
    this->setup_servo();
  }

  // make this service always available
  this->set_us_service = nh->create_service<mirte_msgs::srv::SetServoUS>(
      "servo/" + name + "/_set_us",
      std::bind(&ServoBase::set_us_service_callback, this,
                std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS().get_rmw_qos_profile(), this->callback_group);
}

/**********************
// SERVO MODE
*********************/
void ServoBase::setup_servo() {
  this->set_angle_service = nh->create_service<mirte_msgs::srv::SetServoAngle>(
      "servo/" + name + "/set_angle",
      std::bind(&ServoBase::set_angle_service_callback, this,
                std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS().get_rmw_qos_profile(), this->callback_group);

  this->get_range_service = nh->create_service<mirte_msgs::srv::GetServoRange>(
      "servo/" + name + "/get_range",
      std::bind(&ServoBase::get_range_service_callback, this,
                std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS().get_rmw_qos_profile(), this->callback_group);
}

float map(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
void ServoBase::set_angle_service_callback(
    mirte_msgs::srv::SetServoAngle::Request::ConstSharedPtr req,
    mirte_msgs::srv::SetServoAngle::Response::SharedPtr res) {
  float angle = req->angle;
  bool is_degrees = req->degrees;

  if (is_degrees == mirte_msgs::srv::SetServoAngle::Request::RADIANS) {
    angle = angle * (180.0 / std::numbers::pi);
  }

  if (angle > data.max_angle || angle < data.min_angle) {
    RCLCPP_WARN(logger,
                "The provided angle is out of range. Angle %.3f degrees was "
                "requested, but range is [%.3f, "
                "%.3f]",
                angle, data.min_angle, data.max_angle);
    res->status = false;
    return;
  }

  float fraction = map(angle, data.min_angle, data.max_angle, 0.0, 100.0);
  if (data.invert) {
    fraction = 100.0 - fraction;
  }

  uint16_t duty_cycle =
      (uint16_t)(map(fraction, 0.0, 100.0, data.min_pulse, data.max_pulse));
  RCLCPP_WARN(logger, "Setting servo %s to angle %.3f degrees (duty cycle: %u)",
              name.c_str(), angle, duty_cycle);
  res->status = set_angle_us(duty_cycle);
}

void ServoBase::get_range_service_callback(
    const mirte_msgs::srv::GetServoRange::Request::ConstSharedPtr req,
    mirte_msgs::srv::GetServoRange::Response::SharedPtr res) {
  res->max = data.max_angle;
  res->min = data.min_angle;
}

void ServoBase::set_us_service_callback(
    const mirte_msgs::srv::SetServoUS::Request::ConstSharedPtr req,
    mirte_msgs::srv::SetServoUS::Response::SharedPtr res) {
  uint16_t time_us = req->time_us;

  if (time_us > data.max_pulse || time_us < data.min_pulse) {
    RCLCPP_WARN(
        logger,
        "The provided time in microseconds is out of range. Time %u us "
        "was requested, but range is [%u, "
        "%u]. Set min_pulse and max_pulse parameters to change this range.",
        time_us, data.min_pulse, data.max_pulse);
    res->status = false;
    return;
  }

  RCLCPP_WARN(logger, "Setting servo %s to %u microseconds pulse width",
              name.c_str(), time_us);
  res->status = set_angle_us(time_us);
}

/**********************
// MOTOR MODE
*********************/
void ServoBase::setup_motor() {
  this->set_speed_service = nh->create_service<mirte_msgs::srv::SetMotorSpeed>(
      "servo/" + name + "/set_speed",
      std::bind(&ServoBase::set_speed_service_callback, this,
                std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS().get_rmw_qos_profile(), this->callback_group);
}

void ServoBase::set_speed_service_callback(
    const mirte_msgs::srv::SetMotorSpeed::Request::ConstSharedPtr req,
    mirte_msgs::srv::SetMotorSpeed::Response::SharedPtr res) {
  float speed = req->speed;
  RCLCPP_WARN(logger, "Setting motor %s to speed %.3f", name.c_str(), speed);
  set_speed(speed);
  res->status = true;
}

void ServoBase::set_speed(float speed) {
  // This is a simple linear mapping from speed to duty cycle.
  speed = std::clamp(speed, data.min_speed, data.max_speed);
  // linear map from [min_speed, max_speed] to [data.min_pulse, data.max_pulse]
  auto mins = data.min_speed;
  auto maxs = data.max_speed;
  if (data.invert) {
    mins = data.max_speed;
    maxs = data.min_speed;
  }

  float time = map(speed, mins, maxs, data.min_pulse, data.max_pulse);
  RCLCPP_WARN(logger, "Setting motor %s to time %.3f us", name.c_str(), time);

  this->set_angle_us((uint16_t)time);
}

/***
 * PWM MODE, only useful for PCA 'servos'.
 */
void ServoBase::setup_pwm() {
  this->set_pwm_service = nh->create_service<mirte_msgs::srv::SetServoPWM>(
      "servo/" + name + "/set_pwm",
      std::bind(&ServoBase::set_pwm_service_callback, this,
                std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS().get_rmw_qos_profile(), this->callback_group);
}

void ServoBase::set_pwm_service_callback(
    const mirte_msgs::srv::SetServoPWM::Request::ConstSharedPtr req,
    mirte_msgs::srv::SetServoPWM::Response::SharedPtr res) {
  float percentage = req->percentage;
  RCLCPP_WARN(logger, "Setting servo %s to %.3f percent", name.c_str(),
              percentage);
  set_percentage(percentage);
  res->status = true;
}