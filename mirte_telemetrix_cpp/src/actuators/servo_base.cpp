#include <algorithm>
#include <functional>
#include <numbers>

#include <rclcpp/callback_group.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tmx_cpp/tmx.hpp>

#include <mirte_telemetrix_cpp/actuators/servo_base.hpp>

#include <mirte_msgs/srv/set_servo_angle.hpp>

/* The Servo callback group can be reentrant (Parrallel), since the second
 * callback does not influence the hardware. */
ServoBase::ServoBase(NodeData node_data, std::vector<pin_t> pins,
                     ServoData servo_data,
                     rclcpp::CallbackGroupType callback_group_type)
    : TelemetrixDevice(node_data, pins, (DeviceData)servo_data,
                       callback_group_type),
      data(servo_data) {
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
  this->set_us_service = nh->create_service<mirte_msgs::srv::SetServoUS>(
      "servo/" + name + "/_set_us",
      std::bind(&ServoBase::set_us_service_callback, this,
                std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS().get_rmw_qos_profile(), this->callback_group);
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

  float fraction = (angle - data.min_angle) / (data.max_angle - data.min_angle);
  uint16_t duty_cycle =
      (uint16_t)(fraction * (data.max_pulse - data.min_pulse)) + data.min_pulse;
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