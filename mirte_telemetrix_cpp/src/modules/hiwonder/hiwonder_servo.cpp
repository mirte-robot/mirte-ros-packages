#include <functional>
#include <numbers>

#include <mirte_telemetrix_cpp/modules/hiwonder/hiwonder_servo.hpp>

// NOTE: Uses a timer per servo instead of the device timer of the bus

// TODO: Maybe make it inherit from Servo
Hiwonder_servo::Hiwonder_servo(
    NodeData node_data, std::shared_ptr<HiWonderServoData> servo_data,
    std::shared_ptr<tmx_cpp::HiwonderServo_module> bus_mod,
    std::string servo_group, DeviceData::DeviceDuration duration,
    rclcpp::CallbackGroup::SharedPtr callback_group)
    : servo_data(servo_data), bus_mod(bus_mod), nh(node_data.nh) {
  using namespace std::placeholders;
  auto logger = nh->get_logger();

  if (!this->bus_mod->verify_id(this->servo_data->id))
    RCLCPP_ERROR(logger,
                 "HiWonder Servo '%s' ID not present. [Expected ID %d, but was "
                 "not found]",
                 this->servo_data->name.c_str(), this->servo_data->id);

  if (this->bus_mod->register_servo_id(this->servo_data->id))
    RCLCPP_ERROR(logger,
                 "HiWonder Servo '%s' ID is out of range [Requesed ID %d, but "
                 "range is 0-253]",
                 this->servo_data->name.c_str(), this->servo_data->id);

  auto range = this->bus_mod->get_range(servo_data->id);
  assert(range.has_value());
  auto [lower, upper] = range.value();
  if (lower != this->servo_data->min_angle_out)
    RCLCPP_WARN(logger,
                "HiWonder Servo '%s' lower range does not match the config. "
                "[Expected %d , Actual %d]",
                this->servo_data->name.c_str(), this->servo_data->min_angle_out,
                lower);

  if (upper != this->servo_data->max_angle_out)
    RCLCPP_WARN(logger,
                "HiWonder Servo '%s' upper range does not match the config. "
                "[Expected %d , Actual %d]",
                this->servo_data->name.c_str(), this->servo_data->max_angle_out,
                upper);

  // create enable service
  this->enable_service = nh->create_service<std_srvs::srv::SetBool>(
      "servo/" + servo_group + this->servo_data->name + "/set_enable",
      std::bind(&Hiwonder_servo::enable_service_callback, this, _1, _2),
      rclcpp::ServicesQoS().get_rmw_qos_profile(), callback_group);

  // create angle service
  this->angle_service = nh->create_service<mirte_msgs::srv::SetServoAngle>(
      "servo/" + servo_group + this->servo_data->name + "/set_angle",
      std::bind(&Hiwonder_servo::set_angle_service_callback, this, _1, _2),
      rclcpp::ServicesQoS().get_rmw_qos_profile(), callback_group);

  // create angle speed service
  this->angle_speed_service =
      nh->create_service<mirte_msgs::srv::SetServoAngleWithSpeed>(
          "servo/" + servo_group + this->servo_data->name +
              "/set_angle_with_speed",
          std::bind(&Hiwonder_servo::set_angle_with_speed_service_callback,
                    this, _1, _2),
          rclcpp::ServicesQoS().get_rmw_qos_profile(), callback_group);

  // create range service
  this->range_service = nh->create_service<mirte_msgs::srv::GetServoRange>(
      "servo/" + servo_group + this->servo_data->name + "/get_range",
      std::bind(&Hiwonder_servo::get_range_service_callback, this, _1, _2),
      rclcpp::ServicesQoS().get_rmw_qos_profile(), callback_group);

  // create publisher
  // Use default QoS for sensor publishers as specified in REP2003
  this->position_pub = nh->create_publisher<mirte_msgs::msg::ServoPosition>(
      "servo/" + servo_group + this->servo_data->name + "/position",
      rclcpp::SystemDefaultsQoS());

  // TODO: Maybe add to a separate callbackgroup?
  // Currently overpublishing slightly
  this->servo_timer = nh->create_wall_timer(
      duration, std::bind(&Hiwonder_servo::update, this), callback_group);

  if (this->servo_data->enable_motor) {
    // this->bus_mod->motor_mode_write(this->servo_data->id, 1);
    this->motor_speed_service =
        nh->create_service<mirte_msgs::srv::SetMotorSpeed>(
            "servo/" + servo_group + this->servo_data->name +
                "/set_motor_speed",
            std::bind(&Hiwonder_servo::set_motor_speed_service_callback, this,
                      _1, _2),
            rclcpp::ServicesQoS().get_rmw_qos_profile(), callback_group);
  }
}

// TODO: Add update en fix time
void Hiwonder_servo::position_cb(
    tmx_cpp::HiwonderServo_module::Servo_pos &pos) {
  servo_timer->call();
  last_angle = calc_angle_in(pos.angle);
  last_raw = pos.angle;
  this->update();
  servo_timer->reset();
}

void Hiwonder_servo::update() {
  auto msg = mirte_msgs::build<mirte_msgs::msg::ServoPosition>()
                 .header(this->get_header())
                 .angle(this->last_angle)
                 .raw(this->last_raw);
  this->position_pub->publish(msg);
}

void Hiwonder_servo::enable_service_callback(
    const std_srvs::srv::SetBool::Request::ConstSharedPtr req,
    std_srvs::srv::SetBool::Response::SharedPtr res) {
  res->success =
      this->bus_mod->set_enable_servo(this->servo_data->id, req->data);
  res->message = req->data ? "Enabled" : "Disabled";
}

void Hiwonder_servo::set_angle_service_callback(
    const mirte_msgs::srv::SetServoAngle::Request::ConstSharedPtr req,
    mirte_msgs::srv::SetServoAngle::Response::SharedPtr res) {
  float angle = req->angle;
  bool is_degrees = req->degrees;

  if (is_degrees == mirte_msgs::srv::SetServoAngle::Request::DEGREES) {
    angle = angle * (std::numbers::pi / 180.0);
  }

  if (angle > servo_data->max_angle_in || angle < servo_data->min_angle_in) {
    RCLCPP_ERROR(nh->get_logger(),
                 "The provided angle is out of range. Angle %.3f radians was "
                 "requested, but range is [%.3f, "
                 "%.3f]",
                 angle, servo_data->min_angle_in, servo_data->max_angle_in);
    res->status = false;
    return;
  }

  auto angle_out = calc_angle_out(angle);
  res->status =
      this->bus_mod->set_single_servo(this->servo_data->id, angle_out, 100);
}

void Hiwonder_servo::set_angle_with_speed_service_callback(
    const mirte_msgs::srv::SetServoAngleWithSpeed::Request::ConstSharedPtr req,
    mirte_msgs::srv::SetServoAngleWithSpeed::Response::SharedPtr res) {
  using std::numbers::pi;

  float angle = req->angle;
  float speed = req->rate;
  bool is_degrees = req->degrees;

  // Require speed to be positive, since sending 0.0 rad/s results in moving at
  // the max speed
  if (!speed > 0.0) {
    RCLCPP_ERROR(
        nh->get_logger(),
        "Speed must be positive. Provided speed was non-positive (%.3f <= 0.0)",
        speed);
    res->status = false;
    return;
  }

  if (is_degrees == mirte_msgs::srv::SetServoAngleWithSpeed::Request::DEGREES) {
    angle = angle * (pi / 180.0);
    speed = speed * (pi / 180.0);
  }

  if (angle > servo_data->max_angle_in || angle < servo_data->min_angle_in) {
    RCLCPP_ERROR(nh->get_logger(),
                 "The provided angle is out of range. Angle %.3f radians was "
                 "requested, but range is [%.3f, "
                 "%.3f]",
                 angle, servo_data->min_angle_in, servo_data->max_angle_in);
    res->status = false;
    return;
  }

  // Distance calculation based on: https://stackoverflow.com/a/52432897
  auto distance =
      pi - std::abs(std::fmod(std::abs(angle - last_angle), 2.0 * pi) - pi);
  auto time = distance / speed;
  RCLCPP_DEBUG(nh->get_logger(), "TIME: %.3f", time);
  uint16_t time_ms = time * 1000.0;

  if (time > 30.0) {
    std::string unit = is_degrees ? "°" : "rad";
    RCLCPP_ERROR(nh->get_logger(),
                 "The speed angle combo (%.1f%s, %.2f%s/s) results in a move "
                 "time longer than 30s from the "
                 "current position. (%.2f > 30s)",
                 req->angle, unit.c_str(), req->rate, unit.c_str(), time);
    res->status = false;
    return;
  }

  // NOTE: When time_ms is 0 than the servo attemps to move as fast as possible.

  auto angle_out = calc_angle_out(angle);

  RCLCPP_DEBUG(nh->get_logger(), "Moving Servo from %d to %d in %dms",
               (int)last_raw, (int)angle_out, (int)time_ms);

  res->status =
      this->bus_mod->set_single_servo(this->servo_data->id, angle_out, time_ms);
}

void Hiwonder_servo::get_range_service_callback(
    const mirte_msgs::srv::GetServoRange::Request::ConstSharedPtr req,
    mirte_msgs::srv::GetServoRange::Response::SharedPtr res) {
  res->min = this->servo_data->min_angle_in;
  res->max = this->servo_data->max_angle_in;
}

template <typename T> T scale(T x, T in_min, T in_max, T out_min, T out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

uint16_t Hiwonder_servo::calc_angle_out(float angle) {
  float angle_out = scale<float>(
      angle, this->servo_data->min_angle_in, this->servo_data->max_angle_in,
      this->servo_data->min_angle_out, this->servo_data->max_angle_out);
  if (this->servo_data->invert) {
    angle_out = scale<float>(
        angle, this->servo_data->min_angle_in, this->servo_data->max_angle_in,
        this->servo_data->max_angle_out, this->servo_data->min_angle_out);
  }
  return std::max(std::min((int)angle_out, this->servo_data->max_angle_out),
                  this->servo_data->min_angle_out);
}

float Hiwonder_servo::calc_angle_in(uint16_t angle) {
  float angle_in = scale<float>(
      angle, this->servo_data->min_angle_out, this->servo_data->max_angle_out,
      this->servo_data->min_angle_in, this->servo_data->max_angle_in);
  if (this->servo_data->invert) {
    angle_in = scale<float>(
        angle, this->servo_data->max_angle_out, this->servo_data->min_angle_out,
        this->servo_data->min_angle_in, this->servo_data->max_angle_in);
  }
  return angle_in;
}

std_msgs::msg::Header Hiwonder_servo::get_header() {
  return std_msgs::build<std_msgs::msg::Header>()
      .stamp(this->nh->now())
      .frame_id(this->servo_data->frame_id);
}

void Hiwonder_servo::set_motor_speed_service_callback(
    const mirte_msgs::srv::SetMotorSpeed::Request::ConstSharedPtr req,
    mirte_msgs::srv::SetMotorSpeed::Response::SharedPtr res) {
  // input: speed from -100 to 100
  // output: speed from -1000 to 1000
  int16_t speed = std::clamp(req->speed, -100, 100) * 10;
  res->status = this->bus_mod->motor_mode_write(this->servo_data->id, speed);
}
