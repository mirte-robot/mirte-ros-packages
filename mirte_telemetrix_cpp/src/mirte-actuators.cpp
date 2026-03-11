#include <tmx_cpp/tmx.hpp>

#include <mirte_telemetrix_cpp/actuators/motor.hpp>
#include <mirte_telemetrix_cpp/actuators/neopixel.hpp>
#include <mirte_telemetrix_cpp/actuators/servo/servo.hpp>
#include <mirte_telemetrix_cpp/mirte-actuators.hpp>

Mirte_Actuators::Mirte_Actuators(NodeData node_data,
                                 std::shared_ptr<Parser> parser)
    : tmx(node_data.tmx), nh(node_data.nh), board(node_data.board),
      parser(parser), node_data(node_data) {}

void Mirte_Actuators::start() {
  using namespace std::placeholders;
  auto motors = Motor::get_motors(node_data, parser);
  this->motors = motors;
  this->actuators.insert(this->actuators.end(), motors.begin(), motors.end());
  auto servos = Servo::get_servos(node_data, parser);
  this->actuators.insert(this->actuators.end(), servos.begin(), servos.end());
  auto neopixels = Neopixel::get_neopixels(node_data, parser);
  this->actuators.insert(this->actuators.end(), neopixels.begin(),
                         neopixels.end());
  this->digital_pin_service =
      nh->create_service<mirte_msgs::srv::SetDigitalPinValue>(
          "set_digital_pin_value",
          std::bind(&Mirte_Actuators::digital_pin_service_callback, this, _1,
                    _2));
  this->pwm_pin_service = nh->create_service<mirte_msgs::srv::SetPWMPinValue>(
      "set_pwm_pin_value",
      std::bind(&Mirte_Actuators::pwm_pin_service_callback, this, _1, _2));

  this->set_multiple_motors_service =
      nh->create_service<mirte_msgs::srv::SetSpeedMultiple>(
          "set_multiple_motors_speed",
          std::bind(&Mirte_Actuators::set_multiple_motors_service_callback,
                    this, _1, _2));

  this->set_speed_multiple_subscription =
      nh->create_subscription<mirte_msgs::msg::SetSpeedMultiple>(
          "set_multiple_motors_speed", 1,
          [this](const mirte_msgs::msg::SetSpeedMultiple::SharedPtr msg) {
            // Create dummy request and response objects to pass to the service
            // callback
            auto req =
                std::make_shared<mirte_msgs::srv::SetSpeedMultiple::Request>();
            auto res =
                std::make_shared<mirte_msgs::srv::SetSpeedMultiple::Response>();
            req->speeds =
                msg->speeds; // Copy the speeds from the message to the request
            this->set_multiple_motors_service_callback(
                req, res); // Call the service callback with the dummy request
                           // and response
          });
}

void Mirte_Actuators::digital_pin_service_callback(
    const mirte_msgs::srv::SetDigitalPinValue::Request::ConstSharedPtr req,
    mirte_msgs::srv::SetDigitalPinValue::Response::SharedPtr res) {
  auto pin = this->board->resolvePin(req->pin);

  if (pin == -1) {
    // The Pin could not be resolved.
    res->status = false;
    res->message = "Pin '" + req->pin + "' could not be resolved";
    RCLCPP_ERROR(this->nh->get_logger(), "Pin '%s' could not be resolved",
                 req->pin.c_str());
    return;
  }

  this->tmx->setPinMode(pin, tmx_cpp::TMX::PIN_MODES::DIGITAL_OUTPUT, false, 0);
  // TODO: Maybe a sleep is required here?
  this->tmx->digitalWrite(pin, req->value);
  res->status = true;
}

void Mirte_Actuators::pwm_pin_service_callback(
    const mirte_msgs::srv::SetPWMPinValue::Request::ConstSharedPtr req,
    mirte_msgs::srv::SetPWMPinValue::Response::SharedPtr res) {
  auto pin = this->board->resolvePin(req->pin);

  if (pin == -1) {
    // The Pin could not be resolved.
    res->status = false;
    res->message = "Pin '" + req->pin + "' could not be resolved";
    RCLCPP_ERROR(this->nh->get_logger(), "Pin '%s' could not be resolved",
                 req->pin.c_str());
    return;
  }

  if (!this->board->is_pwm_pin(pin)) {
    // The Pin cannot be used as a PWM output.
    res->status = false;
    res->message = "Pin '" + req->pin + "' cannot be used as a PWM output";
    RCLCPP_ERROR(this->nh->get_logger(),
                 "Pin '%s' cannot be used as a PWM output", req->pin.c_str());
    return;
  }

  if (req->value > this->board->get_max_pwm()) {
    // The PWM value is out of range
    res->status = false;
    res->message = "The PWM value '" + std::to_string(req->value) +
                   "' requested for Pin '" + req->pin + "' is out of range(0-" +
                   std::to_string(this->board->get_max_pwm()) + ")";
    RCLCPP_ERROR(this->nh->get_logger(),
                 "The PWM value '%d' requested for Pin '%s' is out of range",
                 req->value, req->pin.c_str());
    return;
  }

  this->tmx->setPinMode(pin, tmx_cpp::TMX::PIN_MODES::PWM_OUTPUT, false, 0);
  // TODO: Maybe a sleep is required here?
  this->tmx->pwmWrite(pin, req->value);
  res->status = true;
}

void Mirte_Actuators::set_multiple_motors_service_callback(
    const mirte_msgs::srv::SetSpeedMultiple::Request::ConstSharedPtr req,
    mirte_msgs::srv::SetSpeedMultiple::Response::SharedPtr res) {
  // Only works for motors directly connected, not for pca motors.
  res->success = true;
  for (size_t i = 0; i < req->speeds.size(); i++) {
    auto motor_name = req->speeds[i].name;
    auto motor_speed = req->speeds[i].speed;
    auto motor_it = std::find_if(this->motors.begin(), this->motors.end(),
                                 [&](const std::shared_ptr<Motor> &motor) {
                                   return motor->name == motor_name;
                                 });
    if (motor_it != this->motors.end()) {
      (*motor_it)->set_speed(motor_speed);
    } else {
      RCLCPP_ERROR(this->nh->get_logger(), "Motor '%s' not found",
                   motor_name.c_str());
      res->success = false;
    }
  }
}