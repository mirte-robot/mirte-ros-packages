#include <tmx_cpp/tmx.hpp>

#include <mirte_telemetrix_cpp/actuators/motor.hpp>
#include <mirte_telemetrix_cpp/actuators/servo/servo.hpp>
#include <mirte_telemetrix_cpp/mirte-actuators.hpp>

Mirte_Actuators::Mirte_Actuators(NodeData node_data,
                                 std::shared_ptr<Parser> parser)
    : tmx(node_data.tmx), nh(node_data.nh), board(node_data.board) {
  using namespace std::placeholders;

  this->actuators = Motor::get_motors(node_data, parser);

  auto servos = Servo::get_servos(node_data, parser);
  this->actuators.insert(this->actuators.end(), servos.begin(), servos.end());

  this->digital_pin_service =
      nh->create_service<mirte_msgs::srv::SetDigitalPinValue>(
          "set_digital_pin_value",
          std::bind(&Mirte_Actuators::digital_pin_service_callback, this, _1,
                    _2));
  this->pwm_pin_service = nh->create_service<mirte_msgs::srv::SetPWMPinValue>(
      "set_pwm_pin_value",
      std::bind(&Mirte_Actuators::pwm_pin_service_callback, this, _1, _2));
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
                   "' requested for Pin '" + req->pin + "' is out of range";
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