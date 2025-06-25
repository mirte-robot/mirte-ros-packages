#include "mirte_telemetrix_cpp/mirte-sensors.hpp"

#include <mirte_telemetrix_cpp/sensors/encoder_monitor.hpp>
#include <mirte_telemetrix_cpp/sensors/intensity_monitor.hpp>
#include <mirte_telemetrix_cpp/sensors/keypad_monitor.hpp>
#include <mirte_telemetrix_cpp/sensors/sonar_monitor.hpp>

Mirte_Sensors::Mirte_Sensors(NodeData node_data, std::shared_ptr<Parser> parser)
    : tmx(node_data.tmx), nh(node_data.nh), board(node_data.board) {
  using namespace std::placeholders;

  auto keypads = KeypadMonitor::get_keypad_monitors(node_data, parser);
  this->sensors.insert(this->sensors.end(), keypads.begin(), keypads.end());

  auto sonars = SonarMonitor::get_sonar_monitors(node_data, parser);
  this->sensors.insert(this->sensors.end(), sonars.begin(), sonars.end());

  auto irs = IntensityMonitor::get_intensity_monitors(node_data, parser);
  this->sensors.insert(this->sensors.end(), irs.begin(), irs.end());

  auto encoders = EncoderMonitor::get_encoder_monitors(node_data, parser);
  this->sensors.insert(this->sensors.end(), encoders.begin(), encoders.end());

  this->digital_pin_service =
      nh->create_service<mirte_msgs::srv::GetDigitalPinValue>(
          "get_digital_pin_value",
          std::bind(&Mirte_Sensors::digital_pin_service_callback, this, _1,
                    _2));
  this->analog_pin_service =
      nh->create_service<mirte_msgs::srv::GetAnalogPinValue>(
          "get_analog_pin_value",
          std::bind(&Mirte_Sensors::analog_pin_service_callback, this, _1, _2));
}

void Mirte_Sensors::digital_pin_service_callback(
    const mirte_msgs::srv::GetDigitalPinValue::Request::ConstSharedPtr req,
    mirte_msgs::srv::GetDigitalPinValue::Response::SharedPtr res) {
  using namespace std::chrono_literals;

  auto pin = this->board->resolvePin(req->pin);

  if (pin == -1) {
    // The Pin could not be resolved.
    res->status = false;
    res->message = "Pin '" + req->pin + "' could not be resolved";
    RCLCPP_ERROR(this->nh->get_logger(), "Pin '%s' could not be resolved",
                 req->pin.c_str());
    return;
  }

  bool has_analog_cb = false;
  bool has_digital_cb = false;

  if (this->pin_map.count(pin)) {
    const auto [type, value, ana_cb, dig_cb] = this->pin_map[pin];
    has_analog_cb = ana_cb;
    has_digital_cb = dig_cb;

    if (type == PIN_USE::DIGITAL_IN && value != -1) {
      res->status = true;
      res->value = value != 0;
      return;
    }
  }

  this->pin_map[pin] = {PIN_USE::DIGITAL_IN, -1, has_analog_cb, true};

  this->tmx->setPinMode(pin, tmx_cpp::TMX::PIN_MODES::DIGITAL_INPUT, true);

  if (!has_digital_cb) {
    RCLCPP_INFO(this->nh->get_logger(), "Add Digital Callback to Pin %d", pin);

    this->tmx->add_digital_callback(pin, [this](auto pin, auto value) {
      const auto [type, old_value, has_analog, has_digital] =
          this->pin_map[pin];
      if (type == PIN_USE::DIGITAL_IN && old_value != value) {
        this->pin_map[pin] = {type, value, has_analog, has_digital};
      }
    });
  }

  // TODO: There are probably nicer ways to do this
  // while time less than 5s, after that its probably not going to change
  auto start = std::chrono::system_clock::now();
  while (std::chrono::system_clock::now() - start < 5s) {
    const auto [type, value, has_analog, has_digital] = this->pin_map[pin];
    if (value != -1) {
      res->status = true;
      res->value = value != 0;
      return;
    }

    // sleep for 1ms
    std::this_thread::sleep_for(1ms);
  }
  res->status = false;
  res->message = "No pin callback recieved in the last 5 seconds";
}

void Mirte_Sensors::analog_pin_service_callback(
    const mirte_msgs::srv::GetAnalogPinValue::Request::ConstSharedPtr req,
    mirte_msgs::srv::GetAnalogPinValue::Response::SharedPtr res) {
  using namespace std::chrono_literals;

  auto pin = this->board->resolvePin(req->pin);

  if (pin == -1) {
    // The Pin could not be resolved.
    res->status = false;
    res->message = "Pin '" + req->pin + "' could not be resolved";
    RCLCPP_ERROR(this->nh->get_logger(), "Pin '%s' could not be resolved",
                 req->pin.c_str());
    return;
  }

  if (!this->board->is_analog_pin(pin)) {
    // The Pin cannot be used as an analog input
    res->status = false;
    res->message = "Pin '" + req->pin + "' cannot be used as an analog input";
    RCLCPP_ERROR(this->nh->get_logger(),
                 "Pin '%s' cannot be used as an analog input",
                 req->pin.c_str());
    return;
  }

  bool has_analog_cb = false;
  bool has_digital_cb = false;

  if (this->pin_map.count(pin)) {
    const auto [type, value, ana_cb, dig_cb] = this->pin_map[pin];
    has_analog_cb = ana_cb;
    has_digital_cb = dig_cb;

    if (type == PIN_USE::ANALOG_IN && value != -1) {
      res->status = true;
      res->value = value;
      return;
    }
  }

  this->pin_map[pin] = {PIN_USE::ANALOG_IN, -1, true, has_digital_cb};

  this->tmx->setPinMode(pin, tmx_cpp::TMX::PIN_MODES::ANALOG_INPUT, true, 0);
  if (!has_analog_cb) {
    RCLCPP_INFO(this->nh->get_logger(), "Add Analog Callback to Pin %d", pin);

    this->tmx->add_analog_callback(pin, [this](auto pin, auto value) {
      const auto [type, old_value, has_analog, has_digital] =
          this->pin_map[pin];
      if (type == PIN_USE::ANALOG_IN && old_value != value) {
        this->pin_map[pin] = {type, value, has_analog, has_digital};
      }
    });
  }

  // TODO: There are probably nicer ways to do this
  // while time less than 5s, after that its probably not going to change
  auto start = std::chrono::system_clock::now();
  while (std::chrono::system_clock::now() - start < 5s) {
    const auto [type, value, has_analog, has_digital] = this->pin_map[pin];
    if (value != -1) {
      res->status = true;
      res->value = value;
      return;
    }

    // sleep for 1ms
    std::this_thread::sleep_for(1ms);
  }
  res->status = false;
  res->message = "No pin callback recieved in the last 5 seconds";
}

void Mirte_Sensors::stop() {
  for (auto sensor : this->sensors) {
    sensor->stop();
  }
}
