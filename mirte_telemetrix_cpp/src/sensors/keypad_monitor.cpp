#include <functional>
#include <memory>
#include <vector>

#include <rclcpp/node.hpp>

#include <mirte_telemetrix_cpp/sensors/keypad_monitor.hpp>

#include <mirte_msgs/msg/intensity.hpp>
#include <mirte_msgs/msg/keypad.hpp>
#include <mirte_msgs/srv/get_keypad.hpp>
#include <ranges>
// #include <view
std::vector<std::shared_ptr<KeypadMonitor>>
KeypadMonitor::get_keypad_monitors(NodeData node_data,
                                   std::shared_ptr<Parser> parser) {
  std::vector<std::shared_ptr<KeypadMonitor>> sensors;

  auto keypads =
      parser->params_object.keypad.keypads_map |
      std::views::transform([&](auto const &pair) {
        const auto &name = pair.first;
        const auto &map_keypad = pair.second;
        std::map<std::string, rclcpp::ParameterValue> parameters;
        parameters["device"] = rclcpp::ParameterValue(map_keypad.device);
        parameters["connector"] = rclcpp::ParameterValue(map_keypad.connector);
        parameters["pins.pin"] = rclcpp::ParameterValue(map_keypad.pins.pin);
        parameters["frame_id"] = rclcpp::ParameterValue(map_keypad.frame_id);
        auto unused_keys = get_keys(parameters);
        return KeypadData(parser, node_data.board, name, parameters,
                          unused_keys);
      }) |
      std::views::transform([&](const auto &data) {
        return std::make_shared<KeypadMonitor>(node_data, data);
      });
  sensors.assign(keypads.begin(), keypads.end());
  return sensors;
}

KeypadMonitor::KeypadMonitor(NodeData node_data, KeypadData keypad_data)
    : Mirte_Sensor(node_data, {keypad_data.pin}, (SensorData)keypad_data),
      keypad_data(keypad_data) {
  using namespace std::placeholders;

  // Use default QOS for sensor publishers as specified in REP2003
  keypad_pub = nh->create_publisher<mirte_msgs::msg::Keypad>(
      "keypad/" + keypad_data.name, rclcpp::SystemDefaultsQoS());
  keypad_pressed_pub = nh->create_publisher<mirte_msgs::msg::Keypad>(
      "keypad/" + keypad_data.name + "/pressed", rclcpp::SystemDefaultsQoS());

  keypad_service = nh->create_service<mirte_msgs::srv::GetKeypad>(
      "keypad/" + keypad_data.name + "/get_key",
      std::bind(&KeypadMonitor::keypad_service_callback, this, _1, _2),
      rclcpp::ServicesQoS().get_rmw_qos_profile(), this->callback_group);
  keypad_analog_pub = nh->create_publisher<mirte_msgs::msg::Intensity>(
      "keypad/" + keypad_data.name + "/analog", rclcpp::SystemDefaultsQoS());
  tmx->setPinMode(keypad_data.pin, tmx_cpp::TMX::PIN_MODES::ANALOG_INPUT, true,
                  0);
  tmx->add_analog_callback(
      keypad_data.pin, [this](auto pin, auto value) { this->callback(value); });
}

void KeypadMonitor::callback(uint16_t value) {
  // Prevent the callback from being executed.
  // TODO: locking
  Key key = Key::NONE;
  auto maxValue = std::pow(2, this->board->get_adc_bits()) - 1;

  auto scale = 1024.0 / maxValue;
  // RCLCPP_INFO(logger, "%d", this->value);
  if (value < 70 / scale) {
    key = LEFT;
  } else if (value < 230 / scale) {
    key = UP;
  } else if (value < 410 / scale) {
    key = DOWN;
  } else if (value < 620 / scale) {
    key = RIGHT;
  } else if (value < 880 / scale) {
    key = ENTER;
  }

  // Do some debouncing
  // This can happen here since key only changes in this function.
  if (this->last_key != key) {
    this->last_debounce_time = nh->now().seconds();
  }
  this->keypad_analog_pub->publish(
      mirte_msgs::build<mirte_msgs::msg::Intensity>()
          .header(get_header())
          .value(value));
  this->last_key = key;
}

void KeypadMonitor::keypad_service_callback(
    const mirte_msgs::srv::GetKeypad::Request::ConstSharedPtr req,
    mirte_msgs::srv::GetKeypad::Response::SharedPtr res) {
  res->data = key_string(this->last_debounced_key);
}

std::string KeypadMonitor::key_string(Key key) {
  switch (key) {
  case Key::LEFT:
    return "left";
  case Key::UP:
    return "up";
  case Key::DOWN:
    return "down";
  case Key::RIGHT:
    return "right";
  case Key::ENTER:
    return "enter";
  case Key::NONE:
  default:
    return "";
  }
}

void KeypadMonitor::update() {
  auto msg_builder =
      mirte_msgs::build<mirte_msgs::msg::Keypad>().header(get_header());

  Key debounced_key = NONE;
  if (nh->now().seconds() - this->last_debounce_time > 0.1) {
    debounced_key = this->last_key;
  }

  // Publish the last debounced key
  if (this->keypad_pub->get_subscription_count() > 0) {
    this->keypad_pub->publish(msg_builder.key(key_string(debounced_key)));
  }

  // # check if we need to send a pressed message
  if (this->last_debounced_key != NONE &&
      this->last_debounced_key != debounced_key) // TODO: check this
  {
    if (this->keypad_pub->get_subscription_count() > 0) {
      this->keypad_pressed_pub->publish(
          msg_builder.key(key_string(this->last_debounced_key)));
    }
  }

  this->last_debounced_key = debounced_key;
}
