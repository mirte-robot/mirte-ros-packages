#include <functional>
#include <vector>

#include <boost/algorithm/string.hpp>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription_options.hpp>

#include <mirte_telemetrix_cpp/device.hpp>

#include <mirte_telemetrix_cpp/actuators/neopixel.hpp>
#include <mirte_telemetrix_cpp/neopixel_parameters.hpp>
#include <ranges>
using namespace std::placeholders;

std::vector<std::shared_ptr<TelemetrixDevice>>
Neopixel::get_neopixels(NodeData node_data, std::shared_ptr<Parser> parser) {
  std::vector<std::shared_ptr<TelemetrixDevice>> neopixels;
  auto modules = parser->update_params_list_type(
      "modules", "neopixel_module_names", "neopixel");
  parser->fix_param_type_str_modules("modules", modules, {"pins.data"});
  auto param_listener =
      std::make_shared<mirte_telemetrix_cpp_neopixel::ParamListener>(
          parser->nh);
  auto params = param_listener->get_params();
  // get modules list from parser
  // loop over items, get one with type==ina226
  // add paramlistener to those with new parameters yaml
  auto datas =
      params.modules.neopixel_module_names_map |
      std::views::filter([](const auto &pair) {
        const auto &map_power = pair.second;
        return boost::algorithm::to_lower_copy(map_power.type) == "neopixel";
      }) |
      std::views::transform([&](const auto &pair) {
        const auto &name = pair.first;
        const auto &map_neo = pair.second;
        std::map<std::string, rclcpp::ParameterValue> parameters;
        parameters["name"] = rclcpp::ParameterValue(name);
        parameters["type"] = rclcpp::ParameterValue(map_neo.type);
        // parameters["connector"] = rclcpp::ParameterValue(map_neo.connector);
        parameters["pins.data"] = rclcpp::ParameterValue(map_neo.pins.data);
        parameters["num_leds"] = rclcpp::ParameterValue(map_neo.num_pixels);
        parameters["default_r"] =
            rclcpp::ParameterValue(map_neo.default_color.r);
        parameters["default_g"] =
            rclcpp::ParameterValue(map_neo.default_color.g);
        parameters["default_b"] =
            rclcpp::ParameterValue(map_neo.default_color.b);
        std::set<std::string> unused_keys = get_keys(parameters);
        return NeopixelData(parser, node_data.board, name, parameters,
                            unused_keys);
      });

  for (auto data : datas) {
    if (neopixels.size() >= 1) {
      RCLCPP_ERROR(parser->nh->get_logger(),
                   "Multiple neopixel modules found, only one is supported, "
                   "skipping module with name: %s",
                   data.name.c_str());
      break;
    }
    auto module = std::make_shared<Neopixel>(node_data, data);
    neopixels.push_back(module);
  }
  return neopixels;
}

Neopixel::Neopixel(NodeData node_data, NeopixelData neopixel_data)
    : TelemetrixDevice(node_data, std::vector<uint8_t>{neopixel_data.data_pin},
                       (DeviceData)neopixel_data) {
  this->num_leds = neopixel_data.num_leds;
  this->data_pin = neopixel_data.data_pin;
  std::cout << "attachin neopixel with data pin: " << (int)this->data_pin
            << " and num leds: " << this->num_leds << std::endl;
  node_data.tmx->attach_neopixel(
      neopixel_data.data_pin, neopixel_data.num_leds, 0,
      {neopixel_data.default_r, neopixel_data.default_g,
       neopixel_data.default_b});

  this->set_neopixel_service = nh->create_service<mirte_msgs::srv::SetNeopixel>(
      "leds/" + this->name + "/set_color",
      std::bind(&Neopixel::set_color_service, this, _1, _2),
      rclcpp::ServicesQoS().get_rmw_qos_profile(), this->callback_group);
  this->set_neopixel_single_service =
      nh->create_service<mirte_msgs::srv::SetNeopixelSingle>(
          "leds/" + this->name + "/set_color_single",
          std::bind(&Neopixel::set_color_single_service, this, _1, _2),
          rclcpp::ServicesQoS().get_rmw_qos_profile(), this->callback_group);
}

Neopixel::~Neopixel() {
  // turn off all leds on destruction
  this->tmx->clear_neopixels();
}

void Neopixel::set_color_service(
    const mirte_msgs::srv::SetNeopixel::Request::ConstSharedPtr req,
    mirte_msgs::srv::SetNeopixel::Response::SharedPtr res) {
  //   std::vector<std::vector<uint8_t>> colors(this->num_leds, );
  this->tmx->fill_neopixels({req->color.r, req->color.g, req->color.b}, true);
  res->status = true;
}

void Neopixel::set_color_single_service(
    const mirte_msgs::srv::SetNeopixelSingle::Request::ConstSharedPtr req,
    mirte_msgs::srv::SetNeopixelSingle::Response::SharedPtr res) {
  if (req->led_index < 0 || req->led_index >= this->num_leds) {
    RCLCPP_ERROR(this->nh->get_logger(), "LED index out of range: %d",
                 req->led_index);
    res->status = false;
    return;
  }
  this->tmx->set_neopixel_color(
      req->led_index, {req->color.r, req->color.g, req->color.b}, true);
  res->status = true;
}

// Motor::Motor(NodeData node_data, std::vector<pin_t> pins, MotorData
// motor_data)
//     : Motor(node_data, pins, (DeviceData)motor_data, motor_data.inverted,
//             node_data.board->get_max_pwm()) {}

// Motor::Motor(NodeData node_data, std::vector<pin_t> pins, DeviceData data,
//              bool inverted, int max_pwm)
//     : TelemetrixDevice(node_data, pins, data,
//                        rclcpp::CallbackGroupType::MutuallyExclusive),
//       inverted(inverted), max_pwm(max_pwm) {
//   set_speed_service = nh->create_service<mirte_msgs::srv::SetMotorSpeed>(
//       "motor/" + this->name + "/set_speed",
//       std::bind(&Motor::set_speed_service_callback, this, _1, _2),
//       rclcpp::ServicesQoS().get_rmw_qos_profile(), this->callback_group);

//   rclcpp::SubscriptionOptions options;
//   options.callback_group = this->callback_group;
//   speed_subscription = nh->create_subscription<std_msgs::msg::Int32>(
//       "motor/" + this->name + "/speed", rclcpp::SystemDefaultsQoS(),
//       std::bind(&Motor::speed_subscription_callback, this, _1), options);

//   // this->device_timer->cancel();
// }

// void Motor::set_speed_service_callback(
//     const mirte_msgs::srv::SetMotorSpeed::Request::ConstSharedPtr req,
//     mirte_msgs::srv::SetMotorSpeed::Response::SharedPtr res) {
//   this->set_speed(req->speed);
//   res->status = true;
// }

// void Motor::speed_subscription_callback(const std_msgs::msg::Int32 &msg) {
//   this->set_speed(msg.data);
// }

NeopixelData::NeopixelData(
    std::shared_ptr<Parser> parser, std::shared_ptr<Mirte_Board> board,
    std::string name, std::map<std::string, rclcpp::ParameterValue> parameters,
    std::set<std::string> &unused_keys)
    : DeviceData(parser, board, name, "neopixel", parameters, unused_keys) {
  if (!unused_keys.erase("pins.data")) {
    throw std::runtime_error("Neopixel module " + name +
                             " is missing required parameter: pins.data");
  }
  this->data_pin = board->resolvePin(get_string(parameters["pins.data"]));

  if (!unused_keys.erase("num_leds")) {
    throw std::runtime_error("Neopixel module " + name +
                             " is missing required parameter: num_leds");
  }
  this->num_leds = parameters["num_leds"].get<int>();

  if (unused_keys.erase("default_color.r")) {
    this->default_r = parameters["default_color.r"].get<int>();
  } else {
    this->default_r = 0;
  }
  if (unused_keys.erase("default_color.g")) {
    this->default_g = parameters["default_color.g"].get<int>();
  } else {
    this->default_g = 0;
  }
  if (unused_keys.erase("default_color.b")) {
    this->default_b = parameters["default_color.b"].get<int>();
  } else {
    this->default_b = 0;
  }
}