#include <chrono>
#include <functional>
#include <ranges>
#include <thread>

using namespace std::chrono_literals;

#include <rclcpp/callback_group.hpp>

#include <mirte_telemetrix_cpp/hiwonder_parameters.hpp>
#include <mirte_telemetrix_cpp/hiwonder_servo_parameters.hpp>
#include <mirte_telemetrix_cpp/modules/hiwonder_module.hpp>
using namespace std::placeholders; // for _1, _2, _3...

// TODO: USE TO DEVICE_TIMER

// hiwonder bus
// TODO: Maybe add a lock future, to prevent outputting warnings during other
// modules...?
HiWonderBus_module::HiWonderBus_module(
    NodeData node_data, HiWonderBusData bus_data,
    std::shared_ptr<tmx_cpp::Modules> modules)
    : Mirte_module(node_data, {bus_data.tx_pin, bus_data.rx_pin},
                   (ModuleData)bus_data,
                   rclcpp::CallbackGroupType::MutuallyExclusive),
      data(bus_data) {
  // this->device_timer->cancel();
  this->logger =
      this->logger.get_child(data.get_device_class()).get_child(data.name);

  // Create a list of ID's
  std::vector<uint8_t> servo_ids;
  // Don't pre-add ids since it can cause errors on missing servos
  // for (auto servo : this->data.servos)
  //   servo_ids.push_back(servo->id);

  this->bus = std::make_shared<tmx_cpp::HiwonderServo_module>(
      this->data.uart_port, this->data.rx_pin, this->data.tx_pin, servo_ids,
      std::bind(&HiWonderBus_module::position_cb, this, _1));

  modules->add_mod(this->bus);

  auto servo_group = this->data.group_name;
  if (!servo_group.ends_with('/')) {
    servo_group.push_back('/');
  }

  std::this_thread::sleep_for(1.20s);
  for (auto servo_data : this->data.servos) {
    if (this->bus->verify_id(servo_data->id)) {
      this->servos.push_back(std::make_shared<Hiwonder_servo>(
          node_data, servo_data, this->bus, servo_group, bus_data.duration,
          this->callback_group));
    } else {
      RCLCPP_ERROR(
          this->logger,
          "HiWonder Servo '%s' is ignored as its ID [%d] was not found.",
          servo_data->name.c_str(), servo_data->id);
    }
  }

  // Create Bus ROS services
  this->enable_all_servos_service = nh->create_service<std_srvs::srv::SetBool>(
      "servo/" + servo_group + "enable_all_servos",
      std::bind(&HiWonderBus_module::enable_service_callback, this, _1, _2),
      rclcpp::ServicesQoS().get_rmw_qos_profile(), this->callback_group);

  this->angle_service = nh->create_service<mirte_msgs::srv::SetAngleMultiple>(
      "servo/" + servo_group + "set_multiple_angles",
      std::bind(&HiWonderBus_module::set_angle_multiple_service_callback, this,
                _1, _2),
      rclcpp::ServicesQoS().get_rmw_qos_profile(), this->callback_group);

  rclcpp::SubscriptionOptions options;
  options.callback_group = this->callback_group;
  this->angle_callback =
      nh->create_subscription<mirte_msgs::msg::SetAngleMultiple>(
          "servo/" + servo_group + "set_multiple_angles_callback", 1,
          [this](const mirte_msgs::msg::SetAngleMultiple::SharedPtr msg) {
            // Create dummy request and response objects
            auto req =
                std::make_shared<mirte_msgs::srv::SetAngleMultiple::Request>();
            auto res =
                std::make_shared<mirte_msgs::srv::SetAngleMultiple::Response>();

            // Fill the request with data from the message
            req->angles = msg->angles;

            // Call the service callback directly
            this->set_angle_multiple_service_callback(req, res);
          },
          options);
}

void HiWonderBus_module::set_angle_multiple_service_callback(
    const mirte_msgs::srv::SetAngleMultiple::Request::ConstSharedPtr req,
    mirte_msgs::srv::SetAngleMultiple::Response::SharedPtr res) {
  bool success = true;
  for (const auto &angle_named : req->angles) {
    auto servo_it = std::find_if(
        this->servos.begin(), this->servos.end(),
        [&angle_named](const std::shared_ptr<Hiwonder_servo> &servo) {
          return servo->servo_data->name == angle_named.name;
        });

    if (servo_it != this->servos.end()) {
      success &= (*servo_it)->set_angle(angle_named.angle);
    } else {
      RCLCPP_WARN(this->logger, "Servo with name '%s' not found.",
                  angle_named.name.c_str());
      success = false;
    }
  }
  res->success = success;
}

// TODO: Make result actually Reflect reality
void HiWonderBus_module::enable_service_callback(
    const std_srvs::srv::SetBool::Request::ConstSharedPtr req,
    std_srvs::srv::SetBool::Response::SharedPtr res) {
  // // TODO: TEMP TEST
  // std::cout << (int)bus->get_offset(4).value_or(99) <<std::endl;
  // auto [min, max] = bus->get_range(4).value_or(std::make_tuple(0,0));
  // std::cout << min << " | " << max <<std::endl;
  // std::cout << bus->verify_id(4) << std::endl;
  // // TODO: TEMP TEST

  res->success = this->bus->set_enabled_all(req->data);
  res->message = req->data ? "Enabled" : "Disabled";
}

std::vector<std::shared_ptr<HiWonderBus_module>>
HiWonderBus_module::get_hiwonder_modules(
    NodeData node_data, std::shared_ptr<Parser> parser,
    std::shared_ptr<tmx_cpp::Modules> modules) {
  std::vector<std::shared_ptr<HiWonderBus_module>> hiwonder_modules;
  auto found_modules = parser->update_params_list_type(
      "modules", "hiwonder_module_names", "Hiwonder_Servo");
  parser->fix_param_type_str_modules("modules", found_modules,
                                     {"pins.rx", "pins.tx"});

  for (auto &found_module : found_modules) {
    std::cout << "found hiwonder module!!!" << std::endl;
    auto module_name = fmt::format("modules.{}", found_module);
    // no need to filter here, just get all names
    auto servo_names = parser->update_params_list(
        module_name + ".servos", module_name + ".hiwonder_servo_names",
        [](const std::string &name) { return true; });
    //  parser->fix_param_type_str_modules("modules."+module_name, servo_names,
    //  {"id", "min_angle_out", "max_angle_out", "home_out", "invert"});
  }
  auto param_listener =
      std::make_shared<mirte_telemetrix_cpp_hiwonder::ParamListener>(
          parser->nh);
  auto params = param_listener->get_params();
  // get modules list from parser
  // loop over items, get one with type==ina226
  // add paramlistener to those with new parameters yaml
  auto datas =
      params.modules.hiwonder_module_names_map |
      std::views::filter([](const auto &pair) {
        const auto &map_power = pair.second;
        return boost::algorithm::to_lower_copy(map_power.type) ==
               "hiwonder_servo";
      }) |
      std::views::transform([&](const auto &pair) {
        const auto &name = pair.first;
        const auto &map_ina = pair.second;
        std::map<std::string, rclcpp::ParameterValue> parameters;
        parameters["type"] = rclcpp::ParameterValue(map_ina.type);

        parameters["pins.rx"] = rclcpp::ParameterValue(map_ina.pins.rx);
        parameters["pins.tx"] = rclcpp::ParameterValue(map_ina.pins.tx);
        // parameters["connector"] = rclcpp::ParameterValue(map_ina.connector);
        // parameters["addr"] = rclcpp::ParameterValue(map_ina.addr);

        auto param_listener_motors = std::make_shared<
            mirte_telemetrix_cpp_hiwonder_servo::ParamListener>(
            parser->nh, fmt::format("modules.{}", name));
        auto params_motors = param_listener_motors->get_params();
        std::vector<std::shared_ptr<HiWonderServoData>> motor_data;
        std::cout << "Parsing motors for PCA module: " << name << std::endl;

        for (auto const &servo_name : params_motors.hiwonder_servo_names) {
          auto motor_params =
              params_motors.servos.hiwonder_servo_names_map.at(servo_name);
          std::map<std::string, rclcpp::ParameterValue> parameters = {
              {"id", rclcpp::ParameterValue(motor_params.id)},
              {"min_angle_out",
               rclcpp::ParameterValue(motor_params.min_angle_out)},
              {"max_angle_out",
               rclcpp::ParameterValue(motor_params.max_angle_out)},
              {"home_out", rclcpp::ParameterValue(motor_params.home_out)},
              {"invert", rclcpp::ParameterValue(motor_params.invert)}};

          std::set<std::string> unused_keys = get_keys(parameters);
          std::shared_ptr<HiWonderServoData> data =
              std::make_shared<HiWonderServoData>(parser, node_data.board,
                                                  servo_name, parameters,
                                                  unused_keys, "");
          motor_data.push_back(data);
        }
        std::set<std::string> unused_keys = get_keys(parameters);
        return HiWonderBusData(parser, node_data.board, name, parameters,
                               unused_keys, motor_data);
      });

  for (auto pca : datas) {
    auto pca_module =
        std::make_shared<HiWonderBus_module>(node_data, pca, modules);
    hiwonder_modules.push_back(pca_module);
  }

  // // auto hiwonder_data =
  // //     parse_all_modules<HiWonderBusData>(parser, node_data.board);
  // for (auto hiwonder : hiwonder_data) {
  //   auto hiwonder_module =
  //       std::make_shared<HiWonderBus_module>(node_data, hiwonder, modules);
  //   hiwonder_modules.push_back(hiwonder_module);
  // }
  return hiwonder_modules;
}

void HiWonderBus_module::position_cb(
    std::vector<std::tuple<uint8_t, tmx_cpp::HiwonderServo_module::Servo_pos>>
        pos) {
  for (auto &[idx, p] : pos) {
    if (idx >= this->servos.size()) {
      continue;
    }
    auto servo = this->servos[idx];
    assert(servo->servo_data->id == p.id);
    servo->position_cb(p);
  }
}
