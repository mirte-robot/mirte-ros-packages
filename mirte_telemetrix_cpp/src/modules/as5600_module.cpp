#include <functional>
#include <stdint.h>

#include <chrono>

using namespace std::chrono_literals;

#include <mirte_telemetrix_cpp/as5600_parameters.hpp>
#include <mirte_telemetrix_cpp/as5600_sensor_parameters.hpp>
#include <mirte_telemetrix_cpp/modules/as5600_module.hpp>
#include <ranges>
using namespace std::placeholders; // for _1, _2, _3...

AS5600_sensor::AS5600_sensor(NodeData node_data, AS5600Data as5600_data,
                             std::shared_ptr<tmx_cpp::Sensors> sensors)
    : Mirte_module(node_data, {as5600_data.scl, as5600_data.sda},
                   (ModuleData)as5600_data),
      data(as5600_data) {
  tmx->setI2CPins(as5600_data.sda, as5600_data.scl, as5600_data.port);
  for (const auto &encoder : as5600_data.encoders) {
    channels.push_back(std::get<0>(encoder));
    auto pub = node_data.nh->create_publisher<std_msgs::msg::Float32>(
        "angle/" + as5600_data.name + "/" + std::get<1>(encoder),
        rclcpp::SystemDefaultsQoS());
    this->angle_pubs.push_back(pub);
    this->angles.push_back(0.0f);
    this->angle_msgs.push_back(std_msgs::msg::Float32());
  }
  this->as5600 = std::make_shared<tmx_cpp::AS5600_tmx_sensor>(
      as5600_data.port, as5600_data.addr, channels,
      std::bind(&AS5600_sensor::data_callback, this, _1));

  sensors->add_sens(this->as5600);
  // add timer
  node_data.nh->create_wall_timer(100ms,
                                  std::bind(&AS5600_sensor::update, this));
}

// TODO: Maybe add mutex lock-out although not fully necessary
void AS5600_sensor::update() {
  auto header = get_header();
  for (int i = 0; i < this->angles.size(); i++) {
    auto &msg = angle_msgs[i];
    // msg.header = header;
    msg.data = this->angles[i];
    angle_pubs[i]->publish(msg);
  }
}

void AS5600_sensor::data_callback(tmx_cpp::AS5600_cb_t::argument_type data) {
  // std::cout << "AS5600 data callback with " << data.size() << " entries"
  //           << std::endl;
  for (int i = 0; i < data.size(); i++) {
    auto &[channel, angle_rad, angle_ticks] = data[i];
    // std::cout << "  Channel: " << channel
    //           << " Angle (rad): " << angle_rad
    //           << " Angle (ticks): " << angle_ticks << std::endl;
    auto channel_index =
        std::find_if(channels.begin(), channels.end(),
                     [channel](const int &elem) { return elem == channel; });
    if (channel_index == channels.end()) {
      // std::cerr << "Channel " << channel << " not found in data"
      //           << std::endl;
      continue;
    }
    auto index = std::distance(channels.begin(), channel_index);
    if (index < 0 || index >= this->angles.size()) {
      // std::cerr << "Index " << index << " out of bounds for angles"
      //           << std::endl;
      continue;
    }
    // std::cout << "  Updating angle at index " << index << std::endl;
    this->angles[index] = angle_rad;
  }
  // this->update();
}

std::vector<std::shared_ptr<AS5600_sensor>>
AS5600_sensor::get_as5600_modules(NodeData node_data,
                                  std::shared_ptr<Parser> parser,
                                  std::shared_ptr<tmx_cpp::Sensors> sensors) {
  // auto as5600_modules = parse_all_modules<AS5600Data>(parser,
  // node_data.board);
  std::vector<std::shared_ptr<AS5600_sensor>> as_modules;
  auto found_modules = parser->update_params_list_type(
      "modules", "as5600_module_names", "as5600_M");
  // as pca has nested lists, rerun for motors struct as well
  for (auto &found_module : found_modules) {
    // std::cout << "found pca module!!!" << std::endl;
    auto module_name = fmt::format("modules.{}", found_module);
    // no need to filter here, just get all names
    parser->update_params_list(module_name + ".encoders",
                               module_name + ".as5600_sensor_names",
                               [](const std::string &name) { return true; });
    // parser->update_params_list(module_name + ".servos",
    //                            module_name + ".pca_servo_names",
    //                            [](const std::string &name) { return true; });
  }
  auto param_listener =
      std::make_shared<mirte_telemetrix_cpp_as5600::ParamListener>(parser->nh);
  auto params = param_listener->get_params();
  // get modules list from parser
  // loop over items, get one with type==ina226
  // add paramlistener to those with new parameters yaml
  auto datas =
      params.modules.as5600_module_names_map |
      std::views::filter([](const auto &pair) {
        const auto &map_power = pair.second;
        return boost::algorithm::to_lower_copy(map_power.type) == "as5600";
      }) |
      std::views::transform([&](const auto &pair) {
        const auto &name = pair.first;
        const auto &map_ina = pair.second;
        std::map<std::string, rclcpp::ParameterValue> parameters;
        parameters["type"] = rclcpp::ParameterValue(map_ina.type);
        parameters["connector"] = rclcpp::ParameterValue(map_ina.connector);
        parameters["pins.sda"] = rclcpp::ParameterValue(map_ina.pins.sda);
        parameters["pins.scl"] = rclcpp::ParameterValue(map_ina.pins.scl);
        parameters["addr"] = rclcpp::ParameterValue(map_ina.addr);

        auto param_listener_motors = std::make_shared<
            mirte_telemetrix_cpp_as5600_sensors::ParamListener>(
            parser->nh, fmt::format("modules.{}", name));
        auto params_motors = param_listener_motors->get_params();
        std::vector<std::tuple<int, std::string>> motor_data;
        std::cout << "Parsing motors for PCA module: " << name << std::endl;
        // std::cout << "motor number of names: "
        // << params_motors.pca_motor_names.size() << std::endl;
        std::cout << "Motor names found: ";
        for (auto const &motor_name : params_motors.as5600_sensor_names) {
          std::cout << "Parsing motor data for motor name: " << motor_name
                    << std::endl;
          if (motor_name == "") {
            continue;
          }
          auto t = std::tuple<int, std::string>(
              params_motors.encoders.as5600_sensor_names_map.at(motor_name).mux,
              motor_name);
          motor_data.push_back(t);
        }
        // std::vector<std::shared_ptr<PCA_Servo_data>> servo_data;
        // TODO: add servo parsing again!
        // for (auto const &servo_name : params_motors.pca_servo_names) {
        //   if(servo_name == "") {
        //     continue;
        //   }
        //   auto servo_params = params_motors.pca_servos_map.at(servo_name);
        //   servo_data.name = servo_name;
        //   servo_data.pin = servo_params.pin;
        //   servo_data.min_pulse = servo_params.min_pulse;
        //   servo_data.max_pulse = servo_params.max_pulse;
        //   servo_data.min_angle = servo_params.min_angle;
        //   servo_data.max_angle = servo_params.max_angle;
        // }
        std::set<std::string> unused_keys = get_keys(parameters);
        return AS5600Data(parser, node_data.board, name, parameters,
                          unused_keys, motor_data);
      });

  for (auto pca : datas) {
    auto pca_module = std::make_shared<AS5600_sensor>(node_data, pca, sensors);
    as_modules.push_back(pca_module);
  }

  return as_modules;
}