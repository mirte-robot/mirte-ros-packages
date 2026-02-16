#include <cmath>

#include <mirte_telemetrix_cpp/parsers/modules/hiwonder/hiwonder_servo_data.hpp>

double deg_to_rad(double deg) { return deg * M_PI / 180; }

// std::vector<std::shared_ptr<Hiwonder_servo_data>>
// Hiwonder_servo_data::parse_hiwonder_servo_data(
//   std::shared_ptr<Parser> parser, std::shared_ptr<Mirte_Board> board,
//   std::string bus_key)
// {
//   std::vector<std::shared_ptr<Hiwonder_servo_data>> servos;
//   auto bus_config = parser->get_params_name(bus_key);
//   auto bus_keys = parser->get_params_keys(bus_key);
//   if (bus_keys.count("servos")) {
//     auto servos_name = parser->build_param_name(bus_key, "servos");
//     auto servos_config = parser->get_params_name(servos_name);
//     for (auto servo_key : parser->get_params_keys(servos_name)) {
//       auto servo_config =
//       parser->get_params_name(parser->build_param_name(servos_name,
//       servo_key)); auto servo_keys =
//       parser->get_params_keys(parser->build_param_name(servos_name,
//       servo_key)); Hiwonder_servo_data servo_data; servo_data.name =
//       servo_key; std::cout << "hiwo" << std::dec << (int)__LINE__ <<
//       std::endl; if (servo_keys.count("id")) {
//         servo_data.id = servo_config["id"].get<uint8_t>();
//       }
//       if (servo_keys.count("min_angle_out")) {
//         std::cout << "hiwo" << std::dec << (int)__LINE__ << std::endl;
//         servo_data.min_angle_out = servo_config["min_angle_out"].get<int>();
//       } else {
//         continue;
//       }
//       std::cout << "hiwo" << std::dec << (int)__LINE__ << std::endl;
//       if (servo_keys.count("max_angle_out")) {
//         servo_data.max_angle_out = servo_config["max_angle_out"].get<int>();
//       } else {
//         continue;
//       }
//       std::cout << "hiwo" << std::dec << (int)__LINE__ << std::endl;
//       if (servo_keys.count("home_out")) {
//         servo_data.home_out = servo_config["home_out"].get<int>();
//       } else {
//         continue;
//       }
//       std::cout << "hiwo" << std::dec << (int)__LINE__ << std::endl;

//       if (servo_keys.count("invert")) {
//         servo_data.min_angle_in = servo_config["invert"].get<bool>();
//       }
//       std::cout << "hiwo" << std::dec << (int)__LINE__ << std::endl;
//       if (servo_keys.count("frame")) {
//         std::cout << "hiwo" << std::dec << (int)__LINE__ << std::endl;
//         servo_data.frame_id = servo_config["frame"].get<std::string>();
//       } else {
//         std::cout << "hiwo" << std::dec << (int)__LINE__ << std::endl;
//         servo_data.frame_id = "servo_" + servo_data.name;
//       }
//       // calculate min_angle_in and max_angle_in
//       double diff_min = servo_data.min_angle_out - servo_data.home_out;
//       diff_min = diff_min / 100;
//       servo_data.min_angle_in = deg_to_rad(diff_min);
//       double diff_max = servo_data.max_angle_out - servo_data.home_out;
//       diff_max = diff_max / 100;
//       servo_data.max_angle_in = deg_to_rad(diff_max);
//       if (servo_data.invert) {
//         double t = servo_data.min_angle_in;
//         servo_data.min_angle_in = -servo_data.max_angle_in;
//         servo_data.max_angle_in = -t;
//       }
//       std::cout << "hiwo" << std::dec << (int)__LINE__ << std::endl;
//       std::cout << "Servo: " << servo_data.name << " min_angle_in: " <<
//       servo_data.min_angle_in
//                 << " max_angle_in: " << servo_data.max_angle_in << std::endl;
//       if (servo_data.check()) {
//         servos.push_back(std::make_shared<Hiwonder_servo_data>(servo_data));
//       }
//     }
//   }
//   return servos;
// }

HiWonderServoData::HiWonderServoData(
    std::shared_ptr<Parser> parser, std::shared_ptr<Mirte_Board> board,
    std::string name, std::map<std::string, rclcpp::ParameterValue> parameters,
    std::set<std::string> &unused_keys, std::string base_frame_id)
    : DeviceData(parser, board, name, get_device_class(), parameters,
                 unused_keys) {
  auto logger = parser->logger;

  if (unused_keys.erase("id")) {
    this->id = parameters["id"].get<uint8_t>();
  } else {
    RCLCPP_ERROR(logger, "HiWonder Servo '%s' is missing an id", name.c_str());
  }

  /* NOTE: In the old implementation, this was required
     TODO: Figure out if that makes sense */
  if (unused_keys.erase("min_angle_out")) {
    this->min_angle_out = parameters["min_angle_out"].get<int>();
  }

  /* NOTE: In the old implementation, this was required
     TODO: Figure out if that makes sense */
  if (unused_keys.erase("max_angle_out")) {
    this->max_angle_out = parameters["max_angle_out"].get<int>();
  }

  /* NOTE: In the old implementation, this was required
     TODO: Figure out if that makes sense */
  if (unused_keys.erase("home_out")) {
    this->home_out = parameters["home_out"].get<int>();
  }

  if (unused_keys.erase("invert")) {
    this->invert = parameters["invert"].get<bool>();
  }

  /* Calculate the max and min angle_in */
  double diff_min = this->min_angle_out - this->home_out;
  diff_min = diff_min / 100;
  this->min_angle_in = deg_to_rad(diff_min);

  double diff_max = this->max_angle_out - this->home_out;
  diff_max = diff_max / 100;
  this->max_angle_in = deg_to_rad(diff_max);

  if (this->invert) {
    auto tmp = this->min_angle_in;
    this->min_angle_in = -this->max_angle_in;
    this->max_angle_in = -tmp;
  }

  // TODO: document this feature
  if (unused_keys.erase("motor_mode")) {
    this->enable_motor = parameters["motor_mode"].get<bool>();
  }

  if (!parameters.count("frame_id")) {
    this->frame_id = base_frame_id + "/" + name;
  }
}

bool HiWonderServoData::check() {
  return id >= 0x00 && id < 0xFE && DeviceData::check();
}

std::vector<std::shared_ptr<HiWonderServoData>>
HiWonderServoData::parse_hiwonder_servo_data(std::shared_ptr<Parser> parser,
                                             std::shared_ptr<Mirte_Board> board,
                                             std::string bus_name,
                                             std::set<std::string> &unused_keys,
                                             std::string base_frame_id) {
  auto logger = parser->logger;

  std::vector<std::shared_ptr<HiWonderServoData>> servos;
  auto bus_config = parser->get_params_name(bus_name);
  auto bus_keys = parser->get_params_keys(bus_name);

  if (bus_keys.count("servos")) {
    auto servos_name = parser->build_param_name(bus_name, "servos");

    auto servo_config = parser->get_params_name(servos_name);
    for (auto servo_key : parser->get_params_keys(servos_name)) {
      auto servo_param_name = parser->build_param_name(servos_name, servo_key);
      auto servo_config = parser->get_params_name(servo_param_name);
      auto servo_keys = parser->get_params_keys(servo_param_name);

      auto data = std::make_shared<HiWonderServoData>(
          parser, board, servo_key, servo_config, servo_keys, base_frame_id);

      if (data->check()) {
        servos.push_back(data);
      } else {
        RCLCPP_ERROR(logger,
                     "HiWonder Servo '%s' is invalid and will not be active",
                     data->name.c_str());
      }

      auto key_prefix = servo_param_name.substr(bus_name.size() + 1);
      for (auto subkey : servo_keys) {
        unused_keys.insert(parser->build_param_name(key_prefix, subkey));
      }
    }
  }

  return servos;
}