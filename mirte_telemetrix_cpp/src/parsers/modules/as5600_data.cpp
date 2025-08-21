#include <mirte_telemetrix_cpp/parsers/modules/as5600_data.hpp>

AS5600Data::AS5600Data(std::shared_ptr<Parser> parser,
                       std::shared_ptr<Mirte_Board> board, std::string name,
                       std::map<std::string, rclcpp::ParameterValue> parameters,
                       std::set<std::string> &unused_keys)
    : I2CModuleData(parser, board, name, parameters, unused_keys,
                    get_module_type()) {
  auto logger =
      parser->logger.get_child(get_device_class()).get_child(this->name);
  std::cout << "AS5600Data constructor called for " << name << std::endl;
  if (unused_keys.erase("encoders")) {
    // encoders is a vector of objects with name:channel, e.g.,
    // - wheel:
    //   mux: 1
    // - arm:
    //   mux: 2
    auto encoders = parameters["encoders"];
    if (parameters.count("encoders")) {
      std::cout << "AS5600 encoders: exist " << std::endl;
      auto encoders = parser->get_params_keys(
          parser->build_param_name(get_device_key(this), "encoders"));
      std::cout << "AS5600 encoders: " << encoders.size() << std::endl;

      for (const auto &encoder : encoders) {
        std::cout << "AS5600 encoder: " << encoder << std::endl;
        // auto key = parser->build_param_name(get_device_key(this), "encoders",
        // encoder + ".mux");
        auto key = parser->build_param_name(
            {get_device_key(this), "encoders", encoder, "mux"});
        std::cout << "AS5600 encoder key: " << key << std::endl;
        if (unused_keys.erase(key) == 0) {
          std::cout << "AS5600 encoder '" << encoder
                    << "' mux not found in parameters" << std::endl;
          // continue;
        }
        auto channel = get_string(
            parameters[parser->build_param_name({"encoders", encoder, "mux"})]);
        this->encoders.push_back(
            std::make_tuple(std::stoi(channel),
                            encoder)); // Store channel and name
      }
    }
  }
}
bool AS5600Data::check() {
  return !encoders.empty() && I2CModuleData::check(get_module_type());
}
