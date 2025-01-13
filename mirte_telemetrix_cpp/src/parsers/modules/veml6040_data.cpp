#include <mirte_telemetrix_cpp/parsers/modules/veml6040_data.hpp>

VEML6040Data::VEML6040Data(
    std::shared_ptr<Parser> parser, std::shared_ptr<Mirte_Board> board,
    std::string name, std::map<std::string, rclcpp::ParameterValue> parameters,
    std::set<std::string> &unused_keys)
    : I2CModuleData(parser, board, name,
                    insert_default_param(parameters, "type",
                                         rclcpp::ParameterValue("veml6040")),
                    insert_default_param(unused_keys, "type"),
                    get_module_type()) {
  auto logger = parser->logger;
  // Set default for address
  // The address cannot be changed on the hardware, therefor we allow an address
  // to be specified.
  //  If it is not valid, it gets detected in the check function.
  if ((!parameters.count("addr")) && this->addr == 0xFF)
    this->addr = 0x10;
  if ((parameters.count("addr")) && this->addr != 0x10) {
    RCLCPP_ERROR(logger,
                 "The color module '%s' was defined with the addr(ess) 0x%X, "
                 "however the only valid address "
                 "is 0x10.",
                 this->name.c_str(), this->addr);
  }
}

bool VEML6040Data::check() {
  return (addr == 0x10) && I2CModuleData::check(get_module_type());
}
