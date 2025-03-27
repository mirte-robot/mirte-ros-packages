#include <boost/format.hpp>

#include <rcpputils/asserts.hpp>

#include <mirte_telemetrix_cpp/parsers/modules/i2c_module_data.hpp>

I2CModuleData::I2CModuleData(
    std::shared_ptr<Parser> parser, std::shared_ptr<Mirte_Board> board,
    std::string name, std::map<std::string, rclcpp::ParameterValue> parameters,
    std::set<std::string> &unused_keys, std::string module_type,
    std::optional<DeviceDuration> duration)
    : ModuleData(parser, board, name, parameters, unused_keys, duration) {
  if (unused_keys.erase("connector")) {
    auto connector = get_string(parameters["connector"]);
    auto pins = board->resolveConnector(connector);
    this->scl = pins["scl"];
    this->sda = pins["sda"];
  } else if (unused_keys.erase("pins")) {
    auto subkeys = parser->get_params_keys(
        parser->build_param_name(module_type + "." + this->name, "pins"));

    if (subkeys.erase("scl")) {
      this->scl = board->resolvePin(get_string(parameters["pins.scl"]));
    }
    if (subkeys.erase("sda")) {
      this->sda = board->resolvePin(get_string(parameters["pins.sda"]));
    }
  } else {
    rcpputils::require_true(
        false,
        (boost::format(
             "No connector or pins tag was supplied to %1% module '%2%'.") %
         get_module_type() % name)
            .str());
  }
  this->port = board->resolveI2CPort(this->sda);

  if (unused_keys.erase("addr")) {
    this->addr = parameters["addr"].get<uint8_t>();
  }
}

bool I2CModuleData::check(std::string module_type) {
  // Check if I2C address is between the valid range of 0 and 127 (0x7F)
  return scl != (pin_t)-1 && sda != (pin_t)-1 && addr != (uint8_t)-1 &&
         addr <= 0x7F && ModuleData::check(module_type);
}