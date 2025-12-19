#include <boost/format.hpp>

#include <rcpputils/asserts.hpp>

#include <mirte_telemetrix_cpp/parsers/modules/i2c_module_data.hpp>

I2CModuleData::I2CModuleData(
    std::shared_ptr<Parser> parser, std::shared_ptr<Mirte_Board> board,
    std::string name, std::map<std::string, rclcpp::ParameterValue> parameters,
    std::set<std::string> &unused_keys, std::string module_type,
    std::optional<DeviceDuration> duration)
    : ModuleData(parser, board, name, parameters, unused_keys, duration) {
  for (const auto &key : unused_keys) {
    std::cout << "Unused key: " << key << std::endl;
  }
  if (unused_keys.erase("connector")) {
    auto connector = get_string(parameters["connector"]);
    auto pins = board->resolveConnector(connector);
    this->scl = pins["scl"];
    this->sda = pins["sda"];
  } else if (unused_keys.erase("pins.scl") &&
             unused_keys.erase("pins.sda")) {
    std::cout << "I2CModuleData: Using direct pin assignment" << std::endl;
     
      this->scl = board->resolvePin(get_string(parameters["pins.scl"]));
     
      this->sda = board->resolvePin(get_string(parameters["pins.sda"]));
  
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
  std::cout << "I2CModuleData check: " << module_type << std::endl;
  std::cout << "I2C address: " << (int)addr << std::endl;
  std::cout << "SCL pin: " << (int)scl << std::endl;
  std::cout << "SDA pin: " << (int)sda << std::endl;
  std::cout << "I2C port: " << (int)port << std::endl;
  return scl != (pin_t)-1 && sda != (pin_t)-1 && addr != (uint8_t)-1 &&
         addr <= 0x7F && ModuleData::check(module_type);
}