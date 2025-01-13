#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>

#include <rcpputils/asserts.hpp>

#include <mirte_telemetrix_cpp/parsers/modules/pca_data.hpp>

PCAData::PCAData(std::shared_ptr<Parser> parser,
                 std::shared_ptr<Mirte_Board> board, std::string name,
                 std::map<std::string, rclcpp::ParameterValue> parameters,
                 std::set<std::string> &unused_keys)
    : I2CModuleData(parser, board, name, parameters, unused_keys, get_module_type()) {
  auto key = get_device_key(this);
  // This logger is only used for DEBUG, so it is a child logger.
  auto logger = parser->logger.get_child(key);

  // Set default for address
  if ((!parameters.count("addr")) && this->addr == 0xFF)
    this->addr = 0x41;

  if (unused_keys.erase("frequency"))
    this->frequency = parameters["frequency"].get<int>();

  if (unused_keys.erase("motors")) {
    RCLCPP_DEBUG(logger, "Attempting to find PCA motors [%s]", key.c_str());
    // TODO: MOTORS REPARSE
    this->motors = PCA_Motor_data::parse_pca_motor_data(parser, /* board,*/ key,
                                                        unused_keys);
  }

  if (unused_keys.erase("servos")) {
    RCLCPP_DEBUG(logger, "Attempting to find PCA servos [%s]", key.c_str());
    // TODO: SERVOS REPARSE
    this->servos = PCA_Servo_data::parse_pca_servo_data(parser, /*board,*/ key,
                                                        unused_keys);
  }
}

bool PCAData::check() { return I2CModuleData::check(get_module_type()); }
