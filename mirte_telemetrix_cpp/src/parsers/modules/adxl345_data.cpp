#include <mirte_telemetrix_cpp/parsers/modules/adxl345_data.hpp>

// TODO: Maybe add (optional) calibration data, since this sensor has a
// relatively large offset...

// TODO: Verify if acc orientation fully correct according to REP
// Use the default frame_id `imu_link` as specified in REP0145
ADXL345Data::ADXL345Data(
    std::shared_ptr<Parser> parser, std::shared_ptr<Mirte_Board> board,
    std::string name, std::map<std::string, rclcpp::ParameterValue> parameters,
    std::set<std::string> &unused_keys)
    : I2CModuleData(parser, board, name, insert_default_frame_id(parameters),
                    insert_default_frame_id(unused_keys), get_module_type()) {
  auto logger = parser->logger;

  // Set default for address
  if ((!parameters.count("addr")) && this->addr == 0xFF) {
    this->addr = 0x53;
  }
  if (this->addr != 0x53 && this->addr != 0x1D) {
    RCLCPP_ERROR(logger,
                 "The ADXL345 IMU module '%s' was defined with the addr(ess) "
                 "0x%X, however the only valid "
                 "addresses are 0x53 and 0x1D.",
                 this->name.c_str(), this->addr);
  }
}

std::map<std::string, rclcpp::ParameterValue>
ADXL345Data::insert_default_frame_id(
    std::map<std::string, rclcpp::ParameterValue> parameters) {
  return insert_default_param(parameters, "frame_id",
                              rclcpp::ParameterValue("imu_link"));
}

std::set<std::string> &
ADXL345Data::insert_default_frame_id(std::set<std::string> &unused_keys) {
  return insert_default_param(unused_keys, "frame_id");
}

bool ADXL345Data::check() { return I2CModuleData::check(get_module_type()); }
