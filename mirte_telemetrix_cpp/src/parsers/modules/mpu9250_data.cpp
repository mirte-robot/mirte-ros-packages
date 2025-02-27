#include <mirte_telemetrix_cpp/parsers/modules/mpu9250_data.hpp>

// Use the default frame_id `imu_link` as specified in REP0145
MPU9250Data::MPU9250Data(
    std::shared_ptr<Parser> parser, std::shared_ptr<Mirte_Board> board,
    std::string name, std::map<std::string, rclcpp::ParameterValue> parameters,
    std::set<std::string> &unused_keys)
    : I2CModuleData(parser, board, name, insert_default_frame_id(parameters),
                    insert_default_frame_id(unused_keys), get_module_type()) {
  // Set default for address
  if ((!parameters.count("addr")) && this->addr == 0xFF)
    this->addr = 0x68;
}

std::map<std::string, rclcpp::ParameterValue>
MPU9250Data::insert_default_frame_id(
    std::map<std::string, rclcpp::ParameterValue> parameters) {
  return insert_default_param(parameters, "frame_id",
                              rclcpp::ParameterValue("imu_link"));
}

std::set<std::string> &
MPU9250Data::insert_default_frame_id(std::set<std::string> &unused_keys) {
  return insert_default_param(unused_keys, "frame_id");
}

bool MPU9250Data::check() { return I2CModuleData::check(get_module_type()); }
