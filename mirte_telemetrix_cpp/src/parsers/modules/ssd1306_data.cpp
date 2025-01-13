#include <chrono>
#include <filesystem>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <mirte_telemetrix_cpp/parsers/modules/ssd1306_data.hpp>

using namespace std::chrono_literals;

SSD1306Data::SSD1306Data(
    std::shared_ptr<Parser> parser, std::shared_ptr<Mirte_Board> board,
    std::string name, std::map<std::string, rclcpp::ParameterValue> parameters,
    std::set<std::string> &unused_keys)
    : I2CModuleData(parser, board, name,
                    insert_default_param(parameters, "type",
                                         rclcpp::ParameterValue("ssd1306")),
                    insert_default_param(unused_keys, "type"),
                    get_device_class(),
                    std::chrono::duration_cast<DeviceDuration>(10s)) {
  // Set default for address
  if ((!parameters.count("addr")) && this->addr == 0xFF)
    this->addr = 0x3C;

  // TODO: Read in size [BLOCKED BY FIXED SIZE FIRMWARE]

  if (unused_keys.erase("legacy"))
    this->legacy = parameters["legacy"].get<bool>();

  if (unused_keys.erase("default_image_path"))
    this->default_image_path =
        parameters["default_image_path"].get<std::string>();

  if (unused_keys.erase("default_screen_script"))
    this->set_default_screen_script(
        parameters["default_screen_script"].get<std::string>());
  else
    this->set_default_screen_script(default_screen_script);
}

bool SSD1306Data::check() { return I2CModuleData::check(get_module_type()); }

void SSD1306Data::set_default_screen_script(std::string path) {
  if ((path.size() > 6 && path.substr(0, 6) == "pkg://") ||
      (path.size() > 11 && path.substr(0, 10) == "package://")) {
    auto prefix_length = (path.substr(0, 6) == "pkg://") ? 6 : 10;
    auto data_path = path.substr(prefix_length);

    auto split_idx = data_path.find('/');

    auto package = data_path.substr(0, split_idx);
    auto remainder = data_path.substr(split_idx + 1);

    this->default_screen_script =
        std::filesystem::path(
            ament_index_cpp::get_package_share_directory(package)) /
        remainder;
  } else if (path[0] != '/') {
    set_default_screen_script("package://mirte_telemetrix_cpp/" + path);
  } else {
    this->default_screen_script = path;
  }
}
