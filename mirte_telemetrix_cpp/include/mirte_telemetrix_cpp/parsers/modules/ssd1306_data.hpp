#pragma once

#include <mirte_telemetrix_cpp/parsers/modules/i2c_module_data.hpp>

class SSD1306Data : public I2CModuleData {
public:
  uint8_t width = 128; // Hardcoded in the Pico
  uint8_t height = 64; // Hardcoded in the Pico

  // Enable LEGACY set OLED image, which works as the old set_oled_image
  bool legacy = false;
  // The default location to look for images, this can be an absolute path or a
  // package uri ('pkg://PKG_NAME/FOLDER_PATH...'). This is also used by the
  // Legacy command as the folder containing an 'images' and 'animations'
  // folder.
  std::string default_image_path = "/usr/local/src/mirte/mirte-oled-images";

  std::string default_screen_script =
      "package://mirte_telemetrix_cpp/scripts/default_screen.sh";

  SSD1306Data(std::shared_ptr<Parser> parser,
              std::shared_ptr<Mirte_Board> board, std::string name,
              std::map<std::string, rclcpp::ParameterValue> parameters,
              std::set<std::string> &unused_keys);

  bool check() override;
  using I2CModuleData::check;

  static std::string get_module_type() { return "ssd1306"; };
  static std::string get_device_class() { return "oled"; };

  void set_default_screen_script(std::string path);
};
