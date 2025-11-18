#pragma once
#include <filesystem>
#include <optional>

#include <tmx_cpp/modules/SSD1306_oled.hpp>

#include <mirte_telemetrix_cpp/modules/base_module.hpp>

#include <mirte_telemetrix_cpp/parsers/modules/ssd1306_data.hpp>

#include <mirte_msgs/srv/set_oled_file.hpp>
#include <mirte_msgs/srv/set_oled_image.hpp>
#include <mirte_msgs/srv/set_oled_image_legacy.hpp>
#include <mirte_msgs/srv/set_oled_text.hpp>

namespace fs = std::filesystem;

class SSD1306_module : public Mirte_module {
public:
  SSD1306_module(NodeData node_data, SSD1306Data oled_data,
                 std::shared_ptr<tmx_cpp::Modules> modules);

  SSD1306Data data;
  std::shared_ptr<tmx_cpp::SSD1306_module> ssd1306;

  static std::vector<std::shared_ptr<SSD1306_module>>
  get_ssd1306_modules(NodeData node_data, std::shared_ptr<Parser> parser,
                      std::shared_ptr<tmx_cpp::Modules> modules);

  bool set_text(std::string text);
  bool set_image(uint8_t width, uint8_t height, uint8_t img_buffer[]);
  bool set_image_from_path(fs::path path);
  bool set_image_from_path(std::string path);

  virtual void device_timer_callback() override;

private:
  bool enabled = true;
  bool default_screen = true;
  std::optional<std::string> last_text;

  // Only enabled if legacy is enabled in the config
  // Service: oled/NAME/set_image_legacy
  rclcpp::Service<mirte_msgs::srv::SetOLEDImageLegacy>::SharedPtr
      set_oled_service_legacy;

  // Service: oled/NAME/set_text
  rclcpp::Service<mirte_msgs::srv::SetOLEDText>::SharedPtr
      set_oled_text_service;
  // Service: oled/NAME/set_image
  rclcpp::Service<mirte_msgs::srv::SetOLEDImage>::SharedPtr
      set_oled_image_service;
  // Service: oled/NAME/set_file
  rclcpp::Service<mirte_msgs::srv::SetOLEDFile>::SharedPtr
      set_oled_file_service;

  bool prewrite(bool is_default = false);

  void set_oled_callback_legacy(
      const mirte_msgs::srv::SetOLEDImageLegacy::Request::ConstSharedPtr req,
      mirte_msgs::srv::SetOLEDImageLegacy::Response::SharedPtr res);

  void set_oled_text_callback(
      const mirte_msgs::srv::SetOLEDText::Request::ConstSharedPtr req,
      mirte_msgs::srv::SetOLEDText::Response::SharedPtr res);

  void set_oled_image_callback(
      const mirte_msgs::srv::SetOLEDImage::Request::ConstSharedPtr req,
      mirte_msgs::srv::SetOLEDImage::Response::SharedPtr res);

  void set_oled_file_callback(
      const mirte_msgs::srv::SetOLEDFile::Request::ConstSharedPtr req,
      mirte_msgs::srv::SetOLEDFile::Response::SharedPtr res);
};
