#include <future>

#include <mirte_telemetrix_cpp/mirte-modules.hpp>
#include <mirte_telemetrix_cpp/modules/adxl345_module.hpp>
#include <mirte_telemetrix_cpp/modules/as5600_module.hpp>
#include <mirte_telemetrix_cpp/modules/hiwonder_module.hpp>
#include <mirte_telemetrix_cpp/modules/ina226_module.hpp>
#include <mirte_telemetrix_cpp/modules/mpu6050_module.hpp>
#include <mirte_telemetrix_cpp/modules/mpu9250_module.hpp>
#include <mirte_telemetrix_cpp/modules/pca_module.hpp>
#include <mirte_telemetrix_cpp/modules/ssd1306_module.hpp>
#include <mirte_telemetrix_cpp/modules/veml6040_module.hpp>
#include <mirte_telemetrix_cpp/util.hpp>

Mirte_modules::Mirte_modules(NodeData node_data, std::shared_ptr<Parser> parser)
    : tmx(node_data.tmx), nh(node_data.nh), board(node_data.board),
      parser(parser), node_data(node_data) {
  this->tmx->module_sys = tmx->module_sys;
  this->sensor_sys = tmx->sensors_sys;
}
void Mirte_modules::start() {
  RCLCPP_INFO(nh->get_logger(), "Proccessing HiWonder Modules [ASYNC]");
  auto hiwonder_mods_future =
      std::async(std::launch::async, HiWonderBus_module::get_hiwonder_modules,
                 node_data, parser, this->tmx->module_sys);

  RCLCPP_INFO(nh->get_logger(), "Adding PCA Modules");
  auto pca_mods =
      PCA_Module::get_pca_modules(node_data, parser, this->tmx->module_sys);
  this->modules.insert(this->modules.end(), pca_mods.begin(), pca_mods.end());

  RCLCPP_INFO(nh->get_logger(), "Adding SSD1306 OLED Modules");
  auto oled_mods = SSD1306_module::get_ssd1306_modules(node_data, parser,
                                                       this->tmx->module_sys);
  this->modules.insert(this->modules.end(), oled_mods.begin(), oled_mods.end());

  //   this->sensor_sys = std::make_shared<tmx_cpp::Sensors>(tmx);
  RCLCPP_INFO(nh->get_logger(), "Adding INA226 Modules");
  auto ina_mods = INA226_sensor::get_ina_modules(
      node_data, parser, this->sensor_sys, this->tmx->module_sys);
  std::cout << "Adding ina modules" << ina_mods.size() << std::endl;
  this->modules.insert(this->modules.end(), ina_mods.begin(), ina_mods.end());

  RCLCPP_INFO(nh->get_logger(), "Adding AS5600 Modules");
  auto as5600_mods =
      AS5600_sensor::get_as5600_modules(node_data, parser, this->sensor_sys);
  std::cout << "Adding as5600 modules" << as5600_mods.size() << std::endl;
  this->modules.insert(this->modules.end(), as5600_mods.begin(),
                       as5600_mods.end());

  RCLCPP_INFO(nh->get_logger(), "Adding MPU9250 Modules");
  auto mpu_mods =
      MPU9250_sensor::get_mpu_modules(node_data, parser, this->sensor_sys);
  auto mpu6050_mods =
      MPU6050_sensor::get_mpu_modules(node_data, parser, this->sensor_sys);
  this->modules.insert(this->modules.end(), mpu6050_mods.begin(),
                       mpu6050_mods.end());

  RCLCPP_INFO(nh->get_logger(), "Adding ADXL345 Modules");
  auto adxl_mods =
      ADXL345_sensor::get_adxl_modules(node_data, parser, this->sensor_sys);
  this->modules.insert(this->modules.end(), adxl_mods.begin(), adxl_mods.end());

  RCLCPP_INFO(nh->get_logger(), "Adding VEML6040 Modules");
  auto veml_mods = VEML6040_sensor::get_veml6040_modules(node_data, parser,
                                                         this->sensor_sys);
  this->modules.insert(this->modules.end(), veml_mods.begin(), veml_mods.end());

  RCLCPP_INFO(nh->get_logger(), "Adding HiWonder Modules");
  auto hiwonder_mods = hiwonder_mods_future.get();
  std::cout << "Adding hiwonder modules" << hiwonder_mods.size() << std::endl;
  this->modules.insert(this->modules.end(), hiwonder_mods.begin(),
                       hiwonder_mods.end());
}
