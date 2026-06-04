#include <functional>
#include <memory>
#include <numbers>

#include <mirte_telemetrix_cpp/modules/mpu6050_module.hpp>
#include <mirte_telemetrix_cpp/mpu6050_parameters.hpp>
#include <ranges>
using namespace std::placeholders;

MPU6050_sensor::MPU6050_sensor(NodeData node_data, MPU6050Data imu_data,
                               std::shared_ptr<tmx_cpp::Sensors> sensors)
    : Mirte_module(node_data, {imu_data.scl, imu_data.sda},
                   (ModuleData)imu_data),
      data(imu_data) {
  std::cout << "Initializing MPU6050 module: " << this->name << std::endl;
  std::cout << "I2C Pins - SCL: " << (int)imu_data.scl
            << ", SDA: " << (int)imu_data.sda
            << ", Port: " << (int)imu_data.port << std::endl;
  tmx->setI2CPins(imu_data.sda, imu_data.scl, imu_data.port);

  this->mpu6050 = std::make_shared<tmx_cpp::MPU6050_module>(
      imu_data.port, imu_data.addr,
      std::bind(&MPU6050_sensor::data_callback, this, _1, _2, _3));

  imu_pub = nh->create_publisher<sensor_msgs::msg::Imu>(
      "imu/" + this->name + "/data", rclcpp::SystemDefaultsQoS());

  imu_service = nh->create_service<mirte_msgs::srv::GetImu>(
      "imu/" + this->name + "/get_data",
      std::bind(&MPU6050_sensor::get_imu_service_callback, this, _1, _2),
      rclcpp::ServicesQoS().get_rmw_qos_profile(), this->callback_group);

  msg.linear_acceleration_covariance[0] =
      std::pow(60.0 * 9.81 / 1000, 2); // Var_x
  msg.linear_acceleration_covariance[4] =
      std::pow(60.0 * 9.81 / 1000, 2); // Var_y
  msg.linear_acceleration_covariance[8] =
      std::pow(80.0 * 9.81 / 1000, 2); // Var_z

  msg.angular_velocity_covariance[0] =
      std::pow(5.0 * std::numbers::pi / 180.0, 2); // Var_x
  msg.angular_velocity_covariance[4] =
      std::pow(5.0 * std::numbers::pi / 180.0, 2); // Var_y
  msg.angular_velocity_covariance[8] =
      std::pow(5.0 * std::numbers::pi / 180.0, 2); // Var_z

  sensors->add_sens(this->mpu6050);
}

void MPU6050_sensor::update() {
  if (this->imu_pub->get_subscription_count() == 0) {
    // No subscribers, so no need to publish
    return;
  }
  if (msg_mutex.try_lock()) {
    const std::lock_guard lock{msg_mutex, std::adopt_lock};
    msg.header = get_header();
    imu_pub->publish(msg);
  }
}

void MPU6050_sensor::data_callback(std::array<float, 3> acceleration,
                                   std::array<float, 3> gyro,
                                   float temperature) {
  const std::lock_guard<std::mutex> lock(msg_mutex);
  msg.header = get_header();
  // std::cout << "MPU6050 data callback: Accel: [" << acceleration[0] << ", "
  //           << acceleration[1] << ", " << acceleration[2] << "], Gyro: ["
  //           << gyro[0] << ", " << gyro[1] << ", " << gyro[2]
  //           << "], Temp: " << temperature << std::endl;
  msg.linear_acceleration.x =
      acceleration[0] * 9.81; // convert from g's to m/s^2
  msg.linear_acceleration.y = acceleration[1] * 9.81;
  msg.linear_acceleration.z = acceleration[2] * 9.81;

  msg.angular_velocity.x =
      gyro[0] * std::numbers::pi / 180.0; // convert from degrees/s to radians/s
  msg.angular_velocity.y = gyro[1] * std::numbers::pi / 180.0;
  msg.angular_velocity.z = gyro[2] * std::numbers::pi / 180.0;
  // msg.temperature = temperature; // not a field and temp is influenced by
  // chip itself
}
void MPU6050_sensor::get_imu_service_callback(
    const mirte_msgs::srv::GetImu::Request::ConstSharedPtr req,
    mirte_msgs::srv::GetImu::Response::SharedPtr res) {
  const std::lock_guard<std::mutex> lock(msg_mutex);
  res->data = sensor_msgs::msg::Imu(msg);
}

std::vector<std::shared_ptr<MPU6050_sensor>>
MPU6050_sensor::get_mpu_modules(NodeData node_data,
                                std::shared_ptr<Parser> parser,
                                std::shared_ptr<tmx_cpp::Sensors> sensors) {
  std::vector<std::shared_ptr<MPU6050_sensor>> mpu_modules;

  parser->update_params_list_type("modules", "mpu6050_module_names", "mpu6050");

  auto param_listener =
      std::make_shared<mirte_telemetrix_cpp_mpu6050::ParamListener>(parser->nh);
  auto params = param_listener->get_params();
  // get modules list from parser
  // loop over items, get one with type==mpu6050
  // add paramlistener to those with new parameters yaml
  auto datas =
      params.modules.mpu6050_module_names_map |
      std::views::filter([](const auto &pair) {
        const auto &map_power = pair.second;
        return boost::algorithm::to_lower_copy(map_power.type) == "mpu6050";
      }) |
      std::views::transform([&](const auto &pair) {
        const auto &name = pair.first;
        const auto &map_mpu = pair.second;
        std::map<std::string, rclcpp::ParameterValue> parameters;
        parameters["type"] = rclcpp::ParameterValue(map_mpu.type);
        parameters["connector"] = rclcpp::ParameterValue(map_mpu.connector);
        parameters["pins.sda"] = rclcpp::ParameterValue(map_mpu.pins.sda);
        parameters["pins.scl"] = rclcpp::ParameterValue(map_mpu.pins.scl);
        parameters["addr"] = rclcpp::ParameterValue(map_mpu.addr);
        std::set<std::string> unused_keys = get_keys(parameters);
        return MPU6050Data(parser, node_data.board, name, parameters,
                           unused_keys);
      });

  for (auto mpu : datas) {
    auto mpu_module = std::make_shared<MPU6050_sensor>(node_data, mpu, sensors);
    mpu_modules.push_back(mpu_module);
  }
  return mpu_modules;
}
