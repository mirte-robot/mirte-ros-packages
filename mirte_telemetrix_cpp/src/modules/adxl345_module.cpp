#include <functional>
#include <memory>
#include <numbers>

#include <mirte_telemetrix_cpp/modules/adxl345_module.hpp>

using namespace std::placeholders;

ADXL345_sensor::ADXL345_sensor(NodeData node_data, ADXL345Data imu_data,
                               std::shared_ptr<tmx_cpp::Sensors> sensors)
    : Mirte_module(node_data, {imu_data.scl, imu_data.sda},
                   (ModuleData)imu_data),
      data(imu_data) {
  tmx->setI2CPins(imu_data.sda, imu_data.scl, imu_data.port);

  this->adxl345 = std::make_shared<tmx_cpp::ADXL345_module>(
      imu_data.port, imu_data.addr,
      std::bind(&ADXL345_sensor::data_callback, this, _1));

  imu_pub = nh->create_publisher<sensor_msgs::msg::Imu>(
      "imu/" + this->name + "/data_raw", rclcpp::SystemDefaultsQoS());

  imu_service = nh->create_service<mirte_msgs::srv::GetImu>(
      "imu/" + this->name + "/get_data_raw",
      std::bind(&ADXL345_sensor::get_imu_service_callback, this, _1, _2),
      rclcpp::ServicesQoS().get_rmw_qos_profile(), this->callback_group);

  // NOTE: There is some covariance between the axes, but this is often
  // considered negligible. ( And unsure about how to convert value from data
  // sheet)
  //  Covariance based on datasheet:
  //  https://www.analog.com/media/en/technical-documentation/data-sheets/adxl345.pdf
  //  msg.linear_acceleration_covariance[0] = std::pow(0.75 * 0.00376390 * 9.81,
  //  2);  // Var_x msg.linear_acceleration_covariance[4] = std::pow(0.75 *
  //  0.00376009 * 9.81, 2);  // Var_y msg.linear_acceleration_covariance[8] =
  //  std::pow(1.1 * 0.00349265 * 9.81, 2);   // Var_z

  // Covariance based on:
  // https://github.com/analogdevicesinc/no-OS/blob/c26d25fe7004edc5a5eef40ca36381b08a187a12/drivers/accel/adxl345/adxl345.h#L183
  msg.linear_acceleration_covariance[0] =
      std::pow(0.75 * 0.0039 * 9.81, 2); // Var_x
  msg.linear_acceleration_covariance[4] =
      std::pow(0.75 * 0.0039 * 9.81, 2); // Var_y
  msg.linear_acceleration_covariance[8] =
      std::pow(1.1 * 0.0039 * 9.81, 2); // Var_z

  sensors->add_sens(this->adxl345);
}

void ADXL345_sensor::update() {
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

void ADXL345_sensor::data_callback(std::array<float, 3> acceleration) {

  const std::lock_guard<std::mutex> lock(msg_mutex);
  msg.header = get_header();

  msg.linear_acceleration.x = acceleration[0] * 9.81;
  msg.linear_acceleration.y = acceleration[1] * 9.81;
  msg.linear_acceleration.z = acceleration[2] * 9.81;
  if (this->imu_pub->get_subscription_count() == 0) {
    // No subscribers, so no need to publish
    return;
  }
  imu_pub->publish(msg);
  this->device_timer->reset();
}

void ADXL345_sensor::get_imu_service_callback(
    const mirte_msgs::srv::GetImu::Request::ConstSharedPtr req,
    mirte_msgs::srv::GetImu::Response::SharedPtr res) {
  const std::lock_guard<std::mutex> lock(msg_mutex);
  res->data = sensor_msgs::msg::Imu(msg);
}

std::vector<std::shared_ptr<ADXL345_sensor>>
ADXL345_sensor::get_adxl_modules(NodeData node_data,
                                 std::shared_ptr<Parser> parser,
                                 std::shared_ptr<tmx_cpp::Sensors> sensors) {
  std::vector<std::shared_ptr<ADXL345_sensor>> adxl_modules;
  auto adxl_data = parse_all_modules<ADXL345Data>(parser, node_data.board);
  for (auto adxl : adxl_data) {
    auto adxl_module =
        std::make_shared<ADXL345_sensor>(node_data, adxl, sensors);
    adxl_modules.push_back(adxl_module);
  }
  return adxl_modules;
}
