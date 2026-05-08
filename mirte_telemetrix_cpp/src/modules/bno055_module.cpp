#include <functional>
#include <memory>
#include <numbers>

#include <mirte_telemetrix_cpp/bno055_parameters.hpp>
#include <mirte_telemetrix_cpp/modules/bno055_module.hpp>
#include <ranges>
using namespace std::placeholders;

struct Quaterniond {
  double w, x, y, z;
};
Quaterniond toQuaternion(double yaw, double pitch,
                         double roll) // yaw (Z), pitch (Y), roll (X)
{
  // Degree to radius:
  yaw = yaw * M_PI / 180;
  pitch = pitch * M_PI / 180;
  roll = roll * M_PI / 180;

  // Abbreviations for the various angular functions
  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);
  double cr = cos(roll * 0.5);
  double sr = sin(roll * 0.5);

  Quaterniond q;
  q.w = cy * cp * cr + sy * sp * sr;
  q.x = cy * cp * sr - sy * sp * cr;
  q.y = sy * cp * sr + cy * sp * cr;
  q.z = sy * cp * cr - cy * sp * sr;
  return q;
}
BNO055_sensor::BNO055_sensor(NodeData node_data, BNO055Data imu_data,
                             std::shared_ptr<tmx_cpp::Sensors> sensors)
    : Mirte_module(node_data, {imu_data.scl, imu_data.sda},
                   (ModuleData)imu_data),
      data(imu_data) {
  tmx->setI2CPins(imu_data.sda, imu_data.scl, imu_data.port);

  this->BNO055 = std::make_shared<tmx_cpp::BNO055_module>(
      imu_data.port, 10, // TODO: set addr
      std::bind(&BNO055_sensor::data_callback, this, _1));

  imu_pub = nh->create_publisher<sensor_msgs::msg::Imu>(
      "imu/" + this->name + "/data", rclcpp::SystemDefaultsQoS());

  imu_service = nh->create_service<mirte_msgs::srv::GetImu>(
      "imu/" + this->name + "/get_data",
      std::bind(&BNO055_sensor::get_imu_service_callback, this, _1, _2),
      rclcpp::ServicesQoS().get_rmw_qos_profile(), this->callback_group);

  // NOTE: There is some covariance between the axes, but this is often
  // considered negligible.
  //  Covariance based on datasheet:
  //  https://invensense.tdk.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf
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

  sensors->add_sens(this->BNO055);
}

void BNO055_sensor::update() {
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

void BNO055_sensor::data_callback(const tmx_cpp::BNO055_MOD_data &data) {
  const std::lock_guard<std::mutex> lock(msg_mutex);
  msg.header = get_header();

  msg.linear_acceleration.x = data.accel[0];
  msg.linear_acceleration.y = data.accel[1];
  msg.linear_acceleration.z = data.accel[2];

  msg.angular_velocity.x = data.gyro[0];
  msg.angular_velocity.y = data.gyro[1];
  msg.angular_velocity.z = data.gyro[2];

  msg.orientation.x = data.quat.x;
  msg.orientation.y = data.quat.y;
  msg.orientation.z = data.quat.z;
  msg.orientation.w = data.quat.w;
}

void BNO055_sensor::get_imu_service_callback(
    const mirte_msgs::srv::GetImu::Request::ConstSharedPtr req,
    mirte_msgs::srv::GetImu::Response::SharedPtr res) {
  const std::lock_guard<std::mutex> lock(msg_mutex);
  res->data = sensor_msgs::msg::Imu(msg);
}

std::vector<std::shared_ptr<BNO055_sensor>>
BNO055_sensor::get_bno_modules(NodeData node_data,
                               std::shared_ptr<Parser> parser,
                               std::shared_ptr<tmx_cpp::Sensors> sensors) {
  std::vector<std::shared_ptr<BNO055_sensor>> mpu_modules;

  parser->update_params_list_type("modules", "bno055_module_names", "bno055");

  auto param_listener =
      std::make_shared<mirte_telemetrix_cpp_bno055::ParamListener>(parser->nh);
  auto params = param_listener->get_params();
  // get modules list from parser
  // loop over items, get one with type==BNO055
  // add paramlistener to those with new parameters yaml
  auto datas =
      params.modules.bno055_module_names_map |
      std::views::filter([](const auto &pair) {
        const auto &map_power = pair.second;
        return boost::algorithm::to_lower_copy(map_power.type) == "bno055";
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
        return BNO055Data(parser, node_data.board, name, parameters,
                          unused_keys);
      });

  for (auto mpu : datas) {
    auto mpu_module = std::make_shared<BNO055_sensor>(node_data, mpu, sensors);
    mpu_modules.push_back(mpu_module);
  }
  return mpu_modules;
}
