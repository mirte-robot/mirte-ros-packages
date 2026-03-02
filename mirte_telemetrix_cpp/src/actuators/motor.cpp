#include <functional>
#include <vector>

#include <rclcpp/callback_group.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription_options.hpp>

#include <mirte_telemetrix_cpp/actuators/motor.hpp>
#include <mirte_telemetrix_cpp/actuators/motor/ddp_motor.hpp>
#include <mirte_telemetrix_cpp/actuators/motor/dp_motor.hpp>
#include <mirte_telemetrix_cpp/actuators/motor/pp_motor.hpp>
#include <mirte_telemetrix_cpp/device.hpp>
#include <ranges>
using namespace std::placeholders;

std::vector<std::shared_ptr<TelemetrixDevice>>
Motor::get_motors(NodeData node_data, std::shared_ptr<Parser> parser) {
  std::vector<std::shared_ptr<TelemetrixDevice>> motors;
  auto motor_datas =
      parser->params_object.motor.motors_map |
      std::views::transform([&](const auto &pair) {
        const auto &name = pair.first;
        const auto &map_motor = pair.second;
        std::map<std::string, rclcpp::ParameterValue> parameters;
        parameters["device"] = rclcpp::ParameterValue(map_motor.device);
        parameters["connector"] = rclcpp::ParameterValue(map_motor.connector);
        parameters["pins.p1"] = rclcpp::ParameterValue(map_motor.pins.p1);
        parameters["pins.p2"] = rclcpp::ParameterValue(map_motor.pins.p2);
        parameters["pins.d1"] = rclcpp::ParameterValue(map_motor.pins.d1);
        parameters["pins.d2"] = rclcpp::ParameterValue(map_motor.pins.d2);
        parameters["type"] = rclcpp::ParameterValue(map_motor.type);
        parameters["inverted"] = rclcpp::ParameterValue(map_motor.inverted);
        parameters["stop_mode"] = rclcpp::ParameterValue(map_motor.stop_mode);
        std::set<std::string> unused_keys = get_keys(parameters);
        return MotorData(parser, node_data.board, name, parameters,
                         unused_keys);
      });
  for (auto motor_data : motor_datas) {
    if (motor_data.check()) {
      if (motor_data.type == MotorData::MotorType::PP) {
        auto motor = std::make_shared<PPMotor>(node_data, motor_data);
        motor->start();
        motors.push_back(motor);
      } else if (motor_data.type == MotorData::MotorType::DP) {
        auto motor = std::make_shared<DPMotor>(node_data, motor_data);
        motor->start();
        motors.push_back(motor);
      } else if (motor_data.type == MotorData::MotorType::DDP) {
        auto motor = std::make_shared<DDPMotor>(node_data, motor_data);
        motor->start();
        motors.push_back(motor);
      }
    }
  }
  return motors;
}

Motor::Motor(NodeData node_data, std::vector<pin_t> pins, MotorData motor_data)
    : Motor(node_data, pins, (DeviceData)motor_data, motor_data.inverted,
            node_data.board->get_max_pwm()) {}

Motor::Motor(NodeData node_data, std::vector<pin_t> pins, DeviceData data,
             bool inverted, int max_pwm)
    : TelemetrixDevice(node_data, pins, data,
                       rclcpp::CallbackGroupType::MutuallyExclusive),
      inverted(inverted), max_pwm(max_pwm) {
  set_speed_service = nh->create_service<mirte_msgs::srv::SetMotorSpeed>(
      "motor/" + this->name + "/set_speed",
      std::bind(&Motor::set_speed_service_callback, this, _1, _2),
      rclcpp::ServicesQoS().get_rmw_qos_profile(), this->callback_group);

  rclcpp::SubscriptionOptions options;
  options.callback_group = this->callback_group;
  speed_subscription = nh->create_subscription<std_msgs::msg::Int32>(
      "motor/" + this->name + "/speed", rclcpp::SystemDefaultsQoS(),
      std::bind(&Motor::speed_subscription_callback, this, _1), options);

  // this->device_timer->cancel();
}

void Motor::set_speed_service_callback(
    const mirte_msgs::srv::SetMotorSpeed::Request::ConstSharedPtr req,
    mirte_msgs::srv::SetMotorSpeed::Response::SharedPtr res) {
  this->set_speed(req->speed);
  res->status = true;
}

void Motor::speed_subscription_callback(const std_msgs::msg::Int32 &msg) {
  this->set_speed(msg.data);
}
