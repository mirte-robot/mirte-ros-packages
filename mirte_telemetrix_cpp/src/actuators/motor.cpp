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

using namespace std::placeholders;

std::vector<std::shared_ptr<TelemetrixDevice>>
Motor::get_motors(NodeData node_data, std::shared_ptr<Parser> parser) {
  std::vector<std::shared_ptr<TelemetrixDevice>> motors;
  auto motor_datas = parse_all<MotorData>(parser, node_data.board);
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
