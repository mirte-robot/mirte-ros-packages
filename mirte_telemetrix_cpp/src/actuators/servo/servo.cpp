#include <vector>

#include <mirte_telemetrix_cpp/actuators/servo/servo.hpp>

std::vector<std::shared_ptr<Servo>>
Servo::get_servos(NodeData node_data, std::shared_ptr<Parser> parser) {
  std::vector<std::shared_ptr<Servo>> actuators;
  auto servos = parse_all<ServoData>(parser, node_data.board);
  for (auto servo : servos) {
    actuators.push_back(std::make_shared<Servo>(node_data, servo));
    // std::cout << "Add Servo: " << servo.name << std::endl;
  }
  return actuators;
}

Servo::Servo(NodeData node_data, ServoData servo_data)
    : ServoBase(node_data, {servo_data.pin}, servo_data) {
  tmx->attach_servo(data.pin, data.min_pulse, data.max_pulse);
}

Servo::~Servo() { tmx->detach_servo(data.pin); }

bool Servo::set_angle_us(uint16_t duty_cycle) {
  tmx->write_servo(data.pin, std::clamp(duty_cycle, (uint16_t)data.min_pulse,
                                        (uint16_t)data.max_pulse));
  return true;
}