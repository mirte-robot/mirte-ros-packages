#pragma once
#include <mirte_telemetrix_cpp/actuators/servo_base.hpp>

// TODO: Consider renaming to: PWMServo
class Servo : public ServoBase {
public:
  Servo(NodeData node_data, ServoData servo_data);

  ~Servo();

  virtual bool set_angle_us(uint16_t duty_cycle) override;

  static std::vector<std::shared_ptr<Servo>>
  get_servos(NodeData node_data, std::shared_ptr<Parser> parser);
};