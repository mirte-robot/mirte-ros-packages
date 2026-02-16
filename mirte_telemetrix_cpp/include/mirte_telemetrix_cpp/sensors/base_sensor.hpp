#pragma once

#include <mirte_telemetrix_cpp/device.hpp>
#include <mirte_telemetrix_cpp/node_data.hpp>

#include <mirte_telemetrix_cpp/parsers/sensors/base_sensor_data.hpp>

// namespace mirte_telemetrix_cpp {

class Mirte_Sensor : public TelemetrixDevice {
public:
  virtual void update() override = 0;
  Mirte_Sensor(NodeData node_data, std::vector<uint8_t> pins, SensorData data);
  Mirte_Sensor(NodeData node_data, std::vector<uint8_t> pins, SensorData data,
               rclcpp::CallbackGroupType callback_group_type);
  virtual ~Mirte_Sensor() = default;

  virtual void device_timer_callback() override;
};

// }  // namespace mirte_telemetrix_cpp
