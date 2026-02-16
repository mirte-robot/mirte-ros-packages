#include <mirte_telemetrix_cpp/sensors/base_sensor.hpp>

Mirte_Sensor::Mirte_Sensor(NodeData node_data, std::vector<uint8_t> pins,
                           SensorData data)
    : TelemetrixDevice(node_data, pins, (DeviceData)data) {}

Mirte_Sensor::Mirte_Sensor(NodeData node_data, std::vector<uint8_t> pins,
                           SensorData data,
                           rclcpp::CallbackGroupType callback_group_type)
    : TelemetrixDevice(node_data, pins, (DeviceData)data, callback_group_type) {
}

void Mirte_Sensor::device_timer_callback() { update(); }