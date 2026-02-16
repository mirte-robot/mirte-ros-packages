#pragma once

#include <mirte_telemetrix_cpp/device.hpp>

#include <mirte_telemetrix_cpp/parsers/modules/module_data.hpp>

class Mirte_module : public TelemetrixDevice {
public:
  // ??? Pins was added
  Mirte_module(NodeData node_data, std::vector<pin_t> pins,
               ModuleData module_data);
  Mirte_module(NodeData node_data, std::vector<pin_t> pins,
               ModuleData module_data,
               rclcpp::CallbackGroupType callback_group_type);

  virtual ~Mirte_module() = default;
  virtual void update() override{};
  virtual void device_timer_callback() override;
};
