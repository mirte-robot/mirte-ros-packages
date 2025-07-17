#include <functional>

#include <mirte_telemetrix_cpp/device.hpp>

TelemetrixDevice::TelemetrixDevice(NodeData node_data,
                                   std::vector<uint8_t> pins, DeviceData data)
    : TelemetrixDevice(node_data, pins, data,
                       rclcpp::CallbackGroupType::MutuallyExclusive) {}

TelemetrixDevice::TelemetrixDevice(
    NodeData node_data, std::vector<uint8_t> pins, DeviceData data,
    rclcpp::CallbackGroupType callback_group_type)
    : tmx(node_data.tmx), nh(node_data.nh), board(node_data.board), pins(pins),
      logger(nh->get_logger()), name(data.name), frame_id(data.frame_id) {
  this->callback_group = nh->create_callback_group(callback_group_type);

  // TODO: Starting from Jazzy Set auto_start false, so they can be started at
  // the end of the initialization.

  // TODO: Could do something like this
  // https://github.com/ros2/demos/blob/rolling/demo_nodes_cpp/src/timers/reuse_timer.cpp
  // To give more room for recieving the callback while the rate can be fixed at
  // the max_frequency

  // TODO: Compensation could be depended on the amount of cores available.
  // To compensate for communication latencies, scale the period up by 20% (on
  // my machine with 16 cores)

  node_data.add_timer(
      data.duration, [this]() { this->device_timer_callback(); });
    // this->device_timer = nh->create_wall_timer(
    //     data.duration, std::bind(&TelemetrixDevice::device_timer_callback, this),
    //     this->callback_group);
}
