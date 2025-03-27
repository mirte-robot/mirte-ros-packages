#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <mirte_telemetrix_cpp/sensors/intensity_monitor.hpp>

#include <mirte_msgs/msg/intensity.hpp>
#include <mirte_msgs/msg/intensity_digital.hpp>

IntensityMonitor::IntensityMonitor(NodeData node_data, std::vector<pin_t> pins,
                                   IntensityData intensity_data)
    : Mirte_Sensor(node_data, pins, (SensorData)intensity_data),
      intensity_data(intensity_data) {}

std::vector<std::shared_ptr<IntensityMonitor>>
IntensityMonitor::get_intensity_monitors(NodeData node_data,
                                         std::shared_ptr<Parser> parser) {
  std::vector<std::shared_ptr<IntensityMonitor>> sensors;
  auto ir_sensors = parse_all<IntensityData>(parser, node_data.board);
  for (auto ir_data : ir_sensors) {
    if (ir_data.a_pin != (pin_t)-1) {
      sensors.push_back(
          std::make_shared<AnalogIntensityMonitor>(node_data, ir_data));
      // std::cout << "Add Analog Intensity: " << ir_data.name << std::endl;
    }
    if (ir_data.d_pin != (pin_t)-1) {
      sensors.push_back(
          std::make_shared<DigitalIntensityMonitor>(node_data, ir_data));
      // std::cout << "Add Digital Intensity: " << ir_data.name << std::endl;
    }
  }
  return sensors;
}

void DigitalIntensityMonitor::data_callback(uint16_t value) {
  this->value = value;
  this->update();
  this->device_timer->reset();
}

void DigitalIntensityMonitor::update() {
  auto msg = mirte_msgs::build<mirte_msgs::msg::IntensityDigital>()
                 .header(get_header()) // Build the message
                 .value(value);

  intensity_pub->publish(msg);
}

DigitalIntensityMonitor::DigitalIntensityMonitor(NodeData node_data,
                                                 IntensityData intensity_data)
    : IntensityMonitor(node_data, {intensity_data.d_pin}, intensity_data) {
  using namespace std::placeholders;

  // Use default QOS for sensor publishers as specified in REP2003
  intensity_pub = nh->create_publisher<mirte_msgs::msg::IntensityDigital>(
      "intensity/" + intensity_data.name + "/digital",
      rclcpp::SystemDefaultsQoS());

  intensity_service = nh->create_service<mirte_msgs::srv::GetIntensityDigital>(
      "intensity/" + intensity_data.name + "/get_digital",
      std::bind(&DigitalIntensityMonitor::service_callback, this, _1, _2),
      rclcpp::ServicesQoS().get_rmw_qos_profile(), this->callback_group);

  tmx->setPinMode(intensity_data.d_pin, tmx_cpp::TMX::PIN_MODES::DIGITAL_INPUT,
                  true, 0);
  tmx->add_digital_callback(intensity_data.d_pin, [this](auto pin, auto value) {
    this->data_callback(value);
  });
}

AnalogIntensityMonitor::AnalogIntensityMonitor(NodeData node_data,
                                               IntensityData intensity_data)
    : IntensityMonitor(node_data, {intensity_data.a_pin}, intensity_data) {
  using namespace std::placeholders;

  // Use default QOS for sensor publishers as specified in REP2003
  intensity_pub = nh->create_publisher<mirte_msgs::msg::Intensity>(
      "intensity/" + intensity_data.name, rclcpp::SystemDefaultsQoS());

  intensity_service = nh->create_service<mirte_msgs::srv::GetIntensity>(
      "intensity/" + intensity_data.name + "/get_analog",
      std::bind(&AnalogIntensityMonitor::service_callback, this, _1, _2),
      rclcpp::ServicesQoS().get_rmw_qos_profile(), this->callback_group);

  tmx->setPinMode(intensity_data.a_pin, tmx_cpp::TMX::PIN_MODES::ANALOG_INPUT,
                  true, 0);
  tmx->add_analog_callback(intensity_data.a_pin, [this](auto pin, auto value) {
    this->data_callback(value);
  });
}

void AnalogIntensityMonitor::data_callback(uint16_t value) {
  this->value = value;
  this->update();
  this->device_timer->reset();
}

void AnalogIntensityMonitor::update() {
  auto msg = mirte_msgs::build<mirte_msgs::msg::Intensity>()
                 .header(get_header()) // Build the message
                 .value(value);

  intensity_pub->publish(msg);
}

void DigitalIntensityMonitor::service_callback(
    const mirte_msgs::srv::GetIntensityDigital::Request::ConstSharedPtr req,
    mirte_msgs::srv::GetIntensityDigital::Response::SharedPtr res) {
  res->data = value;
}

void AnalogIntensityMonitor::service_callback(
    const mirte_msgs::srv::GetIntensity::Request::ConstSharedPtr req,
    mirte_msgs::srv::GetIntensity::Response::SharedPtr res) {
  res->data = value;
}
