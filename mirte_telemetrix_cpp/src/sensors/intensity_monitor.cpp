#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <mirte_telemetrix_cpp/sensors/intensity_monitor.hpp>

#include <mirte_msgs/msg/intensity.hpp>
#include <mirte_msgs/msg/intensity_digital.hpp>
#include <ranges>
IntensityMonitor::IntensityMonitor(NodeData node_data, std::vector<pin_t> pins,
                                   IntensityData intensity_data)
    : Mirte_Sensor(node_data, pins, (SensorData)intensity_data),
      intensity_data(intensity_data) {}

std::vector<std::shared_ptr<IntensityMonitor>>
IntensityMonitor::get_intensity_monitors(NodeData node_data,
                                         std::shared_ptr<Parser> parser) {
  std::vector<std::shared_ptr<IntensityMonitor>> sensors;

  // loop over intensity_monitor_types, for each, read parameters (after fixing
  // types) and create monitors for each with the correct 'type'
  for (const auto type : intensity_monitor_types) {
    auto monitors = get_intensity_monitors(node_data, parser, type);
    sensors.insert(sensors.end(), monitors.begin(), monitors.end());
  }

  return sensors;
}

std::vector<std::shared_ptr<IntensityMonitor>>
IntensityMonitor::get_intensity_monitors(NodeData node_data,
                                         std::shared_ptr<Parser> parser,
                                         std::string type) {
  std::vector<std::shared_ptr<IntensityMonitor>> sensors;

  /*
  TYPE:
    NAME:
      pins:
        analog: x
        digital: y
      connector: z
      name: NAME

  */

  auto found_modules = parser->update_params_list(
      type, type + ".intensity_keys", [](auto name) { return true; });
  parser->fix_param_type_str_modules(type, found_modules,
                                     {"pins.analog", "pins.digital"});

  auto param_listener =
      std::make_shared<mirte_telemetrix_cpp_intensity::ParamListener>(
          parser->nh, type);
  auto params = param_listener->get_params();
  auto ir_sensors =
      params.intensity_keys_map |
      std::views::transform([&](const auto &pair) -> IntensityData {
        const auto &name = pair.first;
        const auto &map_ina = pair.second;
        std::map<std::string, rclcpp::ParameterValue> parameters;
        parameters["type"] = rclcpp::ParameterValue(type);
        parameters["connector"] = rclcpp::ParameterValue(map_ina.connector);
        parameters["pins.analog"] = rclcpp::ParameterValue(map_ina.pins.analog);
        parameters["pins.digital"] =
            rclcpp::ParameterValue(map_ina.pins.digital);

        std::set<std::string> unused_keys = get_keys(parameters);
        return IntensityData(parser, node_data.board, name, parameters,
                             unused_keys);
      }) |
      std::views::transform(
          [&](auto const &ir_data)
              -> std::vector<std::shared_ptr<IntensityMonitor>> {
            std::vector<std::shared_ptr<IntensityMonitor>> result;
            bool has_analog = ir_data.a_pin != (pin_t)-1;
            bool has_digital = ir_data.d_pin != (pin_t)-1;
            if (ir_data.a_pin != (pin_t)-1) {
              result.push_back(std::make_shared<AnalogIntensityMonitor>(
                  node_data, ir_data, !has_digital));
            }
            if (ir_data.d_pin != (pin_t)-1) {
              result.push_back(std::make_shared<DigitalIntensityMonitor>(
                  node_data, ir_data, !has_analog));
            }
            return result;
          });

  for (const auto &ir_sensor_vec : ir_sensors) {
    for (auto &ir_sensor : ir_sensor_vec) {
      sensors.push_back(ir_sensor);
    }
  }

  return sensors;
}

void DigitalIntensityMonitor::data_callback(uint16_t value) {
  this->value = value;
}

void DigitalIntensityMonitor::update() {
  if (this->intensity_pub->get_subscription_count() == 0) {
    // No subscribers, so no need to publish
    return;
  }
  auto msg = mirte_msgs::build<mirte_msgs::msg::IntensityDigital>()
                 .header(get_header()) // Build the message
                 .value(value);

  intensity_pub->publish(msg);
}

DigitalIntensityMonitor::DigitalIntensityMonitor(NodeData node_data,
                                                 IntensityData intensity_data,
                                                 bool single_monitor)
    : IntensityMonitor(node_data, {intensity_data.d_pin}, intensity_data) {
  using namespace std::placeholders;
  // override default frame id, as it otherwise will be
  // intensitydata::get_device_class
  this->frame_id = intensity_data.frame_id;

  auto topic_name = intensity_data.type + "/" + intensity_data.name;
  if (!single_monitor) {
    topic_name += "/digital";
  }
  // Use default QOS for sensor publishers as specified in REP2003
  intensity_pub = nh->create_publisher<mirte_msgs::msg::IntensityDigital>(
      topic_name, rclcpp::SystemDefaultsQoS());

  intensity_service = nh->create_service<mirte_msgs::srv::GetIntensityDigital>(
      intensity_data.type + "/" + intensity_data.name + "/get_digital",
      std::bind(&DigitalIntensityMonitor::service_callback, this, _1, _2),
      rclcpp::ServicesQoS().get_rmw_qos_profile(), this->callback_group);

  tmx->setPinMode(intensity_data.d_pin, tmx_cpp::TMX::PIN_MODES::DIGITAL_INPUT,
                  true, 0);
  tmx->add_digital_callback(intensity_data.d_pin, [this](auto pin, auto value) {
    this->data_callback(value);
  });
}

AnalogIntensityMonitor::AnalogIntensityMonitor(NodeData node_data,
                                               IntensityData intensity_data,
                                               bool single_monitor)
    : IntensityMonitor(node_data, {intensity_data.a_pin}, intensity_data) {
  using namespace std::placeholders;
  // override default frame id, as it otherwise will be
  // intensitydata::get_device_class
  this->frame_id = intensity_data.frame_id;

  auto topic_name = intensity_data.type + "/" + intensity_data.name;
  if (!single_monitor) {
    topic_name += "/analog";
  }

  // Use default QOS for sensor publishers as specified in REP2003
  intensity_pub = nh->create_publisher<mirte_msgs::msg::Intensity>(
      topic_name, rclcpp::SystemDefaultsQoS());

  intensity_service = nh->create_service<mirte_msgs::srv::GetIntensity>(
      intensity_data.type + "/" + intensity_data.name + "/get_analog",
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
}

void AnalogIntensityMonitor::update() {
  if (this->intensity_pub->get_subscription_count() == 0) {
    // No subscribers, so no need to publish
    return;
  }
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
