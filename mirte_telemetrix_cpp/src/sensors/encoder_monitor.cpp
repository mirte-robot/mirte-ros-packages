#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

#include <mirte_telemetrix_cpp/sensors/encoder_monitor.hpp>

#include <mirte_msgs/msg/encoder.hpp>
#include <mirte_msgs/srv/get_encoder.hpp>
#include <ranges>

#include <chrono>

EncoderMonitor::EncoderMonitor(NodeData node_data, EncoderData encoder_data)
    : Mirte_Sensor(node_data, {encoder_data.pinA, encoder_data.pinB},
                   (SensorData)encoder_data),
      encoder_data(encoder_data) {
  using namespace std::placeholders;

  // Use default QOS for sensor publishers as specified in REP2003
  encoder_pub = nh->create_publisher<mirte_msgs::msg::Encoder>(
      "encoder/" + encoder_data.name, rclcpp::SystemDefaultsQoS());

  encoder_service = nh->create_service<mirte_msgs::srv::GetEncoder>(
      "encoder/" + encoder_data.name + "/get_encoder",
      std::bind(&EncoderMonitor::service_callback, this, _1, _2),
      rclcpp::ServicesQoS().get_rmw_qos_profile(), this->callback_group);

  tmx->attach_encoder(
      encoder_data.pinA, encoder_data.pinB,
      [this](auto pin, auto value) { this->data_callback(value); });
}
using namespace std::chrono_literals;

// To have a steady timestamp for control purposes, don't use timestamp, but use
// the frequency of the encoder to calculate the timestamp. Pico should be
// sending at the correct rate, so this should be good and filters out any
// jitter in the timestamp of the messages.
std_msgs::msg::Header EncoderMonitor::create_header() {
  std_msgs::msg::Header header = this->get_header();
  // Update_interval in seconds, based on the frequency of the encoder
  auto update_interval =
      std::chrono::duration<double>(1.0s / this->encoder_data.frequency);
  if (this->last_update_time.nanoseconds() != 0) {
    // If the message already has a timestamp, use that and increase with
    // frequency.
    this->last_update_time += update_interval;

    auto diff = this->nh->now() - this->last_update_time;
    // if pico forgot to send or message missing, forward the timestamp some
    // steps.
    if (diff >= 2 * update_interval) {
      // count is int, so need to use ms for calculations.
      this->last_update_time +=
          std::floor((diff.to_chrono<std::chrono::milliseconds>().count() /
                      (update_interval.count() * 1000))) *
          (update_interval);
    }

  } else {
    this->last_update_time = this->nh->now();
  }
  header.stamp = this->last_update_time;
  return header;
}

void EncoderMonitor::data_callback(int16_t value) {
  if (this->encoder_data.inverted) {
    value = -value;
  }
  this->value += (int32_t)value;
  this->msg = mirte_msgs::build<mirte_msgs::msg::Encoder>()
                  .header(create_header()) // Build the message
                  .value(this->value);
}

void EncoderMonitor::update() {
  if (this->encoder_pub->get_subscription_count() == 0) {
    // No subscribers, so no need to publish
    return;
  }
  // Pico always sends the encoder value, so we can just publish it, header is
  // updated in the data_callback
  encoder_pub->publish(msg);
}

void EncoderMonitor::service_callback(
    const mirte_msgs::srv::GetEncoder::Request::ConstSharedPtr req,
    mirte_msgs::srv::GetEncoder::Response::SharedPtr res) {
  res->data = value;
}

std::vector<std::shared_ptr<EncoderMonitor>>
EncoderMonitor::get_encoder_monitors(NodeData node_data,
                                     std::shared_ptr<Parser> parser) {
  std::vector<std::shared_ptr<EncoderMonitor>> sensors;
  auto encoders =
      parser->params_object.encoder.encoders_map |
      std::views::transform([&](const auto &pair) {
        const auto &name = pair.first;
        const auto &map_encoder = pair.second;
        std::map<std::string, rclcpp::ParameterValue> parameters;

        parameters["ticks_per_wheel"] =
            rclcpp::ParameterValue(map_encoder.ticks_per_wheel);
        parameters["device"] = rclcpp::ParameterValue(map_encoder.device);
        parameters["connector"] = rclcpp::ParameterValue(map_encoder.connector);
        parameters["pins.pin"] = rclcpp::ParameterValue(map_encoder.pins.pin);
        parameters["pins.A"] = rclcpp::ParameterValue(map_encoder.pins.A);
        parameters["pins.B"] = rclcpp::ParameterValue(map_encoder.pins.B);
        parameters["frame_id"] = rclcpp::ParameterValue(map_encoder.frame_id);
        parameters["inverted"] = rclcpp::ParameterValue(map_encoder.inverted);
        auto unused_keys = get_keys(parameters);
        std::set<std::string> unused_keys_set(unused_keys.begin(),
                                              unused_keys.end());
        return std::make_shared<EncoderMonitor>(
            node_data, EncoderData(parser, node_data.board, name, parameters,
                                   unused_keys_set));
      });
  sensors.assign(encoders.begin(), encoders.end());
  return sensors;
}
