#include <functional>
#include <stdint.h>

#include <chrono>

using namespace std::chrono_literals;

#include <mirte_telemetrix_cpp/modules/as5600_module.hpp>

using namespace std::placeholders; // for _1, _2, _3...

AS5600_sensor::AS5600_sensor(NodeData node_data, AS5600Data as5600_data,
                             std::shared_ptr<tmx_cpp::Sensors> modules)
    : Mirte_module(node_data, {as5600_data.scl, as5600_data.sda},
                   (ModuleData)as5600_data),
      data(as5600_data) {
  tmx->setI2CPins(as5600_data.sda, as5600_data.scl, as5600_data.port);
  std::vector<int> channels;
  for (const auto &encoder : as5600_data.encoders) {
    channels.push_back(std::get<0>(encoder));
    auto pub = node_data.nh->create_publisher<std_msgs::msg::Float32>(
        "angle/" + as5600_data.name + "/" + std::get<1>(encoder),
        rclcpp::SystemDefaultsQoS());
    this->angle_pubs.push_back(pub);
    this->angles.push_back(0.0f);
    this->angle_msgs.push_back(std_msgs::msg::Float32());
  }
  this->as5600 = std::make_shared<tmx_cpp::AS5600_tmx_sensor>(
      as5600_data.port, as5600_data.addr, channels,
      std::bind(&AS5600_sensor::data_callback, this, _1));

  modules->add_sens(this->as5600);
  // add timer
  node_data.nh->create_wall_timer(100ms,
                                  std::bind(&AS5600_sensor::update, this));
}

// TODO: Maybe add mutex lock-out although not fully necessary
void AS5600_sensor::update() {
  auto header = get_header();
  for (int i = 0; i < this->angles.size(); i++) {
    auto &msg = angle_msgs[i];
    // msg.header = header;
    msg.data = this->angles[i];
    angle_pubs[i]->publish(msg);
  }
}

void AS5600_sensor::data_callback(tmx_cpp::AS5600_cb_t::argument_type data) {
  for (int i = 0; i < data.size(); i++) {
    auto &[channel, angle_rad, angle_ticks] = data[i];
    this->angles[i] = angle_rad;
  }
  // this->update();
}

std::vector<std::shared_ptr<AS5600_sensor>>
AS5600_sensor::get_as5600_modules(NodeData node_data,
                                  std::shared_ptr<Parser> parser,
                                  std::shared_ptr<tmx_cpp::Sensors> sensors) {
  auto as5600_modules = parse_all_modules<AS5600Data>(parser, node_data.board);
  std::vector<std::shared_ptr<AS5600_sensor>> modules;
  for (const auto &as5600_data : as5600_modules) {
    auto module =
        std::make_shared<AS5600_sensor>(node_data, as5600_data, sensors);
    modules.push_back(module);
  }
  return modules;
}