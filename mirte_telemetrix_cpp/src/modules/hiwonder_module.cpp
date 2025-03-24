#include <chrono>
#include <functional>
#include <thread>

using namespace std::chrono_literals;

#include <rclcpp/callback_group.hpp>

#include <mirte_telemetrix_cpp/modules/hiwonder_module.hpp>

using namespace std::placeholders; // for _1, _2, _3...

// TODO: USE TO DEVICE_TIMER

// hiwonder bus
// TODO: Maybe add a lock future, to prevent outputting warnings during other
// modules...?
HiWonderBus_module::HiWonderBus_module(
    NodeData node_data, HiWonderBusData bus_data,
    std::shared_ptr<tmx_cpp::Modules> modules)
    : Mirte_module(node_data, {bus_data.tx_pin, bus_data.rx_pin},
                   (ModuleData)bus_data,
                   rclcpp::CallbackGroupType::MutuallyExclusive),
      data(bus_data) {
  this->device_timer->cancel();
  this->logger =
      this->logger.get_child(data.get_device_class()).get_child(data.name);

  // Create a list of ID's
  std::vector<uint8_t> servo_ids;
  // Don't pre-add ids since it can cause errors on missing servos
  // for (auto servo : this->data.servos)
  //   servo_ids.push_back(servo->id);

  this->bus = std::make_shared<tmx_cpp::HiwonderServo_module>(
      this->data.uart_port, this->data.rx_pin, this->data.tx_pin, servo_ids,
      std::bind(&HiWonderBus_module::position_cb, this, _1));

  modules->add_mod(this->bus);

  auto servo_group = this->data.group_name;
  if (!servo_group.ends_with('/'))
    servo_group.push_back('/');

  std::this_thread::sleep_for(1.20s);
  for (auto servo_data : this->data.servos) {
    if (this->bus->verify_id(servo_data->id))
      this->servos.push_back(std::make_shared<Hiwonder_servo>(
          node_data, servo_data, this->bus, servo_group, bus_data.duration,
          this->callback_group));
    else
      RCLCPP_ERROR(
          this->logger,
          "HiWonder Servo '%s' is ignored as its ID [%d] was not found.",
          servo_data->name.c_str(), servo_data->id);
  }

  // Create Bus ROS services
  this->enable_all_servos_service = nh->create_service<std_srvs::srv::SetBool>(
      "servo/" + servo_group + "enable_all_servos",
      std::bind(&HiWonderBus_module::enable_service_callback, this, _1, _2),
      rclcpp::ServicesQoS().get_rmw_qos_profile(), this->callback_group);
}

// TODO: Make result actually Reflect reality
void HiWonderBus_module::enable_service_callback(
    const std_srvs::srv::SetBool::Request::ConstSharedPtr req,
    std_srvs::srv::SetBool::Response::SharedPtr res) {
  // // TODO: TEMP TEST
  // std::cout << (int)bus->get_offset(4).value_or(99) <<std::endl;
  // auto [min, max] = bus->get_range(4).value_or(std::make_tuple(0,0));
  // std::cout << min << " | " << max <<std::endl;
  // std::cout << bus->verify_id(4) << std::endl;
  // // TODO: TEMP TEST

  res->success = this->bus->set_enabled_all(req->data);
  res->message = req->data ? "Enabled" : "Disabled";
}

std::vector<std::shared_ptr<HiWonderBus_module>>
HiWonderBus_module::get_hiwonder_modules(
    NodeData node_data, std::shared_ptr<Parser> parser,
    std::shared_ptr<tmx_cpp::Modules> modules) {
  std::vector<std::shared_ptr<HiWonderBus_module>> hiwonder_modules;
  auto hiwonder_data =
      parse_all_modules<HiWonderBusData>(parser, node_data.board);
  for (auto hiwonder : hiwonder_data) {
    auto hiwonder_module =
        std::make_shared<HiWonderBus_module>(node_data, hiwonder, modules);
    hiwonder_modules.push_back(hiwonder_module);
  }
  return hiwonder_modules;
}

void HiWonderBus_module::position_cb(
    std::vector<std::tuple<uint8_t, tmx_cpp::HiwonderServo_module::Servo_pos>>
        pos) {
  for (auto &[idx, p] : pos) {
    if (idx >= this->servos.size())
      continue;
    auto servo = this->servos[idx];
    assert(servo->servo_data->id == p.id);
    servo->position_cb(p);
  }
}
