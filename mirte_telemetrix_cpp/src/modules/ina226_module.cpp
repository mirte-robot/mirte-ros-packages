#include <fstream>
#include <functional>
#include <stdint.h>

#ifdef WITH_GPIO
#include <chrono>

using namespace std::chrono_literals;
#endif

#include <mirte_telemetrix_cpp/modules/ina226_module.hpp>

using namespace std::placeholders; // for _1, _2, _3...

INA226_sensor::INA226_sensor(NodeData node_data, INA226Data ina_data,
                             std::shared_ptr<tmx_cpp::Sensors> sensors,
                             std::shared_ptr<tmx_cpp::Modules> modules)
    : Mirte_module(node_data, {ina_data.scl, ina_data.sda},
                   (ModuleData)ina_data),
      data(ina_data) {
  tmx->setI2CPins(ina_data.sda, ina_data.scl, ina_data.port);
  this->used_time = nh->now();
  this->total_used_mAh = 0;

  this->ina226 = std::make_shared<tmx_cpp::INA226_module>(
      ina_data.port, ina_data.addr,
      std::bind(&INA226_sensor::data_callback, this, _1, _2));

  // Use default QOS for sensor publishers as specified in REP2003
  this->battery_pub = nh->create_publisher<sensor_msgs::msg::BatteryState>(
      "power/" + this->name, rclcpp::SystemDefaultsQoS());

  this->used_pub = nh->create_publisher<std_msgs::msg::Int32>(
      "power/" + this->name + "/used", rclcpp::SystemDefaultsQoS());

  this->shutdown_service = nh->create_service<std_srvs::srv::SetBool>(
      "power/" + this->name + "/shutdown",
      std::bind(&INA226_sensor::shutdown_robot_service_callback, this, _1, _2),
      rclcpp::ServicesQoS().get_rmw_qos_profile(), this->callback_group);

  sensors->add_sens(this->ina226);
  // TODO: add shutdown service

  if (!ina_data.disable_shutdown_relay) {
    this->shutdown_relay_module =
        std::make_shared<tmx_cpp::Shutdown_relay_module>(
            ina_data.shutdown_relay_pin, ina_data.turn_off_time,
            ina_data.shutdown_relay_off_value);
    modules->add_mod(this->shutdown_relay_module);
    std::cout << "Attached shutdown relay to pin: "
              << (int)ina_data.shutdown_relay_pin << std::endl;
    std::cout << "Shutdown relay off value: "
              << ina_data.shutdown_relay_off_value << std::endl;
    std::cout << "Shutdown relay turn off time: " << ina_data.turn_off_time
              << "s" << std::endl;
    if (ina_data.shutdown_switch_in_pin != 0xFF) {
      tmx->setPinMode(ina_data.shutdown_switch_in_pin,
                      tmx_cpp::TMX::PIN_MODES::DIGITAL_INPUT);

      this->tmx->add_digital_callback(
          ina_data.shutdown_switch_in_pin,
          std::bind(&INA226_sensor::switch_cb, this, _1, _2));
      node_data.add_timer(0.2s, std::bind(&INA226_sensor::update_sw, this));
    }
  }
#ifdef WITH_GPIO // LED Battery indicator
  if (this->data.use_percentage_led) {
    node_data.add_timer(
        0.5s, std::bind(&INA226_sensor::battery_led_timer_callback, this));
  }
#endif
}

void INA226_sensor::write_soc(float soc) {
  std::ofstream soc_file;
  soc_file.open("/tmp/batteryState");
  if (soc_file.is_open()) {
    soc_file << soc << std::endl;
    soc_file << soc << std::endl; // for bw compatibility with shutdown script
    soc_file.close();
  } else {
    RCLCPP_ERROR(this->logger, "Could not open soc file for writing");
  }
}

// TODO: Maybe add mutex lock-out although not fully necessary
void INA226_sensor::update() { // only publish at 1Hz
  if (this->battery_pub->get_subscription_count() > 0) {
    auto msg = sensor_msgs::msg::BatteryState();
    msg.header = get_header();

    msg.voltage = voltage_;
    msg.current = current_;
    msg.percentage = calc_soc(voltage_);
    this->write_soc(msg.percentage);
    this->battery_pub->publish(msg);
  }
}

void INA226_sensor::update_sw() {
  if (this->switch_turn_off_time.seconds() > 0) {
    std::cout
        << "Shutdown switch has been triggered, checking if should turn off"
        << std::endl;
    auto current_time = this->nh->now();
    auto duration = current_time - this->switch_turn_off_time;
    std::cout << "Triggering turn off maybe by switch" << duration.seconds()
              << std::endl;
    std::cout << "you have "
              << (this->data.shutdown_switch_time_sec - duration.seconds())
              << "s left" << std::endl;
    if (duration.seconds() > this->data.shutdown_switch_time_sec) {
      std::cout << "Turning off by switch" << std::endl;
      this->shutdown_robot();
    }
  }
}

void INA226_sensor::data_callback(float voltage, float current) {
  std::cout << "INA226 Data callback: Voltage: " << voltage
            << " Current: " << current << std::endl;
  voltage_ = voltage;
  current_ = current;
  // this->integrate_usage(current_);
  this->check_soc(voltage_, current_);
}

float INA226_sensor::calc_soc(float voltage) {
  const auto CELL_COUNT = 3;
  voltage = voltage / CELL_COUNT;
  std::vector<std::pair<float, float>> soc_levels{
      // single cell voltages vs percentage
      {0, 0},       {3.27, 0.00}, {3.61, 0.05}, {3.69, 0.10}, {3.71, 0.15},
      {3.73, 0.20}, {3.75, 0.25}, {3.77, 0.30}, {3.79, 0.35}, {3.80, 0.40},
      {3.82, 0.45}, {3.84, 0.50}, {3.85, 0.55}, {3.87, 0.60}, {3.91, 0.65},
      {3.95, 0.70}, {3.98, 0.75}, {4.02, 0.80}, {4.08, 0.85}, {4.11, 0.90},
      {4.15, 0.95}, {4.20, 1.00}, {10, 1.00}};
  for (size_t i = 1; i < soc_levels.size(); i++) {
    if (voltage < soc_levels[i].first) { // find the first voltage level that is
                                         // higher than the current voltage
      return soc_levels[i - 1]
          .second; // return the soc level of the previous voltage level
    }
  }
  return 1.00;
}

// TODO: Maybe remove,
void INA226_sensor::integrate_usage(float current) {
  auto current_time = this->nh->now();
  auto duration = current_time - this->used_time;

  auto used_mA_sec =
      duration.seconds() * current * 1000.0; // milliAmpere seconds
  auto used_mAh = used_mA_sec / 3600;        // convert to mAh

  this->total_used_mAh += used_mAh;
  this->used_time = current_time;
  if (this->used_pub->get_subscription_count() == 0) {
    // No subscribers, so no need to publish
    return;
  }
  std_msgs::msg::Int32 msg;
  msg.data = (int32_t)this->total_used_mAh;
  this->used_pub->publish(msg);
}

void INA226_sensor::check_soc(float voltage, float current) {
  if (!this->enable_turn_off) {
    if (voltage > 6 && current > 0.1) {
      this->enable_turn_off = true;
      this->in_power_dip = false;
      std::cout << "Enabling turn off" << std::endl;
    }
    return;
  }
  //  at start dip of too low voltage, start timer, when longer than 5s below
  //  trigger voltage, then shut down
  // this makes sure that a short dip (motor start) does not trigger it

  if (voltage > 0.1 && voltage < this->data.min_voltage) {
    if (!this->in_power_dip) {
      this->turn_off_trigger_time = this->nh->now();
      this->in_power_dip = true;
      std::cout << "Triggering turn off in " << this->data.power_low_time << "s"
                << std::endl;
    }
  } else {
    this->in_power_dip = false;
    this->turn_off_trigger_time = rclcpp::Time(0, 0);
  }

  if (this->in_power_dip) {
    auto current_time = this->nh->now();
    auto duration = current_time - this->turn_off_trigger_time;
    std::cout << "Triggering turn off maybe" << duration.seconds() << std::endl;
    std::cout << "you have " << (this->data.power_low_time - duration.seconds())
              << "s left" << std::endl;
    if (duration.seconds() > this->data.power_low_time) {
      std::cout << "Turning off" << std::endl;
      // this->tmx->shutdown();
      this->shutdown_robot();
    }
  }
}

void INA226_sensor::shutdown_robot() {
  if (this->shutdown_triggered) {
    return;
  }
  this->shutdown_triggered = true;
  std::cout << "Shutting down robot" << std::endl;
  if (!this->data.disable_shutdown_relay) {
    std::cout << "Sending shutdown signal to relay" << std::endl;
    this->shutdown_relay_module->send_shutdown_signal(true);
  }
// run shutdown command
#ifndef MIRTE_TESTING_ON_X86
  exec("sudo bash -c \"wall 'Shutting down.'\""); // TODO: check if this works
                                                  // with sudo on a mirte
  exec("sudo shutdown now");
#else
  std::cout << "MIRTE_TESTING_ON_X86 is defined, not actually shutting down "
               "the robot, just printing a message."
            << std::endl;
#endif
}

void INA226_sensor::shutdown_robot_service_callback(
    const std_srvs::srv::SetBool::Request::ConstSharedPtr req,
    std_srvs::srv::SetBool::Response::SharedPtr res) {
  if (req->data) {
    this->shutdown_robot();
  }
  res->success = req->data;
  res->message = "Shutting down" + std::string(req->data ? "true" : "false");
}
#include <mirte_telemetrix_cpp/ina226_parameters.hpp>
#include <ranges>
std::vector<std::shared_ptr<INA226_sensor>>
INA226_sensor::get_ina_modules(NodeData node_data,
                               std::shared_ptr<Parser> parser,
                               std::shared_ptr<tmx_cpp::Sensors> sensors,
                               std::shared_ptr<tmx_cpp::Modules> modules) {
  std::vector<std::shared_ptr<INA226_sensor>> new_modules;
  // auto datas = parse_all_modules<INA226Data>(parser, node_data.board);
  auto found_modules =
      parser->update_params_list_type("modules", "ina_module_names", "ina226");
  parser->fix_param_type_str_modules(
      "modules", found_modules,
      {"pins.sda", "pins.scl", "percentage_led_pin", "shutdown_relay_pin",
       "shutdown_switch_in_pin"});
  parser->fix_param_type_num_modules(
      "modules", found_modules,
      {"min_voltage", "max_voltage", "max_current", "turn_off_time",
       "power_low_time", "shutdown_switch_time_sec", "shutdown_switch_off_time",
       "turn_off_time"});
  auto param_listener =
      std::make_shared<mirte_telemetrix_cpp_ina226::ParamListener>(parser->nh);
  auto params = param_listener->get_params();
  // get modules list from parser
  // loop over items, get one with type==ina226
  // add paramlistener to those with new parameters yaml
  auto datas =
      params.modules.ina_module_names_map |
      std::views::filter([](const auto &pair) {
        const auto &map_power = pair.second;
        return boost::algorithm::to_lower_copy(map_power.type) == "ina226";
      }) |
      std::views::transform([&](const auto &pair) {
        const auto &name = pair.first;
        const auto &map_ina = pair.second;
        std::map<std::string, rclcpp::ParameterValue> parameters;
        parameters["type"] = rclcpp::ParameterValue(map_ina.type);
        parameters["connector"] = rclcpp::ParameterValue(map_ina.connector);
        parameters["pins.sda"] = rclcpp::ParameterValue(map_ina.pins.sda);
        parameters["pins.scl"] = rclcpp::ParameterValue(map_ina.pins.scl);
        parameters["addr"] = rclcpp::ParameterValue(map_ina.addr);
        // parameters["port"] = rclcpp::ParameterValue(map_ina.port);
        parameters["min_voltage"] = rclcpp::ParameterValue(map_ina.min_voltage);
        parameters["power_low_time"] =
            rclcpp::ParameterValue(map_ina.power_low_time);
        parameters["use_percentage_led"] =
            rclcpp::ParameterValue(map_ina.use_percentage_led);
        parameters["percentage_led_pin"] =
            rclcpp::ParameterValue(map_ina.percentage_led_pin);
        parameters["shutdown_relay_pin"] =
            rclcpp::ParameterValue(map_ina.shutdown_relay_pin);
        parameters["shutdown_switch_in_pin"] =
            rclcpp::ParameterValue(map_ina.shutdown_switch_in_pin);
        parameters["shutdown_switch_off_value"] =
            rclcpp::ParameterValue(map_ina.shutdown_switch_off_value);
        parameters["shutdown_switch_off_time"] =
            rclcpp::ParameterValue(map_ina.shutdown_switch_off_time);
        parameters["disable_shutdown_relay"] =
            rclcpp::ParameterValue(map_ina.disable_shutdown_relay);
        parameters["shutdown_relay_off_value"] =
            rclcpp::ParameterValue(map_ina.shutdown_relay_off_value);
        parameters["turn_off_time"] =
            rclcpp::ParameterValue(map_ina.turn_off_time);
        std::set<std::string> unused_keys = get_keys(parameters);
        return INA226Data(parser, node_data.board, name, parameters,
                          unused_keys);
      });
  for (auto data : datas) {
    auto module =
        std::make_shared<INA226_sensor>(node_data, data, sensors, modules);
    new_modules.push_back(module);
  }
  return new_modules;
}

#ifdef WITH_GPIO
void INA226_sensor::battery_led_timer_callback() {
  // show the SOC by blinking the led. Shorter pulse -> lower SOC
  // cycle time of 5s
  auto now = std::chrono::system_clock::now().time_since_epoch() / 1ms;
  auto time_msec = now % 5000;

  auto percentage = calc_soc(voltage_) * 5000;
  if (time_msec > percentage) {
    // turn off the led
    data.percentage_led_pin->write(0);
  } else {
    // turn on the led
    data.percentage_led_pin->write(1);
  }
}
#endif

void INA226_sensor::switch_cb(uint8_t pin, uint8_t signal) {
  std::cout << "Shutdown switch signal: " << (int)signal << std::endl;
  bool sig = (bool)signal;
  if (sig == this->data.shutdown_switch_off_value) {
    if (!this->switch_pin_started) {
      return;
    }
    std::cout << "Shutdown switch triggered, shutting down robot" << std::endl;
    if (this->switch_turn_off_time == rclcpp::Time(0, 0)) {
      this->switch_turn_off_time = this->nh->now();
    }
  } else {
    std::cout << "Shutdown switch turned back on" << std::endl;
    this->switch_pin_started = true; // has been not triggered at least once
    this->switch_turn_off_time = rclcpp::Time(0, 0);
  }
}
