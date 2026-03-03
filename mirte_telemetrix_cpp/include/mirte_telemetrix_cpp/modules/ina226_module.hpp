#pragma once
#include <atomic>

#include <tmx_cpp/sensors/INA226.hpp>

#include <mirte_telemetrix_cpp/modules/base_module.hpp>

#include <mirte_telemetrix_cpp/parsers/modules/ina226_data.hpp>

#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tmx_cpp/modules/Shutdown_relay.hpp>
class INA226_sensor : public Mirte_module {
public:
  INA226_sensor(NodeData node_data, INA226Data ina_data,
                std::shared_ptr<tmx_cpp::Sensors> sensors, std::shared_ptr<tmx_cpp::Modules> modules);

  INA226Data data;
  std::shared_ptr<tmx_cpp::INA226_module> ina226;

  virtual void update() override;

  void data_callback(float voltage, float current);

  float calc_soc(float voltage);
  void write_soc(float soc);
  void integrate_usage(float current);
  void check_soc(float voltage, float current);

  void shutdown_robot();

  static std::vector<std::shared_ptr<INA226_sensor>>
  get_ina_modules(NodeData node_data, std::shared_ptr<Parser> parser,
                  std::shared_ptr<tmx_cpp::Sensors> sensors, std::shared_ptr<tmx_cpp::Modules> modules);

private:
  std::atomic<float> total_used_mAh = 0;
  rclcpp::Time used_time = rclcpp::Time(0, 0);

  bool enable_turn_off = false;
  bool shutdown_triggered = false;
  rclcpp::Time turn_off_trigger_time = rclcpp::Time(0, 0);
  std::atomic<bool> in_power_dip = false;
  rclcpp::Time turn_off_time = rclcpp::Time(0, 0);
  std::atomic<float> voltage_ = 0;
  std::atomic<float> current_ = 0;

  // Publisher: power/NAME/used
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr used_pub;
  // Publisher: power/NAME
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub;
  // Service: power/NAME/shutdown
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr shutdown_service;

  void shutdown_robot_service_callback(
      const std_srvs::srv::SetBool::Request::ConstSharedPtr req,
      std_srvs::srv::SetBool::Response::SharedPtr res);
std::shared_ptr<tmx_cpp::Shutdown_relay_module> shutdown_relay_module;
#ifdef WITH_GPIO
  rclcpp::TimerBase::SharedPtr battery_led_timer;
  void battery_led_timer_callback();
#endif


    // switch part:
    void switch_cb(bool signal);
    rclcpp::Time switch_turn_off_time = rclcpp::Time(0, 0);
    bool switch_pin_started = false; // to prevent triggering on startup if the switch is already in the off position
};