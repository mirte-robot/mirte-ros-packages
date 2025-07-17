#include <algorithm>
#include <cstdint>
#include <functional>

#include <rclcpp/callback_group.hpp>
#include <rclcpp/qos.hpp>

#include <color_util/convert.hpp>

#include <tmx_cpp/tmx.hpp>

#include <mirte_telemetrix_cpp/modules/veml6040_module.hpp>

VEML6040_sensor::VEML6040_sensor(NodeData node_data, VEML6040Data veml_data,
                                 std::shared_ptr<tmx_cpp::Sensors> modules)
    : Mirte_module(node_data, {veml_data.scl, veml_data.sda},
                   (ModuleData)veml_data, rclcpp::CallbackGroupType::Reentrant),
      data(veml_data) {
  using namespace std::placeholders;

  tmx->setI2CPins(veml_data.sda, veml_data.scl, veml_data.port);

  this->veml6040 = std::make_shared<tmx_cpp::VEML6040_module>(
      veml_data.port, veml_data.addr,
      std::bind(&VEML6040_sensor::data_callback, this, _1, _2, _3, _4));

  this->rgbw_pub = nh->create_publisher<mirte_msgs::msg::ColorRGBWStamped>(
      "color/" + this->name + "/rgbw", rclcpp::SystemDefaultsQoS());
  this->rgba_pub = nh->create_publisher<mirte_msgs::msg::ColorRGBAStamped>(
      "color/" + this->name + "/rgba", rclcpp::SystemDefaultsQoS());
  this->hsl_pub = nh->create_publisher<mirte_msgs::msg::ColorHSLStamped>(
      "color/" + this->name + "/hsl", rclcpp::SystemDefaultsQoS());

  this->rgbw_service = nh->create_service<mirte_msgs::srv::GetColorRGBW>(
      "color/" + this->name + "/get_rgbw",
      std::bind(&VEML6040_sensor::get_rgbw_service_callback, this, _1, _2),
      rclcpp::ServicesQoS().get_rmw_qos_profile(), this->callback_group);
  this->rgba_service = nh->create_service<mirte_msgs::srv::GetColorRGBA>(
      "color/" + this->name + "/get_rgba",
      std::bind(&VEML6040_sensor::get_rgba_service_callback, this, _1, _2),
      rclcpp::ServicesQoS().get_rmw_qos_profile(), this->callback_group);
  this->hsl_service = nh->create_service<mirte_msgs::srv::GetColorHSL>(
      "color/" + this->name + "/get_hsl",
      std::bind(&VEML6040_sensor::get_hsl_service_callback, this, _1, _2),
      rclcpp::ServicesQoS().get_rmw_qos_profile(), this->callback_group);

  modules->add_sens(this->veml6040);
}

void VEML6040_sensor::update() {
  using mirte_msgs::msg::ColorHSLStamped;
  using mirte_msgs::msg::ColorRGBAStamped;
  using mirte_msgs::msg::ColorRGBWStamped;
  if (this->rgbw_pub->get_subscription_count() == 0 &&
      this->rgba_pub->get_subscription_count() == 0 &&
      this->hsl_pub->get_subscription_count() == 0) {
    // No subscribers, so no need to publish
    return;
  }
  if (msg_mutex.try_lock_shared()) {
    const std::shared_lock lock{msg_mutex, std::adopt_lock};
    auto header = get_header();

    rgbw_pub->publish(
        mirte_msgs::build<ColorRGBWStamped>().header(header).color(last_rgbw));
    rgba_pub->publish(
        mirte_msgs::build<ColorRGBAStamped>().header(header).color(last_rgba));
    hsl_pub->publish(
        mirte_msgs::build<ColorHSLStamped>().header(header).color(last_hsl));
  }
}

void VEML6040_sensor::data_callback(uint16_t red, uint16_t green, uint16_t blue,
                                    uint16_t white) {
  using mirte_msgs::msg::ColorHSLStamped;
  using mirte_msgs::msg::ColorRGBAStamped;
  using mirte_msgs::msg::ColorRGBWStamped;

  const std::unique_lock<std::shared_mutex> lock(msg_mutex);
  // this->device_timer->reset();
  // To get to HSI/HSV/HSL we need to set a max to the intensity (setting). This
  // could be 2^16, but to get more resolution, you could also cap this to a
  // lower value.

  // Getting the normalized values from fig 5 (normalized spectrum response)
  // normalized red @ 619 -> 0.77
  // normalized green @ 518 -> 0.57
  // normalized blue @ 467 - > 0.91

  // The irradiance response (counts / (uW/cm2)) from the characteristics table
  // red @ 619 = 96
  // green @ 518 = 74
  // blue @ 467 = 56

  // So, the normalized irrandiance repsonse is (see
  // https://electronics.stackexchange.com/questions/372345/relative-responisivity-of-the-veml6040)
  // red: 96 / 0.77 = 125
  // green: 74 / 0.57 = 130
  // blue: 56 / 0.91 = 62

  // With that we can get the (uW/cm2) per channel. We can also calsulate the
  // max uW/cm2 for green: 2^16 / 130 = 504 uW/cm2. And even though red and
  // espcially blue can meadure more, they need to be capped to this value so
  // they will be even. And all can be normalized to [0,1]
  float max_uW_cm2 =
      250.0; // calculated (504), emperical (250) (TODO: make setting)
  float r = std::min(((float)red) / 125.f, max_uW_cm2) / max_uW_cm2;
  float g = std::min(((float)green) / 130.f, max_uW_cm2) / max_uW_cm2;
  float b = std::min(((float)blue) / 62.f, max_uW_cm2) / max_uW_cm2;
  float w = (float)white / 65535.0;

  auto header = get_header();
  last_rgbw = mirte_msgs::msg::ColorRGBW();

  last_rgbw.r = r;
  last_rgbw.g = g;
  last_rgbw.b = b;
  last_rgbw.w = w;

  if (this->rgbw_pub->get_subscription_count() > 0) {
    rgbw_pub->publish(
        mirte_msgs::build<ColorRGBWStamped>().header(header).color(last_rgbw));
  }

  last_rgba.r = r;
  last_rgba.g = g;
  last_rgba.b = b;
  last_rgba.a = 1;
  if (this->rgba_pub->get_subscription_count() > 0) {
    rgba_pub->publish(
        mirte_msgs::build<ColorRGBAStamped>().header(header).color(last_rgba));
  }

  auto color_hsva = color_util::changeColorspace(color_util::ColorRGBA(
      last_rgba.r, last_rgba.g, last_rgba.b, last_rgba.a));
  // Convert from hsv to hsl
  // Source:
  // https://en.wikipedia.org/w/index.php?title=HSL_and_HSV&oldid=1236077814#HSV_to_HSL
  last_hsl.h = color_hsva.h * 360;

  auto v = color_hsva.v;
  auto l = v * (1.0 - (color_hsva.s / 2.0));

  last_hsl.l = l;
  last_hsl.s = (l >= 1.0 or l <= 0.0) ? 0.0 : ((v - l) / std::min(l, 1 - l));
  if (this->hsl_pub->get_subscription_count() > 0) {
    hsl_pub->publish(
        mirte_msgs::build<ColorHSLStamped>().header(header).color(last_hsl));
  }
}

void VEML6040_sensor::get_rgbw_service_callback(
    const mirte_msgs::srv::GetColorRGBW::Request::ConstSharedPtr req,
    mirte_msgs::srv::GetColorRGBW::Response::SharedPtr res) {
  const std::shared_lock<std::shared_mutex> lock(msg_mutex);
  res->color = last_rgbw;
}

void VEML6040_sensor::get_rgba_service_callback(
    const mirte_msgs::srv::GetColorRGBA::Request::ConstSharedPtr req,
    mirte_msgs::srv::GetColorRGBA::Response::SharedPtr res) {
  const std::shared_lock<std::shared_mutex> lock(msg_mutex);
  res->color = last_rgba;
}

void VEML6040_sensor::get_hsl_service_callback(
    const mirte_msgs::srv::GetColorHSL::Request::ConstSharedPtr req,
    mirte_msgs::srv::GetColorHSL::Response::SharedPtr res) {
  const std::shared_lock<std::shared_mutex> lock(msg_mutex);
  res->color = last_hsl;
}

std::vector<std::shared_ptr<VEML6040_sensor>>
VEML6040_sensor::get_veml6040_modules(
    NodeData node_data, std::shared_ptr<Parser> parser,
    std::shared_ptr<tmx_cpp::Sensors> sensors) {
  std::vector<std::shared_ptr<VEML6040_sensor>> new_modules;
  auto datas = parse_all<VEML6040Data>(parser, node_data.board);
  for (auto data : datas) {
    auto module = std::make_shared<VEML6040_sensor>(node_data, data, sensors);
    new_modules.push_back(module);
  }
  return new_modules;
}
