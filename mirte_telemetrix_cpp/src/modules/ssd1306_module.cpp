#include <algorithm>
#include <chrono>
#include <functional>

#include <boost/algorithm/string/replace.hpp>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>

#include <rclcpp/callback_group.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rcpputils/asserts.hpp>

// Pre & Post IRON compatability.
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include <mirte_telemetrix_cpp/modules/ssd1306_module.hpp>
#include <mirte_telemetrix_cpp/util.hpp>

using namespace std::placeholders; // for _1, _2, _3...
using namespace std::chrono_literals;

std::vector<std::shared_ptr<SSD1306_module>>
SSD1306_module::get_ssd1306_modules(NodeData node_data,
                                    std::shared_ptr<Parser> parser,
                                    std::shared_ptr<tmx_cpp::Modules> modules) {
  std::vector<std::shared_ptr<SSD1306_module>> new_modules;
  auto datas = parse_all<SSD1306Data>(parser, node_data.board);
  for (auto data : datas) {
    auto module = std::make_shared<SSD1306_module>(node_data, data, modules);
    new_modules.push_back(module);
  }
  return new_modules;
}

// Use a mutally Exclusive callback group to ensure no OLED commands get mixed.
SSD1306_module::SSD1306_module(NodeData node_data, SSD1306Data oled_data,
                               std::shared_ptr<tmx_cpp::Modules> modules)
    : Mirte_module(node_data, {oled_data.scl, oled_data.sda},
                   (ModuleData)oled_data,
                   rclcpp::CallbackGroupType::MutuallyExclusive),
      data(oled_data) {
  this->logger =
      this->logger.get_child(data.get_device_class()).get_child(data.name);

  tmx->setI2CPins(data.sda, data.scl, data.port);

  this->ssd1306 = std::make_shared<tmx_cpp::SSD1306_module>(
      data.port, data.addr, data.width, data.height);

  if (data.legacy) {
    this->set_oled_service_legacy =
        nh->create_service<mirte_msgs::srv::SetOLEDImageLegacy>(
            "oled/" + data.name + "/set_image_legacy",
            std::bind(&SSD1306_module::set_oled_callback_legacy, this, _1, _2),
            rmw_qos_profile_services_default, this->callback_group);
  }

  this->set_oled_text_service =
      nh->create_service<mirte_msgs::srv::SetOLEDText>(
          "oled/" + data.name + "/set_text",
          std::bind(&SSD1306_module::set_oled_text_callback, this, _1, _2),
          rmw_qos_profile_services_default, this->callback_group);

  this->set_oled_image_service =
      nh->create_service<mirte_msgs::srv::SetOLEDImage>(
          "oled/" + data.name + "/set_image",
          std::bind(&SSD1306_module::set_oled_image_callback, this, _1, _2),
          rmw_qos_profile_services_default, this->callback_group);

  this->set_oled_file_service =
      nh->create_service<mirte_msgs::srv::SetOLEDFile>(
          "oled/" + data.name + "/set_file",
          std::bind(&SSD1306_module::set_oled_file_callback, this, _1, _2),
          rmw_qos_profile_services_default, this->callback_group);

  this->device_timer->reset();

  modules->add_mod(this->ssd1306);
  // Write an initial text to the screen, and instantly kill the timer if it has
  // failed.
  /* NOTE: This needs to use the raw send_text, because otherwise the
   * default_screen_timer will be canceled. */
  if (!this->ssd1306->send_text("Booting...", 500ms)) {
    RCLCPP_ERROR(this->logger,
                 "Writing to OLED module '%s' failed, shutting down default "
                 "screen timer.",
                 this->data.name.c_str());
    this->device_timer->cancel();
  }
}

bool SSD1306_module::prewrite(bool is_default) {
  if (!is_default) {
    device_timer->cancel();
  }

  if (!enabled) {
    RCLCPP_ERROR(logger, "Writing to OLED Module %s failed", data.name.c_str());
    return false;
  }
  return true;
}

bool SSD1306_module::set_text(std::string text) {
  if (!prewrite()) {
    return false;
  }

  auto escaped_text = boost::algorithm::replace_all_copy(text, "\\n", "\n");
  if (escaped_text == last_text) {
    return true;
  }
  last_text = escaped_text;
  auto succes = ssd1306->send_text(escaped_text);
  if (not succes) {
    enabled = false;
  }

  return succes;
}

bool SSD1306_module::set_image(uint8_t width, uint8_t height,
                               uint8_t img_buffer[]) {
  if (!prewrite()) {
    return false;
  }

  last_text.reset();

  if (width != data.width || height != data.height) {
    RCLCPP_ERROR(logger,
                 "Supplied image of wrong size. Expected %ux%u, got %ux%u",
                 data.width, data.height, width, height);
    return false;
  }

  auto succes = ssd1306->send_image(width, height, img_buffer);
  if (not succes) {
    enabled = false;
  }

  return succes;
}

bool SSD1306_module::set_image_from_path(std::string path) {
  fs::path actual_path;
  if ((path.size() > 6 && path.substr(0, 6) == "pkg://") ||
      (path.size() > 11 && path.substr(0, 10) == "package://")) {
    auto prefix_length = (path.substr(0, 6) == "pkg://") ? 6 : 10;
    auto data_path = path.substr(prefix_length);

    auto split_idx = data_path.find('/');

    auto package = data_path.substr(0, split_idx);
    auto remainder = data_path.substr(split_idx + 1);

    actual_path =
        fs::path(ament_index_cpp::get_package_share_directory(package)) /
        remainder;
  } else if (path[0] != '/') {
    // Check the local path from the DEFAULT_PACKAGE or Folder.
    return set_image_from_path(data.default_image_path + "/" + path);
  } else {
    actual_path = fs::path(path);
  }

  return set_image_from_path(actual_path);
}

bool SSD1306_module::set_image_from_path(fs::path path) {
  /* Check if the specified path exists */
  if (!fs::exists(path)) {
    RCLCPP_ERROR(logger, "The specified image path does not exist");
    return false;
  }

  if (path.has_filename() && path.has_extension()) {
    // Single Image
    cv::Mat image = cv::imread(path.string(), cv::IMREAD_GRAYSCALE);
    return set_image(image.cols, image.rows, image.data);
  } else {
    // Animation
    std::vector<cv::Mat> images;

    std::set<fs::path> paths;

    for (auto img_file : fs::directory_iterator(
             path, fs::directory_options::follow_directory_symlink |
                       fs::directory_options::skip_permission_denied)) {
      paths.insert(img_file);
    }

    RCLCPP_DEBUG(logger, "Loading animation frames from %s", path.c_str());

    for (auto img_file : paths) {
      RCLCPP_DEBUG(logger, "Loading frame %ld: %s", images.size(),
                   img_file.c_str());
      images.push_back(cv::imread(img_file.string(), cv::IMREAD_GRAYSCALE));
    }

    bool result = true;
    for (auto image : images) {
      result &= set_image(image.cols, image.rows, image.data);

      if (!result) {
        RCLCPP_ERROR(logger,
                     "Some Error has occured during the animation playback.");
        return result;
      }
    }

    return result;
  }
}

void SSD1306_module::set_oled_callback_legacy(
    const mirte_msgs::srv::SetOLEDImageLegacy::Request::ConstSharedPtr req,
    mirte_msgs::srv::SetOLEDImageLegacy::Response::SharedPtr res) {
  if (req->type == "text") {
    res->status = set_text(req->value);
  } else if (req->type == "image") {
    // If not an absolute path or a package local path use the old behavior
    auto path = (req->value[0] != '/' && req->value.substr(0, 6) != "pkg://" &&
                 req->value.substr(0, 10) != "package://" &&
                 req->value.rfind('.') == -1)
                    ? data.default_image_path + "/images/" + req->value + ".png"
                    : req->value;
    res->status = set_image_from_path(path);
  } else if (req->type == "animation") {
    auto path =
        (req->value[0] != '/' && req->value.substr(0, 6) != "pkg://" &&
         req->value.substr(0, 10) != "package://" && req->value.back() != '/')
            ? data.default_image_path + "/animations/" + req->value + "/"
            : req->value;
    res->status = set_image_from_path(path);
  } else {
    RCLCPP_ERROR(
        logger,
        "'%s' is not a valid type. (Available: 'text', 'image', 'animation')",
        req->type.c_str());
    res->status = false;
  }
}

void SSD1306_module::set_oled_text_callback(
    const mirte_msgs::srv::SetOLEDText::Request::ConstSharedPtr req,
    mirte_msgs::srv::SetOLEDText::Response::SharedPtr res) {
  res->status = set_text(req->text);
}

void SSD1306_module::set_oled_image_callback(
    const mirte_msgs::srv::SetOLEDImage::Request::ConstSharedPtr req,
    mirte_msgs::srv::SetOLEDImage::Response::SharedPtr res) {
  auto bridged_img = cv_bridge::toCvShare(req->image, req, "mono8");
  res->status = set_image(bridged_img->image.cols, bridged_img->image.rows,
                          bridged_img->image.data);
}

void SSD1306_module::set_oled_file_callback(
    const mirte_msgs::srv::SetOLEDFile::Request::ConstSharedPtr req,
    mirte_msgs::srv::SetOLEDFile::Response::SharedPtr res) {
  res->status = set_image_from_path(req->path);
}

void SSD1306_module::device_timer_callback() {
  if (!prewrite(true)) {
    return;
  }

  bool succes;
  if ((fs::status(data.default_screen_script).permissions() &
       (fs::perms::owner_exec | fs::perms::group_exec |
        fs::perms::others_exec)) != fs::perms::none) {
    auto text = exec(data.default_screen_script);

    auto escaped_text = boost::algorithm::replace_all_copy(text, "\\n", "\n");
    if (escaped_text == last_text) {
      return;
    }
    last_text = escaped_text;
    succes = ssd1306->send_text(escaped_text);
  } else {
    succes = set_image_from_path(data.default_screen_script);
  }

  if (not succes) {
    enabled = false;
    device_timer->cancel();
    RCLCPP_ERROR(
        logger,
        "Default screen update failed. Disabling screen and update timer.");
  }
}
