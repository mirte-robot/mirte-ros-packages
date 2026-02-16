#include <boost/format.hpp>

#include <rcpputils/asserts.hpp>

#include <mirte_telemetrix_cpp/parsers/device_data.hpp>
#include <mirte_telemetrix_cpp/parsers/parsers.hpp>

DeviceData::DeviceData(std::shared_ptr<Parser> parser,
                       std::shared_ptr<Mirte_Board>, std::string name,
                       std::string sensor_type,
                       std::map<std::string, rclcpp::ParameterValue> parameters,
                       std::set<std::string> &unused_keys,
                       std::optional<DeviceDuration> duration)
    : name(name) {
  if (unused_keys.erase("name")) {
    rcpputils::require_true(
        this->name.compare(get_string(parameters["name"])) == 0,
        (boost::format("The optional name parameter does not match it's key. [ "
                       "%1% != %2% ]") %
         name % get_string(parameters["name"]))
            .str());
  }

  std::string frame_prefix =
      (parser->params.count("frame_prefix") &&
       !get_string(parser->params["frame_prefix"]).empty())
          ? get_string(parser->params["frame_prefix"])
          : "";

  if (unused_keys.erase("frame_id")) {
    this->frame_id = frame_prefix + get_string(parameters["frame_id"]);
  } else {
    this->frame_id = frame_prefix + sensor_type + "_" + name;
  }

  this->duration = duration.value_or(
      std::chrono::duration<double>(1.0 / parser->get_frequency()));
}

DeviceData::DeviceData(std::string name, std::string frame_id,
                       DeviceDuration duration)
    : name(name), frame_id(frame_id), duration(duration) {}

bool DeviceData::check() { return name != ""; }