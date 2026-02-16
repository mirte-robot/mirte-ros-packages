#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>

#include <rcpputils/asserts.hpp>

#include <mirte_telemetrix_cpp/parsers/modules/module_data.hpp>

ModuleData::ModuleData(std::shared_ptr<Parser> parser,
                       std::shared_ptr<Mirte_Board> board, std::string name,
                       std::map<std::string, rclcpp::ParameterValue> parameters,
                       std::set<std::string> &unused_keys,
                       std::optional<DeviceDuration> duration)
    : DeviceData(parser, board, name, this->get_device_class(), parameters,
                 unused_keys, duration) {
  rcpputils::require_true(
      unused_keys.erase("type"),
      (boost::format(
           "A module configuration requires a 'type' to be specified, but no "
           "'type' was for module '%1%'") %
       name)
          .str());
  this->type = get_string(parameters["type"]);
}

bool ModuleData::check(std::string module_type) {
  return boost::to_lower_copy(type) == boost::to_lower_copy(module_type) &&
         DeviceData::check();
}

bool ModuleData::check() { return false; }
