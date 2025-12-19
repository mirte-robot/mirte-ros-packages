#include "mirte_telemetrix_cpp/parsers/parsers.hpp"

#include <set>
#include <string>
#include <vector>
#include <regex>
#include <functional>
#include <ranges>
#include <iostream>

#include <boost/algorithm/string.hpp>
std::map<std::string, rclcpp::ParameterValue>
get_params_name(std::shared_ptr<rclcpp::Node> nh, std::string name) {
  auto node_parameters_iface = nh->get_node_parameters_interface();
  auto parameter_overrides = node_parameters_iface->get_parameter_overrides();
  decltype(parameter_overrides) out_params;
  for (auto &servo_it : parameter_overrides) {
    //     std::cout << servo_it.first << std::endl;
    rclcpp::ParameterValue servo_config = servo_it.second;
    if (starts_with(servo_it.first, name)) {
      out_params[servo_it.first] = servo_config;
    }
  }
  return out_params;
}

std::set<std::string> get_params_key_names(std::shared_ptr<rclcpp::Node> nh,
                                           std::string name) {
  auto node_parameters_iface = nh->get_node_parameters_interface();
  auto parameter_overrides = node_parameters_iface->get_parameter_overrides();
  std::set<std::string> out_params;
  for (auto &servo_it : parameter_overrides) {
    //     std::cout << servo_it.first << std::endl;
    rclcpp::ParameterValue servo_config = servo_it.second;

    if (starts_with(servo_it.first, name)) {
      std::cout << servo_it.first << std::endl;
      std::string key = servo_it.first.substr(name.length() + 1);
      std::cout << "1:" << key << std::endl;
      auto next_dot = key.find(".");
      key = key.substr(0, next_dot);
      std::cout << "2:" << key << "," << next_dot << std::endl;

      out_params.insert(key);
    }
  }
  return out_params;
}

Parser::Parser(std::shared_ptr<rclcpp::Node> nh)
    : nh(nh), logger(nh->get_logger().get_child("parser")) {
  auto node_parameters_iface = nh->get_node_parameters_interface();
  auto parameter_overrides = node_parameters_iface->get_parameter_overrides();
  this->params = parameter_overrides;

  for (auto const &[key, val] : this->params) {
    std::cout << key                           // string (key)
              << ':' << rclcpp::to_string(val) // string's value
              << std::endl;
  }

  auto list_items = {"encoder", "color", "distance", "intensity",
                     "motor",   "servo", "oled",     "keypad"};
  for(auto item : list_items) {
    this->update_params_list(fmt::format("{}", item), fmt::format("{}s", item), [](std::string x){return true;});
  }

   auto param_listener =
      std::make_shared<mirte_telemetrix_cpp::ParamListener>(nh);
  auto params = param_listener->get_params();
  this->params_object = params;
}

/**
 * get the parameters starting with name
 * removes the name from the key, including the dot
 *
 */
std::map<std::string, rclcpp::ParameterValue>
Parser::get_params_name(std::string name) {
  std::map<std::string, rclcpp::ParameterValue> out_params;
  for (auto &servo_it : this->params) {
    if (starts_with(servo_it.first, name)) {
      std::string key = servo_it.first.substr(name.length() + 1);
      out_params[key] = servo_it.second;
    }
  }
  return out_params;
}

std::set<std::string> Parser::get_params_keys(std::string name) {
  std::set<std::string> out_params;
  name += "."; // add dot to ensure we only take objects with exactly the same
               // name, not oledTest.. when oled.. is requested
  for (auto &servo_it : this->params) {
    if (starts_with(servo_it.first, name)) {
      std::string key = servo_it.first.substr(name.length());
      auto next_dot = key.find(".");
      key = key.substr(0, next_dot);
      out_params.insert(key);
    }
  }
  return out_params;
}
std::string Parser::build_param_name(std::string name, std::string key) {
  return name + "." + key;
}
std::string
Parser::build_param_name(std::initializer_list<const std::string> keys) {
  std::string full_name = "";
  for (const auto &key : keys) {
    full_name += "." + key;
  }
  if (!full_name.empty() && full_name.front() == '.') {
    full_name.erase(0, 1); // Remove leading dot
  }
  return full_name;
}

int Parser::get_frequency() {
  auto keys = get_params_keys("device.mirte");
  auto values = get_params_name("device.mirte");
  return this->params_object.device.mirte.max_frequency;
  if (keys.erase("max_frequency")) {
    return values["max_frequency"].get<int>();
  } else {
    return 50;
  }
}

std::string
Parser::get_last(std::string name) { // convert modules.servobus to servobus
  auto last_dot = name.find_last_of(".");
  if (last_dot == std::string::npos) {
    return name;
  }
  auto last = name.substr(last_dot + 1);
  return last;
}

std::map<std::string, rclcpp::ParameterValue>
insert_default_param(std::map<std::string, rclcpp::ParameterValue> parameters,
                     std::string key, rclcpp::ParameterValue value) {
  parameters.emplace(key, value);
  return parameters;
}

std::set<std::string> &insert_default_param(std::set<std::string> &unused_keys,
                                            std::string key) {
  unused_keys.emplace(key);
  return unused_keys;
}



// bool update_params_list(std::string const& prefix_, std::string const & list_name) {
//       // __map_xxxxx does not automatically update the list of names, so we need to do it here
//    auto x = this->nh->get_node_parameters_interface()->list_parameters({prefix_}, 10).names | std::views::transform([prefix_](const std::string& param_name){
//         // std::cout << "Checking parameter: " << param_name << std::endl;
//              std::regex word_regex("^" + prefix_ + "\\.");
//                 if( std::regex_search(param_name, word_regex)) {
//                     auto replaced = std::regex_replace(param_name, word_regex, "");
//                     // std::cout << "  replaced string: " << replaced << std::endl;
//                     auto next_key = replaced.substr(0, replaced.find('.'));
//                     // std::cout << "  extracted key: " << next_key << std::endl;
//                     return next_key;
//                 } else {
//                     return std::string{};
//                 }

//         //   return param.get_name().starts_with(prefix_ + "modules.");
//       }) | std::views::filter([](const std::string& s){ return !s.empty(); }) ;
//       std::set<std::string> vec_x;
//       for (const auto& v : x) {
//         // std::cout << "inserting found dynamic parameter module name: " << v << std::endl;
//         vec_x.insert(v); 
//       }
//       auto module_names = std::vector<std::string>(vec_x.begin(), vec_x.end());
//       // std::cout << "Final module "<< list_name << " list: ";
//       // for(const auto& name : module_names) {
//       //     std::cout << name << " ";
//       // }
//       // std::cout << std::endl;
//       this->nh->get_node_parameters_interface()->set_parameters({rclcpp::Parameter(list_name, module_names)});
//       return true;

// }
std::vector<std::string> Parser::update_params_list(std::string const& prefix_, std::string const & list_name,  std::function<bool(const std::string& name)> filter_func) {
      // __map_xxxxx does not automatically update the list of names, so we need to do it here
      std::cout << "Updating parameter list for " << list_name << " with prefix " << prefix_ << std::endl;
   auto x = this->nh->get_node_parameters_interface()->list_parameters({prefix_}, 10).names | std::views::transform([prefix_](const std::string& param_name){
        // std::cout << "Checking parameter: " << param_name << std::endl;
             std::regex word_regex("^" + prefix_ + "\\.");
                if( std::regex_search(param_name, word_regex)) {
                    auto replaced = std::regex_replace(param_name, word_regex, "");
                    // std::cout << "  replaced string: " << replaced << std::endl;
                    auto next_key = replaced.substr(0, replaced.find('.'));
                    // std::cout << "  extracted key: " << next_key << std::endl;
                    return std::pair(param_name, next_key);
                } else {
                    return std::pair<std::string, std::string>{};
                }

        //   return param.get_name().starts_with(prefix_ + "modules.");
      }) | std::views::filter([](const std::pair<std::string, std::string>& s){ return !s.second.empty(); }) | std::views::filter([filter_func](const std::pair<std::string, std::string>& s){
          return filter_func(s.first);
      })
      | std::views::transform([](const std::pair<std::string, std::string>& s){
          return s.second;
      });
      std::set<std::string> vec_x;
      for (const auto& v : x) {
        std::cout << "inserting found dynamic parameter module name: " << v  << " for " << list_name << std::endl;
        vec_x.insert(v); 
      }
      auto module_names = std::vector<std::string>(vec_x.begin(), vec_x.end());
      // std::cout << "Final module "<< list_name << " list: ";
      // for(const auto& name : module_names) {
      //     std::cout << name << " ";
      // }
      // std::cout << std::endl;
      this->nh->get_node_parameters_interface()->set_parameters({rclcpp::Parameter(list_name, module_names)});
      return module_names;

}

std::vector<std::string> Parser::update_params_list_type(std::string const& prefix_, std::string const & list_name, std::string const& type_filter) {
      // __map_xxxxx does not automatically update the list of names, so we need to do it here

 return this->update_params_list(prefix_, list_name, [&](const std::string& name){
            // check if ends with type, then check if param type == "INA226"
            std::cout << "Filtering parameter: " << name << std::endl;
            if(name.size() < 5) {
                std::cout << "  too short" << std::endl;
                return false;
            }
            if(name.substr(name.size() - 4) != "type") {
                std::cout << "  does not end with type" << std::endl;
                return false;
            }
            rclcpp::Parameter param;
            param = this->nh->get_node_parameters_interface()->get_parameter(name);
            if(boost::algorithm::to_lower_copy(param.value_to_string()) != type_filter) {
                // std::cout << "  type is not INA226" << std::endl;
                return false;
            }
            // std::cout << "  accepted INA226 parameter" << std::endl;
            return true;
        });
}