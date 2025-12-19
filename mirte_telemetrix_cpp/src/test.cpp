#include "rclcpp/rclcpp.hpp"
#include <mirte_telemetrix_cpp/telemetrix_parameters.hpp> // auto-generated parameter file from telemetrix_parameters.yaml
#include <mirte_telemetrix_cpp/ina226_parameters.hpp> // auto-generated parameter file from ina226_parameters.yaml
#include <ranges>
#include <regex>
#include <set>
#include <algorithm>

//   START EDITTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT
//  std::cout << "Refreshing dynamic parameters..." << std::endl;
//  std::cout << "prefix_: " << prefix_ << std::endl;
//     //   search for dynamic parameters, e.g., parameters that start with prefix_ + "modules.", then extract the module name
//       auto x = parameters_interface_->list_parameters({}, 10).names | std::views::transform([this](const std::string& param_name){
//         std::cout << "Checking parameter: " << param_name << std::endl;
//              std::regex word_regex(fmt::format("^{}{}\\.", prefix_, "a"));
//                 if( std::regex_search(param_name, word_regex)) {
//                     auto replaced = std::regex_replace(param_name, word_regex, "");
//                     auto next_key = replaced.substr(0, replaced.find('.'));
//                     return next_key;
//                 } else {
//                     return std::string{};
//                 }

//         //   return param.get_name().starts_with(prefix_ + "modules.");
//       }) | std::views::filter([](const std::string& s){ return !s.empty(); }) ;
//       for (const auto& v : x) {
//         std::cout << "Found dynamic parameter module name: " << v << std::endl;
//           if (std::find(updated_params.module_names.begin(), updated_params.module_names.end(), v) == updated_params.module_names.end()) {
//             std::cout << "Adding dynamic parameter module name: " << v << std::endl;
//               updated_params.module_names.push_back(v);
//               std::cout << "size now: " << updated_params.module_names.size() << std::endl;
//               parameters_interface_->set_parameters({rclcpp::Parameter(fmt::format("{}{}", prefix_, "module_names"), updated_params.module_names)});
//           }
//       }


    //   END EDITTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT
bool update_params_list(std::string const& prefix_, std::string const & list_name,
    const std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface>& parameters_interface_) {
      // __map_xxxxx does not automatically update the list of names, so we need to do it here
  //  auto x = parameters_interface_->list_parameters({prefix_}, 10).names | std::views::transform([prefix_](const std::string& param_name){
  //       // std::cout << "Checking parameter: " << param_name << std::endl;
  //            std::regex word_regex("^" + prefix_ + "\\.");
  //               if( std::regex_search(param_name, word_regex)) {
  //                   auto replaced = std::regex_replace(param_name, word_regex, "");
  //                   // std::cout << "  replaced string: " << replaced << std::endl;
  //                   auto next_key = replaced.substr(0, replaced.find('.'));
  //                   // std::cout << "  extracted key: " << next_key << std::endl;
  //                   return next_key;
  //               } else {
  //                   return std::string{};
  //               }

  //       //   return param.get_name().starts_with(prefix_ + "modules.");
  //     }) | std::views::filter([](const std::string& s){ return !s.empty(); }) ;
  //     std::set<std::string> vec_x;
  //     for (const auto& v : x) {
  //       std::cout << "inserting found dynamic parameter module name: " << v << std::endl;
  //       vec_x.insert(v); 
  //     }
  //     auto module_names = std::vector<std::string>(vec_x.begin(), vec_x.end());
  //     std::cout << "Final module "<< list_name << " list: ";
  //     for(const auto& name : module_names) {
  //         std::cout << name << " ";
  //     }
  //     std::cout << std::endl;
  //     parameters_interface_->set_parameters({rclcpp::Parameter(list_name, module_names)});
      return true;

}
bool update_params_list(std::string const& prefix_, std::string const & list_name,
    const std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface>& parameters_interface_, std::function<bool(const std::string& name)> filter_func) {
      // __map_xxxxx does not automatically update the list of names, so we need to do it here
  //  auto x = parameters_interface_->list_parameters({prefix_}, 10).names | std::views::transform([prefix_](const std::string& param_name){
  //       // std::cout << "Checking parameter: " << param_name << std::endl;
  //            std::regex word_regex("^" + prefix_ + "\\.");
  //               if( std::regex_search(param_name, word_regex)) {
  //                   auto replaced = std::regex_replace(param_name, word_regex, "");
  //                   // std::cout << "  replaced string: " << replaced << std::endl;
  //                   auto next_key = replaced.substr(0, replaced.find('.'));
  //                   // std::cout << "  extracted key: " << next_key << std::endl;
  //                   return std::pair(param_name, next_key);
  //               } else {
  //                   return std::pair<std::string, std::string>{};
  //               }

  //       //   return param.get_name().starts_with(prefix_ + "modules.");
  //     }) | std::views::filter([](const std::pair<std::string, std::string>& s){ return !s.second.empty(); }) | std::views::filter([filter_func](const std::pair<std::string, std::string>& s){
  //         return filter_func(s.first);
  //     })
  //     | std::views::transform([](const std::pair<std::string, std::string>& s){
  //         return s.second;
  //     });
  //     std::set<std::string> vec_x;
  //     for (const auto& v : x) {
  //       std::cout << "inserting found dynamic parameter module name: " << v << std::endl;
  //       vec_x.insert(v); 
  //     }
  //     auto module_names = std::vector<std::string>(vec_x.begin(), vec_x.end());
  //     std::cout << "Final module "<< list_name << " list: ";
  //     for(const auto& name : module_names) {
  //         std::cout << name << " ";
  //     }
  //     std::cout << std::endl;
  //     parameters_interface_->set_parameters({rclcpp::Parameter(list_name, module_names)});
      return true;

}

void update_params_list_type(std::string const& prefix_, std::string const & list_name,
    const std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface>& parameters_interface_, std::string const& type_filter) {
      // __map_xxxxx does not automatically update the list of names, so we need to do it here

  update_params_list(prefix_, list_name, parameters_interface_, [&](const std::string& name){
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
            param = parameters_interface_->get_parameter(name);
            if(param.value_to_string() != type_filter) {
                // std::cout << "  type is not INA226" << std::endl;
                return false;
            }
            // std::cout << "  accepted INA226 parameter" << std::endl;
            return true;
        });
}


int main(int argc, char **argv) {
  // Your code here
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
      "mirte_telemetrix_test_node",
      rclcpp::NodeOptions()
          .allow_undeclared_parameters(true)
          .automatically_declare_parameters_from_overrides(true)
        );
  // auto parser = std::make_shared<mirte_telemetrix_cpp::Parser>(node);
  try {
    auto param_listener =
        std::make_shared<mirte_telemetrix_cpp::ParamListener>(node);
    auto params = param_listener->get_params();
    
    std::cout << params.device.mirte.type << std::endl;

        
        // param_listener2->parameters_interface_->set_parameters({rclcpp::Parameter("module_names", std::vector<std::string>{"ina_module_1", "ina_module_2"})});
        if(update_params_list( "modules", "module_names",node->get_node_parameters_interface())) {
         
        // param_listener2->refresh_dynamic_parameters();   // param_listener2->update_internal_params(params2);
        }
        update_params_list_type("modules", "ina_module_names",node->get_node_parameters_interface(), "INA226");
        //     // check if ends with type, then check if param type == "INA226"
        //     std::cout << "Filtering parameter: " << name << std::endl;
        //     if(name.size() < 5) {
        //         std::cout << "  too short" << std::endl;
        //         return false;
        //     }
        //     if(name.substr(name.size() - 4) != "type") {
        //         std::cout << "  does not end with type" << std::endl;
        //         return false;
        //     }
        //     rclcpp::Parameter param;
        //     param = node->get_node_parameters_interface()->get_parameter(name);
        //     if(param.value_to_string() != "INA226") {
        //         // std::cout << "  type is not INA226" << std::endl;
        //         return false;
        //     }
        //     // std::cout << "  accepted INA226 parameter" << std::endl;
        //     return true;
        // });
    auto param_listener2 =
        std::make_shared<mirte_telemetrix_cpp_ina226::ParamListener>(node);
        // doe het maar zo:
            auto params2 = param_listener2->get_params();
    // std::cout << params2.a.module_names_map.size() << std::endl;
    // std::cout << params2.a.module_names_map.size() << std::endl;
    // std::cout << params2.a.module_names_map.begin()->first << std::endl;
    // std::cout << params2.test << std::endl;
    // std::cout << params2.module_names.size() << std::endl;
    std::cout << params2.modules.ina_module_names_map.size() << std::endl;
    // for(const auto& [key, value] : params2.modules.module_names_map) {
    //     std::cout << "Module name: " << key << std::endl;
    //     std::cout << "  Type: " << value.type << std::endl;
    //     std::cout << "  Pins SDA: " << value.pins.sda << std::endl;
    //     std::cout << "  Pins SCL: " << value.pins.scl << std::endl;
    //     std::cout << "  Addr: " << value.addr << std::endl;
    // }
    std::cout << "Parameters loaded successfully." << std::endl;
    // std::cout << params2.modules.ina_module_names_map.begin()->second.power_low_time << std::endl;
  } catch (const rclcpp::exceptions::InvalidParameterValueException &e) {
    RCLCPP_FATAL(node->get_logger(), "Failed to initialize ParamListener: %s",
                 e.what());
    return 1;
  }
  rclcpp::spin(node);
  // auto param_listener =
  // std::make_shared<mirte_telemetrix_cpp::ParamListener>(node); auto params =
  // param_listener->get_params();

  return 0;
}
