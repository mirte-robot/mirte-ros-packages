// #include <chrono>
// #include <functional>
// #include <mirte_base_control.hpp>
// // #include <ros/callback_queue.h>

// void controlLoop(mirte_base_control::MirteBaseHWInterface &hw,
//                  controller_manager::ControllerManager &cm,
//                  std::chrono::system_clock::time_point &last_time) {
//   std::chrono::system_clock::time_point current_time =
//       std::chrono::system_clock::now();
//   std::chrono::duration<double> elapsed_time = current_time - last_time;
//   rclcpp::Duration elapsed(elapsed_time.count());
//   last_time = current_time;

//   hw.read(elapsed);
//   cm.update(rclcpp::Time::now(), elapsed);
//   hw.write(elapsed);
// }

// int main(int argc, char **argv) {
//   rclcpp::init(argc, argv, "my_robot_base_node");

//   MirteBaseHWInterface hw;
//   controller_manager::ControllerManager cm(&hw, hw.nh);

//   double control_frequency;
//   hw.private_nh.param<double>("control_frequency", control_frequency, 10.0);

//   rclcpp::CallbackQueue my_robot_queue;
//   rclcpp::AsyncSpinner my_robot_spinner(1, &my_robot_queue);

//   std::chrono::system_clock::time_point last_time =
//       std::chrono::system_clock::now();
//   rclcpp::TimerOptions control_timer(
//       rclcpp::Duration(1 / control_frequency),
//       std::bind(controlLoop, std::ref(hw), std::ref(cm),
//       std::ref(last_time)), &my_robot_queue);
//   rclcpp::Timer control_loop = hw.nh.createTimer(control_timer);
//   my_robot_spinner.start();
//   rclcpp::spin();

//   return 0;
// }
