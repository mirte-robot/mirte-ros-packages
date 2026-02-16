#pragma once
#include <memory>

#include <rclcpp/node.hpp>

#include <tmx_cpp/tmx.hpp>
#include <vector>

class Mirte_Board;
struct NodeData {
  std::shared_ptr<rclcpp::Node> nh;
  std::shared_ptr<tmx_cpp::TMX> tmx;
  std::shared_ptr<Mirte_Board> board;
  std::function<void(std::chrono::duration<double, std::milli>,
                     std::function<void()>)>
      add_timer;
  // timers: vector of duration and vector of callback, one ros timer per
  // duration
  std::vector<std::pair<std::chrono::duration<double, std::milli>,
                        std::pair<rclcpp::TimerBase::SharedPtr,
                                  std::vector<std::function<void()>>>>>
      timers;
};
