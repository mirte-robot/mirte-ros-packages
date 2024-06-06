/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Example ros_control hardware interface blank template for the RRBot
           For a more detailed simulation example, see sim_hw_interface.cpp
*/

#include "ros/ros.h"
#include <chrono>
#include <iostream>
#include <mirte_msgs/ServoPosition.h>
#include <mirte_msgs/SetServoAngle.h>
#include <rrbot_control/rrbot_hw_interface.h>

namespace rrbot_control {
const std::string servo_names[] = {"Rot", "Shoulder", "Elbow", "Wrist"};
const auto NUM_SERVOS = std::size(servo_names);
struct Servo_data {
  double data;
  bool init = false;
  bool moved = false;
  double last_move_update = -100;
};
std::array<Servo_data, NUM_SERVOS> servo_data;
bool initialized = false;
int init_steps = 0;

std::vector<ros::Subscriber> subs;
std::vector<ros::ServiceClient> clients;
std::vector<mirte_msgs::SetServoAngle> srvs;

void callbackJoint(const mirte_msgs::ServoPosition::ConstPtr &msg, int joint) {
  auto &servo = servo_data[joint];
  servo.data = msg->angle;
  servo.init = true;
  // the servo should only be written to iff the servo gets a new location or
  // when it's moved by gravity, then it needs the command again this will check
  // that the servo is moved
  if (std::abs(servo.last_move_update - servo.data) >
      0.01) { // 0.57deg, bit more than the 24 centidegrees/LSB
    servo.last_move_update = servo.data;
    servo.moved = true;
  }
}

const auto topic_format = "/mirte/servos/servo%s/position";

RRBotHWInterface::RRBotHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
    : ros_control_boilerplate::GenericHWInterface(nh, urdf_model) {
  this->connectServices();
  for (auto i = 0; i < NUM_SERVOS; i++) {
    auto topic = (boost::format(topic_format) % servo_names[i]).str();
    subs.push_back(nh.subscribe<mirte_msgs::ServoPosition>(
        topic, 1, boost::bind(&rrbot_control::callbackJoint, _1, i)));
    srvs.push_back(mirte_msgs::SetServoAngle());
  }
  ROS_INFO_NAMED("rrbot_hw_interface", "RRBotHWInterface Ready.");
}

const auto service_format = "/mirte/set_servo%s_servo_angle";
void RRBotHWInterface::connectServices() {
  ROS_INFO_NAMED("rrbot_hw_interface", "Connecting to the services...");
  for (auto i = 0; i < NUM_SERVOS; i++) {
    auto service = (boost::format(service_format) % servo_names[i]).str();

    ros::service::waitForService(service, -1);
  }
  { // Only mutex when actually writing to class vars.
    clients.clear();
    const std::lock_guard<std::mutex> lock(this->service_clients_mutex);
    for (auto i = 0; i < NUM_SERVOS; i++) {
      auto service = (boost::format(service_format) % servo_names[i]).str();

      clients.push_back(
          nh_.serviceClient<mirte_msgs::SetServoAngle>(service, true));
    }

    ROS_INFO_NAMED("rrbot_hw_interface", "Connected to the services");
  }
}
void RRBotHWInterface::read(ros::Duration &elapsed_time) {
  for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id) {
    joint_position_[joint_id] = servo_data[joint_id].data;
  }
}
void RRBotHWInterface ::start_reconnect() {
  using namespace std::chrono_literals;

  if (this->reconnect_thread.valid()) { // does it already exist or not?

    // Use wait_for() with zero milliseconds to check thread status.
    auto status = this->reconnect_thread.wait_for(0ms);

    if (status !=
        std::future_status::ready) { // Still running -> already reconnecting
      return;
    }
  }

  /* Run the reconnection on a different thread to not pause the ros-control
    loop. The launch policy std::launch::async makes sure that the task is run
    asynchronously on a new thread. */

  this->reconnect_thread =
      std::async(std::launch::async, [this] { this->connectServices(); });
}
void RRBotHWInterface::write(ros::Duration &elapsed_time) {
  // Safety
  enforceLimits(elapsed_time);
  if (initialized) {
    for (auto i = 0; i < NUM_SERVOS; i++) {
      srvs[i].request.angle = (float)(joint_position_command_[i]);
    }
  } else {
    if (std::all_of(std::begin(servo_data), std::end(servo_data),
                    [](Servo_data &x) { return x.init; })) {

      // TOOD: why do we get a segfault when we
      // set joint_positino_command_ in the contructor?
      for (auto i = 0; i < NUM_SERVOS; i++) {
        srvs[i].request.angle = servo_data[i].data;
      }
      ++init_steps;
      for (auto i = 0; i < NUM_SERVOS; i++) {
        joint_position_command_[i] = servo_data[i].data;
      }
      if (init_steps == 50) {
        initialized = true;
        ROS_INFO_NAMED("rrbot_hw_interface", "Initialized arm");
      }
    }
  }
  // all initialized?
  if (std::all_of(std::begin(servo_data), std::end(servo_data),
                  [](auto x) { return x.init; })) {
    const std::lock_guard<std::mutex> lock(this->service_clients_mutex);
    static float last_req[NUM_SERVOS] = {-100.0};
    for (auto i = 0; i < NUM_SERVOS; i++) {

      // Only set the servo when there is a new command or the servo is moved by
      // hand or gravity.
      if (last_req[i] != srvs[i].request.angle || servo_data[i].moved) {
        last_req[i] = srvs[i].request.angle;
        servo_data[i].moved = false;
        if (!clients[i].call(srvs[i])) {
          //   // ROS_INFO_NAMED("rrbot_hw_interface", "Motor 0 error");

          this->start_reconnect();
          return;
        }
      }
    }
  }
}

void RRBotHWInterface::enforceLimits(ros::Duration &period) {
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
  //
  // CHOOSE THE TYPE OF JOINT LIMITS INTERFACE YOU WANT TO USE
  // YOU SHOULD ONLY NEED TO USE ONE SATURATION INTERFACE,
  // DEPENDING ON YOUR CONTROL METHOD
  //
  // EXAMPLES:
  //
  // Saturation Limits ---------------------------
  //
  // Enforces position and velocity
  pos_jnt_sat_interface_.enforceLimits(period);
  //
  // Enforces velocity and acceleration limits
  // vel_jnt_sat_interface_.enforceLimits(period);
  //
  // Enforces position, velocity, and effort
  // eff_jnt_sat_interface_.enforceLimits(period);

  // Soft limits ---------------------------------
  //
  // pos_jnt_soft_limits_.enforceLimits(period);
  // vel_jnt_soft_limits_.enforceLimits(period);
  // eff_jnt_soft_limits_.enforceLimits(period);
  //
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
}

} // namespace rrbot_control
