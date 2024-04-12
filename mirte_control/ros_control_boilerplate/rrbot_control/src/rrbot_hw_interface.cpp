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

#include <rrbot_control/rrbot_hw_interface.h>
#include <iostream>
#include "ros/ros.h"
#include <mirte_msgs/SetServoAngle.h>
#include <mirte_msgs/ServoPosition.h>


namespace rrbot_control
{

double data[4] = {0.0, 0.0, 0.0, 0.0};

ros::Subscriber sub0;
ros::Subscriber sub1;
ros::Subscriber sub2;
ros::Subscriber sub3;

ros::ServiceClient client0;
ros::ServiceClient client1;
ros::ServiceClient client2;
ros::ServiceClient client3;

mirte_msgs::SetServoAngle srv0;
mirte_msgs::SetServoAngle srv1;
mirte_msgs::SetServoAngle srv2;
mirte_msgs::SetServoAngle srv3;


void callbackJoint0(const mirte_msgs::ServoPosition::ConstPtr& msg)
{
  data[0] = msg->angle;
}

void callbackJoint1(const mirte_msgs::ServoPosition::ConstPtr& msg)
{
  data[1] = msg->angle;
}

void callbackJoint2(const mirte_msgs::ServoPosition::ConstPtr& msg)
{
  data[2] = msg->angle;
}

void callbackJoint3(const mirte_msgs::ServoPosition::ConstPtr& msg)
{
  data[3] = msg->angle;
}



RRBotHWInterface::RRBotHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model)
  : ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
{
  ros::service::waitForService("/mirte/set_servoRot_servo_angle", -1);
  ros::service::waitForService("/mirte/set_servoShoulder_servo_angle", -1);
  ros::service::waitForService("/mirte/set_servoElbow_servo_angle", -1);
  ros::service::waitForService("/mirte/set_servoWrist_servo_angle", -1);

  client0 = nh.serviceClient<mirte_msgs::SetServoAngle>("/mirte/set_servoRot_servo_angle", true);
  client1 = nh.serviceClient<mirte_msgs::SetServoAngle>("/mirte/set_servoShoulder_servo_angle", true);
  client2 = nh.serviceClient<mirte_msgs::SetServoAngle>("/mirte/set_servoElbow_servo_angle", true);
  client3 = nh.serviceClient<mirte_msgs::SetServoAngle>("/mirte/set_servoWrist_servo_angle", true);

  sub0 = nh.subscribe("/mirte/servos/servoRot/position", 1, rrbot_control::callbackJoint0);
  sub1 = nh.subscribe("/mirte/servos/servoShoulder/position", 1, rrbot_control::callbackJoint1);
  sub2 = nh.subscribe("/mirte/servos/servoElbow/position", 1, rrbot_control::callbackJoint2);
  sub3 = nh.subscribe("/mirte/servos/servoWrist/position", 1, rrbot_control::callbackJoint3);



  ROS_INFO_NAMED("rrbot_hw_interface", "RRBotHWInterface Ready.");
}


void RRBotHWInterface::read(ros::Duration& elapsed_time)
{
  for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id){
    if (joint_id == 1 || joint_id == 3){
       joint_position_[joint_id] = (data[joint_id]);
    } else {
       joint_position_[joint_id] = data[joint_id];
    }
  }
}

void RRBotHWInterface::write(ros::Duration& elapsed_time)
{
  // Safety
  enforceLimits(elapsed_time);

  srv0.request.angle = (float)(joint_position_command_[0]);
  if (!client0.call(srv0)){
//     ROS_INFO_NAMED("rrbot_hw_interface", "Motor 0 error");
  }


  srv1.request.angle = (float)(joint_position_command_[1]);
  if (!client1.call(srv1)){
//     ROS_INFO_NAMED("rrbot_hw_interface", "Motor 1 error");
  }

  srv2.request.angle = (float)(joint_position_command_[2]);
  if (!client2.call(srv2)){
//     ROS_INFO_NAMED("rrbot_hw_interface", "Motor 2 error");
  }

  srv3.request.angle = (float)(joint_position_command_[3]);
  if (!client3.call(srv3)){
//     ROS_INFO_NAMED("rrbot_hw_interface", "Motor 3 error");
  }


}

void RRBotHWInterface::enforceLimits(ros::Duration& period)
{
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

}  // namespace rrbot_control
