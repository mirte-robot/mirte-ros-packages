#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty, EmptyResponse
from mirte_msgs.srv import SetMotorSpeed, SetMotorSpeedResponse, SetMotorSpeedRequest


def my_service_callback(request, i):
    # Add your service logic here
    print(request, i)
    # rospy.loginfo("Service called", i, request.speed)
    # Return an appropriate response
    return SetMotorSpeedResponse()


if __name__ == "__main__":
    # Initialize the ROS node
    rospy.init_node("my_node")
    # Create the service
    my_service1 = rospy.Service(
        "/mirte/set_left_front_speed",
        SetMotorSpeed,
        lambda request: my_service_callback(request, 1),
    )
    my_service2 = rospy.Service(
        "/mirte/set_left_back_speed",
        SetMotorSpeed,
        lambda request: my_service_callback(request, 2),
    )
    my_service3 = rospy.Service(
        "/mirte/set_right_front_speed",
        SetMotorSpeed,
        lambda request: my_service_callback(request, 3),
    )
    my_service4 = rospy.Service(
        "/mirte/set_right_back_speed",
        SetMotorSpeed,
        lambda request: my_service_callback(request, 4),
    )

    # Spin the node to process callbacks
    rospy.spin()
