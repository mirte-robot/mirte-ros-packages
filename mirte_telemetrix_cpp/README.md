# mirte-telemetrix-cpp

```make -k CXX=include-what-you-use CXXFLAGS="-Xiwyu --error_always"```


# Feature matrix:

|                        | ROS_Telemetrix_aio (ros1) | mirte-telemetrix-cpp | remarks                                                                               |
| ---------------------- | ------------------------- | -------------------- | ------------------------------------------------------------------------------------- |
| **Sensors**            |                           |                      |                                                                                       |
| Sonar                  | ✅                        | ✅                   |                                                                                       |
| intensity              | ✅                        | ✅                   |                                                                                       |
| encoders               | ✅                        | ✅                   |                                                                                       |
| get pin service        | ✅                        | ✅                   | Reading output pins results in undefined behavior (Tested digital read a digital out) |
| **Actuators**          |                           |                      |                                                                                       |
| oled                   | ✅                        | 🔷                   | OLED is now a Module. (Maybe the parsing can be improved to be more like old style)   |
| servos                 | ✅                        | ✅                   |                                                                                       |
| keypad                 | ✅                        | ✅                   |                                                                                       |
| motors                 | ✅                        | ❓                   | L9110S (PP) & L298N (DDP) work                                                        |
| set pin service        | ✅                        | ✅                   | Digital not tested as much                                                            |
| **Mirte-master parts** |                           |                      |                                                                                       |
| hiwonder servo         | ✅                        | ✅                   |                                                                                      |
| pca9685 pwm            | ✅                        | ✅                   |                                                                                       |
| ina226                 | ✅                        | ✅                   | shutdown relay not implemented as hw is not working.                                  |
| imu                    | ✅                        | ✅                   | mpu9250                                                                               |
| ledstrip               | ✅                        | ✅                   |                                                                                       |
| oled module            | ✅                        | ✅                   |                                                                                       |



# local run:
./mirte... --ros-args -r __node:=mirte -r __ns:=/mirte --params-file /root/ros2_ws/install/mirte_telemetrix_cpp/share/mirte_telemetrix_cpp/config/mirte_user_config.yaml 

# REP adhearance
All sensor publishers use QoS profile `SystemDefaultQoS`, as specified in [REP2003](https://www.ros.org/reps/rep-2003.html)

The Sonar Distance Sensor reports out of range values as specified in [REP0117](https://www.ros.org/reps/rep-0117.html)

The IMU (`MPU9250`) sensors behaves as specified in [REP0145](https://www.ros.org/reps/rep-0145.html)
