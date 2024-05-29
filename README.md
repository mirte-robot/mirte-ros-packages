# API

The motors are controlled with PWM as as service. By default the topic published by the ROS diff_drive_controller (/mobile_base_controller/cmd_vel) will call that service. One has to disable the ROS diff_drive_controller () to manually set the PWM signal by using the service (eg from commandline).
Currently, the default parameters are set to use the Mirte Arduino-nano PCB.

## Subscribed Topics
/mobile_base_controller/cmd_vel (geometry_msgs/Twist)

## Published Topics
Topics published to depend on the parameters given.

/mirte/<name>_distance (sensor_msgs/Range)
/mirte/<name>_encoder (mirte_msgs/Encoder)
/mirte/<name>_intensity (mirte_msgs/Intensity)

## Services
/mirte_navigation/move
/mirte_navigation/turn
/mirte_pymata/set_<name>_motor_pwm
/mirte/get_pin_value
/mirte/set_pin_mode
/mirte/set_pin_value
/mirte_service_api/get_<topic_name>


## Parameters
Default parameters are in config/mirte_base_config.yaml

### Device settings
device/mirte/type (string, default: "mirte")
    The type of board connected. Currently only suppots Mirte. Future support will also include Lego Mindstorms and M-bot.
device/mirte/dev (string, default: "/dev/ttyUSB0")
    The linux device name of the board.

#### Distance sensor
distance/<name>_distance/dev (string, default: "mirte")
distance/<name>_distance/frequency (int, default: 10)
    The frequency to publish new sensor data
distance/<name>_distance/pin (array)
    The (arduino/stm32) pins it is connected to [trigger-pin, echo-pin] (eg [8. 13])

#### Encoder sensor
Note: the encoder sensor does not have a frequency since it uses the interrupt pins
encoder/<name>_encoder/device (string, default: "mirte")
encoder/<name>_encoder/pin (int)
encoder/<name>_encoder/ticks_per_wheel (int, default: 20)

#### Intensity sensor
intensity/<name>_intensity/device (string, default: "mirte")
intensity/<name>_intensity/frequency (int, default: 10)
intensity/<name>_intensity/pin (int)

#### Motor settings
motor/<name>_motor/device (string, default: "mirte")
motor/<name>_motor/pin (array)
     The (arduino/stm32) pins the motor (h-bridge) is connected to. When using 2 pins per motor it will assume MX1919 (both PWM signals). When using 3 pins it will assume L298N (1 PWM, 2 non-PWM)



# Test code style
To check the C++ and Python code style run:
```sh
pip install black
black --check **/**.py
# Fix by using
black **/**.py

sudo apt install clang-format # preferably version 14, 10 should be fine
clang-format --dry-run --Werror ./**/**.cpp -style=llvm
# Fix by using
clang-format --Werror ./**/**.cpp -style=llvm -i
```

# Required packages Telemetrix
Requires https://github.com/mirte-robot/tmx-pico-aio.git to be installed ( ```pip install git+https://github.com/mirte-robot/tmx-pico-aio.git``` ) for the Pico and https://github.com/mirte-robot/telemetrix-aio.git for the STM32 and Arduino Nano (```pip install git+https://github.com/mirte-robot/telemetrix-aio.git```).