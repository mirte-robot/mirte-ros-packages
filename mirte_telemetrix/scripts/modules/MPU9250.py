import rospy
from sensor_msgs import Imu
import math


class MPU9250:
    def __init__(self, board, module_name, module, board_mapping):
        self.name = module_name
        self.board = board
        self.module = module
        self.board_mapping = board_mapping
        self.acceleration = [0, 0, 0]
        self.gyroscope = [0, 0, 0]
        self.magnetometer = [0, 0, 0]
        self.i2c_port = None

    async def start(self):
        if self.board_mapping.get_mcu() == "pico":
            if "connector" in self.module:
                pins = self.board_mapping.connector_to_pins(self.module["connector"])
            else:
                # TODO: no other boards have support for this yet
                pins = self.module["pins"]
            pin_numbers = {}
            for item in pins:
                pin_numbers[item] = self.board_mapping.pin_name_to_pin_number(
                    pins[item]
                )
            self.i2c_port = self.board_mapping.get_I2C_port(pin_numbers["sda"])
            try:
                await self.board.set_pin_mode_i2c(
                    i2c_port=self.i2c_port,
                    sda_gpio=pin_numbers["sda"],
                    scl_gpio=pin_numbers["scl"],
                )
            except Exception as e:
                pass

        self.imu_publisher = rospy.Publisher(
            f"/mirte/{self.name}/imu", Imu, queue_size=1
        )

        # TODO: no support yet for other addresses than 0x68
        await self.board.sensors.add_mpu9250(self.i2c_port, self.callback)

    async def callback(self, acc, gyro, mag):
        self.acceleration = acc
        self.gyroscope = gyro
        self.magnetometer = mag
        self.pub()

    def pub(self):
        self.imu_publisher.publish(
            Imu(
                linear_acceleration=[
                    a * 9.81 for a in self.acceleration
                ],  # convert from g to m/s^2
                angular_velocity=[
                    g * math.pi / 180.0 for g in self.gyroscope
                ],  # convert from degrees/s to rad/s
                orientation=self.magnetometer,  # unknown if we need to convert this
                # TODO: what should these values be?
                # currently set to -1 to indicate unknown
                orientation_covariance=[-1, 0, 0, 0, 0, 0, 0, 0, 0],
                angular_velocity_covariance=[-1, 0, 0, 0, 0, 0, 0, 0, 0],
                linear_acceleration_covariance=[-1, 0, 0, 0, 0, 0, 0, 0, 0],
            )
        )
