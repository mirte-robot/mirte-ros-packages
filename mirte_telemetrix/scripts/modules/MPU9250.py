import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Quaternion
from mirte_msgs.srv import GetIMU, GetIMUResponse
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
        self.last_message = Imu()

    async def start(self):
        if self.board_mapping.get_mcu() != "pico":
            return

        if "connector" in self.module:
            pins = self.board_mapping.connector_to_pins(self.module["connector"])
        else:
            # TODO: no other boards have support for this yet
            pins = self.module["pins"]
        pin_numbers = {}
        for item in pins:
            pin_numbers[item] = self.board_mapping.pin_name_to_pin_number(pins[item])
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
            f"mirte/{self.name}/imu", Imu, queue_size=1
        )

        if "frame_id" in self.module:
            self.frame_id = self.module["frame_id"]
        else:
            self.frame_id = f"imu_{self.name}"

        # self.i2c_port = 0
        # TODO: no support yet for other addresses than 0x68
        await self.board.sensors.add_mpu9250(self.i2c_port, self.callback)

        self.serv = rospy.Service(
            f"mirte/{self.name}/get_imu",
            GetIMU,
            self.get_imu_service,
        )

    async def callback(self, acc, gyro, mag):
        self.acceleration = acc
        self.gyroscope = gyro
        self.magnetometer = mag
        self.pub()

    def pub(self):
        x_index = 1
        y_index = 0
        z_index = 2
        lin_acc = Vector3(
            x=self.acceleration[x_index]
            * 9.81,  # positive forward, Imu is positioned differently
            y=-self.acceleration[y_index] * 9.81,  # positive left
            z=-self.acceleration[z_index] * 9.81,  # up is positive
        )

        # this is probably okay. Only real usefull value is z.
        ang_vel = Vector3(
            x=-self.gyroscope[x_index] * math.pi / 180.0,
            y=self.gyroscope[y_index] * math.pi / 180.0,
            z=self.gyroscope[z_index] * math.pi / 180.0,
        )
        orie = Quaternion(
            x=self.magnetometer[x_index],
            y=self.magnetometer[y_index],
            z=self.magnetometer[z_index],
        )
        # lin_acc.x =
        self.last_message = Imu(
            linear_acceleration=lin_acc,  # converted from g to m/s^2
            angular_velocity=ang_vel,  # converted from degrees/s to rad/s
            orientation=orie,  # unknown if we need to convert this
            # TODO: what should these values be?
            # currently set to -1 to indicate unknown
            orientation_covariance=[-1, 0, 0, 0, 0, 0, 0, 0, 0],
            angular_velocity_covariance=[-1, 0, 0, 0, 0, 0, 0, 0, 0],
            linear_acceleration_covariance=[-1, 0, 0, 0, 0, 0, 0, 0, 0],
        )
        self.last_message.header.stamp = rospy.Time.now()
        self.last_message.header.frame_id = self.frame_id
        self.imu_publisher.publish(self.last_message)

    def get_imu_service(self, req):
        return GetIMUResponse(self.last_message)
