import time

import rospy
from mirte_msgs.msg import Heading
import subprocess
import asyncio


class HMC5883:
    def __init__(
        self,
        board,
        module_name,
        module,
        board_mapping,
        loop,
        global_data,  # obj with some shared data, like SOC
    ):
        self.board = board
        self.sensor = module
        self.name = module_name
        self.loop = loop
        self.global_data = global_data
        self.init_awaits = []
        self.enabled = True
        if board_mapping.get_mcu() == "pico":
            if "connector" in self.sensor:
                pins = board_mapping.connector_to_pins(self.sensor["connector"])
            else:
                pins = self.sensor["pins"]
            pin_numbers = {}
            for item in pins:
                pin_numbers[item] = board_mapping.pin_name_to_pin_number(pins[item])
            self.i2c_port = board_mapping.get_I2C_port(pin_numbers["sda"])
            self.init_awaits.append(
                self.board.set_pin_mode_i2c(
                    i2c_port=self.i2c_port,
                    sda_gpio=pin_numbers["sda"],
                    scl_gpio=pin_numbers["scl"],
                )
            )
        else:
            print("hmc module probably not supported on your hardware!!!!")
            self.init_awaits.append(self.board.set_pin_mode_i2c(i2c_port=self.i2c_port))
        self.pub = rospy.Publisher(
            "mirte/compass/" + self.name, Heading, queue_size=1, latch=True
        )

    async def start(self):
        for ev in self.init_awaits:
            try:  # catch set_pin_mode_i2c already for this port
                z = await ev
            except Exception as e:
                pass
        await asyncio.sleep(3)
        await self.board.sensors.add_hmc5883(self.i2c_port, self.callback)

    async def callback(self, data):
        # TODO: this takes the raw heading values, offset is not removed from it
        msg = Heading(data[0], data[1], data[2])
        self.pub.publish(msg)
