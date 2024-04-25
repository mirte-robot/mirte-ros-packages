#!/usr/bin/env python3.8
import asyncio
import os, os.path
import sys
import time
import math
import rospy
import signal
import aiorospy
import io
from inspect import signature
from tmx_pico_aio import tmx_pico_aio
from telemetrix_aio import telemetrix_aio
from typing import Literal, Tuple
import subprocess

try:
    import gpiod
except:
    pass

from modules import MPU9250


devices = rospy.get_param("/mirte/device")


current_soc = "???"  # TODO: change to something better, but for now we communicate SOC from the powerwatcher to the Oled using a global


# Until we update our own fork of TelemtrixAIO to the renamed pwm calls
# we need to add a simple wrapper
async def set_pin_mode_analog_output(board, pin):
    if board_mapping.get_mcu() == "pico":
        await board.set_pin_mode_pwm_output(pin)
    else:
        await board.set_pin_mode_analog_output(pin)


async def analog_write(board, pin, value):
    if board_mapping.get_mcu() == "pico":
        await board.pwm_write(pin, value)
    else:
        await board.analog_write(pin, value)


def get_obj_value(s, obj, key, def_value=None):
    # shorthand of self.key = obj["key"] if "key" in obj else def_value with type checking
    def_type = type(def_value)
    out = obj[key] if key in obj else def_value
    out_type = type(out)
    if def_type != out_type:
        if def_type == int:
            out = int(out)
        if def_type == float:
            out = float(out)
        if def_type == str:
            out = str(out)
        if def_type == bool:
            out = bool(out)
    setattr(s, key, out)


# Import ROS message types
from std_msgs.msg import Header, Int32, Float32
from std_srvs.srv import SetBool, SetBoolResponse
from sensor_msgs.msg import Range, BatteryState
from mirte_msgs.msg import *

# Import ROS services
from mirte_msgs.srv import *

from bitstring import BitArray
import textwrap

from PIL import Image, ImageDraw, ImageFont

# Currently loading the PIL default font, which is
# monospace, so works with python textwrap
font = ImageFont.load_default()

from adafruit_ssd1306 import _SSD1306


import mappings.default
import mappings.nanoatmega328
import mappings.pico
import mappings.blackpill_f103c8
import mappings.pcb

board_mapping = mappings.default

devices = rospy.get_param("/mirte/device")

if devices["mirte"]["type"] == "pcb":
    board_mapping = mappings.pcb
    if "version" in devices["mirte"]:
        if "board" in devices["mirte"]:
            board_mapping.set_version(
                devices["mirte"]["version"], devices["mirte"]["board"]
            )
        else:
            board_mapping.set_version(devices["mirte"]["version"])

if devices["mirte"]["type"] == "breadboard":
    if "board" in devices["mirte"]:
        if devices["mirte"]["board"] == "blackpill_f103c8":
            board_mapping = mappings.blackpill_f103c8
        elif (
            "nanoatmega328"
            in devices["mirte"][
                "board"
            ]  # will trigger for nanoatmega328new and nanoatmega328
            or devices["mirte"]["board"] == "uno"  # uno has the same pinout
        ):
            board_mapping = mappings.nanoatmega328
        elif devices["mirte"]["board"] == "pico":
            board_mapping = mappings.pico
        else:
            board_mapping = mappings.default


def get_pin_numbers(component):
    # devices = rospy.get_param("/mirte/device")
    # device = devices[component["device"]]
    pins = {}
    if "connector" in component:
        pins = board_mapping.connector_to_pins(component["connector"])
    if "pins" in component:
        pins = component["pins"]
    if "pin" in component:
        pins["pin"] = component["pin"]
    # convert pin naming to numbers
    pin_numbers = {}
    for item in pins:
        pin_numbers[item] = board_mapping.pin_name_to_pin_number(pins[item])

    return pin_numbers


# Abstract Sensor class
class SensorMonitor:
    def __init__(self, board, sensor, publisher):
        self.board = board
        self.pins = get_pin_numbers(sensor)
        self.publisher = publisher
        self.max_freq = 10
        if "max_frequency" in sensor:
            self.max_freq = sensor["max_frequency"]
        self.differential = 0
        if "differential" in sensor:
            self.differential = sensor["differential"]
        self.loop = asyncio.get_event_loop()
        self.last_publish_time = -1
        self.last_publish_value = {}
        rospy.loginfo(
            "Sensor initialized on topic %s (max_freq: %d, differential: %d)",
            self.publisher.name,
            self.max_freq,
            self.differential,
        )

    def get_header(self):
        header = Header()
        header.stamp = rospy.Time.now()
        return header

    # NOTE: although there are no async functions in this
    # the function needs to be async since it is called
    # inside a callback of an awaited part of telemetrix
    async def publish_imp(self, data):
        self.publisher.publish(data)
        self.last_publish_value = data

    async def publish(self, data):
        if self.max_freq == -1:
            await self.publish_imp(data)
        else:
            now_millis = int(round(time.time() * 1000))

            # always publish the first message (TODO: and maybe messages that took too long 2x 1/freq?)
            if self.last_publish_time == -1:
                await self.publish_imp(data)
                self.last_publish_time = now_millis

            # from then on publish if needed based on max_freq
            if now_millis - self.last_publish_time >= 1000.0 / self.max_freq:
                await self.publish_imp(data)
                self.last_publish_time += (
                    1000.0 / self.max_freq
                )  # Note: this should not be set to now_millis. This is due to Nyquist.


class KeypadMonitor(SensorMonitor):
    def __init__(self, board, sensor):
        pub = rospy.Publisher("/mirte/keypad/" + sensor["name"], Keypad, queue_size=1)
        srv = rospy.Service(
            "/mirte/get_keypad_" + sensor["name"], GetKeypad, self.get_data
        )
        super().__init__(board, sensor, pub)
        self.last_debounce_time = 0
        self.last_key = ""
        self.last_debounced_key = ""
        self.pressed_publisher = rospy.Publisher(
            "/mirte/keypad/" + sensor["name"] + "_pressed", Keypad, queue_size=1
        )
        self.last_publish_value = Keypad()

    def get_data(self, req):
        return GetKeypadResponse(self.last_publish_value.key)

    async def start(self):
        await self.board.set_pin_mode_analog_input(
            self.pins["pin"] - board_mapping.get_analog_offset(),
            differential=self.differential,
            callback=self.publish_data,
        )

    async def publish_data(self, data):
        # Determine the key that is pressed
        # TODO: these values were found on a 12 bits adc, and
        # added a scaling for the actual bits used. We could
        # calculate this with the R values used.
        key = ""
        if data[2] < 70 / 4096 * (2 ** board_mapping.get_adc_bits()):
            key = "left"
        elif data[2] < 230 / 4096 * (2 ** board_mapping.get_adc_bits()):
            key = "up"
        elif data[2] < 410 / 4096 * (2 ** board_mapping.get_adc_bits()):
            key = "down"
        elif data[2] < 620 / 4096 * (2 ** board_mapping.get_adc_bits()):
            key = "right"
        elif data[2] < 880 / 4096 * (2 ** board_mapping.get_adc_bits()):
            key = "enter"

        # Do some debouncing
        if self.last_key is not key:
            self.last_debounce_time = data[3]

        debounced_key = ""
        if data[3] - self.last_debounce_time > 0.1:
            debounced_key = key

        # Publish the last debounced key
        keypad = Keypad()
        keypad.header = self.get_header()
        keypad.key = debounced_key
        await self.publish(keypad)

        # check if we need to send a pressed message
        if (self.last_debounced_key != "") and (
            self.last_debounced_key is not debounced_key
        ):
            pressed = Keypad()
            pressed.header = self.get_header()
            pressed.key = self.last_debounced_key
            self.pressed_publisher.publish(pressed)

        self.last_key = key
        self.last_debounced_key = debounced_key


class DistanceSensorMonitor(SensorMonitor):
    def __init__(self, board, sensor):
        pub = rospy.Publisher(
            "/mirte/distance/" + sensor["name"], Range, queue_size=1, latch=True
        )
        srv = rospy.Service(
            "/mirte/get_distance_" + sensor["name"], GetDistance, self.get_data
        )
        super().__init__(board, sensor, pub)
        self.last_publish_value = Range()

    def get_data(self, req):
        return GetDistanceResponse(self.last_publish_value.range)

    async def start(self):
        #   await self.board.set_scan_delay(100)
        await self.board.set_pin_mode_sonar(
            self.pins["trigger"], self.pins["echo"], self.publish_data
        )

    async def publish_data(self, data):
        # Although the initialization of this Range message
        # including some of the values could be placed in the
        # constructor for efficiency reasons. This does
        # for some reason not work though.
        range = Range()
        range.radiation_type = range.ULTRASOUND
        range.field_of_view = math.pi * 5
        range.min_range = 0.02
        range.max_range = 1.5
        range.header = self.get_header()
        range.range = data[2]
        await self.publish(range)


class DigitalIntensitySensorMonitor(SensorMonitor):
    def __init__(self, board, sensor):
        pub = rospy.Publisher(
            "/mirte/intensity/" + sensor["name"] + "_digital",
            IntensityDigital,
            queue_size=1,
            latch=True,
        )
        srv = rospy.Service(
            "/mirte/get_intensity_" + sensor["name"] + "_digital",
            GetIntensityDigital,
            self.get_data,
        )
        super().__init__(board, sensor, pub)
        self.last_publish_value = IntensityDigital()

    def get_data(self, req):
        return GetIntensityDigitalResponse(self.last_publish_value.value)

    async def start(self):
        await self.board.set_pin_mode_digital_input(
            self.pins["digital"], callback=self.publish_data
        )

    async def publish_data(self, data):
        intensity = IntensityDigital()
        intensity.header = self.get_header()
        intensity.value = bool(data[2])
        await self.publish(intensity)


class AnalogIntensitySensorMonitor(SensorMonitor):
    def __init__(self, board, sensor):
        pub = rospy.Publisher(
            "/mirte/intensity/" + sensor["name"], Intensity, queue_size=100
        )
        srv = rospy.Service(
            "/mirte/get_intensity_" + sensor["name"], GetIntensity, self.get_data
        )
        super().__init__(board, sensor, pub)
        self.last_publish_value = Intensity()

    def get_data(self, req):
        return GetIntensityResponse(self.last_publish_value.value)

    async def start(self):
        await self.board.set_pin_mode_analog_input(
            self.pins["analog"] - board_mapping.get_analog_offset(),
            differential=self.differential,
            callback=self.publish_data,
        )

    async def publish_data(self, data):
        intensity = Intensity()
        intensity.header = self.get_header()
        intensity.value = data[2]
        await self.publish(intensity)


class EncoderSensorMonitor(SensorMonitor):
    def __init__(self, board, sensor):
        pub = rospy.Publisher(
            "/mirte/encoder/" + sensor["name"], Encoder, queue_size=1, latch=True
        )
        srv = rospy.Service(
            "/mirte/get_encoder_" + sensor["name"], GetEncoder, self.get_data
        )
        self.speed_pub = rospy.Publisher(
            "/mirte/encoder_speed/" + sensor["name"], Encoder, queue_size=1, latch=True
        )
        super().__init__(board, sensor, pub)
        self.ticks_per_wheel = 20
        if "ticks_per_wheel" in sensor:
            self.ticks_per_wheel = sensor["ticks_per_wheel"]
        self.max_freq = -1
        self.last_publish_value = Encoder()
        self.speed_count = 0
        self.last_step = 0
        self.inverted = sensor["inverted"] if "inverted" in sensor else False

    def get_data(self, req):
        return GetEncoderResponse(self.last_publish_value.value)

    async def start(self):
        if board_mapping.get_mcu() == "pico":
            if "A" in self.pins:  # Only yet Pico support for quadrature encoder
                await self.board.set_pin_mode_encoder(
                    self.pins["A"], self.pins["B"], self.publish_data, True
                )
            else:
                await self.board.set_pin_mode_encoder(
                    self.pins["pin"], 0, self.publish_data, False
                )
        else:
            await self.board.set_pin_mode_encoder(
                self.pins["pin"], 2, self.ticks_per_wheel, self.publish_data
            )
        rospy.Timer(rospy.Duration(1.0 / 10.0), self.publish_speed_data)

    def publish_speed_data(self, event=None):
        encoder = Encoder()
        encoder.header = self.get_header()
        encoder.value = self.speed_count
        self.speed_count = 0
        self.speed_pub.publish(encoder)

    async def publish_data(self, data):
        encoder = Encoder()
        encoder.header = self.get_header()
        encoder.value = data[2]

        # Invert encoder pulses when quadrate is wired incorrectly
        if self.inverted:
            encoder.value = -encoder.value

        difference = self.last_step - encoder.value
        self.speed_count = self.speed_count + difference
        self.last_step = encoder.value

        await self.publish(encoder)


class Neopixel:
    def __init__(self, board, neo_obj):
        self.board = board
        self.settings = neo_obj
        self.pins = get_pin_numbers(neo_obj)
        self.name = neo_obj["name"]
        self.pixels = neo_obj["pixels"]  # num of leds
        get_obj_value(self, neo_obj, "max_intensity", 50)
        get_obj_value(self, neo_obj, "default_color", 0x000000)
        server = rospy.Service(
            f"/mirte/set_{self.name}_color_all", SetLEDValue, self.set_color_all_service
        )
        server = rospy.Service(
            f"/mirte/set_{self.name}_color_single",
            SetSingleLEDValue,
            self.set_color_single_service,
        )

    async def start(self):
        colors = self.unpack_color_and_scale(self.default_color)
        await board.set_pin_mode_neopixel(
            pin_number=self.pins["pin"],
            num_pixels=self.pixels,
            fill_r=colors[0],
            fill_g=colors[1],
            fill_b=colors[2],
        )

    async def set_color_all(self, color):
        colors = self.unpack_color_and_scale(color)
        await board.neopixel_fill(r=colors[0], g=colors[1], b=colors[2])

    async def set_color_single(self, pixel, color):
        colors = self.unpack_color_and_scale(color)
        if pixel >= self.pixels:
            return False
        await board.neo_pixel_set_value(
            pixel, r=colors[0], g=colors[1], b=colors[2], auto_show=True
        )
        return True

    def set_color_all_service(self, req):
        asyncio.run(self.set_color_all(req.value))
        return SetLEDValueResponse(True)

    def set_color_single_service(self, req):
        ok = asyncio.run(self.set_color_single(req.pixel, req.value))
        return SetSingleLEDValueResponse(ok)

    def unpack_color_and_scale(
        self, color_num
    ):  # hex color number (0x123456) or string ("0x123456")
        return [
            int(x * 0.5)
            for x in struct.unpack(
                "BBB", bytes.fromhex(hex(int(str(color_num), 0))[2:].zfill(6))
            )
        ]


class Servo:
    def __init__(self, board, servo_obj):
        self.board = board
        self.pins = get_pin_numbers(servo_obj)
        self.name = servo_obj["name"]
        self.min_pulse = 544
        if "min_pulse" in servo_obj:
            self.min_pulse = servo_obj["min_pulse"]
        self.max_pulse = 2400
        if "max_pulse" in servo_obj:
            self.max_pulse = servo_obj["max_pulse"]

    async def stop(self):
        await board.detach_servo(self.pins["pin"])

    async def start(self):
        await board.set_pin_mode_servo(self.pins["pin"], self.min_pulse, self.max_pulse)
        server = rospy.Service(
            "/mirte/set_" + self.name + "_servo_angle",
            SetServoAngle,
            self.set_servo_angle_service,
        )

    def set_servo_angle_service(self, req):
        asyncio.run(board.servo_write(self.pins["pin"], req.angle))
        return SetServoAngleResponse(True)


class Motor:
    def __init__(self, board, motor_obj):
        self.board = board
        self.pins = get_pin_numbers(motor_obj)
        self.name = motor_obj["name"]
        self.prev_motor_speed = 0
        self.initialized = False
        self.inverted = motor_obj["inverted"] if "inverted" in motor_obj else False

    async def start(self):
        server = rospy.Service(
            "/mirte/set_" + self.name + "_speed",
            SetMotorSpeed,
            self.set_motor_speed_service,
        )
        sub = rospy.Subscriber(
            "/mirte/motor_" + self.name + "_speed", Int32, self.callback
        )

    def callback(self, data):
        asyncio.run(self.set_speed(data.data))

    def set_motor_speed_service(self, req):
        asyncio.run(self.set_speed(req.speed))
        return SetMotorSpeedResponse(True)


class PPMotor(Motor):
    # Ideally one would initialize the pins in the constructor. But
    # since some mcu's have some voltage on pins when they are not
    # initialized yet icw some motor controllers that use the
    # difference between the pins to determine speed and direction
    # the motor will briefly move when initializing. This is unwanted.
    # When setting this on the mcu itself the this will be done fast
    # enough. But using telemetrix is a bit too slow fow this. We
    # therefore set the pin type on first move, and do this in a way
    # where it creates a movement in the same direction.
    async def init_motors(self, speed):
        if not self.initialized:
            if speed > 0:
                await set_pin_mode_analog_output(self.board, self.pins["p2"])
                await set_pin_mode_analog_output(self.board, self.pins["p1"])
            if speed < 0:
                await set_pin_mode_analog_output(self.board, self.pins["p1"])
                await set_pin_mode_analog_output(self.board, self.pins["p2"])
            self.initialized = True

    async def set_speed(self, speed):
        if self.inverted:
            speed = -speed
        if self.prev_motor_speed != speed:
            if speed == 0:
                await analog_write(self.board, self.pins["p2"], 0)
                await analog_write(self.board, self.pins["p1"], 0)
            elif speed > 0:
                await self.init_motors(speed)
                await analog_write(self.board, self.pins["p2"], 0)
                await analog_write(
                    self.board,
                    self.pins["p1"],
                    int(min(speed, 100) / 100.0 * board_mapping.get_max_pwm_value()),
                )
            elif speed < 0:
                await self.init_motors(speed)
                await analog_write(self.board, self.pins["p1"], 0)
                await analog_write(
                    self.board,
                    self.pins["p2"],
                    int(min(-speed, 100) / 100.0 * board_mapping.get_max_pwm_value()),
                )
            self.prev_motor_speed = speed


class DPMotor(Motor):
    # Ideally one would initialize the pins in the constructor. But
    # since some mcu's have some voltage on pins when they are not
    # initialized yet icw some motor controllers that use the
    # difference between the pins to determine speed and direction
    # the motor will briefly move when initializing. This is unwanted.
    # When setting this on the mcu itself the this will be done fast
    # enough. But using telemetrix is a bit too slow fow this. We
    # therefore set the pin type on first move, and do this in a way
    # where it creates a movement in the same direction.
    async def init_motors(self, speed):
        if not self.initialized:
            if speed > 0:
                await self.board.set_pin_mode_digital_output(self.pins["d1"])
                await set_pin_mode_analog_output(self.board, self.pins["p1"])
            if speed < 0:
                await set_pin_mode_analog_output(self.board, self.pins["p1"])
                await self.board.set_pin_mode_digital_output(self.pins["d1"])
            self.initialized = True

    async def set_speed(self, speed):
        if self.prev_motor_speed != speed:
            if speed == 0:
                await self.board.digital_write(self.pins["d1"], 0)
                await analog_write(self.board, self.pins["p1"], 0)
            elif speed > 0:
                await self.init_motors(speed)
                await self.board.digital_write(self.pins["d1"], 0)
                await analog_write(
                    self.board,
                    self.pins["p1"],
                    int(min(speed, 100) / 100.0 * board_mapping.get_max_pwm_value()),
                )
            elif speed < 0:
                await self.init_motors(speed)
                await self.board.digital_write(self.pins["d1"], 1)
                await analog_write(
                    self.board,
                    self.pins["p1"],
                    int(
                        board_mapping.get_max_pwm_value()
                        - min(abs(speed), 100)
                        / 100.0
                        * board_mapping.get_max_pwm_value()
                    ),
                )
            self.prev_motor_speed = speed


class DDPMotor(Motor):
    async def init_motors(self):
        if not self.initialized:
            await set_pin_mode_analog_output(self.board, self.pins["p1"])
            await self.board.set_pin_mode_digital_output(self.pins["d1"])
            await self.board.set_pin_mode_digital_output(self.pins["d2"])
            self.initialized = True

    async def set_speed(self, speed):
        # Make sure to set first set teh low pin. In this case the H-bridge
        # will never have two high pins.
        if self.prev_motor_speed != speed:
            await self.init_motors()
            if speed >= 0:
                await self.board.digital_write(self.pins["d1"], 0)
                await self.board.digital_write(self.pins["d2"], 0)
                await analog_write(
                    self.board,
                    self.pins["p1"],
                    int(min(speed, 100) / 100.0 * board_mapping.get_max_pwm_value()),
                )
                await self.board.digital_write(self.pins["d2"], 1)
            elif speed < 0:
                await self.board.digital_write(self.pins["d2"], 0)
                await self.board.digital_write(self.pins["d1"], 0)
                await analog_write(
                    self.board,
                    self.pins["p1"],
                    int(
                        min(abs(speed), 100) / 100.0 * board_mapping.get_max_pwm_value()
                    ),
                )
                await self.board.digital_write(self.pins["d1"], 1)
            self.prev_motor_speed = speed


# Extended adafruit _SSD1306
class Oled(_SSD1306):
    def __init__(
        self,
        width,
        height,
        board,
        oled_obj,
        port,
        loop,
        addr=0x3C,
        external_vcc=False,
        reset=None,
    ):
        self.board = board
        self.oled_obj = oled_obj
        self.addr = addr
        self.temp = bytearray(2)
        self.i2c_port = port
        self.failed = False
        self.loop = loop
        self.init_awaits = []
        self.write_commands = []

        # Add an extra byte to the data buffer to hold an I2C data/command byte
        # to use hardware-compatible I2C transactions.  A memoryview of the
        # buffer is used to mask this byte from the framebuffer operations
        # (without a major memory hit as memoryview doesn't copy to a separate
        # buffer).
        self.buffer = bytearray(((height // 8) * width) + 1)
        # self.buffer = bytearray(16)
        # self.buffer[0] = 0x40  # Set first byte of data buffer to Co=0, D/C=1
        if board_mapping.get_mcu() == "pico":
            if "connector" in oled_obj:
                pins = board_mapping.connector_to_pins(oled_obj["connector"])
            else:
                pins = oled_obj["pins"]
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
            self.init_awaits.append(self.board.set_pin_mode_i2c(i2c_port=self.i2c_port))
        time.sleep(1)
        super().__init__(
            memoryview(self.buffer)[1:],
            width,
            height,
            external_vcc=external_vcc,
            reset=reset,
            page_addressing=False,
        )

    async def start(self):
        server = rospy.Service(
            "/mirte/set_" + self.oled_obj["name"] + "_image",
            SetOLEDImage,
            self.set_oled_image_service,
        )
        for ev in self.init_awaits:
            try:  # catch set_pin_mode_i2c already for this port
                await ev
            except Exception as e:
                pass
        for cmd in self.write_commands:
            # // TODO: arduino will just stop forwarding i2c write messages after a single failed message. No feedback from it yet.
            out = await self.board.i2c_write(60, cmd, i2c_port=self.i2c_port)
            if out is None:
                await asyncio.sleep(0.05)
            if (
                out == False
            ):  # pico returns true/false, arduino returns always none, only catch false
                print("write failed start", self.oled_obj["name"])
                self.failed = True
                return
        self.default_image = True
        rospy.Timer(rospy.Duration(10), self.show_default)
        await self.show_default_async()

    def show_default(self, event=None):
        if not self.default_image:
            return
        try:
            # the ros service is started on a different thread than the asyncio loop
            # When using the normal loop.run_until_complete() function, both threads join in and the oled communication will get broken faster
            future = asyncio.run_coroutine_threadsafe(
                self.show_default_async(), self.loop
            )
            future.result()  # wait for it to be done
        except Exception as e:
            print(e)

    async def show_default_async(self):
        text = ""
        if "show_ip" in self.oled_obj and self.oled_obj["show_ip"]:
            ips = subprocess.getoutput("hostname -I").split(" ")
            text += "IPs: " + ", ".join(filter(None, ips))
        if "show_hostname" in self.oled_obj and self.oled_obj["show_hostname"]:
            text += "\nHn:" + subprocess.getoutput("cat /etc/hostname")
        if "show_wifi" in self.oled_obj and self.oled_obj["show_wifi"]:
            wifi = subprocess.getoutput("iwgetid -r").strip()
            if len(wifi) > 0:
                text += "\nWi-Fi:" + wifi
        if "show_soc" in self.oled_obj and self.oled_obj["show_soc"]:
            # TODO: change to soc ros service
            text += f"\nSOC: {current_soc}%"
        if len(text) > 0:
            await self.set_oled_image_service_async(
                SetOLEDImageRequest(type="text", value=text)
            )

    async def set_oled_image_service_async(self, req):
        if req.type == "text":
            text = req.value.replace("\\n", "\n")
            image = Image.new("1", (128, 64))
            draw = ImageDraw.Draw(image)
            split_text = text.splitlines()
            lines = []
            for i in split_text:
                lines.extend(textwrap.wrap(i, width=20))

            y_text = 1
            for line in lines:
                width, height = font.getsize(line)
                draw.text((1, y_text), line, font=font, fill=255)
                y_text += height
            self.image(image)
            await self.show_async()
        if req.type == "image":
            await self.show_png(
                "/usr/local/src/mirte/mirte-oled-images/images/" + req.value + ".png"
            )  # open color image

        if req.type == "animation":
            folder = (
                "/usr/local/src/mirte/mirte-oled-images/animations/" + req.value + "/"
            )
            number_of_images = len(
                [
                    name
                    for name in os.listdir(folder)
                    if os.path.isfile(os.path.join(folder, name))
                ]
            )
            for i in range(number_of_images):
                await self.show_png(folder + req.value + "_" + str(i) + ".png")

    def set_oled_image_service(self, req):
        self.default_image = False
        if self.failed:
            print("oled writing failed")
            return SetOLEDImageResponse(False)

        try:
            # the ros service is started on a different thread than the asyncio loop
            # When using the normal loop.run_until_complete() function, both threads join in and the oled communication will get broken faster
            future = asyncio.run_coroutine_threadsafe(
                self.set_oled_image_service_async(req), self.loop
            )
            future.result()  # wait for it to be done
        except Exception as e:
            print(e)
        return SetOLEDImageResponse(True)

    def show(self):
        """Update the display"""
        xpos0 = 0
        xpos1 = self.width - 1
        if self.width == 64:
            # displays with width of 64 pixels are shifted by 32
            xpos0 += 32
            xpos1 += 32
        if self.width == 72:
            # displays with width of 72 pixels are shifted by 28
            xpos0 += 28
            xpos1 += 28
        self.write_cmd(0x21)  # SET_COL_ADDR)
        self.write_cmd(xpos0)
        self.write_cmd(xpos1)
        self.write_cmd(0x22)  # SET_PAGE_ADDR)
        self.write_cmd(0)
        self.write_cmd(self.pages - 1)
        self.write_framebuf()

    def write_cmd(self, cmd):
        self.temp[0] = 0x80
        self.temp[1] = cmd
        self.write_commands.append([0x80, cmd])

    async def write_cmd_async(self, cmd):
        if self.failed:
            return
        self.temp[0] = 0x80
        self.temp[1] = cmd
        out = await self.board.i2c_write(60, self.temp, i2c_port=self.i2c_port)
        if out is None:
            await asyncio.sleep(0.05)
        if out == False:
            print("failed write oled 2")
            self.failed = True

    async def show_async(self):
        """Update the display"""
        # TODO: only update pixels that are changed
        xpos0 = 0
        xpos1 = self.width - 1
        if self.width == 64:
            # displays with width of 64 pixels are shifted by 32
            xpos0 += 32
            xpos1 += 32
        if self.width == 72:
            # displays with width of 72 pixels are shifted by 28
            xpos0 += 28
            xpos1 += 28

        try:
            await self.write_cmd_async(0x21)  # SET_COL_ADDR)
            await self.write_cmd_async(xpos0)
            await self.write_cmd_async(xpos1)
            await self.write_cmd_async(0x22)  # SET_PAGE_ADDR)
            await self.write_cmd_async(0)
            await self.write_cmd_async(self.pages - 1)
            await self.write_framebuf_async()
        except Exception as e:
            print(e)

    async def write_framebuf_async(self):
        if self.failed:
            return

        async def task(self, i):
            buf = self.buffer[i * 16 : (i + 1) * 16 + 1]
            buf[0] = 0x40
            out = await self.board.i2c_write(60, buf, i2c_port=self.i2c_port)
            if out is None:
                await asyncio.sleep(0.05)
            if out == False:
                print("failed wrcmd")
                self.failed = True

        for i in range(64):
            await task(self, i)

    def write_framebuf(self):
        for i in range(64):
            buf = self.buffer[i * 16 : (i + 1) * 16 + 1]
            buf[0] = 0x40
            self.write_commands.append(buf)

    async def show_png(self, file):
        image_file = Image.open(file)  # open color image
        image_file = image_file.convert("1", dither=Image.NONE)
        self.image(image_file)
        await self.show_async()


async def handle_set_led_value(req):
    led = rospy.get_param("/mirte/led")
    await analog_write(
        board,
        get_pin_numbers(led)["pin"],
        int(min(req.value, 100) / 100.0 * board_mapping.get_max_pwm_value()),
    )
    return SetLEDValueResponse(True)


# TODO: This needs a full refactor. Probably needs its own class
# with a member storing all settings of the pins (analog/digital)
# and whether or not a callback needs to be called.
# It pwill prbably only need one callback function anyway, pushing
# the values into the member variable.

pin_values = {}


# TODO: and this one probably needs to keep track of
# time as well, making sure that I can not call
# this one more often than another pin.
async def data_callback(data):
    global pin_values
    pin_number = data[1]
    if data[0] == 3:
        pin_number += board_mapping.get_analog_offset()
    pin_values[pin_number] = data[2]


def handle_get_pin_value(req):
    global pin_values
    # Map pin to the pin map if it is in there, or to
    # an int if raw pin number
    try:
        pin = board_mapping.pin_name_to_pin_number(req.pin)
    except:
        pin = int(req.pin)

    if not pin in pin_values:
        if req.type == "analog":
            asyncio.run(
                board.set_pin_mode_analog_input(
                    pin - board_mapping.get_analog_offset(), callback=data_callback
                )
            )
        if req.type == "digital":
            asyncio.run(board.set_pin_mode_digital_input(pin, callback=data_callback))

    # timeout after 5s, don't keep waiting on something that will never happen.
    start_time = time.time()
    while not pin in pin_values and time.time() - start_time < 5.0:
        time.sleep(0.001)
    if pin in pin_values:
        value = pin_values[pin]
    else:
        value = -1  # device did not report back, so return error value.

    return GetPinValueResponse(value)


# TODO: check on existing pin configuration?
def handle_set_pin_value(req):
    # Map pin to the pin map if it is in there, or to
    # an int if raw pin number
    try:
        pin = board_mapping.pin_name_to_pin_number(req.pin)
    except:
        pin = int(req.pin)

    if req.type == "analog":
        # This should be a PWM capable pin. Therefore we do not need to
        # account for the board_mapping.analog_offset. We do need to account for the
        # max pwm_value though.
        capped_value = min(req.value, board_mapping.get_max_pwm_value())
        asyncio.run(set_pin_mode_analog_output(board, pin))
        asyncio.run(asyncio.sleep(0.001))
        asyncio.run(analog_write(board, pin, capped_value))
    if req.type == "digital":
        asyncio.run(board.set_pin_mode_digital_output(pin))
        asyncio.run(asyncio.sleep(0.001))
        asyncio.run(board.digital_write(pin, req.value))
    return SetPinValueResponse(True)


# Initialize the actuators. Each actuator will become a service
# which can be called.
def actuators(loop, board, device):
    servers = []

    if rospy.has_param("/mirte/oled"):
        oleds = rospy.get_param("/mirte/oled")
        oleds = {k: v for k, v in oleds.items() if v["device"] == device}
        oled_id = 0
        for oled in oleds:
            oled_settings = oleds[oled]
            if "name" not in oled_settings:
                oled_settings["name"] = oled
            oled_obj = Oled(
                128, 64, board, oleds[oled], port=oled_id, loop=loop
            )  # get_pin_numbers(oleds[oled]))
            oled_id = oled_id + 1
            servers.append(loop.create_task(oled_obj.start()))

    # TODO: support multiple leds
    if rospy.has_param("/mirte/led"):
        led = rospy.get_param("/mirte/led")
        loop.run_until_complete(
            set_pin_mode_analog_output(board, get_pin_numbers(led)["pin"])
        )
        server = aiorospy.AsyncService(
            "/mirte/set_led_value", SetLEDValue, handle_set_led_value
        )
        servers.append(loop.create_task(server.start()))

    if rospy.has_param("/mirte/motor"):
        motors = rospy.get_param("/mirte/motor")
        motors = {k: v for k, v in motors.items() if v["device"] == device}
        for motor in motors:
            motor_obj = {}
            if motors[motor]["type"] == "ddp":
                motor_obj = DDPMotor(board, motors[motor])
            elif motors[motor]["type"] == "dp":
                motor_obj = DPMotor(board, motors[motor])
            elif motors[motor]["type"] == "pp":
                motor_obj = PPMotor(board, motors[motor])
            else:
                rospy.loginfo("Unsupported motor interface (ddp, dp, or pp)")
            servers.append(loop.create_task(motor_obj.start()))

    if rospy.has_param("/mirte/servo"):
        servos = rospy.get_param("/mirte/servo")
        servos = {k: v for k, v in servos.items() if v["device"] == device}
        for servo in servos:
            servo = Servo(board, servos[servo])
            servers.append(loop.create_task(servo.start()))

    if rospy.has_param("/mirte/neopixel"):
        neopixel = rospy.get_param("/mirte/neopixel")
        servers.append(loop.create_task(Neopixel(board, neopixel).start()))

    if rospy.has_param("/mirte/modules"):
        servers += add_modules(rospy.get_param("/mirte/modules"), device)
    # Set a raw pin value
    server = rospy.Service("/mirte/set_pin_value", SetPinValue, handle_set_pin_value)

    return servers


# Initialize all sensors based on their definition in ROS param
# server. For each sensor a topic is created which publishes
# the data.
def sensors(loop, board, device):
    tasks = []
    max_freq = 30
    if rospy.has_param("/mirte/device/mirte/max_frequency"):
        max_freq = rospy.get_param("/mirte/device/mirte/max_frequency")

    # For now, we need to set the analog scan interval to teh max_freq. When we set
    # this to 0, we do get the updates from telemetrix as fast as possible. In that
    # case the aiorospy creates a latency for the analog sensors (data will be
    # updated with a delay). This also happens when you try to implement this with
    # nest_asyncio icw rospy services.
    # Maybe there is a better solution for this, to make sure that we get the
    # data here asap.
    if board_mapping.get_mcu() == "pico":
        if max_freq <= 1:
            tasks.append(loop.create_task(board.set_scan_delay(1)))
        else:
            try:
                tasks.append(
                    loop.create_task(board.set_scan_delay(int(1000.0 / max_freq)))
                )
            except:
                print("failed scan delay")
                pass
    else:
        if max_freq <= 0:
            tasks.append(loop.create_task(board.set_analog_scan_interval(0)))
        else:
            tasks.append(
                loop.create_task(board.set_analog_scan_interval(int(1000.0 / max_freq)))
            )

    # initialze distance sensors
    if rospy.has_param("/mirte/distance"):
        distance_sensors = rospy.get_param("/mirte/distance")
        distance_sensors = {
            k: v for k, v in distance_sensors.items() if v["device"] == device
        }
        for sensor in distance_sensors:
            distance_sensors[sensor]["max_frequency"] = max_freq
            distance_publisher = rospy.Publisher(
                "/mirte/" + sensor, Range, queue_size=1, latch=True
            )
            monitor = DistanceSensorMonitor(board, distance_sensors[sensor])
            tasks.append(loop.create_task(monitor.start()))

    # Initialize intensity sensors
    if rospy.has_param("/mirte/intensity"):
        intensity_sensors = rospy.get_param("/mirte/intensity")
        intensity_sensors = {
            k: v for k, v in intensity_sensors.items() if v["device"] == device
        }
        for sensor in intensity_sensors:
            intensity_sensors[sensor]["max_frequency"] = max_freq
            if "analog" in get_pin_numbers(intensity_sensors[sensor]):
                monitor = AnalogIntensitySensorMonitor(board, intensity_sensors[sensor])
                tasks.append(loop.create_task(monitor.start()))
            if "digital" in get_pin_numbers(intensity_sensors[sensor]):
                monitor = DigitalIntensitySensorMonitor(
                    board, intensity_sensors[sensor]
                )
                tasks.append(loop.create_task(monitor.start()))

    # Initialize keypad sensors
    if rospy.has_param("/mirte/keypad"):
        keypad_sensors = rospy.get_param("/mirte/keypad")
        keypad_sensors = {
            k: v for k, v in keypad_sensors.items() if v["device"] == device
        }
        for sensor in keypad_sensors:
            keypad_sensors[sensor]["max_frequency"] = max_freq
            monitor = KeypadMonitor(board, keypad_sensors[sensor])
            tasks.append(loop.create_task(monitor.start()))

    # Initialize encoder sensors
    if rospy.has_param("/mirte/encoder"):
        encoder_sensors = rospy.get_param("/mirte/encoder")
        encoder_sensors = {
            k: v for k, v in encoder_sensors.items() if v["device"] == device
        }
        for sensor in encoder_sensors:
            monitor = EncoderSensorMonitor(board, encoder_sensors[sensor])
            tasks.append(loop.create_task(monitor.start()))
            # encoder sensors do not need a max_frequency. They are interrupts on
            # on the mcu side.

    # Get a raw pin value
    # TODO: this still needs to be tested. We are waiting on an implementation of ananlog_read()
    # on the telemetrix side
    rospy.Service("/mirte/get_pin_value", GetPinValue, handle_get_pin_value)
    # server = aiorospy.AsyncService('/mirte/get_pin_value', GetPinValue, handle_get_pin_value)
    # tasks.append(loop.create_task(server.start()))

    return tasks


def add_modules(modules: dict, device: dict) -> []:
    tasks = []
    # pca9685 module:
    module_names = {k for k, v in modules.items() if v["device"] == device}
    for module_name in module_names:
        print(module_name, modules[module_name])
        module = modules[module_name]
        if module["type"].lower() == "pca9685":
            pca_module = PCA9685(board, module_name, module)
            tasks.append(loop.create_task(pca_module.start()))
        if module["type"].lower() == "ina226":
            ina_module = INA226(board, module_name, module)
            tasks.append(loop.create_task(ina_module.start()))
        if module["type"].lower() == "hiwonder_servo":
            servo_module = Hiwonder_Bus(board, module_name, module)
            tasks.append(loop.create_task(servo_module.start()))
        if module["type"].lower() == "mpu9250":
            imu_module = MPU9250.MPU9250(board, module_name, module, board_mapping)
            tasks.append(loop.create_task(imu_module.start()))

    return tasks


def sign(i: float) -> Literal[-1, 0, 1]:
    if i == 0:
        return 0
    return 1 if i / abs(i) > 0 else -1


def scale(val: float, src: Tuple[int, int], dst: Tuple[int, int]) -> float:
    """
    Scale the given value from the scale of src to the scale of dst.
    """
    return ((val - src[0]) / (src[1] - src[0])) * (dst[1] - dst[0]) + dst[0]


class PCA_Servo:
    def __init__(self, servo_name, servo_obj, pca_update_func):
        self.pin = servo_obj["pin"]
        self.name = servo_name
        self.pca_update_func = pca_update_func["set_pwm"]
        self.min_pulse = 544
        if "min_pulse" in servo_obj:
            self.min_pulse = servo_obj["min_pulse"]
        self.min_pulse = int(scale(self.min_pulse, [0, 1_000_000 / 50], [0, 4095]))
        self.max_pulse = 2400
        if "max_pulse" in servo_obj:
            self.max_pulse = servo_obj["max_pulse"]
        self.max_pulse = int(scale(self.max_pulse, [0, 1_000_000 / 50], [0, 4095]))

    async def start(self):
        await self.pca_update_func(self.pin, 0, 0)
        server = rospy.Service(
            "/mirte/set_" + self.name + "_servo_angle",
            SetServoAngle,
            self.set_servo_angle_service,
        )

    async def servo_write(self, angle):
        pwm = int(scale(angle, [0, 180], [self.min_pulse, self.max_pulse]))
        await self.pca_update_func(self.pin, pwm, 0)

    def set_servo_angle_service(self, req):
        asyncio.run(self.servo_write(req.angle))
        return SetServoAngleResponse(True)


class PCA_Motor(Motor):
    def __init__(self, motor_name, motor_obj, pca_update_func):
        self.pca_update_func = pca_update_func["set_pwm"]
        self.pin_A = motor_obj["pin_A"]
        self.pin_B = motor_obj["pin_B"]
        self.name = motor_name
        self.prev_motor_speed = 0
        self.inverted = motor_obj["inverted"] if "inverted" in motor_obj else False

    async def start(self):
        await Motor.start(self)
        await self.init_motors()

    async def init_motors(self):
        await self.pca_update_func(self.pin_A, 0)
        await self.pca_update_func(self.pin_B, 0)

    async def set_speed(self, speed):
        if self.inverted:
            speed = -speed
        if self.prev_motor_speed != speed:
            change_dir = sign(self.prev_motor_speed) != sign(speed)
            if (
                change_dir
            ):  # stop the motor before sending out new values if changing direction
                await self.pca_update_func(self.pin_A, 0)
                await self.pca_update_func(self.pin_B, 0)
            if speed == 0:
                await self.pca_update_func(self.pin_A, 0)
                await self.pca_update_func(self.pin_B, 0)
            elif speed > 0:
                await self.pca_update_func(
                    self.pin_A, int(min(speed, 100) / 100.0 * 4095)
                )
            elif speed < 0:
                await self.pca_update_func(
                    self.pin_B, int(min(-speed, 100) / 100.0 * 4095)
                )
            self.prev_motor_speed = speed


class PCA9685:
    def __init__(self, board, module_name, module):
        self.name = module_name
        self.module = module
        self.board = board
        self.motors = {}
        self.servos = {}

    async def start(self):
        # setup i2c, check with oled to not init twice
        if board_mapping.get_mcu() == "pico":
            if "connector" in self.module:
                pins = board_mapping.connector_to_pins(self.module["connector"])
            else:
                pins = self.module["pins"]
            pin_numbers = {}
            for item in pins:
                pin_numbers[item] = board_mapping.pin_name_to_pin_number(pins[item])
            self.i2c_port = board_mapping.get_I2C_port(pin_numbers["sda"])
            try:
                await self.board.set_pin_mode_i2c(
                    i2c_port=self.i2c_port,
                    sda_gpio=pin_numbers["sda"],
                    scl_gpio=pin_numbers["scl"],
                )
            except Exception as e:
                pass
        frequency = 200
        if "frequency" in self.module:
            frequency = self.module["frequency"]
        if "servos" in self.module:  # servos need to run at 50Hz
            frequency = 50
        id = 0x40  # default pca id
        if "id" in self.module:
            id = self.module["id"]
        # setup pca
        self.write_pca = await self.board.modules.add_pca9685(
            self.i2c_port, id, frequency
        )
        # create motors
        if "motors" in self.module:
            for motor_name in self.module["motors"]:
                self.motors[motor_name] = PCA_Motor(
                    motor_name, self.module["motors"][motor_name], self.write_pca
                )
                await self.motors[motor_name].start()
        # create servos
        if "servos" in self.module:
            for servo_name in self.module["servos"]:
                servo_obj = self.module["servos"][servo_name]
                self.servos[servo_name] = PCA_Servo(
                    servo_name, servo_obj, self.write_pca
                )
                await self.servos[servo_name].start()


class INA226:
    def __init__(self, board, module_name, module):
        self.name = module_name
        self.module = module
        self.board = board
        self.min_voltage = module["min_voltage"] if "min_voltage" in module else -1
        self.max_voltage = module["max_voltage"] if "max_voltage" in module else -1
        self.max_current = module["max_current"] if "max_current" in module else -1
        self.turn_off_pin = (
            board_mapping.pin_name_to_pin_number(module["turn_off_pin"])
            if "turn_off_pin" in module
            else -1
        )
        self.turn_off_pin_value = (
            module["turn_off_pin_value"] if "turn_off_pin_value" in module else True
        )  # what should be the output when the relay should be opened.
        self.turn_off_time = (
            module["turn_off_time"] if "turn_off_time" in module else 30
        )  # time to wait for computer to shut down
        self.enable_turn_off = False  # require at least a single message with a real value before arming the turn off system
        get_obj_value(
            self, module, "power_low_time", 5
        )  # how long(s) for the voltage to be below the trigger voltage before triggering shutting down
        self.shutdown_triggered = False
        self.turn_off_trigger_start_time = -1
        self.ina_publisher = rospy.Publisher(
            "/mirte/power/" + module_name, BatteryState, queue_size=1
        )
        self.ina_publisher_used = rospy.Publisher(
            f"/mirte/power/{module_name}/used", Int32, queue_size=1
        )
        self.used_energy = 0  # mah
        self.last_used_calc = time.time()
        server = rospy.Service("/mirte/shutdown", SetBool, self.shutdown_service)

        self.last_low_voltage = -1

        self.switch_pin = (
            board_mapping.pin_name_to_pin_number(module["switch_pin"])
            if "switch_pin" in module
            else -1
        )
        get_obj_value(self, module, "switch_off_value", True)
        get_obj_value(self, module, "switch_pull", 0)
        get_obj_value(self, module, "switch_time", 5)

    async def start(self):
        # setup i2c, check with oled to not init twice
        if board_mapping.get_mcu() == "pico":
            if "connector" in self.module:
                pins = board_mapping.connector_to_pins(self.module["connector"])
            else:
                # TODO: no other boards have support for this yet
                pins = self.module["pins"]
            pin_numbers = {}
            for item in pins:
                pin_numbers[item] = board_mapping.pin_name_to_pin_number(pins[item])
            self.i2c_port = board_mapping.get_I2C_port(pin_numbers["sda"])
            try:
                await self.board.set_pin_mode_i2c(
                    i2c_port=self.i2c_port,
                    sda_gpio=pin_numbers["sda"],
                    scl_gpio=pin_numbers["scl"],
                )
            except Exception as e:
                pass
        id = 0x40  # default ina id
        if "id" in self.module:
            id = self.module["id"]

        await self.board.sensors.add_ina226(self.i2c_port, self.callback, id)
        if self.turn_off_pin != -1:
            self.trigger_shutdown_relay = await self.board.modules.add_shutdown_relay(
                self.turn_off_pin,
                not not self.turn_off_pin_value,
                self.turn_off_time
                + 10,  # add 10s to the shutdown time for the 10s shutdown command wait time
            )
        if self.switch_pin > 0:
            if self.switch_pull == 1:
                await self.board.set_pin_mode_digital_input_pullup(
                    self.switch_pin,
                    callback=self.switch_data,
                )
            elif self.switch_pull == -1:
                await self.board.set_pin_mode_digital_input_pull_down(
                    self.switch_pin,
                    callback=self.switch_data,
                )
            else:
                await self.board.set_pin_mode_digital_input(
                    self.switch_pin,
                    callback=self.switch_data,
                )
            self.switch_armed = False
            self.switch_trigger_start_time = -1
            self.switch_val = -1

            rospy.Timer(rospy.Duration(0.5), self.check_switch_sync)

        if "gpiod" in sys.modules:
            await self.setup_percentage_led()

    async def setup_percentage_led(self):
        if "percentage_led_chip" in self.module:
            return
        if not "percentage_led_line" in self.module:
            return

        chip = gpiod.chip(self.module["percentage_led_chip"])
        line = self.module["percentage_led_line"]
        led = chip.get_line(line)

        config = gpiod.line_request()
        config.consumer = "ROS percentage"
        config.request_type = gpiod.line_request.DIRECTION_OUTPUT

        led.request(config)
        self.percentage_led = led
        rospy.Timer(rospy.Duration(0.5), self.check_percentage_sync)

    async def switch_data(self, data):
        self.switch_val = bool(data[2])
        await self.check_switch()

    def check_switch_sync(self, event=None):
        asyncio.run(self.check_switch())

    def check_percentage_sync(self, event=None):
        asyncio.run(self.show_percentage())

    async def show_percentage(self):
        # show the SOC by blinking the led. Shorter pulse -> lower SOC
        # cycle time of 5s
        time_sec = time.time() % 5
        percentage = self.calculate_percentage() / 20
        if time_sec > percentage:
            # turn off the led
            self.percentage_led.set_value(0)

        else:
            # turn on the led
            self.percentage_led.set_value(1)

    async def check_switch(self):
        if self.switch_val == -1:
            return
        if not self.switch_armed:
            if self.switch_val != self.switch_off_value:
                rospy.logwarn("Shutdown switch armed")
                self.switch_armed = True
            return

        if self.switch_val == self.switch_off_value:
            if self.switch_trigger_start_time == -1:
                self.switch_trigger_start_time = time.time()
                rospy.logwarn(
                    f"Switch turned off, shutting down in {self.switch_time}s if not restored."
                )

                # Send a message to all users that the switch is off and possibly shutting down
                subprocess.run(
                    f"wall 'Switch turned off, shutting down in {self.switch_time}s if not restored.'",
                    shell=True,
                )
        else:
            self.switch_trigger_start_time = -1

        if (
            self.switch_trigger_start_time != -1
            and time.time() - self.switch_trigger_start_time > self.switch_time
        ):
            await self.shutdown_robot()

    def calculate_percentage(self):
        soc_levels = {  # single cell voltages
            3.27: 0,
            3.61: 5,
            3.69: 10,
            3.71: 15,
            3.73: 20,
            3.75: 25,
            3.77: 30,
            3.79: 35,
            3.80: 40,
            3.82: 45,
            3.84: 50,
            3.85: 55,
            3.87: 60,
            3.91: 65,
            3.95: 70,
            3.98: 75,
            4.02: 80,
            4.08: 85,
            4.11: 90,
            4.15: 95,
            4.20: 100,
        }
        voltage = self.voltage / 3  # 3s lipo
        percentage = None
        for level, percent in soc_levels.items():
            if voltage >= level:  # take the highest soc that is lower than voltage
                percentage = percent
        if percentage is None:
            percentage = 1
        return percentage

    async def callback(self, data):
        # TODO: move this decoding to the library
        ints = list(map(lambda i: i.to_bytes(1, "big"), data))
        bytes_obj = b"".join(ints)
        vals = list(struct.unpack("<2f", bytes_obj))
        self.voltage = vals[0]
        self.current = vals[1]
        if (
            self.min_voltage != -1
            and self.voltage < self.min_voltage
            and self.last_low_voltage != self.voltage
        ):
            rospy.logwarn("Low voltage: %f", self.voltage)
            self.last_low_voltage = self.voltage
        if self.max_voltage != -1 and self.voltage > self.max_voltage:
            rospy.logwarn("High voltage: %f", self.voltage)
        if self.max_current != -1 and self.current > self.max_current:
            rospy.logwarn("High current: %f", self.current)
        bs = BatteryState()
        bs.header = Header()
        bs.header.stamp = rospy.Time.now()
        bs.voltage = self.voltage
        bs.current = self.current
        bs.temperature = math.nan
        bs.charge = math.nan
        bs.capacity = math.nan
        bs.design_capacity = math.nan
        bs.percentage = self.calculate_percentage() / 100
        global current_soc
        current_soc = int(self.calculate_percentage())
        # uint8   power_supply_health     # The battery health metric. Values defined above
        # uint8   power_supply_technology # The battery chemistry. Values defined above
        bs.power_supply_status = 0  # uint8 POWER_SUPPLY_STATUS_UNKNOWN = 0
        bs.power_supply_health = 0  # uint8 POWER_SUPPLY_HEALTH_UNKNOWN = 0
        bs.power_supply_technology = 3  # uint8 POWER_SUPPLY_TECHNOLOGY_LIPO = 3
        self.ina_publisher.publish(bs)
        self.integrate_usage()
        await self.turn_off_cb()

    def integrate_usage(self):
        time_diff = time.time() - self.last_used_calc  # seconds
        self.last_used_calc = time.time()
        used_mA_sec = time_diff * self.current * 1000
        used_mAh = used_mA_sec / 3600
        self.used_energy += used_mAh

        self.ina_publisher_used.publish(int(self.used_energy))

    async def turn_off_cb(self):
        if self.turn_off_pin == -1:
            # Dont turn off when this feature is disabled
            return

        # make sure that there are real values coming from the ina226 module
        if not self.enable_turn_off:
            if self.voltage > 6 and self.current > 0.3:
                self.enable_turn_off = True
                rospy.logwarn("Shutdown relay armed")
            return

        # at start dip of too low voltage, start timer, when longer than 5s below trigger voltage, then shut down
        # this makes sure that a short dip (motor start) does not trigger it
        if self.min_voltage != -1 and self.voltage < self.min_voltage:
            if self.turn_off_trigger_start_time < 0:
                self.turn_off_trigger_start_time = time.time()
                # Send a message to all users that the voltage is low and possibly shutting down
                subprocess.run(
                    f"wall 'Low voltage, shutting down in {self.power_low_time}s if not restored.'"
                )
            rospy.logwarn(
                "Low voltage, %ss till shutdown.",
                self.power_low_time - (time.time() - self.turn_off_trigger_start_time),
            )
        else:
            self.turn_off_trigger_start_time = -1

        if (
            self.turn_off_trigger_start_time != -1
            and time.time() - self.turn_off_trigger_start_time > self.power_low_time
        ):
            subprocess.run("wall 'Low voltage, shutting down now.'", shell=True)
            await self.shutdown_robot()

    async def shutdown_robot(self):
        if not self.shutdown_triggered:
            subprocess.run(
                # wall does not show up on vscode terminal
                f"bash -c \"wall 'Shutting down.'\"",
                shell=True,
            )

            rospy.logerr("Triggering shutdown, shutting down in 10s")
            # This will make the pico unresponsive after the delay, so ping errors are expected. Need to restart telemetrix to continue.
            if hasattr(self, "trigger_shutdown_relay"):
                await self.trigger_shutdown_relay()
            self.shutdown_triggered = True
            subprocess.run("sleep 10; sudo shutdown now", shell=True)

    def shutdown_service(self, req):
        # also called from systemd shutdown service
        asyncio.run(self.shutdown_robot())
        return SetBoolResponse(True, "")


class Hiwonder_Servo:
    def __init__(self, servo_name, servo_obj, bus):
        self.id = servo_obj["id"]
        self.name = servo_name
        self.bus = bus

        # angles for the servo, differs probably per servo
        self.min_angle_out = 0
        self.max_angle_out = 24000  # centidegrees
        self.home_out = 1000  # centidegrees, will be mapped to ros angle 0 rad.
        for name in ["home_out", "min_angle_out", "max_angle_out"]:
            if name in servo_obj:
                setattr(self, name, servo_obj[name])
        if "home_out" not in servo_obj:  # set it to the lowest value
            self.home_out = self.min_angle_out
        if self.home_out < self.min_angle_out:
            raise Exception(
                f"Home_out{self.home_out} should be more than min_angle_out{self.min_angle_out}"
            )
        if self.home_out > self.max_angle_out:
            raise Exception(
                f"Home_out{self.home_out} should be less than max_angle_out{self.max_angle_out}"
            )
        diff_min = self.min_angle_out - self.home_out  # centidegrees
        diff_min = diff_min / 100  # degrees
        self.min_angle_in = math.radians(diff_min)
        diff_max = self.max_angle_out - self.home_out  # centidegrees
        diff_max = diff_max / 100  # degrees
        self.max_angle_in = math.radians(diff_max)
        get_obj_value(self, servo_obj, "invert", False)
        if (
            self.invert
        ):  # swap min and max angle values, home should stay at the same spot.
            t = self.min_angle_in
            self.min_angle_in = -self.max_angle_in
            self.max_angle_in = -t
        print("rad range", self.name, [self.min_angle_in, self.max_angle_in])

    async def start(self):
        server = rospy.Service(
            "/mirte/set_" + self.name + "_servo_angle",
            SetServoAngle,
            self.set_servo_angle_service,
        )
        rospy.Service(
            "/mirte/set_" + self.name + "_servo_enable",
            SetBool,
            self.set_servo_enabled_service,
        )
        self.publisher = rospy.Publisher(
            f"/mirte/servos/{self.name}/position",
            ServoPosition,
            queue_size=1,
            latch=True,
        )

    def set_servo_enabled_service(self, req):
        asyncio.run(self.bus.set_enabled(self.id, req.data))
        return SetBoolResponse(True, "enabled" if req.data else "disabled")

    async def servo_write(self, angle):
        angle = scale(
            angle,
            [self.min_angle_in, self.max_angle_in],
            # when inverted, xxx_angle_IN is swapped, so also swap xxx_angle_OUT
            (
                [self.min_angle_out, self.max_angle_out]
                if not self.invert
                else [self.max_angle_out, self.min_angle_out]
            ),
        )
        angle = int(max(self.min_angle_out, min(angle, self.max_angle_out)))  # clamp
        # print("clamp", angle)
        await self.bus.set_single_servo(self.id, angle, 0)

    def set_servo_angle_service(self, req):
        if req.angle > self.max_angle_in or req.angle < self.min_angle_in:
            return SetServoAngleResponse(False)
        asyncio.run(self.servo_write(req.angle))
        return SetServoAngleResponse(True)

    def callback(self, data):
        angle = float(
            scale(
                data["angle"],
                (
                    [self.min_angle_out, self.max_angle_out]
                    if not self.invert
                    else [self.max_angle_out, self.min_angle_out]
                ),
                [self.min_angle_in, self.max_angle_in],
            )
        )
        header = Header()
        header.stamp = rospy.Time.now()

        position = ServoPosition()
        position.header = header
        position.raw = data["angle"]
        position.angle = angle

        self.publisher.publish(position)


class Hiwonder_Bus:
    def __init__(self, board, module_name, module):
        self.name = module_name
        self.module = module
        self.board = board
        self.servos = {}

    async def start(self):
        uart = self.module["uart"]
        if uart != 0 and uart != 1:
            return
        rx = self.module["rx_pin"]
        tx = self.module["tx_pin"]
        ids = []
        if "servos" in self.module:
            for servo_name in self.module["servos"]:
                servo_obj = self.module["servos"][servo_name]
                servo = Hiwonder_Servo(servo_name, servo_obj, self)
                ids.append(servo.id)
                await servo.start()
                self.servos[servo_name] = servo
                self.servos[servo.id] = servo

        updaters = await self.board.modules.add_hiwonder_servo(
            uart, rx, tx, ids, self.callback
        )
        self.set_single_servo = updaters["set_single_servo"]
        self.set_multiple_servos = updaters["set_multiple_servos"]
        self.set_enabled = updaters["set_enabled"]
        self.set_enabled_all = updaters["set_enabled_all"]
        rospy.Service(
            "/mirte/set_" + self.name + "_all_servos_enable",
            SetBool,
            self.set_all_servos_enabled,
        )

        # TODO: add service to update multiple servos

    def set_all_servos_enabled(self, req):
        asyncio.run(self.set_enabled_all(req.data))
        return SetBoolResponse(True, "enabled" if req.data else "disabled")

    async def callback(self, data):
        try:
            for servo_update in data:
                sid = servo_update["id"]
                if sid in self.servos:
                    self.servos[sid].callback(servo_update)
        except Exception as e:
            print("hiwonder servo callback err:")
            print(e)


# Shutdown procedure
closing = False


async def shutdown(loop, board):
    global closing

    # We need to check if this closing is not already
    # running by an escalated signal.
    if not closing:
        closing = True
        await board.shutdown()

        # Stop the asyncio loop
        loop.stop()
        print("Telemetrix shutdown nicely")
        rospy.signal_shutdown(0)
        time.sleep(1)
        exit(0)


# check any ttyACM or ttyUSB devices
# if none found, sleep for 5 seconds and try again
# if still none found, exit the program
def check_tty():
    while True:
        out = subprocess.getoutput("ls /dev/ttyACM* /dev/ttyUSB* 2> /dev/null")
        if len(out) > 0:
            return
        rospy.logwarn("No ttyACM/ttyUSB device, trying again in 5s")
        time.sleep(5)


if __name__ == "__main__":
    # Initialize the ROS node as anonymous since there
    # should only be one instance running.
    rospy.init_node("mirte_telemetrix", anonymous=False)
    check_tty()

    loop = asyncio.new_event_loop()

    # Initialize the telemetrix board
    if board_mapping.get_mcu() == "pico":
        board = tmx_pico_aio.TmxPicoAio(
            allow_i2c_errors=True, loop=loop, autostart=False, hard_shutdown=True
        )
        loop.run_until_complete(board.start_aio())
    else:
        board = telemetrix_aio.TelemetrixAIO(loop=loop)

    # Catch signals to exit properly
    # We need to do it this way instead of usgin the try/catch
    # as in the telemetrix examples
    signals = (signal.SIGHUP, signal.SIGTERM, signal.SIGINT)
    for s in signals:
        l = lambda loop=loop, board=board: asyncio.ensure_future(shutdown(loop, board))
        loop.add_signal_handler(s, l)

    # Escalate siging to this process in order to shutdown nicely
    # This is needed when only this process is killed (eg. rosnode kill)
    # This cannot be done by calling shutdown() because that is
    # a different thread without asyncio loop.
    l = lambda pid=os.getpid(), sig=signal.SIGINT: os.kill(pid, sig)
    rospy.on_shutdown(l)

    # Start all tasks for sensors and actuators
    device = "mirte"
    sensor_tasks = sensors(loop, board, device)
    actuator_tasks = actuators(loop, board, device)
    all_tasks = sensor_tasks + actuator_tasks
    for task in all_tasks:
        loop.run_until_complete(task)

    # Is equivalent to rospy.spin() in a sense that this
    # will just keep the node running only in a asyncio
    # way.
    loop.run_forever()
