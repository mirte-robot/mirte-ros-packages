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


# Import the right Telemetrix AIO
devices = rospy.get_param("/mirte/device")


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


# Import ROS message types
from std_msgs.msg import Header, Int32
from sensor_msgs.msg import Range
from mirte_msgs.msg import *

# Import ROS services
from mirte_msgs.srv import *

from bitstring import BitArray
import textwrap

from PIL import Image, ImageDraw, ImageFont

# font = ImageFont.truetype("/usr/share/fonts/truetype/terminus.ttf", 12)
font = ImageFont.truetype("/usr/share/fonts/dejavu/DejaVuSans.ttf", 12)

from adafruit_ssd1306 import _SSD1306
from concurrent.futures import ThreadPoolExecutor

executor = ThreadPoolExecutor(10)


import mappings.default
import mappings.nanoatmega328
import mappings.pico
import mappings.stm32
import mappings.pcb

board_mapping = mappings.default

devices = rospy.get_param("/mirte/device")

if devices["mirte"]["type"] == "pcb":
    board_mapping = mappings.pcb
    if "version" in devices["mirte"]:
        if "mcu" in devices["mirte"]:  # pcb 0.4 allows for nano or stm32
            board_mapping.set_version(
                devices["mirte"]["version"], devices["mirte"]["mcu"]
            )
        else:
            board_mapping.set_version(devices["mirte"]["version"])

if devices["mirte"]["type"] == "breadboard":
    if "mcu" in devices["mirte"]:
        if devices["mirte"]["mcu"] == "stm32":
            board_mapping = mappings.stm32
        elif (
            "nano"
            in devices["mirte"][
                "mcu"
            ]  # will trigger for nano, nano_old and nanoatmega328
            or devices["mirte"]["mcu"] == "uno"  # uno has the same pinout
        ):
            board_mapping = mappings.nanoatmega328
        elif devices["mirte"]["mcu"] == "pico":
            board_mapping = mappings.pico
        else:
            board_mapping = mappings.default


def get_pin_numbers(component):
    devices = rospy.get_param("/mirte/device")
    device = devices[component["device"]]
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

    def get_data(self, req):
        return GetEncoderResponse(self.last_publish_value.value)

    async def start(self):
        if board_mapping.get_mcu() == "pico":
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
        self.speed_count = self.speed_count + 1
        encoder = Encoder()
        encoder.header = self.get_header()
        encoder.value = data[2]
        await self.publish(encoder)


class Servo:
    def __init__(self, board, servo_obj):
        self.board = board
        self.pins = get_pin_numbers(servo_obj)
        self.name = servo_obj["name"]

    async def stop(self):
        await board.detach_servo(self.pins["pin"])

    async def start(self):
        await board.set_pin_mode_servo(self.pins["pin"])
        server = rospy.Service(
            "/mirte/set_" + self.name + "_servo_angle",
            SetServoAngle,
            self.set_servo_angle_service,
        )

    def set_servo_angle_service(self, req):
        asyncio.run(board.servo_write(self.pins["pin"], req.angle))
        return SetServoAngleResponse(True)


# TODO: create motor classs and inherit from that one
class PWMMotor:
    def __init__(self, board, motor_obj):
        self.board = board
        self.pins = get_pin_numbers(motor_obj)
        self.name = motor_obj["name"]
        self.prev_motor_speed = 0
        self.initialized = False

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

    # Ideally one would initialize the pins in the constructor. But
    # since some mcu's have some voltage on pins when they are not
    # initialized yet icw some motor controllers that use the
    # difference between the pins to determine speed and direction
    # the motor will briefly move when initializing. This is unwanted.
    # When setting this on the mcu itself the this will be done fast
    # enough. But using telemetrix is a bit too slow fow this. We
    # therefore set the pin type on first move, and do this in a way
    # where is creates a movement in teh same direction.
    async def init_motors(self, speed):
        if not self.initialized:
            if speed > 0:
                await self.board.set_pin_mode_digital_output(self.pins["1a"])
                await set_pin_mode_analog_output(self.board, self.pins["1b"])
            if speed < 0:
                await set_pin_mode_analog_output(self.board, self.pins["1b"])
                await self.board.set_pin_mode_digital_output(self.pins["1a"])
            self.initialized = True

    async def set_speed(self, speed):
        if self.prev_motor_speed != speed:
            if speed == 0:
                await self.board.digital_write(self.pins["1a"], 0)
                await analog_write(self.board, self.pins["1b"], 0)
            elif speed > 0:
                await self.init_motors(speed)
                await self.board.digital_write(self.pins["1a"], 0)
                await analog_write(
                    self.board,
                    self.pins["1b"],
                    int(min(speed, 100) / 100.0 * board_mapping.get_max_pwm_value()),
                )
            elif speed < 0:
                await self.init_motors(speed)
                await self.board.digital_write(self.pins["1a"], 1)
                await analog_write(
                    self.board,
                    self.pins["1b"],
                    int(
                        board_mapping.get_max_pwm_value()
                        - min(abs(speed), 100)
                        / 100.0
                        * board_mapping.get_max_pwm_value()
                    ),
                )
            self.prev_motor_speed = speed


class L298NMotor:
    def __init__(self, board, pins):
        self.board = board
        self.pins = pins
        self.prev_motor_speed = 0
        self.loop = asyncio.get_event_loop()
        self.initialized = False

    async def init_motors(self):
        if not self.initialized:
            await set_pin_mode_analog_output(self.board, self.pins["en"])
            await self.board.set_pin_mode_digital_output(self.pins["in1"])
            await self.board.set_pin_mode_digital_output(self.pins["in2"])
            self.initialized = True

    async def set_speed(self, speed):
        # Make sure to set first set teh low pin. In this case the H-bridge
        # will never have two high pins.
        if self.prev_motor_speed != speed:
            await self.init_motors()
            if speed >= 0:
                await self.board.digital_write(self.pins["in1"], 0)
                await self.board.digital_write(self.pins["in2"], 1)
                await analog_write(
                    self.board,
                    self.pins["en"],
                    int(min(speed, 100) / 100.0 * board_mapping.get_max_pwm_value()),
                )
            elif speed < 0:
                await self.board.digital_write(self.pins["in2"], 0)
                await self.board.digital_write(self.pins["in1"], 1)
                await analog_write(
                    self.board,
                    self.pins["en"],
                    int(
                        min(abs(speed), 100) / 100.0 * board_mapping.get_max_pwm_value()
                    ),
                )
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
        addr=0x3C,
        external_vcc=False,
        reset=None,
    ):
        self.board = board
        self.oled_obj = oled_obj
        self.addr = addr
        self.temp = bytearray(2)
        self.i2c_port = port
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
            asyncio.run(
                self.board.set_pin_mode_i2c(
                    i2c_port=self.i2c_port,
                    sda_gpio=pin_numbers["sda"],
                    scl_gpio=pin_numbers["scl"],
                )
            )
        else:
            asyncio.run(self.board.set_pin_mode_i2c(i2c_port=self.i2c_port))
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

    # TODO: make faster with asyncio
    def set_oled_image_service(self, req):
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
            self.show()

        if req.type == "image":
            self.show_png(
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
                self.show_png(folder + req.value + "_" + str(i) + ".png")

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
        asyncio.run(self.board.i2c_write(60, self.temp, i2c_port=self.i2c_port))

    def write_framebuf(self):
        for i in range(
            64
        ):  # TODO: can we have higher i2c buffer (limited by firmata 64 bits and wire 32 bits, so actually 16 bits since we need 1 bit)
            buf = self.buffer[i * 16 : (i + 1) * 16 + 1]
            buf[0] = 0x40
            asyncio.run(self.board.i2c_write(60, buf, i2c_port=self.i2c_port))

    def show_png(self, file):
        image_file = Image.open(file)  # open color image
        image_file = image_file.convert("1", dither=Image.NONE)
        self.image(image_file)
        self.show()


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

    while not pin in pin_values:
        time.sleep(0.00001)

    value = pin_values[pin]
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
            oled_obj = Oled(
                128, 64, board, oleds[oled], port=oled_id
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
            if motors[motor]["type"] == "l298n":
                motor_obj = L298NMotor(board, motors[motor])
            else:
                motor_obj = PWMMotor(board, motors[motor])
            servers.append(loop.create_task(motor_obj.start()))

    if rospy.has_param("/mirte/servo"):
        servos = rospy.get_param("/mirte/servo")
        servos = {k: v for k, v in servos.items() if v["device"] == device}
        for servo in servos:
            servo = Servo(board, servos[servo])
            servers.append(loop.create_task(servo.start()))

    # Set a raw pin value
    server = rospy.Service("/mirte/set_pin_value", SetPinValue, handle_set_pin_value)

    return servers


# Initialize all sensors based on their definition in ROS param
# server. For each sensor a topic is created which publishes
# the data.
def sensors(loop, board, device):
    tasks = []
    max_freq = 10
    if rospy.has_param("/mirte/device/mirte/max_frequency"):
        max_freq = rospy.get_param("/mirte/device/mirte/max_frequency")

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
                pass
    else:
        if max_freq <= 0:
            tasks.append(loop.create_task(board.set_analog_scan_interval(0)))
        else:
            tasks.append(
                loop.create_task(board.set_analog_scan_interval(int(1000.0 / max_freq)))
            )

    return tasks


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


if __name__ == "__main__":
    loop = asyncio.get_event_loop()

    # Initialize the telemetrix board
    if board_mapping.get_mcu() == "pico":
        board = tmx_pico_aio.TmxPicoAio(allow_i2c_errors=True)
    else:
        board = telemetrix_aio.TelemetrixAIO()

    # Catch signals to exit properly
    # We need to do it this way instead of usgin the try/catch
    # as in the telemetrix examples
    signals = (signal.SIGHUP, signal.SIGTERM, signal.SIGINT)
    for s in signals:
        l = lambda loop=loop, board=board: asyncio.ensure_future(shutdown(loop, board))
        loop.add_signal_handler(s, l)

    # Initialize the ROS node as anonymous since there
    # should only be one instnace running.
    rospy.init_node("mirte_telemetrix", anonymous=False)

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
