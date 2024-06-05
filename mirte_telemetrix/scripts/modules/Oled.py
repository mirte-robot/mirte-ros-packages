import textwrap

from PIL import Image, ImageDraw, ImageFont

# Currently loading the PIL default font, which is
# monospace, so works with python textwrap
font = ImageFont.load_default()

from adafruit_ssd1306 import _SSD1306
import time

import rospy
from mirte_msgs.srv import SetOLEDImage, SetOLEDImageRequest, SetOLEDImageResponse
import subprocess
import asyncio
# Extended adafruit _SSD1306

class Oled_interface:
   
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
            # print("asdf", self.global_data)
            text += f"\nSOC: {self.global_data['current_soc']}%"
        if len(text) > 0:
            await self.set_oled_image_service_async(
                SetOLEDImageRequest(type="text", value=text)
            )
    async def set_oled_image_service_async(self, req):
        if req.type =="text":
            await self.set_oled_text_async(req.value.replace('\\n', '\n'))
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

    async def set_oled_text_async(self, text):
        print("todo setoled text async")
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
    async def start(self):
        print("todo start")
    def show_default(self, event=None):
        if not self.default_image:
            return
        try:
            # the ros service is started on a different thread than the asyncio loop
            # When using the normal loop.run_until_complete() function, both threads join in and the oled communication will get broken faster
            future = asyncio.run_coroutine_threadsafe(
                self.show_default_async(), self.loop
            )
        except Exception as e:
            print(e)
    async def show_png(self, file):
        print("todo show png")
# class Oled_module:


class Oled_module(Oled_interface):
    def __init__(
            self,
            board,
            module_name,
            module,
            board_mapping,
            loop,
            global_data, # obj with some shared data, like SOC
            addr=0x3C, # not yet possible with modules
            external_vcc=False,
            reset=None,
        ):
        self.board = board
        self.oled_obj = module
        self.addr = addr
        self.failed = False
        self.loop = loop
        self.global_data = global_data
        width = 128 # hardcoded in the pico code
        height = 64
        self.init_awaits = []
        # Add an extra byte to the data buffer to hold an I2C data/command byte
        # to use hardware-compatible I2C transactions.  A memoryview of the
        # buffer is used to mask this byte from the framebuffer operations
        # (without a major memory hit as memoryview doesn't copy to a separate
        # buffer).
        if board_mapping.get_mcu() == "pico":
            if "connector" in self.oled_obj:
                pins = board_mapping.connector_to_pins(self.oled_obj["connector"])
            else:
                pins = self.oled_obj["pins"]
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
        print("done init oledmod")
        
    async def start(self):
        for ev in self.init_awaits:
            try:  # catch set_pin_mode_i2c already for this port
                z = await ev
                print("z",z)
            except Exception as e:
                print(e)
                pass
        await asyncio.sleep(1)
        print("add")
        self.update_functions = await self.board.modules.add_tmx_ssd1306(self.i2c_port)
        await asyncio.sleep(1)
        print("upd func")
        await self.update_functions["send_text"]("Starting ROS...")
        self.default_image = True
        rospy.Timer(rospy.Duration(10), self.show_default)
        await self.show_default_async()



    async def set_oled_text_async(self, text):
        print("udpate text")
        await self.update_functions["send_text"](text.replace('\\n', '\n'))
    def show(self):
        print("unused?")
    async def show_async(self):
        print("todo show async")
    async def show_png(self, file):
        print("todo show png")


class Oled(_SSD1306, Oled_interface):
    def __init__(
        self,
        board,
        module_name,
        module,
        board_mapping,
        port,
        loop,
        global_data, # obj with some shared data, like SOC
        addr=0x3C,
        external_vcc=False,
        reset=None,
    ):
        self.board = board
        self.oled_obj = module
        self.addr = addr
        self.temp = bytearray(2)
        self.i2c_port = port
        self.failed = False
        self.loop = loop
        self.init_awaits = []
        self.write_commands = []
        self.global_data = global_data
        width = 128 # hardcoded in the pico code
        height = 64
        # Add an extra byte to the data buffer to hold an I2C data/command byte
        # to use hardware-compatible I2C transactions.  A memoryview of the
        # buffer is used to mask this byte from the framebuffer operations
        # (without a major memory hit as memoryview doesn't copy to a separate
        # buffer).
        self.buffer = bytearray(((height // 8) * width) + 1)
        # self.buffer = bytearray(16)
        # self.buffer[0] = 0x40  # Set first byte of data buffer to Co=0, D/C=1
        if board_mapping.get_mcu() == "pico":
            if "connector" in self.oled_obj:
                pins = board_mapping.connector_to_pins(self.oled_obj["connector"])
            else:
                pins = self.oled_obj["pins"]
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
            "mirte/set_" + self.oled_obj["name"] + "_image",
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
        except Exception as e:
            print(e)

    
    async def set_oled_text_async(self, text):
            text = text.replace("\\n", "\n")
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

