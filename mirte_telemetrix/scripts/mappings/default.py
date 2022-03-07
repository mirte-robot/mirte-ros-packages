# No mapping, just assume the config is correct


def connector_to_pins(connector):
    raise RuntimeError(
        f"Unknown conversion from connector {connector} to pins. Set a board type and version."
    )


def pin_name_to_pin_number(pin):
    if int(pin) == pin:
        return pin
    raise RuntimeError(f"Unknown conversion from pin {pin} to an IO number")


def get_mcu():
    return "none"


def get_I2C_port(sda):
    return 0


def get_analog_offset():
    return 0


def get_adc_bits():
    return 12


def get_max_pwm_value():
    return 255
