# Map pcb names to pins, based on version number


def pin_name_to_pin_number(pin):  # pico has no mapping
    pin = str(pin)
    if pin.startswith("GP"):
        pin = pin[2:]
    if pin.isdigit():
        return int(pin)

    raise RuntimeError(f"Unknown conversion from pin {pin} to an IO number.")


def get_analog_offset():
    return 26


def get_adc_bits():
    # NOTE: the pico itself has 12 bits, but micropython will upgrade to 16
    # no clue why 14 works
    return 14


def get_max_pwm_value():
    return 100


def get_mcu():
    return "pico"


i2c_port0_sda_pins = [0, 4, 8, 12, 20, 16]
i2c_port1_sda_pins = [2, 6, 10, 14, 26, 18]


def get_I2C_port(sda):
    if sda in i2c_port0_sda_pins:
        return 0
    elif sda in i2c_port1_sda_pins:
        return 1
    else:
        raise RuntimeError(f"No i2c port for SDA pin {sda}.")
