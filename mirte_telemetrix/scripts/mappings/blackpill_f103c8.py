# Map to convert from STM32 to pin numbers
stm32_map = {
    "B9": 0,
    "B8": 1,
    "B7": 2,
    "B6": 3,
    "B5": 4,
    "B4": 5,
    "B3": 6,
    "A15": 7,
    "A12": 8,
    "A11": 9,
    "A10": 10,
    "A9": 11,
    "A8": 12,
    "B15": 13,
    "B14": 14,
    "B13": 15,
    "B12": 16,
    "C13": 17,  # LED
    "C14": 18,
    "C15": 19,
    "A0": 20,
    "A1": 21,
    "A2": 22,
    "A3": 23,
    "A4": 24,
    "A5": 25,
    "A6": 26,
    "A7": 27,
    "B0": 28,
    "B1": 29,
    "B10": 30,
    "B11": 31,
}


def get_max_pwm_value():
    return 255


def get_analog_offset():
    return 20


def get_adc_bits():
    return 12


def pin_name_to_pin_number(pin):
    if pin in stm32_map:
        return stm32_map[pin]
    raise RuntimeError(f"Unknown conversion from pin {pin} to an IO number")


def get_mcu():
    return "stm32"
