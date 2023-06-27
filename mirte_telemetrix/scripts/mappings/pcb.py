import mappings.pico
import mappings.nanoatmega328
import mappings.stm32


mirte_pico_pcb_map06 = {
    "IR1": {"digital": "16", "analog": "26"},
    "IR2": {"digital": "17", "analog": "27"},
    "SRF1": {"trigger": "7", "echo": "6"},
    "SRF2": {"trigger": "9", "echo": "8"},
    "I2C1": {"scl": "5", "sda": "4"},
    "I2C2": {"scl": "11", "sda": "10"},
    "ENC1": {"pin": "15"},
    "ENC2": {"pin": "14"},
    "Keypad": {"pin": "28"},
    "Servo1": {
        "pin": "2"
    },  # These 2 servos don't work together with the motor controllers at the same time
    "Servo2": {
        "pin": "3"
    },  # These 2 servos don't work together with the motor controllers at the same time
    "Servo3": {"pin": "12"},
    "Servo4": {"pin": "13"},
    "LED": {"pin": "25"},
    "MC1-A": {"1a": "19", "1b": "18"},
    "MC1-B": {"1a": "21", "1b": "20"},
    "MC2-A": {"1a": "16", "1b": "26"},
    "MC2-B": {"1a": "17", "1b": "27"},
}


version = 0.6
board_mapping = mappings.pico
connector_mapping = mirte_pico_pcb_map06


def get_mcu():
    return board_mapping.get_mcu()


def get_analog_offset():
    return board_mapping.get_analog_offset()


def connector_to_pins(connector):
    if connector in connector_mapping:
        return connector_mapping[connector]
    raise RuntimeError(
        f"Unknown conversion from connector {connector} to pins for Pico PCB v{version}."
    )


def pin_name_to_pin_number(pin):
    return board_mapping.pin_name_to_pin_number(pin)


def get_I2C_port(sda):
    return board_mapping.get_I2C_port(sda)


def set_version(new_version, mcu=""):
    global version, board_mapping, connector_mapping
    version = new_version
    if version == 0.6:
        board_mapping = mappings.pico
    if version == 0.4:
        if mcu == "" or mcu == "stm32":
            board_mapping = mappings.stm32
            connector_mapping = mirte_pcb04_stm_map
        else:
            board_mapping = mappings.nano
            connector_mapping = mirte_pcb04_nano_map
    if version == 0.3:
        board_mapping = mappings.stm32
        connector_mapping = mirte_pcb03_stm_map
    if version == 0.2:
        board_mapping = mappings.stm32
        connector_mapping = mirte_pcb02_stm_map


def get_max_pwm_value():
    return board_mapping.get_max_pwm_value()


# mappings for older pcbs

mirte_pcb04_stm_map = {
    "IR1": {"digital": "C15", "analog": "A0"},
    "IR2": {"digital": "A2", "analog": "A1"},
    "SRF1": {"trigger": "A15", "echo": "C14"},
    "SRF2": {"trigger": "A5", "echo": "A6"},
    "I2C1": {"scl": "B6", "sda": "B7"},
    "I2C2": {"scl": "B10", "sda": "B11"},
    "ENCA": {"pin": "B12"},
    "ENCB": {"pin": "B4"},
    "MISC1": {"pin": "B0"},
    "MISC2": {"pin": "B1"},
    "Keypad": {"pin": "A4"},
    "Servo1": {"pin": "B3"},
    "Servo2": {"pin": "A3"},
    "LED": {"pin": "C13"},
    "MA": {"1a": "A8", "1b": "B5"},
    "MB": {"1a": "B14", "1b": "B15"},
    "MC": {"1a": "A10", "1b": "A7"},
    "MD": {"1a": "B13", "1b": "A9"},
}

mirte_pcb04_nano_map = {
    "IR1": {"digital": "A7", "analog": "A1"},
    "IR2": {"digital": "A6", "analog": "A0"},
    "SRF1": {"trigger": "D9", "echo": "D8"},
    "SRF2": {"trigger": "D11", "echo": "D10"},
    "I2C1": {"scl": "A5", "sda": "A4"},
    "ENCA": {"pin": "D2"},
    "ENCB": {"pin": "D3"},
    "Servo1": {"pin": "A3"},
    "Servo2": {"pin": "D12"},
    "LED": {"pin": "D13"},
    "MA": {"1a": "D6", "1b": "D7"},
    "MB": {"1a": "D4", "1b": "D5"},
}

mirte_pcb03_stm_map = {
    "IR1": {"digital": "C15", "analog": "A0"},
    "IR2": {"digital": "B0", "analog": "A1"},
    "SRF1": {"trigger": "A5", "echo": "A6"},
    "SRF2": {"trigger": "B7", "echo": "C14"},
    "I2C1": {"scl": "B6", "sda": "B7"},
    "I2C2": {"scl": "B10", "sda": "B11"},
    "ENCA": {"pin": "B12"},
    "ENCB": {"pin": "B4"},
    "Keypad": {"pin": "A4"},
    "Servo1": {"pin": "B5"},
    "Servo1": {"pin": "A7"},
    "A2": {"pin": "A2"},
    "A3": {"pin": "A3"},
    "LED": {"pin": "C13"},
    "MC1A": {"1a": "A8", "1b": "B3"},
    "MC1B": {"1a": "B14", "1b": "B15"},
    "MC2A": {"1a": "A10", "1b": "B1"},
    "MC2B": {"1a": "B13", "1b": "A9"},
}


mirte_pcb02_stm_map = {
    "IR1": {"digital": "B1", "analog": "A0"},
    "IR2": {"digital": "B0", "analog": "A1"},
    "SRF1": {"trigger": "A9", "echo": "B8"},
    "SRF2": {"trigger": "A10", "echo": "B9"},
    "I2C1": {"scl": "B6", "sda": "B7"},
    "I2C2": {"scl": "B10", "sda": "B11"},
    "ENCA": {"pin": "B4"},
    "ENCB": {"pin": "B13"},
    "Keypad": {"pin": "A4"},
    "Servo1": {"pin": "B5"},
    "A2": {"pin": "A2"},
    "A3": {"pin": "A3"},
    "LED": {"pin": "C13"},
    "MA": {"1a": "A8", "1b": "B3"},
    "MB": {"1a": "B14", "1b": "B15"},
}
