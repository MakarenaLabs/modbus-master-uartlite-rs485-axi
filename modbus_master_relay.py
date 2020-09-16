from pynq import Overlay
from pynq import MMIO
from time import sleep, time
import numpy as np

# HARDWARE CONSTANTS
RX_FIFO = 0x00
TX_FIFO = 0x04

# Status Reg
STAT_REG = 0x08
RX_VALID = 0
RX_FULL = 1
TX_EMPTY = 2
TX_FULL = 3
IS_INTR = 4
OVERRUN_ERR = 5
FRAME_ERR = 6
PARITY_ERR = 7

# Ctrl Reg
CTRL_REG = 0x0C
RST_TX = 0
RST_RX = 1
INTR_EN = 4

# Offset Settings
XGPIO_DATA_OFFSET = 0x0
XGPIO_TRI_OFFSET = 0x4


class UartAXI:
    def __init__(self, address):
        # Setup axi core
        self.uart = MMIO(address, 0x10000, debug=False)
        self.address = address

    def getBit(self, num, pos):
        return (num & 1 << pos) >> pos

    def setupCtrlReg(self):
        # Reset FIFOs, disable interrupts
        self.uart.write(CTRL_REG, 1 << RST_TX | 1 << RST_RX)
        sleep(1)
        self.uart.write(CTRL_REG, 0)
        sleep(1)

    def currentStatus(self):
        # Returns object that specifies current status of axi core
        status = self.uart.read(STAT_REG)
        return {'RX_VALID': self.getBit(status, RX_VALID),
                'RX_FULL': self.getBit(status, RX_FULL),
                'TX_EMPTY': self.getBit(status, TX_EMPTY),
                'TX_FULL': self.getBit(status, TX_FULL),
                'IS_INTR': self.getBit(status, IS_INTR),
                'OVERRUN_ERR': self.getBit(status, OVERRUN_ERR),
                'FRAME_ERR': self.getBit(status, FRAME_ERR),
                'PARITY_ERR': self.getBit(status, PARITY_ERR)}

    def read(self, count, timeout=10):
        # status = currentStatus(uart) bad idea
        buf = ""
        stop_time = time() + timeout
        for i in range(count):
            # Wait till RX fifo has valid data, stop waiting if timeoutpasses
            while (not (self.uart.read(STAT_REG) & 1 << RX_VALID)) and (time() < stop_time):
                pass
            if time() >= stop_time:
                break
            buf += chr(self.uart.read(RX_FIFO))
        return buf

    def write_bytes(self, buf, timeout=10):
        # Write bytes via UART
        stop_time = time() + timeout
        wr_count = 0
        for i in buf:
            # Wait while TX FIFO is Full, stop waiting if timeout passes
            while (self.uart.read(STAT_REG) & 1 << TX_FULL) and (time() < stop_time):
                pass
            # Check timeout
            if time() > stop_time:
                break
            self.uart.write(TX_FIFO, i)
            wr_count += 1
        return wr_count

    def write(self, buf, timeout=10):
        # Write bytes via UART
        stop_time = time() + timeout
        wr_count = 0
        for i in buf:
            # Wait while TX FIFO is Full, stop waiting if timeout passes
            while (self.uart.read(STAT_REG) & 1 << TX_FULL) and (time() < stop_time):
                pass
            # Check timeout
            if time() > stop_time:
                break
            self.uart.write(TX_FIFO, ord(i))
            wr_count += 1
        return wr_count

    def readLine(self):
        buf = self.read(1)
        if len(buf) == 0:
            return ""
        while '\n' not in buf:
            buf += self.read(1)
        return buf


class GPIO:
    def __init__(self, address):
        # Setup axi core
        self.gpio = MMIO(address, 0x10000, debug=False)
        self.address = address

    def write(self, signal):
        # Send signal via GPIO using defined offset
        self.gpio.write(XGPIO_DATA_OFFSET, signal)


class RS485:
    def __init__(self, uart, gpio):
        # Initializing UART and GPIO
        self.uart = uart
        self.gpio = gpio

    def write_bytes(self, data):
        # Write bytes to UART via GPIO pinout
        self.gpio.write(3)
        return self.uart.write_bytes(data)

    def read(self, n_bytes, timeout=10):
        # Read bytes from UART via GPIO pinout
        self.gpio.write(0)
        return self.uart.read(n_bytes, timeout)

    def readLine(self, n_bytes):
        # Read line from UART via GPIO pinout
        self.gpio.write(0)
        return self.uart.readLine()


class Modbus_Relay:
    def __init__(self, rs485):
        # Initializing Modbus "Master" class
        self.rs485 = rs485
        self.id = 0x01

    def crc16(self, data: bytes):
        # CRC-16-ModBus Algorithm
        data = bytearray(data)
        poly = 0xA001
        crc = 0xFFFF
        for b in data:
            crc ^= (0xFF & b)
            for _ in range(0, 8):
                if (crc & 0x0001):
                    crc = ((crc >> 1) & 0xFFFF) ^ poly
                else:
                    crc = ((crc >> 1) & 0xFFFF)

        ccc = np.uint16(crc)

        temp_str = (hex(ccc)).replace("0x", "")
        temp_full = temp_str.rjust(4, '0')
        byte_r = (int(temp_full[0:2], 16))
        byte_l = (int(temp_full[2:4], 16))
        return (byte_r, byte_l)

    def set_relay(self, relay, state, debug=True):
        # Sending a state (0 or 1) to a specific relay
        mb_function = 0x05
        _bytes = [self.id, mb_function, 0x00, relay, state, 0x00]
        (crc_r, crc_l) = self.crc16(_bytes)
        frame = _bytes
        frame.append(crc_l)
        frame.append(crc_r)
        if debug:
            print((frame))
        self.rs485.write_bytes(frame)


fpgabit = None
uart = None
gpio = None
rs485 = None
relay = None


def init_all(uart_address, gpio_address):

    global fpgabit
    global uart
    global gpio
    global rs485
    global relay

    # Address of the ip core
    uart = UartAXI(uart_address)

    # Setup AXI UART register
    uart.setupCtrlReg()

    # Setup GPIO address
    gpio = GPIO(gpio_address)

    # Initializing UART interface
    rs485 = RS485(uart, gpio)

    # Initializing Modbus "master" class with specific UART interface
    relay = Modbus_Relay(rs485)


def set_relay_function(relay_number, enabled):
    global relay
    # Function to be called by other modules
    relay.set_relay(relay_number, 1 if enabled else 0, False)


if __name__ == "__main__":
    # Main class, a simple demo that switches on and off relay from 0 to 3 (included)

    # Address of the ip core
    ADDRESS = 0x00a0000000
    uart = UartAXI(ADDRESS)

    # Setup AXI UART register
    uart.setupCtrlReg()

    # Setup GPIO address
    address = 0x00a0010000
    gpio = GPIO(address)

    # Initializing UART interface
    rs485 = RS485(uart, gpio)

    # Initializing Modbus "master" class with specific UART interface
    relay = Modbus_Relay(rs485)

    relay.set_relay(0, 1)

    relay.set_relay(0, 0)

    relay.set_relay(1, 1)

    relay.set_relay(1, 0)

    relay.set_relay(2, 1)

    relay.set_relay(2, 0)

    relay.set_relay(3, 1)

    relay.set_relay(3, 0)
