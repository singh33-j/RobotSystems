#!/usr/bin/env python3
from .basic import _Basic_class
""" from .utils import run_command
from smbus2 import SMBus
import multiprocessing """


def _retry_wrapper(func):
    return func
"""     def wrapper(self, *arg, **kwargs):
        for _ in range(self.RETRY):
            try:
                return func(self, *arg, **kwargs)
            except OSError:
                self._debug(f"OSError: {func.__name__}")
                continue
        else:
            return False
 """

class I2C(_Basic_class):
    """
    I2C bus read/write functions
    """
    RETRY = 5

    # i2c_lock = multiprocessing.Value('i', 0)

    def __init__(self, address=None, bus=1, *args, **kwargs):
        """
        Initialize the I2C bus

        :param address: I2C device address
        :type address: int
        :param bus: I2C bus number
        :type bus: int
        """
        super().__init__(*args, **kwargs)
        self._bus = bus
        # self._smbus = SMBus(self._bus)
        self.address = address

    @_retry_wrapper
    def _write_byte(self, data):
        pass
    @_retry_wrapper
    def _write_byte_data(self, reg, data):
        pass
    @_retry_wrapper
    def _write_word_data(self, reg, data):
        pass
    @_retry_wrapper
    def _write_i2c_block_data(self, reg, data):
        pass
    @_retry_wrapper
    def _read_byte(self):
        return 0x00
    @_retry_wrapper
    def _read_byte_data(self, reg):
        return 0x00

    @_retry_wrapper
    def _read_word_data(self, reg):
        return [0x00,0x00]

    @_retry_wrapper
    def _read_i2c_block_data(self, reg, num):
        return 0x00 *num

    @_retry_wrapper
    def is_ready(self):
        return False

    def scan(self):
        return []

    def write(self, data):
        pass
        """Write data to the I2C device

        :param data: Data to write
        :type data: int/list/bytearray
        :raises: ValueError if write is not an int, list or bytearray
        """
    def read(self, length=1):
        return [0x00]*length

    def mem_write(self, data, memaddr):
        """Send data to specific register address

        :param data: Data to send, int, list or bytearray
        :type data: int/list/bytearray
        :param memaddr: Register address
        :type memaddr: int
        :raise ValueError: If data is not int, list, or bytearray
        """
        pass
    def mem_read(self, length, memaddr):
        """Read data from specific register address

        :param length: Number of bytes to receive
        :type length: int
        :param memaddr: Register address
        :type memaddr: int
        :return: Received bytearray data or False if error
        :rtype: list/False
        """
        return [0x00]*length

    def is_avaliable(self):
        """
        Check if the I2C device is avaliable

        :return: True if the I2C device is avaliable, False otherwise
        :rtype: bool
        """
        return False

    def __del__(self):
        if hasattr(self, "_smbus") and self._smbus is not None:
            self._smbus.close()
            self._smbus = None

if __name__ == "__main__":
    i2c = I2C(address=[0x17, 0x15], debug_level='debug')