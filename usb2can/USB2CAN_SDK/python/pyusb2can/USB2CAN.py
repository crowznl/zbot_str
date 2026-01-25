import ctypes
from copy import deepcopy
from ctypes import *
import os



class FrameInfo(Structure):
    _fields_ = [("canID", c_uint32),
                ("frameType", c_uint8),
                ("dataLength", c_uint8)]
STANDARD: int = 0
EXTENDED: int = 1

class USB2CAN:
    def __init__(self, device_name:str):
        current_dir = os.path.dirname(os.path.abspath(__file__))
        try:
            self._so_lib = cdll.LoadLibrary(os.path.join(current_dir, "libusb_can.so"))

            self._open_fun = self._so_lib.openUSBCAN
            self._open_fun.restype = c_int32
            self._open_fun.argtypes = [c_char_p]

            self._close_fun = self._so_lib.closeUSBCAN
            self._close_fun.restype = c_int32
            self._close_fun.argtypes = [c_int32]

            self._send_fun = self._so_lib.sendUSBCAN
            self._send_fun.restype = c_int32
            self._send_fun.argtypes = [c_int32, c_uint8, POINTER(FrameInfo), POINTER(c_uint8)]

            self._read_fun = self._so_lib.readUSBCAN
            self._read_fun.restype = c_int32
            self._read_fun.argtypes = [c_int32, POINTER(c_uint8), POINTER(FrameInfo), POINTER(c_uint8), c_int32]

        except Exception as e:
            raise ImportError(e)
        device_name = device_name.encode()
        self._c_handler = self._open_fun(c_char_p(device_name))

    def send_usbcan(self, channel: int, info: FrameInfo, data: bytearray):
        data_c = (ctypes.c_uint8 * 8)(*data)

        return self._so_lib.sendUSBCAN(self._c_handler, c_uint8(channel), pointer(info), data_c)

    def read_usbcan(self, timeout: int = 1e7):
        channel_c = c_uint8()
        info = FrameInfo()
        data_c = (ctypes.c_uint8 * 8)()
        timeout = int(timeout)
        ret = self._so_lib.readUSBCAN(self._c_handler, channel_c, pointer(info), data_c, c_int32(timeout))

        if ret < 0:
            return None
        else:
            return channel_c.value, deepcopy(info), bytearray(data_c)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self._so_lib.closeUSBCAN(self._c_handler)
