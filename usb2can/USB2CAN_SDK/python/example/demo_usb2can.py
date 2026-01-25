import time

from pyusb2can import *
from threading import Thread, Event


def read_thread(can, stop_event):
    while not stop_event.is_set():
        channel, info, data = can.read_usbcan()
        if channel is not None:
            print(f"Channel: {channel}")
            print(f"CAN ID: {info.canID:#08x}")
            print(f"Length: {info.dataLength}")
            print(f"Type: {info.frameType}")
            print(f"Data: {[f'{byte:#02x}' for byte in data]}")



def main():
    with USB2CAN("/dev/USB2CAN0") as can:
        stop_event = Event()
        info = FrameInfo()
        t = Thread(target=read_thread, args=(can, stop_event))
        t.start()
        while True:
            info.canID = (0x01 & 0xff) | ((0xfd & 0xffff) << 8) | ((3 & 0x1f) << 24)
            info.dataLength = 8
            info.frameType = 1

            can.send_usbcan(1, info, bytearray([0x00, 0x00, 0x00, 0x00, 0x00, 0x033, 0x00, 0x3]))

            time.sleep(0.01)

        stop_event.set()
        t.join()

if __name__ == '__main__':
    main()
