from rclpy.node import Node
import os
import termios


# magic number for uart
SETTINGS = [
    0, 0, 6322, 0, 4098, 4098,
    [
        b'\x00', b'\x00', b'\x00', b'\x00', b'\x00', 1, 254, b'\x00', b'\x00', b'\x00', b'\x00', b'\x00',
        b'\x00', b'\x00', b'\x00', b'\x00', b'\x00', b'\x00', b'\x00', b'\x00', b'\x00', b'\x00', b'\x00',
        b'\x00', b'\x00', b'\x00', b'\x00', b'\x00', b'\x00', b'\x00', b'\x00', b'\x00'
    ]
]

# 养了一只猫猫，路过的人可以摸摸她
# 　／l、
# （ﾟ､ 。 ７
# 　l、 ~ヽ
# 　じしf_, )ノ


class TtyReader:  # 读串口

    def __init__(self, path):
        self.tty_dev = open(path, 'rb')
        termios.tcsetattr(self.tty_dev.fileno(), termios.TCSANOW, SETTINGS)
        self.name = path

    def read(self, length):
        try:
            buff = os.read(self.tty_dev.fileno(), length)
            return True, buff
        except:
            # print("uart read failed")
            return False, b"\x00"

    def close(self):
        try:
            self.tty_dev.close()
        except:
            print("uart has been already closed")

    def __del__(self):
        try:
            self.tty_dev.close()
        except:
            print("uart has been already closed")


class TtyWriter:  # 写串口

    def __init__(self, path):
        self.tty_dev = open(path, 'w')
        termios.tcsetattr(self.tty_dev.fileno(), termios.TCSANOW, SETTINGS)

    def write(self, data):
        try:
            os.write(self.tty_dev.fileno(), data)
            return True
        except:
            # print("uart writed failed")
            return False

    def close(self):
        try:
            self.tty_dev.close()
        except:
            print("uart has been already closed")

    def __del__(self):
        try:
            self.tty_dev.close()
        except:
            print("uart has been already closed")
