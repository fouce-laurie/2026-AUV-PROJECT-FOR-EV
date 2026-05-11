import time
import struct
import json

from uv_msgs.msg import PidParams
from uv_msgs.msg import PidControllers

from uv_control_py import Serial

# 养了一只猫猫，路过的人可以摸摸她
# 　／l、
# （ﾟ､ 。 ７
# 　l、 ~ヽ
# 　じしf_, )ノ

PID_DIC_INIT = {
    "x": {
        "p": 0.0,
        "i": 0.0,
        "d": 0.0,
        "i_limit": 0.0,
        "output_limit": 0.0,
        "name": "x"
    },
    "y": {
        "p": 0.0,
        "i": 0.0,
        "d": 0.0,
        "i_limit": 0.0,
        "output_limit": 0.0,
        "name": "y"
    },
    "z": {
        "p": 0.0,
        "i": 0.0,
        "d": 0.0,
        "i_limit": 0.0,
        "output_limit": 0.0,
        "name": "z"
    },
    "rx": {
        "p": 0.0,
        "i": 0.0,
        "d": 0.0,
        "i_limit": 0.0,
        "output_limit": 0.0,
        "name": "rx"
    },
    "ry": {
        "p": 0.0,
        "i": 0.0,
        "d": 0.0,
        "i_limit": 0.0,
        "output_limit": 0.0,
        "name": "ry"
    },
    "rz": {
        "p": 0.0,
        "i": 0.0,
        "d": 0.0,
        "i_limit": 0.0,
        "output_limit": 0.0,
        "name": "rz"
    },
    "vx": {
        "p": 0.0,
        "i": 0.0,
        "d": 0.0,
        "i_limit": 0.0,
        "output_limit": 0.0,
        "name": "vx"
    },
    "vy": {
        "p": 0.0,
        "i": 0.0,
        "d": 0.0,
        "i_limit": 0.0,
        "output_limit": 0.0,
        "name": "vy"
    }
}


class PID:
    def __init__(self, tty, save_path):

        self.pid = PidControllers()

        self.pid.x.name = "x"
        self.pid.y.name = "y"
        self.pid.z.name = "z"
        self.pid.rx.name = "rx"
        self.pid.ry.name = "ry"
        self.pid.rz.name = "rz"
        
        self.pid.ry.name = "vx"
        self.pid.rz.name = "vy"

        self.path = save_path
        self.tty_writer = tty

        self.pid_dic = PID_DIC_INIT
        # 读取储存的信息
        self.fileread()

    def dic2class(self):
        self.pid.x.p = self.pid_dic["x"]["p"]
        self.pid.x.i = self.pid_dic["x"]["i"]
        self.pid.x.d = self.pid_dic["x"]["d"]
        self.pid.x.i_limit = self.pid_dic["x"]["i_limit"]
        self.pid.x.output_limit = self.pid_dic["x"]["output_limit"]

        self.pid.y.p = self.pid_dic["y"]["p"]
        self.pid.y.i = self.pid_dic["y"]["i"]
        self.pid.y.d = self.pid_dic["y"]["d"]
        self.pid.y.i_limit = self.pid_dic["y"]["i_limit"]
        self.pid.y.output_limit = self.pid_dic["y"]["output_limit"]

        self.pid.z.p = self.pid_dic["z"]["p"]
        self.pid.z.i = self.pid_dic["z"]["i"]
        self.pid.z.d = self.pid_dic["z"]["d"]
        self.pid.z.i_limit = self.pid_dic["z"]["i_limit"]
        self.pid.z.output_limit = self.pid_dic["z"]["output_limit"]

        self.pid.rx.p = self.pid_dic["rx"]["p"]
        self.pid.rx.i = self.pid_dic["rx"]["i"]
        self.pid.rx.d = self.pid_dic["rx"]["d"]
        self.pid.rx.i_limit = self.pid_dic["rx"]["i_limit"]
        self.pid.rx.output_limit = self.pid_dic["rx"]["output_limit"]

        self.pid.ry.p = self.pid_dic["ry"]["p"]
        self.pid.ry.i = self.pid_dic["ry"]["i"]
        self.pid.ry.d = self.pid_dic["ry"]["d"]
        self.pid.ry.i_limit = self.pid_dic["ry"]["i_limit"]
        self.pid.ry.output_limit = self.pid_dic["ry"]["output_limit"]

        self.pid.rz.p = self.pid_dic["rz"]["p"]
        self.pid.rz.i = self.pid_dic["rz"]["i"]
        self.pid.rz.d = self.pid_dic["rz"]["d"]
        self.pid.rz.i_limit = self.pid_dic["rz"]["i_limit"]
        self.pid.rz.output_limit = self.pid_dic["rz"]["output_limit"]
        
        self.pid.vx.p = self.pid_dic["vx"]["p"]
        self.pid.vx.i = self.pid_dic["vx"]["i"]
        self.pid.vx.d = self.pid_dic["vx"]["d"]
        self.pid.vx.i_limit = self.pid_dic["vx"]["i_limit"]
        self.pid.vx.output_limit = self.pid_dic["vx"]["output_limit"]

        self.pid.vy.p = self.pid_dic["vy"]["p"]
        self.pid.vy.i = self.pid_dic["vy"]["i"]
        self.pid.vy.d = self.pid_dic["vy"]["d"]
        self.pid.vy.i_limit = self.pid_dic["vy"]["i_limit"]
        self.pid.vy.output_limit = self.pid_dic["vy"]["output_limit"]

    def topicrec(self, data):
        self.pid_dic[data.name]["p"] = data.p
        self.pid_dic[data.name]["i"] = data.i
        self.pid_dic[data.name]["d"] = data.d
        self.pid_dic[data.name]["i_limit"] = data.i_limit
        self.pid_dic[data.name]["output_limit"] = data.output_limit
        self.dic2class()

    def hwinit(self):
        # X
        buff = b"\xfa\xaf\x05\x00" + \
            struct.pack("<fffff", self.pid.x.p,self.pid.x.i, 
                        self.pid.x.d, self.pid.x.i_limit,self.pid.x.output_limit)+b"\xfb\xbf"
        self.tty_writer.write(buff)
        time.sleep(0.05)
        # Y
        buff = b"\xfa\xaf\x05\x01" + \
            struct.pack("<fffff", self.pid.y.p, self.pid.y.i,
                        self.pid.y.d, self.pid.y.i_limit,self.pid.y.output_limit)+b"\xfb\xbf"
        self.tty_writer.write(buff)
        time.sleep(0.01)
        # Z
        buff = b"\xfa\xaf\x05\x02" + \
            struct.pack("<fffff", self.pid.z.p, self.pid.z.i,
                        self.pid.z.d, self.pid.z.i_limit,self.pid.z.output_limit)+b"\xfb\xbf"
        self.tty_writer.write(buff)
        time.sleep(0.01)
        # RX
        buff = b"\xfa\xaf\x05\x03" + \
            struct.pack("<fffff", self.pid.rx.p,
                        self.pid.rx.i, self.pid.rx.d, self.pid.rx.i_limit,self.pid.rx.output_limit)+b"\xfb\xbf"
        self.tty_writer.write(buff)
        time.sleep(0.01)
        # RY
        buff = b"\xfa\xaf\x05\x04" + \
            struct.pack("<fffff", self.pid.ry.p,
                        self.pid.ry.i, self.pid.ry.d, self.pid.ry.i_limit,self.pid.ry.output_limit)+b"\xfb\xbf"
        self.tty_writer.write(buff)
        time.sleep(0.01)
        # RZ
        buff = b"\xfa\xaf\x05\x05" + \
            struct.pack("<fffff", self.pid.rz.p,
                        self.pid.rz.i, self.pid.rz.d, self.pid.rz.i_limit,self.pid.rz.output_limit)+b"\xfb\xbf"
        self.tty_writer.write(buff)
        time.sleep(0.01)       
        # VX
        buff = b"\xfa\xaf\x05\x06" + \
            struct.pack("<fffff", self.pid.vx.p,
                        self.pid.vx.i, self.pid.vx.d, self.pid.vx.i_limit,self.pid.vx.output_limit)+b"\xfb\xbf"
        self.tty_writer.write(buff)
        time.sleep(0.01)

        # VY
        buff = b"\xfa\xaf\x05\x07" + \
            struct.pack("<fffff", self.pid.vy.p,
                        self.pid.vy.i, self.pid.vy.d, self.pid.vy.i_limit,self.pid.vy.output_limit)+b"\xfb\xbf"
        self.tty_writer.write(buff)
        time.sleep(0.01)

    def fileread(self):
        with open(self.path, 'r', encoding='utf-8') as file:
            self.pid_dic = json.load(file)
            self.dic2class()

    def filesave(self):
        with open(self.path, 'w', encoding='utf-8') as file:
            json.dump(self.pid_dic, file)
