import time
import struct
import json

from uv_msgs.msg import ThrustCurve
from uv_msgs.msg import ThrustCurves

from uv_control_py import Serial

# 养了一只猫猫，路过的人可以摸摸她
# 　／l、
# （ﾟ､ 。 ７
# 　l、 ~ヽ
# 　じしf_, )ノ

GET_NAME = [
    "m0",
    "m1",
    "m2",
    "m3",
    "m4",
    "m5"
]

CURVE_DIC_INIT = {
    "m0": {
        "np_mid": 2500.0,
        "np_ini": 3000.0,
        "pp_ini": 3000.0,
        "pp_mid": 3500.0,

        "nt_end": -1500.0,
        "nt_mid": -750.0,
        "pt_mid": 750.0,
        "pt_end": 1500.0,

        "name": "m0",
        "num": 0
    },
    "m1": {
        "np_mid": 2500.0,
        "np_ini": 3000.0,
        "pp_ini": 3000.0,
        "pp_mid": 3500.0,

        "nt_end": -1500.0,
        "nt_mid": -750.0,
        "pt_mid": 750.0,
        "pt_end": 1500.0,

        "name": "m1",
        "num": 1

    },
    "m2": {
        "np_mid": 2500.0,
        "np_ini": 3000.0,
        "pp_ini": 3000.0,
        "pp_mid": 3500.0,

        "nt_end": -1500.0,
        "nt_mid": -750.0,
        "pt_mid": 750.0,
        "pt_end": 1500.0,

        "name": "m2",
        "num": 2

    },
    "m3": {
        "np_mid": 2500.0,
        "np_ini": 3000.0,
        "pp_ini": 3000.0,
        "pp_mid": 3500.0,

        "nt_end": -1500.0,
        "nt_mid": -750.0,
        "pt_mid": 750.0,
        "pt_end": 1500.0,

        "name": "m3",
        "num": 3

    },
    "m4": {
        "np_mid": 2500.0,
        "np_ini": 3000.0,
        "pp_ini": 3000.0,
        "pp_mid": 3500.0,

        "nt_end": -1500.0,
        "nt_mid": -750.0,
        "pt_mid": 750.0,
        "pt_end": 1500.0,

        "name": "m4",
        "num": 4

    },
    "m5": {
        "np_mid": 2500.0,
        "np_ini": 3000.0,
        "pp_ini": 3000.0,
        "pp_mid": 3500.0,

        "nt_end": -1500.0,
        "nt_mid": -750.0,
        "pt_mid": 750.0,
        "pt_end": 1500.0,

        "name": "m5",
        "num": 5
    }
}


class CURVE:
    def __init__(self, tty, save_path):

        self.curves = ThrustCurves()
        self.curves_dic = CURVE_DIC_INIT

        self.curves.m0.num = 0
        self.curves.m1.num = 1
        self.curves.m2.num = 2
        self.curves.m3.num = 3
        self.curves.m4.num = 4
        self.curves.m5.num = 5

        self.path = save_path
        self.tty_writer = tty

        # 读取储存的信息
        self.fileread()

    def dic2class(self):
        self.curves.m0.np_mid = self.curves_dic["m0"]["np_mid"]
        self.curves.m0.np_ini = self.curves_dic["m0"]["np_ini"]
        self.curves.m0.pp_ini = self.curves_dic["m0"]["pp_ini"]
        self.curves.m0.pp_mid = self.curves_dic["m0"]["pp_mid"]
        self.curves.m0.nt_end = self.curves_dic["m0"]["nt_end"]
        self.curves.m0.nt_mid = self.curves_dic["m0"]["nt_mid"]
        self.curves.m0.pt_mid = self.curves_dic["m0"]["pt_mid"]
        self.curves.m0.pt_end = self.curves_dic["m0"]["pt_end"]

        self.curves.m1.np_mid = self.curves_dic["m1"]["np_mid"]
        self.curves.m1.np_ini = self.curves_dic["m1"]["np_ini"]
        self.curves.m1.pp_ini = self.curves_dic["m1"]["pp_ini"]
        self.curves.m1.pp_mid = self.curves_dic["m1"]["pp_mid"]
        self.curves.m1.nt_end = self.curves_dic["m1"]["nt_end"]
        self.curves.m1.nt_mid = self.curves_dic["m1"]["nt_mid"]
        self.curves.m1.pt_mid = self.curves_dic["m1"]["pt_mid"]
        self.curves.m1.pt_end = self.curves_dic["m1"]["pt_end"]

        self.curves.m2.np_mid = self.curves_dic["m2"]["np_mid"]
        self.curves.m2.np_ini = self.curves_dic["m2"]["np_ini"]
        self.curves.m2.pp_ini = self.curves_dic["m2"]["pp_ini"]
        self.curves.m2.pp_mid = self.curves_dic["m2"]["pp_mid"]
        self.curves.m2.nt_end = self.curves_dic["m2"]["nt_end"]
        self.curves.m2.nt_mid = self.curves_dic["m2"]["nt_mid"]
        self.curves.m2.pt_mid = self.curves_dic["m2"]["pt_mid"]
        self.curves.m2.pt_end = self.curves_dic["m2"]["pt_end"]

        self.curves.m3.np_mid = self.curves_dic["m3"]["np_mid"]
        self.curves.m3.np_ini = self.curves_dic["m3"]["np_ini"]
        self.curves.m3.pp_ini = self.curves_dic["m3"]["pp_ini"]
        self.curves.m3.pp_mid = self.curves_dic["m3"]["pp_mid"]
        self.curves.m3.nt_end = self.curves_dic["m3"]["nt_end"]
        self.curves.m3.nt_mid = self.curves_dic["m3"]["nt_mid"]
        self.curves.m3.pt_mid = self.curves_dic["m3"]["pt_mid"]
        self.curves.m3.pt_end = self.curves_dic["m3"]["pt_end"]

        self.curves.m4.np_mid = self.curves_dic["m4"]["np_mid"]
        self.curves.m4.np_ini = self.curves_dic["m4"]["np_ini"]
        self.curves.m4.pp_ini = self.curves_dic["m4"]["pp_ini"]
        self.curves.m4.pp_mid = self.curves_dic["m4"]["pp_mid"]
        self.curves.m4.nt_end = self.curves_dic["m4"]["nt_end"]
        self.curves.m4.nt_mid = self.curves_dic["m4"]["nt_mid"]
        self.curves.m4.pt_mid = self.curves_dic["m4"]["pt_mid"]
        self.curves.m4.pt_end = self.curves_dic["m4"]["pt_end"]

        self.curves.m5.np_mid = self.curves_dic["m5"]["np_mid"]
        self.curves.m5.np_ini = self.curves_dic["m5"]["np_ini"]
        self.curves.m5.pp_ini = self.curves_dic["m5"]["pp_ini"]
        self.curves.m5.pp_mid = self.curves_dic["m5"]["pp_mid"]
        self.curves.m5.nt_end = self.curves_dic["m5"]["nt_end"]
        self.curves.m5.nt_mid = self.curves_dic["m5"]["nt_mid"]
        self.curves.m5.pt_mid = self.curves_dic["m5"]["pt_mid"]
        self.curves.m5.pt_end = self.curves_dic["m5"]["pt_end"]

    def topicrec(self, data):
        self.curves_dic[GET_NAME[data.num]]["np_mid"] = data.np_mid
        self.curves_dic[GET_NAME[data.num]]["np_ini"] = data.np_ini
        self.curves_dic[GET_NAME[data.num]]["pp_ini"] = data.pp_ini
        self.curves_dic[GET_NAME[data.num]]["pp_mid"] = data.pp_mid
        self.curves_dic[GET_NAME[data.num]]["nt_end"] = data.nt_end
        self.curves_dic[GET_NAME[data.num]]["nt_mid"] = data.nt_mid
        self.curves_dic[GET_NAME[data.num]]["pt_mid"] = data.pt_mid
        self.curves_dic[GET_NAME[data.num]]["pt_end"] = data.pt_end
        self.dic2class()
        

    def hwinit(self):
        # M0
        buff = b"\xfa\xaf\x07" \
            + struct.pack("<Bffffffff",self.curves.m0.num ,
                          self.curves.m0.np_mid, self.curves.m0.np_ini,
                          self.curves.m0.pp_ini, self.curves.m0.pp_mid,
                          self.curves.m0.nt_end, self.curves.m0.nt_mid,
                          self.curves.m0.pt_mid, self.curves.m0.pt_end) +b"\xfb\xbf"
        self.tty_writer.write(buff)
        time.sleep(0.05)
        # M1
        buff = b"\xfa\xaf\x07" \
            + struct.pack("<Bffffffff",self.curves.m1.num ,
                          self.curves.m1.np_mid, self.curves.m1.np_ini,
                          self.curves.m1.pp_ini, self.curves.m1.pp_mid,
                          self.curves.m1.nt_end, self.curves.m1.nt_mid,
                          self.curves.m1.pt_mid, self.curves.m1.pt_end) +b"\xfb\xbf"
        self.tty_writer.write(buff)
        time.sleep(0.05)
        # M2
        buff = b"\xfa\xaf\x07" \
            + struct.pack("<Bffffffff",self.curves.m2.num ,
                          self.curves.m2.np_mid, self.curves.m2.np_ini,
                          self.curves.m2.pp_ini, self.curves.m2.pp_mid,
                          self.curves.m2.nt_end, self.curves.m2.nt_mid,
                          self.curves.m2.pt_mid, self.curves.m2.pt_end) +b"\xfb\xbf"
        self.tty_writer.write(buff)
        time.sleep(0.05)
        # M3
        buff = b"\xfa\xaf\x07" \
            + struct.pack("<Bffffffff",self.curves.m3.num ,
                          self.curves.m3.np_mid, self.curves.m3.np_ini,
                          self.curves.m3.pp_ini, self.curves.m3.pp_mid,
                          self.curves.m3.nt_end, self.curves.m3.nt_mid,
                          self.curves.m3.pt_mid, self.curves.m3.pt_end) +b"\xfb\xbf"
        self.tty_writer.write(buff)
        time.sleep(0.05)
        # M4
        buff = b"\xfa\xaf\x07" \
            + struct.pack("<Bffffffff",self.curves.m4.num ,
                          self.curves.m4.np_mid, self.curves.m4.np_ini,
                          self.curves.m4.pp_ini, self.curves.m4.pp_mid,
                          self.curves.m4.nt_end, self.curves.m4.nt_mid,
                          self.curves.m4.pt_mid, self.curves.m4.pt_end) +b"\xfb\xbf"
        self.tty_writer.write(buff)
        time.sleep(0.05)
        # M5
        buff = b"\xfa\xaf\x07" \
            + struct.pack("<Bffffffff",self.curves.m5.num ,
                          self.curves.m5.np_mid, self.curves.m5.np_ini,
                          self.curves.m5.pp_ini, self.curves.m5.pp_mid,
                          self.curves.m5.nt_end, self.curves.m5.nt_mid,
                          self.curves.m5.pt_mid, self.curves.m5.pt_end) +b"\xfb\xbf"
        self.tty_writer.write(buff)
        time.sleep(0.05)


    def fileread(self):
        with open(self.path, 'r', encoding='utf-8') as file:
            self.curves_dic = json.load(file)
            self.dic2class()

    def filesave(self):
        with open(self.path, 'w', encoding='utf-8') as file:
            json.dump(self.curves_dic, file)
