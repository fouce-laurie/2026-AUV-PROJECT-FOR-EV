import argparse
import json
import threading

import rclpy

from .interface import RobotInterface
from .movement import MovementSkills
from .tasks import TaskSkills


class AutomatonNode(RobotInterface, MovementSkills, TaskSkills):
    pass


def get_act():
    command = input("请输入任务: \n")
    parts = command.split()
    if parts[0] == 'movexyz':
        args = parts[1:]
        if len(args) == 3:
            x = float(args[0])
            y = float(args[1])
            z = float(args[2])
            dic = {
                "name": "movexyz",
                "params": {
                    "x": x,
                    "y": y,
                    "z": z
                }
            }
            return dic
        else:
            print("指令格式不正确")
            return None

    elif parts[0] == 'throw_golf':
        args = parts[1:]
        if len(args) == 2:
            dy = float(args[0])
            depth = float(args[1])
            dic = {
                "name": "throw_golf",
                "params": {
                    "dy": dy,
                    "depth": depth,
                }
            }
            return dic
        else:
            print("指令格式不正确")
            return None

    elif parts[0] == 'movexy':
        args = parts[1:]
        if len(args) == 2:
            x = float(args[0])
            y = float(args[1])
            dic = {
                "name": "movexy",
                "params": {
                    "x": x,
                    "y": y,
                }
            }
            return dic
        else:
            print("指令格式不正确")
            return None
    elif parts[0] == 'movex':
        args = parts[1:]
        if len(args) == 1:
            x = float(args[0])
            dic = {
                "name": "movex",
                "params": {
                    "x": x,
                }
            }
            return dic
        else:
            print("指令格式不正确")
            return None
    elif parts[0] == 'movey':
        args = parts[1:]
        if len(args) == 1:
            y = float(args[0])
            dic = {
                "name": "movey",
                "params": {
                    "y": y,
                }
            }
            return dic
        else:
            print("指令格式不正确")
            return None
    elif parts[0] == 'movez':
        args = parts[1:]
        if len(args) == 1:
            z = float(args[0])
            dic = {
                "name": "movez",
                "params": {
                    "z": z,
                }
            }
            return dic
        else:
            print("指令格式不正确")
            return None
    elif parts[0] == 'moverz':
        args = parts[1:]
        if len(args) == 1:
            rz = float(args[0])
            dic = {
                "name": "moverz",
                "params": {
                    "rz": rz,
                }
            }
            return dic
        else:
            print("指令格式不正确")
            return None
    elif parts[0] == 'setz':
        args = parts[1:]
        if len(args) == 1:
            z = float(args[0])
            dic = {
                "name": "setz",
                "params": {
                    "z": z,
                }
            }
            return dic
        else:
            print("指令格式不正确")
            return None
    elif parts[0] == 'setrz':
        args = parts[1:]
        if len(args) == 1:
            rz = float(args[0])
            dic = {
                "name": "setrz",
                "params": {
                    "rz": rz,
                }
            }
            return dic
        else:
            print("指令格式不正确")
            return None
    elif parts[0] == 'pow':
        args = parts[1:]
        if len(args) == 1:
            a = int(args[0])
            dic = {
                "name": "pow",
                "params": a
            }
            return dic
        else:
            print("指令格式不正确")
            return None
    elif parts[0] == 'mtty':
        args = parts[1:]
        if len(args) == 2:
            y = float(args[0])
            z = float(args[1])
            dic = {
                "name": "mtty",
                "params": {
                    "y": y,
                    "z": z
                }
            }
            return dic
        else:
            print("指令格式不正确")
            return None
    elif parts[0] == 'mttxf':
        args = parts[1:]
        if len(args) == 2:
            dx = float(args[0])
            dy = float(args[1])
            dic = {
                "name": "mttxf",
                "params": {
                    "dx": dx,
                    "dy": dy
                }
            }
            return dic
        else:
            print("指令格式不正确")
            return None
    elif parts[0] == 'mttz':
        args = parts[1:]
        if len(args) == 2:
            y = float(args[0])
            z = float(args[1])
            dic = {
                "name": "mttz",
                "params": {
                    "y": y,
                    "z": z
                }
            }
            return dic
        else:
            print("指令格式不正确")
            return None
    elif parts[0] == 'mttzxy':
        args = parts[1:]
        if len(args) == 3:
            dz = float(args[0])
            dx = float(args[1])
            dy = float(args[2])
            dic = {
                "name": "mttzxy",
                "params": {
                    "dz": dz,
                    "dx": dx,
                    "dy": dy
                }
            }
            return dic
        else:
            print("指令格式不正确")
    elif parts[0] == 'setp':
        if len(parts) == 1:
            dic = {
                "name": "setp",
                "params": {}
            }
            return dic
        else:
            print("指令格式不正确")
            return None
    elif parts[0] == 'back':
        if len(parts) == 1:
            dic = {
                "name": "back",
                "params": {}
            }
            return dic
        else:
            print("指令格式不正确")
            return None
    elif parts[0] == 'backy':
        if len(parts) == 1:
            dic = {
                "name": "backy",
                "params": {}
            }
            return dic
        else:
            print("指令格式不正确")
            return None
    elif parts[0] == 'search':
        args = parts[1:]
        if len(args) == 2:
            name = args[0]
            cam = args[1]
            dic = {
                "name": "search",
                "params": {
                    "name": name,
                    "cam": cam
                }
            }
            return dic
        else:
            print("指令格式不正确")
            return None
    elif parts[0] == 'line':
        args = parts[1:]
        if len(args) == 1:
            ys_dep = float(args[0])
            dic = {
                "name": "line",
                "params": {
                    "ys_dep": ys_dep
                }
            }
            return dic
        else:
            print("指令格式不正确")
            return None 
        
    elif parts[0] == 'mttpos':
        args = parts[1:]
        if len(args) == 5:
            x = float(args[0])
            y = float(args[1])
            z = float(args[2])
            rz = float(args[3])
            dy = float(args[4])
            dic = {
                "name": "mttpos",
                "params": {
                    "x": x,
                    "y": y,
                    "z": z,
                    "rz": rz,
                    "dy": dy
                }
            }
            return dic
        else:
            print("指令格式不正确")
            return None
        
    elif parts[0] == 'mttzpos':
        args = parts[1:]
        if len(args) == 4:
            x = float(args[0])
            y = float(args[1])
            z = float(args[2])
            dy = float(args[3])
            dic = {
                "name": "mttzpos",
                "params": {
                    "x": x,
                    "y": y,
                    "z": z,
                    "dy": dy
                }
            }
            return dic
        else:
            print("指令格式不正确")
            return None

    elif parts[0] == 'mttpos_amend':
        args = parts[1:]
        if len(args) == 5:
            x = float(args[0])
            y = float(args[1])
            z = float(args[2])
            rz = float(args[3])
            dy = float(args[4])
            dic = {
                "name": "mttpos_amend",
                "params": {
                    "x": x,
                    "y": y,
                    "z": z,
                    "rz": rz,
                    "dy": dy
                }
            }
            return dic
        else:
            print("指令格式不正确")
            return None
        
    elif parts[0] == 'mttzpos_amend':
        args = parts[1:]
        if len(args) == 4:
            x = float(args[0])
            y = float(args[1])
            z = float(args[2])
            dy = float(args[3])
            dic = {
                "name": "mttzpos_amend",
                "params": {
                    "x": x,
                    "y": y,
                    "z": z,
                    "dy": dy
                }
            }
            return dic
        else:
            print("指令格式不正确")
            return None
    
    elif parts[0] == 'graball':
        args = parts[1:]
        if len(args) == 6:
            color       =   str(args[0])
            depth       =   float(args[1])
            timeout     =   float(args[2])
            pr          =   float(args[3])
            k           =   float(args[4])
            step_time   =   float(args[5])           
            dic = {
                "name": "graball",
                "params": {
                    "color": color,
                    "depth": depth,
                    "timeout": timeout,
                    "pr": pr,
                    "k" : k,
                    "step_time":step_time
                }
            }
            return dic
        else:
            print("指令格式不正确")
            return None
    elif parts[0] == 'thrball':
        args = parts[1:]
        if len(args) == 4:
            pr        =   float(args[0])
            timeout   =   float(args[1])
            k         =   float(args[2])
            step_time =   float(args[3])  
            dic = {
                "name": "thrball",
                "params": {
                    "pr": pr,
                    "timeout": timeout,
                    "k" : k,
                    "step_time":step_time
                }
            }
            return dic
        else:
            print("指令格式不正确")
            return None  
    elif parts[0] == 'pass_door':
        args = parts[1:]
        if len(args) == 1:
            num       =   int(args[0])
            dic = {
                "name": "pass_door",
                "params": {
                    "num":num
                }
            }
            return dic
        else:
            print("指令格式不正确")
            return None
    elif parts[0] == 'led':
        args = parts[1:]
        if len(args) == 2:
            led0 = int(args[0])
            led1 = int(args[1])
            dic = {
                "name": "led",
                "params": {
                    "led0": led0,
                    "led1": led1
                }
            }
            return dic
        else:
            print("指令格式不正确")
            return None
    
    elif parts[0] == 'delay':
        args = parts[1:]
        if len(args) == 1:
            t = float(args[0])
            dic = {
                "name": "delay",
                "params": t
            }
            return dic
        else:
            print("指令格式不正确")
            return None

    elif parts[0] == 'grab_golf':
        args = parts[1:]
        if len(args) == 5:
            kind = str(args[0])
            dx = float(args[1])
            dy = float(args[2])
            down_depth = float(args[3])
            up_depth = float(args[4])
            dic = {
                "name": "grab_golf",
                "params": {
                    "kind": kind,
                    "dx": dx,
                    "dy": dy,
                    "down_depth": down_depth,
                    "up_depth": up_depth
                }
            }
            return dic
        else:
            print("指令格式不正确")
            return None
    
    elif parts[0] == 'line_qd':
        args = parts[1:]
        if len(args) == 0:
            dic = {
                "name": "line_qd",
                "params": {}  # 无参数时为空字典
            }
            return dic
        else:
            print("指令格式不正确，line_qd不需要参数")
            return None 
        
    elif parts[0] == 'endfloat':
        args = parts[1:]
        if len(args) == 0:
            dic = {
                "name": "endfloat",
                "params": {}  # 无参数时为空字典
            }
            return dic
        else:
            print("指令格式不正确，line_qd不需要参数")
            return None 
    
    elif parts[0] == 'put_t':
        args = parts[1:]
        if len(args) == 3:
            num = int(args[0])
            dy = float(args[1])
            dz = float(args[2])
            dic = {
                "name": "put_t",
                "params": {
                    "num": num,
                    "dy": dy,
                    "dz": dz
                }
            }
            return dic
        else:
            print("指令格式不正确")
            return None
        
    elif parts[0] == 'strike_ball':
        args = parts[1:]
        if len(args) == 1:
            num = int(args[0])
            dic = {
                "name": "strike_ball",
                "params": {
                    "num": num
                }
            }
            return dic
        else:
            print("指令格式不正确")
            return None
    
    elif parts[0] == 'strike_ball3':
        args = parts[1:]
        if len(args) == 3:
            num1 = int(args[0])
            num2 = int(args[1])
            num3 = int(args[2])
            dic = {
                "name": "strike_ball3",
                "params": {
                    "num1": num1,
                    "num2": num2,
                    "num3": num3
                }
            }
            return dic
        else:
            print("指令格式不正确")
            return None

    else:
        print("无效指令")
        return None
    
    


# 读取并从文件中加载任务队列
def load_actions(path):
    with open(path, 'r', encoding='utf-8') as f:
        dic = json.load(f)
    return dic

# 保存任务队列


def save_actions(dic, path):
    print("所保存的任务:\n"+str(dic)+"\n")
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(dic, f)

# 普通模式


def commom_loop(node, opt):
    list = load_actions(opt.data_path[0]+"uv_tasks.json")

    node.start()
    num = 0

    for act in list["tasks"]:
        node.get_logger().info(
            f"Info:=========执行第 {num:d} 步任务: " + act["name"] + "=========")
        node.run(act)
        node.get_logger().info(
            f"Info:=========执行第 {num:d} 步任务: " + act["name"] + "=========")
        num += 1
    
    node.end()

# 调试模式


def debug_loop(node: AutomatonNode, opt):
    node.get_logger().info("Info:进入调试模式")

    tasklist = load_actions(opt.data_path[0]+"uv_tasks.json")
     
    help_txt = ""
    with open(opt.data_path[0]+"help.txt", 'r', encoding='utf-8') as f:
        s = f.read()
        help_txt = s

    node.start()

    while True:
        node.get_logger().info(f"Info:等待指令输入")
        command = input("请输入调试指令: \n")
        parts = command.split()
        if parts[0] == 'help':
            print(help_txt)
        elif parts[0] == 'act':
            act = get_act()

            if act == None:
                node.get_logger().info(f"Warn:非法任务")
            else:
                node.get_logger().info(
                    f"Info:=========执行任务: " + act["name"] + "=========")
                node.run(act)
                node.get_logger().info(
                    f"Info:=========完成任务: " + act["name"] + "=========")
        elif parts[0] == "task":
            if parts[1] == "run":
                if len(parts[1:]) == 1:
                    num = 0
                    for act in tasklist["tasks"]:
                        node.get_logger().info(
                            f"Info:=========执行第 {num:d} 步任务: " + act["name"] + "=========")
                        node.run(act)
                        node.get_logger().info(
                            f"Info:=========完成第 {num:d} 步任务: " + act["name"] + "=========")
                        num += 1
                elif len(parts[1:]) == 2:
                    num = int(parts[parts.index('run') + 1])
                    if num > len(tasklist["tasks"]) - 1 or num < 0:
                        node.get_logger().info("Warn: 所请求任务不在列表范围内！")
                    else:
                        n = num
                        for act in tasklist["tasks"][num:]:
                            node.get_logger().info(
                                f"Info:=========执行第 {num:d} 步任务: " + act["name"] + "=========")
                            node.run(act)
                            node.get_logger().info(
                                f"Info:=========完成第 {num:d} 步任务: " + act["name"] + "=========")
                            n += 1
                else:
                    node.get_logger().info("Warn:非法指令")
            elif parts[1] == "runonly":
                if len(parts[1:]) == 2:
                    num = int(parts[parts.index('runonly') + 1])
                    if num > len(tasklist["tasks"]) - 1 or num < 0:
                        node.get_logger().info("Warn: 所请求任务不在列表范围内！")
                    else:
                        act = tasklist["tasks"][num]
                        node.get_logger().info(
                            "Info:=========执行任务: " + act["name"] + "=========")
                        node.run(act)
                        node.get_logger().info(
                            "Info:=========完成任务: " + act["name"] + "=========")
                else:
                    node.get_logger().info("Warn:非法指令")
            elif parts[1] == "add":
                if len(parts[1:]) == 2:
                    num = int(parts[parts.index('add') + 1])
                    if num > len(tasklist["tasks"]) - 1 or num < 0:
                        node.get_logger().info("Warn: 所请求任务不在列表范围内！")
                    else:
                        act = get_act()
                        if act == None:
                            node.get_logger().info("非法任务")
                        else:
                            tasklist["tasks"].insert(num, act)
                            save_actions(
                                tasklist, opt.data_path[0]+"uv_tasks.json")
                elif len(parts[1:]) == 1:
                    act = get_act()
                    if act == None:
                        node.get_logger().info("非法任务")
                    else:
                        tasklist["tasks"].append(act)
                        save_actions(
                            tasklist, opt.data_path[0]+"uv_tasks.json")
                else:
                    node.get_logger().info("Warn:非法指令")
            elif parts[1] == "del":
                if len(parts[1:]) == 2:
                    num = int(parts[parts.index('del') + 1])
                    if num > len(tasklist["tasks"]) - 1 or num < 0:
                        node.get_logger().info("Warn: 所请求任务不在列表范围内！")
                    else:
                        tasklist["tasks"].pop(num)
                        save_actions(
                            tasklist, opt.data_path[0]+"uv_tasks.json")
                else:
                    node.get_logger().info("Warn:非法指令")
            elif parts[1] == "clear":
                tasklist["tasks"] = []
                save_actions(tasklist, opt.data_path[0]+"uv_tasks.json")
            elif parts[1] == "list":
                node.get_logger().info("Info: 打印任务列表")
                num = 0
                for i in tasklist["tasks"]:
                    print("编号:   " + str(num))
                    print("任务名: "+i["name"])
                    print("参数:   " + str(i["params"]))
                    num += 1
            elif parts[1] == "mod":
                if len(parts[1:]) == 2:
                    num = int(parts[parts.index('mod') + 1])
                    if num > len(tasklist["tasks"]) - 1 or num < 0:
                        node.get_logger().info("Warn: 所请求任务不在列表范围内！")
                    else:
                        act = get_act()
                        if act == None:
                            node.get_logger().info("非法任务")
                        else:
                            tasklist["tasks"][num] = act
                            save_actions(
                                tasklist, opt.data_path[0]+"uv_tasks.json")
                else:
                    node.get_logger().info("Warn:非法指令")
            else:
                node.get_logger().info("Warn:非法指令")


def main(args=None):

    # 加载参数
    parser = argparse.ArgumentParser()
    parser.add_argument('--front-topic', nargs='+', type=str, default=[
                        'front_cam/rectified'], help='前视摄像头')
    parser.add_argument('--down-topic', nargs='+', type=str, default=[
                        'down_cam/rectified'], help='下视摄像头')
    parser.add_argument('--data-path', nargs='+', type=str, default=[
                        '/home/nvidia/Workspace/Cruise/datas/'], help='PID参数路径')
    parser.add_argument('--debug', nargs='+', type=bool,
                        default=False, help='PID参数路径')

    opt = parser.parse_args()

    rclpy.init(args=args)  # 初始化rclpy

    if opt.debug:
        node = AutomatonNode("uv_automaton_debug", opt)  # 新建一个节点
        thread_debug = threading.Thread(
            target=debug_loop, args=(node, opt))  # 创建调度线程
        thread_debug.start()
    else:
        node = AutomatonNode("uv_automaton", opt)  # 新建一个节点
        thread_common = threading.Thread(
            target=commom_loop, args=(node, opt))    # 创建调度线程
        thread_common.start()

    node.get_logger().info("节点与调度线程成功启动")

    rclpy.spin(node)  # 保持节点运行，检测是否收到退出指令（Ctrl+Z）
    rclpy.shutdown()  # 关闭rclpy
