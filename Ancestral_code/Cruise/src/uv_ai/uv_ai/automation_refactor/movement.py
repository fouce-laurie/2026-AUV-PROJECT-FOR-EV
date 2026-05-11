# 基础移动与路径相关能力（尽量保持原逻辑）
import time
from math import atan2, sqrt

from uv_control_py.CoordinateSystem import AngleCorrect
from uv_msgs.msg import TargetPosDown

from .config import AllowedError, RAD2DEG, Step


class MovementSkills:
    def _make_rel_cmd(self):
        """Builds a relative TargetPosDown command template."""
        # 相对位移指令模板（统一 cs/姿态）
        cmd = TargetPosDown()
        cmd.cs = 2
        cmd.pos.rx = 0.0
        cmd.pos.ry = 0.0
        return cmd

    def _publish_rel_cmd(self, cmd, x=0.0, y=0.0, z=0.0, rz=0.0):
        """Fills and publishes a relative move command."""
        # 统一发布相对位移指令，避免重复字段赋值
        cmd.pos.x = x
        cmd.pos.y = y
        cmd.pos.z = z
        cmd.pos.rz = rz
        self.target_pos_down_pub.publish(cmd)

    def _step_move(self, axis, value, step, delay, label, log_steps=False):
        """Moves along one axis in steps and waits at the end."""
        # 将“分步逼近 + 最终等待”抽成通用逻辑，减少重复
        cmd = self._make_rel_cmd()
        cnt = 0
        while True:
            cnt += 1
            if log_steps:
                self.get_logger().info(f"Info: 开始第 {cnt:d} 次{label}")
            if abs(value) <= step:
                self.get_logger().info(f"Info: 最终{label}开始")
                x = y = z = rz = 0.0
                if axis == "x":
                    x = value
                elif axis == "y":
                    y = value
                elif axis == "z":
                    z = value
                elif axis == "rz":
                    rz = value
                self._publish_rel_cmd(cmd, x=x, y=y, z=z, rz=rz)
                ok = self.move_wait()
                if ok:
                    self.get_logger().info(f"Info: 最终{label}完成")
                else:
                    self.get_logger().info(f"Warn: 最终{label}超时！")
                break
            else:
                step_delta = step if value > 0 else -step
                value -= step_delta
                x = y = z = rz = 0.0
                if axis == "x":
                    x = step_delta
                elif axis == "y":
                    y = step_delta
                elif axis == "z":
                    z = step_delta
                elif axis == "rz":
                    rz = step_delta
                self._publish_rel_cmd(cmd, x=x, y=y, z=z, rz=rz)
                time.sleep(delay)

    def _target_in_base_from_backpoint(self):
        """Transforms backpoint world pose into base-frame delta."""
        # 从回退点计算目标在机体坐标系下的位移
        self.robot.target_inworld.vector.x = self.backpoint["x"]
        self.robot.target_inworld.vector.y = self.backpoint["y"]
        self.robot.target_inworld.vector.z = self.backpoint["z"]
        self.robot.target_inworld.vector.rz = self.backpoint["rz"]
        self.robot.world2base()
        return (
            self.robot.target_inbase.vector.x,
            self.robot.target_inbase.vector.y,
            self.robot.target_inbase.vector.z,
            self.robot.target_inbase.vector.rz,
        )

    def _target_in_base_from_target(self):
        """Transforms target world pose into base-frame delta."""
        # 从目标点计算目标在机体坐标系下的位移
        self.robot.target_inworld.vector.x = self.target["x"]
        self.robot.target_inworld.vector.y = self.target["y"]
        self.robot.target_inworld.vector.z = self.target["z"]
        self.robot.world2base()
        return (
            self.robot.target_inbase.vector.x,
            self.robot.target_inbase.vector.y,
            self.robot.target_inbase.vector.z,
        )

    def move_wait(self):
        """Waits until pose error is within AllowedError or times out."""
        cnt = 0
        time.sleep(0.5)
        while True:
            judge = 0
            if abs(self.MotionController.tpos_inbase.x) > AllowedError["x"]:
                judge += 1
            if abs(self.MotionController.tpos_inbase.y) > AllowedError["y"]:
                judge += 1
            if abs(self.MotionController.tpos_inbase.z) > AllowedError["z"]:
                judge += 1
            if abs(self.MotionController.tpos_inbase.rz) > AllowedError["rz"]:
                judge += 1

            if judge == 0:  # 进入容许范围
                return True

            if cnt > 200:  # 超时退出
                return False

            cnt += 1
            time.sleep(0.05)

    def movez(self, z):
        """Moves along z in steps (relative depth change)."""
         # 校验
        if self.robot.base.vector.z + z < -1.0:
            self.get_logger().info("Warn: 深度设置超出范围")
            z = -self.robot.base.vector.z

        self.get_logger().info("======开始移动======")

        # 调整深度
        step_cnt = int(abs(z)/Step["z"]) + 1
        #self.get_logger().info(f"Info: 开始调整深度, 需要调整 {step_cnt:d} 次")
        self._step_move("z", z, Step["z"], 0.2, "深度调整", log_steps=True)

        self.get_logger().info("======移动结束======")
    
        # 移动 z 的相对位移
    def fast_movez(self, z):
        """Moves along z once without stepping."""
        # 直接调整
        if self.robot.base.vector.z + z < 0:
            self.get_logger().info("Warn: 深度设置超出范围")
        else:
            self.get_logger().info("======开始移动======")
            self.get_logger().info("Info: 深度快速微调开始")
            cmd = self._make_rel_cmd()
            self._publish_rel_cmd(cmd, z=z)
            self.get_logger().info("======移动结束======")

        # 渐进调整
        # # 校验
        # if self.robot.base.vector.z + z < 0.2:
        #     self.get_logger().info("Warn: 深度设置超出范围")
        # else:
        #     self.get_logger().info("======开始移动======")

        #     cmd = TargetPosDown()
        #     cmd.cs = 2
        #     cmd.pos.rx = 0.0
        #     cmd.pos.ry = 0.0

        #     # 调整深度
        #     step_cnt = int(abs(z)/Step["z"]) + 1
        #     self.get_logger().info(f"Info: 开始调整深度, 需要调整 {step_cnt:d} 次")
        #     cnt = 0
        #     while True:
        #         cnt += 1
        #         self.get_logger().info(f"Info: 开始第 {cnt:d} 次调整深度")
        #         if abs(z) <= Step["z"]:
        #             self.get_logger().info("Info: 最终深度调整开始")
        #             cmd.pos.z = z
        #             cmd.pos.x = cmd.pos.y = cmd.pos.rz = 0.0
        #             self.target_pos_down_pub.publish(cmd)
        #             re = self.move_wait()
        #             if re:
        #                 self.get_logger().info("Info: 最终深度调整完成")
        #             else:
        #                 self.get_logger().info("Warn: 最终深度调整超时！")
        #             break
        #         else:
        #             if z > 0:
        #                 z -= Step["z"]
        #                 cmd.pos.z = Step["z"]
        #             else:
        #                 z += Step["z"]
        #                 cmd.pos.z = -Step["z"]
        #             cmd.pos.x = cmd.pos.y = cmd.pos.rz = 0.0
        #             self.target_pos_down_pub.publish(cmd)
        #             self.get_logger().info(f"Info: 第 {cnt:d} 次深度调整完成")
        #             time.sleep(0.2)

        #     self.get_logger().info("======移动结束======")

    # 移动 x 的相对位移
    def movex(self, x):
        """Moves along x in steps (relative lateral move)."""
        if x > 10:
            self.get_logger().info("Warn: 横向位置设置超出范围！")
        else:
            self.get_logger().info("======开始移动======")

            # 调整横向位置
            step_cnt = int(abs(x)/Step["x"]) + 1
            self.get_logger().info(f"Info: 开始调整横向位置, 需要调整 {step_cnt:d} 次")
            self._step_move("x", x, Step["x"], 0.1, "横向位置调整")
            self.get_logger().info("======移动结束======")

    # 移动 x 的相对位移
    def fast_movex(self, x):
        """Moves along x once without stepping."""
        if x > 10:
            self.get_logger().info("Warn: 横向位置设置超出范围！")
        else:
            self.get_logger().info("======开始移动======")
            self.get_logger().info("Info: 横向位置快速微调开始")
            cmd = self._make_rel_cmd()
            self._publish_rel_cmd(cmd, x=x)
            self.get_logger().info("======移动结束======")
    


    # 移动 y 的相对位移
    def movey(self, y):
        """Moves along y in steps (relative forward/back move)."""
        if y > 10:
            self.get_logger().info("Warn: 前后位置设置超出范围！")
        else:
            self.get_logger().info("======开始移动======")

            # 调整
            step_cnt = int(abs(y)/Step["y"]) + 1
            self.get_logger().info(f"Info: 开始调整前后位置, 需要调整 {step_cnt:d} 次")
            self._step_move("y", y, Step["y"], 0.1, "前后位置调整")
            self.get_logger().info("======移动结束======")

                # 移动 y 的相对位移
    def fast_movey(self, y):
        """Moves along y once without stepping."""
        if y > 10:
            self.get_logger().info("Warn: 前后位置设置超出范围！")
        else:
            self.get_logger().info("======开始移动======")
            self.get_logger().info("Info: 前后位置快速微整开始")
            cmd = self._make_rel_cmd()
            self._publish_rel_cmd(cmd, y=y)
            self.get_logger().info("======移动结束======")

    # 移动 rz 的相对位移
    def moverz(self, rz):
        """Rotates around z in steps (relative yaw)."""
        if rz > 180 or rz < -180:
            self.get_logger().info("Warn: 角度设置超出范围！")
            rz = AngleCorrect(rz)
            self.get_logger().info(f"Info: 角度等效为 {rz:.2f}°")

        self.get_logger().info("======开始旋转======")

        # 调整
        step_cnt = int(abs(rz)/Step["rz"]) + 1
        self.get_logger().info(f"Info: 开始调整角度, 需要调整 {step_cnt:d} 次")
        self._step_move("rz", rz, Step["rz"], 0.1, "角度调整")
        self.get_logger().info("======旋转结束======")
        
        
    def fast_moverz(self, rz):
        """Rotates around z once without stepping."""
        if rz > 180 or rz < -180:
            self.get_logger().info("Warn: 角度设置超出范围！")
            rz = AngleCorrect(rz)
            self.get_logger().info(f"Info: 角度等效为 {rz:.2f}°")

        self.get_logger().info("======开始旋转======")

        self.get_logger().info("Info: 前后位置快速微整开始")
        cmd = self._make_rel_cmd()
        self._publish_rel_cmd(cmd, rz=rz)
        self.get_logger().info("======移动结束======")

    # 移动x,y相对位移
    def movexy(self, x, y):
        """Turns to target direction then moves planar distance."""
        if x == 0 and y == 0:
            self.get_logger().info("Warn: 未设置合法位移！")
        else:
            rz = atan2(y, x)*RAD2DEG - 90
            d = sqrt(x*x + y*y)
            self.get_logger().info(f"Info: ======转向目标点,旋转{rz:.2f}°======")
            self.moverz(rz)
            self.get_logger().info("Info: ======转向结束======")
            time.sleep(0.1)
            self.get_logger().info(f"Info: ======移动指定距离,移动{d:.2f}m======")
            self.movey(d)
            self.get_logger().info("Info: ======移动结束======")
            time.sleep(0.1)
            #self.get_logger().info(f"Info: ======朝向回正,旋转{-rz:.2f}°======")
            #self.moverz(-rz)
            #self.get_logger().info("Info: ======转向结束======")

    # 移动x,y,z相对位移
    def movexyz(self, x, y, z):
        """Adjusts depth then moves in the horizontal plane."""
        if z == 0:
            self.get_logger().info("Warn: 未设置合法深度位移！")
        else:
            self.movez(z)
        time.sleep(0.1)
        self.movexy(x, y)

    # 设置 z
    def setz(self, z):
        """Sets absolute depth while keeping current pose."""
        if z >= -1.0:
            self.get_logger().info("======开始调整深度======")
            cmd = TargetPosDown()
            cmd.cs = 0
            cmd.pos.rx = self.MotionController.pos.rx
            cmd.pos.ry = self.MotionController.pos.ry
            cmd.pos.x = self.MotionController.pos.x
            cmd.pos.y = self.MotionController.pos.y
            cmd.pos.rz = self.MotionController.pos.rz
            cmd.pos.z = z
            self.target_pos_down_pub.publish(cmd)
            s = self.move_wait()
            if s:
                self.get_logger().info("Info: 深度调整完成")
            else:
                self.get_logger().info("Warn: 深度调整超时！")
            self.get_logger().info("======移动结束======")
        else:
            self.get_logger().info("Warn: 深度设置错误！")

    # 设置rz
    def setrz(self, rz):
        """Sets absolute yaw while keeping current pose."""
        if rz > 180 or rz < -180:
            self.get_logger().info("Warn: 角度设置超出范围！")
            rz = AngleCorrect(rz)
            self.get_logger().info(f"Info: 角度等效为 {rz:.2f}°")

        self.get_logger().info("======开始调整角度======")
        cmd = TargetPosDown()
        cmd.cs = 0
        cmd.pos.rx = self.MotionController.pos.rx
        cmd.pos.ry = self.MotionController.pos.ry
        cmd.pos.x = self.MotionController.pos.x
        cmd.pos.y = self.MotionController.pos.y
        cmd.pos.z = self.MotionController.pos.z
        cmd.pos.rz = rz
        self.target_pos_down_pub.publish(cmd)
        s = self.move_wait()
        if s:
            self.get_logger().info("Info: 角度调整完成")
        else:
            self.get_logger().info("Warn: 角度调整超时！")
        self.get_logger().info("======移动结束======")

    # 设置路径点
    def setp(self):
        """Stores current pose as backpoint."""
        self.backpoint["x"] = self.MotionController.pos.x
        self.backpoint["y"] = self.MotionController.pos.y
        self.backpoint["z"] = self.MotionController.pos.z
        self.backpoint["rz"] = self.MotionController.pos.rz
        self.get_logger().info(
            f'Info:已保存当前位置 x: {self.backpoint["x"]:.2f} y: {self.backpoint["y"]:.2f} z: {self.backpoint["z"]:.2f} rz : {self.backpoint["rz"]:.2f}')

    # 回到路径点
    def back(self):
        """Returns to backpoint with stepwise adjustments."""
        x, y, z, rz = self._target_in_base_from_backpoint()

        self.get_logger().info(
            f"Info:需要调整的位移 x: {x:.2f} y: {y:.2f} z: {z:.2f} rz : {rz:.2f}")

        if z == 0:
            self.get_logger().info("Warn: 深度位移非法！")
        else:
            self.movez(z)
        time.sleep(0.1)

        if x == 0 and y == 0:
            self.get_logger().info("Warn: 非法位移！")
            self.get_logger().info(f"Info: ======调整姿态,旋转{rz:.2f}°======")
            self.moverz(rz)
            self.get_logger().info("Info: ======转向结束======")
        else:
            rz_ = atan2(y, x)*RAD2DEG - 270
            d = sqrt(x*x + y*y)
            self.get_logger().info(f"Info: ======转向目标点,旋转{rz_:.2f}°======")
            self.moverz(rz_)
            self.get_logger().info("Info: ======转向结束======")
            time.sleep(0.1)
            self.get_logger().info(f"Info: ======移动指定距离,移动{rz:.2f}m======")
            self.movey(-d)
            self.get_logger().info("Info: ======移动结束======")
            self.get_logger().info(f"Info: ======调整姿态,旋转{rz_:.2f}°======")
            self.moverz(rz-rz_)
            self.get_logger().info("Info: ======转向结束======")
            
    def fast_back(self):
        """Returns to backpoint with larger steps and waits."""
        x, y, z, rz = self._target_in_base_from_backpoint()

        self.get_logger().info(
            f"Info:需要调整的位移 x: {x:.2f} y: {y:.2f} z: {z:.2f} rz : {rz:.2f}")

        if z == 0:
            self.get_logger().info("Warn: 深度位移非法！")
        else:
            self.fast_movez(z)
        time.sleep(2)

        if x == 0 and y == 0:
            self.get_logger().info("Warn: 非法位移！")
            self.get_logger().info(f"Info: ======调整姿态,旋转{rz:.2f}°======")
            self.fast_moverz(rz)
            self.get_logger().info("Info: ======转向结束======")
        else:
            rz_ = atan2(y, x)*RAD2DEG - 270
            d = sqrt(x*x + y*y)
            self.get_logger().info(f"Info: ======转向目标点,旋转{rz_:.2f}°======")
            self.fast_moverz(rz_)
            self.get_logger().info("Info: ======转向结束======")
            time.sleep(5.0)
            self.get_logger().info(f"Info: ======移动指定距离,移动{rz:.2f}m======")
            self.fast_movey(-d)
            time.sleep(5.0)
            self.get_logger().info("Info: ======移动结束======")
            self.get_logger().info(f"Info: ======调整姿态,旋转{rz_:.2f}°======")
            self.fast_moverz(rz-rz_)
            time.sleep(2.0)
            self.get_logger().info("Info: ======转向结束======")
            
    # 回到路径点
    def backy(self):
        """Returns to backpoint with planar move then depth."""
        x, y, z, rz = self._target_in_base_from_backpoint()

        self.get_logger().info(
            f"Info:需要调整的位移 x: {x:.2f} y: {y:.2f} z: {z:.2f} rz : {rz:.2f}")


        if x == 0 and y == 0:
            self.get_logger().info("Warn: 非法位移！")
            self.get_logger().info(f"Info: ======调整姿态,旋转{rz:.2f}°======")
            self.moverz(rz)
            self.get_logger().info("Info: ======转向结束======")
        else:
            rz_ = atan2(y, x)*RAD2DEG - 270
            d = sqrt(x*x + y*y)
            self.get_logger().info(f"Info: ======转向目标点,旋转{rz_:.2f}°======")
            self.moverz(rz_)
            self.get_logger().info("Info: ======转向结束======")
            time.sleep(0.1)
            self.get_logger().info(f"Info: ======移动指定距离,移动{rz:.2f}m======")
            self.movey(-d)
            self.get_logger().info("Info: ======移动结束======")
            self.get_logger().info(f"Info: ======调整姿态,旋转{rz_:.2f}°======")
            self.moverz(rz-rz_)
            self.get_logger().info("Info: ======转向结束======")

        if z == 0:
            self.get_logger().info("Warn: 深度位移非法！")
        else:
            self.movez(z)
        time.sleep(0.1)

    # 移动至寄存器 self.target 所指定的位置
    def mtty(self, dy, dz):
        """Moves toward target with depth and forward offsets."""
        x, y, z = self._target_in_base_from_target()

        z -= dz

        self.get_logger().info(
            f"Info:需要调整的位移 x: {x:.2f} y: {y:.2f} z: {z:.2f}")

        if z == 0:
            self.get_logger().info("Warn: 深度位移非法！")
        else:
            self.movez(z)
        time.sleep(0.1)

        if x == 0 and y == 0:
            self.get_logger().info("Warn: 非法位移！")
        else:
            rz = atan2(y, x)*RAD2DEG - 90
            d = sqrt(x*x + y*y) - dy
            self.get_logger().info(f"Info: ======转向目标点,旋转{rz:.2f}°======")
            self.moverz(rz)
            self.get_logger().info("Info: ======转向结束======")
            time.sleep(0.1)
            self.get_logger().info(f"Info: ======移动指定距离,移动{d:.2f}m======")
            self.movey(d)
            self.get_logger().info("Info: ======移动结束======")

    def mttzxy(self, dz, dx, dy):
        """Moves toward target with dz/dx/dy offsets."""
        x, y, z = self._target_in_base_from_target()

        x += dx
        y += dy
        z += dz

        self.get_logger().info(
            f"Info:需要调整的位移 x: {x:.2f} y: {y:.2f} z: {z:.2f}")
        
        if z == 0:
            self.get_logger().info("Warn: 深度位移非法！")
        else:
            self.get_logger().info(f"Info: ======深度移动指定距离,移动{z:.2f}m======")
            self.movez(z)
            self.get_logger().info("Info: ======移动结束======")
        time.sleep(0.1)

        if x == 0:
            self.get_logger().info("Warn: 水平位移非法！")
        else:
            self.get_logger().info(f"Info: ======横移指定距离,移动{x:.2f}m======")
            self.movex(x)
            self.get_logger().info("Info: ======移动结束======")
        time.sleep(0.1)

        if y == 0:
            self.get_logger().info("Warn: 前后位移非法！")
        else:
            self.get_logger().info(f"Info: ======前后指定距离,移动{y:.2f}m======")
            self.movey(y)
        time.sleep(0.1)

        self.get_logger().info(f"Info: ======移动结束======")


    def mttxf(self,dx,dy):
        """Moves toward target with lateral/forward offsets."""
        x, y, z = self._target_in_base_from_target()

        self.get_logger().info(
            f"Info:需要调整的位移 x: {x:.2f} y: {y:.2f} z: {z:.2f}")
        x_target = x - dx
        self.get_logger().info(f"Info: ======横移指定距离,移动{x:.2f}m======")
        self.movex(x_target)
        self.get_logger().info("Info: ======移动结束======")
        time.sleep(0.1)

        if y == 0:
            self.get_logger().info("Warn: 非法位移！")
        else:
            d = y - dy
            time.sleep(0.1)
            self.get_logger().info(f"Info: ======移动指定距离,移动{d:.2f}m======")
            self.movey(d)
            self.get_logger().info("Info: ======移动结束======")
        time.sleep(0.1)


    # 移动至寄存器 self.target 所指定的位置
    def mttz(self, dy, dz):
        """Moves toward target, then adjusts depth."""
        x, y, z = self._target_in_base_from_target()

        z -= dz

        self.get_logger().info(
            f"Info:需要调整的位移 x: {x:.2f} y: {y:.2f} z: {z:.2f}")

        if x == 0 and y == 0:
            self.get_logger().info("Warn: 非法位移！")
        else:
            rz = atan2(y, x)*RAD2DEG - 90
            d = sqrt(x*x + y*y) - dy
            self.get_logger().info(f"Info: ======转向目标点,旋转{rz:.2f}°======")
            self.moverz(rz)
            self.get_logger().info("Info: ======转向结束======")
            time.sleep(0.1)
            self.get_logger().info(f"Info: ======移动指定距离,移动{rz:.2f}m======")
            self.movey(d)
            self.get_logger().info("Info: ======移动结束======")

        if z == 0:
            self.get_logger().info("Warn: 深度位移非法！")
        else:
            self.movez(z)
        time.sleep(0.1)
    
    #移动至指定世界坐标位置
    def mttpos(self, x, y, z, rz, dy):
        """Sets target and runs alignment/approach flow."""
        self.target["x"] = x
        self.target["y"] = y
        self.target["z"] = z

        Step["y"] = 0.04
        self.mttz(dy, 0.0)#小范围微调
        self.setrz(rz)
        Step["y"] = 0.02
    
    #移动至指定世界坐标位置，但最后不旋转
    def mttzpos(self, x, y, z, dy):
        """Sets target and runs planar + depth adjustment."""
        self.target["x"] = x
        self.target["y"] = y
        self.target["z"] = z
        Step["z"] = 0.2
        Step["y"] = 0.04
        self.mttz(dy, 0.0)
        Step["y"] = 0.02
        Step["z"] = 0.05
    
    #移动至指定世界坐标位置，以比赛开始位置为坐标
    def mttpos_amend(self, x, y, z, rz, dy):
        """Approaches target with extra correction parameters."""
        self.start_pos.target_inbase.vector.x = x
        self.start_pos.target_inbase.vector.y = y
        self.start_pos.target_inbase.vector.z = z
        self.start_pos.target_inbase.vector.rz = rz
        self.start_pos.target_inbase.vector.rx = self.start_pos.target_inbase.vector.ry= 0.0
        self.start_pos.base2world()
        self.target["x"] = self.start_pos.target_inworld.vector.x
        self.target["y"] = self.start_pos.target_inworld.vector.y
        self.target["z"] = self.start_pos.target_inworld.vector.z
        self.target["rz"] = self.start_pos.target_inworld.vector.rz
        
        self.get_logger().info(
            f"x: {self.target['x']:.2f} y: {self.target['y']:.2f} z: {self.target['z']:.2f}")
              
        Step["y"] = 0.04
        self.mttz(dy, 0.0)
        self.setrz(rz+self.start_pos.base.vector.rz)
        Step["y"] = 0.02
    
    #移动至指定世界坐标位置，以比赛开始位置为坐标
    def mttzpos_amend(self, x, y, z, dy):
        """Approaches target with planar/depth corrections."""
        self.start_pos.target_inbase.vector.x = x
        self.start_pos.target_inbase.vector.y = y
        self.start_pos.target_inbase.vector.z = z
        self.start_pos.target_inbase.vector.rx = self.start_pos.target_inbase.vector.ry= 0.0
        self.start_pos.base2world()
        self.target["x"] = self.start_pos.target_inworld.vector.x
        self.target["y"] = self.start_pos.target_inworld.vector.y
        self.target["z"] = self.start_pos.target_inworld.vector.z
        Step["z"] = 0.2
        Step["y"] = 0.04
        self.mttz(dy, 0.0)
        Step["y"] = 0.02
        Step["z"] = 0.05
    
    # 搜寻目标
