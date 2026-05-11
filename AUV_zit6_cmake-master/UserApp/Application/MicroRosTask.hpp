#ifndef __MICROROS_TASK_HPP
#define __MICROROS_TASK_HPP

#include "FreeRTOS.h"
#include <stdint.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rcutils/allocator.h>

#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/u_int32.h>
#include <std_msgs/msg/u_int8.h>
#include <zit6_interfaces/msg/zit_setpoint.h>
#include <zit6_interfaces/msg/zit_status.h>
#include <zit6_interfaces/msg/zit_pid.h>
#include <zit6_interfaces/msg/zit_pid_status.h>

class MicroRosTask {
public:
	MicroRosTask() = default;
	void run();

private:
	// 单例指针，用于把 C 风格回调转发到当前对象
	static MicroRosTask *instance_;

	// micro-ROS 实体
	rclc_support_t support_;
	rcl_node_t node_;
	rclc_executor_t executor_;
	rcl_subscription_t setpoint_sub_, arm_sub_, ins_cmd_sub_, pid_sub_, servo_sub_, led_sub_;
	rcl_publisher_t pos_pub_, vel_pub_, thr_pub_, zithbt_pub_, status_pub_, pid_status_pub_;

	// 消息 + 缓冲区
	std_msgs__msg__Float32 servo_msg_;
	std_msgs__msg__UInt8 led_msg_;
	std_msgs__msg__Float32MultiArray pos_fb_msg_, vel_fb_msg_, thr_fb_msg_;
	zit6_interfaces__msg__ZitSetpoint setpoint_msg_;
	zit6_interfaces__msg__ZitStatus status_msg_;
	zit6_interfaces__msg__ZitPid pid_msg_;
	zit6_interfaces__msg__ZitPidStatus pid_status_msg_;
	std_msgs__msg__UInt8 ins_cmd_msg_;
	std_msgs__msg__UInt32 arm_msg_, node_heartbeat_msg_;

	float pos_buf_[4], vel_buf_[4], thr_buf_[4];

	// 处理函数（原来位于匿名命名空间）
	void onZitPid(const void *msgin);
	void onZitSetpoint(const void *msgin);
	void onArmHeartbeat(const void *msgin);
	void onInsCommand(const void *msgin);
	void onServoCmd(const void *msgin);
	void onLedCmd(const void *msgin);
	void cleanupMicroRos();

	// 静态回调封装（传入 rclc）
	static void zitPidCb(const void *msgin) { if (instance_) instance_->onZitPid(msgin); }
	static void setpointCb(const void *msgin) { if (instance_) instance_->onZitSetpoint(msgin); }
	static void armCb(const void *msgin) { if (instance_) instance_->onArmHeartbeat(msgin); }
	static void insCmdCb(const void *msgin) { if (instance_) instance_->onInsCommand(msgin); }
	static void servoCb(const void *msgin) { if (instance_) instance_->onServoCmd(msgin); }
	static void ledCb(const void *msgin) { if (instance_) instance_->onLedCmd(msgin); }
};

#endif // __MICROROS_TASK_HPP
