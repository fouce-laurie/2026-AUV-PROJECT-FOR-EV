// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from zit6_interfaces:msg/ZitStatus.idl
// generated code does not contain a copyright notice

#ifndef ZIT6_INTERFACES__MSG__DETAIL__ZIT_STATUS__STRUCT_H_
#define ZIT6_INTERFACES__MSG__DETAIL__ZIT_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'LEVEL_NONE'.
/**
  * 控制层级常量 (control_level)
  * 无人控/安全停
 */
enum
{
  zit6_interfaces__msg__ZitStatus__LEVEL_NONE = 0
};

/// Constant 'LEVEL_POS'.
/**
  * 位置环
 */
enum
{
  zit6_interfaces__msg__ZitStatus__LEVEL_POS = 1
};

/// Constant 'LEVEL_VEL'.
/**
  * 速度环
 */
enum
{
  zit6_interfaces__msg__ZitStatus__LEVEL_VEL = 2
};

/// Constant 'LEVEL_FORCE'.
/**
  * 推力环 (直接控制)
 */
enum
{
  zit6_interfaces__msg__ZitStatus__LEVEL_FORCE = 3
};

/// Constant 'ERROR_FORCE_STOP'.
/**
  * 错误标志位常量 (error_flags)
  * Bit0: 强制停机（致命）
 */
enum
{
  zit6_interfaces__msg__ZitStatus__ERROR_FORCE_STOP = 1ul
};

/// Constant 'ERROR_SENSOR_FAIL'.
/**
  * Bit1: 传感器故障（IMU/DVL）
 */
enum
{
  zit6_interfaces__msg__ZitStatus__ERROR_SENSOR_FAIL = 2ul
};

/// Constant 'ERROR_VOLTAGE_LOW'.
/**
  * Bit2: 电压异常
 */
enum
{
  zit6_interfaces__msg__ZitStatus__ERROR_VOLTAGE_LOW = 4ul
};

/// Constant 'ERROR_COMM_TIMEOUT'.
/**
  * Bit3: 通讯超时
 */
enum
{
  zit6_interfaces__msg__ZitStatus__ERROR_COMM_TIMEOUT = 8ul
};

/// Struct defined in msg/ZitStatus in the package zit6_interfaces.
/**
  * ZIT6 AUV 核心状态汇总 (v5.0)
  * 对应话题: /zit6/state/status
 */
typedef struct zit6_interfaces__msg__ZitStatus
{
  /// 是否已解锁（允许执行推力输出）
  bool is_armed;
  /// 解锁模式 (0:默认, 3:遥控模式/绕过导航检查)
  uint8_t arm_mode;
  /// 当前控制层级 (见 LEVEL_* 常量)
  uint8_t control_level;
  /// 惯导内部状态 (0:待机, 1:粗对准, 2:精对准, 3:SINS/GPS/DVL, 4:SINS/DVL, 5:MRU)
  uint8_t ins_state;
  /// 导航准备就绪 (惯导、DVL等传感器健康并对准)
  bool navigation_ready;
  /// 4-DOF 目标推力/力矩 [Fx, Fy, Fz, Mz] (归一化或单位N/Nm)
  float forces[4];
  /// 控制主循环耗时 (单位: ms)
  float cycle_time_ms;
  /// 电池电压 (单位: V)
  float battery_voltage;
  /// 位域错误/警告标识位 (见 ERROR_* 常量)
  uint32_t error_flags;
} zit6_interfaces__msg__ZitStatus;

// Struct for a sequence of zit6_interfaces__msg__ZitStatus.
typedef struct zit6_interfaces__msg__ZitStatus__Sequence
{
  zit6_interfaces__msg__ZitStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} zit6_interfaces__msg__ZitStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ZIT6_INTERFACES__MSG__DETAIL__ZIT_STATUS__STRUCT_H_
