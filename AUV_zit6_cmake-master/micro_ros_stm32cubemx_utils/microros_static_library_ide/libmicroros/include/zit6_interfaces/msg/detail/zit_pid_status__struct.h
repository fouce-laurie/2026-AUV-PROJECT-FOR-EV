// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from zit6_interfaces:msg/ZitPidStatus.idl
// generated code does not contain a copyright notice

#ifndef ZIT6_INTERFACES__MSG__DETAIL__ZIT_PID_STATUS__STRUCT_H_
#define ZIT6_INTERFACES__MSG__DETAIL__ZIT_PID_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/ZitPidStatus in the package zit6_interfaces.
/**
  * 全轴 PID 与 规划器状态汇总反馈 (1Hz)
 */
typedef struct zit6_interfaces__msg__ZitPidStatus
{
  /// 位置环 P 参数与规划器限值 [X, Y, Z, Yaw]
  float pos_kp[4];
  float pos_max_v[4];
  float pos_max_a[4];
  float pos_out_limit[4];
  /// 速度环 PID 参数 [X, Y, Z, Yaw]
  float vel_kp[4];
  float vel_ki[4];
  float vel_kd[4];
  float vel_i_limit[4];
  float vel_out_limit[4];
} zit6_interfaces__msg__ZitPidStatus;

// Struct for a sequence of zit6_interfaces__msg__ZitPidStatus.
typedef struct zit6_interfaces__msg__ZitPidStatus__Sequence
{
  zit6_interfaces__msg__ZitPidStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} zit6_interfaces__msg__ZitPidStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ZIT6_INTERFACES__MSG__DETAIL__ZIT_PID_STATUS__STRUCT_H_
