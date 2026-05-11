// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from zit6_interfaces:msg/ZitPid.idl
// generated code does not contain a copyright notice

#ifndef ZIT6_INTERFACES__MSG__DETAIL__ZIT_PID__STRUCT_H_
#define ZIT6_INTERFACES__MSG__DETAIL__ZIT_PID__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/ZitPid in the package zit6_interfaces.
typedef struct zit6_interfaces__msg__ZitPid
{
  /// 0:X, 1:Y, 2:Z, 3:Yaw
  uint8_t axis;
  /// true: 位置环(含规划器), false: 速度环
  bool is_pos_ring;
  float kp;
  float ki;
  float kd;
  float i_limit;
  float out_limit;
  /// 规划器参数 (仅在 is_pos_ring 为 true 时有效)
  float max_v;
  float max_a;
} zit6_interfaces__msg__ZitPid;

// Struct for a sequence of zit6_interfaces__msg__ZitPid.
typedef struct zit6_interfaces__msg__ZitPid__Sequence
{
  zit6_interfaces__msg__ZitPid * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} zit6_interfaces__msg__ZitPid__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ZIT6_INTERFACES__MSG__DETAIL__ZIT_PID__STRUCT_H_
