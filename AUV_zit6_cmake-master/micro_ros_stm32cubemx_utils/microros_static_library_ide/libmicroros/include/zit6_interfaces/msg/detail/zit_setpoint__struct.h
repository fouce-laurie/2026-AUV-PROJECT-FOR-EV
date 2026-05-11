// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from zit6_interfaces:msg/ZitSetpoint.idl
// generated code does not contain a copyright notice

#ifndef ZIT6_INTERFACES__MSG__DETAIL__ZIT_SETPOINT__STRUCT_H_
#define ZIT6_INTERFACES__MSG__DETAIL__ZIT_SETPOINT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/ZitSetpoint in the package zit6_interfaces.
typedef struct zit6_interfaces__msg__ZitSetpoint
{
  uint8_t control_key;
  uint8_t type_mask;
  float x;
  float y;
  float z;
  float yaw;
  uint32_t seq;
} zit6_interfaces__msg__ZitSetpoint;

// Struct for a sequence of zit6_interfaces__msg__ZitSetpoint.
typedef struct zit6_interfaces__msg__ZitSetpoint__Sequence
{
  zit6_interfaces__msg__ZitSetpoint * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} zit6_interfaces__msg__ZitSetpoint__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ZIT6_INTERFACES__MSG__DETAIL__ZIT_SETPOINT__STRUCT_H_
