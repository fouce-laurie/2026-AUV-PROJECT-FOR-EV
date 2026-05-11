// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from zit6_interfaces:msg/ZitPidStatus.idl
// generated code does not contain a copyright notice

#ifndef ZIT6_INTERFACES__MSG__DETAIL__ZIT_PID_STATUS__FUNCTIONS_H_
#define ZIT6_INTERFACES__MSG__DETAIL__ZIT_PID_STATUS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "zit6_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "zit6_interfaces/msg/detail/zit_pid_status__struct.h"

/// Initialize msg/ZitPidStatus message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * zit6_interfaces__msg__ZitPidStatus
 * )) before or use
 * zit6_interfaces__msg__ZitPidStatus__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_zit6_interfaces
bool
zit6_interfaces__msg__ZitPidStatus__init(zit6_interfaces__msg__ZitPidStatus * msg);

/// Finalize msg/ZitPidStatus message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_zit6_interfaces
void
zit6_interfaces__msg__ZitPidStatus__fini(zit6_interfaces__msg__ZitPidStatus * msg);

/// Create msg/ZitPidStatus message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * zit6_interfaces__msg__ZitPidStatus__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_zit6_interfaces
zit6_interfaces__msg__ZitPidStatus *
zit6_interfaces__msg__ZitPidStatus__create();

/// Destroy msg/ZitPidStatus message.
/**
 * It calls
 * zit6_interfaces__msg__ZitPidStatus__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_zit6_interfaces
void
zit6_interfaces__msg__ZitPidStatus__destroy(zit6_interfaces__msg__ZitPidStatus * msg);

/// Check for msg/ZitPidStatus message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_zit6_interfaces
bool
zit6_interfaces__msg__ZitPidStatus__are_equal(const zit6_interfaces__msg__ZitPidStatus * lhs, const zit6_interfaces__msg__ZitPidStatus * rhs);

/// Copy a msg/ZitPidStatus message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_zit6_interfaces
bool
zit6_interfaces__msg__ZitPidStatus__copy(
  const zit6_interfaces__msg__ZitPidStatus * input,
  zit6_interfaces__msg__ZitPidStatus * output);

/// Initialize array of msg/ZitPidStatus messages.
/**
 * It allocates the memory for the number of elements and calls
 * zit6_interfaces__msg__ZitPidStatus__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_zit6_interfaces
bool
zit6_interfaces__msg__ZitPidStatus__Sequence__init(zit6_interfaces__msg__ZitPidStatus__Sequence * array, size_t size);

/// Finalize array of msg/ZitPidStatus messages.
/**
 * It calls
 * zit6_interfaces__msg__ZitPidStatus__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_zit6_interfaces
void
zit6_interfaces__msg__ZitPidStatus__Sequence__fini(zit6_interfaces__msg__ZitPidStatus__Sequence * array);

/// Create array of msg/ZitPidStatus messages.
/**
 * It allocates the memory for the array and calls
 * zit6_interfaces__msg__ZitPidStatus__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_zit6_interfaces
zit6_interfaces__msg__ZitPidStatus__Sequence *
zit6_interfaces__msg__ZitPidStatus__Sequence__create(size_t size);

/// Destroy array of msg/ZitPidStatus messages.
/**
 * It calls
 * zit6_interfaces__msg__ZitPidStatus__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_zit6_interfaces
void
zit6_interfaces__msg__ZitPidStatus__Sequence__destroy(zit6_interfaces__msg__ZitPidStatus__Sequence * array);

/// Check for msg/ZitPidStatus message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_zit6_interfaces
bool
zit6_interfaces__msg__ZitPidStatus__Sequence__are_equal(const zit6_interfaces__msg__ZitPidStatus__Sequence * lhs, const zit6_interfaces__msg__ZitPidStatus__Sequence * rhs);

/// Copy an array of msg/ZitPidStatus messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_zit6_interfaces
bool
zit6_interfaces__msg__ZitPidStatus__Sequence__copy(
  const zit6_interfaces__msg__ZitPidStatus__Sequence * input,
  zit6_interfaces__msg__ZitPidStatus__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ZIT6_INTERFACES__MSG__DETAIL__ZIT_PID_STATUS__FUNCTIONS_H_
