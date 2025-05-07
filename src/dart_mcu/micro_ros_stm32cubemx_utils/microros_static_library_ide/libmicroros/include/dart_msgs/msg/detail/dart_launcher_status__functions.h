// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from dart_msgs:msg/DartLauncherStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "dart_msgs/msg/dart_launcher_status.h"


#ifndef DART_MSGS__MSG__DETAIL__DART_LAUNCHER_STATUS__FUNCTIONS_H_
#define DART_MSGS__MSG__DETAIL__DART_LAUNCHER_STATUS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_runtime_c/type_description/type_description__struct.h"
#include "rosidl_runtime_c/type_description/type_source__struct.h"
#include "rosidl_runtime_c/type_hash.h"
#include "rosidl_runtime_c/visibility_control.h"
#include "dart_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "dart_msgs/msg/detail/dart_launcher_status__struct.h"

/// Initialize msg/DartLauncherStatus message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * dart_msgs__msg__DartLauncherStatus
 * )) before or use
 * dart_msgs__msg__DartLauncherStatus__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_dart_msgs
bool
dart_msgs__msg__DartLauncherStatus__init(dart_msgs__msg__DartLauncherStatus * msg);

/// Finalize msg/DartLauncherStatus message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dart_msgs
void
dart_msgs__msg__DartLauncherStatus__fini(dart_msgs__msg__DartLauncherStatus * msg);

/// Create msg/DartLauncherStatus message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * dart_msgs__msg__DartLauncherStatus__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_dart_msgs
dart_msgs__msg__DartLauncherStatus *
dart_msgs__msg__DartLauncherStatus__create(void);

/// Destroy msg/DartLauncherStatus message.
/**
 * It calls
 * dart_msgs__msg__DartLauncherStatus__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dart_msgs
void
dart_msgs__msg__DartLauncherStatus__destroy(dart_msgs__msg__DartLauncherStatus * msg);

/// Check for msg/DartLauncherStatus message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_dart_msgs
bool
dart_msgs__msg__DartLauncherStatus__are_equal(const dart_msgs__msg__DartLauncherStatus * lhs, const dart_msgs__msg__DartLauncherStatus * rhs);

/// Copy a msg/DartLauncherStatus message.
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
ROSIDL_GENERATOR_C_PUBLIC_dart_msgs
bool
dart_msgs__msg__DartLauncherStatus__copy(
  const dart_msgs__msg__DartLauncherStatus * input,
  dart_msgs__msg__DartLauncherStatus * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_dart_msgs
const rosidl_type_hash_t *
dart_msgs__msg__DartLauncherStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_dart_msgs
const rosidl_runtime_c__type_description__TypeDescription *
dart_msgs__msg__DartLauncherStatus__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_dart_msgs
const rosidl_runtime_c__type_description__TypeSource *
dart_msgs__msg__DartLauncherStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_dart_msgs
const rosidl_runtime_c__type_description__TypeSource__Sequence *
dart_msgs__msg__DartLauncherStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of msg/DartLauncherStatus messages.
/**
 * It allocates the memory for the number of elements and calls
 * dart_msgs__msg__DartLauncherStatus__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_dart_msgs
bool
dart_msgs__msg__DartLauncherStatus__Sequence__init(dart_msgs__msg__DartLauncherStatus__Sequence * array, size_t size);

/// Finalize array of msg/DartLauncherStatus messages.
/**
 * It calls
 * dart_msgs__msg__DartLauncherStatus__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dart_msgs
void
dart_msgs__msg__DartLauncherStatus__Sequence__fini(dart_msgs__msg__DartLauncherStatus__Sequence * array);

/// Create array of msg/DartLauncherStatus messages.
/**
 * It allocates the memory for the array and calls
 * dart_msgs__msg__DartLauncherStatus__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_dart_msgs
dart_msgs__msg__DartLauncherStatus__Sequence *
dart_msgs__msg__DartLauncherStatus__Sequence__create(size_t size);

/// Destroy array of msg/DartLauncherStatus messages.
/**
 * It calls
 * dart_msgs__msg__DartLauncherStatus__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dart_msgs
void
dart_msgs__msg__DartLauncherStatus__Sequence__destroy(dart_msgs__msg__DartLauncherStatus__Sequence * array);

/// Check for msg/DartLauncherStatus message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_dart_msgs
bool
dart_msgs__msg__DartLauncherStatus__Sequence__are_equal(const dart_msgs__msg__DartLauncherStatus__Sequence * lhs, const dart_msgs__msg__DartLauncherStatus__Sequence * rhs);

/// Copy an array of msg/DartLauncherStatus messages.
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
ROSIDL_GENERATOR_C_PUBLIC_dart_msgs
bool
dart_msgs__msg__DartLauncherStatus__Sequence__copy(
  const dart_msgs__msg__DartLauncherStatus__Sequence * input,
  dart_msgs__msg__DartLauncherStatus__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // DART_MSGS__MSG__DETAIL__DART_LAUNCHER_STATUS__FUNCTIONS_H_
