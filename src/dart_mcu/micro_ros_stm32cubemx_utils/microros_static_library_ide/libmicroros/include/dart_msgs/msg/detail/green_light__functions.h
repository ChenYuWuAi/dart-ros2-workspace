// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from dart_msgs:msg/GreenLight.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "dart_msgs/msg/green_light.h"


#ifndef DART_MSGS__MSG__DETAIL__GREEN_LIGHT__FUNCTIONS_H_
#define DART_MSGS__MSG__DETAIL__GREEN_LIGHT__FUNCTIONS_H_

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

#include "dart_msgs/msg/detail/green_light__struct.h"

/// Initialize msg/GreenLight message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * dart_msgs__msg__GreenLight
 * )) before or use
 * dart_msgs__msg__GreenLight__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_dart_msgs
bool
dart_msgs__msg__GreenLight__init(dart_msgs__msg__GreenLight * msg);

/// Finalize msg/GreenLight message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dart_msgs
void
dart_msgs__msg__GreenLight__fini(dart_msgs__msg__GreenLight * msg);

/// Create msg/GreenLight message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * dart_msgs__msg__GreenLight__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_dart_msgs
dart_msgs__msg__GreenLight *
dart_msgs__msg__GreenLight__create(void);

/// Destroy msg/GreenLight message.
/**
 * It calls
 * dart_msgs__msg__GreenLight__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dart_msgs
void
dart_msgs__msg__GreenLight__destroy(dart_msgs__msg__GreenLight * msg);

/// Check for msg/GreenLight message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_dart_msgs
bool
dart_msgs__msg__GreenLight__are_equal(const dart_msgs__msg__GreenLight * lhs, const dart_msgs__msg__GreenLight * rhs);

/// Copy a msg/GreenLight message.
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
dart_msgs__msg__GreenLight__copy(
  const dart_msgs__msg__GreenLight * input,
  dart_msgs__msg__GreenLight * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_dart_msgs
const rosidl_type_hash_t *
dart_msgs__msg__GreenLight__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_dart_msgs
const rosidl_runtime_c__type_description__TypeDescription *
dart_msgs__msg__GreenLight__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_dart_msgs
const rosidl_runtime_c__type_description__TypeSource *
dart_msgs__msg__GreenLight__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_dart_msgs
const rosidl_runtime_c__type_description__TypeSource__Sequence *
dart_msgs__msg__GreenLight__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of msg/GreenLight messages.
/**
 * It allocates the memory for the number of elements and calls
 * dart_msgs__msg__GreenLight__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_dart_msgs
bool
dart_msgs__msg__GreenLight__Sequence__init(dart_msgs__msg__GreenLight__Sequence * array, size_t size);

/// Finalize array of msg/GreenLight messages.
/**
 * It calls
 * dart_msgs__msg__GreenLight__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dart_msgs
void
dart_msgs__msg__GreenLight__Sequence__fini(dart_msgs__msg__GreenLight__Sequence * array);

/// Create array of msg/GreenLight messages.
/**
 * It allocates the memory for the array and calls
 * dart_msgs__msg__GreenLight__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_dart_msgs
dart_msgs__msg__GreenLight__Sequence *
dart_msgs__msg__GreenLight__Sequence__create(size_t size);

/// Destroy array of msg/GreenLight messages.
/**
 * It calls
 * dart_msgs__msg__GreenLight__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dart_msgs
void
dart_msgs__msg__GreenLight__Sequence__destroy(dart_msgs__msg__GreenLight__Sequence * array);

/// Check for msg/GreenLight message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_dart_msgs
bool
dart_msgs__msg__GreenLight__Sequence__are_equal(const dart_msgs__msg__GreenLight__Sequence * lhs, const dart_msgs__msg__GreenLight__Sequence * rhs);

/// Copy an array of msg/GreenLight messages.
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
dart_msgs__msg__GreenLight__Sequence__copy(
  const dart_msgs__msg__GreenLight__Sequence * input,
  dart_msgs__msg__GreenLight__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // DART_MSGS__MSG__DETAIL__GREEN_LIGHT__FUNCTIONS_H_
