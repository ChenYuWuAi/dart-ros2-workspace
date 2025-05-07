// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dart_msgs:msg/GreenLight.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "dart_msgs/msg/green_light.h"


#ifndef DART_MSGS__MSG__DETAIL__GREEN_LIGHT__STRUCT_H_
#define DART_MSGS__MSG__DETAIL__GREEN_LIGHT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'location'
#include "geometry_msgs/msg/detail/point__struct.h"

/// Struct defined in msg/GreenLight in the package dart_msgs.
typedef struct dart_msgs__msg__GreenLight
{
  std_msgs__msg__Header header;
  bool is_detected;
  geometry_msgs__msg__Point location;
} dart_msgs__msg__GreenLight;

// Struct for a sequence of dart_msgs__msg__GreenLight.
typedef struct dart_msgs__msg__GreenLight__Sequence
{
  dart_msgs__msg__GreenLight * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dart_msgs__msg__GreenLight__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DART_MSGS__MSG__DETAIL__GREEN_LIGHT__STRUCT_H_
