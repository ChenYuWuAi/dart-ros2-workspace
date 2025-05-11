// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dart_msgs:msg/DartLauncherParams.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "dart_msgs/msg/dart_launcher_params.h"


#ifndef DART_MSGS__MSG__DETAIL__DART_LAUNCHER_PARAMS__STRUCT_H_
#define DART_MSGS__MSG__DETAIL__DART_LAUNCHER_PARAMS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/DartLauncherParams in the package dart_msgs.
typedef struct dart_msgs__msg__DartLauncherParams
{
  int32_t primary_yaw;
  int32_t primary_force;
  /// 用于根据测速校准
  int32_t primary_force_offset;
  int32_t auxiliary_yaw_offsets[4];
  int32_t auxiliary_force_offsets[4];
  uint8_t dart_launch_process_offset_begin;
  uint8_t dart_launch_process_offset_end;
  bool auto_aim_enabled;
  double target_auto_aim_x_axis;
  uint64_t last_param_update_time;
} dart_msgs__msg__DartLauncherParams;

// Struct for a sequence of dart_msgs__msg__DartLauncherParams.
typedef struct dart_msgs__msg__DartLauncherParams__Sequence
{
  dart_msgs__msg__DartLauncherParams * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dart_msgs__msg__DartLauncherParams__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DART_MSGS__MSG__DETAIL__DART_LAUNCHER_PARAMS__STRUCT_H_
