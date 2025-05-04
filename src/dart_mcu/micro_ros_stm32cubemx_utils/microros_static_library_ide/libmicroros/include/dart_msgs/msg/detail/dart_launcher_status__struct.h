// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dart_msgs:msg/DartLauncherStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "dart_msgs/msg/dart_launcher_status.h"


#ifndef DART_MSGS__MSG__DETAIL__DART_LAUNCHER_STATUS__STRUCT_H_
#define DART_MSGS__MSG__DETAIL__DART_LAUNCHER_STATUS__STRUCT_H_

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
// Member 'params'
// Member 'protocols'
#include "dart_msgs/msg/detail/dart_launcher_params__struct.h"

/// Struct defined in msg/DartLauncherStatus in the package dart_msgs.
typedef struct dart_msgs__msg__DartLauncherStatus
{
  std_msgs__msg__Header header;
  bool motor_yaw_online;
  bool motor_loader_online[2];
  bool motor_trigger_online;
  bool judge_online;
  bool rc_online;
  uint8_t dart_state;
  uint8_t dart_launch_process;
  int32_t motor_yaw_angle;
  int32_t motor_trigger_angle;
  int32_t motor_loader_current[2];
  int32_t motor_loader_angle[2];
  double last_launch_speed;
  uint64_t last_launch_time;
  uint8_t dart_launch_opening_status;
  uint8_t game_progress;
  uint8_t dart_remaining_time;
  uint16_t latest_launch_cmd_time;
  uint16_t stage_remain_time;
  dart_msgs__msg__DartLauncherParams params;
  dart_msgs__msg__DartLauncherParams protocols;
} dart_msgs__msg__DartLauncherStatus;

// Struct for a sequence of dart_msgs__msg__DartLauncherStatus.
typedef struct dart_msgs__msg__DartLauncherStatus__Sequence
{
  dart_msgs__msg__DartLauncherStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dart_msgs__msg__DartLauncherStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DART_MSGS__MSG__DETAIL__DART_LAUNCHER_STATUS__STRUCT_H_
