// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from turtlebot3_msgs:msg/SensorState.idl
// generated code does not contain a copyright notice

#ifndef TURTLEBOT3_MSGS__MSG__DETAIL__SENSOR_STATE__STRUCT_H_
#define TURTLEBOT3_MSGS__MSG__DETAIL__SENSOR_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'BUMPER_FORWARD'.
enum
{
  turtlebot3_msgs__msg__SensorState__BUMPER_FORWARD = 1
};

/// Constant 'BUMPER_BACKWARD'.
enum
{
  turtlebot3_msgs__msg__SensorState__BUMPER_BACKWARD = 2
};

/// Constant 'CLIFF'.
enum
{
  turtlebot3_msgs__msg__SensorState__CLIFF = 1
};

/// Constant 'SONAR'.
enum
{
  turtlebot3_msgs__msg__SensorState__SONAR = 1
};

/// Constant 'ILLUMINATION'.
enum
{
  turtlebot3_msgs__msg__SensorState__ILLUMINATION = 1
};

/// Constant 'BUTTON0'.
enum
{
  turtlebot3_msgs__msg__SensorState__BUTTON0 = 1
};

/// Constant 'BUTTON1'.
enum
{
  turtlebot3_msgs__msg__SensorState__BUTTON1 = 2
};

/// Constant 'ERROR_LEFT_MOTOR'.
enum
{
  turtlebot3_msgs__msg__SensorState__ERROR_LEFT_MOTOR = 1
};

/// Constant 'ERROR_RIGHT_MOTOR'.
enum
{
  turtlebot3_msgs__msg__SensorState__ERROR_RIGHT_MOTOR = 2
};

/// Constant 'TORQUE_ON'.
enum
{
  turtlebot3_msgs__msg__SensorState__TORQUE_ON = 1
};

/// Constant 'TORQUE_OFF'.
enum
{
  turtlebot3_msgs__msg__SensorState__TORQUE_OFF = 2
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

// Struct defined in msg/SensorState in the package turtlebot3_msgs.
typedef struct turtlebot3_msgs__msg__SensorState
{
  std_msgs__msg__Header header;
  uint8_t bumper;
  float cliff;
  float sonar;
  float illumination;
  uint8_t led;
  uint8_t button;
  bool torque;
  int32_t left_encoder;
  int32_t right_encoder;
  float battery;
} turtlebot3_msgs__msg__SensorState;

// Struct for a sequence of turtlebot3_msgs__msg__SensorState.
typedef struct turtlebot3_msgs__msg__SensorState__Sequence
{
  turtlebot3_msgs__msg__SensorState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} turtlebot3_msgs__msg__SensorState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TURTLEBOT3_MSGS__MSG__DETAIL__SENSOR_STATE__STRUCT_H_
