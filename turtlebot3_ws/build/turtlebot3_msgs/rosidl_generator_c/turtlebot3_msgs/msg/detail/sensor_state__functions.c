// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from turtlebot3_msgs:msg/SensorState.idl
// generated code does not contain a copyright notice
#include "turtlebot3_msgs/msg/detail/sensor_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
turtlebot3_msgs__msg__SensorState__init(turtlebot3_msgs__msg__SensorState * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    turtlebot3_msgs__msg__SensorState__fini(msg);
    return false;
  }
  // bumper
  // cliff
  // sonar
  // illumination
  // led
  // button
  // torque
  // left_encoder
  // right_encoder
  // battery
  return true;
}

void
turtlebot3_msgs__msg__SensorState__fini(turtlebot3_msgs__msg__SensorState * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // bumper
  // cliff
  // sonar
  // illumination
  // led
  // button
  // torque
  // left_encoder
  // right_encoder
  // battery
}

turtlebot3_msgs__msg__SensorState *
turtlebot3_msgs__msg__SensorState__create()
{
  turtlebot3_msgs__msg__SensorState * msg = (turtlebot3_msgs__msg__SensorState *)malloc(sizeof(turtlebot3_msgs__msg__SensorState));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(turtlebot3_msgs__msg__SensorState));
  bool success = turtlebot3_msgs__msg__SensorState__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
turtlebot3_msgs__msg__SensorState__destroy(turtlebot3_msgs__msg__SensorState * msg)
{
  if (msg) {
    turtlebot3_msgs__msg__SensorState__fini(msg);
  }
  free(msg);
}


bool
turtlebot3_msgs__msg__SensorState__Sequence__init(turtlebot3_msgs__msg__SensorState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  turtlebot3_msgs__msg__SensorState * data = NULL;
  if (size) {
    data = (turtlebot3_msgs__msg__SensorState *)calloc(size, sizeof(turtlebot3_msgs__msg__SensorState));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = turtlebot3_msgs__msg__SensorState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        turtlebot3_msgs__msg__SensorState__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
turtlebot3_msgs__msg__SensorState__Sequence__fini(turtlebot3_msgs__msg__SensorState__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      turtlebot3_msgs__msg__SensorState__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

turtlebot3_msgs__msg__SensorState__Sequence *
turtlebot3_msgs__msg__SensorState__Sequence__create(size_t size)
{
  turtlebot3_msgs__msg__SensorState__Sequence * array = (turtlebot3_msgs__msg__SensorState__Sequence *)malloc(sizeof(turtlebot3_msgs__msg__SensorState__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = turtlebot3_msgs__msg__SensorState__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
turtlebot3_msgs__msg__SensorState__Sequence__destroy(turtlebot3_msgs__msg__SensorState__Sequence * array)
{
  if (array) {
    turtlebot3_msgs__msg__SensorState__Sequence__fini(array);
  }
  free(array);
}
