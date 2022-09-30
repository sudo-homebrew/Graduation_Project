// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from turtlebot3_msgs:msg/VersionInfo.idl
// generated code does not contain a copyright notice
#include "turtlebot3_msgs/msg/detail/version_info__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `hardware`
// Member `firmware`
// Member `software`
#include "rosidl_runtime_c/string_functions.h"

bool
turtlebot3_msgs__msg__VersionInfo__init(turtlebot3_msgs__msg__VersionInfo * msg)
{
  if (!msg) {
    return false;
  }
  // hardware
  if (!rosidl_runtime_c__String__init(&msg->hardware)) {
    turtlebot3_msgs__msg__VersionInfo__fini(msg);
    return false;
  }
  // firmware
  if (!rosidl_runtime_c__String__init(&msg->firmware)) {
    turtlebot3_msgs__msg__VersionInfo__fini(msg);
    return false;
  }
  // software
  if (!rosidl_runtime_c__String__init(&msg->software)) {
    turtlebot3_msgs__msg__VersionInfo__fini(msg);
    return false;
  }
  return true;
}

void
turtlebot3_msgs__msg__VersionInfo__fini(turtlebot3_msgs__msg__VersionInfo * msg)
{
  if (!msg) {
    return;
  }
  // hardware
  rosidl_runtime_c__String__fini(&msg->hardware);
  // firmware
  rosidl_runtime_c__String__fini(&msg->firmware);
  // software
  rosidl_runtime_c__String__fini(&msg->software);
}

turtlebot3_msgs__msg__VersionInfo *
turtlebot3_msgs__msg__VersionInfo__create()
{
  turtlebot3_msgs__msg__VersionInfo * msg = (turtlebot3_msgs__msg__VersionInfo *)malloc(sizeof(turtlebot3_msgs__msg__VersionInfo));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(turtlebot3_msgs__msg__VersionInfo));
  bool success = turtlebot3_msgs__msg__VersionInfo__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
turtlebot3_msgs__msg__VersionInfo__destroy(turtlebot3_msgs__msg__VersionInfo * msg)
{
  if (msg) {
    turtlebot3_msgs__msg__VersionInfo__fini(msg);
  }
  free(msg);
}


bool
turtlebot3_msgs__msg__VersionInfo__Sequence__init(turtlebot3_msgs__msg__VersionInfo__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  turtlebot3_msgs__msg__VersionInfo * data = NULL;
  if (size) {
    data = (turtlebot3_msgs__msg__VersionInfo *)calloc(size, sizeof(turtlebot3_msgs__msg__VersionInfo));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = turtlebot3_msgs__msg__VersionInfo__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        turtlebot3_msgs__msg__VersionInfo__fini(&data[i - 1]);
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
turtlebot3_msgs__msg__VersionInfo__Sequence__fini(turtlebot3_msgs__msg__VersionInfo__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      turtlebot3_msgs__msg__VersionInfo__fini(&array->data[i]);
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

turtlebot3_msgs__msg__VersionInfo__Sequence *
turtlebot3_msgs__msg__VersionInfo__Sequence__create(size_t size)
{
  turtlebot3_msgs__msg__VersionInfo__Sequence * array = (turtlebot3_msgs__msg__VersionInfo__Sequence *)malloc(sizeof(turtlebot3_msgs__msg__VersionInfo__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = turtlebot3_msgs__msg__VersionInfo__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
turtlebot3_msgs__msg__VersionInfo__Sequence__destroy(turtlebot3_msgs__msg__VersionInfo__Sequence * array)
{
  if (array) {
    turtlebot3_msgs__msg__VersionInfo__Sequence__fini(array);
  }
  free(array);
}
