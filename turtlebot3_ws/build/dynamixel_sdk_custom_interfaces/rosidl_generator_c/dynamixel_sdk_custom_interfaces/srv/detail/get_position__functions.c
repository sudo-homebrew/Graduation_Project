// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from dynamixel_sdk_custom_interfaces:srv/GetPosition.idl
// generated code does not contain a copyright notice
#include "dynamixel_sdk_custom_interfaces/srv/detail/get_position__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

bool
dynamixel_sdk_custom_interfaces__srv__GetPosition_Request__init(dynamixel_sdk_custom_interfaces__srv__GetPosition_Request * msg)
{
  if (!msg) {
    return false;
  }
  // id
  return true;
}

void
dynamixel_sdk_custom_interfaces__srv__GetPosition_Request__fini(dynamixel_sdk_custom_interfaces__srv__GetPosition_Request * msg)
{
  if (!msg) {
    return;
  }
  // id
}

dynamixel_sdk_custom_interfaces__srv__GetPosition_Request *
dynamixel_sdk_custom_interfaces__srv__GetPosition_Request__create()
{
  dynamixel_sdk_custom_interfaces__srv__GetPosition_Request * msg = (dynamixel_sdk_custom_interfaces__srv__GetPosition_Request *)malloc(sizeof(dynamixel_sdk_custom_interfaces__srv__GetPosition_Request));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dynamixel_sdk_custom_interfaces__srv__GetPosition_Request));
  bool success = dynamixel_sdk_custom_interfaces__srv__GetPosition_Request__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
dynamixel_sdk_custom_interfaces__srv__GetPosition_Request__destroy(dynamixel_sdk_custom_interfaces__srv__GetPosition_Request * msg)
{
  if (msg) {
    dynamixel_sdk_custom_interfaces__srv__GetPosition_Request__fini(msg);
  }
  free(msg);
}


bool
dynamixel_sdk_custom_interfaces__srv__GetPosition_Request__Sequence__init(dynamixel_sdk_custom_interfaces__srv__GetPosition_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  dynamixel_sdk_custom_interfaces__srv__GetPosition_Request * data = NULL;
  if (size) {
    data = (dynamixel_sdk_custom_interfaces__srv__GetPosition_Request *)calloc(size, sizeof(dynamixel_sdk_custom_interfaces__srv__GetPosition_Request));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dynamixel_sdk_custom_interfaces__srv__GetPosition_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dynamixel_sdk_custom_interfaces__srv__GetPosition_Request__fini(&data[i - 1]);
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
dynamixel_sdk_custom_interfaces__srv__GetPosition_Request__Sequence__fini(dynamixel_sdk_custom_interfaces__srv__GetPosition_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      dynamixel_sdk_custom_interfaces__srv__GetPosition_Request__fini(&array->data[i]);
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

dynamixel_sdk_custom_interfaces__srv__GetPosition_Request__Sequence *
dynamixel_sdk_custom_interfaces__srv__GetPosition_Request__Sequence__create(size_t size)
{
  dynamixel_sdk_custom_interfaces__srv__GetPosition_Request__Sequence * array = (dynamixel_sdk_custom_interfaces__srv__GetPosition_Request__Sequence *)malloc(sizeof(dynamixel_sdk_custom_interfaces__srv__GetPosition_Request__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = dynamixel_sdk_custom_interfaces__srv__GetPosition_Request__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
dynamixel_sdk_custom_interfaces__srv__GetPosition_Request__Sequence__destroy(dynamixel_sdk_custom_interfaces__srv__GetPosition_Request__Sequence * array)
{
  if (array) {
    dynamixel_sdk_custom_interfaces__srv__GetPosition_Request__Sequence__fini(array);
  }
  free(array);
}


bool
dynamixel_sdk_custom_interfaces__srv__GetPosition_Response__init(dynamixel_sdk_custom_interfaces__srv__GetPosition_Response * msg)
{
  if (!msg) {
    return false;
  }
  // position
  return true;
}

void
dynamixel_sdk_custom_interfaces__srv__GetPosition_Response__fini(dynamixel_sdk_custom_interfaces__srv__GetPosition_Response * msg)
{
  if (!msg) {
    return;
  }
  // position
}

dynamixel_sdk_custom_interfaces__srv__GetPosition_Response *
dynamixel_sdk_custom_interfaces__srv__GetPosition_Response__create()
{
  dynamixel_sdk_custom_interfaces__srv__GetPosition_Response * msg = (dynamixel_sdk_custom_interfaces__srv__GetPosition_Response *)malloc(sizeof(dynamixel_sdk_custom_interfaces__srv__GetPosition_Response));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dynamixel_sdk_custom_interfaces__srv__GetPosition_Response));
  bool success = dynamixel_sdk_custom_interfaces__srv__GetPosition_Response__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
dynamixel_sdk_custom_interfaces__srv__GetPosition_Response__destroy(dynamixel_sdk_custom_interfaces__srv__GetPosition_Response * msg)
{
  if (msg) {
    dynamixel_sdk_custom_interfaces__srv__GetPosition_Response__fini(msg);
  }
  free(msg);
}


bool
dynamixel_sdk_custom_interfaces__srv__GetPosition_Response__Sequence__init(dynamixel_sdk_custom_interfaces__srv__GetPosition_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  dynamixel_sdk_custom_interfaces__srv__GetPosition_Response * data = NULL;
  if (size) {
    data = (dynamixel_sdk_custom_interfaces__srv__GetPosition_Response *)calloc(size, sizeof(dynamixel_sdk_custom_interfaces__srv__GetPosition_Response));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dynamixel_sdk_custom_interfaces__srv__GetPosition_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dynamixel_sdk_custom_interfaces__srv__GetPosition_Response__fini(&data[i - 1]);
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
dynamixel_sdk_custom_interfaces__srv__GetPosition_Response__Sequence__fini(dynamixel_sdk_custom_interfaces__srv__GetPosition_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      dynamixel_sdk_custom_interfaces__srv__GetPosition_Response__fini(&array->data[i]);
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

dynamixel_sdk_custom_interfaces__srv__GetPosition_Response__Sequence *
dynamixel_sdk_custom_interfaces__srv__GetPosition_Response__Sequence__create(size_t size)
{
  dynamixel_sdk_custom_interfaces__srv__GetPosition_Response__Sequence * array = (dynamixel_sdk_custom_interfaces__srv__GetPosition_Response__Sequence *)malloc(sizeof(dynamixel_sdk_custom_interfaces__srv__GetPosition_Response__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = dynamixel_sdk_custom_interfaces__srv__GetPosition_Response__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
dynamixel_sdk_custom_interfaces__srv__GetPosition_Response__Sequence__destroy(dynamixel_sdk_custom_interfaces__srv__GetPosition_Response__Sequence * array)
{
  if (array) {
    dynamixel_sdk_custom_interfaces__srv__GetPosition_Response__Sequence__fini(array);
  }
  free(array);
}
