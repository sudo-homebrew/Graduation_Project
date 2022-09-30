// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from turtlebot3_msgs:action/Patrol.idl
// generated code does not contain a copyright notice
#include "turtlebot3_msgs/action/detail/patrol__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
turtlebot3_msgs__action__Patrol_Goal__init(turtlebot3_msgs__action__Patrol_Goal * msg)
{
  if (!msg) {
    return false;
  }
  // radius
  return true;
}

void
turtlebot3_msgs__action__Patrol_Goal__fini(turtlebot3_msgs__action__Patrol_Goal * msg)
{
  if (!msg) {
    return;
  }
  // radius
}

turtlebot3_msgs__action__Patrol_Goal *
turtlebot3_msgs__action__Patrol_Goal__create()
{
  turtlebot3_msgs__action__Patrol_Goal * msg = (turtlebot3_msgs__action__Patrol_Goal *)malloc(sizeof(turtlebot3_msgs__action__Patrol_Goal));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(turtlebot3_msgs__action__Patrol_Goal));
  bool success = turtlebot3_msgs__action__Patrol_Goal__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
turtlebot3_msgs__action__Patrol_Goal__destroy(turtlebot3_msgs__action__Patrol_Goal * msg)
{
  if (msg) {
    turtlebot3_msgs__action__Patrol_Goal__fini(msg);
  }
  free(msg);
}


bool
turtlebot3_msgs__action__Patrol_Goal__Sequence__init(turtlebot3_msgs__action__Patrol_Goal__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  turtlebot3_msgs__action__Patrol_Goal * data = NULL;
  if (size) {
    data = (turtlebot3_msgs__action__Patrol_Goal *)calloc(size, sizeof(turtlebot3_msgs__action__Patrol_Goal));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = turtlebot3_msgs__action__Patrol_Goal__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        turtlebot3_msgs__action__Patrol_Goal__fini(&data[i - 1]);
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
turtlebot3_msgs__action__Patrol_Goal__Sequence__fini(turtlebot3_msgs__action__Patrol_Goal__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      turtlebot3_msgs__action__Patrol_Goal__fini(&array->data[i]);
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

turtlebot3_msgs__action__Patrol_Goal__Sequence *
turtlebot3_msgs__action__Patrol_Goal__Sequence__create(size_t size)
{
  turtlebot3_msgs__action__Patrol_Goal__Sequence * array = (turtlebot3_msgs__action__Patrol_Goal__Sequence *)malloc(sizeof(turtlebot3_msgs__action__Patrol_Goal__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = turtlebot3_msgs__action__Patrol_Goal__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
turtlebot3_msgs__action__Patrol_Goal__Sequence__destroy(turtlebot3_msgs__action__Patrol_Goal__Sequence * array)
{
  if (array) {
    turtlebot3_msgs__action__Patrol_Goal__Sequence__fini(array);
  }
  free(array);
}


bool
turtlebot3_msgs__action__Patrol_Result__init(turtlebot3_msgs__action__Patrol_Result * msg)
{
  if (!msg) {
    return false;
  }
  // success
  return true;
}

void
turtlebot3_msgs__action__Patrol_Result__fini(turtlebot3_msgs__action__Patrol_Result * msg)
{
  if (!msg) {
    return;
  }
  // success
}

turtlebot3_msgs__action__Patrol_Result *
turtlebot3_msgs__action__Patrol_Result__create()
{
  turtlebot3_msgs__action__Patrol_Result * msg = (turtlebot3_msgs__action__Patrol_Result *)malloc(sizeof(turtlebot3_msgs__action__Patrol_Result));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(turtlebot3_msgs__action__Patrol_Result));
  bool success = turtlebot3_msgs__action__Patrol_Result__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
turtlebot3_msgs__action__Patrol_Result__destroy(turtlebot3_msgs__action__Patrol_Result * msg)
{
  if (msg) {
    turtlebot3_msgs__action__Patrol_Result__fini(msg);
  }
  free(msg);
}


bool
turtlebot3_msgs__action__Patrol_Result__Sequence__init(turtlebot3_msgs__action__Patrol_Result__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  turtlebot3_msgs__action__Patrol_Result * data = NULL;
  if (size) {
    data = (turtlebot3_msgs__action__Patrol_Result *)calloc(size, sizeof(turtlebot3_msgs__action__Patrol_Result));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = turtlebot3_msgs__action__Patrol_Result__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        turtlebot3_msgs__action__Patrol_Result__fini(&data[i - 1]);
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
turtlebot3_msgs__action__Patrol_Result__Sequence__fini(turtlebot3_msgs__action__Patrol_Result__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      turtlebot3_msgs__action__Patrol_Result__fini(&array->data[i]);
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

turtlebot3_msgs__action__Patrol_Result__Sequence *
turtlebot3_msgs__action__Patrol_Result__Sequence__create(size_t size)
{
  turtlebot3_msgs__action__Patrol_Result__Sequence * array = (turtlebot3_msgs__action__Patrol_Result__Sequence *)malloc(sizeof(turtlebot3_msgs__action__Patrol_Result__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = turtlebot3_msgs__action__Patrol_Result__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
turtlebot3_msgs__action__Patrol_Result__Sequence__destroy(turtlebot3_msgs__action__Patrol_Result__Sequence * array)
{
  if (array) {
    turtlebot3_msgs__action__Patrol_Result__Sequence__fini(array);
  }
  free(array);
}


bool
turtlebot3_msgs__action__Patrol_Feedback__init(turtlebot3_msgs__action__Patrol_Feedback * msg)
{
  if (!msg) {
    return false;
  }
  // left_time
  return true;
}

void
turtlebot3_msgs__action__Patrol_Feedback__fini(turtlebot3_msgs__action__Patrol_Feedback * msg)
{
  if (!msg) {
    return;
  }
  // left_time
}

turtlebot3_msgs__action__Patrol_Feedback *
turtlebot3_msgs__action__Patrol_Feedback__create()
{
  turtlebot3_msgs__action__Patrol_Feedback * msg = (turtlebot3_msgs__action__Patrol_Feedback *)malloc(sizeof(turtlebot3_msgs__action__Patrol_Feedback));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(turtlebot3_msgs__action__Patrol_Feedback));
  bool success = turtlebot3_msgs__action__Patrol_Feedback__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
turtlebot3_msgs__action__Patrol_Feedback__destroy(turtlebot3_msgs__action__Patrol_Feedback * msg)
{
  if (msg) {
    turtlebot3_msgs__action__Patrol_Feedback__fini(msg);
  }
  free(msg);
}


bool
turtlebot3_msgs__action__Patrol_Feedback__Sequence__init(turtlebot3_msgs__action__Patrol_Feedback__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  turtlebot3_msgs__action__Patrol_Feedback * data = NULL;
  if (size) {
    data = (turtlebot3_msgs__action__Patrol_Feedback *)calloc(size, sizeof(turtlebot3_msgs__action__Patrol_Feedback));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = turtlebot3_msgs__action__Patrol_Feedback__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        turtlebot3_msgs__action__Patrol_Feedback__fini(&data[i - 1]);
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
turtlebot3_msgs__action__Patrol_Feedback__Sequence__fini(turtlebot3_msgs__action__Patrol_Feedback__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      turtlebot3_msgs__action__Patrol_Feedback__fini(&array->data[i]);
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

turtlebot3_msgs__action__Patrol_Feedback__Sequence *
turtlebot3_msgs__action__Patrol_Feedback__Sequence__create(size_t size)
{
  turtlebot3_msgs__action__Patrol_Feedback__Sequence * array = (turtlebot3_msgs__action__Patrol_Feedback__Sequence *)malloc(sizeof(turtlebot3_msgs__action__Patrol_Feedback__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = turtlebot3_msgs__action__Patrol_Feedback__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
turtlebot3_msgs__action__Patrol_Feedback__Sequence__destroy(turtlebot3_msgs__action__Patrol_Feedback__Sequence * array)
{
  if (array) {
    turtlebot3_msgs__action__Patrol_Feedback__Sequence__fini(array);
  }
  free(array);
}


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `goal`
// already included above
// #include "turtlebot3_msgs/action/detail/patrol__functions.h"

bool
turtlebot3_msgs__action__Patrol_SendGoal_Request__init(turtlebot3_msgs__action__Patrol_SendGoal_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    turtlebot3_msgs__action__Patrol_SendGoal_Request__fini(msg);
    return false;
  }
  // goal
  if (!turtlebot3_msgs__action__Patrol_Goal__init(&msg->goal)) {
    turtlebot3_msgs__action__Patrol_SendGoal_Request__fini(msg);
    return false;
  }
  return true;
}

void
turtlebot3_msgs__action__Patrol_SendGoal_Request__fini(turtlebot3_msgs__action__Patrol_SendGoal_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // goal
  turtlebot3_msgs__action__Patrol_Goal__fini(&msg->goal);
}

turtlebot3_msgs__action__Patrol_SendGoal_Request *
turtlebot3_msgs__action__Patrol_SendGoal_Request__create()
{
  turtlebot3_msgs__action__Patrol_SendGoal_Request * msg = (turtlebot3_msgs__action__Patrol_SendGoal_Request *)malloc(sizeof(turtlebot3_msgs__action__Patrol_SendGoal_Request));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(turtlebot3_msgs__action__Patrol_SendGoal_Request));
  bool success = turtlebot3_msgs__action__Patrol_SendGoal_Request__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
turtlebot3_msgs__action__Patrol_SendGoal_Request__destroy(turtlebot3_msgs__action__Patrol_SendGoal_Request * msg)
{
  if (msg) {
    turtlebot3_msgs__action__Patrol_SendGoal_Request__fini(msg);
  }
  free(msg);
}


bool
turtlebot3_msgs__action__Patrol_SendGoal_Request__Sequence__init(turtlebot3_msgs__action__Patrol_SendGoal_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  turtlebot3_msgs__action__Patrol_SendGoal_Request * data = NULL;
  if (size) {
    data = (turtlebot3_msgs__action__Patrol_SendGoal_Request *)calloc(size, sizeof(turtlebot3_msgs__action__Patrol_SendGoal_Request));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = turtlebot3_msgs__action__Patrol_SendGoal_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        turtlebot3_msgs__action__Patrol_SendGoal_Request__fini(&data[i - 1]);
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
turtlebot3_msgs__action__Patrol_SendGoal_Request__Sequence__fini(turtlebot3_msgs__action__Patrol_SendGoal_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      turtlebot3_msgs__action__Patrol_SendGoal_Request__fini(&array->data[i]);
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

turtlebot3_msgs__action__Patrol_SendGoal_Request__Sequence *
turtlebot3_msgs__action__Patrol_SendGoal_Request__Sequence__create(size_t size)
{
  turtlebot3_msgs__action__Patrol_SendGoal_Request__Sequence * array = (turtlebot3_msgs__action__Patrol_SendGoal_Request__Sequence *)malloc(sizeof(turtlebot3_msgs__action__Patrol_SendGoal_Request__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = turtlebot3_msgs__action__Patrol_SendGoal_Request__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
turtlebot3_msgs__action__Patrol_SendGoal_Request__Sequence__destroy(turtlebot3_msgs__action__Patrol_SendGoal_Request__Sequence * array)
{
  if (array) {
    turtlebot3_msgs__action__Patrol_SendGoal_Request__Sequence__fini(array);
  }
  free(array);
}


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
turtlebot3_msgs__action__Patrol_SendGoal_Response__init(turtlebot3_msgs__action__Patrol_SendGoal_Response * msg)
{
  if (!msg) {
    return false;
  }
  // accepted
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    turtlebot3_msgs__action__Patrol_SendGoal_Response__fini(msg);
    return false;
  }
  return true;
}

void
turtlebot3_msgs__action__Patrol_SendGoal_Response__fini(turtlebot3_msgs__action__Patrol_SendGoal_Response * msg)
{
  if (!msg) {
    return;
  }
  // accepted
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
}

turtlebot3_msgs__action__Patrol_SendGoal_Response *
turtlebot3_msgs__action__Patrol_SendGoal_Response__create()
{
  turtlebot3_msgs__action__Patrol_SendGoal_Response * msg = (turtlebot3_msgs__action__Patrol_SendGoal_Response *)malloc(sizeof(turtlebot3_msgs__action__Patrol_SendGoal_Response));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(turtlebot3_msgs__action__Patrol_SendGoal_Response));
  bool success = turtlebot3_msgs__action__Patrol_SendGoal_Response__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
turtlebot3_msgs__action__Patrol_SendGoal_Response__destroy(turtlebot3_msgs__action__Patrol_SendGoal_Response * msg)
{
  if (msg) {
    turtlebot3_msgs__action__Patrol_SendGoal_Response__fini(msg);
  }
  free(msg);
}


bool
turtlebot3_msgs__action__Patrol_SendGoal_Response__Sequence__init(turtlebot3_msgs__action__Patrol_SendGoal_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  turtlebot3_msgs__action__Patrol_SendGoal_Response * data = NULL;
  if (size) {
    data = (turtlebot3_msgs__action__Patrol_SendGoal_Response *)calloc(size, sizeof(turtlebot3_msgs__action__Patrol_SendGoal_Response));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = turtlebot3_msgs__action__Patrol_SendGoal_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        turtlebot3_msgs__action__Patrol_SendGoal_Response__fini(&data[i - 1]);
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
turtlebot3_msgs__action__Patrol_SendGoal_Response__Sequence__fini(turtlebot3_msgs__action__Patrol_SendGoal_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      turtlebot3_msgs__action__Patrol_SendGoal_Response__fini(&array->data[i]);
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

turtlebot3_msgs__action__Patrol_SendGoal_Response__Sequence *
turtlebot3_msgs__action__Patrol_SendGoal_Response__Sequence__create(size_t size)
{
  turtlebot3_msgs__action__Patrol_SendGoal_Response__Sequence * array = (turtlebot3_msgs__action__Patrol_SendGoal_Response__Sequence *)malloc(sizeof(turtlebot3_msgs__action__Patrol_SendGoal_Response__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = turtlebot3_msgs__action__Patrol_SendGoal_Response__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
turtlebot3_msgs__action__Patrol_SendGoal_Response__Sequence__destroy(turtlebot3_msgs__action__Patrol_SendGoal_Response__Sequence * array)
{
  if (array) {
    turtlebot3_msgs__action__Patrol_SendGoal_Response__Sequence__fini(array);
  }
  free(array);
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"

bool
turtlebot3_msgs__action__Patrol_GetResult_Request__init(turtlebot3_msgs__action__Patrol_GetResult_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    turtlebot3_msgs__action__Patrol_GetResult_Request__fini(msg);
    return false;
  }
  return true;
}

void
turtlebot3_msgs__action__Patrol_GetResult_Request__fini(turtlebot3_msgs__action__Patrol_GetResult_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
}

turtlebot3_msgs__action__Patrol_GetResult_Request *
turtlebot3_msgs__action__Patrol_GetResult_Request__create()
{
  turtlebot3_msgs__action__Patrol_GetResult_Request * msg = (turtlebot3_msgs__action__Patrol_GetResult_Request *)malloc(sizeof(turtlebot3_msgs__action__Patrol_GetResult_Request));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(turtlebot3_msgs__action__Patrol_GetResult_Request));
  bool success = turtlebot3_msgs__action__Patrol_GetResult_Request__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
turtlebot3_msgs__action__Patrol_GetResult_Request__destroy(turtlebot3_msgs__action__Patrol_GetResult_Request * msg)
{
  if (msg) {
    turtlebot3_msgs__action__Patrol_GetResult_Request__fini(msg);
  }
  free(msg);
}


bool
turtlebot3_msgs__action__Patrol_GetResult_Request__Sequence__init(turtlebot3_msgs__action__Patrol_GetResult_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  turtlebot3_msgs__action__Patrol_GetResult_Request * data = NULL;
  if (size) {
    data = (turtlebot3_msgs__action__Patrol_GetResult_Request *)calloc(size, sizeof(turtlebot3_msgs__action__Patrol_GetResult_Request));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = turtlebot3_msgs__action__Patrol_GetResult_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        turtlebot3_msgs__action__Patrol_GetResult_Request__fini(&data[i - 1]);
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
turtlebot3_msgs__action__Patrol_GetResult_Request__Sequence__fini(turtlebot3_msgs__action__Patrol_GetResult_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      turtlebot3_msgs__action__Patrol_GetResult_Request__fini(&array->data[i]);
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

turtlebot3_msgs__action__Patrol_GetResult_Request__Sequence *
turtlebot3_msgs__action__Patrol_GetResult_Request__Sequence__create(size_t size)
{
  turtlebot3_msgs__action__Patrol_GetResult_Request__Sequence * array = (turtlebot3_msgs__action__Patrol_GetResult_Request__Sequence *)malloc(sizeof(turtlebot3_msgs__action__Patrol_GetResult_Request__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = turtlebot3_msgs__action__Patrol_GetResult_Request__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
turtlebot3_msgs__action__Patrol_GetResult_Request__Sequence__destroy(turtlebot3_msgs__action__Patrol_GetResult_Request__Sequence * array)
{
  if (array) {
    turtlebot3_msgs__action__Patrol_GetResult_Request__Sequence__fini(array);
  }
  free(array);
}


// Include directives for member types
// Member `result`
// already included above
// #include "turtlebot3_msgs/action/detail/patrol__functions.h"

bool
turtlebot3_msgs__action__Patrol_GetResult_Response__init(turtlebot3_msgs__action__Patrol_GetResult_Response * msg)
{
  if (!msg) {
    return false;
  }
  // status
  // result
  if (!turtlebot3_msgs__action__Patrol_Result__init(&msg->result)) {
    turtlebot3_msgs__action__Patrol_GetResult_Response__fini(msg);
    return false;
  }
  return true;
}

void
turtlebot3_msgs__action__Patrol_GetResult_Response__fini(turtlebot3_msgs__action__Patrol_GetResult_Response * msg)
{
  if (!msg) {
    return;
  }
  // status
  // result
  turtlebot3_msgs__action__Patrol_Result__fini(&msg->result);
}

turtlebot3_msgs__action__Patrol_GetResult_Response *
turtlebot3_msgs__action__Patrol_GetResult_Response__create()
{
  turtlebot3_msgs__action__Patrol_GetResult_Response * msg = (turtlebot3_msgs__action__Patrol_GetResult_Response *)malloc(sizeof(turtlebot3_msgs__action__Patrol_GetResult_Response));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(turtlebot3_msgs__action__Patrol_GetResult_Response));
  bool success = turtlebot3_msgs__action__Patrol_GetResult_Response__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
turtlebot3_msgs__action__Patrol_GetResult_Response__destroy(turtlebot3_msgs__action__Patrol_GetResult_Response * msg)
{
  if (msg) {
    turtlebot3_msgs__action__Patrol_GetResult_Response__fini(msg);
  }
  free(msg);
}


bool
turtlebot3_msgs__action__Patrol_GetResult_Response__Sequence__init(turtlebot3_msgs__action__Patrol_GetResult_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  turtlebot3_msgs__action__Patrol_GetResult_Response * data = NULL;
  if (size) {
    data = (turtlebot3_msgs__action__Patrol_GetResult_Response *)calloc(size, sizeof(turtlebot3_msgs__action__Patrol_GetResult_Response));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = turtlebot3_msgs__action__Patrol_GetResult_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        turtlebot3_msgs__action__Patrol_GetResult_Response__fini(&data[i - 1]);
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
turtlebot3_msgs__action__Patrol_GetResult_Response__Sequence__fini(turtlebot3_msgs__action__Patrol_GetResult_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      turtlebot3_msgs__action__Patrol_GetResult_Response__fini(&array->data[i]);
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

turtlebot3_msgs__action__Patrol_GetResult_Response__Sequence *
turtlebot3_msgs__action__Patrol_GetResult_Response__Sequence__create(size_t size)
{
  turtlebot3_msgs__action__Patrol_GetResult_Response__Sequence * array = (turtlebot3_msgs__action__Patrol_GetResult_Response__Sequence *)malloc(sizeof(turtlebot3_msgs__action__Patrol_GetResult_Response__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = turtlebot3_msgs__action__Patrol_GetResult_Response__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
turtlebot3_msgs__action__Patrol_GetResult_Response__Sequence__destroy(turtlebot3_msgs__action__Patrol_GetResult_Response__Sequence * array)
{
  if (array) {
    turtlebot3_msgs__action__Patrol_GetResult_Response__Sequence__fini(array);
  }
  free(array);
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `feedback`
// already included above
// #include "turtlebot3_msgs/action/detail/patrol__functions.h"

bool
turtlebot3_msgs__action__Patrol_FeedbackMessage__init(turtlebot3_msgs__action__Patrol_FeedbackMessage * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    turtlebot3_msgs__action__Patrol_FeedbackMessage__fini(msg);
    return false;
  }
  // feedback
  if (!turtlebot3_msgs__action__Patrol_Feedback__init(&msg->feedback)) {
    turtlebot3_msgs__action__Patrol_FeedbackMessage__fini(msg);
    return false;
  }
  return true;
}

void
turtlebot3_msgs__action__Patrol_FeedbackMessage__fini(turtlebot3_msgs__action__Patrol_FeedbackMessage * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // feedback
  turtlebot3_msgs__action__Patrol_Feedback__fini(&msg->feedback);
}

turtlebot3_msgs__action__Patrol_FeedbackMessage *
turtlebot3_msgs__action__Patrol_FeedbackMessage__create()
{
  turtlebot3_msgs__action__Patrol_FeedbackMessage * msg = (turtlebot3_msgs__action__Patrol_FeedbackMessage *)malloc(sizeof(turtlebot3_msgs__action__Patrol_FeedbackMessage));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(turtlebot3_msgs__action__Patrol_FeedbackMessage));
  bool success = turtlebot3_msgs__action__Patrol_FeedbackMessage__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
turtlebot3_msgs__action__Patrol_FeedbackMessage__destroy(turtlebot3_msgs__action__Patrol_FeedbackMessage * msg)
{
  if (msg) {
    turtlebot3_msgs__action__Patrol_FeedbackMessage__fini(msg);
  }
  free(msg);
}


bool
turtlebot3_msgs__action__Patrol_FeedbackMessage__Sequence__init(turtlebot3_msgs__action__Patrol_FeedbackMessage__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  turtlebot3_msgs__action__Patrol_FeedbackMessage * data = NULL;
  if (size) {
    data = (turtlebot3_msgs__action__Patrol_FeedbackMessage *)calloc(size, sizeof(turtlebot3_msgs__action__Patrol_FeedbackMessage));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = turtlebot3_msgs__action__Patrol_FeedbackMessage__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        turtlebot3_msgs__action__Patrol_FeedbackMessage__fini(&data[i - 1]);
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
turtlebot3_msgs__action__Patrol_FeedbackMessage__Sequence__fini(turtlebot3_msgs__action__Patrol_FeedbackMessage__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      turtlebot3_msgs__action__Patrol_FeedbackMessage__fini(&array->data[i]);
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

turtlebot3_msgs__action__Patrol_FeedbackMessage__Sequence *
turtlebot3_msgs__action__Patrol_FeedbackMessage__Sequence__create(size_t size)
{
  turtlebot3_msgs__action__Patrol_FeedbackMessage__Sequence * array = (turtlebot3_msgs__action__Patrol_FeedbackMessage__Sequence *)malloc(sizeof(turtlebot3_msgs__action__Patrol_FeedbackMessage__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = turtlebot3_msgs__action__Patrol_FeedbackMessage__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
turtlebot3_msgs__action__Patrol_FeedbackMessage__Sequence__destroy(turtlebot3_msgs__action__Patrol_FeedbackMessage__Sequence * array)
{
  if (array) {
    turtlebot3_msgs__action__Patrol_FeedbackMessage__Sequence__fini(array);
  }
  free(array);
}
