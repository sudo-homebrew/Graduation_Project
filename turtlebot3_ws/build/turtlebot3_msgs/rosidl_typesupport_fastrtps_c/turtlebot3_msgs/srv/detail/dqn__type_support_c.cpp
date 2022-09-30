// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from turtlebot3_msgs:srv/Dqn.idl
// generated code does not contain a copyright notice
#include "turtlebot3_msgs/srv/detail/dqn__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "turtlebot3_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "turtlebot3_msgs/srv/detail/dqn__struct.h"
#include "turtlebot3_msgs/srv/detail/dqn__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _Dqn_Request__ros_msg_type = turtlebot3_msgs__srv__Dqn_Request;

static bool _Dqn_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _Dqn_Request__ros_msg_type * ros_message = static_cast<const _Dqn_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: action
  {
    cdr << ros_message->action;
  }

  // Field name: init
  {
    cdr << (ros_message->init ? true : false);
  }

  return true;
}

static bool _Dqn_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _Dqn_Request__ros_msg_type * ros_message = static_cast<_Dqn_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: action
  {
    cdr >> ros_message->action;
  }

  // Field name: init
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->init = tmp ? true : false;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_turtlebot3_msgs
size_t get_serialized_size_turtlebot3_msgs__srv__Dqn_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Dqn_Request__ros_msg_type * ros_message = static_cast<const _Dqn_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name action
  {
    size_t item_size = sizeof(ros_message->action);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name init
  {
    size_t item_size = sizeof(ros_message->init);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _Dqn_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_turtlebot3_msgs__srv__Dqn_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_turtlebot3_msgs
size_t max_serialized_size_turtlebot3_msgs__srv__Dqn_Request(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: action
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: init
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  return current_alignment - initial_alignment;
}

static size_t _Dqn_Request__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_turtlebot3_msgs__srv__Dqn_Request(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_Dqn_Request = {
  "turtlebot3_msgs::srv",
  "Dqn_Request",
  _Dqn_Request__cdr_serialize,
  _Dqn_Request__cdr_deserialize,
  _Dqn_Request__get_serialized_size,
  _Dqn_Request__max_serialized_size
};

static rosidl_message_type_support_t _Dqn_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_Dqn_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, turtlebot3_msgs, srv, Dqn_Request)() {
  return &_Dqn_Request__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "turtlebot3_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "turtlebot3_msgs/srv/detail/dqn__struct.h"
// already included above
// #include "turtlebot3_msgs/srv/detail/dqn__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/primitives_sequence.h"  // state
#include "rosidl_runtime_c/primitives_sequence_functions.h"  // state

// forward declare type support functions


using _Dqn_Response__ros_msg_type = turtlebot3_msgs__srv__Dqn_Response;

static bool _Dqn_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _Dqn_Response__ros_msg_type * ros_message = static_cast<const _Dqn_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: state
  {
    size_t size = ros_message->state.size;
    auto array_ptr = ros_message->state.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: reward
  {
    cdr << ros_message->reward;
  }

  // Field name: done
  {
    cdr << (ros_message->done ? true : false);
  }

  return true;
}

static bool _Dqn_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _Dqn_Response__ros_msg_type * ros_message = static_cast<_Dqn_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: state
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->state.data) {
      rosidl_runtime_c__float__Sequence__fini(&ros_message->state);
    }
    if (!rosidl_runtime_c__float__Sequence__init(&ros_message->state, size)) {
      return "failed to create array for field 'state'";
    }
    auto array_ptr = ros_message->state.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: reward
  {
    cdr >> ros_message->reward;
  }

  // Field name: done
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->done = tmp ? true : false;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_turtlebot3_msgs
size_t get_serialized_size_turtlebot3_msgs__srv__Dqn_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Dqn_Response__ros_msg_type * ros_message = static_cast<const _Dqn_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name state
  {
    size_t array_size = ros_message->state.size;
    auto array_ptr = ros_message->state.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name reward
  {
    size_t item_size = sizeof(ros_message->reward);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name done
  {
    size_t item_size = sizeof(ros_message->done);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _Dqn_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_turtlebot3_msgs__srv__Dqn_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_turtlebot3_msgs
size_t max_serialized_size_turtlebot3_msgs__srv__Dqn_Response(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: state
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: reward
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: done
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  return current_alignment - initial_alignment;
}

static size_t _Dqn_Response__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_turtlebot3_msgs__srv__Dqn_Response(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_Dqn_Response = {
  "turtlebot3_msgs::srv",
  "Dqn_Response",
  _Dqn_Response__cdr_serialize,
  _Dqn_Response__cdr_deserialize,
  _Dqn_Response__get_serialized_size,
  _Dqn_Response__max_serialized_size
};

static rosidl_message_type_support_t _Dqn_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_Dqn_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, turtlebot3_msgs, srv, Dqn_Response)() {
  return &_Dqn_Response__type_support;
}

#if defined(__cplusplus)
}
#endif

#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "turtlebot3_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "turtlebot3_msgs/srv/dqn.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t Dqn__callbacks = {
  "turtlebot3_msgs::srv",
  "Dqn",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, turtlebot3_msgs, srv, Dqn_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, turtlebot3_msgs, srv, Dqn_Response)(),
};

static rosidl_service_type_support_t Dqn__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &Dqn__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, turtlebot3_msgs, srv, Dqn)() {
  return &Dqn__handle;
}

#if defined(__cplusplus)
}
#endif
