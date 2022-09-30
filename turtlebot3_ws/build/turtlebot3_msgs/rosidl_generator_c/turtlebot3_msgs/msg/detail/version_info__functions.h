// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from turtlebot3_msgs:msg/VersionInfo.idl
// generated code does not contain a copyright notice

#ifndef TURTLEBOT3_MSGS__MSG__DETAIL__VERSION_INFO__FUNCTIONS_H_
#define TURTLEBOT3_MSGS__MSG__DETAIL__VERSION_INFO__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "turtlebot3_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "turtlebot3_msgs/msg/detail/version_info__struct.h"

/// Initialize msg/VersionInfo message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * turtlebot3_msgs__msg__VersionInfo
 * )) before or use
 * turtlebot3_msgs__msg__VersionInfo__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
bool
turtlebot3_msgs__msg__VersionInfo__init(turtlebot3_msgs__msg__VersionInfo * msg);

/// Finalize msg/VersionInfo message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
void
turtlebot3_msgs__msg__VersionInfo__fini(turtlebot3_msgs__msg__VersionInfo * msg);

/// Create msg/VersionInfo message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * turtlebot3_msgs__msg__VersionInfo__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
turtlebot3_msgs__msg__VersionInfo *
turtlebot3_msgs__msg__VersionInfo__create();

/// Destroy msg/VersionInfo message.
/**
 * It calls
 * turtlebot3_msgs__msg__VersionInfo__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
void
turtlebot3_msgs__msg__VersionInfo__destroy(turtlebot3_msgs__msg__VersionInfo * msg);


/// Initialize array of msg/VersionInfo messages.
/**
 * It allocates the memory for the number of elements and calls
 * turtlebot3_msgs__msg__VersionInfo__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
bool
turtlebot3_msgs__msg__VersionInfo__Sequence__init(turtlebot3_msgs__msg__VersionInfo__Sequence * array, size_t size);

/// Finalize array of msg/VersionInfo messages.
/**
 * It calls
 * turtlebot3_msgs__msg__VersionInfo__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
void
turtlebot3_msgs__msg__VersionInfo__Sequence__fini(turtlebot3_msgs__msg__VersionInfo__Sequence * array);

/// Create array of msg/VersionInfo messages.
/**
 * It allocates the memory for the array and calls
 * turtlebot3_msgs__msg__VersionInfo__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
turtlebot3_msgs__msg__VersionInfo__Sequence *
turtlebot3_msgs__msg__VersionInfo__Sequence__create(size_t size);

/// Destroy array of msg/VersionInfo messages.
/**
 * It calls
 * turtlebot3_msgs__msg__VersionInfo__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
void
turtlebot3_msgs__msg__VersionInfo__Sequence__destroy(turtlebot3_msgs__msg__VersionInfo__Sequence * array);

#ifdef __cplusplus
}
#endif

#endif  // TURTLEBOT3_MSGS__MSG__DETAIL__VERSION_INFO__FUNCTIONS_H_
