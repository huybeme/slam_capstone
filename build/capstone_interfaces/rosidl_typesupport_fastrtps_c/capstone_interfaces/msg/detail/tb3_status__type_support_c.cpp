// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from capstone_interfaces:msg/TB3Status.idl
// generated code does not contain a copyright notice
#include "capstone_interfaces/msg/detail/tb3_status__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "capstone_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "capstone_interfaces/msg/detail/tb3_status__struct.h"
#include "capstone_interfaces/msg/detail/tb3_status__functions.h"
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


using _TB3Status__ros_msg_type = capstone_interfaces__msg__TB3Status;

static bool _TB3Status__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _TB3Status__ros_msg_type * ros_message = static_cast<const _TB3Status__ros_msg_type *>(untyped_ros_message);
  // Field name: lidar_data
  {
    size_t size = 8;
    auto array_ptr = ros_message->lidar_data;
    cdr.serializeArray(array_ptr, size);
  }

  return true;
}

static bool _TB3Status__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _TB3Status__ros_msg_type * ros_message = static_cast<_TB3Status__ros_msg_type *>(untyped_ros_message);
  // Field name: lidar_data
  {
    size_t size = 8;
    auto array_ptr = ros_message->lidar_data;
    cdr.deserializeArray(array_ptr, size);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_capstone_interfaces
size_t get_serialized_size_capstone_interfaces__msg__TB3Status(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _TB3Status__ros_msg_type * ros_message = static_cast<const _TB3Status__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name lidar_data
  {
    size_t array_size = 8;
    auto array_ptr = ros_message->lidar_data;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _TB3Status__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_capstone_interfaces__msg__TB3Status(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_capstone_interfaces
size_t max_serialized_size_capstone_interfaces__msg__TB3Status(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: lidar_data
  {
    size_t array_size = 8;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static size_t _TB3Status__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_capstone_interfaces__msg__TB3Status(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_TB3Status = {
  "capstone_interfaces::msg",
  "TB3Status",
  _TB3Status__cdr_serialize,
  _TB3Status__cdr_deserialize,
  _TB3Status__get_serialized_size,
  _TB3Status__max_serialized_size
};

static rosidl_message_type_support_t _TB3Status__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_TB3Status,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, capstone_interfaces, msg, TB3Status)() {
  return &_TB3Status__type_support;
}

#if defined(__cplusplus)
}
#endif
