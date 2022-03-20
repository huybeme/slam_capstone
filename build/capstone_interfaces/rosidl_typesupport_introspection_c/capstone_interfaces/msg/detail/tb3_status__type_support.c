// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from capstone_interfaces:msg/TB3Status.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "capstone_interfaces/msg/detail/tb3_status__rosidl_typesupport_introspection_c.h"
#include "capstone_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "capstone_interfaces/msg/detail/tb3_status__functions.h"
#include "capstone_interfaces/msg/detail/tb3_status__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void TB3Status__rosidl_typesupport_introspection_c__TB3Status_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  capstone_interfaces__msg__TB3Status__init(message_memory);
}

void TB3Status__rosidl_typesupport_introspection_c__TB3Status_fini_function(void * message_memory)
{
  capstone_interfaces__msg__TB3Status__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember TB3Status__rosidl_typesupport_introspection_c__TB3Status_message_member_array[2] = {
  {
    "lidar_data",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    8,  // array size
    false,  // is upper bound
    offsetof(capstone_interfaces__msg__TB3Status, lidar_data),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "hit_wall",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(capstone_interfaces__msg__TB3Status, hit_wall),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers TB3Status__rosidl_typesupport_introspection_c__TB3Status_message_members = {
  "capstone_interfaces__msg",  // message namespace
  "TB3Status",  // message name
  2,  // number of fields
  sizeof(capstone_interfaces__msg__TB3Status),
  TB3Status__rosidl_typesupport_introspection_c__TB3Status_message_member_array,  // message members
  TB3Status__rosidl_typesupport_introspection_c__TB3Status_init_function,  // function to initialize message memory (memory has to be allocated)
  TB3Status__rosidl_typesupport_introspection_c__TB3Status_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t TB3Status__rosidl_typesupport_introspection_c__TB3Status_message_type_support_handle = {
  0,
  &TB3Status__rosidl_typesupport_introspection_c__TB3Status_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_capstone_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, capstone_interfaces, msg, TB3Status)() {
  if (!TB3Status__rosidl_typesupport_introspection_c__TB3Status_message_type_support_handle.typesupport_identifier) {
    TB3Status__rosidl_typesupport_introspection_c__TB3Status_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &TB3Status__rosidl_typesupport_introspection_c__TB3Status_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
