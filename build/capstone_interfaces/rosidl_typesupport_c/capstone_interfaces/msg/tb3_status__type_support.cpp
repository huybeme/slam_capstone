// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from capstone_interfaces:msg/TB3Status.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "capstone_interfaces/msg/rosidl_typesupport_c__visibility_control.h"
#include "capstone_interfaces/msg/detail/tb3_status__struct.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/message_type_support_dispatch.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace capstone_interfaces
{

namespace msg
{

namespace rosidl_typesupport_c
{

typedef struct _TB3Status_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _TB3Status_type_support_ids_t;

static const _TB3Status_type_support_ids_t _TB3Status_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _TB3Status_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _TB3Status_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _TB3Status_type_support_symbol_names_t _TB3Status_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, capstone_interfaces, msg, TB3Status)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, capstone_interfaces, msg, TB3Status)),
  }
};

typedef struct _TB3Status_type_support_data_t
{
  void * data[2];
} _TB3Status_type_support_data_t;

static _TB3Status_type_support_data_t _TB3Status_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _TB3Status_message_typesupport_map = {
  2,
  "capstone_interfaces",
  &_TB3Status_message_typesupport_ids.typesupport_identifier[0],
  &_TB3Status_message_typesupport_symbol_names.symbol_name[0],
  &_TB3Status_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t TB3Status_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_TB3Status_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace msg

}  // namespace capstone_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_C_EXPORT_capstone_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, capstone_interfaces, msg, TB3Status)() {
  return &::capstone_interfaces::msg::rosidl_typesupport_c::TB3Status_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
