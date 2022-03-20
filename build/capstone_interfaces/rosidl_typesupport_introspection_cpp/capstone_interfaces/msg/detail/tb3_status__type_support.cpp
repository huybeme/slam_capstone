// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from capstone_interfaces:msg/TB3Status.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "capstone_interfaces/msg/detail/tb3_status__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace capstone_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void TB3Status_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) capstone_interfaces::msg::TB3Status(_init);
}

void TB3Status_fini_function(void * message_memory)
{
  auto typed_message = static_cast<capstone_interfaces::msg::TB3Status *>(message_memory);
  typed_message->~TB3Status();
}

size_t size_function__TB3Status__lidar_data(const void * untyped_member)
{
  (void)untyped_member;
  return 8;
}

const void * get_const_function__TB3Status__lidar_data(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 8> *>(untyped_member);
  return &member[index];
}

void * get_function__TB3Status__lidar_data(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 8> *>(untyped_member);
  return &member[index];
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember TB3Status_message_member_array[2] = {
  {
    "lidar_data",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    8,  // array size
    false,  // is upper bound
    offsetof(capstone_interfaces::msg::TB3Status, lidar_data),  // bytes offset in struct
    nullptr,  // default value
    size_function__TB3Status__lidar_data,  // size() function pointer
    get_const_function__TB3Status__lidar_data,  // get_const(index) function pointer
    get_function__TB3Status__lidar_data,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "hit_wall",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(capstone_interfaces::msg::TB3Status, hit_wall),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers TB3Status_message_members = {
  "capstone_interfaces::msg",  // message namespace
  "TB3Status",  // message name
  2,  // number of fields
  sizeof(capstone_interfaces::msg::TB3Status),
  TB3Status_message_member_array,  // message members
  TB3Status_init_function,  // function to initialize message memory (memory has to be allocated)
  TB3Status_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t TB3Status_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &TB3Status_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace capstone_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<capstone_interfaces::msg::TB3Status>()
{
  return &::capstone_interfaces::msg::rosidl_typesupport_introspection_cpp::TB3Status_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, capstone_interfaces, msg, TB3Status)() {
  return &::capstone_interfaces::msg::rosidl_typesupport_introspection_cpp::TB3Status_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
