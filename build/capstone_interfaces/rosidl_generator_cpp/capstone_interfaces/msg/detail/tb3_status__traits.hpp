// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from capstone_interfaces:msg/TB3Status.idl
// generated code does not contain a copyright notice

#ifndef CAPSTONE_INTERFACES__MSG__DETAIL__TB3_STATUS__TRAITS_HPP_
#define CAPSTONE_INTERFACES__MSG__DETAIL__TB3_STATUS__TRAITS_HPP_

#include "capstone_interfaces/msg/detail/tb3_status__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<capstone_interfaces::msg::TB3Status>()
{
  return "capstone_interfaces::msg::TB3Status";
}

template<>
inline const char * name<capstone_interfaces::msg::TB3Status>()
{
  return "capstone_interfaces/msg/TB3Status";
}

template<>
struct has_fixed_size<capstone_interfaces::msg::TB3Status>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<capstone_interfaces::msg::TB3Status>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<capstone_interfaces::msg::TB3Status>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CAPSTONE_INTERFACES__MSG__DETAIL__TB3_STATUS__TRAITS_HPP_
