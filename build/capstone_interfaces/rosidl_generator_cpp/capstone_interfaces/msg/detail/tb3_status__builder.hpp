// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from capstone_interfaces:msg/TB3Status.idl
// generated code does not contain a copyright notice

#ifndef CAPSTONE_INTERFACES__MSG__DETAIL__TB3_STATUS__BUILDER_HPP_
#define CAPSTONE_INTERFACES__MSG__DETAIL__TB3_STATUS__BUILDER_HPP_

#include "capstone_interfaces/msg/detail/tb3_status__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace capstone_interfaces
{

namespace msg
{

namespace builder
{

class Init_TB3Status_hit_wall
{
public:
  explicit Init_TB3Status_hit_wall(::capstone_interfaces::msg::TB3Status & msg)
  : msg_(msg)
  {}
  ::capstone_interfaces::msg::TB3Status hit_wall(::capstone_interfaces::msg::TB3Status::_hit_wall_type arg)
  {
    msg_.hit_wall = std::move(arg);
    return std::move(msg_);
  }

private:
  ::capstone_interfaces::msg::TB3Status msg_;
};

class Init_TB3Status_lidar_data
{
public:
  Init_TB3Status_lidar_data()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TB3Status_hit_wall lidar_data(::capstone_interfaces::msg::TB3Status::_lidar_data_type arg)
  {
    msg_.lidar_data = std::move(arg);
    return Init_TB3Status_hit_wall(msg_);
  }

private:
  ::capstone_interfaces::msg::TB3Status msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::capstone_interfaces::msg::TB3Status>()
{
  return capstone_interfaces::msg::builder::Init_TB3Status_lidar_data();
}

}  // namespace capstone_interfaces

#endif  // CAPSTONE_INTERFACES__MSG__DETAIL__TB3_STATUS__BUILDER_HPP_
