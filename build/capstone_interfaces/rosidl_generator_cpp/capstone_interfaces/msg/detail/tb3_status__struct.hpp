// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from capstone_interfaces:msg/TB3Status.idl
// generated code does not contain a copyright notice

#ifndef CAPSTONE_INTERFACES__MSG__DETAIL__TB3_STATUS__STRUCT_HPP_
#define CAPSTONE_INTERFACES__MSG__DETAIL__TB3_STATUS__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__capstone_interfaces__msg__TB3Status __attribute__((deprecated))
#else
# define DEPRECATED__capstone_interfaces__msg__TB3Status __declspec(deprecated)
#endif

namespace capstone_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TB3Status_
{
  using Type = TB3Status_<ContainerAllocator>;

  explicit TB3Status_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->hit_wall = false;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      std::fill<typename std::array<float, 8>::iterator, float>(this->lidar_data.begin(), this->lidar_data.end(), 0.0f);
      this->hit_wall = false;
    }
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<float, 8>::iterator, float>(this->lidar_data.begin(), this->lidar_data.end(), 0.0f);
    }
  }

  explicit TB3Status_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : lidar_data(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->hit_wall = false;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      std::fill<typename std::array<float, 8>::iterator, float>(this->lidar_data.begin(), this->lidar_data.end(), 0.0f);
      this->hit_wall = false;
    }
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<float, 8>::iterator, float>(this->lidar_data.begin(), this->lidar_data.end(), 0.0f);
    }
  }

  // field types and members
  using _lidar_data_type =
    std::array<float, 8>;
  _lidar_data_type lidar_data;
  using _hit_wall_type =
    bool;
  _hit_wall_type hit_wall;

  // setters for named parameter idiom
  Type & set__lidar_data(
    const std::array<float, 8> & _arg)
  {
    this->lidar_data = _arg;
    return *this;
  }
  Type & set__hit_wall(
    const bool & _arg)
  {
    this->hit_wall = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    capstone_interfaces::msg::TB3Status_<ContainerAllocator> *;
  using ConstRawPtr =
    const capstone_interfaces::msg::TB3Status_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<capstone_interfaces::msg::TB3Status_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<capstone_interfaces::msg::TB3Status_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      capstone_interfaces::msg::TB3Status_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<capstone_interfaces::msg::TB3Status_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      capstone_interfaces::msg::TB3Status_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<capstone_interfaces::msg::TB3Status_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<capstone_interfaces::msg::TB3Status_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<capstone_interfaces::msg::TB3Status_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__capstone_interfaces__msg__TB3Status
    std::shared_ptr<capstone_interfaces::msg::TB3Status_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__capstone_interfaces__msg__TB3Status
    std::shared_ptr<capstone_interfaces::msg::TB3Status_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TB3Status_ & other) const
  {
    if (this->lidar_data != other.lidar_data) {
      return false;
    }
    if (this->hit_wall != other.hit_wall) {
      return false;
    }
    return true;
  }
  bool operator!=(const TB3Status_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TB3Status_

// alias to use template instance with default allocator
using TB3Status =
  capstone_interfaces::msg::TB3Status_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace capstone_interfaces

#endif  // CAPSTONE_INTERFACES__MSG__DETAIL__TB3_STATUS__STRUCT_HPP_
