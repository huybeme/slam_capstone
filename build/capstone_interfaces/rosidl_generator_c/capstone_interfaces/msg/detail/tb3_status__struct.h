// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from capstone_interfaces:msg/TB3Status.idl
// generated code does not contain a copyright notice

#ifndef CAPSTONE_INTERFACES__MSG__DETAIL__TB3_STATUS__STRUCT_H_
#define CAPSTONE_INTERFACES__MSG__DETAIL__TB3_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/TB3Status in the package capstone_interfaces.
typedef struct capstone_interfaces__msg__TB3Status
{
  float lidar_data[8];
  bool hit_wall;
} capstone_interfaces__msg__TB3Status;

// Struct for a sequence of capstone_interfaces__msg__TB3Status.
typedef struct capstone_interfaces__msg__TB3Status__Sequence
{
  capstone_interfaces__msg__TB3Status * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} capstone_interfaces__msg__TB3Status__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CAPSTONE_INTERFACES__MSG__DETAIL__TB3_STATUS__STRUCT_H_
