// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from capstone_interfaces:msg/TB3Status.idl
// generated code does not contain a copyright notice
#include "capstone_interfaces/msg/detail/tb3_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
capstone_interfaces__msg__TB3Status__init(capstone_interfaces__msg__TB3Status * msg)
{
  if (!msg) {
    return false;
  }
  // lidar_data
  // hit_wall
  msg->hit_wall = false;
  return true;
}

void
capstone_interfaces__msg__TB3Status__fini(capstone_interfaces__msg__TB3Status * msg)
{
  if (!msg) {
    return;
  }
  // lidar_data
  // hit_wall
}

capstone_interfaces__msg__TB3Status *
capstone_interfaces__msg__TB3Status__create()
{
  capstone_interfaces__msg__TB3Status * msg = (capstone_interfaces__msg__TB3Status *)malloc(sizeof(capstone_interfaces__msg__TB3Status));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(capstone_interfaces__msg__TB3Status));
  bool success = capstone_interfaces__msg__TB3Status__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
capstone_interfaces__msg__TB3Status__destroy(capstone_interfaces__msg__TB3Status * msg)
{
  if (msg) {
    capstone_interfaces__msg__TB3Status__fini(msg);
  }
  free(msg);
}


bool
capstone_interfaces__msg__TB3Status__Sequence__init(capstone_interfaces__msg__TB3Status__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  capstone_interfaces__msg__TB3Status * data = NULL;
  if (size) {
    data = (capstone_interfaces__msg__TB3Status *)calloc(size, sizeof(capstone_interfaces__msg__TB3Status));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = capstone_interfaces__msg__TB3Status__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        capstone_interfaces__msg__TB3Status__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
capstone_interfaces__msg__TB3Status__Sequence__fini(capstone_interfaces__msg__TB3Status__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      capstone_interfaces__msg__TB3Status__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

capstone_interfaces__msg__TB3Status__Sequence *
capstone_interfaces__msg__TB3Status__Sequence__create(size_t size)
{
  capstone_interfaces__msg__TB3Status__Sequence * array = (capstone_interfaces__msg__TB3Status__Sequence *)malloc(sizeof(capstone_interfaces__msg__TB3Status__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = capstone_interfaces__msg__TB3Status__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
capstone_interfaces__msg__TB3Status__Sequence__destroy(capstone_interfaces__msg__TB3Status__Sequence * array)
{
  if (array) {
    capstone_interfaces__msg__TB3Status__Sequence__fini(array);
  }
  free(array);
}
