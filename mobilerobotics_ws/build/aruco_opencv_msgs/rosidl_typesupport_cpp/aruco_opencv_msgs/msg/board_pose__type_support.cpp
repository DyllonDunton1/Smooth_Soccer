// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from aruco_opencv_msgs:msg/BoardPose.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "aruco_opencv_msgs/msg/detail/board_pose__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace aruco_opencv_msgs
{

namespace msg
{

namespace rosidl_typesupport_cpp
{

typedef struct _BoardPose_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _BoardPose_type_support_ids_t;

static const _BoardPose_type_support_ids_t _BoardPose_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _BoardPose_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _BoardPose_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _BoardPose_type_support_symbol_names_t _BoardPose_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, aruco_opencv_msgs, msg, BoardPose)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, aruco_opencv_msgs, msg, BoardPose)),
  }
};

typedef struct _BoardPose_type_support_data_t
{
  void * data[2];
} _BoardPose_type_support_data_t;

static _BoardPose_type_support_data_t _BoardPose_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _BoardPose_message_typesupport_map = {
  2,
  "aruco_opencv_msgs",
  &_BoardPose_message_typesupport_ids.typesupport_identifier[0],
  &_BoardPose_message_typesupport_symbol_names.symbol_name[0],
  &_BoardPose_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t BoardPose_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_BoardPose_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace msg

}  // namespace aruco_opencv_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<aruco_opencv_msgs::msg::BoardPose>()
{
  return &::aruco_opencv_msgs::msg::rosidl_typesupport_cpp::BoardPose_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, aruco_opencv_msgs, msg, BoardPose)() {
  return get_message_type_support_handle<aruco_opencv_msgs::msg::BoardPose>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp
