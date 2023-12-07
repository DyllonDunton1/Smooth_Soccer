// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from aruco_opencv_msgs:msg/BoardPose.idl
// generated code does not contain a copyright notice
#include "aruco_opencv_msgs/msg/detail/board_pose__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "aruco_opencv_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "aruco_opencv_msgs/msg/detail/board_pose__struct.h"
#include "aruco_opencv_msgs/msg/detail/board_pose__functions.h"
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

#include "geometry_msgs/msg/detail/pose__functions.h"  // pose
#include "rosidl_runtime_c/string.h"  // board_name
#include "rosidl_runtime_c/string_functions.h"  // board_name

// forward declare type support functions
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_aruco_opencv_msgs
size_t get_serialized_size_geometry_msgs__msg__Pose(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_aruco_opencv_msgs
size_t max_serialized_size_geometry_msgs__msg__Pose(
  bool & full_bounded,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_aruco_opencv_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Pose)();


using _BoardPose__ros_msg_type = aruco_opencv_msgs__msg__BoardPose;

static bool _BoardPose__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _BoardPose__ros_msg_type * ros_message = static_cast<const _BoardPose__ros_msg_type *>(untyped_ros_message);
  // Field name: board_name
  {
    const rosidl_runtime_c__String * str = &ros_message->board_name;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: pose
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Pose
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->pose, cdr))
    {
      return false;
    }
  }

  return true;
}

static bool _BoardPose__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _BoardPose__ros_msg_type * ros_message = static_cast<_BoardPose__ros_msg_type *>(untyped_ros_message);
  // Field name: board_name
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->board_name.data) {
      rosidl_runtime_c__String__init(&ros_message->board_name);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->board_name,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'board_name'\n");
      return false;
    }
  }

  // Field name: pose
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Pose
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->pose))
    {
      return false;
    }
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_aruco_opencv_msgs
size_t get_serialized_size_aruco_opencv_msgs__msg__BoardPose(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _BoardPose__ros_msg_type * ros_message = static_cast<const _BoardPose__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name board_name
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->board_name.size + 1);
  // field.name pose

  current_alignment += get_serialized_size_geometry_msgs__msg__Pose(
    &(ros_message->pose), current_alignment);

  return current_alignment - initial_alignment;
}

static uint32_t _BoardPose__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_aruco_opencv_msgs__msg__BoardPose(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_aruco_opencv_msgs
size_t max_serialized_size_aruco_opencv_msgs__msg__BoardPose(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: board_name
  {
    size_t array_size = 1;

    full_bounded = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: pose
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_geometry_msgs__msg__Pose(
        full_bounded, current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

static size_t _BoardPose__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_aruco_opencv_msgs__msg__BoardPose(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_BoardPose = {
  "aruco_opencv_msgs::msg",
  "BoardPose",
  _BoardPose__cdr_serialize,
  _BoardPose__cdr_deserialize,
  _BoardPose__get_serialized_size,
  _BoardPose__max_serialized_size
};

static rosidl_message_type_support_t _BoardPose__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_BoardPose,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, aruco_opencv_msgs, msg, BoardPose)() {
  return &_BoardPose__type_support;
}

#if defined(__cplusplus)
}
#endif
