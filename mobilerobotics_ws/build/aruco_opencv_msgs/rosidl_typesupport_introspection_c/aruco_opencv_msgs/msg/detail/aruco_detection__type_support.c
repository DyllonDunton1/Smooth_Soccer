// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from aruco_opencv_msgs:msg/ArucoDetection.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "aruco_opencv_msgs/msg/detail/aruco_detection__rosidl_typesupport_introspection_c.h"
#include "aruco_opencv_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "aruco_opencv_msgs/msg/detail/aruco_detection__functions.h"
#include "aruco_opencv_msgs/msg/detail/aruco_detection__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `markers`
#include "aruco_opencv_msgs/msg/marker_pose.h"
// Member `markers`
#include "aruco_opencv_msgs/msg/detail/marker_pose__rosidl_typesupport_introspection_c.h"
// Member `boards`
#include "aruco_opencv_msgs/msg/board_pose.h"
// Member `boards`
#include "aruco_opencv_msgs/msg/detail/board_pose__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void ArucoDetection__rosidl_typesupport_introspection_c__ArucoDetection_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  aruco_opencv_msgs__msg__ArucoDetection__init(message_memory);
}

void ArucoDetection__rosidl_typesupport_introspection_c__ArucoDetection_fini_function(void * message_memory)
{
  aruco_opencv_msgs__msg__ArucoDetection__fini(message_memory);
}

size_t ArucoDetection__rosidl_typesupport_introspection_c__size_function__MarkerPose__markers(
  const void * untyped_member)
{
  const aruco_opencv_msgs__msg__MarkerPose__Sequence * member =
    (const aruco_opencv_msgs__msg__MarkerPose__Sequence *)(untyped_member);
  return member->size;
}

const void * ArucoDetection__rosidl_typesupport_introspection_c__get_const_function__MarkerPose__markers(
  const void * untyped_member, size_t index)
{
  const aruco_opencv_msgs__msg__MarkerPose__Sequence * member =
    (const aruco_opencv_msgs__msg__MarkerPose__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ArucoDetection__rosidl_typesupport_introspection_c__get_function__MarkerPose__markers(
  void * untyped_member, size_t index)
{
  aruco_opencv_msgs__msg__MarkerPose__Sequence * member =
    (aruco_opencv_msgs__msg__MarkerPose__Sequence *)(untyped_member);
  return &member->data[index];
}

bool ArucoDetection__rosidl_typesupport_introspection_c__resize_function__MarkerPose__markers(
  void * untyped_member, size_t size)
{
  aruco_opencv_msgs__msg__MarkerPose__Sequence * member =
    (aruco_opencv_msgs__msg__MarkerPose__Sequence *)(untyped_member);
  aruco_opencv_msgs__msg__MarkerPose__Sequence__fini(member);
  return aruco_opencv_msgs__msg__MarkerPose__Sequence__init(member, size);
}

size_t ArucoDetection__rosidl_typesupport_introspection_c__size_function__BoardPose__boards(
  const void * untyped_member)
{
  const aruco_opencv_msgs__msg__BoardPose__Sequence * member =
    (const aruco_opencv_msgs__msg__BoardPose__Sequence *)(untyped_member);
  return member->size;
}

const void * ArucoDetection__rosidl_typesupport_introspection_c__get_const_function__BoardPose__boards(
  const void * untyped_member, size_t index)
{
  const aruco_opencv_msgs__msg__BoardPose__Sequence * member =
    (const aruco_opencv_msgs__msg__BoardPose__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ArucoDetection__rosidl_typesupport_introspection_c__get_function__BoardPose__boards(
  void * untyped_member, size_t index)
{
  aruco_opencv_msgs__msg__BoardPose__Sequence * member =
    (aruco_opencv_msgs__msg__BoardPose__Sequence *)(untyped_member);
  return &member->data[index];
}

bool ArucoDetection__rosidl_typesupport_introspection_c__resize_function__BoardPose__boards(
  void * untyped_member, size_t size)
{
  aruco_opencv_msgs__msg__BoardPose__Sequence * member =
    (aruco_opencv_msgs__msg__BoardPose__Sequence *)(untyped_member);
  aruco_opencv_msgs__msg__BoardPose__Sequence__fini(member);
  return aruco_opencv_msgs__msg__BoardPose__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember ArucoDetection__rosidl_typesupport_introspection_c__ArucoDetection_message_member_array[3] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(aruco_opencv_msgs__msg__ArucoDetection, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "markers",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(aruco_opencv_msgs__msg__ArucoDetection, markers),  // bytes offset in struct
    NULL,  // default value
    ArucoDetection__rosidl_typesupport_introspection_c__size_function__MarkerPose__markers,  // size() function pointer
    ArucoDetection__rosidl_typesupport_introspection_c__get_const_function__MarkerPose__markers,  // get_const(index) function pointer
    ArucoDetection__rosidl_typesupport_introspection_c__get_function__MarkerPose__markers,  // get(index) function pointer
    ArucoDetection__rosidl_typesupport_introspection_c__resize_function__MarkerPose__markers  // resize(index) function pointer
  },
  {
    "boards",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(aruco_opencv_msgs__msg__ArucoDetection, boards),  // bytes offset in struct
    NULL,  // default value
    ArucoDetection__rosidl_typesupport_introspection_c__size_function__BoardPose__boards,  // size() function pointer
    ArucoDetection__rosidl_typesupport_introspection_c__get_const_function__BoardPose__boards,  // get_const(index) function pointer
    ArucoDetection__rosidl_typesupport_introspection_c__get_function__BoardPose__boards,  // get(index) function pointer
    ArucoDetection__rosidl_typesupport_introspection_c__resize_function__BoardPose__boards  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ArucoDetection__rosidl_typesupport_introspection_c__ArucoDetection_message_members = {
  "aruco_opencv_msgs__msg",  // message namespace
  "ArucoDetection",  // message name
  3,  // number of fields
  sizeof(aruco_opencv_msgs__msg__ArucoDetection),
  ArucoDetection__rosidl_typesupport_introspection_c__ArucoDetection_message_member_array,  // message members
  ArucoDetection__rosidl_typesupport_introspection_c__ArucoDetection_init_function,  // function to initialize message memory (memory has to be allocated)
  ArucoDetection__rosidl_typesupport_introspection_c__ArucoDetection_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ArucoDetection__rosidl_typesupport_introspection_c__ArucoDetection_message_type_support_handle = {
  0,
  &ArucoDetection__rosidl_typesupport_introspection_c__ArucoDetection_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_aruco_opencv_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, aruco_opencv_msgs, msg, ArucoDetection)() {
  ArucoDetection__rosidl_typesupport_introspection_c__ArucoDetection_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  ArucoDetection__rosidl_typesupport_introspection_c__ArucoDetection_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, aruco_opencv_msgs, msg, MarkerPose)();
  ArucoDetection__rosidl_typesupport_introspection_c__ArucoDetection_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, aruco_opencv_msgs, msg, BoardPose)();
  if (!ArucoDetection__rosidl_typesupport_introspection_c__ArucoDetection_message_type_support_handle.typesupport_identifier) {
    ArucoDetection__rosidl_typesupport_introspection_c__ArucoDetection_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ArucoDetection__rosidl_typesupport_introspection_c__ArucoDetection_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
