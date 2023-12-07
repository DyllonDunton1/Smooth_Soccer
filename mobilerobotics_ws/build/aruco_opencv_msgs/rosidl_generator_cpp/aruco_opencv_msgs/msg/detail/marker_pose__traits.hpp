// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from aruco_opencv_msgs:msg/MarkerPose.idl
// generated code does not contain a copyright notice

#ifndef ARUCO_OPENCV_MSGS__MSG__DETAIL__MARKER_POSE__TRAITS_HPP_
#define ARUCO_OPENCV_MSGS__MSG__DETAIL__MARKER_POSE__TRAITS_HPP_

#include "aruco_opencv_msgs/msg/detail/marker_pose__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<aruco_opencv_msgs::msg::MarkerPose>()
{
  return "aruco_opencv_msgs::msg::MarkerPose";
}

template<>
inline const char * name<aruco_opencv_msgs::msg::MarkerPose>()
{
  return "aruco_opencv_msgs/msg/MarkerPose";
}

template<>
struct has_fixed_size<aruco_opencv_msgs::msg::MarkerPose>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Pose>::value> {};

template<>
struct has_bounded_size<aruco_opencv_msgs::msg::MarkerPose>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Pose>::value> {};

template<>
struct is_message<aruco_opencv_msgs::msg::MarkerPose>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ARUCO_OPENCV_MSGS__MSG__DETAIL__MARKER_POSE__TRAITS_HPP_
