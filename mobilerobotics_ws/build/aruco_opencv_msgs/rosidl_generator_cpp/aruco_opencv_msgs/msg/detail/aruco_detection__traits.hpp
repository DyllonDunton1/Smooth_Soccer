// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from aruco_opencv_msgs:msg/ArucoDetection.idl
// generated code does not contain a copyright notice

#ifndef ARUCO_OPENCV_MSGS__MSG__DETAIL__ARUCO_DETECTION__TRAITS_HPP_
#define ARUCO_OPENCV_MSGS__MSG__DETAIL__ARUCO_DETECTION__TRAITS_HPP_

#include "aruco_opencv_msgs/msg/detail/aruco_detection__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<aruco_opencv_msgs::msg::ArucoDetection>()
{
  return "aruco_opencv_msgs::msg::ArucoDetection";
}

template<>
inline const char * name<aruco_opencv_msgs::msg::ArucoDetection>()
{
  return "aruco_opencv_msgs/msg/ArucoDetection";
}

template<>
struct has_fixed_size<aruco_opencv_msgs::msg::ArucoDetection>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<aruco_opencv_msgs::msg::ArucoDetection>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<aruco_opencv_msgs::msg::ArucoDetection>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ARUCO_OPENCV_MSGS__MSG__DETAIL__ARUCO_DETECTION__TRAITS_HPP_
