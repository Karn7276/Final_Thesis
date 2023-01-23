// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from bboxes_ex_msgs:msg/BoundingBoxes.idl
// generated code does not contain a copyright notice

#ifndef BBOXES_EX_MSGS__MSG__DETAIL__BOUNDING_BOXES__TRAITS_HPP_
#define BBOXES_EX_MSGS__MSG__DETAIL__BOUNDING_BOXES__TRAITS_HPP_

#include "bboxes_ex_msgs/msg/detail/bounding_boxes__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'header'
// Member 'image_header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<bboxes_ex_msgs::msg::BoundingBoxes>()
{
  return "bboxes_ex_msgs::msg::BoundingBoxes";
}

template<>
inline const char * name<bboxes_ex_msgs::msg::BoundingBoxes>()
{
  return "bboxes_ex_msgs/msg/BoundingBoxes";
}

template<>
struct has_fixed_size<bboxes_ex_msgs::msg::BoundingBoxes>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<bboxes_ex_msgs::msg::BoundingBoxes>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<bboxes_ex_msgs::msg::BoundingBoxes>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // BBOXES_EX_MSGS__MSG__DETAIL__BOUNDING_BOXES__TRAITS_HPP_
