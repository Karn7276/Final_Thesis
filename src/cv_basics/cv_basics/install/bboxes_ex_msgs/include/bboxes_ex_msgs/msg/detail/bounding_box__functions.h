// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from bboxes_ex_msgs:msg/BoundingBox.idl
// generated code does not contain a copyright notice

#ifndef BBOXES_EX_MSGS__MSG__DETAIL__BOUNDING_BOX__FUNCTIONS_H_
#define BBOXES_EX_MSGS__MSG__DETAIL__BOUNDING_BOX__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "bboxes_ex_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "bboxes_ex_msgs/msg/detail/bounding_box__struct.h"

/// Initialize msg/BoundingBox message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * bboxes_ex_msgs__msg__BoundingBox
 * )) before or use
 * bboxes_ex_msgs__msg__BoundingBox__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_bboxes_ex_msgs
bool
bboxes_ex_msgs__msg__BoundingBox__init(bboxes_ex_msgs__msg__BoundingBox * msg);

/// Finalize msg/BoundingBox message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_bboxes_ex_msgs
void
bboxes_ex_msgs__msg__BoundingBox__fini(bboxes_ex_msgs__msg__BoundingBox * msg);

/// Create msg/BoundingBox message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * bboxes_ex_msgs__msg__BoundingBox__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_bboxes_ex_msgs
bboxes_ex_msgs__msg__BoundingBox *
bboxes_ex_msgs__msg__BoundingBox__create();

/// Destroy msg/BoundingBox message.
/**
 * It calls
 * bboxes_ex_msgs__msg__BoundingBox__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_bboxes_ex_msgs
void
bboxes_ex_msgs__msg__BoundingBox__destroy(bboxes_ex_msgs__msg__BoundingBox * msg);


/// Initialize array of msg/BoundingBox messages.
/**
 * It allocates the memory for the number of elements and calls
 * bboxes_ex_msgs__msg__BoundingBox__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_bboxes_ex_msgs
bool
bboxes_ex_msgs__msg__BoundingBox__Sequence__init(bboxes_ex_msgs__msg__BoundingBox__Sequence * array, size_t size);

/// Finalize array of msg/BoundingBox messages.
/**
 * It calls
 * bboxes_ex_msgs__msg__BoundingBox__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_bboxes_ex_msgs
void
bboxes_ex_msgs__msg__BoundingBox__Sequence__fini(bboxes_ex_msgs__msg__BoundingBox__Sequence * array);

/// Create array of msg/BoundingBox messages.
/**
 * It allocates the memory for the array and calls
 * bboxes_ex_msgs__msg__BoundingBox__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_bboxes_ex_msgs
bboxes_ex_msgs__msg__BoundingBox__Sequence *
bboxes_ex_msgs__msg__BoundingBox__Sequence__create(size_t size);

/// Destroy array of msg/BoundingBox messages.
/**
 * It calls
 * bboxes_ex_msgs__msg__BoundingBox__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_bboxes_ex_msgs
void
bboxes_ex_msgs__msg__BoundingBox__Sequence__destroy(bboxes_ex_msgs__msg__BoundingBox__Sequence * array);

#ifdef __cplusplus
}
#endif

#endif  // BBOXES_EX_MSGS__MSG__DETAIL__BOUNDING_BOX__FUNCTIONS_H_
