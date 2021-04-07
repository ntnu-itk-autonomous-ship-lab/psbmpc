// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from psbmpc_interfaces:msg/Reference.idl
// generated code does not contain a copyright notice

#ifndef PSBMPC_INTERFACES__MSG__DETAIL__REFERENCE__STRUCT_HPP_
#define PSBMPC_INTERFACES__MSG__DETAIL__REFERENCE__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'configuration'
// Member 'velocity'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__psbmpc_interfaces__msg__Reference __attribute__((deprecated))
#else
# define DEPRECATED__psbmpc_interfaces__msg__Reference __declspec(deprecated)
#endif

namespace psbmpc_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Reference_
{
  using Type = Reference_<ContainerAllocator>;

  explicit Reference_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : configuration(_init),
    velocity(_init)
  {
    (void)_init;
  }

  explicit Reference_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : configuration(_alloc, _init),
    velocity(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _configuration_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _configuration_type configuration;
  using _velocity_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _velocity_type velocity;

  // setters for named parameter idiom
  Type & set__configuration(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->configuration = _arg;
    return *this;
  }
  Type & set__velocity(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->velocity = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    psbmpc_interfaces::msg::Reference_<ContainerAllocator> *;
  using ConstRawPtr =
    const psbmpc_interfaces::msg::Reference_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<psbmpc_interfaces::msg::Reference_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<psbmpc_interfaces::msg::Reference_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      psbmpc_interfaces::msg::Reference_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<psbmpc_interfaces::msg::Reference_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      psbmpc_interfaces::msg::Reference_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<psbmpc_interfaces::msg::Reference_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<psbmpc_interfaces::msg::Reference_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<psbmpc_interfaces::msg::Reference_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__psbmpc_interfaces__msg__Reference
    std::shared_ptr<psbmpc_interfaces::msg::Reference_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__psbmpc_interfaces__msg__Reference
    std::shared_ptr<psbmpc_interfaces::msg::Reference_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Reference_ & other) const
  {
    if (this->configuration != other.configuration) {
      return false;
    }
    if (this->velocity != other.velocity) {
      return false;
    }
    return true;
  }
  bool operator!=(const Reference_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Reference_

// alias to use template instance with default allocator
using Reference =
  psbmpc_interfaces::msg::Reference_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace psbmpc_interfaces

#endif  // PSBMPC_INTERFACES__MSG__DETAIL__REFERENCE__STRUCT_HPP_
