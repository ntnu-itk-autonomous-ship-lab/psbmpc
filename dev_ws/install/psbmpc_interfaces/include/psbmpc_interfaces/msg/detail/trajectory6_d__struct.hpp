// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from psbmpc_interfaces:msg/Trajectory6D.idl
// generated code does not contain a copyright notice

#ifndef PSBMPC_INTERFACES__MSG__DETAIL__TRAJECTORY6_D__STRUCT_HPP_
#define PSBMPC_INTERFACES__MSG__DETAIL__TRAJECTORY6_D__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'predicted_trajectory'
#include "psbmpc_interfaces/msg/detail/reference__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__psbmpc_interfaces__msg__Trajectory6D __attribute__((deprecated))
#else
# define DEPRECATED__psbmpc_interfaces__msg__Trajectory6D __declspec(deprecated)
#endif

namespace psbmpc_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Trajectory6D_
{
  using Type = Trajectory6D_<ContainerAllocator>;

  explicit Trajectory6D_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit Trajectory6D_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _predicted_trajectory_type =
    std::vector<psbmpc_interfaces::msg::Reference_<ContainerAllocator>, typename ContainerAllocator::template rebind<psbmpc_interfaces::msg::Reference_<ContainerAllocator>>::other>;
  _predicted_trajectory_type predicted_trajectory;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__predicted_trajectory(
    const std::vector<psbmpc_interfaces::msg::Reference_<ContainerAllocator>, typename ContainerAllocator::template rebind<psbmpc_interfaces::msg::Reference_<ContainerAllocator>>::other> & _arg)
  {
    this->predicted_trajectory = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    psbmpc_interfaces::msg::Trajectory6D_<ContainerAllocator> *;
  using ConstRawPtr =
    const psbmpc_interfaces::msg::Trajectory6D_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<psbmpc_interfaces::msg::Trajectory6D_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<psbmpc_interfaces::msg::Trajectory6D_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      psbmpc_interfaces::msg::Trajectory6D_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<psbmpc_interfaces::msg::Trajectory6D_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      psbmpc_interfaces::msg::Trajectory6D_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<psbmpc_interfaces::msg::Trajectory6D_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<psbmpc_interfaces::msg::Trajectory6D_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<psbmpc_interfaces::msg::Trajectory6D_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__psbmpc_interfaces__msg__Trajectory6D
    std::shared_ptr<psbmpc_interfaces::msg::Trajectory6D_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__psbmpc_interfaces__msg__Trajectory6D
    std::shared_ptr<psbmpc_interfaces::msg::Trajectory6D_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Trajectory6D_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->predicted_trajectory != other.predicted_trajectory) {
      return false;
    }
    return true;
  }
  bool operator!=(const Trajectory6D_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Trajectory6D_

// alias to use template instance with default allocator
using Trajectory6D =
  psbmpc_interfaces::msg::Trajectory6D_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace psbmpc_interfaces

#endif  // PSBMPC_INTERFACES__MSG__DETAIL__TRAJECTORY6_D__STRUCT_HPP_
