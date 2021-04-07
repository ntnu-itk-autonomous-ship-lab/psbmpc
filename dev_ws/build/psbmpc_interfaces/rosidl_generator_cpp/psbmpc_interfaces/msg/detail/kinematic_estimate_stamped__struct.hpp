// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from psbmpc_interfaces:msg/KinematicEstimateStamped.idl
// generated code does not contain a copyright notice

#ifndef PSBMPC_INTERFACES__MSG__DETAIL__KINEMATIC_ESTIMATE_STAMPED__STRUCT_HPP_
#define PSBMPC_INTERFACES__MSG__DETAIL__KINEMATIC_ESTIMATE_STAMPED__STRUCT_HPP_

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
// Member 'kinematic_estimate'
#include "psbmpc_interfaces/msg/detail/kinematic_estimate__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__psbmpc_interfaces__msg__KinematicEstimateStamped __attribute__((deprecated))
#else
# define DEPRECATED__psbmpc_interfaces__msg__KinematicEstimateStamped __declspec(deprecated)
#endif

namespace psbmpc_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct KinematicEstimateStamped_
{
  using Type = KinematicEstimateStamped_<ContainerAllocator>;

  explicit KinematicEstimateStamped_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    kinematic_estimate(_init)
  {
    (void)_init;
  }

  explicit KinematicEstimateStamped_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    kinematic_estimate(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _kinematic_estimate_type =
    psbmpc_interfaces::msg::KinematicEstimate_<ContainerAllocator>;
  _kinematic_estimate_type kinematic_estimate;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__kinematic_estimate(
    const psbmpc_interfaces::msg::KinematicEstimate_<ContainerAllocator> & _arg)
  {
    this->kinematic_estimate = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    psbmpc_interfaces::msg::KinematicEstimateStamped_<ContainerAllocator> *;
  using ConstRawPtr =
    const psbmpc_interfaces::msg::KinematicEstimateStamped_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<psbmpc_interfaces::msg::KinematicEstimateStamped_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<psbmpc_interfaces::msg::KinematicEstimateStamped_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      psbmpc_interfaces::msg::KinematicEstimateStamped_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<psbmpc_interfaces::msg::KinematicEstimateStamped_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      psbmpc_interfaces::msg::KinematicEstimateStamped_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<psbmpc_interfaces::msg::KinematicEstimateStamped_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<psbmpc_interfaces::msg::KinematicEstimateStamped_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<psbmpc_interfaces::msg::KinematicEstimateStamped_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__psbmpc_interfaces__msg__KinematicEstimateStamped
    std::shared_ptr<psbmpc_interfaces::msg::KinematicEstimateStamped_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__psbmpc_interfaces__msg__KinematicEstimateStamped
    std::shared_ptr<psbmpc_interfaces::msg::KinematicEstimateStamped_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const KinematicEstimateStamped_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->kinematic_estimate != other.kinematic_estimate) {
      return false;
    }
    return true;
  }
  bool operator!=(const KinematicEstimateStamped_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct KinematicEstimateStamped_

// alias to use template instance with default allocator
using KinematicEstimateStamped =
  psbmpc_interfaces::msg::KinematicEstimateStamped_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace psbmpc_interfaces

#endif  // PSBMPC_INTERFACES__MSG__DETAIL__KINEMATIC_ESTIMATE_STAMPED__STRUCT_HPP_
