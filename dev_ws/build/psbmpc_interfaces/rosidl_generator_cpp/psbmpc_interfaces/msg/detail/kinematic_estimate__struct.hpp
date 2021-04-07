// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from psbmpc_interfaces:msg/KinematicEstimate.idl
// generated code does not contain a copyright notice

#ifndef PSBMPC_INTERFACES__MSG__DETAIL__KINEMATIC_ESTIMATE__STRUCT_HPP_
#define PSBMPC_INTERFACES__MSG__DETAIL__KINEMATIC_ESTIMATE__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'pos_est'
// Member 'vel_est'
#include "psbmpc_interfaces/msg/detail/vector2__struct.hpp"
// Member 'pos_cov'
// Member 'vel_cov'
#include "psbmpc_interfaces/msg/detail/covariance2__struct.hpp"
// Member 'pos_vel_corr'
#include "psbmpc_interfaces/msg/detail/cross_covariance2__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__psbmpc_interfaces__msg__KinematicEstimate __attribute__((deprecated))
#else
# define DEPRECATED__psbmpc_interfaces__msg__KinematicEstimate __declspec(deprecated)
#endif

namespace psbmpc_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct KinematicEstimate_
{
  using Type = KinematicEstimate_<ContainerAllocator>;

  explicit KinematicEstimate_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pos_est(_init),
    vel_est(_init),
    pos_cov(_init),
    vel_cov(_init),
    pos_vel_corr(_init)
  {
    (void)_init;
  }

  explicit KinematicEstimate_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pos_est(_alloc, _init),
    vel_est(_alloc, _init),
    pos_cov(_alloc, _init),
    vel_cov(_alloc, _init),
    pos_vel_corr(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _pos_est_type =
    psbmpc_interfaces::msg::Vector2_<ContainerAllocator>;
  _pos_est_type pos_est;
  using _vel_est_type =
    psbmpc_interfaces::msg::Vector2_<ContainerAllocator>;
  _vel_est_type vel_est;
  using _pos_cov_type =
    psbmpc_interfaces::msg::Covariance2_<ContainerAllocator>;
  _pos_cov_type pos_cov;
  using _vel_cov_type =
    psbmpc_interfaces::msg::Covariance2_<ContainerAllocator>;
  _vel_cov_type vel_cov;
  using _pos_vel_corr_type =
    psbmpc_interfaces::msg::CrossCovariance2_<ContainerAllocator>;
  _pos_vel_corr_type pos_vel_corr;

  // setters for named parameter idiom
  Type & set__pos_est(
    const psbmpc_interfaces::msg::Vector2_<ContainerAllocator> & _arg)
  {
    this->pos_est = _arg;
    return *this;
  }
  Type & set__vel_est(
    const psbmpc_interfaces::msg::Vector2_<ContainerAllocator> & _arg)
  {
    this->vel_est = _arg;
    return *this;
  }
  Type & set__pos_cov(
    const psbmpc_interfaces::msg::Covariance2_<ContainerAllocator> & _arg)
  {
    this->pos_cov = _arg;
    return *this;
  }
  Type & set__vel_cov(
    const psbmpc_interfaces::msg::Covariance2_<ContainerAllocator> & _arg)
  {
    this->vel_cov = _arg;
    return *this;
  }
  Type & set__pos_vel_corr(
    const psbmpc_interfaces::msg::CrossCovariance2_<ContainerAllocator> & _arg)
  {
    this->pos_vel_corr = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    psbmpc_interfaces::msg::KinematicEstimate_<ContainerAllocator> *;
  using ConstRawPtr =
    const psbmpc_interfaces::msg::KinematicEstimate_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<psbmpc_interfaces::msg::KinematicEstimate_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<psbmpc_interfaces::msg::KinematicEstimate_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      psbmpc_interfaces::msg::KinematicEstimate_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<psbmpc_interfaces::msg::KinematicEstimate_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      psbmpc_interfaces::msg::KinematicEstimate_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<psbmpc_interfaces::msg::KinematicEstimate_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<psbmpc_interfaces::msg::KinematicEstimate_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<psbmpc_interfaces::msg::KinematicEstimate_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__psbmpc_interfaces__msg__KinematicEstimate
    std::shared_ptr<psbmpc_interfaces::msg::KinematicEstimate_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__psbmpc_interfaces__msg__KinematicEstimate
    std::shared_ptr<psbmpc_interfaces::msg::KinematicEstimate_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const KinematicEstimate_ & other) const
  {
    if (this->pos_est != other.pos_est) {
      return false;
    }
    if (this->vel_est != other.vel_est) {
      return false;
    }
    if (this->pos_cov != other.pos_cov) {
      return false;
    }
    if (this->vel_cov != other.vel_cov) {
      return false;
    }
    if (this->pos_vel_corr != other.pos_vel_corr) {
      return false;
    }
    return true;
  }
  bool operator!=(const KinematicEstimate_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct KinematicEstimate_

// alias to use template instance with default allocator
using KinematicEstimate =
  psbmpc_interfaces::msg::KinematicEstimate_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace psbmpc_interfaces

#endif  // PSBMPC_INTERFACES__MSG__DETAIL__KINEMATIC_ESTIMATE__STRUCT_HPP_
