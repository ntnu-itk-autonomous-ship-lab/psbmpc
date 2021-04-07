// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from psbmpc_interfaces:msg/CrossCovariance2.idl
// generated code does not contain a copyright notice

#ifndef PSBMPC_INTERFACES__MSG__DETAIL__CROSS_COVARIANCE2__STRUCT_HPP_
#define PSBMPC_INTERFACES__MSG__DETAIL__CROSS_COVARIANCE2__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__psbmpc_interfaces__msg__CrossCovariance2 __attribute__((deprecated))
#else
# define DEPRECATED__psbmpc_interfaces__msg__CrossCovariance2 __declspec(deprecated)
#endif

namespace psbmpc_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CrossCovariance2_
{
  using Type = CrossCovariance2_<ContainerAllocator>;

  explicit CrossCovariance2_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->cor_px_vx = 0.0;
      this->cor_px_vy = 0.0;
      this->cor_py_vx = 0.0;
      this->cor_py_vy = 0.0;
    }
  }

  explicit CrossCovariance2_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->cor_px_vx = 0.0;
      this->cor_px_vy = 0.0;
      this->cor_py_vx = 0.0;
      this->cor_py_vy = 0.0;
    }
  }

  // field types and members
  using _cor_px_vx_type =
    double;
  _cor_px_vx_type cor_px_vx;
  using _cor_px_vy_type =
    double;
  _cor_px_vy_type cor_px_vy;
  using _cor_py_vx_type =
    double;
  _cor_py_vx_type cor_py_vx;
  using _cor_py_vy_type =
    double;
  _cor_py_vy_type cor_py_vy;

  // setters for named parameter idiom
  Type & set__cor_px_vx(
    const double & _arg)
  {
    this->cor_px_vx = _arg;
    return *this;
  }
  Type & set__cor_px_vy(
    const double & _arg)
  {
    this->cor_px_vy = _arg;
    return *this;
  }
  Type & set__cor_py_vx(
    const double & _arg)
  {
    this->cor_py_vx = _arg;
    return *this;
  }
  Type & set__cor_py_vy(
    const double & _arg)
  {
    this->cor_py_vy = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    psbmpc_interfaces::msg::CrossCovariance2_<ContainerAllocator> *;
  using ConstRawPtr =
    const psbmpc_interfaces::msg::CrossCovariance2_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<psbmpc_interfaces::msg::CrossCovariance2_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<psbmpc_interfaces::msg::CrossCovariance2_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      psbmpc_interfaces::msg::CrossCovariance2_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<psbmpc_interfaces::msg::CrossCovariance2_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      psbmpc_interfaces::msg::CrossCovariance2_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<psbmpc_interfaces::msg::CrossCovariance2_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<psbmpc_interfaces::msg::CrossCovariance2_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<psbmpc_interfaces::msg::CrossCovariance2_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__psbmpc_interfaces__msg__CrossCovariance2
    std::shared_ptr<psbmpc_interfaces::msg::CrossCovariance2_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__psbmpc_interfaces__msg__CrossCovariance2
    std::shared_ptr<psbmpc_interfaces::msg::CrossCovariance2_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CrossCovariance2_ & other) const
  {
    if (this->cor_px_vx != other.cor_px_vx) {
      return false;
    }
    if (this->cor_px_vy != other.cor_px_vy) {
      return false;
    }
    if (this->cor_py_vx != other.cor_py_vx) {
      return false;
    }
    if (this->cor_py_vy != other.cor_py_vy) {
      return false;
    }
    return true;
  }
  bool operator!=(const CrossCovariance2_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CrossCovariance2_

// alias to use template instance with default allocator
using CrossCovariance2 =
  psbmpc_interfaces::msg::CrossCovariance2_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace psbmpc_interfaces

#endif  // PSBMPC_INTERFACES__MSG__DETAIL__CROSS_COVARIANCE2__STRUCT_HPP_
