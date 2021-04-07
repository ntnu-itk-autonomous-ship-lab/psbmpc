// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from psbmpc_interfaces:msg/Covariance2.idl
// generated code does not contain a copyright notice

#ifndef PSBMPC_INTERFACES__MSG__DETAIL__COVARIANCE2__STRUCT_HPP_
#define PSBMPC_INTERFACES__MSG__DETAIL__COVARIANCE2__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__psbmpc_interfaces__msg__Covariance2 __attribute__((deprecated))
#else
# define DEPRECATED__psbmpc_interfaces__msg__Covariance2 __declspec(deprecated)
#endif

namespace psbmpc_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Covariance2_
{
  using Type = Covariance2_<ContainerAllocator>;

  explicit Covariance2_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->var_x = 0.0;
      this->var_y = 0.0;
      this->cor_xy = 0.0;
    }
  }

  explicit Covariance2_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->var_x = 0.0;
      this->var_y = 0.0;
      this->cor_xy = 0.0;
    }
  }

  // field types and members
  using _var_x_type =
    double;
  _var_x_type var_x;
  using _var_y_type =
    double;
  _var_y_type var_y;
  using _cor_xy_type =
    double;
  _cor_xy_type cor_xy;

  // setters for named parameter idiom
  Type & set__var_x(
    const double & _arg)
  {
    this->var_x = _arg;
    return *this;
  }
  Type & set__var_y(
    const double & _arg)
  {
    this->var_y = _arg;
    return *this;
  }
  Type & set__cor_xy(
    const double & _arg)
  {
    this->cor_xy = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    psbmpc_interfaces::msg::Covariance2_<ContainerAllocator> *;
  using ConstRawPtr =
    const psbmpc_interfaces::msg::Covariance2_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<psbmpc_interfaces::msg::Covariance2_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<psbmpc_interfaces::msg::Covariance2_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      psbmpc_interfaces::msg::Covariance2_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<psbmpc_interfaces::msg::Covariance2_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      psbmpc_interfaces::msg::Covariance2_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<psbmpc_interfaces::msg::Covariance2_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<psbmpc_interfaces::msg::Covariance2_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<psbmpc_interfaces::msg::Covariance2_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__psbmpc_interfaces__msg__Covariance2
    std::shared_ptr<psbmpc_interfaces::msg::Covariance2_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__psbmpc_interfaces__msg__Covariance2
    std::shared_ptr<psbmpc_interfaces::msg::Covariance2_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Covariance2_ & other) const
  {
    if (this->var_x != other.var_x) {
      return false;
    }
    if (this->var_y != other.var_y) {
      return false;
    }
    if (this->cor_xy != other.cor_xy) {
      return false;
    }
    return true;
  }
  bool operator!=(const Covariance2_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Covariance2_

// alias to use template instance with default allocator
using Covariance2 =
  psbmpc_interfaces::msg::Covariance2_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace psbmpc_interfaces

#endif  // PSBMPC_INTERFACES__MSG__DETAIL__COVARIANCE2__STRUCT_HPP_
