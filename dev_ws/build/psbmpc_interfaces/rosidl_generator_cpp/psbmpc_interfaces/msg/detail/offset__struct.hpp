// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from psbmpc_interfaces:msg/Offset.idl
// generated code does not contain a copyright notice

#ifndef PSBMPC_INTERFACES__MSG__DETAIL__OFFSET__STRUCT_HPP_
#define PSBMPC_INTERFACES__MSG__DETAIL__OFFSET__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__psbmpc_interfaces__msg__Offset __attribute__((deprecated))
#else
# define DEPRECATED__psbmpc_interfaces__msg__Offset __declspec(deprecated)
#endif

namespace psbmpc_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Offset_
{
  using Type = Offset_<ContainerAllocator>;

  explicit Offset_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->u_m = 0.0;
      this->chi_m = 0.0;
    }
  }

  explicit Offset_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->u_m = 0.0;
      this->chi_m = 0.0;
    }
  }

  // field types and members
  using _u_m_type =
    double;
  _u_m_type u_m;
  using _chi_m_type =
    double;
  _chi_m_type chi_m;

  // setters for named parameter idiom
  Type & set__u_m(
    const double & _arg)
  {
    this->u_m = _arg;
    return *this;
  }
  Type & set__chi_m(
    const double & _arg)
  {
    this->chi_m = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    psbmpc_interfaces::msg::Offset_<ContainerAllocator> *;
  using ConstRawPtr =
    const psbmpc_interfaces::msg::Offset_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<psbmpc_interfaces::msg::Offset_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<psbmpc_interfaces::msg::Offset_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      psbmpc_interfaces::msg::Offset_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<psbmpc_interfaces::msg::Offset_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      psbmpc_interfaces::msg::Offset_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<psbmpc_interfaces::msg::Offset_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<psbmpc_interfaces::msg::Offset_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<psbmpc_interfaces::msg::Offset_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__psbmpc_interfaces__msg__Offset
    std::shared_ptr<psbmpc_interfaces::msg::Offset_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__psbmpc_interfaces__msg__Offset
    std::shared_ptr<psbmpc_interfaces::msg::Offset_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Offset_ & other) const
  {
    if (this->u_m != other.u_m) {
      return false;
    }
    if (this->chi_m != other.chi_m) {
      return false;
    }
    return true;
  }
  bool operator!=(const Offset_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Offset_

// alias to use template instance with default allocator
using Offset =
  psbmpc_interfaces::msg::Offset_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace psbmpc_interfaces

#endif  // PSBMPC_INTERFACES__MSG__DETAIL__OFFSET__STRUCT_HPP_
