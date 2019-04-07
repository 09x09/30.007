// Generated by gencpp from file trolley/Wheels.msg
// DO NOT EDIT!


#ifndef TROLLEY_MESSAGE_WHEELS_H
#define TROLLEY_MESSAGE_WHEELS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace trolley
{
template <class ContainerAllocator>
struct Wheels_
{
  typedef Wheels_<ContainerAllocator> Type;

  Wheels_()
    : left(0.0)
    , right(0.0)  {
    }
  Wheels_(const ContainerAllocator& _alloc)
    : left(0.0)
    , right(0.0)  {
  (void)_alloc;
    }



   typedef double _left_type;
  _left_type left;

   typedef double _right_type;
  _right_type right;





  typedef boost::shared_ptr< ::trolley::Wheels_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::trolley::Wheels_<ContainerAllocator> const> ConstPtr;

}; // struct Wheels_

typedef ::trolley::Wheels_<std::allocator<void> > Wheels;

typedef boost::shared_ptr< ::trolley::Wheels > WheelsPtr;
typedef boost::shared_ptr< ::trolley::Wheels const> WheelsConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::trolley::Wheels_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::trolley::Wheels_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace trolley

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'trolley': ['/home/odroid/catkin_ws/src/trolley/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::trolley::Wheels_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::trolley::Wheels_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::trolley::Wheels_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::trolley::Wheels_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::trolley::Wheels_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::trolley::Wheels_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::trolley::Wheels_<ContainerAllocator> >
{
  static const char* value()
  {
    return "50c2436c38cded221d061b57126c4e40";
  }

  static const char* value(const ::trolley::Wheels_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x50c2436c38cded22ULL;
  static const uint64_t static_value2 = 0x1d061b57126c4e40ULL;
};

template<class ContainerAllocator>
struct DataType< ::trolley::Wheels_<ContainerAllocator> >
{
  static const char* value()
  {
    return "trolley/Wheels";
  }

  static const char* value(const ::trolley::Wheels_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::trolley::Wheels_<ContainerAllocator> >
{
  static const char* value()
  {
    return "#Wheels.msgs\n\
\n\
float64 left\n\
float64 right\n\
";
  }

  static const char* value(const ::trolley::Wheels_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::trolley::Wheels_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.left);
      stream.next(m.right);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Wheels_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::trolley::Wheels_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::trolley::Wheels_<ContainerAllocator>& v)
  {
    s << indent << "left: ";
    Printer<double>::stream(s, indent + "  ", v.left);
    s << indent << "right: ";
    Printer<double>::stream(s, indent + "  ", v.right);
  }
};

} // namespace message_operations
} // namespace ros

#endif // TROLLEY_MESSAGE_WHEELS_H