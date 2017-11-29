// Generated by gencpp from file autorally_msgs/chassisCommand.msg
// DO NOT EDIT!


#ifndef AUTORALLY_MSGS_MESSAGE_CHASSISCOMMAND_H
#define AUTORALLY_MSGS_MESSAGE_CHASSISCOMMAND_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace autorally_msgs
{
template <class ContainerAllocator>
struct chassisCommand_
{
  typedef chassisCommand_<ContainerAllocator> Type;

  chassisCommand_()
    : header()
    , sender()
    , throttle(0.0)
    , steering(0.0)
    , frontBrake(0.0)  {
    }
  chassisCommand_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , sender(_alloc)
    , throttle(0.0)
    , steering(0.0)
    , frontBrake(0.0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _sender_type;
  _sender_type sender;

   typedef double _throttle_type;
  _throttle_type throttle;

   typedef double _steering_type;
  _steering_type steering;

   typedef double _frontBrake_type;
  _frontBrake_type frontBrake;




  typedef boost::shared_ptr< ::autorally_msgs::chassisCommand_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::autorally_msgs::chassisCommand_<ContainerAllocator> const> ConstPtr;

}; // struct chassisCommand_

typedef ::autorally_msgs::chassisCommand_<std::allocator<void> > chassisCommand;

typedef boost::shared_ptr< ::autorally_msgs::chassisCommand > chassisCommandPtr;
typedef boost::shared_ptr< ::autorally_msgs::chassisCommand const> chassisCommandConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::autorally_msgs::chassisCommand_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::autorally_msgs::chassisCommand_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace autorally_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'sensor_msgs': ['/opt/ros/indigo/share/sensor_msgs/cmake/../msg'], 'autorally_msgs': ['/home/apoorva/catkin_ws/src/autorally/autorally_msgs/msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/indigo/share/geometry_msgs/cmake/../msg'], 'diagnostic_msgs': ['/opt/ros/indigo/share/diagnostic_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::autorally_msgs::chassisCommand_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::autorally_msgs::chassisCommand_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::autorally_msgs::chassisCommand_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::autorally_msgs::chassisCommand_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::autorally_msgs::chassisCommand_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::autorally_msgs::chassisCommand_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::autorally_msgs::chassisCommand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "aba3aba8fc9c485b4f2735439447f458";
  }

  static const char* value(const ::autorally_msgs::chassisCommand_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xaba3aba8fc9c485bULL;
  static const uint64_t static_value2 = 0x4f2735439447f458ULL;
};

template<class ContainerAllocator>
struct DataType< ::autorally_msgs::chassisCommand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "autorally_msgs/chassisCommand";
  }

  static const char* value(const ::autorally_msgs::chassisCommand_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::autorally_msgs::chassisCommand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n\
\n\
string sender\n\
float64 throttle\n\
float64 steering\n\
float64 frontBrake\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
";
  }

  static const char* value(const ::autorally_msgs::chassisCommand_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::autorally_msgs::chassisCommand_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.sender);
      stream.next(m.throttle);
      stream.next(m.steering);
      stream.next(m.frontBrake);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct chassisCommand_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::autorally_msgs::chassisCommand_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::autorally_msgs::chassisCommand_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "sender: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.sender);
    s << indent << "throttle: ";
    Printer<double>::stream(s, indent + "  ", v.throttle);
    s << indent << "steering: ";
    Printer<double>::stream(s, indent + "  ", v.steering);
    s << indent << "frontBrake: ";
    Printer<double>::stream(s, indent + "  ", v.frontBrake);
  }
};

} // namespace message_operations
} // namespace ros

#endif // AUTORALLY_MSGS_MESSAGE_CHASSISCOMMAND_H