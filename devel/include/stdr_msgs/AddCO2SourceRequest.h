// Generated by gencpp from file stdr_msgs/AddCO2SourceRequest.msg
// DO NOT EDIT!


#ifndef STDR_MSGS_MESSAGE_ADDCO2SOURCEREQUEST_H
#define STDR_MSGS_MESSAGE_ADDCO2SOURCEREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <stdr_msgs/CO2Source.h>

namespace stdr_msgs
{
template <class ContainerAllocator>
struct AddCO2SourceRequest_
{
  typedef AddCO2SourceRequest_<ContainerAllocator> Type;

  AddCO2SourceRequest_()
    : newSource()  {
    }
  AddCO2SourceRequest_(const ContainerAllocator& _alloc)
    : newSource(_alloc)  {
  (void)_alloc;
    }



   typedef  ::stdr_msgs::CO2Source_<ContainerAllocator>  _newSource_type;
  _newSource_type newSource;




  typedef boost::shared_ptr< ::stdr_msgs::AddCO2SourceRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::stdr_msgs::AddCO2SourceRequest_<ContainerAllocator> const> ConstPtr;

}; // struct AddCO2SourceRequest_

typedef ::stdr_msgs::AddCO2SourceRequest_<std::allocator<void> > AddCO2SourceRequest;

typedef boost::shared_ptr< ::stdr_msgs::AddCO2SourceRequest > AddCO2SourceRequestPtr;
typedef boost::shared_ptr< ::stdr_msgs::AddCO2SourceRequest const> AddCO2SourceRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::stdr_msgs::AddCO2SourceRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::stdr_msgs::AddCO2SourceRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace stdr_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'nav_msgs': ['/opt/ros/kinetic/share/nav_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'stdr_msgs': ['/home/tahlia/cw1/src/comp313p/stdr_simulator/stdr_msgs/msg', '/home/tahlia/cw1/devel/share/stdr_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::stdr_msgs::AddCO2SourceRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::stdr_msgs::AddCO2SourceRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::stdr_msgs::AddCO2SourceRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::stdr_msgs::AddCO2SourceRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::stdr_msgs::AddCO2SourceRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::stdr_msgs::AddCO2SourceRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::stdr_msgs::AddCO2SourceRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "7199b309078089de32dcffa91f18ebd0";
  }

  static const char* value(const ::stdr_msgs::AddCO2SourceRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x7199b309078089deULL;
  static const uint64_t static_value2 = 0x32dcffa91f18ebd0ULL;
};

template<class ContainerAllocator>
struct DataType< ::stdr_msgs::AddCO2SourceRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "stdr_msgs/AddCO2SourceRequest";
  }

  static const char* value(const ::stdr_msgs::AddCO2SourceRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::stdr_msgs::AddCO2SourceRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "stdr_msgs/CO2Source newSource\n\
\n\
================================================================================\n\
MSG: stdr_msgs/CO2Source\n\
# Source description\n\
\n\
string id\n\
float32 ppm\n\
\n\
# sensor pose, relative to the map origin\n\
geometry_msgs/Pose2D pose \n\
\n\
================================================================================\n\
MSG: geometry_msgs/Pose2D\n\
# This expresses a position and orientation on a 2D manifold.\n\
\n\
float64 x\n\
float64 y\n\
float64 theta\n\
";
  }

  static const char* value(const ::stdr_msgs::AddCO2SourceRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::stdr_msgs::AddCO2SourceRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.newSource);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct AddCO2SourceRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::stdr_msgs::AddCO2SourceRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::stdr_msgs::AddCO2SourceRequest_<ContainerAllocator>& v)
  {
    s << indent << "newSource: ";
    s << std::endl;
    Printer< ::stdr_msgs::CO2Source_<ContainerAllocator> >::stream(s, indent + "  ", v.newSource);
  }
};

} // namespace message_operations
} // namespace ros

#endif // STDR_MSGS_MESSAGE_ADDCO2SOURCEREQUEST_H
