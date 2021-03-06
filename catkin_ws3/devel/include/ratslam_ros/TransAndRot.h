// Generated by gencpp from file ratslam_ros/TransAndRot.msg
// DO NOT EDIT!


#ifndef RATSLAM_ROS_MESSAGE_TRANSANDROT_H
#define RATSLAM_ROS_MESSAGE_TRANSANDROT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace ratslam_ros
{
template <class ContainerAllocator>
struct TransAndRot_
{
  typedef TransAndRot_<ContainerAllocator> Type;

  TransAndRot_()
    : header()
    , t()
    , R()  {
    }
  TransAndRot_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , t(_alloc)
    , R(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _t_type;
  _t_type t;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _R_type;
  _R_type R;




  typedef boost::shared_ptr< ::ratslam_ros::TransAndRot_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ratslam_ros::TransAndRot_<ContainerAllocator> const> ConstPtr;

}; // struct TransAndRot_

typedef ::ratslam_ros::TransAndRot_<std::allocator<void> > TransAndRot;

typedef boost::shared_ptr< ::ratslam_ros::TransAndRot > TransAndRotPtr;
typedef boost::shared_ptr< ::ratslam_ros::TransAndRot const> TransAndRotConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ratslam_ros::TransAndRot_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ratslam_ros::TransAndRot_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace ratslam_ros

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'ratslam_ros': ['/home/younes/Musique/catkin_ws3/src/ratslam_ros/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::ratslam_ros::TransAndRot_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ratslam_ros::TransAndRot_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ratslam_ros::TransAndRot_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ratslam_ros::TransAndRot_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ratslam_ros::TransAndRot_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ratslam_ros::TransAndRot_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ratslam_ros::TransAndRot_<ContainerAllocator> >
{
  static const char* value()
  {
    return "01e1fb5fba3618d68a00aa3d51d727ce";
  }

  static const char* value(const ::ratslam_ros::TransAndRot_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x01e1fb5fba3618d6ULL;
  static const uint64_t static_value2 = 0x8a00aa3d51d727ceULL;
};

template<class ContainerAllocator>
struct DataType< ::ratslam_ros::TransAndRot_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ratslam_ros/TransAndRot";
  }

  static const char* value(const ::ratslam_ros::TransAndRot_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ratslam_ros::TransAndRot_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n\
float64[] t\n\
float64[] R\n\
\n\
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

  static const char* value(const ::ratslam_ros::TransAndRot_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ratslam_ros::TransAndRot_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.t);
      stream.next(m.R);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct TransAndRot_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ratslam_ros::TransAndRot_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ratslam_ros::TransAndRot_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "t[]" << std::endl;
    for (size_t i = 0; i < v.t.size(); ++i)
    {
      s << indent << "  t[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.t[i]);
    }
    s << indent << "R[]" << std::endl;
    for (size_t i = 0; i < v.R.size(); ++i)
    {
      s << indent << "  R[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.R[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // RATSLAM_ROS_MESSAGE_TRANSANDROT_H
