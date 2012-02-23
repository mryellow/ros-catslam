/* Auto-generated by genmsg_cpp for file /home/will/ros/bow_extract/msg/cvKeypointVec.msg */
#ifndef BOW_EXTRACT_MESSAGE_CVKEYPOINTVEC_H
#define BOW_EXTRACT_MESSAGE_CVKEYPOINTVEC_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "std_msgs/Header.h"
#include "bow_extract/cvKeypoint.h"

namespace bow_extract
{
template <class ContainerAllocator>
struct cvKeypointVec_ {
  typedef cvKeypointVec_<ContainerAllocator> Type;

  cvKeypointVec_()
  : header()
  , keypoints()
  {
  }

  cvKeypointVec_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , keypoints(_alloc)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef std::vector< ::bow_extract::cvKeypoint_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::bow_extract::cvKeypoint_<ContainerAllocator> >::other >  _keypoints_type;
  std::vector< ::bow_extract::cvKeypoint_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::bow_extract::cvKeypoint_<ContainerAllocator> >::other >  keypoints;


  ROS_DEPRECATED uint32_t get_keypoints_size() const { return (uint32_t)keypoints.size(); }
  ROS_DEPRECATED void set_keypoints_size(uint32_t size) { keypoints.resize((size_t)size); }
  ROS_DEPRECATED void get_keypoints_vec(std::vector< ::bow_extract::cvKeypoint_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::bow_extract::cvKeypoint_<ContainerAllocator> >::other > & vec) const { vec = this->keypoints; }
  ROS_DEPRECATED void set_keypoints_vec(const std::vector< ::bow_extract::cvKeypoint_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::bow_extract::cvKeypoint_<ContainerAllocator> >::other > & vec) { this->keypoints = vec; }
private:
  static const char* __s_getDataType_() { return "bow_extract/cvKeypointVec"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "5b91cc77b3aa3ef99c20d571a3f680dc"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "Header header\n\
cvKeypoint[] keypoints\n\
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
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: bow_extract/cvKeypoint\n\
float32 x\n\
float32 y\n\
float32 size\n\
float32 angle\n\
float32 response\n\
uint32 octave\n\
uint32 class_id\n\
\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, header);
    ros::serialization::serialize(stream, keypoints);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, header);
    ros::serialization::deserialize(stream, keypoints);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(header);
    size += ros::serialization::serializationLength(keypoints);
    return size;
  }

  typedef boost::shared_ptr< ::bow_extract::cvKeypointVec_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::bow_extract::cvKeypointVec_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct cvKeypointVec
typedef  ::bow_extract::cvKeypointVec_<std::allocator<void> > cvKeypointVec;

typedef boost::shared_ptr< ::bow_extract::cvKeypointVec> cvKeypointVecPtr;
typedef boost::shared_ptr< ::bow_extract::cvKeypointVec const> cvKeypointVecConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::bow_extract::cvKeypointVec_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::bow_extract::cvKeypointVec_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace bow_extract

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::bow_extract::cvKeypointVec_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::bow_extract::cvKeypointVec_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::bow_extract::cvKeypointVec_<ContainerAllocator> > {
  static const char* value() 
  {
    return "5b91cc77b3aa3ef99c20d571a3f680dc";
  }

  static const char* value(const  ::bow_extract::cvKeypointVec_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x5b91cc77b3aa3ef9ULL;
  static const uint64_t static_value2 = 0x9c20d571a3f680dcULL;
};

template<class ContainerAllocator>
struct DataType< ::bow_extract::cvKeypointVec_<ContainerAllocator> > {
  static const char* value() 
  {
    return "bow_extract/cvKeypointVec";
  }

  static const char* value(const  ::bow_extract::cvKeypointVec_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::bow_extract::cvKeypointVec_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Header header\n\
cvKeypoint[] keypoints\n\
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
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: bow_extract/cvKeypoint\n\
float32 x\n\
float32 y\n\
float32 size\n\
float32 angle\n\
float32 response\n\
uint32 octave\n\
uint32 class_id\n\
\n\
";
  }

  static const char* value(const  ::bow_extract::cvKeypointVec_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::bow_extract::cvKeypointVec_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::bow_extract::cvKeypointVec_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::bow_extract::cvKeypointVec_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.keypoints);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct cvKeypointVec_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::bow_extract::cvKeypointVec_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::bow_extract::cvKeypointVec_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "keypoints[]" << std::endl;
    for (size_t i = 0; i < v.keypoints.size(); ++i)
    {
      s << indent << "  keypoints[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::bow_extract::cvKeypoint_<ContainerAllocator> >::stream(s, indent + "    ", v.keypoints[i]);
    }
  }
};


} // namespace message_operations
} // namespace ros

#endif // BOW_EXTRACT_MESSAGE_CVKEYPOINTVEC_H

