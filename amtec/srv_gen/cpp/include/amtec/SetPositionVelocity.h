/* Auto-generated by genmsg_cpp for file /home/asd/MyRosPack/amtec/srv/SetPositionVelocity.srv */
#ifndef AMTEC_SERVICE_SETPOSITIONVELOCITY_H
#define AMTEC_SERVICE_SETPOSITIONVELOCITY_H
#include <string>
#include <vector>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/message.h"
#include "ros/time.h"

#include "ros/service_traits.h"




namespace amtec
{
template <class ContainerAllocator>
struct SetPositionVelocityRequest_ : public ros::Message
{
  typedef SetPositionVelocityRequest_<ContainerAllocator> Type;

  SetPositionVelocityRequest_()
  : position_pan(0.0)
  , position_tilt(0.0)
  , velocity_pan(0.0)
  , velocity_tilt(0.0)
  {
  }

  SetPositionVelocityRequest_(const ContainerAllocator& _alloc)
  : position_pan(0.0)
  , position_tilt(0.0)
  , velocity_pan(0.0)
  , velocity_tilt(0.0)
  {
  }

  typedef double _position_pan_type;
  double position_pan;

  typedef double _position_tilt_type;
  double position_tilt;

  typedef double _velocity_pan_type;
  double velocity_pan;

  typedef double _velocity_tilt_type;
  double velocity_tilt;


private:
  static const char* __s_getDataType_() { return "amtec/SetPositionVelocityRequest"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROSCPP_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "1faf4b4c83e1e50040afc67afdd423b0"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROSCPP_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getServerMD5Sum_() { return "1faf4b4c83e1e50040afc67afdd423b0"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getServerMD5Sum() { return __s_getServerMD5Sum_(); }

  ROSCPP_DEPRECATED const std::string __getServerMD5Sum() const { return __s_getServerMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "float64 position_pan\n\
float64 position_tilt\n\
float64 velocity_pan\n\
float64 velocity_tilt\n\
\n\
"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROSCPP_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROSCPP_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, position_pan);
    ros::serialization::serialize(stream, position_tilt);
    ros::serialization::serialize(stream, velocity_pan);
    ros::serialization::serialize(stream, velocity_tilt);
    return stream.getData();
  }

  ROSCPP_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, position_pan);
    ros::serialization::deserialize(stream, position_tilt);
    ros::serialization::deserialize(stream, velocity_pan);
    ros::serialization::deserialize(stream, velocity_tilt);
    return stream.getData();
  }

  ROSCPP_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(position_pan);
    size += ros::serialization::serializationLength(position_tilt);
    size += ros::serialization::serializationLength(velocity_pan);
    size += ros::serialization::serializationLength(velocity_tilt);
    return size;
  }

  typedef boost::shared_ptr< ::amtec::SetPositionVelocityRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::amtec::SetPositionVelocityRequest_<ContainerAllocator>  const> ConstPtr;
}; // struct SetPositionVelocityRequest
typedef  ::amtec::SetPositionVelocityRequest_<std::allocator<void> > SetPositionVelocityRequest;

typedef boost::shared_ptr< ::amtec::SetPositionVelocityRequest> SetPositionVelocityRequestPtr;
typedef boost::shared_ptr< ::amtec::SetPositionVelocityRequest const> SetPositionVelocityRequestConstPtr;


template <class ContainerAllocator>
struct SetPositionVelocityResponse_ : public ros::Message
{
  typedef SetPositionVelocityResponse_<ContainerAllocator> Type;

  SetPositionVelocityResponse_()
  {
  }

  SetPositionVelocityResponse_(const ContainerAllocator& _alloc)
  {
  }


private:
  static const char* __s_getDataType_() { return "amtec/SetPositionVelocityResponse"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROSCPP_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "d41d8cd98f00b204e9800998ecf8427e"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROSCPP_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getServerMD5Sum_() { return "1faf4b4c83e1e50040afc67afdd423b0"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getServerMD5Sum() { return __s_getServerMD5Sum_(); }

  ROSCPP_DEPRECATED const std::string __getServerMD5Sum() const { return __s_getServerMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "\n\
"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROSCPP_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROSCPP_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    return stream.getData();
  }

  ROSCPP_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    return stream.getData();
  }

  ROSCPP_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    return size;
  }

  typedef boost::shared_ptr< ::amtec::SetPositionVelocityResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::amtec::SetPositionVelocityResponse_<ContainerAllocator>  const> ConstPtr;
}; // struct SetPositionVelocityResponse
typedef  ::amtec::SetPositionVelocityResponse_<std::allocator<void> > SetPositionVelocityResponse;

typedef boost::shared_ptr< ::amtec::SetPositionVelocityResponse> SetPositionVelocityResponsePtr;
typedef boost::shared_ptr< ::amtec::SetPositionVelocityResponse const> SetPositionVelocityResponseConstPtr;

struct SetPositionVelocity
{

typedef SetPositionVelocityRequest Request;
typedef SetPositionVelocityResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct SetPositionVelocity
} // namespace amtec

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator>
struct MD5Sum< ::amtec::SetPositionVelocityRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "1faf4b4c83e1e50040afc67afdd423b0";
  }

  static const char* value(const  ::amtec::SetPositionVelocityRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x1faf4b4c83e1e500ULL;
  static const uint64_t static_value2 = 0x40afc67afdd423b0ULL;
};

template<class ContainerAllocator>
struct DataType< ::amtec::SetPositionVelocityRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "amtec/SetPositionVelocityRequest";
  }

  static const char* value(const  ::amtec::SetPositionVelocityRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::amtec::SetPositionVelocityRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "float64 position_pan\n\
float64 position_tilt\n\
float64 velocity_pan\n\
float64 velocity_tilt\n\
\n\
";
  }

  static const char* value(const  ::amtec::SetPositionVelocityRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::amtec::SetPositionVelocityRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator>
struct MD5Sum< ::amtec::SetPositionVelocityResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const  ::amtec::SetPositionVelocityResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::amtec::SetPositionVelocityResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "amtec/SetPositionVelocityResponse";
  }

  static const char* value(const  ::amtec::SetPositionVelocityResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::amtec::SetPositionVelocityResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
";
  }

  static const char* value(const  ::amtec::SetPositionVelocityResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::amtec::SetPositionVelocityResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::amtec::SetPositionVelocityRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.position_pan);
    stream.next(m.position_tilt);
    stream.next(m.velocity_pan);
    stream.next(m.velocity_tilt);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct SetPositionVelocityRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::amtec::SetPositionVelocityResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct SetPositionVelocityResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<amtec::SetPositionVelocity> {
  static const char* value() 
  {
    return "1faf4b4c83e1e50040afc67afdd423b0";
  }

  static const char* value(const amtec::SetPositionVelocity&) { return value(); } 
};

template<>
struct DataType<amtec::SetPositionVelocity> {
  static const char* value() 
  {
    return "amtec/SetPositionVelocity";
  }

  static const char* value(const amtec::SetPositionVelocity&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<amtec::SetPositionVelocityRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "1faf4b4c83e1e50040afc67afdd423b0";
  }

  static const char* value(const amtec::SetPositionVelocityRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<amtec::SetPositionVelocityRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "amtec/SetPositionVelocity";
  }

  static const char* value(const amtec::SetPositionVelocityRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<amtec::SetPositionVelocityResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "1faf4b4c83e1e50040afc67afdd423b0";
  }

  static const char* value(const amtec::SetPositionVelocityResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<amtec::SetPositionVelocityResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "amtec/SetPositionVelocity";
  }

  static const char* value(const amtec::SetPositionVelocityResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // AMTEC_SERVICE_SETPOSITIONVELOCITY_H

