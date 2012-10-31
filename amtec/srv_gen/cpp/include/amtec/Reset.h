/* Auto-generated by genmsg_cpp for file /home/asd/MyRosPack/amtec/srv/Reset.srv */
#ifndef AMTEC_SERVICE_RESET_H
#define AMTEC_SERVICE_RESET_H
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
struct ResetRequest_ : public ros::Message
{
  typedef ResetRequest_<ContainerAllocator> Type;

  ResetRequest_()
  {
  }

  ResetRequest_(const ContainerAllocator& _alloc)
  {
  }


private:
  static const char* __s_getDataType_() { return "amtec/ResetRequest"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROSCPP_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "d41d8cd98f00b204e9800998ecf8427e"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROSCPP_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getServerMD5Sum_() { return "d41d8cd98f00b204e9800998ecf8427e"; }
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

  typedef boost::shared_ptr< ::amtec::ResetRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::amtec::ResetRequest_<ContainerAllocator>  const> ConstPtr;
}; // struct ResetRequest
typedef  ::amtec::ResetRequest_<std::allocator<void> > ResetRequest;

typedef boost::shared_ptr< ::amtec::ResetRequest> ResetRequestPtr;
typedef boost::shared_ptr< ::amtec::ResetRequest const> ResetRequestConstPtr;


template <class ContainerAllocator>
struct ResetResponse_ : public ros::Message
{
  typedef ResetResponse_<ContainerAllocator> Type;

  ResetResponse_()
  {
  }

  ResetResponse_(const ContainerAllocator& _alloc)
  {
  }


private:
  static const char* __s_getDataType_() { return "amtec/ResetResponse"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROSCPP_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "d41d8cd98f00b204e9800998ecf8427e"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROSCPP_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getServerMD5Sum_() { return "d41d8cd98f00b204e9800998ecf8427e"; }
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

  typedef boost::shared_ptr< ::amtec::ResetResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::amtec::ResetResponse_<ContainerAllocator>  const> ConstPtr;
}; // struct ResetResponse
typedef  ::amtec::ResetResponse_<std::allocator<void> > ResetResponse;

typedef boost::shared_ptr< ::amtec::ResetResponse> ResetResponsePtr;
typedef boost::shared_ptr< ::amtec::ResetResponse const> ResetResponseConstPtr;

struct Reset
{

typedef ResetRequest Request;
typedef ResetResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct Reset
} // namespace amtec

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator>
struct MD5Sum< ::amtec::ResetRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const  ::amtec::ResetRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::amtec::ResetRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "amtec/ResetRequest";
  }

  static const char* value(const  ::amtec::ResetRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::amtec::ResetRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
";
  }

  static const char* value(const  ::amtec::ResetRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::amtec::ResetRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator>
struct MD5Sum< ::amtec::ResetResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const  ::amtec::ResetResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::amtec::ResetResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "amtec/ResetResponse";
  }

  static const char* value(const  ::amtec::ResetResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::amtec::ResetResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
";
  }

  static const char* value(const  ::amtec::ResetResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::amtec::ResetResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::amtec::ResetRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct ResetRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::amtec::ResetResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct ResetResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<amtec::Reset> {
  static const char* value() 
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const amtec::Reset&) { return value(); } 
};

template<>
struct DataType<amtec::Reset> {
  static const char* value() 
  {
    return "amtec/Reset";
  }

  static const char* value(const amtec::Reset&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<amtec::ResetRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const amtec::ResetRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<amtec::ResetRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "amtec/Reset";
  }

  static const char* value(const amtec::ResetRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<amtec::ResetResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const amtec::ResetResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<amtec::ResetResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "amtec/Reset";
  }

  static const char* value(const amtec::ResetResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // AMTEC_SERVICE_RESET_H

