// Generated by gencpp from file jetson_camera/QRdata.msg
// DO NOT EDIT!


#ifndef JETSON_CAMERA_MESSAGE_QRDATA_H
#define JETSON_CAMERA_MESSAGE_QRDATA_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace jetson_camera
{
template <class ContainerAllocator>
struct QRdata_
{
  typedef QRdata_<ContainerAllocator> Type;

  QRdata_()
    : qr_id(0)
    , move(0)
    , rotate(0)
    , perfect(0)  {
    }
  QRdata_(const ContainerAllocator& _alloc)
    : qr_id(0)
    , move(0)
    , rotate(0)
    , perfect(0)  {
  (void)_alloc;
    }



   typedef int32_t _qr_id_type;
  _qr_id_type qr_id;

   typedef int32_t _move_type;
  _move_type move;

   typedef int32_t _rotate_type;
  _rotate_type rotate;

   typedef int32_t _perfect_type;
  _perfect_type perfect;





  typedef boost::shared_ptr< ::jetson_camera::QRdata_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::jetson_camera::QRdata_<ContainerAllocator> const> ConstPtr;

}; // struct QRdata_

typedef ::jetson_camera::QRdata_<std::allocator<void> > QRdata;

typedef boost::shared_ptr< ::jetson_camera::QRdata > QRdataPtr;
typedef boost::shared_ptr< ::jetson_camera::QRdata const> QRdataConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::jetson_camera::QRdata_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::jetson_camera::QRdata_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::jetson_camera::QRdata_<ContainerAllocator1> & lhs, const ::jetson_camera::QRdata_<ContainerAllocator2> & rhs)
{
  return lhs.qr_id == rhs.qr_id &&
    lhs.move == rhs.move &&
    lhs.rotate == rhs.rotate &&
    lhs.perfect == rhs.perfect;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::jetson_camera::QRdata_<ContainerAllocator1> & lhs, const ::jetson_camera::QRdata_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace jetson_camera

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::jetson_camera::QRdata_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::jetson_camera::QRdata_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::jetson_camera::QRdata_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::jetson_camera::QRdata_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jetson_camera::QRdata_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jetson_camera::QRdata_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::jetson_camera::QRdata_<ContainerAllocator> >
{
  static const char* value()
  {
    return "427d1d80e217f255995aca6b86b027df";
  }

  static const char* value(const ::jetson_camera::QRdata_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x427d1d80e217f255ULL;
  static const uint64_t static_value2 = 0x995aca6b86b027dfULL;
};

template<class ContainerAllocator>
struct DataType< ::jetson_camera::QRdata_<ContainerAllocator> >
{
  static const char* value()
  {
    return "jetson_camera/QRdata";
  }

  static const char* value(const ::jetson_camera::QRdata_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::jetson_camera::QRdata_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# QRdata.msg\n"
"int32 qr_id\n"
"int32 move\n"
"int32 rotate\n"
"int32 perfect\n"
;
  }

  static const char* value(const ::jetson_camera::QRdata_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::jetson_camera::QRdata_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.qr_id);
      stream.next(m.move);
      stream.next(m.rotate);
      stream.next(m.perfect);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct QRdata_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::jetson_camera::QRdata_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::jetson_camera::QRdata_<ContainerAllocator>& v)
  {
    s << indent << "qr_id: ";
    Printer<int32_t>::stream(s, indent + "  ", v.qr_id);
    s << indent << "move: ";
    Printer<int32_t>::stream(s, indent + "  ", v.move);
    s << indent << "rotate: ";
    Printer<int32_t>::stream(s, indent + "  ", v.rotate);
    s << indent << "perfect: ";
    Printer<int32_t>::stream(s, indent + "  ", v.perfect);
  }
};

} // namespace message_operations
} // namespace ros

#endif // JETSON_CAMERA_MESSAGE_QRDATA_H
