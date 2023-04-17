// Generated by gencpp from file motoman_msgs/DynamicJointsGroup.msg
// DO NOT EDIT!


#ifndef MOTOMAN_MSGS_MESSAGE_DYNAMICJOINTSGROUP_H
#define MOTOMAN_MSGS_MESSAGE_DYNAMICJOINTSGROUP_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace motoman_msgs
{
template <class ContainerAllocator>
struct DynamicJointsGroup_
{
  typedef DynamicJointsGroup_<ContainerAllocator> Type;

  DynamicJointsGroup_()
    : group_number(0)
    , num_joints(0)
    , valid_fields(0)
    , positions()
    , velocities()
    , accelerations()
    , effort()
    , time_from_start()  {
    }
  DynamicJointsGroup_(const ContainerAllocator& _alloc)
    : group_number(0)
    , num_joints(0)
    , valid_fields(0)
    , positions(_alloc)
    , velocities(_alloc)
    , accelerations(_alloc)
    , effort(_alloc)
    , time_from_start()  {
  (void)_alloc;
    }



   typedef int16_t _group_number_type;
  _group_number_type group_number;

   typedef int16_t _num_joints_type;
  _num_joints_type num_joints;

   typedef int16_t _valid_fields_type;
  _valid_fields_type valid_fields;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _positions_type;
  _positions_type positions;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _velocities_type;
  _velocities_type velocities;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _accelerations_type;
  _accelerations_type accelerations;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _effort_type;
  _effort_type effort;

   typedef ros::Duration _time_from_start_type;
  _time_from_start_type time_from_start;





  typedef boost::shared_ptr< ::motoman_msgs::DynamicJointsGroup_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::motoman_msgs::DynamicJointsGroup_<ContainerAllocator> const> ConstPtr;

}; // struct DynamicJointsGroup_

typedef ::motoman_msgs::DynamicJointsGroup_<std::allocator<void> > DynamicJointsGroup;

typedef boost::shared_ptr< ::motoman_msgs::DynamicJointsGroup > DynamicJointsGroupPtr;
typedef boost::shared_ptr< ::motoman_msgs::DynamicJointsGroup const> DynamicJointsGroupConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::motoman_msgs::DynamicJointsGroup_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::motoman_msgs::DynamicJointsGroup_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::motoman_msgs::DynamicJointsGroup_<ContainerAllocator1> & lhs, const ::motoman_msgs::DynamicJointsGroup_<ContainerAllocator2> & rhs)
{
  return lhs.group_number == rhs.group_number &&
    lhs.num_joints == rhs.num_joints &&
    lhs.valid_fields == rhs.valid_fields &&
    lhs.positions == rhs.positions &&
    lhs.velocities == rhs.velocities &&
    lhs.accelerations == rhs.accelerations &&
    lhs.effort == rhs.effort &&
    lhs.time_from_start == rhs.time_from_start;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::motoman_msgs::DynamicJointsGroup_<ContainerAllocator1> & lhs, const ::motoman_msgs::DynamicJointsGroup_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace motoman_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::motoman_msgs::DynamicJointsGroup_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::motoman_msgs::DynamicJointsGroup_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::motoman_msgs::DynamicJointsGroup_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::motoman_msgs::DynamicJointsGroup_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::motoman_msgs::DynamicJointsGroup_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::motoman_msgs::DynamicJointsGroup_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::motoman_msgs::DynamicJointsGroup_<ContainerAllocator> >
{
  static const char* value()
  {
    return "fb56acaba1b90a9d7640af3e785681ca";
  }

  static const char* value(const ::motoman_msgs::DynamicJointsGroup_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xfb56acaba1b90a9dULL;
  static const uint64_t static_value2 = 0x7640af3e785681caULL;
};

template<class ContainerAllocator>
struct DataType< ::motoman_msgs::DynamicJointsGroup_<ContainerAllocator> >
{
  static const char* value()
  {
    return "motoman_msgs/DynamicJointsGroup";
  }

  static const char* value(const ::motoman_msgs::DynamicJointsGroup_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::motoman_msgs::DynamicJointsGroup_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# DynamicJointsGroup\n"
"#group: # length of this array must match num_groups\n"
"#    id:   control-group ID for use on-controller\n"
"#    num_joints: # of joints in this motion group\n"
"#    valid_fields: #bit field for following items\n"
"#    # length of the following items must match num_joints, order set by controller.  Invalid fields (see bit field above) are not included, resulting in a shorter message.\n"
"#    positions[]\n"
"#    velocities[]\n"
"#    accelerations[]\n"
"#    effort[]\n"
"#    time_from_start\n"
"\n"
"\n"
"int16 group_number\n"
"int16 num_joints\n"
"int16 valid_fields\n"
"float64[] positions\n"
"float64[] velocities\n"
"float64[] accelerations\n"
"float64[] effort\n"
"duration time_from_start\n"
;
  }

  static const char* value(const ::motoman_msgs::DynamicJointsGroup_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::motoman_msgs::DynamicJointsGroup_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.group_number);
      stream.next(m.num_joints);
      stream.next(m.valid_fields);
      stream.next(m.positions);
      stream.next(m.velocities);
      stream.next(m.accelerations);
      stream.next(m.effort);
      stream.next(m.time_from_start);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct DynamicJointsGroup_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::motoman_msgs::DynamicJointsGroup_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::motoman_msgs::DynamicJointsGroup_<ContainerAllocator>& v)
  {
    s << indent << "group_number: ";
    Printer<int16_t>::stream(s, indent + "  ", v.group_number);
    s << indent << "num_joints: ";
    Printer<int16_t>::stream(s, indent + "  ", v.num_joints);
    s << indent << "valid_fields: ";
    Printer<int16_t>::stream(s, indent + "  ", v.valid_fields);
    s << indent << "positions[]" << std::endl;
    for (size_t i = 0; i < v.positions.size(); ++i)
    {
      s << indent << "  positions[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.positions[i]);
    }
    s << indent << "velocities[]" << std::endl;
    for (size_t i = 0; i < v.velocities.size(); ++i)
    {
      s << indent << "  velocities[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.velocities[i]);
    }
    s << indent << "accelerations[]" << std::endl;
    for (size_t i = 0; i < v.accelerations.size(); ++i)
    {
      s << indent << "  accelerations[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.accelerations[i]);
    }
    s << indent << "effort[]" << std::endl;
    for (size_t i = 0; i < v.effort.size(); ++i)
    {
      s << indent << "  effort[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.effort[i]);
    }
    s << indent << "time_from_start: ";
    Printer<ros::Duration>::stream(s, indent + "  ", v.time_from_start);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MOTOMAN_MSGS_MESSAGE_DYNAMICJOINTSGROUP_H
