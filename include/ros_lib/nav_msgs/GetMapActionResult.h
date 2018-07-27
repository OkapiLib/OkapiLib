#ifndef _ROS_nav_msgs_GetMapActionResult_h
#define _ROS_nav_msgs_GetMapActionResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros_lib/ros/msg.h"
#include "ros_lib/std_msgs/Header.h"
#include "ros_lib/actionlib_msgs/GoalStatus.h"
#include "ros_lib/nav_msgs/GetMapResult.h"

namespace nav_msgs
{

  class GetMapActionResult : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalStatus _status_type;
      _status_type status;
      typedef nav_msgs::GetMapResult _result_type;
      _result_type result;

    GetMapActionResult():
      header(),
      status(),
      result()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->status.serialize(outbuffer + offset);
      offset += this->result.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->status.deserialize(inbuffer + offset);
      offset += this->result.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "nav_msgs/GetMapActionResult"; };
    const char * getMD5(){ return "ac66e5b9a79bb4bbd33dab245236c892"; };

  };

}
#endif