#ifndef _ROS_actionlib_tutorials_AveragingActionResult_h
#define _ROS_actionlib_tutorials_AveragingActionResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros_lib/ros/msg.h"
#include "ros_lib/std_msgs/Header.h"
#include "ros_lib/actionlib_msgs/GoalStatus.h"
#include "ros_lib/actionlib_tutorials/AveragingResult.h"

namespace actionlib_tutorials
{

  class AveragingActionResult : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalStatus _status_type;
      _status_type status;
      typedef actionlib_tutorials::AveragingResult _result_type;
      _result_type result;

    AveragingActionResult():
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

    const char * getType(){ return "actionlib_tutorials/AveragingActionResult"; };
    const char * getMD5(){ return "8672cb489d347580acdcd05c5d497497"; };

  };

}
#endif