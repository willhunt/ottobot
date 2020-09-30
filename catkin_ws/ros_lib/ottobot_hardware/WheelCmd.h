#ifndef _ROS_ottobot_hardware_WheelCmd_h
#define _ROS_ottobot_hardware_WheelCmd_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ottobot_hardware
{

  class WheelCmd : public ros::Msg
  {
    public:
      typedef uint8_t _mode_type;
      _mode_type mode;
      typedef float _angular_velocity_left_type;
      _angular_velocity_left_type angular_velocity_left;
      typedef float _angular_velocity_right_type;
      _angular_velocity_right_type angular_velocity_right;
      typedef float _duty_left_type;
      _duty_left_type duty_left;
      typedef float _duty_right_type;
      _duty_right_type duty_right;

    WheelCmd():
      mode(0),
      angular_velocity_left(0),
      angular_velocity_right(0),
      duty_left(0),
      duty_right(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->mode >> (8 * 0)) & 0xFF;
      offset += sizeof(this->mode);
      union {
        float real;
        uint32_t base;
      } u_angular_velocity_left;
      u_angular_velocity_left.real = this->angular_velocity_left;
      *(outbuffer + offset + 0) = (u_angular_velocity_left.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_angular_velocity_left.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_angular_velocity_left.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_angular_velocity_left.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->angular_velocity_left);
      union {
        float real;
        uint32_t base;
      } u_angular_velocity_right;
      u_angular_velocity_right.real = this->angular_velocity_right;
      *(outbuffer + offset + 0) = (u_angular_velocity_right.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_angular_velocity_right.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_angular_velocity_right.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_angular_velocity_right.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->angular_velocity_right);
      union {
        float real;
        uint32_t base;
      } u_duty_left;
      u_duty_left.real = this->duty_left;
      *(outbuffer + offset + 0) = (u_duty_left.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_duty_left.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_duty_left.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_duty_left.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->duty_left);
      union {
        float real;
        uint32_t base;
      } u_duty_right;
      u_duty_right.real = this->duty_right;
      *(outbuffer + offset + 0) = (u_duty_right.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_duty_right.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_duty_right.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_duty_right.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->duty_right);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->mode =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->mode);
      union {
        float real;
        uint32_t base;
      } u_angular_velocity_left;
      u_angular_velocity_left.base = 0;
      u_angular_velocity_left.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_angular_velocity_left.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_angular_velocity_left.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_angular_velocity_left.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->angular_velocity_left = u_angular_velocity_left.real;
      offset += sizeof(this->angular_velocity_left);
      union {
        float real;
        uint32_t base;
      } u_angular_velocity_right;
      u_angular_velocity_right.base = 0;
      u_angular_velocity_right.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_angular_velocity_right.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_angular_velocity_right.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_angular_velocity_right.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->angular_velocity_right = u_angular_velocity_right.real;
      offset += sizeof(this->angular_velocity_right);
      union {
        float real;
        uint32_t base;
      } u_duty_left;
      u_duty_left.base = 0;
      u_duty_left.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_duty_left.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_duty_left.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_duty_left.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->duty_left = u_duty_left.real;
      offset += sizeof(this->duty_left);
      union {
        float real;
        uint32_t base;
      } u_duty_right;
      u_duty_right.base = 0;
      u_duty_right.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_duty_right.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_duty_right.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_duty_right.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->duty_right = u_duty_right.real;
      offset += sizeof(this->duty_right);
     return offset;
    }

    const char * getType(){ return "ottobot_hardware/WheelCmd"; };
    const char * getMD5(){ return "0265396faf73c2cb851c16dad414ea9f"; };

  };

}
#endif
