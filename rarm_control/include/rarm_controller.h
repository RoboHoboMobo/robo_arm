#ifndef RARM_CONTROLLER_H
#define RARM_CONTROLLER_H

#include <ros/ros.h>
#include "rarm_ctrl_vector_msg/Ctrl_vector_msg.h"
#include "rarm_framework/singleton.h"

class rarm_controller : public Singleton<rarm_controller>
{
private:
public:
  virtual void init(ros::NodeHandle* n) = 0;
  virtual bool get_input() = 0;
  virtual void send_msg() = 0;
  virtual void quit() = 0;
};

#endif // RARM_CONTROLLER_H
