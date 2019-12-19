#include <iostream>

#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"

int main (int argc, char **argv){

  ros::init(argc,argv,"aaaaaaa");
  ros::NodeHandle node;

  ROS_INFO("Test0");

  return 0;
}
