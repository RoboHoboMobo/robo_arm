#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"

#include <string>
#include <iostream>
#include <curses.h>

enum commands
{
  HALT,
  FORWARD,
  BACKWARD,
  RIGHT,
  LEFT,
  UP,
  DOWN,
  SPINUP,
  SPINDOWN,
  GRIP,
  UNGRIP
};

