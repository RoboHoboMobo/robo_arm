#include "main.h"

double b_x, b_y, b_z, b_theta, b_gr_ang; // buffer

void commandCallback(const rarm_ctrl_vector_msg::Ctrl_vector_msg &msg)
{
  for(std::vector<rarm_ctrl_msg::Ctrl_msg>::const_iterator it = msg.ctrl_vector.begin();
      it != msg.ctrl_vector.end(); it++)
  {
    if(it->dof_name.data == "x")
    {
      if(it->control_value.data == 0 && b_x > 0.0011)
        b_x -= 0.001;

      if(it->control_value.data == 1 &&
         (sqrt(b_x*b_x+b_y*b_y+b_z*b_z) <= L2+L3+L4+L5+F0+F1 || b_x < 0.0))
        b_x += 0.001;
    }

    if(it->dof_name.data == "y")
    {
      if(it->control_value.data == 0 &&
         (sqrt(b_x*b_x+b_y*b_y+b_z*b_z) <= L2+L3+L4+L5+F0+F1 || b_y > 0.0))
        b_y -= 0.001;

      if(it->control_value.data == 1 &&
         (sqrt(b_x*b_x+b_y*b_y+b_z*b_z) <= L2+L3+L4+L5+F0+F1 || b_y < 0.0))
        b_y += 0.001;
    }

    if(it->dof_name.data == "z")
    {
      if(it->control_value.data == 0 &&
         (sqrt(b_x*b_x+b_y*b_y+b_z*b_z) <= L2+L3+L4+L5+F0+F1 || b_z > 0.0011))
        b_z -= 0.001;

      if(it->control_value.data == 1 &&
         (sqrt(b_x*b_x+b_y*b_y+b_z*b_z) <= L2+L3+L4+L5+F0+F1))
        b_z += 0.001;
    }

    if(it->dof_name.data == "spin")
    {
      if(it->control_value.data == 0 &&
         b_theta >= -88.0/180.0*M_PI)
        b_theta -= M_PI/180.0 * 2.0;

      if(it->control_value.data == 1 &&
         b_theta <= 178.0/180.0*M_PI)
        b_theta += M_PI/180.0 * 2.0;
    }

    if(it->dof_name.data == "grip")
    {
      if(it->control_value.data == 0 &&
         b_gr_ang >= 2.0/180.0*M_PI)
        b_gr_ang -= M_PI/180.0 * 2.0;

      if(it->control_value.data == 1 &&
         b_gr_ang <= 88.0/180.0*M_PI)
        b_gr_ang += M_PI/180.0 * 2.0;
    }
  }

}

int main (int argc, char **argv){

  ros::init(argc,argv,"rarm_control_node");
  ros::NodeHandle node;

  // Keyboard commands subscriber
  ros::Subscriber command_sub = node.subscribe("/rarm_keyboard_control", 1, commandCallback);

  // Manipulator joints' Publishers
  ros::Publisher j1_pub = node.advertise<std_msgs::Float64>("/rarm/joint1_position_controller/command", 1);
  ros::Publisher j2_pub = node.advertise<std_msgs::Float64>("/rarm/joint2_position_controller/command", 1);
  ros::Publisher j3_pub = node.advertise<std_msgs::Float64>("/rarm/joint3_position_controller/command", 1);
  ros::Publisher j4_pub = node.advertise<std_msgs::Float64>("/rarm/joint4_position_controller/command", 1);
  ros::Publisher j5_pub = node.advertise<std_msgs::Float64>("/rarm/joint5_position_controller/command", 1);

  // Gripper joints' publishers
  ros::Publisher grip_j_l0_pub = node.advertise<std_msgs::Float64>("/rarm/gripper_joint_position_controller/command", 1);
  ros::Publisher grip_j_l1_pub = node.advertise<std_msgs::Float64>("/rarm/grp_jnt_l1_position_controller/command", 1);
  ros::Publisher grip_j_r0_pub = node.advertise<std_msgs::Float64>("/rarm/grp_jnt_r0_position_controller/command", 1);
  ros::Publisher grip_j_r1_pub = node.advertise<std_msgs::Float64>("/rarm/grp_jnt_r1_position_controller/command", 1);

  b_x = 0.0;
  b_y = 0.0;
  b_z = 0.3;
  b_theta = 0.0;
  b_gr_ang = 0.0;

  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double theta = 0.0;
  double gripper_ang = 0.0;

  ros::Rate rate(30); //Hz
  while(ros::ok())
  {

    std::vector<double> angles;
    if(Kinematics::solveIK(b_x, b_y, b_z,
                           b_theta,
                           b_gr_ang,
                           angles))
    {
      // Update values
      x = b_x;
      y = b_y;
      z = b_z;
      theta = b_theta;
      gripper_ang = b_gr_ang;

      std_msgs::Float64 msg;
      msg.data = angles[0];
      j1_pub.publish(msg);

      msg.data = angles[1];
      j2_pub.publish(msg);

      msg.data = angles[2];
      j3_pub.publish(msg);

      msg.data = angles[3];
      j4_pub.publish(msg);

      msg.data = angles[4];
      j5_pub.publish(msg);

      msg.data = angles[5];
      grip_j_l0_pub.publish(msg);

      msg.data = - angles[5];
      grip_j_r0_pub.publish(msg);

      msg.data = - angles[5];
      grip_j_l1_pub.publish(msg);

      msg.data = angles[5];
      grip_j_r1_pub.publish(msg);
      //ROS_INFO("%f\t%f\t%f\t%f\t%f\n", x, y, z, theta, gripper_ang);

    }
    else
    {
      ROS_INFO("IK error!");
      // Downgrade buffer to valid values
      b_x = x;
      b_y = y;
      b_z = z;
      b_theta = theta;
      b_gr_ang = gripper_ang;
    }

    ros::spinOnce();
    rate.sleep();
  }


  return 0;

}
