#include "../include/main.h"

int main (int argc, char **argv){

  ros::init(argc,argv,"rarm_control_node");
  ros::NodeHandle node;

  ROS_INFO("Test0");

  Kinematics *kn0 = new Kinematics();
  Kinematics *kn1 = new Kinematics( 0.0,  1.0,  2.0,   3.0,
                                    4.0,  5.0,  6.0,   7.0,
                                    8.0,  9.0, 10.0,  11.0,
                                   12.0, 13.0, 14.0,  15.0);

  double arr[] = { 0.0,  1.0,  2.0,   3.0,
                   4.0,  5.0,  6.0,   7.0,
                   8.0,  9.0, 10.0,  11.0,
                  12.0, 13.0, 14.0,  15.0};
  Kinematics *kn2 = new Kinematics(arr);

  double arr1[][4] = {{ 0.0,  1.0,  2.0,   3.0},
                      { 4.0,  5.0,  6.0,   7.0},
                      { 8.0,  9.0, 10.0,  11.0},
                      {12.0, 13.0, 14.0,  15.0}};

  Kinematics *kn3 = new Kinematics(arr1);
  *kn3 += *kn2;


  //*kn3 = (2.0  * ((Kinematics const) ref)) ;

  double test_arr[4][4];
  kn3->getTMatrix(test_arr);

  std::cout<<"(): "<<std::endl;
  for(int i=0; i<4; i++)
  {
    for(int j=0; j<4; j++)
      std::cout<<test_arr[i][j]<<" ";
    std::cout<<std::endl;
  }



  return 0;
}
