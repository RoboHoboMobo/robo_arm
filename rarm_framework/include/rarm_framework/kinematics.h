#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <cmath>
#include "trans_matrix.h"

#define L1 0.041  // Links lengts
#define L2 0.090
#define L3 0.090
#define L4 0.090
#define L5 0.049
#define F0 0.0459 // Gripper fingers' lengths
#define F1 0.060


class Kinematics
{
private:
  TransMatrix ee_pose;
  double th1, th2, th3, th4, th5, th6; // th6: gripper's finger angle
  double theta;

  double k;  // IK parametr, 0<=k<=1
public:
  Kinematics();

  Kinematics(double th1, double th2, double th3,
             double th4, double th5, double th6);

  Kinematics(double x, double y, double z,
             double theta, double grip_angle);

  Kinematics(TransMatrix tm);

  TransMatrix solveFK();
  static TransMatrix solveFK(double th1, double th2, double th3, double th4, double th5, double th6);
  static TransMatrix solveFK(double j_angles[6]);
  static TransMatrix solveFK(std::vector<double> j_angles);

  void setK(double k);
  void solveIK(double result[5]);
  bool solveIK(std::vector<double> &angles);
  static bool solveIK(double x, double y, double z,
                      double theta,
                      double grip_angle,
                      double result[5]);

  static bool solveIK(double x, double y, double z,
                      double theta,
                      double grip_angle,
                      std::vector<double> &res_ang);

  void getJAngles(double &th1, double &th2, double &th3,
                  double &th4, double &th5, double &th6);

  std::vector<double> getJAngles();
  TransMatrix getMatrix();

};

#endif // KINEMATICS_H
