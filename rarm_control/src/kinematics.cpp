#include "../include/kinematics.h"

Kinematics::Kinematics(double th1, double th2, double th3,
                       double th4, double th5, double th6)
{
  this->th1 = th1;
  this->th2 = th2;
  this->th3 = th3;
  this->th4 = th4;
  this->th5 = th5;
  this->th6 = th6;
}

TransMatrix Kinematics::getFK(double th1, double th2, double th3, double th4, double th5, double th6)
{
  TransMatrix result;

  result = TransMatrix::DHToM(0.0, 0.0, L1, th1) * TransMatrix::DHToM(-M_PI/2, 0.0, 0.0, th2-M_PI/2) *
           TransMatrix::DHToM(0.0, L2, 0.0, th3) * TransMatrix::DHToM(0.0, L3, 0.0, th4) *
           TransMatrix::DHToM(0.0, L4, 0.0, th5) * TransMatrix::DHToM(0.0, L5, 0.0, 0.0) *
           TransMatrix::DHToM(0.0, F0*cos(th6), 0.0, 0.0) * TransMatrix::DHToM(0.0, F1, 0.0, 0.0);

  return result;
}

TransMatrix Kinematics::getFK(double j_angles[6])
{
  TransMatrix result;
  result = TransMatrix::DHToM(0.0, 0.0, L1, j_angles[0]) * TransMatrix::DHToM(-M_PI/2, 0.0, 0.0, j_angles[1]-M_PI/2) *
           TransMatrix::DHToM(0.0, L2, 0.0, j_angles[2]) * TransMatrix::DHToM(0.0, L3, 0.0, j_angles[3]) *
           TransMatrix::DHToM(0.0, L4, 0.0, j_angles[4]) * TransMatrix::DHToM(0.0, L5, 0.0, 0.0) *
           TransMatrix::DHToM(0.0, F0*cos(j_angles[5]), 0.0, 0.0) * TransMatrix::DHToM(0.0, F1, 0.0, 0.0);

  return result;
}

TransMatrix Kinematics::getFK(std::vector<double> j_angles)
{
  TransMatrix result;
  result = TransMatrix::DHToM(0.0, 0.0, L1, j_angles[0]) * TransMatrix::DHToM(-M_PI/2, 0.0, 0.0, j_angles[1]-M_PI/2) *
           TransMatrix::DHToM(0.0, L2, 0.0, j_angles[2]) * TransMatrix::DHToM(0.0, L3, 0.0, j_angles[3]) *
           TransMatrix::DHToM(0.0, L4, 0.0, j_angles[4]) * TransMatrix::DHToM(0.0, L5, 0.0, 0.0) *
           TransMatrix::DHToM(0.0, F0*cos(j_angles[5]), 0.0, 0.0) * TransMatrix::DHToM(0.0, F1, 0.0, 0.0);

  return result;
}
